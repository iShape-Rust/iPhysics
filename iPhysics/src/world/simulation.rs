use super::{ContactBodyIndex, ContactPair, World};
use crate::body::Body;
use crate::collision::{collide, Contact};
use crate::quantity::{LinearVelocity, Position};
use alloc::vec;

const WAKE_SPEED_RAW: i32 = 205; // approximately 0.2 m/s in Q10
const WAKE_PENETRATION_RAW: u32 = 655; // approximately 0.01 m in Q16
const POSITION_SLOP_RAW: u32 = 64; // 1/1024 m
const MAX_POSITION_CORRECTION_RAW: u32 = 16_384; // 0.25 m
const MAX_RELATIVE_NORMAL_SPEED_RAW: i32 = 4 * LinearVelocity::MAX_VELOCITY;
const MAX_VELOCITY_CHANGE_RAW: u64 = 2 * MAX_RELATIVE_NORMAL_SPEED_RAW as u64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StepStats {
    pub tested_pairs: usize,
    pub aabb_pairs: usize,
    pub contacts: usize,
    pub sleeping_bodies: usize,
}

impl World {
    pub fn step(&mut self) -> StepStats {
        self.integrate_velocities();

        let mut stats = self.build_contacts();
        self.wake_impacted_bodies();

        for _ in 0..self.settings.velocity_iterations.max(1) {
            self.solve_velocities();
        }
        self.correct_positions();
        self.integrate_transforms();

        let mut has_contact = vec![false; self.bodies.len()];
        for pair in self.contact_pairs.iter().copied() {
            has_contact[pair.a] = true;
            if let ContactBodyIndex::Dynamic(index_b) = pair.b {
                has_contact[index_b] = true;
            }
        }

        for (index, body) in self.bodies.iter_mut().enumerate() {
            body.state_mut()
                .update_sleep(has_contact[index], self.settings.sleep);
            if body.state().is_sleeping() {
                stats.sleeping_bodies += 1;
            }
        }

        stats
    }

    fn integrate_velocities(&mut self) {
        for body in &mut self.bodies {
            if body.state().is_sleeping() {
                continue;
            }

            body.state_mut().linear_velocity = body
                .state()
                .linear_velocity()
                .advance(self.settings.gravity);
        }
    }

    fn build_contacts(&mut self) -> StepStats {
        self.contacts.clear();
        self.contact_pairs.clear();
        let mut stats = StepStats::default();

        for index_a in 0..self.bodies.len() {
            let a = &self.bodies[index_a];
            let aabb_a = a.collider().aabb(a.state().transform());

            for index_b in index_a + 1..self.bodies.len() {
                let b = &self.bodies[index_b];
                if a.state().is_sleeping() && b.state().is_sleeping() {
                    continue;
                }

                stats.tested_pairs += 1;
                let aabb_b = b.collider().aabb(b.state().transform());
                if !aabb_a.intersects(aabb_b) {
                    continue;
                }
                stats.aabb_pairs += 1;

                if let Some(contact) = collide(
                    a.id(),
                    a.collider(),
                    a.state().transform(),
                    b.id(),
                    b.collider(),
                    b.state().transform(),
                ) {
                    self.contacts.push(contact);
                    self.contact_pairs.push(ContactPair {
                        a: index_a,
                        b: ContactBodyIndex::Dynamic(index_b),
                    });
                }
            }

            if a.state().is_sleeping() {
                continue;
            }

            for (static_index, static_body) in self.static_bodies.iter().enumerate() {
                if !aabb_a.intersects(static_body.aabb()) {
                    continue;
                }

                for part in static_body.collider().parts() {
                    stats.tested_pairs += 1;
                    let part_transform = static_body.transform().compose(part.local_transform());
                    let part_aabb = part.collider().aabb(part_transform);
                    if !aabb_a.intersects(part_aabb) {
                        continue;
                    }
                    stats.aabb_pairs += 1;

                    if let Some(contact) = collide(
                        a.id(),
                        a.collider(),
                        a.state().transform(),
                        static_body.id(),
                        part.collider(),
                        part_transform,
                    ) {
                        self.contacts.push(contact);
                        self.contact_pairs.push(ContactPair {
                            a: index_a,
                            b: ContactBodyIndex::Static(static_index),
                        });
                    }
                }
            }
        }

        stats.contacts = self.contacts.len();
        stats
    }

    fn wake_impacted_bodies(&mut self) {
        for (contact, pair) in self.contacts.iter().zip(self.contact_pairs.iter().copied()) {
            let normal_speed = match pair.b {
                ContactBodyIndex::Dynamic(index_b) => relative_normal_speed(
                    &self.bodies[pair.a],
                    Some(&self.bodies[index_b]),
                    contact,
                ),
                ContactBodyIndex::Static(_) => {
                    relative_normal_speed(&self.bodies[pair.a], None, contact)
                }
            };
            let strong =
                normal_speed < -WAKE_SPEED_RAW || contact.penetration.raw() > WAKE_PENETRATION_RAW;
            if !strong {
                continue;
            }

            match pair.b {
                ContactBodyIndex::Dynamic(index_b) => {
                    let (a, b) = two_bodies_mut(&mut self.bodies, pair.a, index_b);
                    a.state_mut().wake();
                    b.state_mut().wake();
                }
                ContactBodyIndex::Static(_) => {
                    self.bodies[pair.a].state_mut().wake();
                }
            }
        }
    }

    fn solve_velocities(&mut self) {
        for (contact, pair) in self.contacts.iter().zip(self.contact_pairs.iter().copied()) {
            if let ContactBodyIndex::Static(static_index) = pair.b {
                let a = &mut self.bodies[pair.a];
                let inverse_a = a.inverse_mass_q24() as u64;
                if inverse_a == 0 {
                    continue;
                }
                let normal_speed = relative_normal_speed(a, None, contact);
                if normal_speed >= 0 {
                    continue;
                }
                let restitution = a.material().restitution_raw().max(
                    self.static_bodies[static_index]
                        .material()
                        .restitution_raw(),
                );
                let velocity_change = restitution_velocity_change(normal_speed, restitution);
                let [change_x, change_y] = contact.normal.scaled_wide_raw(velocity_change);
                add_velocity(a, -change_x, -change_y);
                continue;
            }

            let ContactBodyIndex::Dynamic(index_b) = pair.b else {
                unreachable!()
            };
            let (a, b) = two_bodies_mut(&mut self.bodies, pair.a, index_b);
            let inverse_a = a.inverse_mass_q24() as u64;
            let inverse_b = b.inverse_mass_q24() as u64;
            let inverse_sum = inverse_a + inverse_b;
            if inverse_sum == 0 {
                continue;
            }

            let normal_speed = relative_normal_speed(a, Some(b), contact);
            if normal_speed >= 0 {
                continue;
            }

            let restitution = a
                .material()
                .restitution_raw()
                .max(b.material().restitution_raw());
            let velocity_change = restitution_velocity_change(normal_speed, restitution);
            let change_a = div_round(velocity_change * inverse_a, inverse_sum);
            let change_b = div_round(velocity_change * inverse_b, inverse_sum);

            if inverse_a != 0 {
                let [change_x, change_y] = contact.normal.scaled_wide_raw(change_a);
                add_velocity(a, -change_x, -change_y);
            }
            if inverse_b != 0 {
                let [change_x, change_y] = contact.normal.scaled_wide_raw(change_b);
                add_velocity(b, change_x, change_y);
            }
        }
    }

    fn correct_positions(&mut self) {
        for (contact, pair) in self.contacts.iter().zip(self.contact_pairs.iter().copied()) {
            let correction = contact
                .penetration
                .raw()
                .saturating_sub(POSITION_SLOP_RAW)
                .saturating_mul(4)
                / 5;
            let correction = correction.min(MAX_POSITION_CORRECTION_RAW);
            if correction == 0 {
                continue;
            }

            if let ContactBodyIndex::Static(_) = pair.b {
                let [move_x, move_y] = contact.normal.scaled_wide_raw(correction as u64);
                add_position(&mut self.bodies[pair.a], -move_x, -move_y);
                continue;
            }

            let ContactBodyIndex::Dynamic(index_b) = pair.b else {
                unreachable!()
            };
            let (a, b) = two_bodies_mut(&mut self.bodies, pair.a, index_b);
            let inverse_a = a.inverse_mass_q24() as u64;
            let inverse_b = b.inverse_mass_q24() as u64;
            let inverse_sum = inverse_a + inverse_b;
            if inverse_sum == 0 {
                continue;
            }

            let move_a = div_round(correction as u64 * inverse_a, inverse_sum);
            let move_b = div_round(correction as u64 * inverse_b, inverse_sum);
            if inverse_a != 0 {
                let [move_x, move_y] = contact.normal.scaled_wide_raw(move_a);
                add_position(a, -move_x, -move_y);
            }
            if inverse_b != 0 {
                let [move_x, move_y] = contact.normal.scaled_wide_raw(move_b);
                add_position(b, move_x, move_y);
            }
        }
    }

    fn integrate_transforms(&mut self) {
        for body in &mut self.bodies {
            if body.state().is_sleeping() {
                continue;
            }

            let next = body.state().transform().advance(
                body.state().linear_velocity(),
                body.state().angular_velocity(),
            );
            body.state_mut().transform = next;
        }
    }
}

fn relative_normal_speed(a: &Body, b: Option<&Body>, contact: &Contact) -> i32 {
    let av = a.state().linear_velocity();
    let bv = b
        .map(|body| body.state().linear_velocity())
        .unwrap_or(LinearVelocity::ZERO);
    let speed = contact.normal.dot(bv - av);
    debug_assert!(speed.unsigned_abs() <= MAX_RELATIVE_NORMAL_SPEED_RAW as u64);
    speed as i32
}

fn add_velocity(body: &mut Body, dx: i64, dy: i64) {
    let [x, y] = body.state().linear_velocity().raw();
    body.state_mut().linear_velocity =
        LinearVelocity::from_wide_saturated(x as i64 + dx, y as i64 + dy);
}

fn add_position(body: &mut Body, dx: i64, dy: i64) {
    let [x, y] = body.state().transform().position.raw();
    body.state_mut().transform.position = Position::from_i64(x as i64 + dx, y as i64 + dy);
}

fn two_bodies_mut(bodies: &mut [Body], a: usize, b: usize) -> (&mut Body, &mut Body) {
    debug_assert!(a < b);
    let (left, right) = bodies.split_at_mut(b);
    (&mut left[a], &mut right[0])
}

#[inline(always)]
fn restitution_velocity_change(normal_speed: i32, restitution: u32) -> u64 {
    debug_assert!(normal_speed < 0);
    debug_assert!(restitution <= 1 << 16);
    let closing_speed = normal_speed.unsigned_abs() as u64;
    let restitution_factor = (1_u64 << 16) + restitution as u64;
    let result = round_shift(closing_speed * restitution_factor, 16);
    debug_assert!(result <= MAX_VELOCITY_CHANGE_RAW);
    result
}

#[inline(always)]
fn round_shift(value: u64, shift: u32) -> u64 {
    (value + (1_u64 << (shift - 1))) >> shift
}

#[inline(always)]
fn div_round(numerator: u64, denominator: u64) -> u64 {
    debug_assert!(denominator > 0);
    debug_assert!(numerator <= u64::MAX - (denominator >> 1));
    (numerator + (denominator >> 1)) / denominator
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyId, BodyState, Material, SleepConfig, StaticBody};
    use crate::collider::{Circle, ColliderPart, CompositeCollider};
    use crate::geometry::{GeometryPoint, UnitVector};
    use crate::quantity::{Angle, AngularVelocity, Length, LinearAcceleration, Mass};
    use crate::transform::Transform;
    use crate::world::WorldSettings;

    fn circle_body(id: u64, x: f64, velocity: f64, material: Material) -> Body {
        Body::dynamic(
            BodyId::new(id),
            Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
            Mass::ONE,
            material,
            BodyState::new(
                Transform::new(Position::from_meters(x, 0.0).unwrap(), Angle::ZERO),
                LinearVelocity::from_meters_per_second(velocity, 0.0).unwrap(),
                AngularVelocity::ZERO,
            ),
        )
    }

    fn zero_gravity_world() -> World {
        World::new(WorldSettings::new(LinearAcceleration::ZERO))
    }

    #[test]
    fn relative_speed_subtracts_extreme_velocities_without_overflow() {
        let mut a = circle_body(1, 0.0, 0.0, Material::INELASTIC);
        let mut b = circle_body(2, 0.0, 0.0, Material::INELASTIC);
        a.state_mut()
            .set_linear_velocity(LinearVelocity::from_raw(i32::MIN, i32::MIN));
        b.state_mut()
            .set_linear_velocity(LinearVelocity::from_raw(i32::MAX, i32::MAX));
        let contact = Contact {
            body_a: a.id(),
            body_b: b.id(),
            point: GeometryPoint::ZERO,
            normal: UnitVector::from_raw(1 << 30, 1 << 30),
            penetration: Length::ZERO,
        };

        assert_eq!(
            relative_normal_speed(&a, Some(&b), &contact),
            MAX_RELATIVE_NORMAL_SPEED_RAW
        );
    }

    #[test]
    fn maximum_solver_impulse_fits_u64_chain() {
        let normal_speed = -MAX_RELATIVE_NORMAL_SPEED_RAW;
        let impulse =
            restitution_velocity_change(normal_speed, Material::ELASTIC.restitution_raw());
        let inverse_mass = u32::MAX as u64;
        let inverse_sum = 2 * inverse_mass;

        assert_eq!(impulse, MAX_VELOCITY_CHANGE_RAW);
        assert!(impulse <= u32::MAX as u64);
        assert_eq!(div_round(impulse * inverse_mass, inverse_sum), impulse / 2);
    }

    #[test]
    fn elastic_equal_mass_circles_exchange_velocity() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, -0.5, 1.0, Material::ELASTIC))
            .unwrap();
        world
            .add_body(circle_body(2, 0.5, -1.0, Material::ELASTIC))
            .unwrap();

        let stats = world.step();

        assert_eq!(stats.contacts, 1);
        assert_eq!(
            world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity()
                .to_meters_per_second(),
            [-1.0, 0.0]
        );
        assert_eq!(
            world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity()
                .to_meters_per_second(),
            [1.0, 0.0]
        );
    }

    #[test]
    fn replay_from_cloned_snapshot_is_bit_exact() {
        let mut first = zero_gravity_world();
        first
            .add_body(circle_body(1, -0.5, 1.0, Material::INELASTIC))
            .unwrap();
        first
            .add_body(circle_body(2, 0.5, -1.0, Material::INELASTIC))
            .unwrap();

        first.step();
        let mut replay = first.clone();
        first.contacts.clear();

        for _ in 0..32 {
            first.step();
            replay.step();
        }

        assert_eq!(first.bodies(), replay.bodies());
    }

    #[test]
    fn resting_dynamic_circle_sleeps_on_static_circle() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, 0.0, 0.0, Material::INELASTIC))
            .unwrap();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(2),
                Transform::new(Position::from_meters(1.0, 0.0).unwrap(), Angle::ZERO),
                CompositeCollider::single(
                    Circle::new(Length::from_meters(0.5).unwrap())
                        .unwrap()
                        .into(),
                )
                .unwrap(),
                Material::INELASTIC,
            ))
            .unwrap();

        for _ in 0..SleepConfig::FAST_EFFECTS.required_ticks() {
            world.step();
        }

        assert!(world.body(BodyId::new(1)).unwrap().state().is_sleeping());
    }

    #[test]
    fn composite_part_identity_is_discarded_after_narrow_phase() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, 3.0, 0.0, Material::INELASTIC))
            .unwrap();
        let small_circle = Circle::new(Length::from_meters(0.5).unwrap()).unwrap();
        let composite = CompositeCollider::new(vec![
            ColliderPart::new(
                Transform::new(Position::from_meters(-3.0, 0.0).unwrap(), Angle::ZERO),
                small_circle.into(),
            ),
            ColliderPart::new(
                Transform::new(Position::from_meters(3.5, 0.0).unwrap(), Angle::ZERO),
                small_circle.into(),
            ),
        ])
        .unwrap();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(2),
                Transform::IDENTITY,
                composite,
                Material::INELASTIC,
            ))
            .unwrap();

        let stats = world.step();

        assert_eq!(stats.contacts, 1);
        assert_eq!(world.contacts()[0].body_a, BodyId::new(1));
        assert_eq!(world.contacts()[0].body_b, BodyId::new(2));
    }
}
