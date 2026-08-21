use super::{ContactBodyIndex, ContactPair, World};
use crate::body::Body;
use crate::collision::{Contact, collide};
use crate::quantity::{AngularVelocity, LinearVelocity, Position};
use crate::{GeometryPoint, UnitVector};
use alloc::vec;

const WAKE_SPEED_RAW: i32 = 205; // approximately 0.2 m/s in Q10
const WAKE_PENETRATION_RAW: u32 = 655; // approximately 0.01 m in Q16
const POSITION_SLOP_RAW: u32 = 64; // 1/1024 m
const MAX_POSITION_CORRECTION_RAW: u32 = 16_384; // 0.25 m
const MAX_RELATIVE_CONTACT_SPEED_RAW: i32 = 4 * LinearVelocity::MAX_VELOCITY;
const MAX_VELOCITY_CHANGE_RAW: u64 = 2 * MAX_RELATIVE_CONTACT_SPEED_RAW as u64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StepStats {
    pub tested_pairs: usize,
    pub aabb_pairs: usize,
    pub contacts: usize,
    pub sleeping_bodies: usize,
}

#[derive(Debug, Clone, Copy, Default)]
struct ContactImpulseState {
    // These are impulse numerators: dividing each by the corresponding Q24
    // effective inverse mass yields the scalar contact impulse. Keeping Q10
    // numerators avoids introducing another stored fixed-point format.
    normal_velocity_change_q10: u64,
    tangent_velocity_change_q10: i64,
}

impl World {
    pub fn step(&mut self) -> StepStats {
        self.integrate_velocities();

        let mut stats = self.build_contacts();
        self.wake_impacted_bodies();

        let mut impulse_states = vec![ContactImpulseState::default(); self.contacts.len()];
        for _ in 0..self.settings.velocity_iterations.max(1) {
            self.solve_velocities(&mut impulse_states);
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

    fn solve_velocities(&mut self, impulse_states: &mut [ContactImpulseState]) {
        debug_assert_eq!(impulse_states.len(), self.contacts.len());

        for ((contact, pair), impulse_state) in self
            .contacts
            .iter()
            .zip(self.contact_pairs.iter().copied())
            .zip(impulse_states.iter_mut())
        {
            match pair.b {
                ContactBodyIndex::Static(static_index) => {
                    let material_a = self.bodies[pair.a].material();
                    let material_b = self.static_bodies[static_index].material();
                    solve_contact_velocity(
                        &mut self.bodies[pair.a],
                        None,
                        contact,
                        material_a.combined_restitution_raw(material_b),
                        material_a.combined_friction_raw(material_b),
                        impulse_state,
                    );
                }
                ContactBodyIndex::Dynamic(index_b) => {
                    let (a, b) = two_bodies_mut(&mut self.bodies, pair.a, index_b);
                    let material_a = a.material();
                    let material_b = b.material();
                    solve_contact_velocity(
                        a,
                        Some(b),
                        contact,
                        material_a.combined_restitution_raw(material_b),
                        material_a.combined_friction_raw(material_b),
                        impulse_state,
                    );
                }
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

fn solve_contact_velocity(
    a: &mut Body,
    mut b: Option<&mut Body>,
    contact: &Contact,
    restitution_q16: u32,
    friction_q16: u32,
    impulse_state: &mut ContactImpulseState,
) {
    let normal = contact.normal;
    let rap = contact_lever_cross_axis(a, contact.point, normal);
    let rbp = b
        .as_deref()
        .map(|body| contact_lever_cross_axis(body, contact.point, normal))
        .unwrap_or(0);
    let normal_inverse_sum = contact_inverse_mass_q24(a, b.as_deref(), rap, rbp);
    if normal_inverse_sum == 0 {
        return;
    }

    let normal_speed = relative_speed_along(a, b.as_deref(), contact.point, normal);
    if normal_speed < 0 {
        let velocity_change = restitution_velocity_change(normal_speed, restitution_q16);
        impulse_state.normal_velocity_change_q10 = impulse_state
            .normal_velocity_change_q10
            .saturating_add(velocity_change);
        apply_contact_impulse(
            a,
            b.as_deref_mut(),
            normal,
            velocity_change as i64,
            normal_inverse_sum,
            rap,
            rbp,
        );
    }

    if friction_q16 == 0 || impulse_state.normal_velocity_change_q10 == 0 {
        return;
    }

    let tangent = normal.perpendicular();
    let rat = contact_lever_cross_axis(a, contact.point, tangent);
    let rbt = b
        .as_deref()
        .map(|body| contact_lever_cross_axis(body, contact.point, tangent))
        .unwrap_or(0);
    let tangent_inverse_sum = contact_inverse_mass_q24(a, b.as_deref(), rat, rbt);
    if tangent_inverse_sum == 0 {
        return;
    }

    let tangent_speed = relative_speed_along(a, b.as_deref(), contact.point, tangent);
    let previous = impulse_state.tangent_velocity_change_q10;
    let candidate = previous.saturating_sub(tangent_speed as i64);
    let limit = friction_velocity_change_limit_q10(
        friction_q16,
        impulse_state.normal_velocity_change_q10,
        normal_inverse_sum,
        tangent_inverse_sum,
    );
    let accumulated = candidate.clamp(-limit, limit);
    let velocity_change = accumulated - previous;
    impulse_state.tangent_velocity_change_q10 = accumulated;

    apply_contact_impulse(
        a,
        b,
        tangent,
        velocity_change,
        tangent_inverse_sum,
        rat,
        rbt,
    );
}

#[inline(always)]
fn contact_inverse_mass_q24(a: &Body, b: Option<&Body>, lever_a_q16: i64, lever_b_q16: i64) -> u64 {
    let inverse_b = b.map(Body::inverse_mass_q24).unwrap_or(0) as u64;
    let inverse_inertia_b = b.map(Body::inverse_inertia_q40).unwrap_or(0);
    (a.inverse_mass_q24() as u64)
        .saturating_add(inverse_b)
        .saturating_add(rotational_inverse_mass_q24(
            lever_a_q16,
            a.inverse_inertia_q40(),
        ))
        .saturating_add(rotational_inverse_mass_q24(lever_b_q16, inverse_inertia_b))
}

fn apply_contact_impulse(
    a: &mut Body,
    b: Option<&mut Body>,
    mut axis: UnitVector,
    impulse_numerator_q10: i64,
    inverse_sum_q24: u64,
    mut lever_a_q16: i64,
    mut lever_b_q16: i64,
) {
    if impulse_numerator_q10 == 0 {
        return;
    }

    if impulse_numerator_q10 < 0 {
        axis = -axis;
        lever_a_q16 = -lever_a_q16;
        lever_b_q16 = -lever_b_q16;
    }
    let magnitude = impulse_numerator_q10.unsigned_abs();
    debug_assert!(magnitude <= MAX_VELOCITY_CHANGE_RAW);
    let inverse_a = a.inverse_mass_q24() as u64;
    let inverse_b = b.as_deref().map(Body::inverse_mass_q24).unwrap_or(0) as u64;
    let inverse_inertia_a = a.inverse_inertia_q40();
    let inverse_inertia_b = b.as_deref().map(Body::inverse_inertia_q40).unwrap_or(0);

    let change_a = div_round_u128(magnitude as u128 * inverse_a as u128, inverse_sum_q24);
    if change_a != 0 {
        let [change_x, change_y] = axis.scaled_wide_raw(change_a);
        add_velocity(a, -change_x, -change_y);
    }
    let angular_change_a =
        angular_velocity_change_raw(magnitude, inverse_inertia_a, lever_a_q16, inverse_sum_q24);
    add_angular_velocity(a, -angular_change_a);

    if let Some(body) = b {
        let change_b = div_round_u128(magnitude as u128 * inverse_b as u128, inverse_sum_q24);
        if change_b != 0 {
            let [change_x, change_y] = axis.scaled_wide_raw(change_b);
            add_velocity(body, change_x, change_y);
        }
        let angular_change_b =
            angular_velocity_change_raw(magnitude, inverse_inertia_b, lever_b_q16, inverse_sum_q24);
        add_angular_velocity(body, angular_change_b);
    }
}

#[inline(always)]
fn friction_velocity_change_limit_q10(
    friction_q16: u32,
    normal_velocity_change_q10: u64,
    normal_inverse_sum_q24: u64,
    tangent_inverse_sum_q24: u64,
) -> i64 {
    let numerator = (friction_q16 as u128)
        .saturating_mul(normal_velocity_change_q10 as u128)
        .saturating_mul(tangent_inverse_sum_q24 as u128);
    let denominator = (normal_inverse_sum_q24 as u128) << 16;
    (numerator / denominator).min(i64::MAX as u128) as i64
}

fn relative_normal_speed(a: &Body, b: Option<&Body>, contact: &Contact) -> i32 {
    relative_speed_along(a, b, contact.point, contact.normal)
}

fn relative_speed_along(a: &Body, b: Option<&Body>, point: GeometryPoint, axis: UnitVector) -> i32 {
    let av = a.state().linear_velocity();
    let bv = b
        .map(|body| body.state().linear_velocity())
        .unwrap_or(LinearVelocity::ZERO);
    let linear_speed = axis.dot(bv - av);
    let angular_a = angular_contact_speed_raw(a, point, axis);
    let angular_b = b
        .map(|body| angular_contact_speed_raw(body, point, axis))
        .unwrap_or(0);
    let speed = linear_speed + angular_b - angular_a;
    speed.clamp(
        -(MAX_RELATIVE_CONTACT_SPEED_RAW as i64),
        MAX_RELATIVE_CONTACT_SPEED_RAW as i64,
    ) as i32
}

#[inline(always)]
fn contact_lever_cross_axis(body: &Body, point: GeometryPoint, axis: UnitVector) -> i64 {
    let center = GeometryPoint::from(body.state().transform().position);
    let lever = point - center;
    -axis.cross(lever)
}

#[inline(always)]
fn angular_contact_speed_raw(body: &Body, point: GeometryPoint, axis: UnitVector) -> i64 {
    body.state()
        .angular_velocity()
        .projected_point_speed_raw(contact_lever_cross_axis(body, point, axis))
}

#[inline(always)]
fn rotational_inverse_mass_q24(lever_q16: i64, inverse_inertia_q40: u64) -> u64 {
    // (Q16)^2 * Q40 -> Q72; shift to the inverse-mass Q24 used by k.
    let lever = lever_q16.unsigned_abs() as u128;
    let product = lever * lever * inverse_inertia_q40 as u128;
    let result = (product + (1_u128 << 47)) >> 48;
    result.min(u64::MAX as u128) as u64
}

#[inline(always)]
fn angular_velocity_change_raw(
    velocity_change_q10: u64,
    inverse_inertia_q40: u64,
    lever_q16: i64,
    inverse_sum_q24: u64,
) -> i128 {
    if inverse_inertia_q40 == 0 || lever_q16 == 0 {
        return 0;
    }

    // Q10 * Q40 * Q16 / Q24 -> Q42; the extra shift yields angular Q24.
    let numerator = velocity_change_q10 as u128
        * inverse_inertia_q40 as u128
        * lever_q16.unsigned_abs() as u128;
    let denominator = (inverse_sum_q24 as u128) << 18;
    let magnitude = (numerator + (denominator >> 1)) / denominator;
    if lever_q16 < 0 {
        -(magnitude as i128)
    } else {
        magnitude as i128
    }
}

fn add_velocity(body: &mut Body, dx: i64, dy: i64) {
    let [x, y] = body.state().linear_velocity().raw();
    body.state_mut().linear_velocity =
        LinearVelocity::from_wide_saturated(x as i64 + dx, y as i64 + dy);
}

fn add_angular_velocity(body: &mut Body, delta: i128) {
    let raw = body.state().angular_velocity().raw() as i128 + delta;
    let raw = raw.clamp(i32::MIN as i128, i32::MAX as i128) as i32;
    body.state_mut().angular_velocity = AngularVelocity::from_raw(raw);
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

#[inline(always)]
fn div_round_u128(numerator: u128, denominator: u64) -> u64 {
    debug_assert!(denominator > 0);
    let denominator = denominator as u128;
    let result = (numerator + (denominator >> 1)) / denominator;
    result.min(u64::MAX as u128) as u64
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
            MAX_RELATIVE_CONTACT_SPEED_RAW
        );
    }

    #[test]
    fn maximum_solver_impulse_fits_u64_chain() {
        let normal_speed = -MAX_RELATIVE_CONTACT_SPEED_RAW;
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
    fn off_center_contact_transfers_linear_momentum_into_spin() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, -0.5, 1.0, Material::INELASTIC))
            .unwrap();
        world
            .add_body(circle_body(2, 0.5, -1.0, Material::INELASTIC))
            .unwrap();
        let contact = Contact {
            body_a: BodyId::new(1),
            body_b: BodyId::new(2),
            point: Position::from_meters(0.0, 0.25).unwrap().into(),
            normal: UnitVector::X,
            penetration: Length::ZERO,
        };
        world.contacts.push(contact);
        world.contact_pairs.push(ContactPair {
            a: 0,
            b: ContactBodyIndex::Dynamic(1),
        });

        let mut impulse_states = [ContactImpulseState::default()];
        world.solve_velocities(&mut impulse_states);

        let a = world.body(BodyId::new(1)).unwrap();
        let b = world.body(BodyId::new(2)).unwrap();
        assert_eq!(a.state().linear_velocity().raw(), [341, 0]);
        assert_eq!(b.state().linear_velocity().raw(), [-341, 0]);
        assert!(a.state().angular_velocity().raw() > 0);
        assert!(b.state().angular_velocity().raw() < 0);
        assert!(relative_normal_speed(a, Some(b), &contact).abs() <= 1);
    }

    #[test]
    fn tangent_impulse_stops_sliding_at_a_rough_static_contact() {
        let material = Material::new(0.0, 1.0).unwrap();
        let mut body = circle_body(1, 0.0, 1.0, material);
        body.state_mut()
            .set_linear_velocity(LinearVelocity::from_meters_per_second(1.0, -1.0).unwrap());

        let mut world = zero_gravity_world();
        world.add_body(body).unwrap();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(2),
                Transform::new(Position::from_meters(0.0, -1.0).unwrap(), Angle::ZERO),
                CompositeCollider::single(
                    Circle::new(Length::from_meters(0.5).unwrap())
                        .unwrap()
                        .into(),
                )
                .unwrap(),
                material,
            ))
            .unwrap();
        let contact = Contact {
            body_a: BodyId::new(1),
            body_b: BodyId::new(2),
            point: Position::from_meters(0.0, -0.5).unwrap().into(),
            normal: UnitVector::from_raw(0, -(1 << 30)),
            penetration: Length::ZERO,
        };
        world.contacts.push(contact);
        world.contact_pairs.push(ContactPair {
            a: 0,
            b: ContactBodyIndex::Static(0),
        });

        let mut impulse_states = [ContactImpulseState::default()];
        world.solve_velocities(&mut impulse_states);

        let body = world.body(BodyId::new(1)).unwrap();
        assert_eq!(body.state().linear_velocity().raw(), [683, 0]);
        assert!(body.state().angular_velocity().raw() < 0);
        assert!(
            relative_speed_along(body, None, contact.point, contact.normal.perpendicular(),).abs()
                <= 1
        );

        let after_first_solve = *body.state();
        world.solve_velocities(&mut impulse_states);
        assert_eq!(
            *world.body(BodyId::new(1)).unwrap().state(),
            after_first_solve
        );
    }

    #[test]
    fn tangent_impulse_affects_both_dynamic_bodies() {
        let material = Material::new(0.0, 1.0).unwrap();
        let mut a = circle_body(1, 0.0, 1.0, material);
        a.state_mut()
            .set_linear_velocity(LinearVelocity::from_meters_per_second(1.0, -1.0).unwrap());
        let mut b = circle_body(2, 0.0, 0.0, material);
        b.state_mut().set_transform(Transform::new(
            Position::from_meters(0.0, -1.0).unwrap(),
            Angle::ZERO,
        ));

        let mut world = zero_gravity_world();
        world.add_body(a).unwrap();
        world.add_body(b).unwrap();
        let contact = Contact {
            body_a: BodyId::new(1),
            body_b: BodyId::new(2),
            point: Position::from_meters(0.0, -0.5).unwrap().into(),
            normal: UnitVector::from_raw(0, -(1 << 30)),
            penetration: Length::ZERO,
        };
        world.contacts.push(contact);
        world.contact_pairs.push(ContactPair {
            a: 0,
            b: ContactBodyIndex::Dynamic(1),
        });

        let mut impulse_states = [ContactImpulseState::default()];
        world.solve_velocities(&mut impulse_states);

        let a = world.body(BodyId::new(1)).unwrap();
        let b = world.body(BodyId::new(2)).unwrap();
        assert_eq!(a.state().linear_velocity().raw(), [853, -512]);
        assert_eq!(b.state().linear_velocity().raw(), [171, -512]);
        assert!(a.state().angular_velocity().raw() < 0);
        assert!(b.state().angular_velocity().raw() < 0);
        assert!(
            relative_speed_along(a, Some(b), contact.point, contact.normal.perpendicular(),).abs()
                <= 1
        );
    }

    #[test]
    fn coulomb_limit_scales_with_normal_impulse() {
        let inverse_mass = 1_u64 << 24;

        assert_eq!(
            friction_velocity_change_limit_q10(1 << 13, 1 << 10, inverse_mass, 3 * inverse_mass,),
            384
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
