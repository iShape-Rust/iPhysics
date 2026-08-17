use super::{ContactBodyIndex, ContactPair, World};
use crate::body::{Body, BodyId};
use crate::collision::{Contact, collide};
use crate::quantity::{LinearVelocity, Position};
use alloc::vec;

const WAKE_SPEED_RAW: i64 = 3_355_443; // 0.2 m/s in Q24
const WAKE_PENETRATION_RAW: u32 = 655; // approximately 0.01 m in Q16
const POSITION_SLOP_RAW: u32 = 64; // 1/1024 m
const MAX_POSITION_CORRECTION_RAW: u32 = 16_384; // 0.25 m

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepError {
    BoundaryOverflow(BodyId),
    VelocityOverflow(BodyId),
    PositionOverflow(BodyId),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StepStats {
    pub tested_pairs: usize,
    pub aabb_pairs: usize,
    pub contacts: usize,
    pub sleeping_bodies: usize,
}

impl World {
    pub fn step(&mut self) -> Result<StepStats, StepError> {
        self.integrate_velocities()?;

        let mut stats = self.build_contacts()?;
        self.wake_impacted_bodies();

        for _ in 0..self.settings.velocity_iterations.max(1) {
            self.solve_velocities()?;
        }
        self.correct_positions()?;
        self.integrate_transforms()?;

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

        Ok(stats)
    }

    fn integrate_velocities(&mut self) -> Result<(), StepError> {
        for body in &mut self.bodies {
            if body.state().is_sleeping() {
                continue;
            }

            body.state_mut().linear_velocity = body
                .state()
                .linear_velocity()
                .checked_advance(self.settings.gravity)
                .ok_or(StepError::VelocityOverflow(body.id()))?;
        }
        Ok(())
    }

    fn build_contacts(&mut self) -> Result<StepStats, StepError> {
        self.contacts.clear();
        self.contact_pairs.clear();
        let mut stats = StepStats::default();

        for index_a in 0..self.bodies.len() {
            let a = &self.bodies[index_a];
            let aabb_a = a
                .collider()
                .aabb(a.state().transform())
                .ok_or(StepError::BoundaryOverflow(a.id()))?;

            for index_b in index_a + 1..self.bodies.len() {
                let b = &self.bodies[index_b];
                if a.state().is_sleeping() && b.state().is_sleeping() {
                    continue;
                }

                stats.tested_pairs += 1;
                let aabb_b = b
                    .collider()
                    .aabb(b.state().transform())
                    .ok_or(StepError::BoundaryOverflow(b.id()))?;
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
                    let part_transform = static_body
                        .transform()
                        .checked_compose(part.local_transform())
                        .ok_or(StepError::BoundaryOverflow(static_body.id()))?;
                    let part_aabb = part
                        .collider()
                        .aabb(part_transform)
                        .ok_or(StepError::BoundaryOverflow(static_body.id()))?;
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
        Ok(stats)
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

    fn solve_velocities(&mut self) -> Result<(), StepError> {
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
                ) as i128;
                let velocity_change =
                    round_shift(-(normal_speed as i128) * ((1_i128 << 16) + restitution), 16);
                let [nx, ny] = contact.normal.raw();
                add_velocity(
                    a,
                    -round_shift(nx as i128 * velocity_change, 30),
                    -round_shift(ny as i128 * velocity_change, 30),
                )?;
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
                .max(b.material().restitution_raw()) as i128;
            let velocity_change =
                round_shift(-(normal_speed as i128) * ((1_i128 << 16) + restitution), 16);
            let change_a = div_round(velocity_change * inverse_a as i128, inverse_sum as i128);
            let change_b = div_round(velocity_change * inverse_b as i128, inverse_sum as i128);
            let [nx, ny] = contact.normal.raw();

            if inverse_a != 0 {
                add_velocity(
                    a,
                    -round_shift(nx as i128 * change_a, 30),
                    -round_shift(ny as i128 * change_a, 30),
                )?;
            }
            if inverse_b != 0 {
                add_velocity(
                    b,
                    round_shift(nx as i128 * change_b, 30),
                    round_shift(ny as i128 * change_b, 30),
                )?;
            }
        }
        Ok(())
    }

    fn correct_positions(&mut self) -> Result<(), StepError> {
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
                let [nx, ny] = contact.normal.raw();
                add_position(
                    &mut self.bodies[pair.a],
                    -round_shift(nx as i128 * correction as i128, 30),
                    -round_shift(ny as i128 * correction as i128, 30),
                )?;
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

            let move_a = div_round(correction as i128 * inverse_a as i128, inverse_sum as i128);
            let move_b = div_round(correction as i128 * inverse_b as i128, inverse_sum as i128);
            let [nx, ny] = contact.normal.raw();
            if inverse_a != 0 {
                add_position(
                    a,
                    -round_shift(nx as i128 * move_a, 30),
                    -round_shift(ny as i128 * move_a, 30),
                )?;
            }
            if inverse_b != 0 {
                add_position(
                    b,
                    round_shift(nx as i128 * move_b, 30),
                    round_shift(ny as i128 * move_b, 30),
                )?;
            }
        }
        Ok(())
    }

    fn integrate_transforms(&mut self) -> Result<(), StepError> {
        for body in &mut self.bodies {
            if body.state().is_sleeping() {
                continue;
            }

            let next = body
                .state()
                .transform()
                .checked_advance(
                    body.state().linear_velocity(),
                    body.state().angular_velocity(),
                )
                .ok_or(StepError::PositionOverflow(body.id()))?;
            body.state_mut().transform = next;
        }
        Ok(())
    }
}

fn relative_normal_speed(a: &Body, b: Option<&Body>, contact: &Contact) -> i64 {
    let [avx, avy] = a.state().linear_velocity().raw();
    let [bvx, bvy] = b
        .map(|body| body.state().linear_velocity().raw())
        .unwrap_or([0, 0]);
    let [nx, ny] = contact.normal.raw();
    round_shift(
        (bvx as i128 - avx as i128) * nx as i128 + (bvy as i128 - avy as i128) * ny as i128,
        30,
    ) as i64
}

fn add_velocity(body: &mut Body, dx: i128, dy: i128) -> Result<(), StepError> {
    let [x, y] = body.state().linear_velocity().raw();
    let x = i32::try_from(x as i128 + dx).map_err(|_| StepError::VelocityOverflow(body.id()))?;
    let y = i32::try_from(y as i128 + dy).map_err(|_| StepError::VelocityOverflow(body.id()))?;
    body.state_mut().linear_velocity = LinearVelocity::from_raw(x, y);
    Ok(())
}

fn add_position(body: &mut Body, dx: i128, dy: i128) -> Result<(), StepError> {
    let [x, y] = body.state().transform().position.raw();
    let x = i32::try_from(x as i128 + dx).map_err(|_| StepError::PositionOverflow(body.id()))?;
    let y = i32::try_from(y as i128 + dy).map_err(|_| StepError::PositionOverflow(body.id()))?;
    body.state_mut().transform.position = Position::from_raw(x, y);
    Ok(())
}

fn two_bodies_mut(bodies: &mut [Body], a: usize, b: usize) -> (&mut Body, &mut Body) {
    debug_assert!(a < b);
    let (left, right) = bodies.split_at_mut(b);
    (&mut left[a], &mut right[0])
}

#[inline(always)]
fn round_shift(value: i128, shift: u32) -> i128 {
    let half = 1_i128 << (shift - 1);
    if value < 0 {
        -((-value + half) >> shift)
    } else {
        (value + half) >> shift
    }
}

#[inline(always)]
fn div_round(numerator: i128, denominator: i128) -> i128 {
    debug_assert!(numerator >= 0 && denominator > 0);
    (numerator + (denominator >> 1)) / denominator
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyState, Material, SleepConfig, StaticBody};
    use crate::collider::{Circle, ColliderPart, CompositeCollider};
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
    fn elastic_equal_mass_circles_exchange_velocity() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, -0.5, 1.0, Material::ELASTIC))
            .unwrap();
        world
            .add_body(circle_body(2, 0.5, -1.0, Material::ELASTIC))
            .unwrap();

        let stats = world.step().unwrap();

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

        first.step().unwrap();
        let mut replay = first.clone();
        first.contacts.clear();

        for _ in 0..32 {
            first.step().unwrap();
            replay.step().unwrap();
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
            .add_static_body(
                StaticBody::new(
                    BodyId::new(2),
                    Transform::new(Position::from_meters(1.0, 0.0).unwrap(), Angle::ZERO),
                    CompositeCollider::single(
                        Circle::new(Length::from_meters(0.5).unwrap())
                            .unwrap()
                            .into(),
                    )
                    .unwrap(),
                    Material::INELASTIC,
                )
                .unwrap(),
            )
            .unwrap();

        for _ in 0..SleepConfig::FAST_EFFECTS.required_ticks() {
            world.step().unwrap();
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
            .add_static_body(
                StaticBody::new(
                    BodyId::new(2),
                    Transform::IDENTITY,
                    composite,
                    Material::INELASTIC,
                )
                .unwrap(),
            )
            .unwrap();

        let stats = world.step().unwrap();

        assert_eq!(stats.contacts, 1);
        assert_eq!(world.contacts()[0].body_a, BodyId::new(1));
        assert_eq!(world.contacts()[0].body_b, BodyId::new(2));
    }
}
