mod constraint;
mod contact_detection;
mod contact_solver;
mod joint_solver;
mod mouse_solver;

pub(in crate::world) use contact_detection::BroadPhaseScratch;

use super::{ContactBodyIndex, World};
use crate::body::Body;
use alloc::vec;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StepStats {
    pub tested_pairs: usize,
    pub aabb_pairs: usize,
    pub contacts: usize,
    pub sleeping_bodies: usize,
}

impl World {
    pub fn step(&mut self) -> StepStats {
        mouse_solver::wake_bodies(self);
        joint_solver::wake_connected_bodies(self);
        self.integrate_velocities();

        let mut stats = contact_detection::build_contacts(self);
        contact_detection::wake_impacted_bodies(self);

        let mut contact_states =
            vec![contact_solver::ContactImpulseState::default(); self.active_contacts.len()];
        let mut mouse_states =
            vec![mouse_solver::MouseImpulseState::default(); self.mouse_joints.len()];
        let mut distance_states =
            vec![joint_solver::DistanceImpulseState::default(); self.distance_joints.len()];
        let mut rope_states =
            vec![joint_solver::RopeImpulseState::default(); self.rope_joints.len()];
        for iteration in 0..self.settings.velocity_iterations.max(1) {
            let reverse = iteration % 2 != 0;
            joint_solver::solve_distance_velocities(self, &mut distance_states, reverse);
            joint_solver::solve_rope_velocities(self, &mut rope_states, reverse);
            mouse_solver::solve_velocities(self, &mut mouse_states, reverse);
            contact_solver::solve_velocities(self, &mut contact_states);
        }
        contact_solver::correct_positions(self);
        self.integrate_transforms();
        self.update_sleep_states(&mut stats);

        stats
    }

    fn integrate_velocities(&mut self) {
        for body in &mut self.bodies {
            if body.state().is_sleeping() {
                continue;
            }

            let linear_velocity = self
                .settings
                .linear_damping
                .apply_linear(body.state().linear_velocity())
                .advance(self.settings.gravity);
            let angular_velocity = self
                .settings
                .angular_damping
                .apply_angular(body.state().angular_velocity());
            body.state_mut().linear_velocity = linear_velocity;
            body.state_mut().angular_velocity = angular_velocity;
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

    fn update_sleep_states(&mut self, stats: &mut StepStats) {
        let mut has_contact = vec![false; self.bodies.len()];
        for contact in self.active_contacts.iter().copied() {
            has_contact[contact.body_a] = true;
            if let ContactBodyIndex::Dynamic(index_b) = contact.body_b {
                has_contact[index_b] = true;
            }
        }

        let mut has_mouse_joint = vec![false; self.bodies.len()];
        for joint in &self.mouse_joints {
            if let Ok(index) = self.bodies.binary_search_by_key(&joint.body(), Body::id) {
                has_mouse_joint[index] = true;
            }
        }

        let mut has_joint_constraint = vec![false; self.bodies.len()];
        joint_solver::mark_sleep_constraints(self, &mut has_joint_constraint);

        for (index, body) in self.bodies.iter_mut().enumerate() {
            body.state_mut().update_sleep(
                (has_contact[index] || has_joint_constraint[index]) && !has_mouse_joint[index],
                self.settings.sleep,
            );
            if body.state().is_sleeping() {
                stats.sleeping_bodies += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyId, BodyState, Material, SleepConfig, StaticBody};
    use crate::collider::Circle;
    use crate::quantity::{
        Angle, AngularVelocity, Damping, Length, LinearAcceleration, LinearVelocity, Mass, Position,
    };
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
        first.active_contacts.clear();

        for _ in 0..32 {
            first.step();
            replay.step();
        }

        assert_eq!(first.bodies(), replay.bodies());
    }

    #[test]
    fn damping_precedes_transform_integration() {
        let mut settings = WorldSettings::new(LinearAcceleration::ZERO);
        settings.linear_damping = Damping::new(0.25).unwrap();
        settings.angular_damping = Damping::new(0.25).unwrap();
        let mut world = World::new(settings);
        world
            .add_body(Body::dynamic(
                BodyId::new(1),
                Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
                Mass::ONE,
                Material::INELASTIC,
                BodyState::new(
                    Transform::IDENTITY,
                    LinearVelocity::from_meters_per_second(4.0, 0.0).unwrap(),
                    AngularVelocity::from_radians_per_second(4.0).unwrap(),
                ),
            ))
            .unwrap();

        world.step();

        let state = world.body(BodyId::new(1)).unwrap().state();
        assert_eq!(
            state.linear_velocity(),
            LinearVelocity::from_meters_per_second(3.0, 0.0).unwrap()
        );
        assert_eq!(
            state.angular_velocity(),
            AngularVelocity::from_radians_per_second(3.0).unwrap()
        );
        assert_eq!(state.transform().position.raw(), [3_072, 0]);
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
                Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
                Material::INELASTIC,
            ))
            .unwrap();

        for _ in 0..SleepConfig::FAST_EFFECTS.required_ticks() {
            world.step();
        }

        assert!(world.body(BodyId::new(1)).unwrap().state().is_sleeping());
    }
}
