pub(crate) mod constraint;
mod contact_detection;
mod contact_solver;
mod joint_solver;
mod sleep;

pub(in crate::world) use contact_detection::BroadPhaseScratch;

use super::World;
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
        self.wake_mouse_joints_bodies();
        self.wake_distance_joints_bodies();
        self.integrate_velocities();

        let mut stats = self.build_contacts();
        self.wake_impacted_bodies();
        self.decay_deferred_contact_impulses();

        let mut contact_constraints = contact_solver::prepare_constraints(self);
        self.prepare_warm_start(&mut contact_constraints);
        let mut mouse_states =
            vec![joint_solver::MouseImpulseState::default(); self.mouse_joints.len()];
        let mut distance_states =
            vec![joint_solver::DistanceImpulseState::default(); self.distance_joints.len()];
        let mut rope_states =
            vec![joint_solver::RopeImpulseState::default(); self.rope_joints.len()];
        let mut reverse = false;
        for _ in 0..self.settings.velocity_iterations.max(1) {
            self.solve_distance_joints_velocities(&mut distance_states, reverse);
            self.solve_rope_joints_velocities(&mut rope_states, reverse);
            self.solve_mouse_joints_velocities(&mut mouse_states, reverse);
            self.solve_velocities(&mut contact_constraints, false);
            reverse = !reverse;
        }
        self.capture_blocked_contact_impulses(&contact_constraints);
        contact_solver::rebuild_contact_cache(self, &contact_constraints);
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Body;
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
        assert_eq!(first.hot_contacts[0].len(), 1);
        let mut replay = first.clone();
        first.active_contacts.clear();

        for _ in 0..32 {
            first.step();
            replay.step();
        }

        assert_eq!(first.bodies(), replay.bodies());
        assert_eq!(first.active_contacts, replay.active_contacts);
        assert_eq!(first.hot_contacts, replay.hot_contacts);
    }

    #[test]
    fn vanished_contact_is_forgotten_before_it_returns() {
        let mut world = zero_gravity_world();
        world
            .add_body(circle_body(1, 1.0, -1.0, Material::INELASTIC))
            .unwrap();
        world
            .add_static_body(StaticBody::new(
                BodyId::new(2),
                Transform::IDENTITY,
                Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
                Material::INELASTIC,
            ))
            .unwrap();

        world.step();
        assert_eq!(world.hot_contacts[0].len(), 1);

        world
            .body_mut(BodyId::new(1))
            .unwrap()
            .state_mut()
            .set_transform(Transform::new(
                Position::from_meters(5.0, 0.0).unwrap(),
                Angle::ZERO,
            ));
        world.step();
        assert_eq!(world.hot_contacts[0].len(), 0);

        let body = world.body_mut(BodyId::new(1)).unwrap();
        body.state_mut().set_transform(Transform::new(
            Position::from_meters(1.0, 0.0).unwrap(),
            Angle::ZERO,
        ));
        body.state_mut().set_linear_velocity(LinearVelocity::ZERO);
        world.step();

        assert_eq!(
            world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::ZERO
        );
        assert_eq!(world.hot_contacts[0].len(), 0);
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
