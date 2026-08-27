use super::constraint::{
    apply_body_impulse, div_round_signed,
    round_shift_signed, scalar_inverse_mass_q24,
};
use crate::body::BodyId;
use crate::quantity::{Length, Position};
use crate::world::World;
use crate::UnitVector;
use crate::world::body::BodyIndex;

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct DistanceImpulseState {
    /// Accumulated bilateral scalar impulse in Q10 kg*m/s.
    accumulated_impulse_q10: i64,
}

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct RopeImpulseState {
    /// Accumulated pulling-only scalar impulse in Q10 kg*m/s.
    accumulated_impulse_q10: i64,
}

pub(super) fn solve_distance_velocities(
    world: &mut World,
    states: &mut [DistanceImpulseState],
    reverse: bool,
) {
    debug_assert_eq!(states.len(), world.distance_joints.len());
    if reverse {
        for index in (0..world.distance_joints.len()).rev() {
            solve_distance_velocity(world, index, &mut states[index]);
        }
    } else {
        for (index, state) in states.iter_mut().enumerate() {
            solve_distance_velocity(world, index, state);
        }
    }
}

pub(super) fn solve_rope_velocities(
    world: &mut World,
    states: &mut [RopeImpulseState],
    reverse: bool,
) {
    debug_assert_eq!(states.len(), world.rope_joints.len());
    if reverse {
        for index in (0..world.rope_joints.len()).rev() {
            solve_rope_velocity(world, index, &mut states[index]);
        }
    } else {
        for (index, state) in states.iter_mut().enumerate() {
            solve_rope_velocity(world, index, state);
        }
    }
}

fn solve_distance_velocity(
    world: &mut World,
    joint_index: usize,
    state: &mut DistanceImpulseState,
) {
    let joint = world.distance_joints[joint_index];
    solve_scalar_constraint(
        world,
        joint.body_a(),
        joint.local_anchor_a(),
        joint.body_b(),
        joint.local_anchor_b(),
        joint.length(),
        joint.max_force().impulse_per_tick_q10(),
        joint.response_raw(),
        false,
        &mut state.accumulated_impulse_q10,
    );
}

fn solve_rope_velocity(world: &mut World, joint_index: usize, state: &mut RopeImpulseState) {
    let joint = world.rope_joints[joint_index];
    solve_scalar_constraint(
        world,
        joint.body_a(),
        joint.local_anchor_a(),
        joint.body_b(),
        joint.local_anchor_b(),
        joint.max_length(),
        joint.max_force().impulse_per_tick_q10(),
        joint.response_raw(),
        true,
        &mut state.accumulated_impulse_q10,
    );
}

#[allow(clippy::too_many_arguments)]
fn solve_scalar_constraint(
    world: &mut World,
    body_a: BodyId,
    local_anchor_a: Position,
    body_b: BodyId,
    local_anchor_b: Position,
    target_length: Length,
    max_impulse_q10: u64,
    response_q16: u32,
    pulling_only: bool,
    accumulated_impulse_q10: &mut i64,
) {
    let Some(endpoint_a) = world.resolve_endpoint(body_a) else {
        return;
    };
    let Some(endpoint_b) = world.resolve_endpoint(body_b) else {
        return;
    };
    let anchor_a = world.endpoint_anchor(endpoint_a, local_anchor_a);
    let anchor_b = world.endpoint_anchor(endpoint_b, local_anchor_b);
    let delta = anchor_b - anchor_a;
    let current_length = delta.squared_magnitude().isqrt();

    if pulling_only && current_length < target_length.raw() as u64 {
        *accumulated_impulse_q10 = 0;
        return;
    }

    // The deterministic fallback also defines how a zero-separation Distance
    // joint with a positive target length starts expanding.
    let axis = UnitVector::normalized_with_length(delta, current_length).unwrap_or(UnitVector::X);
    let dynamic_a = world.endpoint_body(endpoint_a);
    let dynamic_b = world.endpoint_body(endpoint_b);
    let lever_a = dynamic_a
        .map(|body| body.contact_lever_cross_axis(anchor_a, axis))
        .unwrap_or(0);
    let lever_b = dynamic_b
        .map(|body| body.contact_lever_cross_axis(anchor_b, axis))
        .unwrap_or(0);
    let inverse_mass = scalar_inverse_mass_q24(dynamic_a, dynamic_b, lever_a, lever_b);
    if inverse_mass == 0 {
        return;
    }

    // Axis is A -> B. A positive error means the anchors are too far apart,
    // so the desired B-relative-to-A speed and resulting impulse are negative.
    let error_q16 = current_length as i64 - target_length.raw() as i64;
    let desired_speed_q10 = -round_shift_signed(error_q16 * response_q16 as i64, 16);
    let relative_speed_q10 = world.endpoint_speed(endpoint_b, anchor_b, axis)
        - world.endpoint_speed(endpoint_a, anchor_a, axis);
    let velocity_change_q10 = desired_speed_q10.saturating_sub(relative_speed_q10);
    let impulse_change_q10 =
        div_round_signed((velocity_change_q10 as i128) << 24, inverse_mass as u128);

    let previous = *accumulated_impulse_q10;
    let max_impulse = max_impulse_q10.min(i64::MAX as u64) as i64;
    let candidate = previous.saturating_add(impulse_change_q10);
    let candidate = if pulling_only {
        candidate.clamp(-max_impulse, 0)
    } else {
        candidate.clamp(-max_impulse, max_impulse)
    };
    *accumulated_impulse_q10 = candidate;

    let applied = candidate - previous;
    if applied == 0 {
        return;
    }
    if let BodyIndex::Dynamic(index) = endpoint_a {
        apply_body_impulse(&mut world.bodies[index], axis, -applied, lever_a);
    }
    if let BodyIndex::Dynamic(index) = endpoint_b {
        apply_body_impulse(&mut world.bodies[index], axis, applied, lever_b);
    }
}

#[cfg(test)]
mod tests {
    use crate::Body;
    use super::*;
    use crate::body::{BodyState, Material, SleepConfig, StaticBody};
    use crate::collider::Circle;
    use crate::joint::{DistanceJoint, RopeJoint};
    use crate::quantity::{
        Angle, AngularVelocity, Damping, Force, LinearAcceleration, LinearVelocity, Mass,
    };
    use crate::transform::Transform;
    use crate::world::WorldSettings;

    const HIGH_FORCE: f64 = 10_000.0;

    fn dynamic_body(id: u64, x: f64, y: f64, vx: f64, vy: f64) -> Body {
        Body::dynamic(
            BodyId::new(id),
            Circle::new(Length::from_meters(0.1).unwrap()).unwrap(),
            Mass::ONE,
            Material::INELASTIC,
            BodyState::new(
                Transform::new(Position::from_meters(x, y).unwrap(), Angle::ZERO),
                LinearVelocity::from_meters_per_second(vx, vy).unwrap(),
                AngularVelocity::ZERO,
            ),
        )
    }

    fn static_body(id: u64, x: f64, y: f64) -> StaticBody {
        StaticBody::new(
            BodyId::new(id),
            Transform::new(Position::from_meters(x, y).unwrap(), Angle::ZERO),
            Circle::new(Length::from_meters(0.1).unwrap()).unwrap(),
            Material::INELASTIC,
        )
    }

    fn world() -> World {
        let mut settings = WorldSettings::new(LinearAcceleration::ZERO);
        settings.linear_damping = Damping::NONE;
        settings.angular_damping = Damping::NONE;
        World::new(settings)
    }

    fn force(newtons: f64) -> Force {
        Force::from_newtons(newtons).unwrap()
    }

    fn distance(a: u64, b: u64, length: f64, max_force: f64) -> DistanceJoint {
        DistanceJoint::new(
            BodyId::new(a),
            Position::ZERO,
            BodyId::new(b),
            Position::ZERO,
            Length::from_meters(length).unwrap(),
            force(max_force),
        )
    }

    fn rope(a: u64, b: u64, length: f64, max_force: f64) -> RopeJoint {
        RopeJoint::new(
            BodyId::new(a),
            Position::ZERO,
            BodyId::new(b),
            Position::ZERO,
            Length::from_meters(length).unwrap(),
            force(max_force),
        )
    }

    fn center_distance_raw(world: &World, a: u64, b: u64) -> u64 {
        let a = world
            .body(BodyId::new(a))
            .unwrap()
            .state()
            .transform()
            .position;
        let b = world
            .body(BodyId::new(b))
            .unwrap()
            .state()
            .transform()
            .position;
        (b - a).squared_magnitude().isqrt()
    }

    #[test]
    fn distance_axis_and_impulse_signs_are_fixed() {
        let mut too_long = world();
        too_long
            .add_body(dynamic_body(1, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        too_long
            .add_body(dynamic_body(2, 2.0, 0.0, 0.0, 0.0))
            .unwrap();
        too_long
            .add_distance_joint(distance(1, 2, 1.0, HIGH_FORCE))
            .unwrap();
        let mut long_state = DistanceImpulseState::default();
        solve_distance_velocity(&mut too_long, 0, &mut long_state);

        assert!(long_state.accumulated_impulse_q10 < 0);
        assert!(
            too_long
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity()
                .raw()[0]
                > 0
        );
        assert!(
            too_long
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity()
                .raw()[0]
                < 0
        );

        let mut too_short = world();
        too_short
            .add_body(dynamic_body(1, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        too_short
            .add_body(dynamic_body(2, 1.0, 0.0, 0.0, 0.0))
            .unwrap();
        too_short
            .add_distance_joint(distance(1, 2, 2.0, HIGH_FORCE))
            .unwrap();
        let mut short_state = DistanceImpulseState::default();
        solve_distance_velocity(&mut too_short, 0, &mut short_state);

        assert!(short_state.accumulated_impulse_q10 > 0);
        assert!(
            too_short
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity()
                .raw()[0]
                < 0
        );
        assert!(
            too_short
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity()
                .raw()[0]
                > 0
        );
    }

    #[test]
    fn distance_holds_length_for_two_dynamic_bodies() {
        let mut world = world();
        world
            .add_body(dynamic_body(1, -1.0, 0.0, -4.0, 0.0))
            .unwrap();
        world.add_body(dynamic_body(2, 1.0, 0.0, 4.0, 0.0)).unwrap();
        world
            .add_distance_joint(distance(1, 2, 2.0, HIGH_FORCE))
            .unwrap();

        for _ in 0..32 {
            world.step();
        }

        assert!(
            (center_distance_raw(&world, 1, 2) as i64 - (2 * Position::SCALE) as i64).abs() <= 2
        );
        assert_eq!(
            world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity(),
            world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity()
        );
    }

    #[test]
    fn distance_supports_dynamic_static_in_both_canonical_orders() {
        let mut dynamic_first = world();
        dynamic_first
            .add_body(dynamic_body(1, 0.0, 0.0, -4.0, 0.0))
            .unwrap();
        dynamic_first
            .add_static_body(static_body(2, 2.0, 0.0))
            .unwrap();
        dynamic_first
            .add_distance_joint(distance(1, 2, 2.0, HIGH_FORCE))
            .unwrap();
        dynamic_first.step();

        let mut static_first = world();
        static_first
            .add_static_body(static_body(1, 0.0, 0.0))
            .unwrap();
        static_first
            .add_body(dynamic_body(2, 2.0, 0.0, 4.0, 0.0))
            .unwrap();
        static_first
            .add_distance_joint(distance(2, 1, 2.0, HIGH_FORCE))
            .unwrap();
        static_first.step();

        assert_eq!(
            dynamic_first
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::ZERO
        );
        assert_eq!(
            static_first
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::ZERO
        );
    }

    #[test]
    fn off_center_distance_impulse_rotates_dynamic_body() {
        let mut world = world();
        world.add_static_body(static_body(1, 0.0, 0.0)).unwrap();
        world.add_body(dynamic_body(2, 2.0, 0.0, 0.0, 0.0)).unwrap();
        world
            .add_distance_joint(DistanceJoint::new(
                BodyId::new(1),
                Position::ZERO,
                BodyId::new(2),
                Position::from_meters(0.0, 0.5).unwrap(),
                Length::from_meters(1.0).unwrap(),
                force(HIGH_FORCE),
            ))
            .unwrap();

        world.step();

        assert_ne!(
            world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .angular_velocity(),
            AngularVelocity::ZERO
        );
    }

    #[test]
    fn rope_is_slack_inside_limit_and_unilateral_at_limit() {
        let mut slack = world();
        slack.add_static_body(static_body(1, 0.0, 0.0)).unwrap();
        slack.add_body(dynamic_body(2, 1.0, 0.0, 3.0, 0.0)).unwrap();
        slack.add_rope_joint(rope(1, 2, 2.0, HIGH_FORCE)).unwrap();
        slack.step();
        assert_eq!(
            slack
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::from_meters_per_second(3.0, 0.0).unwrap()
        );

        let mut taut = world();
        taut.add_static_body(static_body(1, 0.0, 0.0)).unwrap();
        taut.add_body(dynamic_body(2, 2.0, 0.0, 3.0, 0.0)).unwrap();
        taut.add_rope_joint(rope(1, 2, 2.0, HIGH_FORCE)).unwrap();
        let mut state = RopeImpulseState::default();
        solve_rope_velocity(&mut taut, 0, &mut state);

        assert!(state.accumulated_impulse_q10 < 0);
        assert_eq!(
            taut.body(BodyId::new(2)).unwrap().state().linear_velocity(),
            LinearVelocity::ZERO
        );
    }

    #[test]
    fn rope_never_pushes_anchors_apart() {
        let mut world = world();
        world.add_static_body(static_body(1, 0.0, 0.0)).unwrap();
        world
            .add_body(dynamic_body(2, 2.0, 0.0, -3.0, 0.0))
            .unwrap();
        world.add_rope_joint(rope(1, 2, 2.0, HIGH_FORCE)).unwrap();
        let mut state = RopeImpulseState::default();

        solve_rope_velocity(&mut world, 0, &mut state);

        assert_eq!(state.accumulated_impulse_q10, 0);
        assert_eq!(
            world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::from_meters_per_second(-3.0, 0.0).unwrap()
        );
    }

    #[test]
    fn rope_supports_dynamic_dynamic_and_dynamic_static_in_both_orders() {
        let mut dynamic_pair = world();
        dynamic_pair
            .add_body(dynamic_body(1, -1.0, 0.0, -3.0, 0.0))
            .unwrap();
        dynamic_pair
            .add_body(dynamic_body(2, 1.0, 0.0, 3.0, 0.0))
            .unwrap();
        dynamic_pair
            .add_rope_joint(rope(2, 1, 2.0, HIGH_FORCE))
            .unwrap();
        dynamic_pair.step();

        let mut dynamic_first = world();
        dynamic_first
            .add_body(dynamic_body(1, 0.0, 0.0, -3.0, 0.0))
            .unwrap();
        dynamic_first
            .add_static_body(static_body(2, 2.0, 0.0))
            .unwrap();
        dynamic_first
            .add_rope_joint(rope(1, 2, 2.0, HIGH_FORCE))
            .unwrap();
        dynamic_first.step();

        let mut static_first = world();
        static_first
            .add_static_body(static_body(1, 0.0, 0.0))
            .unwrap();
        static_first
            .add_body(dynamic_body(2, 2.0, 0.0, 3.0, 0.0))
            .unwrap();
        static_first
            .add_rope_joint(rope(2, 1, 2.0, HIGH_FORCE))
            .unwrap();
        static_first.step();

        assert_eq!(
            dynamic_pair
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::ZERO
        );
        assert_eq!(
            dynamic_pair
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::ZERO
        );
        assert_eq!(
            dynamic_first
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::ZERO
        );
        assert_eq!(
            static_first
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity(),
            LinearVelocity::ZERO
        );
    }

    #[test]
    fn off_center_rope_impulse_rotates_dynamic_body() {
        let mut world = world();
        world.add_static_body(static_body(1, 0.0, 0.0)).unwrap();
        world.add_body(dynamic_body(2, 2.0, 0.0, 0.0, 0.0)).unwrap();
        world
            .add_rope_joint(RopeJoint::new(
                BodyId::new(1),
                Position::ZERO,
                BodyId::new(2),
                Position::from_meters(0.0, 0.5).unwrap(),
                Length::from_meters(1.0).unwrap(),
                force(HIGH_FORCE),
            ))
            .unwrap();

        world.step();

        assert_ne!(
            world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .angular_velocity(),
            AngularVelocity::ZERO
        );
    }

    #[test]
    fn shortening_distance_and_reeling_rope_pull_inward() {
        let mut distance_world = world();
        distance_world
            .add_static_body(static_body(1, 0.0, 0.0))
            .unwrap();
        distance_world
            .add_body(dynamic_body(2, 2.0, 0.0, 0.0, 0.0))
            .unwrap();
        distance_world
            .add_distance_joint(distance(1, 2, 2.0, HIGH_FORCE))
            .unwrap();
        distance_world
            .distance_joint_mut(BodyId::new(1), BodyId::new(2))
            .unwrap()
            .set_length(Length::from_meters(1.0).unwrap());
        distance_world.step();

        let mut rope_world = world();
        rope_world
            .add_static_body(static_body(1, 0.0, 0.0))
            .unwrap();
        rope_world
            .add_body(dynamic_body(2, 2.0, 0.0, 0.0, 0.0))
            .unwrap();
        rope_world
            .add_rope_joint(rope(1, 2, 2.0, HIGH_FORCE))
            .unwrap();
        rope_world
            .rope_joint_mut(BodyId::new(2), BodyId::new(1))
            .unwrap()
            .set_max_length(Length::from_meters(1.0).unwrap());
        rope_world.step();

        assert!(
            distance_world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity()
                .raw()[0]
                < 0
        );
        assert!(
            rope_world
                .body(BodyId::new(2))
                .unwrap()
                .state()
                .linear_velocity()
                .raw()[0]
                < 0
        );
    }

    #[test]
    fn distance_max_force_caps_accumulated_impulse_per_tick() {
        let mut distance_world = world();
        distance_world
            .add_body(dynamic_body(1, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        distance_world
            .add_static_body(static_body(2, 10.0, 0.0))
            .unwrap();
        distance_world
            .add_distance_joint(distance(1, 2, 1.0, 1.0))
            .unwrap();

        distance_world.step();

        assert_eq!(
            distance_world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity()
                .raw(),
            [16, 0]
        );

        let mut rope_world = world();
        rope_world
            .add_body(dynamic_body(1, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        rope_world
            .add_static_body(static_body(2, 10.0, 0.0))
            .unwrap();
        rope_world.add_rope_joint(rope(1, 2, 1.0, 1.0)).unwrap();

        rope_world.step();

        assert_eq!(
            rope_world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity()
                .raw(),
            [16, 0]
        );
    }

    #[test]
    fn joint_storage_order_does_not_change_replay() {
        fn scene(reverse: bool) -> World {
            let mut world = world();
            let bodies = [
                dynamic_body(1, -2.0, 0.0, 0.5, 0.0),
                dynamic_body(2, 0.0, 0.0, 0.0, 0.0),
                dynamic_body(3, 2.0, 0.0, -0.5, 0.0),
            ];
            if reverse {
                for body in bodies.into_iter().rev() {
                    world.add_body(body).unwrap();
                }
                world
                    .add_distance_joint(distance(2, 3, 1.5, HIGH_FORCE))
                    .unwrap();
                world.add_rope_joint(rope(3, 1, 3.0, HIGH_FORCE)).unwrap();
                world
                    .add_distance_joint(distance(2, 1, 1.5, HIGH_FORCE))
                    .unwrap();
            } else {
                for body in bodies {
                    world.add_body(body).unwrap();
                }
                world
                    .add_distance_joint(distance(1, 2, 1.5, HIGH_FORCE))
                    .unwrap();
                world
                    .add_distance_joint(distance(2, 3, 1.5, HIGH_FORCE))
                    .unwrap();
                world.add_rope_joint(rope(1, 3, 3.0, HIGH_FORCE)).unwrap();
            }
            world
        }

        let mut forward = scene(false);
        let mut reverse = scene(true);
        for _ in 0..64 {
            forward.step();
            reverse.step();
        }

        assert_eq!(forward.bodies(), reverse.bodies());
        assert_eq!(forward.distance_joints(), reverse.distance_joints());
        assert_eq!(forward.rope_joints(), reverse.rope_joints());
    }

    #[test]
    fn resting_distance_is_sleepable_and_mutation_wakes_endpoints() {
        let mut world = world();
        world.add_body(dynamic_body(1, 0.0, 0.0, 0.0, 0.0)).unwrap();
        world.add_body(dynamic_body(2, 2.0, 0.0, 0.0, 0.0)).unwrap();
        world
            .add_distance_joint(distance(1, 2, 2.0, HIGH_FORCE))
            .unwrap();

        for _ in 0..SleepConfig::FAST_EFFECTS.required_ticks() {
            world.step();
        }
        assert!(world.body(BodyId::new(1)).unwrap().state().is_sleeping());
        assert!(world.body(BodyId::new(2)).unwrap().state().is_sleeping());

        world
            .distance_joint_mut(BodyId::new(1), BodyId::new(2))
            .unwrap()
            .set_length(Length::from_meters(1.0).unwrap());
        assert!(!world.body(BodyId::new(1)).unwrap().state().is_sleeping());
        assert!(!world.body(BodyId::new(2)).unwrap().state().is_sleeping());
    }
}
