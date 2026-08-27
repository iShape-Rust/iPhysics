use super::constraint::{
    apply_body_impulse, contact_inverse_mass_q24, div_round_signed,
    round_shift_signed,
};
use crate::body::Body;
use crate::world::World;
use crate::{GeometryPoint, UnitVector};

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct MouseImpulseState {
    /// Accumulated world-space impulse in Q10 kg*m/s.
    impulse_q10: [i64; 2],
}

pub(super) fn solve_velocities(
    world: &mut World,
    impulse_states: &mut [MouseImpulseState],
    reverse: bool,
) {
    debug_assert_eq!(impulse_states.len(), world.mouse_joints.len());

    if reverse {
        for index in (0..world.mouse_joints.len()).rev() {
            solve_velocity(world, index, &mut impulse_states[index]);
        }
    } else {
        for (index, impulse_state) in impulse_states.iter_mut().enumerate() {
            solve_velocity(world, index, impulse_state);
        }
    }
}

fn solve_velocity(world: &mut World, joint_index: usize, impulse_state: &mut MouseImpulseState) {
    let joint = world.mouse_joints[joint_index];
    let Ok(body_index) = world.bodies.binary_search_by_key(&joint.body(), Body::id) else {
        return;
    };
    let body = &mut world.bodies[body_index];
    let anchor = body
        .state()
        .transform()
        .apply_geometry(joint.local_anchor());
    let error = GeometryPoint::from(joint.target()) - anchor;
    let [error_x, error_y] = error.raw();
    let response = joint.response_raw() as i64;
    let desired_velocity = [
        round_shift_signed(error_x as i64 * response, 16),
        round_shift_signed(error_y as i64 * response, 16),
    ];
    let max_impulse = joint.max_force().impulse_per_tick_q10();

    solve_constraint(body, anchor, desired_velocity, max_impulse, impulse_state);
}

fn solve_constraint(
    body: &mut Body,
    anchor: GeometryPoint,
    desired_velocity_q10: [i64; 2],
    max_impulse_q10: u64,
    impulse_state: &mut MouseImpulseState,
) {
    let x_axis = UnitVector::X;
    let y_axis = x_axis.perpendicular();
    let lever_x = body.contact_lever_cross_axis(anchor, x_axis);
    let lever_y = body.contact_lever_cross_axis(anchor, y_axis);
    let inverse_xx = contact_inverse_mass_q24(body, None, lever_x, 0);
    let inverse_yy = contact_inverse_mass_q24(body, None, lever_y, 0);
    let inverse_xy =
        rotational_inverse_mass_cross_q24(lever_x, lever_y, body.inverse_inertia_q40());
    let determinant = (inverse_xx as u128 * inverse_yy as u128)
        .saturating_sub(inverse_xy.unsigned_abs() * inverse_xy.unsigned_abs());
    if determinant == 0 {
        return;
    }

    let velocity_change_x =
        desired_velocity_q10[0].saturating_sub(body.point_speed_along(anchor, x_axis));
    let velocity_change_y =
        desired_velocity_q10[1].saturating_sub(body.point_speed_along(anchor, y_axis));
    let impulse_change = [
        div_round_signed(
            (inverse_yy as i128 * velocity_change_x as i128
                - inverse_xy * velocity_change_y as i128)
                << 24,
            determinant,
        ),
        div_round_signed(
            (inverse_xx as i128 * velocity_change_y as i128
                - inverse_xy * velocity_change_x as i128)
                << 24,
            determinant,
        ),
    ];
    let previous = impulse_state.impulse_q10;
    let mut candidate = [
        previous[0].saturating_add(impulse_change[0]),
        previous[1].saturating_add(impulse_change[1]),
    ];
    candidate = clamp_impulse_vector(candidate, max_impulse_q10);
    impulse_state.impulse_q10 = candidate;

    for (impulse_component, impulse_axis) in [x_axis, y_axis].into_iter().enumerate() {
        let applied = candidate[impulse_component] - previous[impulse_component];
        let impulse_lever = body.contact_lever_cross_axis(anchor, impulse_axis);
        apply_body_impulse(body, impulse_axis, applied, impulse_lever);
    }
}

#[inline(always)]
fn rotational_inverse_mass_cross_q24(
    lever_a_q16: i32,
    lever_b_q16: i32,
    inverse_inertia_q40: u64,
) -> i128 {
    let lever_product = lever_a_q16 as i64 * lever_b_q16 as i64;
    let product = lever_product.unsigned_abs() as u128 * inverse_inertia_q40 as u128;
    let magnitude = ((product + (1_u128 << 47)) >> 48).min(u64::MAX as u128) as i128;
    if lever_product < 0 {
        -magnitude
    } else {
        magnitude
    }
}

fn clamp_impulse_vector(candidate: [i64; 2], max_impulse: u64) -> [i64; 2] {
    if max_impulse == 0 {
        return [0, 0];
    }
    let x = candidate[0].unsigned_abs() as u128;
    let y = candidate[1].unsigned_abs() as u128;
    let magnitude_squared = (x * x).saturating_add(y * y);
    let max_squared = max_impulse as u128 * max_impulse as u128;
    if magnitude_squared <= max_squared {
        return candidate;
    }

    let magnitude = magnitude_squared.isqrt();
    [
        scale_signed(candidate[0], max_impulse, magnitude),
        scale_signed(candidate[1], max_impulse, magnitude),
    ]
}

#[inline(always)]
fn scale_signed(value: i64, numerator: u64, denominator: u128) -> i64 {
    let magnitude = value.unsigned_abs() as u128 * numerator as u128 / denominator;
    if value < 0 {
        -(magnitude as i64)
    } else {
        magnitude as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyId, BodyState, Material};
    use crate::collider::Circle;
    use crate::joint::MouseJoint;
    use crate::quantity::{
        Angle, AngularVelocity, Force, Length, LinearAcceleration, LinearVelocity, Mass, Position,
    };
    use crate::transform::Transform;
    use crate::world::WorldSettings;

    fn circle_body(id: u64) -> Body {
        Body::dynamic(
            BodyId::new(id),
            Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
            Mass::ONE,
            Material::INELASTIC,
            BodyState::new(
                Transform::new(Position::ZERO, Angle::ZERO),
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
            ),
        )
    }

    fn zero_gravity_world() -> World {
        World::new(WorldSettings::new(LinearAcceleration::ZERO))
    }

    #[test]
    fn mouse_joint_pulls_body_toward_target() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1)).unwrap();
        world
            .add_mouse_joint(MouseJoint::new(
                BodyId::new(1),
                Position::ZERO,
                Position::from_meters(2.0, 0.0).unwrap(),
                Force::from_newtons(100.0).unwrap(),
            ))
            .unwrap();

        for _ in 0..32 {
            world.step();
        }

        let x = world
            .body(BodyId::new(1))
            .unwrap()
            .state()
            .transform()
            .position
            .to_meters()[0];
        assert!((x - 2.0).abs() < 0.01, "body stopped at x={x}");
    }

    #[test]
    fn mouse_joint_force_is_limited_per_tick() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1)).unwrap();
        world
            .add_mouse_joint(MouseJoint::new(
                BodyId::new(1),
                Position::ZERO,
                Position::from_meters(100.0, 0.0).unwrap(),
                Force::from_newtons(1.0).unwrap(),
            ))
            .unwrap();

        world.step();

        assert_eq!(
            world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .linear_velocity()
                .raw(),
            [16, 0]
        );
    }

    #[test]
    fn diagonal_mouse_force_uses_a_circular_limit() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1)).unwrap();
        world
            .add_mouse_joint(MouseJoint::new(
                BodyId::new(1),
                Position::ZERO,
                Position::from_meters(100.0, 100.0).unwrap(),
                Force::from_newtons(1.0).unwrap(),
            ))
            .unwrap();

        world.step();

        let velocity = world
            .body(BodyId::new(1))
            .unwrap()
            .state()
            .linear_velocity();
        assert!(velocity.raw_sqr_magnitude() <= 16 * 16);
        assert!(velocity.raw()[0] > 0 && velocity.raw()[1] > 0);
    }

    #[test]
    fn off_center_mouse_joint_generates_rotation() {
        let mut world = zero_gravity_world();
        world.add_body(circle_body(1)).unwrap();
        world
            .add_mouse_joint(MouseJoint::new(
                BodyId::new(1),
                Position::from_meters(0.0, 0.5).unwrap(),
                Position::from_meters(2.0, 0.5).unwrap(),
                Force::from_newtons(100.0).unwrap(),
            ))
            .unwrap();

        world.step();

        assert_ne!(
            world
                .body(BodyId::new(1))
                .unwrap()
                .state()
                .angular_velocity(),
            AngularVelocity::ZERO
        );
    }
}
