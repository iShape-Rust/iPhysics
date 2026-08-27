use crate::body::Body;
use crate::quantity::LinearVelocity;
use crate::{GeometryPoint, UnitVector};

pub(super) const MAX_RELATIVE_CONTACT_SPEED_RAW: i32 = 4 * LinearVelocity::MAX_VELOCITY;

#[inline(always)]
pub(super) fn contact_inverse_mass_q24(
    a: &Body,
    b: Option<&Body>,
    lever_a_q16: i32,
    lever_b_q16: i32,
) -> u64 {
    scalar_inverse_mass_q24(Some(a), b, lever_a_q16, lever_b_q16)
}

#[inline(always)]
pub(super) fn scalar_inverse_mass_q24(
    a: Option<&Body>,
    b: Option<&Body>,
    lever_a_q16: i32,
    lever_b_q16: i32,
) -> u64 {
    let inverse_a = a.map(Body::inverse_mass_q24).unwrap_or(0) as u64;
    let inverse_b = b.map(Body::inverse_mass_q24).unwrap_or(0) as u64;
    let inverse_inertia_a = a.map(Body::inverse_inertia_q40).unwrap_or(0);
    let inverse_inertia_b = b.map(Body::inverse_inertia_q40).unwrap_or(0);
    inverse_a
        .saturating_add(inverse_b)
        .saturating_add(rotational_inverse_mass_q24(lever_a_q16, inverse_inertia_a))
        .saturating_add(rotational_inverse_mass_q24(lever_b_q16, inverse_inertia_b))
}

pub(super) fn relative_normal_speed(
    a: &Body,
    b: Option<&Body>,
    point: GeometryPoint,
    normal: UnitVector,
) -> i32 {
    relative_speed_along(a, b, point, normal)
}

pub(super) fn relative_speed_along(
    a: &Body,
    b: Option<&Body>,
    point: GeometryPoint,
    axis: UnitVector,
) -> i32 {
    let lever_a_q16 = a.contact_lever_cross_axis(point, axis);
    let lever_b_q16 = b
        .map(|body| body.contact_lever_cross_axis(point, axis))
        .unwrap_or(0);
    relative_speed_along_levers(a, b, axis, lever_a_q16, lever_b_q16)
}

#[inline(always)]
pub(super) fn relative_speed_along_levers(
    a: &Body,
    b: Option<&Body>,
    axis: UnitVector,
    lever_a_q16: i32,
    lever_b_q16: i32,
) -> i32 {
    let av = a.state().linear_velocity();
    let bv = b
        .map(|body| body.state().linear_velocity())
        .unwrap_or(LinearVelocity::ZERO);
    let linear_speed = axis.dot(bv - av);
    let angular_a = a
        .state()
        .angular_velocity()
        .projected_point_speed_raw(lever_a_q16);
    let angular_b = b
        .map(|body| {
            body.state()
                .angular_velocity()
                .projected_point_speed_raw(lever_b_q16)
        })
        .unwrap_or(0);
    let speed = linear_speed + angular_b as i64 - angular_a as i64;
    speed.clamp(
        -(MAX_RELATIVE_CONTACT_SPEED_RAW as i64),
        MAX_RELATIVE_CONTACT_SPEED_RAW as i64,
    ) as i32
}

#[inline(always)]
fn rotational_inverse_mass_q24(lever_q16: i32, inverse_inertia_q40: u64) -> u64 {
    // (Q16)^2 * Q40 -> Q72; shift to the inverse-mass Q24 used by k.
    let lever = lever_q16.unsigned_abs() as u128;
    let product = lever * lever * inverse_inertia_q40 as u128;
    let result = (product + (1_u128 << 47)) >> 48;
    result.min(u64::MAX as u128) as u64
}

pub(super) fn two_bodies_mut(bodies: &mut [Body], a: usize, b: usize) -> (&mut Body, &mut Body) {
    debug_assert!(a < b);
    let (left, right) = bodies.split_at_mut(b);
    (&mut left[a], &mut right[0])
}

#[inline(always)]
pub(super) fn div_round_signed(numerator: i128, denominator: u128) -> i64 {
    debug_assert!(denominator > 0);
    let magnitude = (numerator.unsigned_abs() + (denominator >> 1)) / denominator;
    let bounded = magnitude.min(i64::MAX as u128) as i64;
    if numerator < 0 { -bounded } else { bounded }
}

#[inline(always)]
pub(crate) fn round_shift_signed(value: i64, shift: u32) -> i64 {
    let rounded = (value.unsigned_abs() + (1_u64 << (shift - 1))) >> shift;
    if value < 0 {
        -(rounded as i64)
    } else {
        rounded as i64
    }
}
