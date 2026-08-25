use crate::body::Body;
use crate::quantity::{AngularVelocity, LinearVelocity, Position};
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
    let av = a.state().linear_velocity();
    let bv = b
        .map(|body| body.state().linear_velocity())
        .unwrap_or(LinearVelocity::ZERO);
    let linear_speed = axis.dot(bv - av);
    let angular_a = angular_contact_speed_raw(a, point, axis);
    let angular_b = b
        .map(|body| angular_contact_speed_raw(body, point, axis))
        .unwrap_or(0);
    let speed = linear_speed + angular_b as i64 - angular_a as i64;
    speed.clamp(
        -(MAX_RELATIVE_CONTACT_SPEED_RAW as i64),
        MAX_RELATIVE_CONTACT_SPEED_RAW as i64,
    ) as i32
}

#[inline(always)]
pub(super) fn contact_lever_cross_axis(body: &Body, point: GeometryPoint, axis: UnitVector) -> i32 {
    let center = GeometryPoint::from(body.state().transform().position);
    let lever = point - center;
    let cross = -axis.cross(lever);
    cross as i32
}

#[inline(always)]
pub(super) fn angular_contact_speed_raw(
    body: &Body,
    point: GeometryPoint,
    axis: UnitVector,
) -> i32 {
    body.state()
        .angular_velocity()
        .projected_point_speed_raw(contact_lever_cross_axis(body, point, axis))
}

#[inline(always)]
pub(super) fn point_speed_along(body: &Body, point: GeometryPoint, axis: UnitVector) -> i64 {
    axis.dot(body.state().linear_velocity().raw().into())
        + angular_contact_speed_raw(body, point, axis) as i64
}

#[inline(always)]
fn rotational_inverse_mass_q24(lever_q16: i32, inverse_inertia_q40: u64) -> u64 {
    // (Q16)^2 * Q40 -> Q72; shift to the inverse-mass Q24 used by k.
    let lever = lever_q16.unsigned_abs() as u128;
    let product = lever * lever * inverse_inertia_q40 as u128;
    let result = (product + (1_u128 << 47)) >> 48;
    result.min(u64::MAX as u128) as u64
}

pub(super) fn add_velocity(body: &mut Body, dx: i64, dy: i64) {
    let [x, y] = body.state().linear_velocity().raw();
    body.state_mut().linear_velocity =
        LinearVelocity::from_wide_saturated(x as i64 + dx, y as i64 + dy);
}

pub(super) fn add_angular_velocity(body: &mut Body, delta: i64) {
    let raw = (body.state().angular_velocity().raw() as i64).saturating_add(delta);
    let raw = raw.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
    body.state_mut().angular_velocity = AngularVelocity::from_raw(raw);
}

pub(super) fn add_position(body: &mut Body, dx: i64, dy: i64) {
    let [x, y] = body.state().transform().position.raw();
    body.state_mut().transform.position = Position::from_i64(x as i64 + dx, y as i64 + dy);
}

pub(super) fn two_bodies_mut(bodies: &mut [Body], a: usize, b: usize) -> (&mut Body, &mut Body) {
    debug_assert!(a < b);
    let (left, right) = bodies.split_at_mut(b);
    (&mut left[a], &mut right[0])
}

pub(super) fn apply_body_impulse(
    body: &mut Body,
    axis: UnitVector,
    impulse_q10: i64,
    lever_q16: i32,
) {
    if impulse_q10 == 0 {
        return;
    }

    let inverse_mass = body.inverse_mass_q24() as u64;
    // Force is Q16/u32, so its per-tick Q10 impulse is at most 2^20 and the
    // signed linear product fits i64.
    let linear_change = round_shift_signed(impulse_q10 * inverse_mass as i64, 24);
    let [change_x, change_y] = axis.scaled_wide_raw(linear_change.unsigned_abs());
    if linear_change < 0 {
        add_velocity(body, -change_x, -change_y);
    } else {
        add_velocity(body, change_x, change_y);
    }

    let negative_angular = (impulse_q10 < 0) ^ (lever_q16 < 0);
    let angular_product = impulse_q10.unsigned_abs() as u128
        * body.inverse_inertia_q40() as u128
        * lever_q16.unsigned_abs() as u128;
    let angular_magnitude = (angular_product + (1_u128 << 41)) >> 42;
    let angular_magnitude = angular_magnitude.min(i64::MAX as u128) as i64;
    let angular_change = if negative_angular {
        -angular_magnitude
    } else {
        angular_magnitude
    };
    add_angular_velocity(body, angular_change);
}

#[inline(always)]
pub(super) fn div_round_signed(numerator: i128, denominator: u128) -> i64 {
    debug_assert!(denominator > 0);
    let magnitude = (numerator.unsigned_abs() + (denominator >> 1)) / denominator;
    let bounded = magnitude.min(i64::MAX as u128) as i64;
    if numerator < 0 { -bounded } else { bounded }
}

#[inline(always)]
pub(super) fn round_shift_signed(value: i64, shift: u32) -> i64 {
    let rounded = (value.unsigned_abs() + (1_u64 << (shift - 1))) >> shift;
    if value < 0 {
        -(rounded as i64)
    } else {
        rounded as i64
    }
}
