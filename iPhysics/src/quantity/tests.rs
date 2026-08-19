use crate::fix::shift::RoundShift;

use super::{
    Angle, AngleDelta, AngularAcceleration, AngularVelocity, LinearAcceleration, LinearVelocity,
    Position, integrate, integrate_angular,
};

#[test]
fn expected_values_fit_the_formats() {
    let position = Position::from_meters(8_000.0, -8_000.0).unwrap();
    let velocity = LinearVelocity::from_meters_per_second(100.0, -100.0).unwrap();
    let acceleration = LinearAcceleration::from_meters_per_second_squared(100.0, -100.0).unwrap();

    assert_eq!(position.raw(), [524_288_000, -524_288_000]);
    assert_eq!(velocity.raw(), [1_677_721_600, -1_677_721_600]);
    assert_eq!(acceleration.raw(), velocity.raw());
}

#[test]
fn linear_quantities_use_full_q24_range() {
    assert_eq!(
        LinearVelocity::from_raw(i32::MIN, i32::MAX).raw(),
        [i32::MIN, i32::MAX]
    );
    assert_eq!(
        LinearAcceleration::from_raw(i32::MIN, i32::MAX).raw(),
        [i32::MIN, i32::MAX]
    );
    assert!(LinearVelocity::from_meters_per_second(-128.0, 0.0).is_some());
    assert!(LinearVelocity::from_meters_per_second(128.0, 0.0).is_none());
    assert!(LinearAcceleration::from_meters_per_second_squared(-128.0, 0.0).is_some());
    assert!(LinearAcceleration::from_meters_per_second_squared(128.0, 0.0).is_none());
    assert!(LinearVelocity::from_meters_per_second(f64::NAN, 0.0).is_none());
}

#[test]
fn integrates_constant_velocity_at_64_hz() {
    let position = Position::ZERO;
    let velocity = LinearVelocity::from_meters_per_second(1.0, -1.0).unwrap();

    let position = position.advance(velocity);

    assert_eq!(position.raw(), [1_024, -1_024]);
    assert_eq!(position.to_meters(), [0.015625, -0.015625]);
}

#[test]
fn integrates_acceleration_before_position() {
    let acceleration = LinearAcceleration::from_meters_per_second_squared(100.0, -100.0).unwrap();

    let (position, velocity) = integrate(Position::ZERO, LinearVelocity::ZERO, acceleration);

    assert_eq!(velocity.to_meters_per_second(), [1.5625, -1.5625]);
    assert_eq!(position.raw(), [1_600, -1_600]);
}

#[test]
fn rounds_small_motion_symmetrically() {
    let positive = LinearVelocity::from_meters_per_second(0.001, 0.0).unwrap();
    let negative = LinearVelocity::from_meters_per_second(-0.001, 0.0).unwrap();

    assert_eq!(Position::ZERO.advance(positive).raw()[0], 1);
    assert_eq!(Position::ZERO.advance(negative).raw()[0], -1);
    assert_eq!(8_i64.round_shift(4), 1);
    assert_eq!((-8_i64).round_shift(4), -1);
}

#[test]
fn integration_saturates_at_world_boundary() {
    let position = Position::from_i32(i32::MAX, 0);
    let velocity = LinearVelocity::from_meters_per_second(1.0, 0.0).unwrap();

    assert_eq!(position.advance(velocity).raw()[0], Position::MAX_POS);
}

#[test]
fn binary_angle_wraps_at_exactly_one_turn() {
    let angle = Angle::from_raw(u32::MAX).wrapping_add(AngleDelta::from_raw(1));

    assert_eq!(angle, Angle::ZERO);
    assert_eq!(Angle::ZERO.delta_to(Angle::QUARTER_TURN).raw(), 1 << 30);
    assert_eq!(Angle::QUARTER_TURN.delta_to(Angle::ZERO).raw(), -(1 << 30));
}

#[test]
fn angle_radian_conversion_uses_canonical_turn_values() {
    let quarter = Angle::from_radians(core::f64::consts::FRAC_PI_2).unwrap();
    let negative_quarter = Angle::from_radians(-core::f64::consts::FRAC_PI_2).unwrap();

    assert_eq!(quarter, Angle::QUARTER_TURN);
    assert_eq!(negative_quarter, Angle::THREE_QUARTER_TURN);
    assert_eq!(Angle::HALF_TURN.to_signed_radians(), -core::f64::consts::PI);
    assert!(Angle::from_radians(f64::INFINITY).is_none());
}

#[test]
fn integrates_angular_acceleration_before_angle() {
    let acceleration = AngularAcceleration::from_radians_per_second_squared(100.0).unwrap();

    let (angle, velocity) = integrate_angular(Angle::ZERO, AngularVelocity::ZERO, acceleration);

    assert_eq!(velocity.to_radians_per_second(), 1.5625);
    let expected_radians = 1.5625 / 64.0;
    let error = (angle.to_radians() - expected_radians).abs();
    assert!(error < 2.0e-9, "angle error: {error}");
}

#[test]
fn angular_velocity_conversion_is_symmetric() {
    let positive = AngularVelocity::from_radians_per_second(100.0).unwrap();
    let negative = AngularVelocity::from_radians_per_second(-100.0).unwrap();

    assert_eq!(
        positive.angle_delta_per_tick().raw(),
        -negative.angle_delta_per_tick().raw()
    );
}

#[test]
fn angular_acceleration_uses_full_q24_range() {
    assert_eq!(AngularAcceleration::from_raw(i32::MIN).raw(), i32::MIN);
    assert_eq!(AngularAcceleration::from_raw(i32::MAX).raw(), i32::MAX);
    assert!(AngularAcceleration::from_radians_per_second_squared(-128.0).is_some());
    assert!(AngularAcceleration::from_radians_per_second_squared(128.0).is_none());
}

#[test]
fn angular_velocity_uses_full_q24_range() {
    let min = AngularVelocity::from_raw(i32::MIN);
    let max = AngularVelocity::from_raw(i32::MAX);

    assert_eq!(min.raw(), i32::MIN);
    assert_eq!(max.raw(), i32::MAX);
    assert!(AngularVelocity::from_radians_per_second(-128.0).is_some());
    assert!(AngularVelocity::from_radians_per_second(128.0).is_none());
    assert_eq!(
        min.advance(AngularAcceleration::from_raw(i32::MIN)).raw(),
        i32::MIN
    );
    assert_eq!(
        max.advance(AngularAcceleration::from_raw(i32::MAX)).raw(),
        i32::MAX
    );
}

#[test]
fn velocity_integration_saturates_at_storage_limit() {
    let velocity = LinearVelocity::from_raw(i32::MAX, i32::MIN);
    let acceleration = LinearAcceleration::from_raw(i32::MAX, i32::MIN);

    assert_eq!(velocity.advance(acceleration).raw(), [i32::MAX, i32::MIN]);
    assert_eq!(
        LinearVelocity::from_raw(i32::MIN, i32::MIN).raw_sqr_magnitude(),
        1_u64 << 63
    );
}
