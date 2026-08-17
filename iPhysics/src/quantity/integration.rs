use super::{
    Angle, AngularAcceleration, AngularVelocity, LinearAcceleration, LinearVelocity, Position,
};

/// Advances velocity and then position by one fixed 64 Hz simulation tick,
/// saturating both quantities at their gameplay limits.
#[inline]
pub fn integrate(
    position: Position,
    velocity: LinearVelocity,
    acceleration: LinearAcceleration,
) -> (Position, LinearVelocity) {
    let velocity = velocity.advance(acceleration);
    let position = position.advance(velocity);
    (position, velocity)
}

/// Advances angular velocity and then angle by one fixed 64 Hz simulation tick,
/// saturating angular velocity at its gameplay limit.
#[inline]
pub fn integrate_angular(
    angle: Angle,
    velocity: AngularVelocity,
    acceleration: AngularAcceleration,
) -> (Angle, AngularVelocity) {
    let velocity = velocity.advance(acceleration);
    let angle = angle.advance(velocity);
    (angle, velocity)
}
