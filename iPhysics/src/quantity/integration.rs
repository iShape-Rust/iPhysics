use super::{
    Angle, AngularAcceleration, AngularVelocity, LinearAcceleration, LinearVelocity, Position,
};

/// Advances velocity and then position by one fixed 64 Hz simulation tick.
#[inline]
pub fn checked_integrate(
    position: Position,
    velocity: LinearVelocity,
    acceleration: LinearAcceleration,
) -> Option<(Position, LinearVelocity)> {
    let velocity = velocity.checked_advance(acceleration)?;
    let position = position.checked_advance(velocity)?;
    Some((position, velocity))
}

/// Advances angular velocity and then angle by one fixed 64 Hz simulation tick.
#[inline]
pub fn checked_integrate_angular(
    angle: Angle,
    velocity: AngularVelocity,
    acceleration: AngularAcceleration,
) -> Option<(Angle, AngularVelocity)> {
    let velocity = velocity.checked_advance(acceleration)?;
    let angle = angle.advance(velocity);
    Some((angle, velocity))
}
