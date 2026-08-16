use crate::quantity::{Angle, AngularVelocity, LinearVelocity, Position};

/// Position and orientation of a body in world space.
///
/// Rotation coefficients are deliberately not cached here: `Transform` stays
/// compact and contains only authoritative rollback state. A deterministic
/// rotator will be derived when transformed collider geometry is introduced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Transform {
    pub position: Position,
    pub angle: Angle,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        position: Position::ZERO,
        angle: Angle::ZERO,
    };

    #[inline(always)]
    pub const fn new(position: Position, angle: Angle) -> Self {
        Self { position, angle }
    }

    /// Advances the transform by one fixed 64 Hz tick.
    #[inline]
    pub fn checked_advance(
        self,
        linear_velocity: LinearVelocity,
        angular_velocity: AngularVelocity,
    ) -> Option<Self> {
        Some(Self {
            position: self.position.checked_advance(linear_velocity)?,
            angle: self.angle.advance(angular_velocity),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn advances_both_position_and_angle() {
        let transform = Transform::IDENTITY;
        let linear = LinearVelocity::from_meters_per_second(64.0, -32.0).unwrap();
        let angular = AngularVelocity::from_radians_per_second(core::f64::consts::PI).unwrap();
        let next = transform.checked_advance(linear, angular).unwrap();

        assert_eq!(next.position.to_meters(), [1.0, -0.5]);
        assert_ne!(next.angle, Angle::ZERO);
    }
}
