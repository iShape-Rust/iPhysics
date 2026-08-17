use crate::geometry::GeometryPoint;
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

    /// Transforms a Q16 point into the bounded world-position domain.
    ///
    /// Rotation uses integer CORDIC coefficients, so the result is identical
    /// on every target and does not depend on a platform floating-point
    /// implementation. Translation saturates at the world boundary because an
    /// arbitrary local point has no collider-radius invariant.
    #[inline]
    pub fn apply(self, local: Position) -> Position {
        let [rx, ry] = rotate_fixed(local.raw(), self.angle);
        let [tx, ty] = self.position.raw();
        Position::from_wide_saturated(tx as i128 + rx as i128, ty as i128 + ry as i128)
    }

    /// Transforms a collider-local point into the full-i32 geometry domain.
    /// Collider construction guarantees the local point's radius, so the
    /// rotated point plus any valid body position fits without saturation.
    #[inline]
    pub(crate) fn apply_geometry(self, local: Position) -> GeometryPoint {
        let [rx, ry] = rotate_fixed(local.raw(), self.angle);
        let [tx, ty] = self.position.raw();
        GeometryPoint::from_wide_narrow(tx as i64 + rx as i64, ty as i64 + ry as i64)
    }

    /// Composes a child-local transform with this parent transform.
    #[inline]
    pub fn compose(self, local: Self) -> Self {
        Self {
            position: self.apply(local.position),
            angle: Angle::from_raw(self.angle.raw().wrapping_add(local.angle.raw())),
        }
    }

    /// Advances the transform by one fixed 64 Hz tick.
    #[inline]
    pub fn advance(
        self,
        linear_velocity: LinearVelocity,
        angular_velocity: AngularVelocity,
    ) -> Self {
        Self {
            position: self.position.advance(linear_velocity),
            angle: self.angle.advance(angular_velocity),
        }
    }
}

#[inline]
pub(crate) fn rotate_fixed(point: [i32; 2], angle: Angle) -> [i32; 2] {
    let [sin, cos] = angle.sin_cos_q30();
    let x = round_shift(
        point[0] as i64 * cos as i64 - point[1] as i64 * sin as i64,
        30,
    );
    let y = round_shift(
        point[0] as i64 * sin as i64 + point[1] as i64 * cos as i64,
        30,
    );
    debug_assert!(x >= i32::MIN as i64 && x <= i32::MAX as i64);
    debug_assert!(y >= i32::MIN as i64 && y <= i32::MAX as i64);
    [x as i32, y as i32]
}

#[inline(always)]
fn round_shift(value: i64, shift: u32) -> i64 {
    let half = 1_i64 << (shift - 1);
    if value < 0 {
        -((-value + half) >> shift)
    } else {
        (value + half) >> shift
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
        let next = transform.advance(linear, angular);

        assert_eq!(next.position.to_meters(), [1.0, -0.5]);
        assert_ne!(next.angle, Angle::ZERO);
    }

    #[test]
    fn applies_cardinal_rotation_exactly() {
        let transform = Transform::new(Position::from_raw(100, 200), Angle::QUARTER_TURN);

        assert_eq!(
            transform.apply(Position::from_raw(30, 10)),
            Position::from_raw(90, 230)
        );
    }

    #[test]
    fn arbitrary_point_transformation_saturates_at_world_boundary() {
        let transform = Transform::new(
            Position::from_raw(Position::MAX_RAW, Position::MAX_RAW),
            Angle::ZERO,
        );

        assert_eq!(
            transform.apply(Position::from_raw(1, 1)),
            Position::from_raw(Position::MAX_RAW, Position::MAX_RAW)
        );
    }
}
