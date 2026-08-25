use crate::geometry::Aabb;
use crate::quantity::{Length, Position};
use crate::transform::Transform;

use super::inertia::from_q24_per_q32_ratio;

/// Circle with a body-local center.
///
/// Radius is limited to `Position::MAX_POS` in Q16 so two radii, their square,
/// and all circle narrow-phase intermediates fit in 64 bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Circle {
    center: Position,
    radius: Length,
}

impl Circle {
    #[inline]
    pub const fn new(radius: Length) -> Option<Self> {
        Self::with_center(Position::ZERO, radius)
    }

    /// Builds a circle whose center is offset from the body origin.
    #[inline]
    pub const fn with_center(center: Position, radius: Length) -> Option<Self> {
        if radius.raw() == 0 || radius.raw() > Position::MAX_POSITION as u32 {
            return None;
        }

        let remaining_radius = Position::MAX_POSITION as u64 - radius.raw() as u64;
        let [x, y] = center.raw();
        let x = x.unsigned_abs() as u64;
        let y = y.unsigned_abs() as u64;
        if x * x + y * y > remaining_radius * remaining_radius {
            None
        } else {
            Some(Self { center, radius })
        }
    }

    #[inline(always)]
    pub const fn center(self) -> Position {
        self.center
    }

    #[inline(always)]
    pub const fn radius(self) -> Length {
        self.radius
    }

    /// Reciprocal moment of inertia about the circle center as unsigned Q40.
    #[inline(always)]
    pub(crate) fn inverse_inertia_q40(self, inverse_mass_q24: u32) -> u64 {
        // I / m = r^2 / 2 + d^2, including the parallel-axis term for an
        // offset local center.
        let radius = self.radius.raw() as u128;
        let center_distance_squared = self.center.squared_distance(Position::ZERO) as u128;
        from_q24_per_q32_ratio(
            2 * inverse_mass_q24 as u128,
            radius * radius + 2 * center_distance_squared,
        )
    }

    /// Returns proportional `(mass_weight, mass_weight * I/m)` values.
    pub(super) fn mass_properties(self) -> (u64, u128) {
        // π in unsigned Q32, rounded to the nearest representable value.
        const PI_Q32: u128 = 13_493_037_705;

        let radius = self.radius.raw() as u128;
        let radius_squared = radius * radius;
        let center_squared = self.center.squared_distance(Position::ZERO) as u128;
        let twice_area = ((2 * PI_Q32 * radius_squared + (1_u128 << 31)) >> 32) as u64;
        let twice_inertia_per_mass = radius_squared + 2 * center_squared;
        let mass_weight = twice_area.saturating_mul(6);
        let weighted_inertia = 3 * twice_area as u128 * twice_inertia_per_mass;
        (mass_weight, weighted_inertia)
    }

    #[inline]
    pub(crate) fn aabb(self, transform: Transform) -> Aabb {
        let center = transform.apply_geometry(self.center);
        let [x, y] = center.raw();
        // Construction bounds the complete local circle to the collider
        // radius, so its translated extrema fit in GeometryPoint.
        let r = self.radius.raw() as i32;
        Aabb::from_raw_unchecked(x - r, x + r, y - r, y + r)
    }

    #[inline(always)]
    pub(crate) fn transformed_center(self, transform: Transform) -> crate::geometry::GeometryPoint {
        transform.apply_geometry(self.center)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Mass;

    #[test]
    fn radius_respects_world_limit() {
        assert!(Circle::new(Length::ZERO).is_none());
        assert!(Circle::new(Length::from_raw(Position::MAX_POSITION as u32)).is_some());
        assert!(Circle::new(Length::from_raw(Position::MAX_POSITION as u32 + 1)).is_none());
    }

    #[test]
    fn offset_circle_respects_combined_radius_limit() {
        let max = Position::MAX_POSITION;
        assert!(
            Circle::with_center(Position::from_i32(max - 10, 0), Length::from_raw(10)).is_some()
        );
        assert!(
            Circle::with_center(Position::from_i32(max - 10, 0), Length::from_raw(11)).is_none()
        );
    }

    #[test]
    fn aabb_can_extend_beyond_position_range() {
        let circle = Circle::new(Length::from_raw(Position::MAX_POSITION as u32)).unwrap();
        let center = Position::from_i32(Position::MAX_POSITION, Position::MAX_POSITION);
        let aabb = circle.aabb(Transform::new(center, crate::quantity::Angle::ZERO));

        assert_eq!(aabb.max().raw()[0], 2 * Position::MAX_POSITION);
        assert!(aabb.max().raw()[0] > Position::MAX_POSITION);
    }

    #[test]
    fn inverse_inertia_uses_radius_and_mass() {
        let circle = Circle::new(Length::from_meters(1.0).unwrap()).unwrap();

        assert_eq!(
            circle.inverse_inertia_q40(Mass::ONE.inverse_q24()),
            2_u64 << 40
        );
    }

    #[test]
    fn offset_center_adds_parallel_axis_inertia() {
        let circle = Circle::with_center(
            Position::from_meters(1.0, 0.0).unwrap(),
            Length::from_meters(1.0).unwrap(),
        )
        .unwrap();

        // I / m = 1/2 + 1 = 3/2, therefore inverse inertia is 2/3.
        let actual = circle.inverse_inertia_q40(Mass::ONE.inverse_q24());
        let expected = ((2_u128 << 40) / 3) as u64;
        assert!(actual.abs_diff(expected) <= 1);
    }
}
