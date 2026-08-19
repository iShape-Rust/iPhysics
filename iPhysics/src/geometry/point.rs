use crate::quantity::{Position, RawVec2};
use i_float::int::point::IntPoint;

/// Derived world-space Q16 point bounded to `±(2^30 - 1)` per component.
///
/// Body origins use the more restrictive [`Position`] type. Collider vertices,
/// contact points, and AABB bounds may extend beyond that center range while
/// remaining in the bounded geometry range under collider-size invariants.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct GeometryPoint {
    x: i32,
    y: i32,
}

impl GeometryPoint {
    pub(crate) const ZERO: Self = Self { x: 0, y: 0 };

    #[inline(always)]
    pub(crate) const fn from_i32_unchecked(x: i32, y: i32) -> Self {
        debug_assert!(x >= Position::MIN_POINT && x <= Position::MAX_POINT);
        debug_assert!(y >= Position::MIN_POINT && y <= Position::MAX_POINT);
        Self { x, y }
    }

    /// Narrows a derived value whose geometry range is guaranteed by collider
    /// construction. Debug builds expose a broken invariant; release performs
    /// only the casts and never clamps the geometry.
    #[inline(always)]
    pub(crate) const fn from_i64_unchecked(x: i64, y: i64) -> Self {
        debug_assert!(x >= Position::MIN_POINT as i64 && x <= Position::MAX_POINT as i64);
        debug_assert!(y >= Position::MIN_POINT as i64 && y <= Position::MAX_POINT as i64);
        Self::from_i32_unchecked(x as i32, y as i32)
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }

    #[inline(always)]
    pub(crate) const fn raw_point(self) -> IntPoint<i32> {
        IntPoint {
            x: self.x,
            y: self.y,
        }
    }

    #[inline(always)]
    pub fn to_meters(self) -> [f64; 2] {
        let scale = Position::SCALE as f64;
        [self.x as f64 / scale, self.y as f64 / scale]
    }

    #[inline(always)]
    pub(crate) const fn midpoint(self, other: Self) -> Self {
        Self::from_i32_unchecked((self.x + other.x) / 2, (self.y + other.y) / 2)
    }

    #[inline(always)]
    pub(crate) fn squared_distance(self, other: Self) -> u64 {
        (self - other).squared_magnitude()
    }
}

impl core::ops::Sub for GeometryPoint {
    type Output = RawVec2;

    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        let [x, y] = self.raw();
        let [rhs_x, rhs_y] = rhs.raw();
        let dx = x - rhs_x;
        let dy = y - rhs_y;
        RawVec2::from_i32(dx, dy)
    }
}

impl From<Position> for GeometryPoint {
    #[inline(always)]
    fn from(position: Position) -> Self {
        let [x, y] = position.raw();
        Self::from_i32_unchecked(x, y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_bounded_geometry_range() {
        let point = GeometryPoint::from_i64_unchecked(
            Position::MIN_POINT as i64,
            Position::MAX_POINT as i64,
        );

        assert_eq!(point.raw(), [Position::MIN_POINT, Position::MAX_POINT]);
        assert_eq!(
            Position::MIN_POINT.checked_add(Position::MIN_POINT),
            Some(i32::MIN + 2)
        );
        assert_eq!(
            Position::MAX_POINT.checked_add(Position::MAX_POINT),
            Some(i32::MAX - 1)
        );
    }

    #[test]
    fn midpoint_stays_inside_geometry_range() {
        let a = GeometryPoint::from_i32_unchecked(Position::MAX_POINT, Position::MAX_POINT);
        let b = GeometryPoint::from_i32_unchecked(Position::MAX_POINT - 2, Position::MIN_POINT);

        assert_eq!(a.midpoint(b).raw(), [Position::MAX_POINT - 1, 0]);
    }

    #[test]
    fn extreme_difference_fits_raw_vector() {
        let min = GeometryPoint::from_i32_unchecked(Position::MIN_POINT, Position::MIN_POINT);
        let max = GeometryPoint::from_i32_unchecked(Position::MAX_POINT, Position::MAX_POINT);
        let expected = 2 * Position::MAX_POINT;

        assert_eq!((max - min).raw(), [expected, expected]);
        assert_eq!(
            max.squared_distance(min),
            2 * expected as u64 * expected as u64
        );
    }
}
