use crate::quantity::{Position, RawVec2};
use i_float::int::point::IntPoint;

/// Derived world-space Q16 point using the full signed `i32` range.
///
/// Body origins use the more restrictive [`Position`] type. Collider vertices,
/// contact points, and AABB bounds may extend beyond that center range while
/// still fitting in full `i32` under collider-size invariants.
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

    /// Narrows a derived value whose full-i32 range is guaranteed by collider
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
        Self::from_i32_unchecked(
            ((self.x as i64 + other.x as i64) / 2) as i32,
            ((self.y as i64 + other.y as i64) / 2) as i32,
        )
    }

    #[inline(always)]
    pub(crate) const fn delta(self, other: Self) -> [i64; 2] {
        [
            self.x as i64 - other.x as i64,
            self.y as i64 - other.y as i64,
        ]
    }

    #[inline(always)]
    pub(crate) const fn squared_distance(self, other: Self) -> u128 {
        let [x, y] = self.delta(other);
        let x = x as i128;
        let y = y as i128;
        (x * x + y * y) as u128
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
}
