use crate::quantity::Position;
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
    pub const ZERO: Self = Self { x: 0, y: 0 };

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// Narrows a derived value whose full-i32 range is guaranteed by collider
    /// construction. Debug builds expose a broken invariant; release performs
    /// only the casts and never clamps the geometry.
    #[inline(always)]
    pub(crate) const fn from_wide_narrow(x: i64, y: i64) -> Self {
        debug_assert!(x >= i32::MIN as i64 && x <= i32::MAX as i64);
        debug_assert!(y >= i32::MIN as i64 && y <= i32::MAX as i64);
        Self::from_raw(x as i32, y as i32)
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }

    #[inline(always)]
    pub const fn raw_point(self) -> IntPoint<i32> {
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
    pub const fn midpoint(self, other: Self) -> Self {
        Self::from_raw(
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

impl From<Position> for GeometryPoint {
    #[inline(always)]
    fn from(position: Position) -> Self {
        let [x, y] = position.raw();
        Self::from_raw(x, y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_full_i32_geometry_range() {
        let point = GeometryPoint::from_wide_narrow(i32::MIN as i64, i32::MAX as i64);

        assert_eq!(point.raw(), [i32::MIN, i32::MAX]);
    }

    #[test]
    fn midpoint_widens_before_adding() {
        let a = GeometryPoint::from_raw(i32::MAX, i32::MAX);
        let b = GeometryPoint::from_raw(i32::MAX - 2, i32::MIN);

        assert_eq!(a.midpoint(b).raw(), [i32::MAX - 1, 0]);
    }
}
