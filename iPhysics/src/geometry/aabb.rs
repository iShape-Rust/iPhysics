use super::GeometryPoint;
use i_float::int::rect::IntRect;

/// Axis-aligned world-space boundary stored as raw Q16 coordinates.
///
/// This is a zero-cost physical-units wrapper around [`IntRect<i32>`]. Borders
/// are included in intersection tests, so exactly touching shapes remain
/// collision candidates for the narrow phase. Bounds use the full `i32` range
/// and may extend beyond the more restrictive center range of
/// [`crate::quantity::Position`].
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Aabb(IntRect<i32>);

impl Aabb {
    /// Creates a boundary from its minimum and maximum world positions.
    #[cfg(test)]
    #[inline]
    const fn from_min_max(min: GeometryPoint, max: GeometryPoint) -> Option<Self> {
        let min = min.raw_point();
        let max = max.raw_point();

        if min.x <= max.x && min.y <= max.y {
            Some(Self(IntRect {
                min_x: min.x,
                max_x: max.x,
                min_y: min.y,
                max_y: max.y,
            }))
        } else {
            None
        }
    }

    /// Creates the smallest boundary containing both positions.
    #[inline(always)]
    pub(crate) fn from_points(a: GeometryPoint, b: GeometryPoint) -> Self {
        Self(IntRect::with_ab(a.raw_point(), b.raw_point()))
    }

    /// Creates a boundary whose ordering and full-i32 range have already been
    /// proved by the caller.
    #[inline(always)]
    pub(crate) const fn from_raw_unchecked(min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Self {
        Self(IntRect {
            min_x,
            max_x,
            min_y,
            max_y,
        })
    }

    #[inline(always)]
    pub const fn min(self) -> GeometryPoint {
        GeometryPoint::from_i32_unchecked(self.0.min_x, self.0.min_y)
    }

    #[inline(always)]
    pub const fn max(self) -> GeometryPoint {
        GeometryPoint::from_i32_unchecked(self.0.max_x, self.0.max_y)
    }

    /// Tests overlap including shared borders.
    #[inline(always)]
    pub(crate) fn intersects(self, other: Self) -> bool {
        self.0.is_intersect_border_include(&other.0)
    }

    #[inline(always)]
    pub(crate) fn union(self, other: Self) -> Self {
        Self(IntRect::with_rects(&self.0, &other.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uses_int_rect_without_storage_overhead() {
        assert_eq!(size_of::<Aabb>(), size_of::<IntRect<i32>>());
    }

    #[test]
    fn touching_boundaries_intersect() {
        let a = Aabb::from_min_max(
            GeometryPoint::from_i32_unchecked(0, 0),
            GeometryPoint::from_i32_unchecked(10, 10),
        )
        .unwrap();
        let b = Aabb::from_min_max(
            GeometryPoint::from_i32_unchecked(10, 4),
            GeometryPoint::from_i32_unchecked(20, 6),
        )
        .unwrap();

        assert!(a.intersects(b));
    }

    #[test]
    fn validates_min_and_max() {
        assert!(
            Aabb::from_min_max(GeometryPoint::from_i32_unchecked(1, 0), GeometryPoint::from_i32_unchecked(0, 1))
                .is_none()
        );
    }

    #[test]
    fn union_contains_both_boundaries() {
        let a = Aabb::from_points(
            GeometryPoint::from_i32_unchecked(-5, 3),
            GeometryPoint::from_i32_unchecked(4, 8),
        );
        let b = Aabb::from_points(
            GeometryPoint::from_i32_unchecked(2, -7),
            GeometryPoint::from_i32_unchecked(9, 5),
        );
        let union = a.union(b);

        assert_eq!(union.min(), GeometryPoint::from_i32_unchecked(-5, -7));
        assert_eq!(union.max(), GeometryPoint::from_i32_unchecked(9, 8));
    }
}
