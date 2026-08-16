use crate::quantity::Position;
use i_float::int::rect::IntRect;

/// Axis-aligned world-space boundary stored as raw Q16 coordinates.
///
/// This is a zero-cost physical-units wrapper around [`IntRect<i32>`]. Borders
/// are included in intersection tests, so exactly touching shapes remain
/// collision candidates for the narrow phase.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Aabb(IntRect<i32>);

impl Aabb {
    /// Creates a boundary from its minimum and maximum world positions.
    #[inline]
    pub const fn from_min_max(min: Position, max: Position) -> Option<Self> {
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
    pub fn from_points(a: Position, b: Position) -> Self {
        Self(IntRect::with_ab(a.raw_point(), b.raw_point()))
    }

    #[inline(always)]
    pub const fn min(self) -> Position {
        Position::from_raw(self.0.min_x, self.0.min_y)
    }

    #[inline(always)]
    pub const fn max(self) -> Position {
        Position::from_raw(self.0.max_x, self.0.max_y)
    }

    /// Tests overlap including shared borders.
    #[inline(always)]
    pub fn intersects(self, other: Self) -> bool {
        self.0.is_intersect_border_include(&other.0)
    }

    #[inline(always)]
    pub fn contains(self, position: Position) -> bool {
        self.0.contains(position.raw_point())
    }

    #[inline(always)]
    pub fn union(self, other: Self) -> Self {
        Self(IntRect::with_rects(&self.0, &other.0))
    }

    /// Exposes the underlying rectangle for interoperability with `i_float`.
    #[inline(always)]
    pub const fn as_int_rect(&self) -> &IntRect<i32> {
        &self.0
    }

    #[inline(always)]
    pub const fn into_int_rect(self) -> IntRect<i32> {
        self.0
    }
}

impl TryFrom<IntRect<i32>> for Aabb {
    type Error = ();

    #[inline(always)]
    fn try_from(rect: IntRect<i32>) -> Result<Self, Self::Error> {
        if rect.min_x <= rect.max_x && rect.min_y <= rect.max_y {
            Ok(Self(rect))
        } else {
            Err(())
        }
    }
}

impl From<Aabb> for IntRect<i32> {
    #[inline(always)]
    fn from(aabb: Aabb) -> Self {
        aabb.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uses_int_rect_without_storage_overhead() {
        assert_eq!(
            core::mem::size_of::<Aabb>(),
            core::mem::size_of::<IntRect<i32>>()
        );
    }

    #[test]
    fn touching_boundaries_intersect() {
        let a = Aabb::from_min_max(Position::from_raw(0, 0), Position::from_raw(10, 10)).unwrap();
        let b = Aabb::from_min_max(Position::from_raw(10, 4), Position::from_raw(20, 6)).unwrap();

        assert!(a.intersects(b));
    }

    #[test]
    fn validates_min_and_max() {
        assert!(Aabb::from_min_max(Position::from_raw(1, 0), Position::from_raw(0, 1)).is_none());
    }

    #[test]
    fn union_contains_both_boundaries() {
        let a = Aabb::from_points(Position::from_raw(-5, 3), Position::from_raw(4, 8));
        let b = Aabb::from_points(Position::from_raw(2, -7), Position::from_raw(9, 5));
        let union = a.union(b);

        assert_eq!(union.min(), Position::from_raw(-5, -7));
        assert_eq!(union.max(), Position::from_raw(9, 8));
    }
}
