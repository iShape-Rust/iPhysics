use crate::fix::clamp::ClampToI32;

/// Bounded raw two-dimensional vector used by geometry.
///
/// `Position - Position` produces this type in Q16. The restricted world
/// range guarantees that the exact difference fits in `i32`, while dot and
/// cross products fit in `i64`. Other raw fixed-point vectors can use it too
/// as long as the caller keeps track of their scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct RawVec2 {
    x: i32,
    y: i32,
}

impl RawVec2 {
    pub const ZERO: Self = Self { x: 0, y: 0 };

    /// Creates a bounded raw vector. `i32::MIN` is clamped by one unit because
    /// it cannot be produced by subtracting two valid world coordinates and
    /// would make the sum of two squared components exceed `i64::MAX`.
    #[inline(always)]
    pub fn from_raw(x: i32, y: i32) -> Self {
        Self::from_wide_clamped(x as i64, y as i64)
    }

    /// Creates a difference whose bounded range has already been proved by
    /// the calling operation. No validation or saturation is performed.
    #[inline(always)]
    pub(crate) const fn from_raw_unchecked(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub(crate) fn from_wide_clamped(x: i64, y: i64) -> Self {
        Self {
            x: x.clamp_to_i32(-i32::MAX, i32::MAX),
            y: y.clamp_to_i32(-i32::MAX, i32::MAX),
        }
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }

    #[inline(always)]
    pub const fn dot(self, other: Self) -> i64 {
        self.x as i64 * other.x as i64 + self.y as i64 * other.y as i64
    }

    #[inline(always)]
    pub const fn cross(self, other: Self) -> i64 {
        self.x as i64 * other.y as i64 - self.y as i64 * other.x as i64
    }

    #[inline(always)]
    pub const fn squared_magnitude(self) -> u64 {
        self.dot(self) as u64
    }
}

impl From<[i32; 2]> for RawVec2 {
    #[inline(always)]
    fn from(value: [i32; 2]) -> Self {
        Self::from_raw(value[0], value[1])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_and_cross_are_exact() {
        let a = RawVec2::from_raw(3, 4);
        let b = RawVec2::from_raw(-2, 5);

        assert_eq!(a.dot(b), 14);
        assert_eq!(a.cross(b), 23);
        assert_eq!(a.squared_magnitude(), 25);
    }

    #[test]
    fn extreme_products_fit_i64() {
        let max = RawVec2::from_raw(i32::MAX, i32::MAX);
        let mixed = RawVec2::from_raw(-i32::MAX, i32::MAX);

        assert_eq!(max.dot(max), 2 * i32::MAX as i64 * i32::MAX as i64);
        assert_eq!(max.cross(mixed), 2 * i32::MAX as i64 * i32::MAX as i64);
        assert_eq!(
            RawVec2::from_wide_clamped(i64::MIN, i64::MAX).raw(),
            [-i32::MAX, i32::MAX]
        );
    }
}
