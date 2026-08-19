use crate::{GeometryPoint, UnitVector};

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

    #[inline(always)]
    pub(crate) fn from_i32(x: i32, y: i32) -> Self {
        Self { x, y}
    }
    #[inline(always)]
    pub(crate) const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }

    #[inline(always)]
    pub(crate) const fn dot(self, other: Self) -> i64 {
        self.x as i64 * other.x as i64 + self.y as i64 * other.y as i64
    }

    #[inline(always)]
    pub(crate) const fn cross(self, other: Self) -> i64 {
        self.x as i64 * other.y as i64 - self.y as i64 * other.x as i64
    }

    #[inline(always)]
    pub(crate) const fn squared_magnitude(self) -> u64 {
        self.dot(self) as u64
    }

    #[inline(always)]
    pub(crate) const fn sqr_length(self) -> u64 {
        let x = self.x as u64;
        let y = self.y as u64;
        x * x + y * y
    }

    #[inline]
    pub(crate) fn normalized(self) -> Option<UnitVector> {
        let sqr_length = self.sqr_length();
        if sqr_length == 0 {
            return None;
        }
        let length = sqr_length.isqrt() as i64;
        let scale = 1_i64 << UnitVector::FRACTION_BITS;

        Some(UnitVector::from_raw(
            (self.x as i64 * scale / length) as i32,
            (self.y as i64 * scale / length) as i32
        ))
    }
}

impl From<[i32; 2]> for RawVec2 {
    #[inline(always)]
    fn from(value: [i32; 2]) -> Self {
        Self::from_i32(value[0], value[1])
    }
}

impl From<GeometryPoint> for RawVec2 {
    #[inline(always)]
    fn from(value: GeometryPoint) -> Self {
        Self::from(value.raw())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_and_cross_are_exact() {
        let a = RawVec2::from_i32(3, 4);
        let b = RawVec2::from_i32(-2, 5);

        assert_eq!(a.dot(b), 14);
        assert_eq!(a.cross(b), 23);
        assert_eq!(a.squared_magnitude(), 25);
    }

    #[test]
    fn extreme_products_fit_i64() {
        let max = RawVec2::from_i32(i32::MAX, i32::MAX);
        let mixed = RawVec2::from_i32(-i32::MAX, i32::MAX);

        assert_eq!(max.dot(max), 2 * i32::MAX as i64 * i32::MAX as i64);
        assert_eq!(max.cross(mixed), 2 * i32::MAX as i64 * i32::MAX as i64);
    }
}
