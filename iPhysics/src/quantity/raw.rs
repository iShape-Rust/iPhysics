use crate::fix::shift::RoundShift;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub(super) struct RawVec2 {
    x: i32,
    y: i32,
}

impl RawVec2 {
    pub(super) const ZERO: Self = Self { x: 0, y: 0 };

    #[inline(always)]
    pub(super) const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// Saturates a wide pair to a quantity-specific inclusive raw range.
    #[inline(always)]
    pub(super) const fn from_wide_saturated(x: i128, y: i128, min: i32, max: i32) -> Self {
        debug_assert!(min <= max);
        Self {
            x: clamp_i128_to_i32(x, min, max),
            y: clamp_i128_to_i32(y, min, max),
        }
    }

    /// Saturates an i64 pair without widening the common integration path.
    #[inline(always)]
    pub(super) const fn from_i64_saturated(x: i64, y: i64, min: i32, max: i32) -> Self {
        debug_assert!(min <= max);
        Self {
            x: clamp_i64_to_i32(x, min, max),
            y: clamp_i64_to_i32(y, min, max),
        }
    }

    #[inline(always)]
    pub(super) const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }

    #[inline]
    pub(super) fn from_f64(x: f64, y: f64, fraction_bits: u32) -> Option<Self> {
        Some(Self {
            x: quantize_f64(x, fraction_bits)?,
            y: quantize_f64(y, fraction_bits)?,
        })
    }

    #[inline(always)]
    pub(super) fn to_f64(self, fraction_bits: u32) -> [f64; 2] {
        let scale = (1_u64 << fraction_bits) as f64;
        [self.x as f64 / scale, self.y as f64 / scale]
    }

    #[inline]
    pub(super) fn add_shifted_saturated(self, rhs: Self, shift: u32, min: i32, max: i32) -> Self {
        let x = self.x as i64 + (rhs.x as i64).round_shift(shift);
        let y = self.y as i64 + (rhs.y as i64).round_shift(shift);
        Self::from_i64_saturated(x, y, min, max)
    }
}

/// Bounded raw two-dimensional difference used by geometry.
///
/// `Position - Position` produces this type in Q16. The restricted world
/// range guarantees that the exact difference fits in `i32`, while dot and
/// cross products fit in `i64`. Other raw fixed-point vectors can use it too
/// as long as the caller keeps track of their scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct DiffVec2 {
    x: i32,
    y: i32,
}

impl DiffVec2 {
    pub const ZERO: Self = Self { x: 0, y: 0 };

    /// Creates a bounded raw vector. `i32::MIN` is clamped by one unit because
    /// it cannot be produced by subtracting two valid world coordinates and
    /// would make the sum of two squared components exceed `i64::MAX`.
    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self::from_wide_clamped(x as i64, y as i64)
    }

    /// Creates a difference whose bounded range has already been proved by
    /// the calling operation. No validation or saturation is performed.
    #[inline(always)]
    pub(crate) const fn from_raw_unchecked(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub(crate) const fn from_wide_clamped(x: i64, y: i64) -> Self {
        Self {
            x: clamp_diff_component(x),
            y: clamp_diff_component(y),
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

#[inline(always)]
const fn clamp_diff_component(value: i64) -> i32 {
    if value < -(i32::MAX as i64) {
        -i32::MAX
    } else if value > i32::MAX as i64 {
        i32::MAX
    } else {
        value as i32
    }
}

#[inline(always)]
pub(super) const fn clamp_i64_to_i32(value: i64, min: i32, max: i32) -> i32 {
    if value < min as i64 {
        min
    } else if value > max as i64 {
        max
    } else {
        value as i32
    }
}

#[inline(always)]
pub(super) const fn clamp_i128_to_i32(value: i128, min: i32, max: i32) -> i32 {
    if value < min as i128 {
        min
    } else if value > max as i128 {
        max
    } else {
        value as i32
    }
}

impl From<[i32; 2]> for DiffVec2 {
    #[inline(always)]
    fn from(value: [i32; 2]) -> Self {
        Self::from_raw(value[0], value[1])
    }
}

#[inline]
pub(super) fn quantize_f64(value: f64, fraction_bits: u32) -> Option<i32> {
    if !value.is_finite() {
        return None;
    }

    let scale = (1_u64 << fraction_bits) as f64;
    let scaled = value * scale;
    if scaled < i32::MIN as f64 || scaled > i32::MAX as f64 {
        return None;
    }

    let truncated = scaled as i32;
    let fraction = scaled - truncated as f64;
    if fraction >= 0.5 {
        truncated.checked_add(1)
    } else if fraction <= -0.5 {
        truncated.checked_sub(1)
    } else {
        Some(truncated)
    }
}

#[inline]
pub(super) fn quantize_u32_f64(value: f64, fraction_bits: u32) -> Option<u32> {
    if !value.is_finite() || value < 0.0 {
        return None;
    }

    let scale = (1_u64 << fraction_bits) as f64;
    let scaled = value * scale;
    if scaled > u32::MAX as f64 {
        return None;
    }

    let truncated = scaled as u32;
    if scaled - truncated as f64 >= 0.5 {
        truncated.checked_add(1)
    } else {
        Some(truncated)
    }
}

#[cfg(test)]
mod diff_tests {
    use super::*;

    #[test]
    fn dot_and_cross_are_exact() {
        let a = DiffVec2::from_raw(3, 4);
        let b = DiffVec2::from_raw(-2, 5);

        assert_eq!(a.dot(b), 14);
        assert_eq!(a.cross(b), 23);
        assert_eq!(a.squared_magnitude(), 25);
    }

    #[test]
    fn extreme_products_fit_i64() {
        let max = DiffVec2::from_raw(i32::MAX, i32::MAX);
        let mixed = DiffVec2::from_raw(-i32::MAX, i32::MAX);

        assert_eq!(max.dot(max), 2 * i32::MAX as i64 * i32::MAX as i64);
        assert_eq!(max.cross(mixed), 2 * i32::MAX as i64 * i32::MAX as i64);
        assert_eq!(
            DiffVec2::from_wide_clamped(i64::MIN, i64::MAX).raw(),
            [-i32::MAX, i32::MAX]
        );
    }
}
