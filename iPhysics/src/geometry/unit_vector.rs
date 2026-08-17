use crate::quantity::DiffVec2;

/// Dimensionless normalized direction stored as signed Q30 components.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct UnitVector {
    x: i32,
    y: i32,
}

impl UnitVector {
    pub const FRACTION_BITS: u32 = 30;
    pub const X: Self = Self { x: 1 << 30, y: 0 };
    pub const Y: Self = Self { x: 0, y: 1 << 30 };

    /// Normalizes a non-zero raw vector into a deterministic Q30 direction.
    #[inline]
    pub fn normalized(vector: DiffVec2) -> Option<Self> {
        Self::normalized_with_length(vector, vector.squared_magnitude().isqrt())
    }

    /// Normalizes using a magnitude already computed by the caller.
    /// Circle collision uses this after its exact squared-distance test.
    #[inline]
    pub(crate) fn normalized_with_length(vector: DiffVec2, length: u64) -> Option<Self> {
        if length == 0 {
            return None;
        }

        let [x, y] = vector.raw();
        let scale = 1_i64 << Self::FRACTION_BITS;
        Some(Self::from_raw(
            (x as i64 * scale / length as i64) as i32,
            (y as i64 * scale / length as i64) as i32,
        ))
    }

    /// Projects a bounded raw vector onto this direction, preserving the
    /// vector's fixed-point scale.
    #[inline(always)]
    pub const fn dot_raw(self, vector: DiffVec2) -> i64 {
        round_shift_i64(
            vector.dot(DiffVec2::from_raw_unchecked(self.x, self.y)),
            Self::FRACTION_BITS,
        )
    }

    /// Scales this direction by a bounded raw magnitude.
    #[inline(always)]
    pub const fn scaled_raw(self, magnitude: i32) -> DiffVec2 {
        DiffVec2::from_wide_clamped(
            round_shift_i64(self.x as i64 * magnitude as i64, Self::FRACTION_BITS),
            round_shift_i64(self.y as i64 * magnitude as i64, Self::FRACTION_BITS),
        )
    }

    /// Solver variant for values whose intermediate range requires i128.
    #[inline(always)]
    pub(crate) const fn scaled_wide_raw(self, magnitude: i128) -> [i128; 2] {
        [
            round_shift_i128(self.x as i128 * magnitude, Self::FRACTION_BITS),
            round_shift_i128(self.y as i128 * magnitude, Self::FRACTION_BITS),
        ]
    }

    /// Solver projection for differences that do not satisfy DiffVec2 bounds.
    #[inline(always)]
    pub(crate) const fn dot_wide_raw(self, vector: [i64; 2]) -> i64 {
        round_shift_i128(
            vector[0] as i128 * self.x as i128 + vector[1] as i128 * self.y as i128,
            Self::FRACTION_BITS,
        ) as i64
    }

    #[inline(always)]
    pub(crate) const fn from_raw(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }
}

impl core::ops::Neg for UnitVector {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

#[inline(always)]
const fn round_shift_i64(value: i64, shift: u32) -> i64 {
    let half = 1_i64 << (shift - 1);
    if value < 0 {
        -((-value + half) >> shift)
    } else {
        (value + half) >> shift
    }
}

#[inline(always)]
const fn round_shift_i128(value: i128, shift: u32) -> i128 {
    let half = 1_i128 << (shift - 1);
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
    fn normalizes_raw_vector_to_q30() {
        let direction = UnitVector::normalized(DiffVec2::from_raw(3, 4)).unwrap();

        assert_eq!(direction.raw(), [644_245_094, 858_993_459]);
        assert_eq!(direction.scaled_raw(10).raw(), [6, 8]);
        assert_eq!(direction.dot_raw(DiffVec2::from_raw(3, 4)), 5);
        assert!(UnitVector::normalized(DiffVec2::ZERO).is_none());
    }
}
