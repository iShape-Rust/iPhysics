use crate::ops::quantize::Quantize;

use super::ANGULAR_KINEMATIC_FRACTION_BITS;

/// Angular acceleration in radians per second squared, stored as signed Q24.
///
/// - Resolution: `2^-24 rad/s^2`, approximately `0.000_000_059_6 rad/s^2`.
/// - Storage range: `-128 rad/s²..128 rad/s²` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AngularAcceleration(i32);

impl AngularAcceleration {
    pub const FRACTION_BITS: u32 = ANGULAR_KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: i32) -> Self {
        Self(raw)
    }

    /// Converts radians per second squared to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_radians_per_second_squared(value: f64) -> Option<Self> {
        Some(Self(value.quantize(Self::FRACTION_BITS)?))
    }

    #[inline(always)]
    pub const fn raw(self) -> i32 {
        self.0
    }

    #[inline(always)]
    pub const fn is_zero(self) -> bool {
        self.0 == 0
    }

    #[inline(always)]
    pub fn to_radians_per_second_squared(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }
}
