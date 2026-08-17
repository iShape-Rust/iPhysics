use super::KINEMATIC_FRACTION_BITS;
use super::raw::{clamp_i128_to_i32, quantize_f64};

/// Angular acceleration in radians per second squared, stored as signed Q24.
///
/// - Resolution: `2^-24 rad/s^2`, approximately `0.000_000_059_6 rad/s^2`.
/// - Underlying Q24/i32 capacity: `-128 rad/s²..128 rad/s²` (exclusive upper bound).
/// - Enforced gameplay range: `-100 rad/s²..100 rad/s²` inclusive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AngularAcceleration(i32);

impl AngularAcceleration {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const MAX_RAW: i32 = 100 * Self::SCALE as i32;
    pub const MIN_RAW: i32 = -Self::MAX_RAW;
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: i32) -> Self {
        Self(clamp_i128_to_i32(raw as i128, Self::MIN_RAW, Self::MAX_RAW))
    }

    #[inline(always)]
    pub const fn checked_from_raw(raw: i32) -> Option<Self> {
        if is_valid_raw(raw) {
            Some(Self(raw))
        } else {
            None
        }
    }

    /// Converts radians per second squared to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_radians_per_second_squared(value: f64) -> Option<Self> {
        let raw = quantize_f64(value, Self::FRACTION_BITS)?;
        Self::checked_from_raw(raw)
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

#[inline(always)]
const fn is_valid_raw(value: i32) -> bool {
    value >= AngularAcceleration::MIN_RAW && value <= AngularAcceleration::MAX_RAW
}
