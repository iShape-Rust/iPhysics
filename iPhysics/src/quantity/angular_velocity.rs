use super::angle::AngleDelta;
use super::angular_acceleration::AngularAcceleration;
use super::raw::{clamp_i64_to_i32, clamp_i128_to_i32, quantize_f64, shift_round};
use super::{ACCELERATION_TO_VELOCITY_SHIFT, KINEMATIC_FRACTION_BITS};

// At 64 Hz, Q24 rad/s converts to binary-angle units per tick by multiplying
// by 2/π. This is 2/π represented as signed Q31.
const RAD_PER_SECOND_TO_ANGLE_DELTA_Q31: i64 = 1_367_130_551;

/// Angular velocity in radians per second, stored as signed Q24.
///
/// - Resolution: `2^-24 rad/s`, approximately `0.000_000_059_6 rad/s`.
/// - Underlying Q24/i32 capacity: `-128 rad/s..128 rad/s` (exclusive upper bound).
/// - Enforced gameplay range: `-100 rad/s..100 rad/s` inclusive, additionally
///   constrained by the body's radius and maximum surface speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AngularVelocity(i32);

impl AngularVelocity {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const MAX_RAW: i32 = 100 * Self::SCALE as i32;
    pub const MIN_RAW: i32 = -Self::MAX_RAW;
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: i32) -> Self {
        Self::from_wide_saturated(raw as i128)
    }

    #[inline(always)]
    pub(crate) const fn from_wide_saturated(raw: i128) -> Self {
        Self(clamp_i128_to_i32(raw, Self::MIN_RAW, Self::MAX_RAW))
    }

    #[inline(always)]
    const fn from_i64_saturated(raw: i64) -> Self {
        Self(clamp_i64_to_i32(raw, Self::MIN_RAW, Self::MAX_RAW))
    }

    #[inline(always)]
    pub const fn checked_from_raw(raw: i32) -> Option<Self> {
        if is_valid_raw(raw) {
            Some(Self(raw))
        } else {
            None
        }
    }

    /// Converts radians per second to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_radians_per_second(value: f64) -> Option<Self> {
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
    pub const fn raw_magnitude(self) -> u32 {
        self.0.unsigned_abs()
    }

    #[inline(always)]
    pub fn to_radians_per_second(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    /// Applies an angular acceleration for one 64 Hz tick and saturates at the
    /// enforced gameplay range.
    #[inline]
    pub fn advance(self, acceleration: AngularAcceleration) -> Self {
        let delta = shift_round(acceleration.raw() as i64, ACCELERATION_TO_VELOCITY_SHIFT);
        Self::from_i64_saturated(self.0 as i64 + delta)
    }

    /// Converts this velocity into a binary angle delta for one 64 Hz tick.
    #[inline]
    pub fn angle_delta_per_tick(self) -> AngleDelta {
        let product = self.0 as i64 * RAD_PER_SECOND_TO_ANGLE_DELTA_Q31;
        AngleDelta::from_raw(shift_round(product, 31) as i32)
    }
}

#[inline(always)]
const fn is_valid_raw(value: i32) -> bool {
    value >= AngularVelocity::MIN_RAW && value <= AngularVelocity::MAX_RAW
}
