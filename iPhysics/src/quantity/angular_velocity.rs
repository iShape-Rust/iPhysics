use crate::ops::{clamp::ClampToI32, quantize::Quantize, shift::RoundShift};

use super::angle::AngleDelta;
use super::angular_acceleration::AngularAcceleration;
use super::{ANGULAR_ACCELERATION_TO_VELOCITY_SHIFT, ANGULAR_KINEMATIC_FRACTION_BITS};

// At 64 Hz, Q24 rad/s converts to binary-angle units per tick by multiplying
// by 2/π. This is 2/π represented as signed Q31.
const RAD_PER_SECOND_TO_ANGLE_DELTA_Q31: i64 = 1_367_130_551;

/// Angular velocity in radians per second, stored as signed Q24.
///
/// - Resolution: `2^-24 rad/s`, approximately `0.000_000_059_6 rad/s`.
/// - Storage range: `-128 rad/s..128 rad/s` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AngularVelocity(i32);

impl AngularVelocity {
    pub const FRACTION_BITS: u32 = ANGULAR_KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: i32) -> Self {
        Self(raw)
    }

    #[inline(always)]
    fn from_i64_saturated(raw: i64) -> Self {
        Self(raw.clamp_to_i32(i32::MIN, i32::MAX))
    }

    /// Converts radians per second to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_radians_per_second(value: f64) -> Option<Self> {
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
    pub const fn raw_magnitude(self) -> u32 {
        self.0.unsigned_abs()
    }

    #[inline(always)]
    pub fn to_radians_per_second(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    /// Applies an angular acceleration for one 64 Hz tick and saturates at the
    /// underlying `i32` storage range.
    #[inline]
    pub fn advance(self, acceleration: AngularAcceleration) -> Self {
        let delta = (acceleration.raw() as i64).round_shift(ANGULAR_ACCELERATION_TO_VELOCITY_SHIFT);
        Self::from_i64_saturated(self.0 as i64 + delta)
    }

    /// Converts this velocity into a binary angle delta for one 64 Hz tick.
    #[inline]
    pub fn angle_delta_per_tick(self) -> AngleDelta {
        let product = self.0 as i64 * RAD_PER_SECOND_TO_ANGLE_DELTA_Q31;
        AngleDelta::from_raw(product.round_shift(31) as i32)
    }
}
