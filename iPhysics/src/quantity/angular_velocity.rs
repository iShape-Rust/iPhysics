use crate::ops::{quantize::Quantize, shift::RoundShift};

use super::AngleDelta;
use super::angular_acceleration::AngularAcceleration;
use super::{
    ANGULAR_ACCELERATION_TO_VELOCITY_SHIFT, ANGULAR_VELOCITY_FRACTION_BITS,
    LINEAR_VELOCITY_FRACTION_BITS, POSITION_FRACTION_BITS,
};

// At 64 Hz, angular velocity converts to binary-angle units per tick by
// multiplying by 2/π. This is 2/π represented as signed Q31.
const RAD_PER_SECOND_TO_ANGLE_DELTA_Q31: i64 = 1_367_130_551;

/// Angular velocity in radians per second, stored as bounded signed Q16.
///
/// - Resolution: `2^-16 rad/s`, approximately `0.000_015_259 rad/s`.
/// - Storage range: `-128 rad/s..128 rad/s` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AngularVelocity(i32);

impl AngularVelocity {
    pub const FRACTION_BITS: u32 = ANGULAR_VELOCITY_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub(crate) const MIN_VELOCITY: i32 = -(1 << 23);
    pub(crate) const MAX_VELOCITY: i32 = (1 << 23) - 1;
    pub(crate) const MAX_CHANGE: u64 =
        (Self::MAX_VELOCITY as i64 - Self::MIN_VELOCITY as i64) as u64;
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: i32) -> Self {
        Self(Self::clamp_raw(raw))
    }

    #[inline(always)]
    const fn clamp_raw(raw: i32) -> i32 {
        if raw < Self::MIN_VELOCITY {
            Self::MIN_VELOCITY
        } else if raw > Self::MAX_VELOCITY {
            Self::MAX_VELOCITY
        } else {
            raw
        }
    }

    #[inline(always)]
    pub(crate) fn from_wide_saturated(raw: i64) -> Self {
        Self(raw.clamp(Self::MIN_VELOCITY as i64, Self::MAX_VELOCITY as i64) as i32)
    }

    /// Converts radians per second to Q16, rounding midpoint values away from zero.
    #[inline]
    pub fn from_radians_per_second(value: f64) -> Option<Self> {
        let raw = value.quantize(Self::FRACTION_BITS)?;
        if raw < Self::MIN_VELOCITY || raw > Self::MAX_VELOCITY {
            None
        } else {
            Some(Self(raw))
        }
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
        Self::from_wide_saturated(self.0 as i64 + delta)
    }

    /// Converts this velocity into a binary angle delta for one 64 Hz tick.
    #[inline]
    pub fn angle_delta_per_tick(self) -> AngleDelta {
        let product = self.0 as i64 * RAD_PER_SECOND_TO_ANGLE_DELTA_Q31;
        AngleDelta::from_raw(product.round_shift(Self::FRACTION_BITS + 7) as i32)
    }

    /// Converts `omega * (r x n)` into the Q10 linear speed of a point along
    /// an arbitrary direction. The lever projection is expressed in Q16.
    #[inline(always)]
    pub(crate) fn projected_point_speed_raw(self, lever_cross_direction_q16: i32) -> i32 {
        const SHIFT: u32 =
            ANGULAR_VELOCITY_FRACTION_BITS + POSITION_FRACTION_BITS - LINEAR_VELOCITY_FRACTION_BITS;
        let result = (self.0 as i64 * lever_cross_direction_q16 as i64).round_shift(SHIFT);
        debug_assert!(i32::try_from(result).is_ok());
        result as i32
    }
}
