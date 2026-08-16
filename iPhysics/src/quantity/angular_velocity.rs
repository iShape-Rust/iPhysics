use super::angle::AngleDelta;
use super::angular_acceleration::AngularAcceleration;
use super::raw::{quantize_f64, shift_round};
use super::{ACCELERATION_TO_VELOCITY_SHIFT, KINEMATIC_FRACTION_BITS};

// At 64 Hz, Q24 rad/s converts to binary-angle units per tick by multiplying
// by 2/π. This is 2/π represented as signed Q31.
const RAD_PER_SECOND_TO_ANGLE_DELTA_Q31: i64 = 1_367_130_551;

/// Angular velocity in radians per second, stored as signed Q24.
///
/// - Resolution: `2^-24 rad/s`, approximately `0.000_000_059_6 rad/s`.
/// - Storage range: `-128 rad/s..128 rad/s` (exclusive upper bound).
/// - Intended gameplay limit: magnitude at most `100 rad/s`, additionally
///   constrained by the body's radius and maximum surface speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AngularVelocity(i32);

impl AngularVelocity {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: i32) -> Self {
        Self(raw)
    }

    /// Converts radians per second to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_radians_per_second(value: f64) -> Option<Self> {
        Some(Self(quantize_f64(value, Self::FRACTION_BITS)?))
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

    /// Applies an angular acceleration for one 64 Hz tick.
    #[inline]
    pub fn checked_advance(self, acceleration: AngularAcceleration) -> Option<Self> {
        let delta = shift_round(acceleration.raw() as i64, ACCELERATION_TO_VELOCITY_SHIFT);
        Some(Self(i32::try_from(self.0 as i64 + delta).ok()?))
    }

    /// Converts this velocity into a binary angle delta for one 64 Hz tick.
    #[inline]
    pub fn angle_delta_per_tick(self) -> AngleDelta {
        let product = self.0 as i64 * RAD_PER_SECOND_TO_ANGLE_DELTA_Q31;
        AngleDelta::from_raw(shift_round(product, 31) as i32)
    }
}
