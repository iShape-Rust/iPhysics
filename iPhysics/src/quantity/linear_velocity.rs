use crate::fix::{clamp::ClampToI32, quantize::Quantize, shift::RoundShift};

use super::linear_acceleration::LinearAcceleration;
use super::{ACCELERATION_TO_VELOCITY_SHIFT, KINEMATIC_FRACTION_BITS};

/// Linear velocity in metres per second, stored as signed Q24 components.
///
/// - Resolution: `2^-24 m/s`, approximately `0.000_000_059_6 m/s`.
/// - Storage range per component: `-128 m/s..128 m/s` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearVelocity([i32; 2]);

impl LinearVelocity {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self([0, 0]);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self([x, y])
    }

    #[inline(always)]
    pub(crate) fn from_wide_saturated(x: i128, y: i128) -> Self {
        Self([
            x.clamp_to_i32(i32::MIN, i32::MAX),
            y.clamp_to_i32(i32::MIN, i32::MAX),
        ])
    }

    /// Converts metres per second to Q24, rounding midpoint values away from zero.
    #[inline(always)]
    pub fn from_meters_per_second(x: f64, y: f64) -> Option<Self> {
        Some(Self([
            x.quantize(Self::FRACTION_BITS)?,
            y.quantize(Self::FRACTION_BITS)?,
        ]))
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        self.0
    }

    #[inline(always)]
    pub const fn is_zero(self) -> bool {
        let [x, y] = self.raw();
        x == 0 && y == 0
    }

    /// Squared raw Q24 magnitude. The result has 48 fractional bits.
    #[inline(always)]
    pub const fn raw_sqr_magnitude(self) -> u64 {
        let [x, y] = self.raw();
        let x = x as i64;
        let y = y as i64;
        (x * x) as u64 + (y * y) as u64
    }

    #[inline(always)]
    pub fn to_meters_per_second(self) -> [f64; 2] {
        let scale = Self::SCALE as f64;
        [self.0[0] as f64 / scale, self.0[1] as f64 / scale]
    }

    /// Applies an acceleration for one 64 Hz tick and saturates each component
    /// at the underlying `i32` storage range.
    #[inline(always)]
    pub fn advance(self, acceleration: LinearAcceleration) -> Self {
        let [x, y] = self.raw();
        let [ax, ay] = acceleration.raw();
        Self([
            (x as i64 + (ax as i64).round_shift(ACCELERATION_TO_VELOCITY_SHIFT))
                .clamp_to_i32(i32::MIN, i32::MAX),
            (y as i64 + (ay as i64).round_shift(ACCELERATION_TO_VELOCITY_SHIFT))
                .clamp_to_i32(i32::MIN, i32::MAX),
        ])
    }
}
