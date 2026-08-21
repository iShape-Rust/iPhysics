use crate::ops::quantize::Quantize;

use super::KINEMATIC_FRACTION_BITS;

/// Linear acceleration in metres per second squared, stored as signed Q24 components.
///
/// - Resolution: `2^-24 m/s^2`, approximately `0.000_000_059_6 m/s^2`.
/// - Storage range per component: `-128 m/s²..128 m/s²` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearAcceleration([i32; 2]);

impl LinearAcceleration {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self([0, 0]);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self([x, y])
    }

    /// Converts metres per second squared to Q24, rounding midpoint values away from zero.
    #[inline(always)]
    pub fn from_meters_per_second_squared(x: f64, y: f64) -> Option<Self> {
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

    #[inline(always)]
    pub fn to_meters_per_second_squared(self) -> [f64; 2] {
        let scale = Self::SCALE as f64;
        [self.0[0] as f64 / scale, self.0[1] as f64 / scale]
    }
}
