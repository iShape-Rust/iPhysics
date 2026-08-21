use crate::ops::quantize::Quantize;

use super::LINEAR_ACCELERATION_FRACTION_BITS;

/// Linear acceleration in metres per second squared, stored as signed Q4 components.
///
/// - Resolution: `2^-4 m/s²`, or `0.0625 m/s²`.
/// - Range per component: `-65_536 m/s²..=65_536 m/s²`.
/// - Raw range: `-2^20..=2^20`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearAcceleration([i32; 2]);

impl LinearAcceleration {
    pub const FRACTION_BITS: u32 = LINEAR_ACCELERATION_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub(crate) const MIN_ACCELERATION: i32 = -(1 << 20);
    pub(crate) const MAX_ACCELERATION: i32 = 1 << 20;
    pub const ZERO: Self = Self([0, 0]);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self([Self::clamp_raw(x), Self::clamp_raw(y)])
    }

    #[inline(always)]
    const fn clamp_raw(value: i32) -> i32 {
        if value < Self::MIN_ACCELERATION {
            Self::MIN_ACCELERATION
        } else if value > Self::MAX_ACCELERATION {
            Self::MAX_ACCELERATION
        } else {
            value
        }
    }

    #[inline(always)]
    const fn checked_from_raw(x: i32, y: i32) -> Option<Self> {
        let is_x = x >= Self::MIN_ACCELERATION && x <= Self::MAX_ACCELERATION;
        let is_y = y >= Self::MIN_ACCELERATION && y <= Self::MAX_ACCELERATION;
        if is_x && is_y {
            Some(Self([x, y]))
        } else {
            None
        }
    }

    /// Converts metres per second squared to Q4, rounding midpoint values away from zero.
    #[inline(always)]
    pub fn from_meters_per_second_squared(x: f64, y: f64) -> Option<Self> {
        Self::checked_from_raw(
            x.quantize(Self::FRACTION_BITS)?,
            y.quantize(Self::FRACTION_BITS)?,
        )
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
