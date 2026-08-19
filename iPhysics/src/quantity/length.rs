use crate::fix::quantize::Quantize;

use super::POSITION_FRACTION_BITS;

/// Non-negative length in metres, stored as unsigned Q16.
///
/// - Resolution: `2^-16 m`, approximately `0.000_015_259 m`.
/// - Storage range: `0 m...65_536 m` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Length(u32);

impl Length {
    pub const FRACTION_BITS: u32 = POSITION_FRACTION_BITS;
    pub const SCALE: u64 = 1_u64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(0);
    pub const MAX_LENGTH: u32 = (1 << 29) - 1;

    #[inline(always)]
    pub const fn from_raw(raw: u32) -> Self {
        debug_assert!(raw <= Self::MAX_LENGTH);
        Self(raw)
    }

    #[inline]
    pub fn from_meters(value: f64) -> Option<Self> {
        let quant: u32 = value.quantize(Self::FRACTION_BITS)?;
        if quant > Self::MAX_LENGTH {
            None
        } else {
            Some(Self(quant))
        }
    }

    #[inline(always)]
    pub const fn raw(self) -> u32 {
        self.0
    }

    #[inline(always)]
    pub fn to_meters(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }
}
