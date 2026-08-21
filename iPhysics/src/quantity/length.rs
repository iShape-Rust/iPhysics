use crate::ops::quantize::Quantize;

use super::{POSITION_FRACTION_BITS, Position};

/// Non-negative length in metres, stored as unsigned Q16.
///
/// - Resolution: `2^-16 m`, approximately `0.000_015_259 m`.
/// - Simulation range: `0 m..16_384 m` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Length(u32);

impl Length {
    pub(crate) const FRACTION_BITS: u32 = POSITION_FRACTION_BITS;
    pub(crate) const SCALE: u64 = 1_u64 << Self::FRACTION_BITS;
    #[cfg(test)]
    pub(crate) const ZERO: Self = Self(0);
    pub(crate) const MAX_LENGTH: u32 = Position::MAX_POINT as u32;

    #[inline(always)]
    pub(crate) const fn from_raw(raw: u32) -> Self {
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
    pub(crate) const fn raw(self) -> u32 {
        self.0
    }

    #[inline(always)]
    pub fn to_meters(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }
}
