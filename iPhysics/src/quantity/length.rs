use crate::ops::quantize::Quantize;

use super::{POSITION_FRACTION_BITS, Position};

/// Non-negative length in metres, stored as unsigned Q16.
///
/// - Resolution: `2^-16 m`, approximately `0.000_015_259 m`.
/// - Simulation range: `0 m..16_384 m` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Length(u32);

impl Length {
    /// Number of fractional bits in the raw Q16 representation.
    pub const FRACTION_BITS: u32 = POSITION_FRACTION_BITS;
    /// Raw units per metre.
    pub const SCALE: u64 = 1_u64 << Self::FRACTION_BITS;
    /// Zero length.
    pub const ZERO: Self = Self(0);
    pub(crate) const MAX_LENGTH: u32 = Position::MAX_POINT as u32;

    /// Creates a length from raw Q16 units.
    ///
    /// Returns `None` when the value exceeds the simulation length range.
    #[inline(always)]
    pub const fn from_raw(raw: u32) -> Option<Self> {
        if raw <= Self::MAX_LENGTH {
            Some(Self(raw))
        } else {
            None
        }
    }

    #[inline]
    pub fn from_meters(value: f64) -> Option<Self> {
        Self::from_raw(value.quantize(Self::FRACTION_BITS)?)
    }

    /// Returns the raw Q16 length.
    #[inline(always)]
    pub const fn raw(self) -> u32 {
        self.0
    }

    #[inline(always)]
    pub fn to_meters(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    #[inline(always)]
    pub fn sqr_length(self) -> u64 {
        let l = self.0 as u64;
        l * l
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_constructor_enforces_the_length_range() {
        let max = Length::from_raw(Length::MAX_LENGTH).unwrap();

        assert_eq!(max.raw(), Length::MAX_LENGTH);
        assert!(Length::from_raw(Length::MAX_LENGTH + 1).is_none());
    }
}
