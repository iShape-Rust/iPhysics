use crate::ops::quantize::Quantize;

/// Positive body mass in kilograms, stored as unsigned Q14.
///
/// - Resolution: `2^-14 kg`, approximately `0.000_061 kg`.
/// - Storage range: `0 kg..262_144 kg` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Mass(u32);

impl Mass {
    pub const FRACTION_BITS: u32 = 14;
    pub const SCALE: u64 = 1_u64 << Self::FRACTION_BITS;
    pub const ONE: Self = Self(Self::SCALE as u32);

    #[inline]
    pub const fn from_raw(raw: u32) -> Option<Self> {
        if raw == 0 {
            None
        } else {
            Some(Self(raw))
        }
    }

    #[inline]
    pub fn from_kilograms(value: f64) -> Option<Self> {
        Self::from_raw(value.quantize(Self::FRACTION_BITS)?)
    }

    #[inline(always)]
    pub const fn raw(self) -> u32 {
        self.0
    }

    #[inline(always)]
    pub fn to_kilograms(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    /// Reciprocal mass as unsigned Q24. Used only inside the solver.
    #[inline(always)]
    pub(crate) const fn inverse_q24(self) -> u32 {
        let numerator = 1_u64 << (Self::FRACTION_BITS + 24);
        let inverse = (numerator + (self.0 as u64 >> 1)) / self.0 as u64;
        if inverse > u32::MAX as u64 {
            u32::MAX
        } else {
            inverse as u32
        }
    }
}

impl Default for Mass {
    fn default() -> Self {
        Self::ONE
    }
}
