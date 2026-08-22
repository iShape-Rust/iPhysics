use crate::ops::quantize::Quantize;

/// Non-negative force in newtons, stored as unsigned Q16.
///
/// At the fixed 64 Hz simulation rate, conversion to the solver's Q10
/// impulse format is an exact 12-bit scale change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Force(u32);

impl Force {
    pub const FRACTION_BITS: u32 = 16;
    pub const SCALE: u64 = 1_u64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    /// Converts newtons to Q16, rounding midpoint values away from zero.
    ///
    /// Floating-point conversion is intended for API boundaries, not for
    /// calculations performed during a simulation step.
    #[inline]
    pub fn from_newtons(value: f64) -> Option<Self> {
        if value < 0.0 {
            return None;
        }
        Some(Self(value.quantize(Self::FRACTION_BITS)?))
    }

    #[inline(always)]
    pub const fn raw(self) -> u32 {
        self.0
    }

    #[inline(always)]
    pub fn to_newtons(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    /// Maximum impulse for one 64 Hz tick in Q10 kg*m/s.
    #[inline(always)]
    pub(crate) const fn impulse_per_tick_q10(self) -> u64 {
        (self.0 as u64) >> 12
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_newton_maps_to_one_tick_impulse() {
        let force = Force::from_newtons(1.0).unwrap();

        assert_eq!(force.raw(), 1 << 16);
        assert_eq!(force.impulse_per_tick_q10(), 16);
        assert_eq!(force.to_newtons(), 1.0);
    }

    #[test]
    fn rejects_negative_force() {
        assert!(Force::from_newtons(-0.001).is_none());
    }
}
