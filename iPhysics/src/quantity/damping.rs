use crate::ops::quantize::Quantize;

use super::{AngularVelocity, LinearVelocity};

/// Fraction of velocity lost during one fixed 64 Hz simulation tick.
///
/// The public coefficient is in `0..=1`: zero preserves all velocity and one
/// removes it completely. Internally the complementary velocity-retention
/// multiplier is stored in Q16 so applying damping only requires a multiply
/// and a shift.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Damping {
    retention_q16: u32,
}

impl Damping {
    const FRACTION_BITS: u32 = 16;
    const SCALE: u32 = 1 << Self::FRACTION_BITS;

    /// No velocity loss.
    pub const NONE: Self = Self {
        retention_q16: Self::SCALE,
    };

    /// Complete velocity loss in one tick.
    pub const FULL: Self = Self { retention_q16: 0 };

    /// Creates a per-tick damping coefficient in `0..=1`.
    #[inline]
    pub fn new(coefficient: f64) -> Option<Self> {
        if !coefficient.is_finite() || !(0.0..=1.0).contains(&coefficient) {
            return None;
        }

        let coefficient_q16: u32 = coefficient.quantize(Self::FRACTION_BITS)?;
        Some(Self {
            retention_q16: Self::SCALE - coefficient_q16,
        })
    }

    /// Creates damping from its Q16 loss coefficient.
    #[inline(always)]
    pub const fn from_raw(coefficient_q16: u32) -> Option<Self> {
        if coefficient_q16 <= Self::SCALE {
            Some(Self {
                retention_q16: Self::SCALE - coefficient_q16,
            })
        } else {
            None
        }
    }

    /// Returns the Q16 fraction of velocity lost per tick.
    #[inline(always)]
    pub const fn raw(self) -> u32 {
        Self::SCALE - self.retention_q16
    }

    /// Returns the quantized fraction of velocity lost per tick.
    #[inline(always)]
    pub fn coefficient(self) -> f64 {
        self.raw() as f64 / Self::SCALE as f64
    }

    #[inline(always)]
    pub(crate) fn apply_linear(self, velocity: LinearVelocity) -> LinearVelocity {
        let [x, y] = velocity.raw();
        LinearVelocity::from_raw(self.apply_raw(x), self.apply_raw(y))
    }

    #[inline(always)]
    pub(crate) fn apply_angular(self, velocity: AngularVelocity) -> AngularVelocity {
        AngularVelocity::from_raw(self.apply_raw(velocity.raw()))
    }

    /// Multiplies a signed value by the unsigned retention coefficient. Work
    /// on the magnitude so the shift truncates toward zero for both signs.
    #[inline(always)]
    fn apply_raw(self, value: i32) -> i32 {
        let magnitude =
            (value.unsigned_abs() as u64 * self.retention_q16 as u64) >> Self::FRACTION_BITS;
        let signed = if value < 0 {
            -(magnitude as i64)
        } else {
            magnitude as i64
        };
        signed as i32
    }
}

impl Default for Damping {
    fn default() -> Self {
        Self::NONE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantizes_public_loss_but_stores_retention() {
        let damping = Damping::new(0.001).unwrap();

        assert_eq!(damping.raw(), 66);
        assert_eq!(damping.retention_q16, (1 << 16) - 66);
        assert_eq!(Damping::NONE.raw(), 0);
        assert_eq!(Damping::FULL.raw(), 1 << 16);
    }

    #[test]
    fn rejects_coefficients_outside_unit_interval() {
        assert!(Damping::new(-0.001).is_none());
        assert!(Damping::new(1.001).is_none());
        assert!(Damping::new(f64::NAN).is_none());
        assert!(Damping::from_raw((1 << 16) + 1).is_none());
    }

    #[test]
    fn damping_is_symmetric_and_truncates_toward_zero() {
        let damping = Damping::new(0.25).unwrap();

        assert_eq!(damping.apply_raw(5), 3);
        assert_eq!(damping.apply_raw(-5), -3);
        assert_eq!(damping.apply_raw(1), 0);
        assert_eq!(damping.apply_raw(-1), 0);
    }

    #[test]
    fn handles_full_i32_range() {
        assert_eq!(Damping::NONE.apply_raw(i32::MIN), i32::MIN);
        assert_eq!(Damping::NONE.apply_raw(i32::MAX), i32::MAX);
        assert_eq!(Damping::FULL.apply_raw(i32::MIN), 0);
    }
}
