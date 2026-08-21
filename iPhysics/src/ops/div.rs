pub(crate) trait DivRoundShift {
    /// Computes `round(self * 2^shift / denominator)` without requiring the
    /// shifted numerator to fit in `u128`, saturating to `u64`.
    fn div_round_shift_saturating(self, denominator: u128, shift: u32) -> u64;
}

pub(crate) trait DivRound {
    fn div_round(self, denominator: Self) -> Self;
}

impl DivRound for i128 {
    #[inline(always)]
    fn div_round(self, denominator: Self) -> Self {
        debug_assert!(denominator != 0);
        let negative = (self < 0) != (denominator < 0);
        let numerator = self.unsigned_abs();
        let denominator = denominator.unsigned_abs();
        let magnitude = (numerator + (denominator >> 1)) / denominator;
        if negative {
            -(magnitude as i128)
        } else {
            magnitude as i128
        }
    }
}

impl DivRoundShift for u128 {
    fn div_round_shift_saturating(self, denominator: u128, shift: u32) -> u64 {
        debug_assert!(denominator > 0);
        debug_assert!(shift < 64);

        let integer = self / denominator;
        if integer > (u64::MAX >> shift) as u128 {
            return u64::MAX;
        }

        let mut result = (integer as u64) << shift;
        let mut remainder = self % denominator;
        for bit in (0..shift).rev() {
            remainder *= 2;
            if remainder >= denominator {
                remainder -= denominator;
                result |= 1_u64 << bit;
            }
        }

        if remainder * 2 >= denominator {
            result = result.saturating_add(1);
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shifted_division_rounds_and_saturates() {
        assert_eq!(1_u128.div_round_shift_saturating(3, 4), 5);
        assert_eq!(u128::MAX.div_round_shift_saturating(1, 40), u64::MAX);
    }

    #[test]
    fn signed_division_rounds_away_from_zero() {
        assert_eq!(5_i128.div_round(2), 3);
        assert_eq!((-5_i128).div_round(2), -3);
        assert_eq!(5_i128.div_round(-2), -3);
    }
}
