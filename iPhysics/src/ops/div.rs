pub(crate) trait DivRoundShift {
    /// Computes `round(self * 2^shift / denominator)` without requiring the
    /// shifted numerator to fit in `u128`, saturating to `u64`.
    fn div_round_shift_saturating(self, denominator: u128, shift: u32) -> u64;
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
}
