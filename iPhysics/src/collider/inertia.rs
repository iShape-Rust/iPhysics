use crate::ops::div::DivRoundShift;

/// Converts a ratio between inverse-mass Q24 and squared-length Q32 values to
/// inverse-inertia Q40. The required scale adjustment is `2^(40 + 32 - 24)`.
pub(super) fn from_q24_per_q32_ratio(numerator: u128, denominator: u128) -> u64 {
    numerator.div_round_shift_saturating(denominator, 48)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q24_per_q32_ratio_becomes_q40() {
        assert_eq!(from_q24_per_q32_ratio(1 << 24, 1 << 32), 1 << 40);
    }
}
