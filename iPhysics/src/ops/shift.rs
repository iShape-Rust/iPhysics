pub(crate) trait RoundShift {
    fn round_shift(self, shift: u32) -> Self;
}

impl RoundShift for i64 {
    #[inline(always)]
    fn round_shift(self, shift: u32) -> i64 {
        let rounded = (self.unsigned_abs() + (1_u64 << (shift - 1))) >> shift;
        if self < 0 {
            -(rounded as i64)
        } else {
            rounded as i64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::RoundShift;

    #[test]
    fn signed_shift_rounds_away_from_zero() {
        assert_eq!(5_i64.round_shift(1), 3);
        assert_eq!((-5_i64).round_shift(1), -3);
        assert_eq!(i64::MIN.round_shift(1), i64::MIN / 2);
    }
}
