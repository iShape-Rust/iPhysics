pub(crate) trait RoundShift {
    fn round_shift(self, shift: u32) -> Self;
}

impl RoundShift for i64 {
    #[inline(always)]
    fn round_shift(self, shift: u32) -> i64 {
        let half = 1_i64 << (shift - 1);
        if self < 0 {
            -((-self + half) >> shift)
        } else {
            (self + half) >> shift
        }
    }
}
