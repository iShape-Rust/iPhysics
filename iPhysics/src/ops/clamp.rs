pub(crate) trait ClampToI32 {
    fn clamp_to_i32(self, min: i32, max: i32) -> i32;
}

impl ClampToI32 for i64 {
    #[inline(always)]
    fn clamp_to_i32(self, min: i32, max: i32) -> i32 {
        debug_assert!(min <= max);
        if self < min as i64 {
            min
        } else if self > max as i64 {
            max
        } else {
            self as i32
        }
    }
}

#[cfg(test)]
mod tests {
    use super::ClampToI32;

    #[test]
    fn clamps_wide_integers_to_inclusive_i32_range() {
        assert_eq!(
            (i32::MIN as i64 - 1).clamp_to_i32(i32::MIN, i32::MAX),
            i32::MIN
        );
        assert_eq!(
            (i32::MAX as i64 + 1).clamp_to_i32(i32::MIN, i32::MAX),
            i32::MAX
        );
        assert_eq!(7_i64.clamp_to_i32(-5, 5), 5);
        assert_eq!((-7_i64).clamp_to_i32(-5, 5), -5);
        assert_eq!(3_i64.clamp_to_i32(-5, 5), 3);
    }
}
