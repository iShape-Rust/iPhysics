pub(crate) trait Quantize<T> {
    fn quantize(self, fraction_bits: u32) -> Option<T>;
}

impl Quantize<i32> for f64 {
    #[inline]
    fn quantize(self, fraction_bits: u32) -> Option<i32> {
        if !self.is_finite() {
            return None;
        }

        let scale = (1_u64 << fraction_bits) as f64;
        let scaled = self * scale;
        if scaled < i32::MIN as f64 || scaled > i32::MAX as f64 {
            return None;
        }

        let truncated = scaled as i32;
        let fraction = scaled - truncated as f64;
        if fraction >= 0.5 {
            truncated.checked_add(1)
        } else if fraction <= -0.5 {
            truncated.checked_sub(1)
        } else {
            Some(truncated)
        }
    }
}

impl Quantize<u32> for f64 {
    #[inline]
    fn quantize(self, fraction_bits: u32) -> Option<u32> {
        if !self.is_finite() || self < 0.0 {
            return None;
        }

        let scale = (1_u64 << fraction_bits) as f64;
        let scaled = self * scale;
        if scaled > u32::MAX as f64 {
            return None;
        }

        let truncated = scaled as u32;
        if scaled - truncated as f64 >= 0.5 {
            truncated.checked_add(1)
        } else {
            Some(truncated)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Quantize;

    #[test]
    fn rounds_midpoints_away_from_zero() {
        let positive: Option<i32> = 0.5_f64.quantize(0);
        let negative: Option<i32> = (-0.5_f64).quantize(0);
        let unsigned: Option<u32> = 0.5_f64.quantize(0);

        assert_eq!(positive, Some(1));
        assert_eq!(negative, Some(-1));
        assert_eq!(unsigned, Some(1));
    }

    #[test]
    fn rejects_invalid_or_unrepresentable_values() {
        let nan: Option<i32> = f64::NAN.quantize(0);
        let negative_unsigned: Option<u32> = (-1.0_f64).quantize(0);
        let signed_overflow: Option<i32> = (i32::MAX as f64 + 1.0).quantize(0);
        let unsigned_overflow: Option<u32> = (u32::MAX as f64 + 1.0).quantize(0);

        assert_eq!(nan, None);
        assert_eq!(negative_unsigned, None);
        assert_eq!(signed_overflow, None);
        assert_eq!(unsigned_overflow, None);
    }
}
