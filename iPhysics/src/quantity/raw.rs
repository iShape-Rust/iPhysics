#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub(super) struct RawVec2 {
    x: i32,
    y: i32,
}

impl RawVec2 {
    pub(super) const ZERO: Self = Self { x: 0, y: 0 };

    #[inline(always)]
    pub(super) const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub(super) const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }

    #[inline]
    pub(super) fn from_f64(x: f64, y: f64, fraction_bits: u32) -> Option<Self> {
        Some(Self {
            x: quantize_f64(x, fraction_bits)?,
            y: quantize_f64(y, fraction_bits)?,
        })
    }

    #[inline(always)]
    pub(super) fn to_f64(self, fraction_bits: u32) -> [f64; 2] {
        let scale = (1_u64 << fraction_bits) as f64;
        [self.x as f64 / scale, self.y as f64 / scale]
    }

    #[inline]
    pub(super) fn checked_add_shifted(self, rhs: Self, shift: u32) -> Option<Self> {
        let x = self.x as i64 + shift_round(rhs.x as i64, shift);
        let y = self.y as i64 + shift_round(rhs.y as i64, shift);

        Some(Self {
            x: i32::try_from(x).ok()?,
            y: i32::try_from(y).ok()?,
        })
    }
}

#[inline]
pub(super) fn quantize_f64(value: f64, fraction_bits: u32) -> Option<i32> {
    if !value.is_finite() {
        return None;
    }

    let scale = (1_u64 << fraction_bits) as f64;
    let scaled = value * scale;
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

/// Rounds to the nearest integer; midpoint values are rounded away from zero.
#[inline(always)]
pub(super) fn shift_round(value: i64, shift: u32) -> i64 {
    debug_assert!(shift > 0);
    let half = 1_i64 << (shift - 1);
    if value < 0 {
        -((-value + half) >> shift)
    } else {
        (value + half) >> shift
    }
}
