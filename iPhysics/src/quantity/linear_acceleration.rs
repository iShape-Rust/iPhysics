use super::KINEMATIC_FRACTION_BITS;
use super::raw::RawVec2;

/// Linear acceleration in metres per second squared, stored as signed Q24 components.
///
/// - Resolution: `2^-24 m/s^2`, approximately `0.000_000_059_6 m/s^2`.
/// - Underlying Q24/i32 capacity: `-128 m/s²..128 m/s²` (exclusive upper bound).
/// - Enforced gameplay range per component: `-100 m/s²..100 m/s²` inclusive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearAcceleration(RawVec2);

impl LinearAcceleration {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const MAX_RAW: i32 = 100 * Self::SCALE as i32;
    pub const MIN_RAW: i32 = -Self::MAX_RAW;
    pub const ZERO: Self = Self(RawVec2::ZERO);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self(RawVec2::from_wide_saturated(
            x as i128,
            y as i128,
            Self::MIN_RAW,
            Self::MAX_RAW,
        ))
    }

    #[inline(always)]
    pub const fn checked_from_raw(x: i32, y: i32) -> Option<Self> {
        if is_valid_raw(x) && is_valid_raw(y) {
            Some(Self(RawVec2::new(x, y)))
        } else {
            None
        }
    }

    /// Converts metres per second squared to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_meters_per_second_squared(x: f64, y: f64) -> Option<Self> {
        let raw = RawVec2::from_f64(x, y, Self::FRACTION_BITS)?;
        let [x, y] = raw.raw();
        Self::checked_from_raw(x, y)
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        self.0.raw()
    }

    #[inline(always)]
    pub const fn is_zero(self) -> bool {
        let [x, y] = self.raw();
        x == 0 && y == 0
    }

    #[inline(always)]
    pub fn to_meters_per_second_squared(self) -> [f64; 2] {
        self.0.to_f64(Self::FRACTION_BITS)
    }

    #[inline(always)]
    pub(super) const fn raw_vec(self) -> RawVec2 {
        self.0
    }
}

#[inline(always)]
const fn is_valid_raw(value: i32) -> bool {
    value >= LinearAcceleration::MIN_RAW && value <= LinearAcceleration::MAX_RAW
}
