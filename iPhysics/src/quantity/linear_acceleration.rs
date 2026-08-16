use super::KINEMATIC_FRACTION_BITS;
use super::raw::RawVec2;

/// Linear acceleration in metres per second squared, stored as signed Q24 components.
///
/// - Resolution: `2^-24 m/s^2`, approximately `0.000_000_059_6 m/s^2`.
/// - Storage range: `-128 m/s^2..128 m/s^2` (exclusive upper bound).
/// - Intended gameplay limit: vector length at most `100 m/s^2`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearAcceleration(RawVec2);

impl LinearAcceleration {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(RawVec2::ZERO);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self(RawVec2::new(x, y))
    }

    /// Converts metres per second squared to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_meters_per_second_squared(x: f64, y: f64) -> Option<Self> {
        Some(Self(RawVec2::from_f64(x, y, Self::FRACTION_BITS)?))
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
