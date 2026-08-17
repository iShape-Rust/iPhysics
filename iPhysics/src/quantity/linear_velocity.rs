use super::linear_acceleration::LinearAcceleration;
use super::raw::RawVec2;
use super::{ACCELERATION_TO_VELOCITY_SHIFT, KINEMATIC_FRACTION_BITS};

/// Linear velocity in metres per second, stored as signed Q24 components.
///
/// - Resolution: `2^-24 m/s`, approximately `0.000_000_059_6 m/s`.
/// - Underlying Q24/i32 capacity: `-128 m/s..128 m/s` (exclusive upper bound).
/// - Enforced gameplay range per component: `-100 m/s..100 m/s` inclusive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearVelocity(RawVec2);

impl LinearVelocity {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const MAX_RAW: i32 = 100 * Self::SCALE as i32;
    pub const MIN_RAW: i32 = -Self::MAX_RAW;
    pub const ZERO: Self = Self(RawVec2::ZERO);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self::from_wide_saturated(x as i128, y as i128)
    }

    #[inline(always)]
    pub(crate) const fn from_wide_saturated(x: i128, y: i128) -> Self {
        Self(RawVec2::from_wide_saturated(
            x,
            y,
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

    /// Converts metres per second to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_meters_per_second(x: f64, y: f64) -> Option<Self> {
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

    /// Squared raw Q24 magnitude. The result has 48 fractional bits.
    #[inline(always)]
    pub const fn raw_sqr_magnitude(self) -> u64 {
        let [x, y] = self.raw();
        let x = x as i64;
        let y = y as i64;
        (x * x) as u64 + (y * y) as u64
    }

    #[inline(always)]
    pub fn to_meters_per_second(self) -> [f64; 2] {
        self.0.to_f64(Self::FRACTION_BITS)
    }

    /// Applies an acceleration for one 64 Hz tick and saturates each component
    /// at the enforced gameplay range.
    #[inline]
    pub const fn advance(self, acceleration: LinearAcceleration) -> Self {
        Self(self.0.add_shifted_saturated(
            acceleration.raw_vec(),
            ACCELERATION_TO_VELOCITY_SHIFT,
            Self::MIN_RAW,
            Self::MAX_RAW,
        ))
    }

    #[inline(always)]
    pub(super) const fn raw_vec(self) -> RawVec2 {
        self.0
    }
}

#[inline(always)]
const fn is_valid_raw(value: i32) -> bool {
    value >= LinearVelocity::MIN_RAW && value <= LinearVelocity::MAX_RAW
}
