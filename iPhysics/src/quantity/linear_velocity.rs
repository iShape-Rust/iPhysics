use super::linear_acceleration::LinearAcceleration;
use super::raw::RawVec2;
use super::{ACCELERATION_TO_VELOCITY_SHIFT, KINEMATIC_FRACTION_BITS};

/// Linear velocity in metres per second, stored as signed Q24 components.
///
/// - Resolution: `2^-24 m/s`, approximately `0.000_000_059_6 m/s`.
/// - Storage range: `-128 m/s..128 m/s` (exclusive upper bound).
/// - Intended gameplay limit: vector length at most `100 m/s`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearVelocity(RawVec2);

impl LinearVelocity {
    pub const FRACTION_BITS: u32 = KINEMATIC_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(RawVec2::ZERO);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self(RawVec2::new(x, y))
    }

    /// Converts metres per second to Q24, rounding midpoint values away from zero.
    #[inline]
    pub fn from_meters_per_second(x: f64, y: f64) -> Option<Self> {
        Some(Self(RawVec2::from_f64(x, y, Self::FRACTION_BITS)?))
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        self.0.raw()
    }

    #[inline(always)]
    pub fn to_meters_per_second(self) -> [f64; 2] {
        self.0.to_f64(Self::FRACTION_BITS)
    }

    /// Applies an acceleration for one 64 Hz tick.
    #[inline]
    pub fn checked_advance(self, acceleration: LinearAcceleration) -> Option<Self> {
        Some(Self(self.0.checked_add_shifted(
            acceleration.raw_vec(),
            ACCELERATION_TO_VELOCITY_SHIFT,
        )?))
    }

    #[inline(always)]
    pub(super) const fn raw_vec(self) -> RawVec2 {
        self.0
    }
}
