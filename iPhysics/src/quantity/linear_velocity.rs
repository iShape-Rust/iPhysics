use crate::geometry::vec::RawVec2;
use crate::ops::{clamp::ClampToI32, quantize::Quantize};

use super::linear_acceleration::LinearAcceleration;
use super::LINEAR_VELOCITY_FRACTION_BITS;

/// Linear velocity in metres per second, stored as signed Q10 components.
///
/// - Resolution: `2^-10 m/s`, or `0.000_976_562_5 m/s`.
/// - Range per component: `-1_024 m/s..=1_024 m/s`.
/// - Raw range: `-2^20..=2^20`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearVelocity([i32; 2]);

impl LinearVelocity {
    pub const FRACTION_BITS: u32 = LINEAR_VELOCITY_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub(crate) const MIN_VELOCITY: i32 = -(1 << 20);
    pub(crate) const MAX_VELOCITY: i32 = 1 << 20;
    pub const ZERO: Self = Self([0, 0]);

    /// Creates a Q10 velocity, saturating each raw component to the supported
    /// symmetric range.
    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self([Self::clamp_raw(x), Self::clamp_raw(y)])
    }

    #[inline(always)]
    const fn clamp_raw(value: i32) -> i32 {
        if value < Self::MIN_VELOCITY {
            Self::MIN_VELOCITY
        } else if value > Self::MAX_VELOCITY {
            Self::MAX_VELOCITY
        } else {
            value
        }
    }

    #[inline(always)]
    const fn checked_from_raw(x: i32, y: i32) -> Option<Self> {
        let is_x = x >= Self::MIN_VELOCITY && x <= Self::MAX_VELOCITY;
        let is_y = y >= Self::MIN_VELOCITY && y <= Self::MAX_VELOCITY;
        if is_x && is_y {
            Some(Self([x, y]))
        } else {
            None
        }
    }

    #[inline(always)]
    pub(crate) fn from_wide_saturated(x: i64, y: i64) -> Self {
        Self([
            x.clamp_to_i32(Self::MIN_VELOCITY, Self::MAX_VELOCITY),
            y.clamp_to_i32(Self::MIN_VELOCITY, Self::MAX_VELOCITY),
        ])
    }

    /// Converts metres per second to Q10, rounding midpoint values away from zero.
    #[inline(always)]
    pub fn from_meters_per_second(x: f64, y: f64) -> Option<Self> {
        Self::checked_from_raw(
            x.quantize(Self::FRACTION_BITS)?,
            y.quantize(Self::FRACTION_BITS)?,
        )
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        self.0
    }

    #[inline(always)]
    pub const fn is_zero(self) -> bool {
        let [x, y] = self.raw();
        x == 0 && y == 0
    }

    /// Squared raw Q10 magnitude. The result has 20 fractional bits.
    #[inline(always)]
    pub const fn raw_sqr_magnitude(self) -> u64 {
        let [x, y] = self.raw();
        let x = x as i64;
        let y = y as i64;
        (x * x) as u64 + (y * y) as u64
    }

    #[inline(always)]
    pub fn to_meters_per_second(self) -> [f64; 2] {
        let scale = Self::SCALE as f64;
        [self.0[0] as f64 / scale, self.0[1] as f64 / scale]
    }

    /// Applies an acceleration for one 64 Hz tick and saturates each component
    /// at the linear-velocity range.
    #[inline(always)]
    pub fn advance(self, acceleration: LinearAcceleration) -> Self {
        let [x, y] = self.raw();
        let [ax, ay] = acceleration.raw();
        Self::from_raw(x + ax, y + ay)
    }
}

impl core::ops::Sub for LinearVelocity {
    type Output = RawVec2;

    /// Returns the exact raw Q10 velocity difference `self - rhs`.
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        let [x, y] = self.raw();
        let [rhs_x, rhs_y] = rhs.raw();
        RawVec2::from_i32(x - rhs_x, y - rhs_y)
    }
}
