use super::linear_velocity::LinearVelocity;
use super::raw::{RawVec2, RawWideVec2};
use super::{POSITION_FRACTION_BITS, VELOCITY_TO_POSITION_SHIFT};
use i_float::int::point::IntPoint;

/// World-space position in metres, stored as signed Q16 components.
///
/// - Resolution: `2^-16 m`, approximately `0.000_015_259 m`.
/// - Storage range: `-32_768 m..32_768 m` (exclusive upper bound).
/// - Conservative `i_float` geometry range: `-16_384 m..16_384 m`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Position(RawVec2);

impl Position {
    pub const FRACTION_BITS: u32 = POSITION_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const ZERO: Self = Self(RawVec2::ZERO);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self(RawVec2::new(x, y))
    }

    /// Converts metres to Q16, rounding midpoint values away from zero.
    ///
    /// Floating-point conversion is intended for API and asset boundaries, not
    /// for calculations performed during a simulation step.
    #[inline]
    pub fn from_meters(x: f64, y: f64) -> Option<Self> {
        Some(Self(RawVec2::from_f64(x, y, Self::FRACTION_BITS)?))
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        self.0.raw()
    }

    /// Returns the raw Q16 coordinates as an `i_float` point.
    #[inline(always)]
    pub const fn raw_point(self) -> IntPoint<i32> {
        let [x, y] = self.raw();
        IntPoint { x, y }
    }

    /// Creates a position from raw Q16 `i_float` coordinates.
    #[inline(always)]
    pub const fn from_raw_point(point: IntPoint<i32>) -> Self {
        Self::from_raw(point.x, point.y)
    }

    #[inline(always)]
    pub fn to_meters(self) -> [f64; 2] {
        self.0.to_f64(Self::FRACTION_BITS)
    }

    /// Advances the position by one 64 Hz tick using semi-implicit velocity.
    #[inline]
    pub fn checked_advance(self, velocity: LinearVelocity) -> Option<Self> {
        Some(Self(self.0.checked_add_shifted(
            velocity.raw_vec(),
            VELOCITY_TO_POSITION_SHIFT,
        )?))
    }
}

impl core::ops::Sub for Position {
    type Output = RawWideVec2;

    /// Returns the exact raw Q16 displacement `self - rhs`.
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        let [x, y] = self.raw();
        let [rhs_x, rhs_y] = rhs.raw();
        RawWideVec2::from_raw(x as i64 - rhs_x as i64, y as i64 - rhs_y as i64)
    }
}

impl From<Position> for IntPoint<i32> {
    #[inline(always)]
    fn from(position: Position) -> Self {
        position.raw_point()
    }
}

impl From<IntPoint<i32>> for Position {
    #[inline(always)]
    fn from(point: IntPoint<i32>) -> Self {
        Self::from_raw_point(point)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subtraction_widens_before_operating() {
        let max = Position::from_raw(i32::MAX, i32::MIN);
        let min = Position::from_raw(i32::MIN, i32::MAX);

        assert_eq!(
            (max - min).raw(),
            [
                i32::MAX as i64 - i32::MIN as i64,
                i32::MIN as i64 - i32::MAX as i64
            ]
        );
    }
}
