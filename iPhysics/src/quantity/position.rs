use super::linear_velocity::LinearVelocity;
use super::raw::RawVec2;
use super::{POSITION_FRACTION_BITS, VELOCITY_TO_POSITION_SHIFT};
use crate::fix::{clamp::ClampToI32, quantize::Quantize, shift::RoundShift};
use crate::{Angle, GeometryPoint};
use i_float::int::point::IntPoint;

/// World-space position in metres, stored as signed Q16 components.
///
/// - Resolution: `2^-16 m`, approximately `0.000_015_259 m`.
/// - World range: `-16_384 m..16_384 m` (exclusive upper bound).
/// - Raw range: `-2^30..2^30` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Position([i32; 2]);

impl Position {
    pub const FRACTION_BITS: u32 = POSITION_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const MIN_RAW: i32 = -(1 << 30);
    pub const MAX_RAW: i32 = (1 << 30) - 1;
    pub const ZERO: Self = Self([0, 0]);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self([clamp_raw(x), clamp_raw(y)])
    }

    /// Creates a position when the caller has already proved the world-range
    /// invariant. No validation or saturation is performed.
    #[inline(always)]
    pub(crate) const fn from_raw_unchecked(x: i32, y: i32) -> Self {
        Self([x, y])
    }

    /// Creates a position from a wide simulation result, saturating at the
    /// representable world boundary instead of wrapping or failing the tick.
    #[inline(always)]
    pub(crate) fn from_wide_saturated(x: i128, y: i128) -> Self {
        Self([
            x.clamp_to_i32(Self::MIN_RAW, Self::MAX_RAW),
            y.clamp_to_i32(Self::MIN_RAW, Self::MAX_RAW),
        ])
    }

    #[inline(always)]
    pub(crate) fn from_i64_saturated(x: i64, y: i64) -> Self {
        Self([
            x.clamp_to_i32(Self::MIN_RAW, Self::MAX_RAW),
            y.clamp_to_i32(Self::MIN_RAW, Self::MAX_RAW),
        ])
    }

    #[inline(always)]
    pub const fn checked_from_raw(x: i32, y: i32) -> Option<Self> {
        if is_valid_raw(x) && is_valid_raw(y) {
            Some(Self([x, y]))
        } else {
            None
        }
    }

    /// Converts metres to Q16, rounding midpoint values away from zero.
    ///
    /// Floating-point conversion is intended for API and asset boundaries, not
    /// for calculations performed during a simulation step.
    #[inline(always)]
    pub fn from_meters(x: f64, y: f64) -> Option<Self> {
        Self::checked_from_raw(
            x.quantize(Self::FRACTION_BITS)?,
            y.quantize(Self::FRACTION_BITS)?,
        )
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        self.0
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
        let scale = Self::SCALE as f64;
        [self.0[0] as f64 / scale, self.0[1] as f64 / scale]
    }

    /// Advances the position by one 64 Hz tick using semi-implicit velocity.
    #[inline(always)]
    pub fn advance(self, velocity: LinearVelocity) -> Self {
        let [x, y] = self.raw();
        let [vx, vy] = velocity.raw();
        Self::from_i64_saturated(
            x as i64 + (vx as i64).round_shift(VELOCITY_TO_POSITION_SHIFT),
            y as i64 + (vy as i64).round_shift(VELOCITY_TO_POSITION_SHIFT),
        )
    }

    /// Returns the component-wise midpoint. The sum is widened before the
    /// division, and the result is guaranteed to remain inside the world.
    #[inline(always)]
    pub const fn midpoint(self, other: Self) -> Self {
        let [ax, ay] = self.raw();
        let [bx, by] = other.raw();
        Self::from_raw_unchecked(
            ((ax as i64 + bx as i64) / 2) as i32,
            ((ay as i64 + by as i64) / 2) as i32,
        )
    }

    /// Returns the squared raw Q16 distance. The result has 32 fractional
    /// bits and fits in `u64` for the bounded world range.
    #[inline(always)]
    pub fn squared_distance(self, other: Self) -> u64 {
        (self - other).squared_magnitude()
    }

    #[inline(always)]
    pub(crate) fn saturating_add(self, point: GeometryPoint) -> Self {
        let [ax, ay] = self.raw();
        let [bx, by] = point.raw();

        Self::from_i64_saturated(ax as i64 + bx as i64, ay as i64 + by as i64)
    }

    #[inline(always)]
    pub(crate) fn uncheck_add(self, point: GeometryPoint) -> GeometryPoint {
        let [ax, ay] = self.raw();
        let [bx, by] = point.raw();

        GeometryPoint::from_raw(ax + bx, ay + by)
    }

    #[inline(always)]
    pub(crate) fn rotate(self, angle: Angle) -> GeometryPoint {
        let [sin, cos] = angle.sin_cos_q30();
        let [px, py] = self.raw();

        let x = (px as i64 * cos as i64 - py as i64 * sin as i64).round_shift(30);
        let y = (px as i64 * sin as i64 + py as i64 * cos as i64).round_shift(30);

        GeometryPoint::from_raw(x as i32, y as i32)
    }
}

impl core::ops::Sub for Position {
    type Output = RawVec2;

    /// Returns the exact raw Q16 displacement `self - rhs`.
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        let [x, y] = self.raw();
        let [rhs_x, rhs_y] = rhs.raw();
        let dx = x as i64 - rhs_x as i64;
        let dy = y as i64 - rhs_y as i64;
        RawVec2::from_raw_unchecked(dx as i32, dy as i32)
    }
}

#[inline(always)]
const fn is_valid_raw(value: i32) -> bool {
    value >= Position::MIN_RAW && value <= Position::MAX_RAW
}

#[inline(always)]
const fn clamp_raw(value: i32) -> i32 {
    if value < Position::MIN_RAW {
        Position::MIN_RAW
    } else if value > Position::MAX_RAW {
        Position::MAX_RAW
    } else {
        value
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

        assert_eq!((max - min).raw(), [i32::MAX, -i32::MAX]);
    }

    #[test]
    fn raw_constructor_clamps_to_world_boundary() {
        assert_eq!(
            Position::from_raw(i32::MIN, i32::MAX).raw(),
            [Position::MIN_RAW, Position::MAX_RAW]
        );
        assert!(Position::checked_from_raw(Position::MIN_RAW, Position::MAX_RAW).is_some());
        assert!(Position::checked_from_raw(Position::MIN_RAW - 1, 0).is_none());
    }

    #[test]
    fn midpoint_widens_and_stays_in_world() {
        let min = Position::from_raw(Position::MIN_RAW, Position::MIN_RAW);
        let max = Position::from_raw(Position::MAX_RAW, Position::MAX_RAW);

        assert_eq!(min.midpoint(max), Position::ZERO);
    }

    #[test]
    fn squared_distance_uses_bounded_difference() {
        let a = Position::from_raw(3, 4);

        assert_eq!(a.squared_distance(Position::ZERO), 25);
    }
}
