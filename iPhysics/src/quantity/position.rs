use super::linear_velocity::LinearVelocity;
use super::{POSITION_FRACTION_BITS, VELOCITY_TO_POSITION_SHIFT};
use crate::ops::{quantize::Quantize, shift::RoundShift};
use crate::geometry::vec::RawVec2;
use crate::{Angle, GeometryPoint};
use i_float::int::point::IntPoint;

/// World-space position in metres, stored as signed Q16 components.
///
/// - Resolution: `2^-16 m`, approximately `0.000_015_259 m`.
/// - World range: approximately `-8_192 m..8_192 m` per component.
/// - Raw range: `-(2^29 - 1)..=(2^29 - 1)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Position([i32; 2]);

impl Position {
    pub(crate) const FRACTION_BITS: u32 = POSITION_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub(crate) const MIN_POSITION: i32 = -(1 << 29) + 1;
    pub(crate) const MAX_POSITION: i32 = (1 << 29) - 1;
    pub(crate) const MIN_POINT: i32 = -(1 << 30) + 1;
    pub(crate) const MAX_POINT: i32 = (1 << 30) - 1;

    pub const ZERO: Self = Self([0, 0]);

    #[inline(always)]
    pub(crate) fn from_i32(x: i32, y: i32) -> Self {
        let x = x.clamp(Position::MIN_POSITION, Position::MAX_POSITION);
        let y = y.clamp(Position::MIN_POSITION, Position::MAX_POSITION);
        Self([x, y])
    }

    #[inline(always)]
    pub(crate) fn from_i64(x: i64, y: i64) -> Self {
        let x = x.clamp(Position::MIN_POSITION as i64, Position::MAX_POSITION as i64);
        let y = y.clamp(Position::MIN_POSITION as i64, Position::MAX_POSITION as i64);
        Self([x as i32, y as i32])
    }

    #[inline(always)]
    pub(crate) fn from_i128(x: i128, y: i128) -> Self {
        let x = x.clamp(
            Position::MIN_POSITION as i128,
            Position::MAX_POSITION as i128,
        );
        let y = y.clamp(
            Position::MIN_POSITION as i128,
            Position::MAX_POSITION as i128,
        );
        Self([x as i32, y as i32])
    }

    /// Creates a position when the caller has already proved the world-range
    /// invariant. No validation or saturation is performed.
    #[cfg(test)]
    #[inline(always)]
    pub(crate) const fn from_i32_unchecked(x: i32, y: i32) -> Self {
        debug_assert!(x >= Self::MIN_POSITION && x <= Self::MAX_POSITION);
        debug_assert!(y >= Self::MIN_POSITION && y <= Self::MAX_POSITION);
        Self([x, y])
    }

    #[inline(always)]
    pub(crate) const fn checked_from_raw(x: i32, y: i32) -> Option<Self> {
        let is_x = x >= Self::MIN_POSITION && x <= Self::MAX_POSITION;
        let is_y = y >= Self::MIN_POSITION && y <= Self::MAX_POSITION;
        if is_x && is_y {
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
    pub(crate) const fn raw(self) -> [i32; 2] {
        self.0
    }

    /// Returns the raw Q16 coordinates as an `i_float` point.
    #[inline(always)]
    pub const fn raw_point(self) -> IntPoint<i32> {
        let [x, y] = self.raw();
        IntPoint { x, y }
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
        Self::from_i64(
            x as i64 + (vx as i64).round_shift(VELOCITY_TO_POSITION_SHIFT),
            y as i64 + (vy as i64).round_shift(VELOCITY_TO_POSITION_SHIFT),
        )
    }

    /// Returns the component-wise midpoint, which remains inside the world.
    #[cfg(test)]
    #[inline(always)]
    pub(crate) const fn midpoint(self, other: Self) -> Self {
        let [ax, ay] = self.raw();
        let [bx, by] = other.raw();
        Self::from_i32_unchecked((ax + bx) / 2, (ay + by) / 2)
    }

    /// Returns the squared raw Q16 distance. The result has 32 fractional
    /// bits and fits in `u64` for the bounded world range.
    #[inline(always)]
    pub(crate) fn squared_distance(self, other: Self) -> u64 {
        (self - other).squared_magnitude()
    }

    #[inline(always)]
    pub(crate) fn add_point(self, point: GeometryPoint) -> Self {
        let [ax, ay] = self.raw();
        let [bx, by] = point.raw();

        Self::from_i32(ax + bx, ay + by)
    }

    #[inline(always)]
    pub(crate) fn add_geometry_unchecked(self, point: GeometryPoint) -> GeometryPoint {
        let [ax, ay] = self.raw();
        let [bx, by] = point.raw();

        GeometryPoint::from_i32_unchecked(ax + bx, ay + by)
    }

    #[inline(always)]
    pub(crate) fn rotate(self, angle: Angle) -> GeometryPoint {
        let [sin, cos] = angle.sin_cos_q30();
        let [px, py] = self.raw();

        let x = (px as i64 * cos as i64 - py as i64 * sin as i64).round_shift(30);
        let y = (px as i64 * sin as i64 + py as i64 * cos as i64).round_shift(30);

        GeometryPoint::from_i32_unchecked(x as i32, y as i32)
    }
}

impl core::ops::Sub for Position {
    type Output = RawVec2;

    /// Returns the exact raw Q16 displacement `self - rhs`.
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        let [x, y] = self.raw();
        let [rhs_x, rhs_y] = rhs.raw();
        let dx = x - rhs_x;
        let dy = y - rhs_y;
        RawVec2::from_i32(dx, dy)
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
        Self::from_i32(point.x, point.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subtraction_stays_inside_raw_vector_range() {
        let max = Position::from_i32(i32::MAX, i32::MIN);
        let min = Position::from_i32(i32::MIN, i32::MAX);

        assert_eq!(
            (max - min).raw(),
            [2 * Position::MAX_POSITION, -2 * Position::MAX_POSITION]
        );
    }

    #[test]
    fn raw_constructor_clamps_to_world_boundary() {
        assert_eq!(
            Position::from_i32(i32::MIN, i32::MAX).raw(),
            [Position::MIN_POSITION, Position::MAX_POSITION]
        );
        assert!(
            Position::checked_from_raw(Position::MIN_POSITION, Position::MAX_POSITION).is_some()
        );
        assert!(Position::checked_from_raw(Position::MIN_POSITION - 1, 0).is_none());
    }

    #[test]
    fn midpoint_stays_in_world() {
        let min = Position::from_i32(Position::MIN_POSITION, Position::MIN_POSITION);
        let max = Position::from_i32(Position::MAX_POSITION, Position::MAX_POSITION);

        assert_eq!(min.midpoint(max), Position::ZERO);
    }

    #[test]
    fn squared_distance_uses_bounded_difference() {
        let a = Position::from_i32(3, 4);

        assert_eq!(a.squared_distance(Position::ZERO), 25);
    }
}
