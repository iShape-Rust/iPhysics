use super::linear_velocity::LinearVelocity;
use super::raw::{DiffVec2, RawVec2};
use super::{POSITION_FRACTION_BITS, VELOCITY_TO_POSITION_SHIFT};
use i_float::int::point::IntPoint;

/// World-space position in metres, stored as signed Q16 components.
///
/// - Resolution: `2^-16 m`, approximately `0.000_015_259 m`.
/// - World range: `-16_384 m..16_384 m` (exclusive upper bound).
/// - Raw range: `-2^30..2^30` (exclusive upper bound).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Position(RawVec2);

impl Position {
    pub const FRACTION_BITS: u32 = POSITION_FRACTION_BITS;
    pub const SCALE: i64 = 1_i64 << Self::FRACTION_BITS;
    pub const MIN_RAW: i32 = -(1 << 30);
    pub const MAX_RAW: i32 = (1 << 30) - 1;
    pub const ZERO: Self = Self(RawVec2::ZERO);

    #[inline(always)]
    pub const fn from_raw(x: i32, y: i32) -> Self {
        Self(RawVec2::new(clamp_raw(x), clamp_raw(y)))
    }

    /// Creates a position when the caller has already proved the world-range
    /// invariant. No validation or saturation is performed.
    #[inline(always)]
    pub(crate) const fn from_raw_unchecked(x: i32, y: i32) -> Self {
        Self(RawVec2::new(x, y))
    }

    /// Creates a position from a wide simulation result, saturating at the
    /// representable world boundary instead of wrapping or failing the tick.
    #[inline(always)]
    pub(crate) const fn from_wide_saturated(x: i128, y: i128) -> Self {
        Self(RawVec2::from_wide_saturated(
            x,
            y,
            Self::MIN_RAW,
            Self::MAX_RAW,
        ))
    }

    /// Creates a position from geometry whose range must fit the world by
    /// construction. Release builds still saturate defensively.
    #[inline(always)]
    pub(crate) const fn from_wide_narrow(x: i64, y: i64) -> Self {
        debug_assert!(is_valid_wide(x) && is_valid_wide(y));
        Self(RawVec2::from_i64_saturated(
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

    /// Converts metres to Q16, rounding midpoint values away from zero.
    ///
    /// Floating-point conversion is intended for API and asset boundaries, not
    /// for calculations performed during a simulation step.
    #[inline]
    pub fn from_meters(x: f64, y: f64) -> Option<Self> {
        let raw = RawVec2::from_f64(x, y, Self::FRACTION_BITS)?;
        let [x, y] = raw.raw();
        Self::checked_from_raw(x, y)
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
    pub const fn advance(self, velocity: LinearVelocity) -> Self {
        Self(self.0.add_shifted_saturated(
            velocity.raw_vec(),
            VELOCITY_TO_POSITION_SHIFT,
            Self::MIN_RAW,
            Self::MAX_RAW,
        ))
    }
}

impl core::ops::Sub for Position {
    type Output = DiffVec2;

    /// Returns the exact raw Q16 displacement `self - rhs`.
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        let [x, y] = self.raw();
        let [rhs_x, rhs_y] = rhs.raw();
        let dx = x as i64 - rhs_x as i64;
        let dy = y as i64 - rhs_y as i64;
        DiffVec2::from_raw_unchecked(dx as i32, dy as i32)
    }
}

#[inline(always)]
const fn is_valid_raw(value: i32) -> bool {
    value >= Position::MIN_RAW && value <= Position::MAX_RAW
}

#[inline(always)]
const fn is_valid_wide(value: i64) -> bool {
    value >= Position::MIN_RAW as i64 && value <= Position::MAX_RAW as i64
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
}
