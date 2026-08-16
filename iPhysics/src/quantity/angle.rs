use super::angular_velocity::AngularVelocity;

const ANGLE_SCALE: f64 = (1_u64 << 32) as f64;
const TAU: f64 = core::f64::consts::TAU;

/// Orientation stored as an unsigned binary angle.
///
/// One full turn is exactly `2^32` raw units, so `u32` wrapping is also angle
/// normalization. Important values are therefore exact powers of two:
///
/// - `0x0000_0000`: 0 degrees;
/// - `0x4000_0000`: 90 degrees;
/// - `0x8000_0000`: 180 degrees;
/// - `0xc000_0000`: 270 degrees.
///
/// Resolution is `2π / 2^32`, approximately `1.46e-9 rad`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Angle(u32);

impl Angle {
    pub const ZERO: Self = Self(0);
    pub const QUARTER_TURN: Self = Self(1 << 30);
    pub const HALF_TURN: Self = Self(1 << 31);
    pub const THREE_QUARTER_TURN: Self = Self(3 << 30);

    #[inline(always)]
    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    /// Converts radians to a wrapped binary angle.
    ///
    /// Floating-point conversion is intended for API and asset boundaries, not
    /// for calculations performed during a simulation step.
    #[inline]
    pub fn from_radians(radians: f64) -> Option<Self> {
        if !radians.is_finite() {
            return None;
        }

        let scaled = (radians % TAU) * (ANGLE_SCALE / TAU);
        let truncated = scaled as i64;
        let fraction = scaled - truncated as f64;
        let rounded = if fraction >= 0.5 {
            truncated + 1
        } else if fraction <= -0.5 {
            truncated - 1
        } else {
            truncated
        };

        Some(Self(rounded as u32))
    }

    #[inline(always)]
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Returns the angle in the range `[0, 2π)`.
    #[inline(always)]
    pub fn to_radians(self) -> f64 {
        self.0 as f64 * (TAU / ANGLE_SCALE)
    }

    /// Returns the equivalent angle in the range `[-π, π)`.
    #[inline(always)]
    pub fn to_signed_radians(self) -> f64 {
        self.0 as i32 as f64 * (TAU / ANGLE_SCALE)
    }

    /// Adds a signed angle delta. Overflow performs the required full-turn wrap.
    #[inline(always)]
    pub const fn wrapping_add(self, delta: AngleDelta) -> Self {
        Self(self.0.wrapping_add(delta.0 as u32))
    }

    /// Returns the shortest signed delta from `self` to `target`.
    #[inline(always)]
    pub const fn delta_to(self, target: Self) -> AngleDelta {
        AngleDelta(target.0.wrapping_sub(self.0) as i32)
    }

    /// Advances the angle by one 64 Hz tick.
    #[inline(always)]
    pub fn advance(self, velocity: AngularVelocity) -> Self {
        self.wrapping_add(velocity.angle_delta_per_tick())
    }
}

/// Signed binary angle difference in the range of one half-turn.
///
/// The raw range maps to `[-π, π)`, with the same approximately `1.46e-9 rad`
/// resolution as [`Angle`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AngleDelta(i32);

impl AngleDelta {
    pub const ZERO: Self = Self(0);

    #[inline(always)]
    pub const fn from_raw(raw: i32) -> Self {
        Self(raw)
    }

    #[inline(always)]
    pub const fn raw(self) -> i32 {
        self.0
    }

    #[inline(always)]
    pub fn to_radians(self) -> f64 {
        self.0 as f64 * (TAU / ANGLE_SCALE)
    }
}
