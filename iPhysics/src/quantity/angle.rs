use super::angular_velocity::AngularVelocity;

const ANGLE_SCALE: f64 = (1_u64 << 32) as f64;
const TAU: f64 = core::f64::consts::TAU;
// The ideal inverse CORDIC gain rounded to Q30 is 652_032_874. Integer shifts
// in the iterations can expand the resulting unit vector by a few Q30 units.
// A 128-unit guard makes every non-cardinal rotation conservatively
// non-expanding, which lets collider-radius invariants survive rotation.
const CORDIC_GAIN_Q30: i64 = 652_032_746;
const CORDIC_ANGLES: [i64; 31] = [
    536_870_912,
    316_933_406,
    167_458_907,
    85_004_756,
    42_667_331,
    21_354_465,
    10_679_838,
    5_340_245,
    2_670_163,
    1_335_087,
    667_544,
    333_772,
    166_886,
    83_443,
    41_722,
    20_861,
    10_430,
    5_215,
    2_608,
    1_304,
    652,
    326,
    163,
    81,
    41,
    20,
    10,
    5,
    3,
    1,
    1,
];

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

    /// Returns sine and cosine as signed Q30 values.
    #[inline]
    pub(crate) fn sin_cos_q30(self) -> [i32; 2] {
        match self.0 {
            0 => return [0, 1 << 30],
            0x4000_0000 => return [1 << 30, 0],
            0x8000_0000 => return [0, -(1 << 30)],
            0xc000_0000 => return [-(1 << 30), 0],
            _ => {}
        }

        let mut z = self.0 as i32 as i64;
        let mut sign = 1_i64;
        if z > 1_i64 << 30 {
            z -= 1_i64 << 31;
            sign = -1;
        } else if z < -(1_i64 << 30) {
            z += 1_i64 << 31;
            sign = -1;
        }

        let mut cos = CORDIC_GAIN_Q30;
        let mut sin = 0_i64;
        for (shift, angle) in CORDIC_ANGLES.into_iter().enumerate() {
            let old_cos = cos;
            if z >= 0 {
                cos -= sin >> shift;
                sin += old_cos >> shift;
                z -= angle;
            } else {
                cos += sin >> shift;
                sin -= old_cos >> shift;
                z += angle;
            }
        }

        debug_assert!(sin * sin + cos * cos <= 1_i64 << 60);
        [(sin * sign) as i32, (cos * sign) as i32]
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q30_sin_cos_uses_conventional_order() {
        assert_eq!(Angle::ZERO.sin_cos_q30(), [0, 1 << 30]);
        assert_eq!(Angle::QUARTER_TURN.sin_cos_q30(), [1 << 30, 0]);
        assert_eq!(Angle::HALF_TURN.sin_cos_q30(), [0, -(1 << 30)]);
    }

    #[test]
    fn cordic_rotation_is_non_expanding() {
        let scale_squared = 1_i64 << 60;
        for raw in (0..=u32::MAX).step_by(65_537) {
            let [sin, cos] = Angle::from_raw(raw).sin_cos_q30();
            assert!(sin as i64 * sin as i64 + cos as i64 * cos as i64 <= scale_squared);
        }
    }
}
