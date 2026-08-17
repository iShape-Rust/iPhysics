use crate::quantity::{Angle, AngularVelocity, LinearVelocity, Position};

/// Position and orientation of a body in world space.
///
/// Rotation coefficients are deliberately not cached here: `Transform` stays
/// compact and contains only authoritative rollback state. A deterministic
/// rotator will be derived when transformed collider geometry is introduced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Transform {
    pub position: Position,
    pub angle: Angle,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        position: Position::ZERO,
        angle: Angle::ZERO,
    };

    #[inline(always)]
    pub const fn new(position: Position, angle: Angle) -> Self {
        Self { position, angle }
    }

    /// Transforms a Q16 point from local space into world space.
    ///
    /// Rotation uses integer CORDIC coefficients, so the result is identical
    /// on every target and does not depend on a platform floating-point
    /// implementation.
    #[inline]
    pub fn checked_apply(self, local: Position) -> Option<Position> {
        let [rx, ry] = rotate_fixed(local.raw(), self.angle)?;
        let [tx, ty] = self.position.raw();
        Some(Position::from_raw(
            i32::try_from(tx as i64 + rx as i64).ok()?,
            i32::try_from(ty as i64 + ry as i64).ok()?,
        ))
    }

    /// Composes a child-local transform with this parent transform.
    #[inline]
    pub fn checked_compose(self, local: Self) -> Option<Self> {
        Some(Self {
            position: self.checked_apply(local.position)?,
            angle: Angle::from_raw(self.angle.raw().wrapping_add(local.angle.raw())),
        })
    }

    /// Advances the transform by one fixed 64 Hz tick.
    #[inline]
    pub fn checked_advance(
        self,
        linear_velocity: LinearVelocity,
        angular_velocity: AngularVelocity,
    ) -> Option<Self> {
        Some(Self {
            position: self.position.checked_advance(linear_velocity)?,
            angle: self.angle.advance(angular_velocity),
        })
    }
}

const CORDIC_GAIN_Q30: i64 = 652_032_874;
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

/// Returns cosine and sine as signed Q30 values.
pub(crate) fn sin_cos_q30(angle: Angle) -> [i32; 2] {
    match angle.raw() {
        0 => return [1 << 30, 0],
        0x4000_0000 => return [0, 1 << 30],
        0x8000_0000 => return [-(1 << 30), 0],
        0xc000_0000 => return [0, -(1 << 30)],
        _ => {}
    }

    let mut z = angle.raw() as i32 as i64;
    let mut sign = 1_i64;
    if z > 1_i64 << 30 {
        z -= 1_i64 << 31;
        sign = -1;
    } else if z < -(1_i64 << 30) {
        z += 1_i64 << 31;
        sign = -1;
    }

    let mut x = CORDIC_GAIN_Q30;
    let mut y = 0_i64;
    for (shift, angle) in CORDIC_ANGLES.into_iter().enumerate() {
        let old_x = x;
        if z >= 0 {
            x -= y >> shift;
            y += old_x >> shift;
            z -= angle;
        } else {
            x += y >> shift;
            y -= old_x >> shift;
            z += angle;
        }
    }

    [(x * sign) as i32, (y * sign) as i32]
}

#[inline]
pub(crate) fn rotate_fixed(point: [i32; 2], angle: Angle) -> Option<[i32; 2]> {
    let [cos, sin] = sin_cos_q30(angle);
    let x = round_shift(
        point[0] as i128 * cos as i128 - point[1] as i128 * sin as i128,
        30,
    );
    let y = round_shift(
        point[0] as i128 * sin as i128 + point[1] as i128 * cos as i128,
        30,
    );
    Some([i32::try_from(x).ok()?, i32::try_from(y).ok()?])
}

#[inline(always)]
fn round_shift(value: i128, shift: u32) -> i128 {
    let half = 1_i128 << (shift - 1);
    if value < 0 {
        -((-value + half) >> shift)
    } else {
        (value + half) >> shift
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn advances_both_position_and_angle() {
        let transform = Transform::IDENTITY;
        let linear = LinearVelocity::from_meters_per_second(64.0, -32.0).unwrap();
        let angular = AngularVelocity::from_radians_per_second(core::f64::consts::PI).unwrap();
        let next = transform.checked_advance(linear, angular).unwrap();

        assert_eq!(next.position.to_meters(), [1.0, -0.5]);
        assert_ne!(next.angle, Angle::ZERO);
    }

    #[test]
    fn applies_cardinal_rotation_exactly() {
        let transform = Transform::new(Position::from_raw(100, 200), Angle::QUARTER_TURN);

        assert_eq!(
            transform.checked_apply(Position::from_raw(30, 10)),
            Some(Position::from_raw(90, 230))
        );
    }
}
