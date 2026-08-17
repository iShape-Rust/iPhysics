use super::Contact;
use crate::body::BodyId;
use crate::collider::Circle;
use crate::geometry::UnitVector;
use crate::quantity::{Length, Position};

/// Computes a single deterministic circle-circle contact. The normal points
/// from A to B. Coincident centers use +X as the canonical direction.
#[inline]
pub fn collide_circles(
    body_a: BodyId,
    circle_a: Circle,
    center_a: Position,
    body_b: BodyId,
    circle_b: Circle,
    center_b: Position,
) -> Option<Contact> {
    let [ax, ay] = center_a.raw();
    let delta = center_b - center_a;
    let [dx, dy] = delta.raw();
    let distance_squared = delta.squared_magnitude();
    let radius_sum = circle_a.radius().raw() as u64 + circle_b.radius().raw() as u64;

    if distance_squared > radius_sum as u128 * radius_sum as u128 {
        return None;
    }

    let distance = integer_sqrt(distance_squared);
    let [nx, ny] = if distance == 0 {
        UnitVector::X.raw()
    } else {
        let scale = 1_i128 << UnitVector::FRACTION_BITS;
        [
            ((dx as i128 * scale) / distance as i128) as i32,
            ((dy as i128 * scale) / distance as i128) as i32,
        ]
    };

    let penetration = radius_sum as u128 - distance;
    let penetration_raw = u32::try_from(penetration).ok()?;
    let contact_offset = circle_a.radius().raw() as i64 - (penetration as i64 >> 1);
    let point_x = ax as i64 + round_shift(nx as i128 * contact_offset as i128, 30) as i64;
    let point_y = ay as i64 + round_shift(ny as i128 * contact_offset as i128, 30) as i64;

    Some(Contact {
        body_a,
        body_b,
        point: Position::from_raw(i32::try_from(point_x).ok()?, i32::try_from(point_y).ok()?),
        normal: UnitVector::from_raw(nx, ny),
        penetration: Length::from_raw(penetration_raw),
    })
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

/// Floor square root with identical results on every target.
fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }

    let mut x = 1_u128 << ((128 - value.leading_zeros() as u128 + 1) >> 1);
    loop {
        let next = (x + value / x) >> 1;
        if next >= x {
            return x;
        }
        x = next;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tangent_circles_create_zero_penetration_contact() {
        let circle = Circle::new(Length::from_meters(1.0).unwrap()).unwrap();
        let contact = collide_circles(
            BodyId::new(1),
            circle,
            Position::ZERO,
            BodyId::new(2),
            circle,
            Position::from_meters(2.0, 0.0).unwrap(),
        )
        .unwrap();

        assert_eq!(contact.normal, UnitVector::X);
        assert_eq!(contact.penetration, Length::ZERO);
        assert_eq!(contact.point.to_meters(), [1.0, 0.0]);
    }

    #[test]
    fn separated_circles_do_not_collide() {
        let circle = Circle::new(Length::from_meters(1.0).unwrap()).unwrap();
        assert!(
            collide_circles(
                BodyId::new(1),
                circle,
                Position::ZERO,
                BodyId::new(2),
                circle,
                Position::from_meters(2.01, 0.0).unwrap(),
            )
            .is_none()
        );
    }
}
