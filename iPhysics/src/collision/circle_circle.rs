use super::Contact;
use crate::body::BodyId;
use crate::collider::Circle;
use crate::geometry::{GeometryPoint, UnitVector};
use crate::quantity::{Length, Position};

/// Computes a single deterministic circle-circle contact. The normal points
/// from A to B. Coincident centers use +X as the canonical direction.
#[inline]
pub fn collide(
    body_a: BodyId,
    circle_a: Circle,
    center_a: Position,
    body_b: BodyId,
    circle_b: Circle,
    center_b: Position,
) -> Option<Contact> {
    let [ax, ay] = center_a.raw();
    let delta = center_b - center_a;
    let distance_squared = delta.squared_magnitude();
    let radius_sum = circle_a.radius().raw() as u64 + circle_b.radius().raw() as u64;

    if distance_squared > radius_sum * radius_sum {
        return None;
    }

    let distance = distance_squared.isqrt();
    let normal = UnitVector::normalized_with_length(delta, distance).unwrap_or(UnitVector::X);

    let penetration = radius_sum - distance;
    let penetration_raw = penetration as u32;
    let contact_offset = circle_a.radius().raw() as i32 - (penetration_raw / 2) as i32;
    let [offset_x, offset_y] = normal.scaled_raw(contact_offset).raw();
    Some(Contact {
        body_a,
        body_b,
        point: GeometryPoint::from_wide_narrow(
            ax as i64 + offset_x as i64,
            ay as i64 + offset_y as i64,
        ),
        normal,
        penetration: Length::from_raw(penetration_raw),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tangent_circles_create_zero_penetration_contact() {
        let circle = Circle::new(Length::from_meters(1.0).unwrap()).unwrap();
        let contact = collide(
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
            collide(
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

    #[test]
    fn contained_circle_uses_signed_contact_offset() {
        let small = Circle::new(Length::from_meters(1.0).unwrap()).unwrap();
        let large = Circle::new(Length::from_meters(3.0).unwrap()).unwrap();
        let contact = collide(
            BodyId::new(1),
            small,
            Position::ZERO,
            BodyId::new(2),
            large,
            Position::ZERO,
        )
        .unwrap();

        assert_eq!(contact.point.to_meters(), [-1.0, 0.0]);
    }

    #[test]
    fn contact_point_can_extend_beyond_position_range() {
        let large = Circle::new(Length::from_raw(Position::MAX_RAW as u32)).unwrap();
        let small = Circle::new(Length::from_raw(1)).unwrap();
        let center = Position::from_raw(Position::MAX_RAW, Position::MAX_RAW);
        let contact =
            collide(BodyId::new(1), large, center, BodyId::new(2), small, center).unwrap();

        assert!(contact.point.raw()[0] > Position::MAX_RAW);
        assert_eq!(contact.point.raw()[1], Position::MAX_RAW);
    }
}
