use super::{ColliderFeature, Contact, ContactKey};
use crate::body::BodyId;
use crate::collider::Circle;
use crate::geometry::{GeometryPoint, UnitVector};
use crate::quantity::{Length, Position};
use crate::transform::Transform;

/// Computes a contact for circles at explicitly supplied world-space centers.
///
/// The circles' body-local centers are intentionally not applied by this
/// low-level helper. The normal points from A to B; coincident centers use +X
/// as the canonical direction.
#[inline]
pub fn collide(
    body_a: BodyId,
    circle_a: Circle,
    center_a: Position,
    body_b: BodyId,
    circle_b: Circle,
    center_b: Position,
) -> Option<Contact> {
    collide_at(
        body_a,
        circle_a,
        center_a.into(),
        body_b,
        circle_b,
        center_b.into(),
    )
}

pub(super) fn collide_transformed(
    body_a: BodyId,
    circle_a: Circle,
    transform_a: Transform,
    body_b: BodyId,
    circle_b: Circle,
    transform_b: Transform,
) -> Option<Contact> {
    let center_a = circle_a.transformed_center(transform_a);
    let center_b = circle_b.transformed_center(transform_b);
    collide_at(body_a, circle_a, center_a, body_b, circle_b, center_b)
}

fn collide_at(
    body_a: BodyId,
    circle_a: Circle,
    center_a: GeometryPoint,
    body_b: BodyId,
    circle_b: Circle,
    center_b: GeometryPoint,
) -> Option<Contact> {
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
    Some(Contact {
        body_a,
        body_b,
        point: center_a.offset(normal, contact_offset),
        normal,
        penetration: Length::from_raw(penetration_raw).expect("circle penetration must fit Length"),
        key: ContactKey::new(ColliderFeature::Circle, ColliderFeature::Circle),
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
        let large = Circle::new(Length::from_raw(Position::MAX_POSITION as u32).unwrap()).unwrap();
        let small = Circle::new(Length::from_raw(1).unwrap()).unwrap();
        let center = Position::from_i32(Position::MAX_POSITION, Position::MAX_POSITION);
        let contact =
            collide(BodyId::new(1), large, center, BodyId::new(2), small, center).unwrap();

        assert!(contact.point.raw()[0] > Position::MAX_POSITION);
        assert_eq!(contact.point.raw()[1], Position::MAX_POSITION);
    }

    #[test]
    fn maximum_penetration_fits_length() {
        let circle = Circle::new(Length::from_raw(Position::MAX_POSITION as u32).unwrap()).unwrap();
        let contact = collide(
            BodyId::new(1),
            circle,
            Position::ZERO,
            BodyId::new(2),
            circle,
            Position::ZERO,
        )
        .unwrap();

        assert_eq!(contact.penetration.raw(), 2 * Position::MAX_POSITION as u32);
    }

    #[test]
    fn transformed_collision_rotates_local_circle_center() {
        use crate::quantity::Angle;

        let offset = Circle::with_center(
            Position::from_meters(1.0, 0.0).unwrap(),
            Length::from_meters(0.5).unwrap(),
        )
        .unwrap();
        let centered = Circle::new(Length::from_meters(0.5).unwrap()).unwrap();

        let contact = collide_transformed(
            BodyId::new(1),
            offset,
            Transform::new(Position::ZERO, Angle::QUARTER_TURN),
            BodyId::new(2),
            centered,
            Transform::new(Position::from_meters(0.0, 2.0).unwrap(), Angle::ZERO),
        )
        .unwrap();

        assert_eq!(contact.penetration, Length::ZERO);
        assert_eq!(contact.point.to_meters(), [0.0, 1.5]);
    }
}
