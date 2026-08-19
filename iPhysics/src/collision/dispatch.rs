use super::{Contact, circle_circle, circle_convex, convex_convex};
use crate::body::BodyId;
use crate::collider::Collider;
use crate::transform::Transform;

pub(crate) fn collide(
    body_a: BodyId,
    collider_a: Collider,
    transform_a: Transform,
    body_b: BodyId,
    collider_b: Collider,
    transform_b: Transform,
) -> Option<Contact> {
    match (collider_a, collider_b) {
        (Collider::Circle(a), Collider::Circle(b)) => circle_circle::collide(
            body_a,
            a,
            transform_a.position,
            body_b,
            b,
            transform_b.position,
        ),
        (Collider::Circle(circle), Collider::Convex(convex)) => {
            circle_convex::collide(body_a, circle, transform_a, body_b, convex, transform_b)
        }
        (Collider::Convex(convex), Collider::Circle(circle)) => {
            circle_convex::collide(body_b, circle, transform_b, body_a, convex, transform_a)
                .map(Contact::flipped)
        }
        (Collider::Convex(a), Collider::Convex(b)) => {
            convex_convex::collide(body_a, a, transform_a, body_b, b, transform_b)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider::{Circle, Convex};
    use crate::quantity::{Angle, Length, Position};

    fn square() -> Convex {
        Convex::new(&[
            Position::from_i32(-65_536, -65_536),
            Position::from_i32(65_536, -65_536),
            Position::from_i32(65_536, 65_536),
            Position::from_i32(-65_536, 65_536),
        ])
        .unwrap()
    }

    #[test]
    fn circle_and_convex_generate_contact_without_shape_identity() {
        let contact = collide(
            BodyId::new(1),
            Circle::new(Length::from_meters(0.5).unwrap())
                .unwrap()
                .into(),
            Transform::new(Position::from_meters(1.25, 0.0).unwrap(), Angle::ZERO),
            BodyId::new(2),
            square().into(),
            Transform::IDENTITY,
        )
        .unwrap();

        assert_eq!(contact.body_a, BodyId::new(1));
        assert_eq!(contact.body_b, BodyId::new(2));
        assert!(contact.penetration.raw() > 0);
    }

    #[test]
    fn separated_convexes_do_not_collide() {
        assert!(
            collide(
                BodyId::new(1),
                square().into(),
                Transform::IDENTITY,
                BodyId::new(2),
                square().into(),
                Transform::new(Position::from_meters(3.0, 0.0).unwrap(), Angle::ZERO),
            )
            .is_none()
        );
    }
}
