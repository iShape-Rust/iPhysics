use super::{ContactManifold, circle_circle, circle_convex, convex_convex};
use crate::body::BodyId;
use crate::collider::{Collider, SimpleCollider};
use crate::transform::Transform;

pub(crate) fn collide(
    body_a: BodyId,
    collider_a: &Collider,
    transform_a: Transform,
    body_b: BodyId,
    collider_b: &Collider,
    transform_b: Transform,
    mut emit: impl FnMut(ContactManifold),
) {
    match (collider_a, collider_b) {
        (Collider::Composite(a), Collider::Composite(b)) => {
            for &simple_a in a.simple_colliders() {
                let aabb_a = simple_a.aabb(transform_a);
                for &simple_b in b.simple_colliders() {
                    if aabb_a.intersects(simple_b.aabb(transform_b)) {
                        collide_simple(
                            body_a,
                            simple_a,
                            transform_a,
                            body_b,
                            simple_b,
                            transform_b,
                            &mut emit,
                        );
                    }
                }
            }
        }
        (Collider::Composite(a), b) => {
            let simple_b = as_simple(b);
            let aabb_b = simple_b.aabb(transform_b);
            for &simple_a in a.simple_colliders() {
                if simple_a.aabb(transform_a).intersects(aabb_b) {
                    collide_simple(
                        body_a,
                        simple_a,
                        transform_a,
                        body_b,
                        simple_b,
                        transform_b,
                        &mut emit,
                    );
                }
            }
        }
        (a, Collider::Composite(b)) => {
            let simple_a = as_simple(a);
            let aabb_a = simple_a.aabb(transform_a);
            for &simple_b in b.simple_colliders() {
                if aabb_a.intersects(simple_b.aabb(transform_b)) {
                    collide_simple(
                        body_a,
                        simple_a,
                        transform_a,
                        body_b,
                        simple_b,
                        transform_b,
                        &mut emit,
                    );
                }
            }
        }
        (a, b) => collide_simple(
            body_a,
            as_simple(a),
            transform_a,
            body_b,
            as_simple(b),
            transform_b,
            &mut emit,
        ),
    }
}

#[inline(always)]
fn as_simple(collider: &Collider) -> SimpleCollider {
    match collider {
        Collider::Circle(circle) => SimpleCollider::Circle(*circle),
        Collider::Convex(convex) => SimpleCollider::Convex(*convex),
        Collider::Composite(_) => unreachable!("composites are expanded before primitive dispatch"),
    }
}

#[allow(clippy::too_many_arguments)]
fn collide_simple(
    body_a: BodyId,
    collider_a: SimpleCollider,
    transform_a: Transform,
    body_b: BodyId,
    collider_b: SimpleCollider,
    transform_b: Transform,
    emit: &mut impl FnMut(ContactManifold),
) {
    let manifold = match (collider_a, collider_b) {
        (SimpleCollider::Circle(a), SimpleCollider::Circle(b)) => {
            circle_circle::collide_transformed(body_a, a, transform_a, body_b, b, transform_b)
                .map(ContactManifold::one)
        }
        (SimpleCollider::Circle(circle), SimpleCollider::Convex(convex)) => {
            circle_convex::collide(body_a, circle, transform_a, body_b, convex, transform_b)
                .map(ContactManifold::one)
        }
        (SimpleCollider::Convex(convex), SimpleCollider::Circle(circle)) => {
            circle_convex::collide(body_b, circle, transform_b, body_a, convex, transform_a)
                .map(|contact| ContactManifold::one(contact.flipped()))
        }
        (SimpleCollider::Convex(a), SimpleCollider::Convex(b)) => {
            convex_convex::collide(body_a, a, transform_a, body_b, b, transform_b)
        }
    };

    if let Some(manifold) = manifold {
        emit(manifold);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider::{Circle, CompositeCollider, Convex};
    use crate::quantity::{Angle, Length, Position};
    use alloc::vec;

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
        let mut contact = None;
        collide(
            BodyId::new(1),
            &Circle::new(Length::from_meters(0.5).unwrap())
                .unwrap()
                .into(),
            Transform::new(Position::from_meters(1.25, 0.0).unwrap(), Angle::ZERO),
            BodyId::new(2),
            &square().into(),
            Transform::IDENTITY,
            |manifold| contact = Some(manifold.first()),
        );
        let contact = contact.unwrap();

        assert_eq!(contact.body_a, BodyId::new(1));
        assert_eq!(contact.body_b, BodyId::new(2));
        assert!(contact.penetration.raw() > 0);
    }

    #[test]
    fn separated_convexes_do_not_collide() {
        let mut collided = false;
        collide(
            BodyId::new(1),
            &square().into(),
            Transform::IDENTITY,
            BodyId::new(2),
            &square().into(),
            Transform::new(Position::from_meters(3.0, 0.0).unwrap(), Angle::ZERO),
            |_| collided = true,
        );
        assert!(!collided);
    }

    #[test]
    fn composite_pair_emits_every_primitive_manifold() {
        let radius = Length::from_meters(0.5).unwrap();
        let composite = |y: f64| {
            CompositeCollider::new(vec![
                Circle::with_center(Position::from_meters(-1.0, y).unwrap(), radius)
                    .unwrap()
                    .into(),
                Circle::with_center(Position::from_meters(1.0, y).unwrap(), radius)
                    .unwrap()
                    .into(),
            ])
        };
        let a: Collider = composite(0.0).into();
        let b: Collider = composite(0.75).into();
        let mut manifolds = 0;

        collide(
            BodyId::new(1),
            &a,
            Transform::IDENTITY,
            BodyId::new(2),
            &b,
            Transform::IDENTITY,
            |_| manifolds += 1,
        );

        assert_eq!(manifolds, 2);
    }
}
