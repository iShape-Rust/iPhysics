use super::{circle_circle, circle_convex, convex_convex, ContactManifold};
use crate::body::BodyId;
use crate::collider::{Collider, SimpleCollider, TransformedVertices};
use crate::transform::Transform;

pub(crate) struct CollisionSolver {
    a_vertices: TransformedVertices,
    b_vertices: TransformedVertices,
}

impl CollisionSolver {
    #[inline(always)]
    pub(crate) const fn new() -> Self {
        Self {
            a_vertices: TransformedVertices::new(),
            b_vertices: TransformedVertices::new(),
        }
    }

    pub(crate) fn collide(
        &mut self,
        body_a: BodyId,
        collider_a: &Collider,
        transform_a: Transform,
        body_b: BodyId,
        collider_b: &Collider,
        transform_b: Transform,
        mut emit: impl FnMut(ContactManifold),
    ) {
        if let (Collider::Circle(a), Collider::Circle(b)) = (collider_a, collider_b) {
            if let Some(contact) =
                circle_circle::collide_transformed(body_a, *a, transform_a, body_b, *b, transform_b)
            {
                emit(ContactManifold::one(contact));
            }
            return;
        }

        match (collider_a, collider_b) {
            (Collider::Composite(a), Collider::Composite(b)) => {
                for (part_a, &simple_a) in a.simple_colliders().iter().enumerate() {
                    let aabb_a = simple_a.aabb(transform_a);
                    for (part_b, &simple_b) in b.simple_colliders().iter().enumerate() {
                        if aabb_a.intersects(simple_b.aabb(transform_b)) {
                            self.collide_simple(
                                body_a,
                                simple_a,
                                transform_a,
                                body_b,
                                simple_b,
                                transform_b,
                                Some(part_a),
                                Some(part_b),
                                &mut emit,
                            );
                        }
                    }
                }
            }
            (Collider::Composite(a), b) => {
                let simple_b = Self::as_simple(b);
                let aabb_b = simple_b.aabb(transform_b);
                for (part_a, &simple_a) in a.simple_colliders().iter().enumerate() {
                    if simple_a.aabb(transform_a).intersects(aabb_b) {
                        self.collide_simple(
                            body_a,
                            simple_a,
                            transform_a,
                            body_b,
                            simple_b,
                            transform_b,
                            Some(part_a),
                            None,
                            &mut emit,
                        );
                    }
                }
            }
            (a, Collider::Composite(b)) => {
                let simple_a = Self::as_simple(a);
                let aabb_a = simple_a.aabb(transform_a);
                for (part_b, &simple_b) in b.simple_colliders().iter().enumerate() {
                    if aabb_a.intersects(simple_b.aabb(transform_b)) {
                        self.collide_simple(
                            body_a,
                            simple_a,
                            transform_a,
                            body_b,
                            simple_b,
                            transform_b,
                            None,
                            Some(part_b),
                            &mut emit,
                        );
                    }
                }
            }
            (a, b) => self.collide_simple(
                body_a,
                Self::as_simple(a),
                transform_a,
                body_b,
                Self::as_simple(b),
                transform_b,
                None,
                None,
                &mut emit,
            ),
        }
    }

    #[inline(always)]
    fn as_simple(collider: &Collider) -> SimpleCollider {
        match collider {
            Collider::Circle(circle) => SimpleCollider::Circle(*circle),
            Collider::Convex(convex) => SimpleCollider::Convex(*convex),
            Collider::Composite(_) => {
                unreachable!("composites are expanded before primitive dispatch")
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn collide_simple(
        &mut self,
        body_a: BodyId,
        collider_a: SimpleCollider,
        transform_a: Transform,
        body_b: BodyId,
        collider_b: SimpleCollider,
        transform_b: Transform,
        part_a: Option<usize>,
        part_b: Option<usize>,
        emit: &mut impl FnMut(ContactManifold),
    ) {
        let manifold = match (collider_a, collider_b) {
            (SimpleCollider::Circle(a), SimpleCollider::Circle(b)) => {
                circle_circle::collide_transformed(body_a, a, transform_a, body_b, b, transform_b)
                    .map(ContactManifold::one)
            }
            (SimpleCollider::Circle(circle), SimpleCollider::Convex(convex)) => {
                circle_convex::collide_with_scratch(
                    body_a,
                    circle,
                    transform_a,
                    body_b,
                    convex,
                    transform_b,
                    &mut self.a_vertices,
                )
                .map(ContactManifold::one)
            }
            (SimpleCollider::Convex(convex), SimpleCollider::Circle(circle)) => {
                circle_convex::collide_with_scratch(
                    body_b,
                    circle,
                    transform_b,
                    body_a,
                    convex,
                    transform_a,
                    &mut self.a_vertices,
                )
                .map(|contact| ContactManifold::one(contact.flipped()))
            }
            (SimpleCollider::Convex(a), SimpleCollider::Convex(b)) => {
                convex_convex::collide_with_scratch(
                    body_a,
                    a,
                    transform_a,
                    body_b,
                    b,
                    transform_b,
                    &mut self.a_vertices,
                    &mut self.b_vertices,
                )
            }
        };

        if let Some(manifold) = manifold {
            emit(manifold.with_parts(part_a, part_b));
        }
    }
}

#[cfg(test)]
#[allow(clippy::too_many_arguments)]
fn collide(
    body_a: BodyId,
    collider_a: &Collider,
    transform_a: Transform,
    body_b: BodyId,
    collider_b: &Collider,
    transform_b: Transform,
    emit: impl FnMut(ContactManifold),
) {
    CollisionSolver::new().collide(
        body_a,
        collider_a,
        transform_a,
        body_b,
        collider_b,
        transform_b,
        emit,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider::{Circle, CompositeCollider, Convex};
    use crate::collision::ColliderFeature;
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
    fn circle_and_convex_generate_topological_features() {
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
        assert_eq!(contact.feature_a, ColliderFeature::Circle);
        assert!(matches!(
            contact.feature_b,
            ColliderFeature::ConvexVertex(_) | ColliderFeature::ConvexEdge(_)
        ));
    }

    #[test]
    fn convex_manifold_features_are_stable_under_translation() {
        let contacts = |offset: f64| {
            let mut result = alloc::vec::Vec::new();
            collide(
                BodyId::new(1),
                &square().into(),
                Transform::new(Position::from_meters(offset, -3.0).unwrap(), Angle::ZERO),
                BodyId::new(2),
                &square().into(),
                Transform::new(
                    Position::from_meters(offset + 2.0, -3.0).unwrap(),
                    Angle::ZERO,
                ),
                |manifold| {
                    result.extend(
                        manifold
                            .into_contacts()
                            .map(|contact| (contact.feature_a, contact.feature_b)),
                    )
                },
            );
            result
        };

        let first = contacts(0.0);
        let shifted = contacts(7.0);
        assert_eq!(first, shifted);
        assert_eq!(first.len(), 2);
        assert_ne!(first[0], first[1]);
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
        let mut parts = alloc::vec::Vec::new();

        collide(
            BodyId::new(1),
            &a,
            Transform::IDENTITY,
            BodyId::new(2),
            &b,
            Transform::IDENTITY,
            |manifold| {
                let contact = manifold.first();
                parts.push((contact.part_a, contact.part_b));
            },
        );

        assert_eq!(parts, [(Some(0), Some(0)), (Some(1), Some(1))]);
    }
}
