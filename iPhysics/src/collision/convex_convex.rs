use super::Contact;
use super::sat::{BestAxis, select_axis, support};
use crate::body::BodyId;
use crate::collider::Convex;
use crate::quantity::Length;
use crate::transform::Transform;

pub(super) fn collide(
    body_a: BodyId,
    convex_a: Convex,
    transform_a: Transform,
    body_b: BodyId,
    convex_b: Convex,
    transform_b: Transform,
) -> Option<Contact> {
    let vertices_a = convex_a.transformed_vertices(transform_a);
    let vertices_b = convex_b.transformed_vertices(transform_b);
    let mut best: BestAxis = None;

    for normal in convex_a.normals() {
        select_axis(
            &mut best,
            normal.rotate(transform_a.angle),
            &vertices_a,
            &vertices_b,
        )?;
    }
    for normal in convex_b.normals() {
        select_axis(
            &mut best,
            normal.rotate(transform_b.angle),
            &vertices_a,
            &vertices_b,
        )?;
    }

    let (penetration, normal) = best?;
    let point_a = support(&vertices_a, normal, true);
    let point_b = support(&vertices_b, normal, false);
    Some(Contact {
        body_a,
        body_b,
        point: point_a.midpoint(point_b),
        normal,
        penetration: Length::from_raw(penetration),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::UnitVector;
    use crate::quantity::{Angle, Position};

    fn square(half_extent: f64) -> Convex {
        Convex::new(&[
            Position::from_meters(-half_extent, -half_extent).unwrap(),
            Position::from_meters(half_extent, -half_extent).unwrap(),
            Position::from_meters(half_extent, half_extent).unwrap(),
            Position::from_meters(-half_extent, half_extent).unwrap(),
        ])
        .unwrap()
    }

    #[test]
    fn projection_distances_orient_the_normal_from_a_to_b() {
        let contact = collide(
            BodyId::new(1),
            square(1.0),
            Transform::IDENTITY,
            BodyId::new(2),
            square(1.0),
            Transform::new(Position::from_meters(1.5, 0.0).unwrap(), Angle::ZERO),
        )
        .unwrap();

        assert_eq!(contact.normal, UnitVector::X);
        assert_eq!(contact.penetration.to_meters(), 0.5);
    }

    #[test]
    fn containment_uses_the_full_separation_distance() {
        let contact = collide(
            BodyId::new(1),
            square(0.5),
            Transform::IDENTITY,
            BodyId::new(2),
            square(2.0),
            Transform::IDENTITY,
        )
        .unwrap();

        assert_eq!(contact.penetration.to_meters(), 2.5);
    }
}
