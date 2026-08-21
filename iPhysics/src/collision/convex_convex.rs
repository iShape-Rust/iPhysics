use super::Contact;
use crate::body::BodyId;
use crate::collider::Convex;
use crate::geometry::{GeometryPoint, UnitVector};
use crate::quantity::Length;
use crate::transform::Transform;

type BestAxis = Option<(u32, UnitVector)>;

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

fn select_axis(
    best: &mut BestAxis,
    axis: UnitVector,
    a: &[GeometryPoint],
    b: &[GeometryPoint],
) -> Option<()> {
    let (min_a, max_a) = project(a, axis);
    let (min_b, max_b) = project(b, axis);
    let move_a_negative = max_a - min_b;
    let move_a_positive = max_b - min_a;
    if move_a_negative < 0 || move_a_positive < 0 {
        return None;
    }

    if move_a_negative <= move_a_positive {
        update_best(best, move_a_negative, axis);
    } else {
        update_best(best, move_a_positive, -axis);
    }
    Some(())
}

#[inline]
fn update_best(best: &mut BestAxis, penetration: i64, axis: UnitVector) {
    let penetration = penetration as u32;
    if best.is_none_or(|(current, _)| penetration < current) {
        *best = Some((penetration, axis));
    }
}

fn project(vertices: &[GeometryPoint], axis: UnitVector) -> (i64, i64) {
    let first = axis.dot(vertices[0].into());
    let mut min = first;
    let mut max = first;
    for &vertex in &vertices[1..] {
        let projection = axis.dot(vertex.into());
        min = min.min(projection);
        max = max.max(projection);
    }
    (min, max)
}

fn support(vertices: &[GeometryPoint], axis: UnitVector, maximum: bool) -> GeometryPoint {
    let mut result = vertices[0];
    let mut best = axis.dot(result.into());
    for &vertex in &vertices[1..] {
        let projection = axis.dot(vertex.into());
        if (maximum && projection > best) || (!maximum && projection < best) {
            result = vertex;
            best = projection;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
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
