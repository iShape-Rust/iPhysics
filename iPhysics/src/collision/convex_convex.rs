use super::{ColliderFeature, Contact, ContactKey, ContactManifold};
use crate::body::BodyId;
use crate::collider::{Convex, TransformedVertices};
use crate::geometry::{GeometryPoint, UnitVector};
use crate::ops::div::DivRound;
use crate::quantity::{Angle, Length};
use crate::transform::Transform;

#[derive(Debug, Clone, Copy)]
enum AxisSource {
    A,
    B,
}

type BestAxis = Option<(u32, UnitVector, AxisSource)>;
const MANIFOLD_SLOP_RAW: i64 = 64; // 1/1024 m in Q16

#[cfg(test)]
pub(super) fn collide(
    body_a: BodyId,
    convex_a: Convex,
    transform_a: Transform,
    body_b: BodyId,
    convex_b: Convex,
    transform_b: Transform,
) -> Option<ContactManifold> {
    let mut vertices_a = TransformedVertices::new();
    let mut vertices_b = TransformedVertices::new();
    collide_with_scratch(
        body_a,
        convex_a,
        transform_a,
        body_b,
        convex_b,
        transform_b,
        &mut vertices_a,
        &mut vertices_b,
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn collide_with_scratch(
    body_a: BodyId,
    convex_a: Convex,
    transform_a: Transform,
    body_b: BodyId,
    convex_b: Convex,
    transform_b: Transform,
    vertices_a: &mut TransformedVertices,
    vertices_b: &mut TransformedVertices,
) -> Option<ContactManifold> {
    convex_a.write_transformed_vertices(transform_a, vertices_a);
    convex_b.write_transformed_vertices(transform_b, vertices_b);
    let mut best: BestAxis = None;

    for normal in convex_a.normals() {
        select_axis(
            &mut best,
            normal.rotate(transform_a.angle),
            AxisSource::A,
            vertices_a,
            vertices_b,
        )?;
    }
    for normal in convex_b.normals() {
        select_axis(
            &mut best,
            normal.rotate(transform_b.angle),
            AxisSource::B,
            vertices_a,
            vertices_b,
        )?;
    }

    let (penetration, normal, source) = best?;
    let tangent = normal.perpendicular();
    let (
        reference_vertices,
        reference_normals,
        reference_angle,
        incident_vertices,
        incident_normals,
        incident_angle,
        reference_outward,
    ) = match source {
        AxisSource::A => (
            &*vertices_a,
            convex_a.normals(),
            transform_a.angle,
            &*vertices_b,
            convex_b.normals(),
            transform_b.angle,
            normal,
        ),
        AxisSource::B => (
            &*vertices_b,
            convex_b.normals(),
            transform_b.angle,
            &*vertices_a,
            convex_a.normals(),
            transform_a.angle,
            -normal,
        ),
    };
    let reference_edge = supporting_edge(
        reference_vertices,
        reference_normals,
        reference_angle,
        reference_outward,
        true,
    );
    let incident_edge = supporting_edge(
        incident_vertices,
        incident_normals,
        incident_angle,
        reference_outward,
        false,
    );
    let reference_interval = segment_interval(reference_edge, tangent);
    let incident_interval = segment_interval(incident_edge, tangent);
    let tangent_min = reference_interval.0.max(incident_interval.0);
    let tangent_max = reference_interval.1.min(incident_interval.1);
    let contact = |candidate: ContactCandidate| {
        let (feature_a, feature_b) = match source {
            AxisSource::A => (candidate.reference_feature, candidate.incident_feature),
            AxisSource::B => (candidate.incident_feature, candidate.reference_feature),
        };
        Contact {
            body_a,
            body_b,
            point: candidate.point,
            normal,
            penetration: Length::from_raw(penetration).expect("convex penetration must fit Length"),
            key: ContactKey::new(feature_a, feature_b),
        }
    };

    // Intersection implies overlap on every projection axis. Fixed-point
    // rounding can still invert the interval by one raw unit, so collapse
    // that degenerate case to a single interpolated point.
    if tangent_min > tangent_max {
        let tangent_projection = (tangent_min + tangent_max) / 2;
        let candidate = contact_candidate(
            reference_edge,
            incident_edge,
            tangent,
            reference_outward,
            tangent_projection,
        );
        return Some(ContactManifold::one(contact(candidate)));
    }

    let first = contact_candidate(
        reference_edge,
        incident_edge,
        tangent,
        reference_outward,
        tangent_min,
    );
    if tangent_min == tangent_max {
        return Some(ContactManifold::one(contact(first)));
    }
    let second = contact_candidate(
        reference_edge,
        incident_edge,
        tangent,
        reference_outward,
        tangent_max,
    );
    if first.separation <= MANIFOLD_SLOP_RAW && second.separation <= MANIFOLD_SLOP_RAW {
        Some(ContactManifold::two(contact(first), contact(second)))
    } else if first.separation <= second.separation {
        Some(ContactManifold::one(contact(first)))
    } else {
        Some(ContactManifold::one(contact(second)))
    }
}

fn select_axis(
    best: &mut BestAxis,
    axis: UnitVector,
    source: AxisSource,
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
        update_best(best, move_a_negative, axis, source);
    } else {
        update_best(best, move_a_positive, -axis, source);
    }
    Some(())
}

#[inline]
fn update_best(best: &mut BestAxis, penetration: i64, axis: UnitVector, source: AxisSource) {
    let penetration = penetration as u32;
    if best.is_none_or(|(current, _, _)| penetration < current) {
        *best = Some((penetration, axis, source));
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

#[derive(Debug, Clone, Copy)]
struct ContactCandidate {
    point: GeometryPoint,
    separation: i64,
    reference_feature: ColliderFeature,
    incident_feature: ColliderFeature,
}

#[derive(Debug, Clone, Copy)]
struct SupportingEdge {
    points: [GeometryPoint; 2],
    edge_index: u8,
    next_index: u8,
}

fn supporting_edge(
    vertices: &[GeometryPoint],
    normals: impl Iterator<Item = UnitVector>,
    angle: Angle,
    reference_outward: UnitVector,
    most_aligned: bool,
) -> SupportingEdge {
    let score = |normal: UnitVector| {
        let [ax, ay] = reference_outward.raw();
        let [bx, by] = normal.rotate(angle).raw();
        ax as i64 * bx as i64 + ay as i64 * by as i64
    };
    let mut normals = normals.enumerate();
    let (_, first) = normals.next().expect("a convex has at least three edges");
    let mut edge = 0;
    let mut best = score(first);
    for (index, normal) in normals {
        let candidate = score(normal);
        if (most_aligned && candidate > best) || (!most_aligned && candidate < best) {
            edge = index;
            best = candidate;
        }
    }

    let next = (edge + 1) % vertices.len();
    SupportingEdge {
        points: [vertices[edge], vertices[next]],
        edge_index: edge as u8,
        next_index: next as u8,
    }
}

#[inline(always)]
fn segment_interval(segment: SupportingEdge, axis: UnitVector) -> (i64, i64) {
    let a = axis.dot(segment.points[0].into());
    let b = axis.dot(segment.points[1].into());
    (a.min(b), a.max(b))
}

fn contact_candidate(
    reference: SupportingEdge,
    incident: SupportingEdge,
    tangent: UnitVector,
    reference_outward: UnitVector,
    tangent_projection: i64,
) -> ContactCandidate {
    let reference_point = point_on_segment(reference.points, tangent, tangent_projection);
    let incident_point = point_on_segment(incident.points, tangent, tangent_projection);
    ContactCandidate {
        point: reference_point.midpoint(incident_point),
        separation: reference_outward.dot(incident_point - reference_point),
        reference_feature: segment_feature(reference, tangent, tangent_projection),
        incident_feature: segment_feature(incident, tangent, tangent_projection),
    }
}

fn segment_feature(segment: SupportingEdge, axis: UnitVector, projection: i64) -> ColliderFeature {
    let projection_a = axis.dot(segment.points[0].into());
    let projection_b = axis.dot(segment.points[1].into());
    if projection_a != projection_b && projection == projection_a {
        ColliderFeature::ConvexVertex(segment.edge_index)
    } else if projection_a != projection_b && projection == projection_b {
        ColliderFeature::ConvexVertex(segment.next_index)
    } else {
        ColliderFeature::ConvexEdge(segment.edge_index)
    }
}

fn point_on_segment(
    segment: [GeometryPoint; 2],
    axis: UnitVector,
    projection: i64,
) -> GeometryPoint {
    let projection_a = axis.dot(segment[0].into());
    let projection_b = axis.dot(segment[1].into());
    if projection == projection_a || projection_a == projection_b {
        return segment[0];
    }
    if projection == projection_b {
        return segment[1];
    }

    let numerator = projection - projection_a;
    let denominator = projection_b - projection_a;
    let [ax, ay] = segment[0].raw();
    let [bx, by] = segment[1].raw();
    let x = ax as i64
        + ((bx as i64 - ax as i64) as i128 * numerator as i128).div_round(denominator as i128)
            as i64;
    let y = ay as i64
        + ((by as i64 - ay as i64) as i128 * numerator as i128).div_round(denominator as i128)
            as i64;
    GeometryPoint::from_i64_unchecked(x, y)
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
        .unwrap()
        .first();

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
        .unwrap()
        .first();

        assert_eq!(contact.penetration.to_meters(), 2.5);
    }

    #[test]
    fn overlapping_faces_keep_both_ends_of_the_clipped_interval() {
        let floor = Convex::new(&[
            Position::from_meters(-5.0, -0.2).unwrap(),
            Position::from_meters(5.0, -0.2).unwrap(),
            Position::from_meters(5.0, 0.2).unwrap(),
            Position::from_meters(-5.0, 0.2).unwrap(),
        ])
        .unwrap();
        let rectangle = Convex::new(&[
            Position::from_meters(-0.65, -0.4).unwrap(),
            Position::from_meters(0.65, -0.4).unwrap(),
            Position::from_meters(0.65, 0.4).unwrap(),
            Position::from_meters(-0.65, 0.4).unwrap(),
        ])
        .unwrap();
        let angle = Angle::from_radians(20_f64.to_radians()).unwrap();
        let transform_a = Transform::new(Position::from_meters(0.8, 0.88).unwrap(), angle);
        let manifold = collide(
            BodyId::new(1),
            rectangle,
            transform_a,
            BodyId::new(2),
            floor,
            Transform::new(Position::ZERO, angle),
        )
        .unwrap();
        let contacts = manifold.into_contacts().collect::<alloc::vec::Vec<_>>();

        assert_eq!(contacts.len(), 2);
        let center = GeometryPoint::from(transform_a.position);
        let tangent = contacts[0].normal.perpendicular();
        let first_lever = tangent.dot(contacts[0].point - center);
        let second_lever = tangent.dot(contacts[1].point - center);
        assert!(first_lever < 0);
        assert!(second_lever > 0);
        assert!((first_lever + second_lever).abs() <= 4);
    }
}
