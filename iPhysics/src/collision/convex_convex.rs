use super::Contact;
use super::sat::{BestAxis, centroid, midpoint, select_axis, support};
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
    let center_a = centroid(&vertices_a);
    let center_b = centroid(&vertices_b);
    let mut best: BestAxis = None;

    for index in 0..convex_a.len() {
        select_axis(
            &mut best,
            convex_a.transformed_normal(index, transform_a),
            &vertices_a,
            &vertices_b,
            center_a,
            center_b,
        )?;
    }
    for index in 0..convex_b.len() {
        select_axis(
            &mut best,
            convex_b.transformed_normal(index, transform_b),
            &vertices_a,
            &vertices_b,
            center_a,
            center_b,
        )?;
    }

    let (penetration, normal) = best?;
    let point_a = support(&vertices_a, normal, true);
    let point_b = support(&vertices_b, normal, false);
    Some(Contact {
        body_a,
        body_b,
        point: midpoint(point_a, point_b),
        normal,
        penetration: Length::from_raw(penetration),
    })
}
