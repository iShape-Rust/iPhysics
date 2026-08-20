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
    let center_a = convex_a.transformed_center(transform_a);
    let center_b = convex_b.transformed_center(transform_b);
    let mut best: BestAxis = None;

    for normal in convex_a.normals() {
        select_axis(
            &mut best,
            normal.rotate(transform_a.angle),
            &vertices_a,
            &vertices_b,
            center_a,
            center_b,
        )?;
    }
    for normal in convex_b.normals() {
        select_axis(
            &mut best,
            normal.rotate(transform_b.angle),
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
        point: point_a.midpoint(point_b),
        normal,
        penetration: Length::from_raw(penetration),
    })
}
