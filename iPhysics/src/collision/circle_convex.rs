use super::Contact;
use super::sat::{
    BestAxis, centroid, midpoint, offset, select_circle_axis, squared_distance, support,
};
use crate::body::BodyId;
use crate::collider::{Circle, Convex};
use crate::geometry::UnitVector;
use crate::quantity::Length;
use crate::transform::Transform;

pub(super) fn collide(
    circle_body: BodyId,
    circle: Circle,
    circle_transform: Transform,
    convex_body: BodyId,
    convex: Convex,
    convex_transform: Transform,
) -> Option<Contact> {
    let vertices = convex.transformed_vertices(convex_transform);
    let circle_center = circle_transform.position;
    let convex_center = centroid(&vertices);
    let mut best: BestAxis = None;

    for index in 0..convex.len() {
        select_circle_axis(
            &mut best,
            convex.transformed_normal(index, convex_transform),
            circle_center,
            circle.radius().raw(),
            &vertices,
            convex_center,
        )?;
    }

    let closest = vertices
        .iter()
        .copied()
        .min_by_key(|vertex| squared_distance(*vertex, circle_center))?;
    if let Some(axis) = UnitVector::normalized(closest - circle_center) {
        select_circle_axis(
            &mut best,
            axis,
            circle_center,
            circle.radius().raw(),
            &vertices,
            convex_center,
        )?;
    }

    let (penetration, normal) = best?;
    let circle_point = offset(circle_center, normal, circle.radius().raw() as i32);
    let convex_point = support(&vertices, normal, false);
    Some(Contact {
        body_a: circle_body,
        body_b: convex_body,
        point: midpoint(circle_point, convex_point),
        normal,
        penetration: Length::from_raw(penetration),
    })
}
