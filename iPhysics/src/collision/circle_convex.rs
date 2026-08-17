use super::Contact;
use super::sat::{BestAxis, offset, select_circle_axis, support};
use crate::body::BodyId;
use crate::collider::{Circle, Convex};
use crate::geometry::{GeometryPoint, UnitVector};
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
    let circle_center = GeometryPoint::from(circle_transform.position);
    let convex_center = convex.transformed_center(convex_transform);
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
        .min_by_key(|vertex| vertex.squared_distance(circle_center))?;
    if let Some(axis) = UnitVector::normalized_wide(closest.delta(circle_center)) {
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
        point: circle_point.midpoint(convex_point),
        normal,
        penetration: Length::from_raw(penetration),
    })
}
