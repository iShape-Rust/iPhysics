use super::Contact;
use super::sat::{BestAxis, offset, project, support};
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
    let circle_center = circle_transform.position.into();
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
        point: circle_point.midpoint(convex_point),
        normal,
        penetration: Length::from_raw(penetration),
    })
}

pub(super) fn select_circle_axis(
    best: &mut BestAxis,
    mut axis: UnitVector,
    circle_center: GeometryPoint,
    radius: u32,
    convex: &[GeometryPoint],
    convex_center: GeometryPoint,
) -> Option<()> {
    if axis.dot(circle_center - convex_center) < 0 {
        axis = -axis;
    }
    let circle_projection = axis.dot_point(circle_center);
    let min_circle = circle_projection - radius as i64;
    let max_circle = circle_projection + radius as i64;
    let (min_convex, max_convex) = project(convex, axis);
    let overlap = max_circle.min(max_convex) - min_circle.max(min_convex);
    if overlap < 0 {
        return None;
    }
    crate::collision::sat::update_best(best, overlap, axis);
    Some(())
}
