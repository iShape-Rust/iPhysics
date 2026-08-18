use crate::geometry::{GeometryPoint, UnitVector};

pub(super) type BestAxis = Option<(u32, UnitVector)>;

pub(super) fn select_axis(
    best: &mut BestAxis,
    mut axis: UnitVector,
    a: &[GeometryPoint],
    b: &[GeometryPoint],
    center_a: GeometryPoint,
    center_b: GeometryPoint,
) -> Option<()> {
    if dot_delta(center_a, center_b, axis) < 0 {
        axis = -axis;
    }
    let (min_a, max_a) = project(a, axis);
    let (min_b, max_b) = project(b, axis);
    let overlap = max_a.min(max_b) - min_a.max(min_b);
    if overlap < 0 {
        return None;
    }
    update_best(best, overlap, axis);
    Some(())
}

pub(super) fn select_circle_axis(
    best: &mut BestAxis,
    mut axis: UnitVector,
    circle_center: GeometryPoint,
    radius: u32,
    convex: &[GeometryPoint],
    convex_center: GeometryPoint,
) -> Option<()> {
    if dot_delta(circle_center, convex_center, axis) < 0 {
        axis = -axis;
    }
    let circle_projection = dot(circle_center, axis);
    let min_circle = circle_projection - radius as i64;
    let max_circle = circle_projection + radius as i64;
    let (min_convex, max_convex) = project(convex, axis);
    let overlap = max_circle.min(max_convex) - min_circle.max(min_convex);
    if overlap < 0 {
        return None;
    }
    update_best(best, overlap, axis);
    Some(())
}

pub(super) fn support(
    vertices: &[GeometryPoint],
    axis: UnitVector,
    maximum: bool,
) -> GeometryPoint {
    let mut result = vertices[0];
    let mut best = dot(result, axis);
    for vertex in &vertices[1..] {
        let projection = dot(*vertex, axis);
        if (maximum && projection > best) || (!maximum && projection < best) {
            result = *vertex;
            best = projection;
        }
    }
    result
}

pub(super) fn offset(point: GeometryPoint, axis: UnitVector, distance: i32) -> GeometryPoint {
    let [x, y] = point.raw();
    let [offset_x, offset_y] = axis.scaled_raw(distance);
    GeometryPoint::from_wide_narrow(x as i64 + offset_x, y as i64 + offset_y)
}

#[inline]
fn update_best(best: &mut BestAxis, overlap: i64, axis: UnitVector) {
    let overlap = overlap as u32;
    if best.is_none_or(|(current, _)| overlap < current) {
        *best = Some((overlap, axis));
    }
}

fn project(vertices: &[GeometryPoint], axis: UnitVector) -> (i64, i64) {
    let first = dot(vertices[0], axis);
    let mut min = first;
    let mut max = first;
    for vertex in &vertices[1..] {
        let projection = dot(*vertex, axis);
        min = min.min(projection);
        max = max.max(projection);
    }
    (min, max)
}

#[inline(always)]
fn dot(point: GeometryPoint, axis: UnitVector) -> i64 {
    let [x, y] = point.raw();
    axis.dot_wide_raw([x as i64, y as i64])
}

#[inline(always)]
fn dot_delta(a: GeometryPoint, b: GeometryPoint, axis: UnitVector) -> i64 {
    axis.dot_wide_raw(b.delta(a))
}
