use crate::geometry::UnitVector;
use crate::quantity::{DiffVec2, Position};

pub(super) type BestAxis = Option<(u32, UnitVector)>;

pub(super) fn select_axis(
    best: &mut BestAxis,
    mut axis: UnitVector,
    a: &[Position],
    b: &[Position],
    center_a: Position,
    center_b: Position,
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
    circle_center: Position,
    radius: u32,
    convex: &[Position],
    convex_center: Position,
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

pub(super) fn support(vertices: &[Position], axis: UnitVector, maximum: bool) -> Position {
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

pub(super) fn centroid(vertices: &[Position]) -> Position {
    debug_assert!(!vertices.is_empty());
    let mut x = 0_i64;
    let mut y = 0_i64;
    for vertex in vertices {
        let raw = vertex.raw();
        x += raw[0] as i64;
        y += raw[1] as i64;
    }
    let count = vertices.len() as i64;
    Position::from_raw_unchecked((x / count) as i32, (y / count) as i32)
}

pub(super) fn midpoint(a: Position, b: Position) -> Position {
    let [ax, ay] = a.raw();
    let [bx, by] = b.raw();
    Position::from_raw_unchecked(
        ((ax as i64 + bx as i64) / 2) as i32,
        ((ay as i64 + by as i64) / 2) as i32,
    )
}

pub(super) fn offset(point: Position, axis: UnitVector, distance: i32) -> Position {
    let [x, y] = point.raw();
    let [offset_x, offset_y] = axis.scaled_raw(distance).raw();
    Position::from_wide_narrow(x as i64 + offset_x as i64, y as i64 + offset_y as i64)
}

#[inline(always)]
pub(super) fn squared_distance(a: Position, b: Position) -> u64 {
    (a - b).squared_magnitude()
}

fn update_best(best: &mut BestAxis, overlap: i64, axis: UnitVector) {
    let overlap = overlap as u32;
    if best.is_none_or(|(current, _)| overlap < current) {
        *best = Some((overlap, axis));
    }
}

fn project(vertices: &[Position], axis: UnitVector) -> (i64, i64) {
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
fn dot(point: Position, axis: UnitVector) -> i64 {
    let [x, y] = point.raw();
    axis.dot_raw(DiffVec2::from_raw_unchecked(x, y))
}

#[inline(always)]
fn dot_delta(a: Position, b: Position, axis: UnitVector) -> i64 {
    axis.dot_raw(b - a)
}
