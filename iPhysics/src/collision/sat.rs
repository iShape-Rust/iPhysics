use crate::geometry::{GeometryPoint, UnitVector};

pub(super) type BestAxis = Option<(u32, UnitVector)>;

pub(super) fn select_axis(
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

pub(super) fn support(
    vertices: &[GeometryPoint],
    axis: UnitVector,
    maximum: bool,
) -> GeometryPoint {
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

pub(super) fn offset(point: GeometryPoint, axis: UnitVector, distance: i32) -> GeometryPoint {
    let [x, y] = point.raw();
    let [offset_x, offset_y] = axis.scaled_raw(distance);
    GeometryPoint::from_i64_unchecked(x as i64 + offset_x, y as i64 + offset_y)
}

#[inline]
pub(super) fn update_best(best: &mut BestAxis, penetration: i64, axis: UnitVector) {
    let penetration = penetration as u32;
    if best.is_none_or(|(current, _)| penetration < current) {
        *best = Some((penetration, axis));
    }
}

pub(super) fn project(vertices: &[GeometryPoint], axis: UnitVector) -> (i64, i64) {
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
