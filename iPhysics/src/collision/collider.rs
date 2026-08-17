use super::{Contact, collide_circles};
use crate::body::BodyId;
use crate::collider::{Circle, Collider, Convex};
use crate::geometry::UnitVector;
use crate::quantity::{DiffVec2, Length, Position};
use crate::transform::Transform;

pub(crate) fn collide(
    body_a: BodyId,
    collider_a: Collider,
    transform_a: Transform,
    body_b: BodyId,
    collider_b: Collider,
    transform_b: Transform,
) -> Option<Contact> {
    match (collider_a, collider_b) {
        (Collider::Circle(a), Collider::Circle(b)) => collide_circles(
            body_a,
            a,
            transform_a.position,
            body_b,
            b,
            transform_b.position,
        ),
        (Collider::Circle(circle), Collider::Convex(convex)) => {
            collide_circle_convex(body_a, circle, transform_a, body_b, convex, transform_b)
        }
        (Collider::Convex(convex), Collider::Circle(circle)) => {
            let contact =
                collide_circle_convex(body_b, circle, transform_b, body_a, convex, transform_a)?;
            Some(Contact {
                body_a,
                body_b,
                normal: negate(contact.normal),
                ..contact
            })
        }
        (Collider::Convex(a), Collider::Convex(b)) => {
            collide_convexes(body_a, a, transform_a, body_b, b, transform_b)
        }
    }
}

fn collide_convexes(
    body_a: BodyId,
    convex_a: Convex,
    transform_a: Transform,
    body_b: BodyId,
    convex_b: Convex,
    transform_b: Transform,
) -> Option<Contact> {
    let vertices_a = convex_a.transformed_vertices(transform_a)?;
    let vertices_b = convex_b.transformed_vertices(transform_b)?;
    let center_a = centroid(&vertices_a)?;
    let center_b = centroid(&vertices_b)?;
    let mut best: Option<(u32, UnitVector)> = None;

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
        point: midpoint(point_a, point_b)?,
        normal,
        penetration: Length::from_raw(penetration),
    })
}

fn collide_circle_convex(
    circle_body: BodyId,
    circle: Circle,
    circle_transform: Transform,
    convex_body: BodyId,
    convex: Convex,
    convex_transform: Transform,
) -> Option<Contact> {
    let vertices = convex.transformed_vertices(convex_transform)?;
    let circle_center = circle_transform.position;
    let convex_center = centroid(&vertices)?;
    let mut best: Option<(u32, UnitVector)> = None;

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
    if let Some(axis) = normalized(closest - circle_center) {
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
    let circle_point = offset(circle_center, normal, circle.radius().raw() as i64)?;
    let convex_point = support(&vertices, normal, false);
    Some(Contact {
        body_a: circle_body,
        body_b: convex_body,
        point: midpoint(circle_point, convex_point)?,
        normal,
        penetration: Length::from_raw(penetration),
    })
}

fn select_axis(
    best: &mut Option<(u32, UnitVector)>,
    mut axis: UnitVector,
    a: &[Position],
    b: &[Position],
    center_a: Position,
    center_b: Position,
) -> Option<()> {
    if dot_delta(center_a, center_b, axis) < 0 {
        axis = negate(axis);
    }
    let (min_a, max_a) = project(a, axis);
    let (min_b, max_b) = project(b, axis);
    let overlap = max_a.min(max_b) - min_a.max(min_b);
    if overlap < 0 {
        return None;
    }
    update_best(best, overlap, axis)
}

fn select_circle_axis(
    best: &mut Option<(u32, UnitVector)>,
    mut axis: UnitVector,
    circle_center: Position,
    radius: u32,
    convex: &[Position],
    convex_center: Position,
) -> Option<()> {
    if dot_delta(circle_center, convex_center, axis) < 0 {
        axis = negate(axis);
    }
    let circle_projection = dot(circle_center, axis);
    let min_circle = circle_projection - radius as i64;
    let max_circle = circle_projection + radius as i64;
    let (min_convex, max_convex) = project(convex, axis);
    let overlap = max_circle.min(max_convex) - min_circle.max(min_convex);
    if overlap < 0 {
        return None;
    }
    update_best(best, overlap, axis)
}

fn update_best(best: &mut Option<(u32, UnitVector)>, overlap: i64, axis: UnitVector) -> Option<()> {
    let overlap = u32::try_from(overlap).ok()?;
    if best.is_none_or(|(current, _)| overlap < current) {
        *best = Some((overlap, axis));
    }
    Some(())
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

fn support(vertices: &[Position], axis: UnitVector, maximum: bool) -> Position {
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

fn centroid(vertices: &[Position]) -> Option<Position> {
    let mut x = 0_i64;
    let mut y = 0_i64;
    for vertex in vertices {
        let raw = vertex.raw();
        x += raw[0] as i64;
        y += raw[1] as i64;
    }
    Position::checked_from_raw(
        i32::try_from(x / vertices.len() as i64).ok()?,
        i32::try_from(y / vertices.len() as i64).ok()?,
    )
}

fn midpoint(a: Position, b: Position) -> Option<Position> {
    let [ax, ay] = a.raw();
    let [bx, by] = b.raw();
    Position::checked_from_raw(
        i32::try_from((ax as i64 + bx as i64) / 2).ok()?,
        i32::try_from((ay as i64 + by as i64) / 2).ok()?,
    )
}

fn offset(point: Position, axis: UnitVector, distance: i64) -> Option<Position> {
    let [x, y] = point.raw();
    let [nx, ny] = axis.raw();
    Position::checked_from_raw(
        i32::try_from(x as i64 + round_shift_i64(nx as i64 * distance, 30)).ok()?,
        i32::try_from(y as i64 + round_shift_i64(ny as i64 * distance, 30)).ok()?,
    )
}

fn dot(point: Position, axis: UnitVector) -> i64 {
    let point = DiffVec2::from(point.raw());
    let axis = DiffVec2::from(axis.raw());
    round_shift_i64(point.dot(axis), 30)
}

fn dot_delta(a: Position, b: Position, axis: UnitVector) -> i64 {
    let axis = DiffVec2::from(axis.raw());
    round_shift_i64((b - a).dot(axis), 30)
}

fn squared_distance(a: Position, b: Position) -> u64 {
    (a - b).squared_magnitude()
}

fn normalized(vector: DiffVec2) -> Option<UnitVector> {
    let length = integer_sqrt(vector.squared_magnitude());
    if length == 0 {
        return None;
    }
    let [x, y] = vector.raw();
    let scale = 1_i64 << UnitVector::FRACTION_BITS;
    Some(UnitVector::from_raw(
        i32::try_from(x as i64 * scale / length as i64).ok()?,
        i32::try_from(y as i64 * scale / length as i64).ok()?,
    ))
}

fn negate(vector: UnitVector) -> UnitVector {
    let [x, y] = vector.raw();
    UnitVector::from_raw(-x, -y)
}

fn integer_sqrt(value: u64) -> u64 {
    if value < 2 {
        return value;
    }
    let mut x = 1_u64 << ((64 - value.leading_zeros() as u64 + 1) >> 1);
    loop {
        let next = (x + value / x) >> 1;
        if next >= x {
            return x;
        }
        x = next;
    }
}

#[inline(always)]
fn round_shift_i64(value: i64, shift: u32) -> i64 {
    let half = 1_i64 << (shift - 1);
    if value < 0 {
        -((-value + half) >> shift)
    } else {
        (value + half) >> shift
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Angle;

    fn square() -> Convex {
        Convex::new(&[
            Position::from_raw(-65_536, -65_536),
            Position::from_raw(65_536, -65_536),
            Position::from_raw(65_536, 65_536),
            Position::from_raw(-65_536, 65_536),
        ])
        .unwrap()
    }

    #[test]
    fn circle_and_convex_generate_contact_without_shape_identity() {
        let contact = collide(
            BodyId::new(1),
            Circle::new(Length::from_meters(0.5).unwrap())
                .unwrap()
                .into(),
            Transform::new(Position::from_meters(1.25, 0.0).unwrap(), Angle::ZERO),
            BodyId::new(2),
            square().into(),
            Transform::IDENTITY,
        )
        .unwrap();

        assert_eq!(contact.body_a, BodyId::new(1));
        assert_eq!(contact.body_b, BodyId::new(2));
        assert!(contact.penetration.raw() > 0);
    }

    #[test]
    fn separated_convexes_do_not_collide() {
        assert!(
            collide(
                BodyId::new(1),
                square().into(),
                Transform::IDENTITY,
                BodyId::new(2),
                square().into(),
                Transform::new(Position::from_meters(3.0, 0.0).unwrap(), Angle::ZERO),
            )
            .is_none()
        );
    }
}
