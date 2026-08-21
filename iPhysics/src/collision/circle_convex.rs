use super::Contact;
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
    let radius = circle.radius().raw() as i64;
    let mut best_index = 0;
    let mut best_separation = i64::MIN;
    let mut best_normal = UnitVector::X;

    for (index, normal) in convex.normals().iter().enumerate() {
        let normal = normal.rotate(convex_transform.angle);
        let separation = normal.dot(circle_center - vertices[index]);
        if separation > radius {
            return None;
        }
        // Interior separations are negative, so the greatest value belongs
        // to the nearest face (the value closest to zero).
        if separation > best_separation {
            best_index = index;
            best_separation = separation;
            best_normal = normal;
        }
    }

    let a = vertices[best_index];
    let b = vertices[(best_index + 1) % vertices.len()];

    if best_separation <= 0 {
        return Some(build_contact(
            circle_body,
            convex_body,
            circle_center,
            -best_normal,
            (radius - best_separation) as u32,
            circle.radius().raw(),
        ));
    }

    let edge = b - a;
    if edge.dot(circle_center - a) <= 0 {
        vertex_contact(
            circle_body,
            convex_body,
            circle_center,
            a,
            circle.radius().raw(),
        )
    } else if edge.dot(circle_center - b) >= 0 {
        vertex_contact(
            circle_body,
            convex_body,
            circle_center,
            b,
            circle.radius().raw(),
        )
    } else {
        Some(build_contact(
            circle_body,
            convex_body,
            circle_center,
            -best_normal,
            (radius - best_separation) as u32,
            circle.radius().raw(),
        ))
    }
}

fn vertex_contact(
    circle_body: BodyId,
    convex_body: BodyId,
    circle_center: GeometryPoint,
    vertex: GeometryPoint,
    radius: u32,
) -> Option<Contact> {
    let delta = vertex - circle_center;
    let distance_squared = delta.squared_magnitude();
    let radius = radius as u64;
    if distance_squared > radius * radius {
        return None;
    }

    let distance = distance_squared.isqrt();
    let normal = UnitVector::normalized_with_length(delta, distance)?;
    Some(build_contact(
        circle_body,
        convex_body,
        circle_center,
        normal,
        (radius - distance) as u32,
        radius as u32,
    ))
}

fn build_contact(
    circle_body: BodyId,
    convex_body: BodyId,
    circle_center: GeometryPoint,
    normal: UnitVector,
    penetration: u32,
    radius: u32,
) -> Contact {
    let contact_offset = radius as i32 - (penetration / 2) as i32;
    Contact {
        body_a: circle_body,
        body_b: convex_body,
        point: offset(circle_center, normal, contact_offset),
        normal,
        penetration: Length::from_raw(penetration),
    }
}

fn offset(point: GeometryPoint, axis: UnitVector, distance: i32) -> GeometryPoint {
    let [x, y] = point.raw();
    let [offset_x, offset_y] = axis.scaled_raw(distance);
    GeometryPoint::from_i64_unchecked(x as i64 + offset_x, y as i64 + offset_y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::{Angle, Position};

    fn square() -> Convex {
        Convex::new(&[
            Position::from_meters(-1.0, -1.0).unwrap(),
            Position::from_meters(1.0, -1.0).unwrap(),
            Position::from_meters(1.0, 1.0).unwrap(),
            Position::from_meters(-1.0, 1.0).unwrap(),
        ])
        .unwrap()
    }

    fn collide_with_square(center: Position, radius: f64) -> Option<Contact> {
        collide(
            BodyId::new(1),
            Circle::new(Length::from_meters(radius).unwrap()).unwrap(),
            Transform::new(center, Angle::ZERO),
            BodyId::new(2),
            square(),
            Transform::IDENTITY,
        )
    }

    #[test]
    fn face_contact_points_from_circle_to_convex() {
        let contact = collide_with_square(Position::from_meters(1.25, 0.0).unwrap(), 0.5).unwrap();

        assert_eq!(contact.normal, -UnitVector::X);
        assert_eq!(contact.penetration.to_meters(), 0.25);
        assert_eq!(contact.point.to_meters(), [0.875, 0.0]);
    }

    #[test]
    fn face_distances_reject_a_circle_past_a_corner() {
        assert!(collide_with_square(Position::from_meters(1.2, 1.2).unwrap(), 0.25).is_none());
    }

    #[test]
    fn corner_contact_uses_the_vertex_direction() {
        let contact = collide_with_square(Position::from_meters(1.2, 1.2).unwrap(), 0.3).unwrap();
        let [normal_x, normal_y] = contact.normal.raw();

        assert!(normal_x < 0);
        assert!(normal_y < 0);
        assert!(contact.penetration.raw() > 0);
    }

    #[test]
    fn contained_circle_uses_distance_to_the_nearest_face() {
        let contact = collide_with_square(Position::ZERO, 0.5).unwrap();

        assert_eq!(contact.normal.raw(), [0, 1 << 30]);
        assert_eq!(contact.penetration.to_meters(), 1.5);
    }
}
