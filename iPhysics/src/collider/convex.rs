use crate::geometry::{Aabb, GeometryPoint, UnitVector};
use crate::quantity::{Position, RawVec2};
use crate::transform::Transform;

pub const MAX_CONVEX_VERTICES: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConvexError {
    TooFewVertices,
    TooManyVertices,
    VertexOutsideLimit,
    DuplicateVertex,
    CollinearEdge,
    NotConvex,
}

/// Strictly convex polygon with three to six local-space vertices.
///
/// Vertices are canonicalized to counter-clockwise order. Vertices and edge
/// normals are stored inline; no allocation is required by a dynamic body.
/// Every vertex must be within `Position::MAX_RAW` of the local origin so a
/// rotated vertex plus any valid body position still fits in full `i32`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Convex {
    vertices: [Position; MAX_CONVEX_VERTICES],
    normals: [UnitVector; MAX_CONVEX_VERTICES],
    center: Position,
    count: u8,
}

impl Convex {
    pub const MAX_VERTICES: usize = MAX_CONVEX_VERTICES;

    pub fn new(vertices: &[Position]) -> Result<Self, ConvexError> {
        if vertices.len() < 3 {
            return Err(ConvexError::TooFewVertices);
        }
        if vertices.len() > MAX_CONVEX_VERTICES {
            return Err(ConvexError::TooManyVertices);
        }

        let count = vertices.len();
        let mut storage = [Position::ZERO; MAX_CONVEX_VERTICES];
        storage[..count].copy_from_slice(vertices);

        let max_radius = Position::MAX_RAW as u64;
        let max_squared_radius = max_radius * max_radius;
        if storage[..count]
            .iter()
            .any(|vertex| vertex.squared_distance(Position::ZERO) > max_squared_radius)
        {
            return Err(ConvexError::VertexOutsideLimit);
        }

        for i in 0..count {
            for j in i + 1..count {
                if storage[i] == storage[j] {
                    return Err(ConvexError::DuplicateVertex);
                }
            }
        }

        let mut winding = 0_i8;
        for i in 0..count {
            let cross = corner_cross(
                storage[i],
                storage[(i + 1) % count],
                storage[(i + 2) % count],
            );
            if cross == 0 {
                return Err(ConvexError::CollinearEdge);
            }
            let sign = if cross > 0 { 1 } else { -1 };
            if winding == 0 {
                winding = sign;
            } else if winding != sign {
                return Err(ConvexError::NotConvex);
            }
        }

        if winding < 0 {
            storage[..count].reverse();
        }

        // A consistent turn at adjacent corners is not sufficient for an
        // arbitrary input order (a self-intersecting star can satisfy it).
        // Every remaining vertex must be strictly inside every CCW edge.
        for edge in 0..count {
            let next = (edge + 1) % count;
            for vertex in 0..count {
                if vertex == edge || vertex == next {
                    continue;
                }
                let side = edge_cross(storage[edge], storage[next], storage[vertex]);
                if side == 0 {
                    return Err(ConvexError::CollinearEdge);
                }
                if side < 0 {
                    return Err(ConvexError::NotConvex);
                }
            }
        }

        // Canonical start makes cyclic permutations and opposite winding
        // produce exactly the same inline representation.
        let first = (0..count)
            .min_by_key(|&index| {
                let [x, y] = storage[index].raw();
                (x, y)
            })
            .expect("a convex always has at least three vertices");
        storage[..count].rotate_left(first);

        let center = center_of_mass(&storage[..count]);

        let mut normals = [UnitVector::X; MAX_CONVEX_VERTICES];
        for i in 0..count {
            let [edge_x, edge_y] = (storage[(i + 1) % count] - storage[i]).raw();
            normals[i] = UnitVector::normalized(RawVec2::from_raw_unchecked(edge_y, -edge_x))
                .ok_or(ConvexError::CollinearEdge)?;
        }

        Ok(Self {
            vertices: storage,
            normals,
            center,
            count: count as u8,
        })
    }

    #[inline(always)]
    pub const fn len(self) -> usize {
        self.count as usize
    }

    #[inline(always)]
    pub const fn is_empty(self) -> bool {
        false
    }

    #[inline(always)]
    pub fn vertices(&self) -> &[Position] {
        &self.vertices[..self.count as usize]
    }

    #[inline(always)]
    pub fn normals(&self) -> &[UnitVector] {
        &self.normals[..self.count as usize]
    }

    /// Local center of mass for a polygon with uniform density.
    #[inline(always)]
    pub const fn center(self) -> Position {
        self.center
    }

    /// Twice the polygon area in raw Q32 square-metre units.
    ///
    /// Area is not part of the per-tick collision path, so it is computed on
    /// demand instead of increasing every convex collider's storage.
    pub fn doubled_area_raw(&self) -> u64 {
        doubled_area(self.vertices())
    }

    pub fn aabb(self, transform: Transform) -> Aabb {
        let vertices = self.transformed_vertices(transform);
        let mut min = vertices[0];
        let mut max = vertices[0];
        for point in &vertices[1..] {
            let [x, y] = point.raw();
            let [min_x, min_y] = min.raw();
            let [max_x, max_y] = max.raw();
            min = GeometryPoint::from_raw(min_x.min(x), min_y.min(y));
            max = GeometryPoint::from_raw(max_x.max(x), max_y.max(y));
        }
        Aabb::from_points(min, max)
    }

    /// Returns derived world-space vertices in the full-i32 geometry domain.
    /// The constructor's radial invariant makes this transformation exact:
    /// no world-boundary saturation is needed.
    pub fn transformed_vertices(self, transform: Transform) -> TransformedVertices {
        let mut result = TransformedVertices {
            vertices: [GeometryPoint::ZERO; MAX_CONVEX_VERTICES],
            count: self.count,
        };
        for (index, vertex) in self.vertices().iter().copied().enumerate() {
            result.vertices[index] = transform.apply_geometry(vertex);
        }
        result
    }

    pub(crate) fn transformed_normal(self, index: usize, transform: Transform) -> UnitVector {
        self.normals[index].rotate(transform.angle)
    }

    #[inline(always)]
    pub(crate) fn transformed_center(self, transform: Transform) -> GeometryPoint {
        transform.apply_geometry(self.center)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TransformedVertices {
    vertices: [GeometryPoint; MAX_CONVEX_VERTICES],
    count: u8,
}

impl core::ops::Deref for TransformedVertices {
    type Target = [GeometryPoint];

    fn deref(&self) -> &Self::Target {
        &self.vertices[..self.count as usize]
    }
}

fn corner_cross(a: Position, b: Position, c: Position) -> i64 {
    (b - a).cross(c - b)
}

fn edge_cross(a: Position, b: Position, point: Position) -> i64 {
    (b - a).cross(point - a)
}

/// Computes the uniform-density center of mass in one traversal. Vertices are
/// already canonicalized counter-clockwise.
fn center_of_mass(vertices: &[Position]) -> Position {
    let mut area2 = 0_i64;
    let mut center_x = 0_i128;
    let mut center_y = 0_i128;

    for edge in vertices.windows(2) {
        accumulate_edge(edge[0], edge[1], &mut area2, &mut center_x, &mut center_y);
    }
    accumulate_edge(
        vertices[vertices.len() - 1],
        vertices[0],
        &mut area2,
        &mut center_x,
        &mut center_y,
    );

    debug_assert!(area2 > 0);
    let denominator = 3 * area2 as i128;
    Position::from_raw_unchecked(
        div_round(center_x, denominator) as i32,
        div_round(center_y, denominator) as i32,
    )
}

fn doubled_area(vertices: &[Position]) -> u64 {
    let mut area2 = 0_i64;
    for edge in vertices.windows(2) {
        area2 = area2.wrapping_add(edge_cross_raw(edge[0], edge[1]));
    }
    area2 = area2.wrapping_add(edge_cross_raw(vertices[vertices.len() - 1], vertices[0]));
    area2 as u64
}

#[inline(always)]
fn accumulate_edge(
    a: Position,
    b: Position,
    area2: &mut i64,
    center_x: &mut i128,
    center_y: &mut i128,
) {
    let [ax, ay] = a.raw();
    let [bx, by] = b.raw();
    let cross = edge_cross_raw(a, b);
    *area2 = area2.wrapping_add(cross);
    *center_x += (ax + bx) as i128 * cross as i128;
    *center_y += (ay + by) as i128 * cross as i128;
}

#[inline(always)]
fn edge_cross_raw(a: Position, b: Position) -> i64 {
    let [ax, ay] = a.raw();
    let [bx, by] = b.raw();
    ax as i64 * by as i64 - ay as i64 * bx as i64
}

#[inline(always)]
fn div_round(numerator: i128, denominator: i128) -> i128 {
    let half = denominator / 2;
    if numerator < 0 {
        -((-numerator + half) / denominator)
    } else {
        (numerator + half) / denominator
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Angle;

    #[test]
    fn canonicalizes_clockwise_vertices() {
        let convex = Convex::new(&[
            Position::from_raw(-10, -10),
            Position::from_raw(-10, 10),
            Position::from_raw(10, 10),
            Position::from_raw(10, -10),
        ])
        .unwrap();

        assert!(
            corner_cross(
                convex.vertices()[0],
                convex.vertices()[1],
                convex.vertices()[2]
            ) > 0
        );
    }

    #[test]
    fn rejects_concave_polygon() {
        let result = Convex::new(&[
            Position::from_raw(0, 0),
            Position::from_raw(10, 0),
            Position::from_raw(5, 5),
            Position::from_raw(10, 10),
            Position::from_raw(0, 10),
        ]);

        assert_eq!(result, Err(ConvexError::NotConvex));
    }

    #[test]
    fn cyclic_permutations_have_identical_storage() {
        let a = Convex::new(&[
            Position::from_raw(-10, -10),
            Position::from_raw(10, -10),
            Position::from_raw(10, 10),
            Position::from_raw(-10, 10),
        ])
        .unwrap();
        let b = Convex::new(&[
            Position::from_raw(10, 10),
            Position::from_raw(-10, 10),
            Position::from_raw(-10, -10),
            Position::from_raw(10, -10),
        ])
        .unwrap();

        assert_eq!(a, b);
    }

    #[test]
    fn rejects_self_intersecting_order() {
        let result = Convex::new(&[
            Position::from_raw(0, 10),
            Position::from_raw(6, -8),
            Position::from_raw(-10, 3),
            Position::from_raw(10, 3),
            Position::from_raw(-6, -8),
        ]);

        assert_eq!(result, Err(ConvexError::NotConvex));
    }

    #[test]
    fn rotated_aabb_is_deterministic() {
        let convex = Convex::new(&[
            Position::from_raw(-20, -10),
            Position::from_raw(20, -10),
            Position::from_raw(20, 10),
            Position::from_raw(-20, 10),
        ])
        .unwrap();
        let aabb = convex.aabb(Transform::new(Position::ZERO, Angle::QUARTER_TURN));

        assert_eq!(aabb.min().raw(), [-10, -20]);
        assert_eq!(aabb.max().raw(), [10, 20]);
    }

    #[test]
    fn stores_area_and_uniform_density_center() {
        let convex = Convex::new(&[
            Position::from_raw(0, 0),
            Position::from_raw(10, 0),
            Position::from_raw(1, 10),
            Position::from_raw(0, 10),
        ])
        .unwrap();

        assert_eq!(convex.doubled_area_raw(), 110);
        assert_eq!(convex.center(), Position::from_raw(3, 4));
    }

    #[test]
    fn vertices_must_fit_local_radius_limit() {
        let max = Position::MAX_RAW;
        assert_eq!(
            Convex::new(&[
                Position::from_raw(max, max),
                Position::from_raw(0, 1),
                Position::from_raw(1, 0),
            ]),
            Err(ConvexError::VertexOutsideLimit)
        );

        assert!(
            Convex::new(&[
                Position::from_raw(max, 0),
                Position::from_raw(0, 1),
                Position::from_raw(0, -1),
            ])
            .is_ok()
        );
    }

    #[test]
    fn aabb_can_extend_beyond_position_range() {
        let max = Position::MAX_RAW;
        let convex = Convex::new(&[
            Position::from_raw(max, 0),
            Position::from_raw(0, 1),
            Position::from_raw(0, -1),
        ])
        .unwrap();
        let aabb = convex.aabb(Transform::new(Position::from_raw(max, max), Angle::ZERO));

        assert_eq!(aabb.max().raw()[0], i32::MAX - 1);
        assert!(aabb.max().raw()[0] > Position::MAX_RAW);
    }

    #[test]
    fn radial_limit_survives_non_cardinal_rotation_at_world_edge() {
        let max = Position::MAX_RAW;
        let convex = Convex::new(&[
            Position::from_raw(max, 0),
            Position::from_raw(0, 1),
            Position::from_raw(0, -1),
        ])
        .unwrap();
        let vertices = convex.transformed_vertices(Transform::new(
            Position::from_raw(max, max),
            Angle::from_raw(0x1234_5678),
        ));

        assert!(vertices.iter().all(|point| {
            let [x, y] = point.raw();
            x >= 0 && y >= 0
        }));
    }
}
