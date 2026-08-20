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
/// Every vertex must be within `Position::MAX_POS` of the local origin so a
/// rotated vertex plus any valid body position fits in `GeometryPoint`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Convex {
    vertices: [Position; MAX_CONVEX_VERTICES],
    normals: [UnitVector; MAX_CONVEX_VERTICES],
    count: u8,
}

impl Convex {
    #[inline(always)]
    pub fn new(vertices: &[Position]) -> Result<Self, ConvexError> {
        let winding = validate_vertices(vertices)?;
        Ok(Self::from_valid_vertices(vertices, winding))
    }

    /// Builds a convex from vertices known to satisfy the same invariants as
    /// [`Self::new`]. The preconditions are verified in debug builds only.
    ///
    /// The vertices may use either winding and any cyclic starting point; the
    /// result is still canonicalized before its normals are built.
    #[cfg(test)]
    #[inline(always)]
    fn new_unchecked(vertices: &[Position]) -> Self {
        debug_assert!(
            validate_vertices(vertices).is_ok(),
            "Convex::new_unchecked requires a valid strict convex"
        );
        Self::from_valid_vertices(vertices, winding_unchecked(vertices))
    }

    #[inline(always)]
    fn from_valid_vertices(vertices: &[Position], winding: i8) -> Self {
        let count = vertices.len();
        let mut storage = [Position::ZERO; MAX_CONVEX_VERTICES];
        storage[..count].copy_from_slice(vertices);

        if winding < 0 {
            storage[..count].reverse();
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

        let mut normals = [UnitVector::X; MAX_CONVEX_VERTICES];
        for i in 0..count {
            let [edge_x, edge_y] = (storage[(i + 1) % count] - storage[i]).raw();
            normals[i] = UnitVector::normalized(RawVec2::from_i32(edge_y, -edge_x))
                .expect("validated convex edges are non-zero");
        }

        Self {
            vertices: storage,
            normals,
            count: count as u8,
        }
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
    pub(crate) fn vertices(&self) -> &[Position] {
        &self.vertices[..self.count as usize]
    }

    #[inline(always)]
    pub(crate) fn normals(&self) -> &[UnitVector] {
        &self.normals[..self.count as usize]
    }

    pub(crate) fn aabb(self, transform: Transform) -> Aabb {
        let vertices = self.transformed_vertices(transform);
        let mut min = vertices[0];
        let mut max = vertices[0];
        for point in &vertices[1..] {
            let [x, y] = point.raw();
            let [min_x, min_y] = min.raw();
            let [max_x, max_y] = max.raw();
            min = GeometryPoint::from_i32_unchecked(min_x.min(x), min_y.min(y));
            max = GeometryPoint::from_i32_unchecked(max_x.max(x), max_y.max(y));
        }
        Aabb::from_points(min, max)
    }

    /// Returns derived world-space vertices in the bounded geometry domain.
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

#[inline(always)]
fn validate_vertices(vertices: &[Position]) -> Result<i8, ConvexError> {
    if vertices.len() < 3 {
        return Err(ConvexError::TooFewVertices);
    }
    if vertices.len() > MAX_CONVEX_VERTICES {
        return Err(ConvexError::TooManyVertices);
    }

    let max_radius = Position::MAX_POSITION as u64;
    let max_squared_radius = max_radius * max_radius;
    if vertices
        .iter()
        .any(|vertex| vertex.squared_distance(Position::ZERO) > max_squared_radius)
    {
        return Err(ConvexError::VertexOutsideLimit);
    }

    for i in 0..vertices.len() {
        for j in i + 1..vertices.len() {
            if vertices[i] == vertices[j] {
                return Err(ConvexError::DuplicateVertex);
            }
        }
    }

    let mut winding = 0_i8;
    for i in 0..vertices.len() {
        let a = vertices[i];
        let b = vertices[(i + 1) % vertices.len()];
        let c = vertices[(i + 2) % vertices.len()];
        let cross = (b - a).cross(c - b);
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

    // A consistent turn at adjacent corners is not sufficient for an
    // arbitrary input order (a self-intersecting star can satisfy it).
    // Every remaining vertex must be strictly inside every oriented edge.
    for edge in 0..vertices.len() {
        let next = (edge + 1) % vertices.len();
        for vertex in 0..vertices.len() {
            if vertex == edge || vertex == next {
                continue;
            }
            let a = vertices[edge];
            let side = (vertices[next] - a).cross(vertices[vertex] - a);
            if side == 0 {
                return Err(ConvexError::CollinearEdge);
            }
            if (side > 0) != (winding > 0) {
                return Err(ConvexError::NotConvex);
            }
        }
    }

    Ok(winding)
}

#[cfg(test)]
#[inline(always)]
fn winding_unchecked(vertices: &[Position]) -> i8 {
    let cross = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[1]);
    if cross < 0 { -1 } else { 1 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Angle;

    #[test]
    fn canonicalizes_clockwise_vertices() {
        let vertices = [
            Position::from_i32(-10, -10),
            Position::from_i32(-10, 10),
            Position::from_i32(10, 10),
            Position::from_i32(10, -10),
        ];
        let convex = Convex::new(&vertices).unwrap();

        let [a, b, c, ..] = convex.vertices() else {
            unreachable!()
        };
        assert!((*b - *a).cross(*c - *b) > 0);
        assert_eq!(Convex::new_unchecked(&vertices), convex);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "Convex::new_unchecked requires a valid strict convex")]
    fn unchecked_constructor_validates_in_debug_builds() {
        let _ = Convex::new_unchecked(&[
            Position::from_i32(0, 0),
            Position::from_i32(10, 0),
            Position::from_i32(5, 5),
            Position::from_i32(10, 10),
            Position::from_i32(0, 10),
        ]);
    }

    #[test]
    fn rejects_concave_polygon() {
        let result = Convex::new(&[
            Position::from_i32(0, 0),
            Position::from_i32(10, 0),
            Position::from_i32(5, 5),
            Position::from_i32(10, 10),
            Position::from_i32(0, 10),
        ]);

        assert_eq!(result, Err(ConvexError::NotConvex));
    }

    #[test]
    fn cyclic_permutations_have_identical_storage() {
        let a = Convex::new(&[
            Position::from_i32(-10, -10),
            Position::from_i32(10, -10),
            Position::from_i32(10, 10),
            Position::from_i32(-10, 10),
        ])
        .unwrap();
        let b = Convex::new(&[
            Position::from_i32(10, 10),
            Position::from_i32(-10, 10),
            Position::from_i32(-10, -10),
            Position::from_i32(10, -10),
        ])
        .unwrap();

        assert_eq!(a, b);
    }

    #[test]
    fn rejects_self_intersecting_order() {
        let result = Convex::new(&[
            Position::from_i32(0, 10),
            Position::from_i32(6, -8),
            Position::from_i32(-10, 3),
            Position::from_i32(10, 3),
            Position::from_i32(-6, -8),
        ]);

        assert_eq!(result, Err(ConvexError::NotConvex));
    }

    #[test]
    fn rotated_aabb_is_deterministic() {
        let convex = Convex::new(&[
            Position::from_i32(-20, -10),
            Position::from_i32(20, -10),
            Position::from_i32(20, 10),
            Position::from_i32(-20, 10),
        ])
        .unwrap();
        let aabb = convex.aabb(Transform::new(Position::ZERO, Angle::QUARTER_TURN));

        assert_eq!(aabb.min().raw(), [-10, -20]);
        assert_eq!(aabb.max().raw(), [10, 20]);
    }

    #[test]
    fn vertices_must_fit_local_radius_limit() {
        let max = Position::MAX_POSITION;
        assert_eq!(
            Convex::new(&[
                Position::from_i32(max, max),
                Position::from_i32(0, 1),
                Position::from_i32(1, 0),
            ]),
            Err(ConvexError::VertexOutsideLimit)
        );

        assert!(
            Convex::new(&[
                Position::from_i32(max, 0),
                Position::from_i32(0, 1),
                Position::from_i32(0, -1),
            ])
            .is_ok()
        );
    }

    #[test]
    fn aabb_can_extend_beyond_position_range() {
        let max = Position::MAX_POSITION;
        let convex = Convex::new(&[
            Position::from_i32(max, 0),
            Position::from_i32(0, 1),
            Position::from_i32(0, -1),
        ])
        .unwrap();
        let aabb = convex.aabb(Transform::new(Position::from_i32(max, max), Angle::ZERO));

        assert_eq!(aabb.max().raw()[0], 2 * Position::MAX_POSITION);
        assert!(aabb.max().raw()[0] > Position::MAX_POSITION);
    }

    #[test]
    fn radial_limit_survives_non_cardinal_rotation_at_world_edge() {
        let max = Position::MAX_POSITION;
        let convex = Convex::new(&[
            Position::from_i32(max, 0),
            Position::from_i32(0, 1),
            Position::from_i32(0, -1),
        ])
        .unwrap();
        let vertices = convex.transformed_vertices(Transform::new(
            Position::from_i32(max, max),
            Angle::from_raw(0x1234_5678),
        ));

        assert!(vertices.iter().all(|point| {
            let [x, y] = point.raw();
            x >= 0 && y >= 0
        }));
    }
}
