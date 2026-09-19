use crate::geometry::{Aabb, GeometryPoint, PackedUnitVector, UnitVector};
use crate::quantity::{Position, RawVec2};
use crate::transform::Transform;

use super::inertia::from_q24_per_q32_ratio;

pub const MAX_CONVEX_VERTICES: usize = 6;

/// Body-local collider coordinate stored as signed Q5 metres.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct PackedPosition {
    x: i16,
    y: i16,
}

impl PackedPosition {
    const FRACTION_BITS: u32 = 5;
    const UNPACK_SHIFT: u32 = Position::FRACTION_BITS - Self::FRACTION_BITS;

    #[inline(always)]
    fn new(position: Position) -> Option<Self> {
        let [x, y] = position.raw();
        Some(Self {
            x: i16::try_from(x / (1 << Self::UNPACK_SHIFT)).ok()?,
            y: i16::try_from(y / (1 << Self::UNPACK_SHIFT)).ok()?,
        })
    }

    #[inline(always)]
    fn unpack(self) -> Position {
        Position::from_i32(
            self.x as i32 * (1 << Self::UNPACK_SHIFT),
            self.y as i32 * (1 << Self::UNPACK_SHIFT),
        )
    }
}

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
/// Vertices are quantized to signed Q5 metres and canonicalized to
/// counter-clockwise order. Vertices and edge normals are stored inline; no
/// allocation is required by a dynamic body. Local coordinates are limited to
/// approximately `-1024..1024 m` per component. Quantization truncates toward
/// zero; construction fails if it merges vertices or makes an edge collinear.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Convex {
    vertices: [PackedPosition; MAX_CONVEX_VERTICES],
    normals: [PackedUnitVector; MAX_CONVEX_VERTICES],
    count: u8,
}

impl Convex {
    #[inline(always)]
    pub fn new(vertices: &[Position]) -> Result<Self, ConvexError> {
        let (storage, count) = quantize_vertices(vertices)?;
        let vertices = &storage[..count];
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
        let (storage, count) = quantize_vertices(vertices)
            .expect("Convex::new_unchecked requires representable vertices");
        let vertices = &storage[..count];
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

        let mut normals = [PackedUnitVector::X; MAX_CONVEX_VERTICES];
        for i in 0..count {
            let [edge_x, edge_y] = (storage[(i + 1) % count] - storage[i]).raw();
            normals[i] = UnitVector::normalized(RawVec2::from_i32(edge_y, -edge_x))
                .expect("validated convex edges are non-zero")
                .into();
        }

        Self {
            vertices: storage.map(|vertex| {
                PackedPosition::new(vertex).expect("quantized vertices are representable")
            }),
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
    fn vertices(self) -> LocalVertices {
        let mut vertices = [Position::ZERO; MAX_CONVEX_VERTICES];
        for (index, vertex) in self.vertices[..self.count as usize]
            .iter()
            .copied()
            .enumerate()
        {
            vertices[index] = vertex.unpack();
        }
        LocalVertices {
            vertices,
            count: self.count,
        }
    }

    #[inline(always)]
    pub(crate) fn normals(&self) -> impl ExactSizeIterator<Item = UnitVector> + '_ {
        self.normals[..self.count as usize]
            .iter()
            .copied()
            .map(UnitVector::from)
    }

    pub(crate) fn aabb(self, transform: Transform) -> Aabb {
        let mut vertices = self.vertices[..self.count as usize]
            .iter()
            .copied()
            .map(|vertex| transform.apply_geometry(vertex.unpack()));
        let first = vertices
            .next()
            .expect("a convex always has at least three vertices");
        let mut min = first;
        let mut max = first;
        for point in vertices {
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
    #[inline(always)]
    pub fn transformed_vertices(self, transform: Transform) -> TransformedVertices {
        let mut result = TransformedVertices::new();
        self.write_transformed_vertices(transform, &mut result);
        result
    }

    #[inline(always)]
    pub(crate) fn write_transformed_vertices(
        self,
        transform: Transform,
        result: &mut TransformedVertices,
    ) {
        result.count = self.count;
        for (index, vertex) in self.vertices().iter().copied().enumerate() {
            result.vertices[index] = transform.apply_geometry(vertex);
        }
    }

    /// Reciprocal moment of inertia of a uniform polygon about the local
    /// origin, stored as unsigned Q40.
    pub(crate) fn inverse_inertia_q40(self, inverse_mass_q24: u32) -> u64 {
        let (twice_area, inertia_numerator) = self.inertia_integrals();
        from_q24_per_q32_ratio(
            inverse_mass_q24 as u128 * 6 * twice_area as u128,
            inertia_numerator,
        )
    }

    /// Returns proportional `(mass_weight, mass_weight * I/m)` values.
    pub(super) fn mass_properties(self) -> (u64, u128) {
        let (twice_area, inertia_numerator) = self.inertia_integrals();
        // Multiplying both the area weight and its weighted inertia by six
        // keeps this ratio exact without an early integer division.
        (twice_area.saturating_mul(6), inertia_numerator)
    }

    fn inertia_integrals(self) -> (u64, u128) {
        let vertices = self.vertices();
        let mut twice_area = 0_i128;
        let mut inertia_numerator = 0_i128;

        // I / m = sum(cross * quadratic) / (6 * sum(cross)).
        for index in 0..vertices.len() {
            let a = vertices[index] - Position::ZERO;
            let b = vertices[(index + 1) % vertices.len()] - Position::ZERO;
            let cross = a.cross(b) as i128;
            let quadratic = (a.dot(a) + a.dot(b) + b.dot(b)) as i128;
            twice_area += cross;
            inertia_numerator += cross * quadratic;
        }

        debug_assert!(twice_area > 0);
        debug_assert!(inertia_numerator > 0);

        (twice_area as u64, inertia_numerator as u128)
    }
}

#[derive(Debug, Clone, Copy)]
struct LocalVertices {
    vertices: [Position; MAX_CONVEX_VERTICES],
    count: u8,
}

impl core::ops::Deref for LocalVertices {
    type Target = [Position];

    fn deref(&self) -> &Self::Target {
        &self.vertices[..self.count as usize]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TransformedVertices {
    vertices: [GeometryPoint; MAX_CONVEX_VERTICES],
    count: u8,
}

impl TransformedVertices {
    #[inline(always)]
    pub(crate) const fn new() -> Self {
        Self {
            vertices: [GeometryPoint::ZERO; MAX_CONVEX_VERTICES],
            count: 0,
        }
    }
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

fn quantize_vertices(
    vertices: &[Position],
) -> Result<([Position; MAX_CONVEX_VERTICES], usize), ConvexError> {
    if vertices.len() < 3 {
        return Err(ConvexError::TooFewVertices);
    }
    if vertices.len() > MAX_CONVEX_VERTICES {
        return Err(ConvexError::TooManyVertices);
    }

    let mut storage = [Position::ZERO; MAX_CONVEX_VERTICES];
    for (index, vertex) in vertices.iter().copied().enumerate() {
        storage[index] = PackedPosition::new(vertex)
            .ok_or(ConvexError::VertexOutsideLimit)?
            .unpack();
    }
    Ok((storage, vertices.len()))
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
    use crate::quantity::{Angle, Mass};

    const QUANTUM_RAW: i32 = 1 << PackedPosition::UNPACK_SHIFT;

    fn position(x: i32, y: i32) -> Position {
        Position::from_i32(x * QUANTUM_RAW, y * QUANTUM_RAW)
    }

    #[test]
    fn packed_vertices_keep_convex_at_50_bytes() {
        assert_eq!(core::mem::size_of::<PackedPosition>(), 4);
        assert_eq!(core::mem::size_of::<Convex>(), 50);
    }

    #[test]
    fn packed_vertices_use_symmetric_q5_truncation() {
        let positive = Position::from_meters(0.05, 0.0).unwrap();
        let negative = Position::from_meters(-0.05, 0.0).unwrap();
        let positive = PackedPosition::new(positive).unwrap();
        let negative = PackedPosition::new(negative).unwrap();

        assert_eq!(positive.x, 1);
        assert_eq!(negative.x, -1);
        assert_eq!(positive.unpack().to_meters()[0], 0.03125);
        assert_eq!(negative.unpack().to_meters()[0], -0.03125);
    }

    #[test]
    fn canonicalizes_clockwise_vertices() {
        let vertices = [
            position(-10, -10),
            position(-10, 10),
            position(10, 10),
            position(10, -10),
        ];
        let convex = Convex::new(&vertices).unwrap();

        let unpacked = convex.vertices();
        let [a, b, c, ..] = &*unpacked else {
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
            position(0, 0),
            position(10, 0),
            position(5, 5),
            position(10, 10),
            position(0, 10),
        ]);
    }

    #[test]
    fn rejects_concave_polygon() {
        let result = Convex::new(&[
            position(0, 0),
            position(10, 0),
            position(5, 5),
            position(10, 10),
            position(0, 10),
        ]);

        assert_eq!(result, Err(ConvexError::NotConvex));
    }

    #[test]
    fn cyclic_permutations_have_identical_storage() {
        let a = Convex::new(&[
            position(-10, -10),
            position(10, -10),
            position(10, 10),
            position(-10, 10),
        ])
        .unwrap();
        let b = Convex::new(&[
            position(10, 10),
            position(-10, 10),
            position(-10, -10),
            position(10, -10),
        ])
        .unwrap();

        assert_eq!(a, b);
    }

    #[test]
    fn rejects_self_intersecting_order() {
        let result = Convex::new(&[
            position(0, 10),
            position(6, -8),
            position(-10, 3),
            position(10, 3),
            position(-6, -8),
        ]);

        assert_eq!(result, Err(ConvexError::NotConvex));
    }

    #[test]
    fn rotated_aabb_is_deterministic() {
        let convex = Convex::new(&[
            position(-20, -10),
            position(20, -10),
            position(20, 10),
            position(-20, 10),
        ])
        .unwrap();
        let aabb = convex.aabb(Transform::new(Position::ZERO, Angle::QUARTER_TURN));

        assert_eq!(aabb.min().raw(), [-10 * QUANTUM_RAW, -20 * QUANTUM_RAW]);
        assert_eq!(aabb.max().raw(), [10 * QUANTUM_RAW, 20 * QUANTUM_RAW]);
    }

    #[test]
    fn vertices_must_fit_packed_coordinate_limit() {
        let outside = (i16::MAX as i32 + 1) * QUANTUM_RAW;
        assert_eq!(
            Convex::new(&[
                Position::from_i32(outside, 0),
                position(0, 1),
                position(1, 0),
            ]),
            Err(ConvexError::VertexOutsideLimit)
        );

        let max = i16::MAX as i32;
        assert!(Convex::new(&[position(max, 0), position(0, 1), position(0, -1),]).is_ok());
    }

    #[test]
    fn aabb_can_extend_beyond_position_range() {
        let world_max = Position::MAX_POSITION;
        let local_max = i16::MAX as i32 * QUANTUM_RAW;
        let convex = Convex::new(&[
            Position::from_i32(local_max, 0),
            position(0, 1),
            position(0, -1),
        ])
        .unwrap();
        let aabb = convex.aabb(Transform::new(
            Position::from_i32(world_max, world_max),
            Angle::ZERO,
        ));

        assert_eq!(aabb.max().raw()[0], world_max + local_max);
        assert!(aabb.max().raw()[0] > Position::MAX_POSITION);
    }

    #[test]
    fn radial_limit_survives_non_cardinal_rotation_at_world_edge() {
        let world_max = Position::MAX_POSITION;
        let local_max = i16::MAX as i32 * QUANTUM_RAW;
        let convex = Convex::new(&[
            Position::from_i32(local_max, 0),
            position(0, 1),
            position(0, -1),
        ])
        .unwrap();
        let vertices = convex.transformed_vertices(Transform::new(
            Position::from_i32(world_max, world_max),
            Angle::from_bits(0x1234_5678),
        ));

        assert!(vertices.iter().all(|point| {
            let [x, y] = point.raw();
            x >= 0 && y >= 0
        }));
    }

    #[test]
    fn square_inverse_inertia_is_three_halves_for_unit_mass() {
        let convex = Convex::new(&[
            Position::from_meters(-1.0, -1.0).unwrap(),
            Position::from_meters(1.0, -1.0).unwrap(),
            Position::from_meters(1.0, 1.0).unwrap(),
            Position::from_meters(-1.0, 1.0).unwrap(),
        ])
        .unwrap();

        assert_eq!(
            convex.inverse_inertia_q40(Mass::ONE.inverse_q24()),
            3_u64 << 39
        );
    }

    #[test]
    fn shifted_polygon_includes_parallel_axis_term() {
        let convex = Convex::new(&[
            Position::from_meters(1.0, -1.0).unwrap(),
            Position::from_meters(3.0, -1.0).unwrap(),
            Position::from_meters(3.0, 1.0).unwrap(),
            Position::from_meters(1.0, 1.0).unwrap(),
        ])
        .unwrap();

        // I / m = 2/3 + 2^2 = 14/3, hence inverse I = 3/14.
        let expected = ((3_u128 << 40) + 7) / 14;
        assert_eq!(
            convex.inverse_inertia_q40(Mass::ONE.inverse_q24()),
            expected as u64
        );
    }
}
