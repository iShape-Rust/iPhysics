use crate::geometry::{Aabb, UnitVector};
use crate::quantity::{Position, RawWideVec2};
use crate::transform::{Transform, rotate_fixed};

pub const MAX_CONVEX_VERTICES: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConvexError {
    TooFewVertices,
    TooManyVertices,
    DuplicateVertex,
    CollinearEdge,
    NotConvex,
}

/// Strictly convex polygon with three to six local-space vertices.
///
/// Vertices are canonicalized to counter-clockwise order. Vertices and edge
/// normals are stored inline; no allocation is required by a dynamic body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Convex {
    vertices: [Position; MAX_CONVEX_VERTICES],
    normals: [UnitVector; MAX_CONVEX_VERTICES],
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

        let mut normals = [UnitVector::X; MAX_CONVEX_VERTICES];
        for i in 0..count {
            let [edge_x, edge_y] = (storage[(i + 1) % count] - storage[i]).raw();
            normals[i] = normalized(RawWideVec2::from_raw(edge_y, -edge_x))
                .ok_or(ConvexError::CollinearEdge)?;
        }

        Ok(Self {
            vertices: storage,
            normals,
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

    pub fn aabb(self, transform: Transform) -> Option<Aabb> {
        let vertices = self.transformed_vertices(transform)?;
        let mut min = vertices[0];
        let mut max = vertices[0];
        for point in &vertices[1..] {
            let [x, y] = point.raw();
            let [min_x, min_y] = min.raw();
            let [max_x, max_y] = max.raw();
            min = Position::from_raw(min_x.min(x), min_y.min(y));
            max = Position::from_raw(max_x.max(x), max_y.max(y));
        }
        Aabb::from_min_max(min, max)
    }

    pub(crate) fn transformed_vertices(self, transform: Transform) -> Option<TransformedVertices> {
        let mut result = TransformedVertices {
            vertices: [Position::ZERO; MAX_CONVEX_VERTICES],
            count: self.count,
        };
        for (index, vertex) in self.vertices().iter().copied().enumerate() {
            result.vertices[index] = transform.checked_apply(vertex)?;
        }
        Some(result)
    }

    pub(crate) fn transformed_normal(self, index: usize, transform: Transform) -> UnitVector {
        let raw = rotate_fixed(self.normals[index].raw(), transform.angle)
            .expect("rotating a Q30 unit vector cannot overflow i32");
        UnitVector::from_raw(raw[0], raw[1])
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TransformedVertices {
    vertices: [Position; MAX_CONVEX_VERTICES],
    count: u8,
}

impl core::ops::Deref for TransformedVertices {
    type Target = [Position];

    fn deref(&self) -> &Self::Target {
        &self.vertices[..self.count as usize]
    }
}

fn corner_cross(a: Position, b: Position, c: Position) -> i128 {
    (b - a).cross(c - b)
}

fn edge_cross(a: Position, b: Position, point: Position) -> i128 {
    (b - a).cross(point - a)
}

fn normalized(vector: RawWideVec2) -> Option<UnitVector> {
    let length = integer_sqrt(vector.squared_magnitude());
    if length == 0 {
        return None;
    }
    let [x, y] = vector.raw();
    let scale = 1_i128 << UnitVector::FRACTION_BITS;
    let nx = (x as i128 * scale) / length as i128;
    let ny = (y as i128 * scale) / length as i128;
    Some(UnitVector::from_raw(
        i32::try_from(nx).ok()?,
        i32::try_from(ny).ok()?,
    ))
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut x = 1_u128 << ((128 - value.leading_zeros() as u128 + 1) >> 1);
    loop {
        let next = (x + value / x) >> 1;
        if next >= x {
            return x;
        }
        x = next;
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
        let aabb = convex
            .aabb(Transform::new(Position::ZERO, Angle::QUARTER_TURN))
            .unwrap();

        assert_eq!(aabb.min(), Position::from_raw(-10, -20));
        assert_eq!(aabb.max(), Position::from_raw(10, 20));
    }
}
