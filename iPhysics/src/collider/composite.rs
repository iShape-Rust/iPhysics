use alloc::{boxed::Box, vec::Vec};

use super::{Circle, Convex};
use crate::geometry::Aabb;
use crate::transform::Transform;

/// Advisory complexity threshold checked only in debug builds.
///
/// This is not a storage or correctness limit. A release build accepts any
/// number of parts, but narrow-phase work can grow with the product of the
/// part counts of two colliding composite colliders.
pub const RECOMMENDED_MAX_COMPOSITE_PARTS: usize = 16;

/// Convex primitive supported directly by the narrow phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SimpleCollider {
    Circle(Circle),
    Convex(Convex),
}

impl From<Circle> for SimpleCollider {
    #[inline(always)]
    fn from(circle: Circle) -> Self {
        Self::Circle(circle)
    }
}

impl From<Convex> for SimpleCollider {
    #[inline(always)]
    fn from(convex: Convex) -> Self {
        Self::Convex(convex)
    }
}

impl SimpleCollider {
    #[inline]
    pub(crate) fn aabb(self, transform: Transform) -> Aabb {
        match self {
            Self::Circle(circle) => circle.aabb(transform),
            Self::Convex(convex) => convex.aabb(transform),
        }
    }

    #[inline(always)]
    pub(super) fn mass_properties(self) -> (u64, u128) {
        match self {
            Self::Circle(circle) => circle.mass_properties(),
            Self::Convex(convex) => convex.mass_properties(),
        }
    }
}

/// Immutable collection of simple colliders in body-local coordinates.
///
/// Dynamic-body mass is distributed between parts in proportion to their
/// area. Overlapping parts contribute their full area independently.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct CompositeCollider {
    simple_colliders: Box<[SimpleCollider]>,
}

impl CompositeCollider {
    /// Builds a composite collider from at least one simple collider.
    ///
    /// # Panics
    ///
    /// Panics when `simple_colliders` is empty. In debug builds it also
    /// panics above [`RECOMMENDED_MAX_COMPOSITE_PARTS`] to flag unexpectedly
    /// expensive collision geometry during development. Release builds do not
    /// impose that advisory limit.
    #[inline]
    pub fn new(simple_colliders: Vec<SimpleCollider>) -> Self {
        assert!(
            !simple_colliders.is_empty(),
            "a composite collider must contain at least one simple collider"
        );
        debug_assert!(
            simple_colliders.len() <= RECOMMENDED_MAX_COMPOSITE_PARTS,
            "composite collider contains {} parts; the recommended maximum is {} because narrow-phase work can grow as O(n * m)",
            simple_colliders.len(),
            RECOMMENDED_MAX_COMPOSITE_PARTS,
        );

        Self {
            simple_colliders: simple_colliders.into_boxed_slice(),
        }
    }

    #[inline(always)]
    pub const fn len(&self) -> usize {
        self.simple_colliders.len()
    }

    #[inline(always)]
    pub const fn is_empty(&self) -> bool {
        false
    }

    #[inline(always)]
    pub fn simple_colliders(&self) -> &[SimpleCollider] {
        &self.simple_colliders
    }

    pub(crate) fn aabb(&self, transform: Transform) -> Aabb {
        let mut colliders = self.simple_colliders.iter().copied();
        let first = colliders
            .next()
            .expect("construction guarantees a non-empty composite collider");
        colliders.fold(first.aabb(transform), |aabb, collider| {
            aabb.union(collider.aabb(transform))
        })
    }

    pub(crate) fn inverse_inertia_q40(&self, inverse_mass_q24: u32) -> u64 {
        let mut total_mass_weight = 0_u128;
        let mut total_weighted_inertia = 0_u128;
        for collider in self.simple_colliders.iter().copied() {
            let (mass_weight, weighted_inertia) = collider.mass_properties();
            total_mass_weight = total_mass_weight.saturating_add(mass_weight as u128);
            total_weighted_inertia = total_weighted_inertia.saturating_add(weighted_inertia);
        }

        super::inertia::from_q24_per_q32_ratio(
            (inverse_mass_q24 as u128).saturating_mul(total_mass_weight),
            total_weighted_inertia,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Length;
    use alloc::vec;

    fn circle() -> SimpleCollider {
        Circle::new(Length::from_raw(1).unwrap()).unwrap().into()
    }

    #[test]
    fn stores_parts_without_spare_capacity() {
        let composite = CompositeCollider::new(vec![circle(), circle()]);

        assert_eq!(composite.len(), 2);
        assert!(!composite.is_empty());
        assert_eq!(composite.simple_colliders().len(), 2);
    }

    #[test]
    #[should_panic(expected = "at least one simple collider")]
    fn rejects_empty_composite() {
        CompositeCollider::new(Vec::new());
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "recommended maximum")]
    fn debug_build_flags_excessive_part_count() {
        CompositeCollider::new(vec![circle(); RECOMMENDED_MAX_COMPOSITE_PARTS + 1]);
    }

    #[test]
    #[cfg(not(debug_assertions))]
    fn release_build_accepts_more_than_recommended_parts() {
        let composite = CompositeCollider::new(vec![circle(); RECOMMENDED_MAX_COMPOSITE_PARTS + 1]);
        assert_eq!(composite.len(), RECOMMENDED_MAX_COMPOSITE_PARTS + 1);
    }

    #[test]
    fn combines_part_aabbs() {
        let left = Circle::with_center(
            crate::quantity::Position::from_meters(-2.0, 0.0).unwrap(),
            Length::from_meters(0.5).unwrap(),
        )
        .unwrap();
        let right = Circle::with_center(
            crate::quantity::Position::from_meters(3.0, 0.0).unwrap(),
            Length::from_meters(1.0).unwrap(),
        )
        .unwrap();
        let composite = CompositeCollider::new(vec![left.into(), right.into()]);

        let aabb = composite.aabb(Transform::IDENTITY);
        assert_eq!(aabb.min().to_meters(), [-2.5, -1.0]);
        assert_eq!(aabb.max().to_meters(), [4.0, 1.0]);
    }

    #[test]
    fn distributes_mass_by_area_and_includes_part_offsets() {
        use crate::quantity::{Mass, Position};

        let radius = Length::from_meters(1.0).unwrap();
        let left = Circle::with_center(Position::from_meters(-1.0, 0.0).unwrap(), radius).unwrap();
        let right = Circle::with_center(Position::from_meters(1.0, 0.0).unwrap(), radius).unwrap();
        let composite = CompositeCollider::new(vec![left.into(), right.into()]);

        // Each equal-area part has I/m = r²/2 + d² = 3/2, so the composite
        // has the same normalized inertia and inverse inertia 2/3.
        let actual = composite.inverse_inertia_q40(Mass::ONE.inverse_q24());
        let expected = ((2_u128 << 40) / 3) as u64;
        assert!(actual.abs_diff(expected) <= 1);
    }
}
