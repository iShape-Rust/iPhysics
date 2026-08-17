use super::Collider;
use crate::geometry::Aabb;
use crate::transform::Transform;
use alloc::vec::Vec;

/// One collider positioned relative to its composite owner.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ColliderPart {
    local_transform: Transform,
    collider: Collider,
}

impl ColliderPart {
    #[inline(always)]
    pub const fn new(local_transform: Transform, collider: Collider) -> Self {
        Self {
            local_transform,
            collider,
        }
    }

    #[inline(always)]
    pub const fn local_transform(self) -> Transform {
        self.local_transform
    }

    #[inline(always)]
    pub const fn collider(self) -> Collider {
        self.collider
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompositeColliderError {
    Empty,
}

/// Arbitrary number of collider parts owned by one static body.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompositeCollider {
    parts: Vec<ColliderPart>,
    local_aabb: Aabb,
}

impl CompositeCollider {
    pub fn new(parts: Vec<ColliderPart>) -> Result<Self, CompositeColliderError> {
        let mut iter = parts.iter().copied();
        let first = iter.next().ok_or(CompositeColliderError::Empty)?;
        let mut local_aabb = first.collider.aabb(first.local_transform);
        for part in iter {
            let part_aabb = part.collider.aabb(part.local_transform);
            local_aabb = local_aabb.union(part_aabb);
        }

        Ok(Self { parts, local_aabb })
    }

    pub fn single(collider: Collider) -> Result<Self, CompositeColliderError> {
        Self::new(alloc::vec![ColliderPart::new(
            Transform::IDENTITY,
            collider
        )])
    }

    #[inline(always)]
    pub fn parts(&self) -> &[ColliderPart] {
        &self.parts
    }

    #[inline(always)]
    pub const fn local_aabb(&self) -> Aabb {
        self.local_aabb
    }

    pub fn aabb(&self, transform: Transform) -> Aabb {
        let mut iter = self.parts.iter().copied();
        let first = iter
            .next()
            .expect("a composite collider always contains at least one part");
        let first_transform = transform.compose(first.local_transform);
        let mut result = first.collider.aabb(first_transform);
        for part in iter {
            let part_transform = transform.compose(part.local_transform);
            result = result.union(part.collider.aabb(part_transform));
        }
        result
    }
}
