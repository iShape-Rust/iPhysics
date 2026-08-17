use super::{BodyId, Material};
use crate::collider::CompositeCollider;
use crate::geometry::Aabb;
use crate::transform::Transform;

/// Immutable collision geometry that carries no dynamic simulation state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StaticBody {
    id: BodyId,
    transform: Transform,
    collider: CompositeCollider,
    material: Material,
    aabb: Aabb,
}

impl StaticBody {
    #[inline(always)]
    pub fn new(
        id: BodyId,
        transform: Transform,
        collider: CompositeCollider,
        material: Material,
    ) -> Self {
        let aabb = collider.aabb(transform);
        Self {
            id,
            transform,
            collider,
            material,
            aabb,
        }
    }

    #[inline(always)]
    pub const fn id(&self) -> BodyId {
        self.id
    }

    #[inline(always)]
    pub const fn transform(&self) -> Transform {
        self.transform
    }

    #[inline(always)]
    pub const fn collider(&self) -> &CompositeCollider {
        &self.collider
    }

    #[inline(always)]
    pub const fn material(&self) -> Material {
        self.material
    }

    #[inline(always)]
    pub const fn aabb(&self) -> Aabb {
        self.aabb
    }
}
