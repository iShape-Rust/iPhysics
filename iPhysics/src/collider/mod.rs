mod circle;
mod composite;
mod convex;

use crate::geometry::Aabb;
use crate::transform::Transform;

pub use circle::Circle;
pub use composite::{ColliderPart, CompositeCollider, CompositeColliderError};
pub use convex::{Convex, ConvexError, TransformedVertices};

/// Inline collision geometry owned by a body or a composite part.
///
/// Every variant is value-stored. A circle therefore also occupies the size
/// of the largest variant, which is an intentional cache-friendly trade-off
/// for the small number of dynamic bodies targeted by this engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Collider {
    Circle(Circle),
    Convex(Convex),
}

impl Collider {
    #[inline]
    pub fn aabb(self, transform: Transform) -> Aabb {
        match self {
            Self::Circle(circle) => circle.aabb(transform.position),
            Self::Convex(convex) => convex.aabb(transform),
        }
    }

    #[inline(always)]
    pub const fn as_circle(self) -> Option<Circle> {
        match self {
            Self::Circle(circle) => Some(circle),
            Self::Convex(_) => None,
        }
    }

    #[inline(always)]
    pub const fn as_convex(self) -> Option<Convex> {
        match self {
            Self::Circle(_) => None,
            Self::Convex(convex) => Some(convex),
        }
    }
}

impl From<Circle> for Collider {
    #[inline(always)]
    fn from(circle: Circle) -> Self {
        Self::Circle(circle)
    }
}

impl From<Convex> for Collider {
    #[inline(always)]
    fn from(convex: Convex) -> Self {
        Self::Convex(convex)
    }
}
