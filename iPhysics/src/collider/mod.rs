mod circle;
mod composite;
mod convex;
mod inertia;

use crate::geometry::Aabb;
use crate::transform::Transform;

pub use circle::Circle;
pub use composite::{CompositeCollider, RECOMMENDED_MAX_COMPOSITE_PARTS, SimpleCollider};
pub use convex::{Convex, ConvexError, TransformedVertices};

/// Collision geometry owned by a body.
///
/// Simple variants remain inline and allocation-free. A composite owns one
/// immutable boxed slice containing all of its simple parts.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Collider {
    Circle(Circle),
    Convex(Convex),
    Composite(CompositeCollider),
}

impl Collider {
    #[inline]
    pub fn aabb(&self, transform: Transform) -> Aabb {
        match self {
            Self::Circle(circle) => circle.aabb(transform),
            Self::Convex(convex) => convex.aabb(transform),
            Self::Composite(composite) => composite.aabb(transform),
        }
    }

    #[inline(always)]
    pub const fn as_circle(&self) -> Option<Circle> {
        match self {
            Self::Circle(circle) => Some(*circle),
            Self::Convex(_) | Self::Composite(_) => None,
        }
    }

    #[inline(always)]
    pub const fn as_convex(&self) -> Option<Convex> {
        match self {
            Self::Circle(_) | Self::Composite(_) => None,
            Self::Convex(convex) => Some(*convex),
        }
    }

    #[inline(always)]
    pub const fn as_composite(&self) -> Option<&CompositeCollider> {
        match self {
            Self::Circle(_) | Self::Convex(_) => None,
            Self::Composite(composite) => Some(composite),
        }
    }

    /// Reciprocal moment of inertia about the collider origin as unsigned Q40.
    ///
    /// The result is derived once when a dynamic body is built. `inverse_mass_q24`
    /// is used instead of the source mass so linear and angular solver weights
    /// share the same low-mass saturation policy.
    pub(crate) fn inverse_inertia_q40(&self, inverse_mass_q24: u32) -> u64 {
        match self {
            Self::Circle(circle) => circle.inverse_inertia_q40(inverse_mass_q24),
            Self::Convex(convex) => convex.inverse_inertia_q40(inverse_mass_q24),
            Self::Composite(composite) => composite.inverse_inertia_q40(inverse_mass_q24),
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

impl From<SimpleCollider> for Collider {
    #[inline(always)]
    fn from(collider: SimpleCollider) -> Self {
        match collider {
            SimpleCollider::Circle(circle) => Self::Circle(circle),
            SimpleCollider::Convex(convex) => Self::Convex(convex),
        }
    }
}

impl From<CompositeCollider> for Collider {
    #[inline(always)]
    fn from(composite: CompositeCollider) -> Self {
        Self::Composite(composite)
    }
}
