use crate::body::BodyId;
use crate::geometry::{GeometryPoint, UnitVector};
use crate::quantity::Length;

/// `normal` points from `body_a` toward `body_b`; the solver response applied
/// to `body_a` therefore acts in the opposite direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Contact {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub point: GeometryPoint,
    pub normal: UnitVector,
    pub penetration: Length,
    pub(crate) feature_a: ColliderFeature,
    pub(crate) feature_b: ColliderFeature,
    pub(crate) part_a: Option<usize>,
    pub(crate) part_b: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) enum ColliderFeature {
    Circle,
    ConvexVertex(u8),
    ConvexEdge(u8),
}

impl Contact {
    /// Reverses the contact endpoints while preserving the same geometry.
    #[inline(always)]
    pub(crate) fn flipped(self) -> Self {
        Self {
            body_a: self.body_b,
            body_b: self.body_a,
            normal: -self.normal,
            feature_a: self.feature_b,
            feature_b: self.feature_a,
            part_a: self.part_b,
            part_b: self.part_a,
            ..self
        }
    }

    #[inline(always)]
    pub(crate) fn with_parts(mut self, part_a: Option<usize>, part_b: Option<usize>) -> Self {
        self.part_a = part_a;
        self.part_b = part_b;
        self
    }
}

/// Fixed-capacity contact manifold produced by one collider pair.
///
/// Curved contacts have one point. Two overlapping polygon faces keep both
/// ends of their clipped overlap so the solver can balance torque.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ContactManifold {
    first: Contact,
    second: Option<Contact>,
}

impl ContactManifold {
    #[inline(always)]
    pub(crate) const fn one(contact: Contact) -> Self {
        Self {
            first: contact,
            second: None,
        }
    }

    #[inline(always)]
    pub(crate) const fn two(first: Contact, second: Contact) -> Self {
        Self {
            first,
            second: Some(second),
        }
    }

    #[cfg(test)]
    #[inline(always)]
    pub(crate) const fn first(self) -> Contact {
        self.first
    }

    #[inline(always)]
    pub(crate) fn into_contacts(self) -> impl Iterator<Item = Contact> {
        [Some(self.first), self.second].into_iter().flatten()
    }

    #[inline(always)]
    pub(crate) fn with_parts(self, part_a: Option<usize>, part_b: Option<usize>) -> Self {
        Self {
            first: self.first.with_parts(part_a, part_b),
            second: self
                .second
                .map(|contact| contact.with_parts(part_a, part_b)),
        }
    }
}
