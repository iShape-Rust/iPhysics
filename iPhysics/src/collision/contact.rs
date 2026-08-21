use crate::body::BodyId;
use crate::geometry::{GeometryPoint, UnitVector};
use crate::quantity::Length;

/// Stateless geometric result generated for the current tick only.
///
/// A contact deliberately carries no collider variant, composite part index,
/// or persistent feature identity. Narrow phase discards that information
/// before handing the result to the solver.
///
/// `normal` points from `body_a` toward `body_b`; the solver response applied
/// to `body_a` therefore acts in the opposite direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Contact {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub point: GeometryPoint,
    pub normal: UnitVector,
    pub penetration: Length,
}

impl Contact {
    /// Reverses the contact endpoints while preserving the same geometry.
    #[inline(always)]
    pub(crate) fn flipped(self) -> Self {
        Self {
            body_a: self.body_b,
            body_b: self.body_a,
            normal: -self.normal,
            ..self
        }
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
}
