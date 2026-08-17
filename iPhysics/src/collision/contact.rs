use crate::body::BodyId;
use crate::geometry::UnitVector;
use crate::quantity::{Length, Position};

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
    pub point: Position,
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
