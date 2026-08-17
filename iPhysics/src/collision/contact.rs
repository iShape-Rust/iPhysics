use crate::body::BodyId;
use crate::geometry::UnitVector;
use crate::quantity::{Length, Position};

/// Stateless geometric result generated for the current tick only.
///
/// A contact deliberately carries no collider variant, composite part index,
/// or persistent feature identity. Narrow phase discards that information
/// before handing the result to the solver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Contact {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub point: Position,
    pub normal: UnitVector,
    pub penetration: Length,
}
