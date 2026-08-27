use super::{DEFAULT_JOINT_RESPONSE_Q16, inverse_transform_point};
use crate::body::BodyId;
use crate::geometry::GeometryPoint;
use crate::quantity::{Force, Position};
use crate::transform::Transform;

/// A spring-like constraint between a point on a dynamic body and a
/// world-space target, intended for pointer-driven dragging.
///
/// The joint stores only deterministic fixed-point state. The target may be
/// updated before every simulation tick without rebuilding the joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MouseJoint {
    body: BodyId,
    local_anchor: Position,
    target: Position,
    max_force: Force,
    response_q16: u32,
}

impl MouseJoint {
    /// The anchor closes one quarter of the remaining distance per tick when
    /// the force limit does not bind.
    pub const DEFAULT_RESPONSE_Q16: u32 = DEFAULT_JOINT_RESPONSE_Q16;

    #[inline(always)]
    pub const fn new(
        body: BodyId,
        local_anchor: Position,
        target: Position,
        max_force: Force,
    ) -> Self {
        Self {
            body,
            local_anchor,
            target,
            max_force,
            response_q16: Self::DEFAULT_RESPONSE_Q16,
        }
    }

    /// Creates a joint whose local anchor is derived from the selected
    /// world-space point. This is the usual constructor for mouse dragging.
    #[inline]
    pub fn at_world_point(
        body: BodyId,
        body_transform: Transform,
        world_anchor: Position,
        max_force: Force,
    ) -> Self {
        Self::new(
            body,
            inverse_transform_point(body_transform, world_anchor),
            world_anchor,
            max_force,
        )
    }

    #[inline(always)]
    pub const fn body(self) -> BodyId {
        self.body
    }

    #[inline(always)]
    pub const fn local_anchor(self) -> Position {
        self.local_anchor
    }

    #[inline(always)]
    pub const fn target(self) -> Position {
        self.target
    }

    #[inline(always)]
    pub const fn max_force(self) -> Force {
        self.max_force
    }

    #[inline(always)]
    pub const fn response_raw(self) -> u32 {
        self.response_q16
    }

    /// Sets the fraction of positional error corrected per tick in unsigned
    /// Q16. Values above `1.0` are clamped to `1.0`.
    #[inline(always)]
    pub fn set_response_raw(&mut self, response_q16: u32) {
        self.response_q16 = response_q16.min(1 << 16);
    }

    #[inline(always)]
    pub fn set_target(&mut self, target: Position) {
        self.target = target;
    }

    #[inline(always)]
    pub fn set_max_force(&mut self, max_force: Force) {
        self.max_force = max_force;
    }

    /// Returns the current world-space body anchor.
    #[inline(always)]
    pub fn world_anchor(self, body_transform: Transform) -> GeometryPoint {
        body_transform.apply_geometry(self.local_anchor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Angle;

    #[test]
    fn world_constructor_preserves_rotated_anchor() {
        let transform = Transform::new(
            Position::from_meters(2.0, 3.0).unwrap(),
            Angle::QUARTER_TURN,
        );
        let anchor = Position::from_meters(1.5, 4.0).unwrap();
        let joint = MouseJoint::at_world_point(
            BodyId::new(1),
            transform,
            anchor,
            Force::from_newtons(10.0).unwrap(),
        );

        assert_eq!(joint.local_anchor().to_meters(), [1.0, 0.5]);
        assert_eq!(joint.world_anchor(transform), GeometryPoint::from(anchor));
    }

    #[test]
    fn response_is_bounded_to_one_tick() {
        let mut joint =
            MouseJoint::new(BodyId::new(1), Position::ZERO, Position::ZERO, Force::ZERO);
        joint.set_response_raw(u32::MAX);

        assert_eq!(joint.response_raw(), 1 << 16);
    }
}
