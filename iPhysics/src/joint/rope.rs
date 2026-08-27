use super::{DEFAULT_JOINT_RESPONSE_Q16, inverse_transform_point, length_between};
use crate::body::BodyId;
use crate::geometry::GeometryPoint;
use crate::quantity::{Force, Length, Position};
use crate::transform::Transform;

/// A unilateral scalar constraint that prevents two body anchors from moving
/// farther apart than a maximum distance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RopeJoint {
    body_a: BodyId,
    body_b: BodyId,
    local_anchor_a: Position,
    local_anchor_b: Position,
    max_length: Length,
    max_force: Force,
    response_q16: u32,
}

impl RopeJoint {
    pub const DEFAULT_RESPONSE_Q16: u32 = DEFAULT_JOINT_RESPONSE_Q16;

    #[inline(always)]
    pub const fn new(
        body_a: BodyId,
        local_anchor_a: Position,
        body_b: BodyId,
        local_anchor_b: Position,
        max_length: Length,
        max_force: Force,
    ) -> Self {
        if body_a.raw() <= body_b.raw() {
            Self {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                max_length,
                max_force,
                response_q16: Self::DEFAULT_RESPONSE_Q16,
            }
        } else {
            Self {
                body_a: body_b,
                body_b: body_a,
                local_anchor_a: local_anchor_b,
                local_anchor_b: local_anchor_a,
                max_length,
                max_force,
                response_q16: Self::DEFAULT_RESPONSE_Q16,
            }
        }
    }

    /// Builds local anchors from two world-space anchor points.
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn at_world_points(
        body_a: BodyId,
        transform_a: Transform,
        world_anchor_a: Position,
        body_b: BodyId,
        transform_b: Transform,
        world_anchor_b: Position,
        max_length: Length,
        max_force: Force,
    ) -> Self {
        Self::new(
            body_a,
            inverse_transform_point(transform_a, world_anchor_a),
            body_b,
            inverse_transform_point(transform_b, world_anchor_b),
            max_length,
            max_force,
        )
    }

    /// Builds local anchors and uses their current separation as the maximum
    /// length. Returns `None` when that separation exceeds [`Length`]'s range.
    #[inline]
    pub fn between_world_points(
        body_a: BodyId,
        transform_a: Transform,
        world_anchor_a: Position,
        body_b: BodyId,
        transform_b: Transform,
        world_anchor_b: Position,
        max_force: Force,
    ) -> Option<Self> {
        let max_length = length_between(world_anchor_a, world_anchor_b)?;
        Some(Self::at_world_points(
            body_a,
            transform_a,
            world_anchor_a,
            body_b,
            transform_b,
            world_anchor_b,
            max_length,
            max_force,
        ))
    }

    two_body_joint_accessors!(max_length, set_max_length);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn response_is_bounded_to_one_tick() {
        let mut joint = RopeJoint::new(
            BodyId::new(1),
            Position::ZERO,
            BodyId::new(2),
            Position::ZERO,
            Length::ZERO,
            Force::ZERO,
        );
        joint.set_response_raw(u32::MAX);

        assert_eq!(joint.response_raw(), 1 << 16);
    }
}
