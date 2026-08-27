use super::{DEFAULT_JOINT_RESPONSE_Q16, inverse_transform_point, length_between};
use crate::body::BodyId;
use crate::geometry::GeometryPoint;
use crate::quantity::{Force, Length, Position};
use crate::transform::Transform;

/// A bilateral scalar constraint that keeps two body anchors at a fixed
/// distance.
///
/// Endpoints are stored in ascending body-ID order. The canonical order fixes
/// the solver axis from anchor A to anchor B independently of insertion and
/// constructor argument order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DistanceJoint {
    body_a: BodyId,
    body_b: BodyId,
    local_anchor_a: Position,
    local_anchor_b: Position,
    length: Length,
    max_force: Force,
    response_q16: u32,
}

impl DistanceJoint {
    pub const DEFAULT_RESPONSE_Q16: u32 = DEFAULT_JOINT_RESPONSE_Q16;

    #[inline(always)]
    pub const fn new(
        body_a: BodyId,
        local_anchor_a: Position,
        body_b: BodyId,
        local_anchor_b: Position,
        length: Length,
        max_force: Force,
    ) -> Self {
        if body_a.raw() <= body_b.raw() {
            Self {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                length,
                max_force,
                response_q16: Self::DEFAULT_RESPONSE_Q16,
            }
        } else {
            Self {
                body_a: body_b,
                body_b: body_a,
                local_anchor_a: local_anchor_b,
                local_anchor_b: local_anchor_a,
                length,
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
        length: Length,
        max_force: Force,
    ) -> Self {
        Self::new(
            body_a,
            inverse_transform_point(transform_a, world_anchor_a),
            body_b,
            inverse_transform_point(transform_b, world_anchor_b),
            length,
            max_force,
        )
    }

    /// Builds local anchors and uses their current separation as the target
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
        let length = length_between(world_anchor_a, world_anchor_b)?;
        Some(Self::at_world_points(
            body_a,
            transform_a,
            world_anchor_a,
            body_b,
            transform_b,
            world_anchor_b,
            length,
            max_force,
        ))
    }

    two_body_joint_accessors!(length, set_length);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::Angle;

    #[test]
    fn world_point_constructors_preserve_both_rotated_anchors() {
        let transform_a = Transform::new(
            Position::from_meters(-2.0, 1.0).unwrap(),
            Angle::QUARTER_TURN,
        );
        let transform_b = Transform::new(
            Position::from_meters(3.0, -1.0).unwrap(),
            Angle::THREE_QUARTER_TURN,
        );
        let anchor_a = Position::from_meters(-2.5, 2.0).unwrap();
        let anchor_b = Position::from_meters(3.5, 0.0).unwrap();
        let joint = DistanceJoint::between_world_points(
            BodyId::new(1),
            transform_a,
            anchor_a,
            BodyId::new(2),
            transform_b,
            anchor_b,
            Force::from_newtons(10.0).unwrap(),
        )
        .unwrap();

        assert_eq!(
            joint.world_anchor_a(transform_a),
            GeometryPoint::from(anchor_a)
        );
        assert_eq!(
            joint.world_anchor_b(transform_b),
            GeometryPoint::from(anchor_b)
        );
        assert!((joint.length().to_meters() - 6.324_55).abs() < 0.000_02);
    }

    #[test]
    fn response_is_bounded_to_one_tick() {
        let mut joint = DistanceJoint::new(
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
