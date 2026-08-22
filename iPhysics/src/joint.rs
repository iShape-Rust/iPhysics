use crate::body::BodyId;
use crate::geometry::GeometryPoint;
use crate::ops::shift::RoundShift;
use crate::quantity::{Force, Length, Position};
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
    pub const DEFAULT_RESPONSE_Q16: u32 = 1 << 14;

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

macro_rules! joint_accessors {
    ($length:ident, $set_length:ident) => {
        #[inline(always)]
        pub const fn body_a(self) -> BodyId {
            self.body_a
        }

        #[inline(always)]
        pub const fn body_b(self) -> BodyId {
            self.body_b
        }

        #[inline(always)]
        pub const fn local_anchor_a(self) -> Position {
            self.local_anchor_a
        }

        #[inline(always)]
        pub const fn local_anchor_b(self) -> Position {
            self.local_anchor_b
        }

        #[inline(always)]
        pub const fn $length(self) -> Length {
            self.$length
        }

        #[inline(always)]
        pub const fn max_force(self) -> Force {
            self.max_force
        }

        #[inline(always)]
        pub const fn response_raw(self) -> u32 {
            self.response_q16
        }

        #[inline(always)]
        pub fn $set_length(&mut self, length: Length) {
            self.$length = length;
        }

        #[inline(always)]
        pub fn set_max_force(&mut self, max_force: Force) {
            self.max_force = max_force;
        }

        #[inline(always)]
        pub fn set_response_raw(&mut self, response_q16: u32) {
            self.response_q16 = response_q16.min(1 << 16);
        }

        #[inline(always)]
        pub fn world_anchor_a(self, transform: Transform) -> GeometryPoint {
            transform.apply_geometry(self.local_anchor_a)
        }

        #[inline(always)]
        pub fn world_anchor_b(self, transform: Transform) -> GeometryPoint {
            transform.apply_geometry(self.local_anchor_b)
        }
    };
}

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
    pub const DEFAULT_RESPONSE_Q16: u32 = MouseJoint::DEFAULT_RESPONSE_Q16;

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

    joint_accessors!(length, set_length);
}

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
    pub const DEFAULT_RESPONSE_Q16: u32 = MouseJoint::DEFAULT_RESPONSE_Q16;

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

    joint_accessors!(max_length, set_max_length);
}

#[inline]
fn length_between(a: Position, b: Position) -> Option<Length> {
    let raw = (b - a).squared_magnitude().isqrt();
    if raw > Length::MAX_LENGTH as u64 {
        None
    } else {
        Some(Length::from_raw(raw as u32))
    }
}

fn inverse_transform_point(transform: Transform, world: Position) -> Position {
    let [world_x, world_y] = world.raw();
    let [center_x, center_y] = transform.position.raw();
    let dx = world_x as i64 - center_x as i64;
    let dy = world_y as i64 - center_y as i64;
    let [sin, cos] = transform.angle.sin_cos_q30();

    let local_x = (dx * cos as i64 + dy * sin as i64).round_shift(30);
    let local_y = (-dx * sin as i64 + dy * cos as i64).round_shift(30);
    Position::from_i64(local_x, local_y)
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

    #[test]
    fn two_body_joints_canonicalize_endpoints_and_anchors() {
        let anchor_a = Position::from_meters(0.25, 0.0).unwrap();
        let anchor_b = Position::from_meters(-0.5, 0.0).unwrap();
        let distance = DistanceJoint::new(
            BodyId::new(9),
            anchor_a,
            BodyId::new(2),
            anchor_b,
            Length::from_meters(1.0).unwrap(),
            Force::from_newtons(10.0).unwrap(),
        );
        let rope = RopeJoint::new(
            BodyId::new(9),
            anchor_a,
            BodyId::new(2),
            anchor_b,
            Length::from_meters(1.0).unwrap(),
            Force::from_newtons(10.0).unwrap(),
        );

        assert_eq!(
            (distance.body_a(), distance.body_b()),
            (BodyId::new(2), BodyId::new(9))
        );
        assert_eq!(distance.local_anchor_a(), anchor_b);
        assert_eq!(distance.local_anchor_b(), anchor_a);
        assert_eq!(
            (rope.body_a(), rope.body_b()),
            (BodyId::new(2), BodyId::new(9))
        );
        assert_eq!(rope.local_anchor_a(), anchor_b);
        assert_eq!(rope.local_anchor_b(), anchor_a);
    }

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
    fn distance_and_rope_response_are_bounded() {
        let mut distance = DistanceJoint::new(
            BodyId::new(1),
            Position::ZERO,
            BodyId::new(2),
            Position::ZERO,
            Length::ZERO,
            Force::ZERO,
        );
        let mut rope = RopeJoint::new(
            BodyId::new(1),
            Position::ZERO,
            BodyId::new(2),
            Position::ZERO,
            Length::ZERO,
            Force::ZERO,
        );

        distance.set_response_raw(u32::MAX);
        rope.set_response_raw(u32::MAX);

        assert_eq!(distance.response_raw(), 1 << 16);
        assert_eq!(rope.response_raw(), 1 << 16);
    }
}
