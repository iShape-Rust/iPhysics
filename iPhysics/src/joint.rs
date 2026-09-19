use crate::ops::shift::RoundShift;
use crate::quantity::{Length, Position};
use crate::transform::Transform;

pub use distance::DistanceJoint;
pub use mouse::MouseJoint;
pub use rope::RopeJoint;

pub(super) const DEFAULT_JOINT_RESPONSE_Q16: u32 = 1 << 14;

macro_rules! two_body_joint_accessors {
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

mod distance;
mod mouse;
mod rope;

#[inline]
pub(super) fn length_between(a: Position, b: Position) -> Option<Length> {
    let raw = (b - a).squared_magnitude().isqrt();
    Length::from_raw(u32::try_from(raw).ok()?)
}

pub(super) fn inverse_transform_point(transform: Transform, world: Position) -> Position {
    let [world_x, world_y] = world.raw();
    let [center_x, center_y] = transform.position.raw();
    let dx = world_x as i64 - center_x as i64;
    let dy = world_y as i64 - center_y as i64;
    let (sin, cos) = transform.angle.sin_cos();

    let local_x = (dx * cos as i64 + dy * sin as i64).round_shift(30);
    let local_y = (-dx * sin as i64 + dy * cos as i64).round_shift(30);
    Position::from_i64(local_x, local_y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::BodyId;
    use crate::quantity::Force;

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
}
