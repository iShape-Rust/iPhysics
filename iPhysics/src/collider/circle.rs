use crate::geometry::Aabb;
use crate::quantity::{Length, Position};

/// Circle centered at its collider transform origin.
///
/// Radius is limited to `Position::MAX_POS` in Q16 so two radii, their square,
/// and all circle narrow-phase intermediates fit in 64 bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Circle {
    radius: Length,
}

impl Circle {
    #[inline]
    pub const fn new(radius: Length) -> Option<Self> {
        if radius.raw() == 0 || radius.raw() > Position::MAX_POSITION as u32 {
            None
        } else {
            Some(Self { radius })
        }
    }

    #[inline(always)]
    pub const fn radius(self) -> Length {
        self.radius
    }

    #[inline]
    pub(crate) fn aabb(self, center: Position) -> Aabb {
        let [x, y] = center.raw();
        // Center coordinates and radius are bounded by 2^29 - 1, so their
        // sums and differences are bounded by 2^30 - 2 and fit in i32.
        let r = self.radius.raw() as i32;
        Aabb::from_raw_unchecked(x - r, x + r, y - r, y + r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radius_respects_world_limit() {
        assert!(Circle::new(Length::ZERO).is_none());
        assert!(Circle::new(Length::from_raw(Position::MAX_POSITION as u32)).is_some());
        assert!(Circle::new(Length::from_raw(Position::MAX_POSITION as u32 + 1)).is_none());
    }

    #[test]
    fn aabb_can_extend_beyond_position_range() {
        let circle = Circle::new(Length::from_raw(Position::MAX_POSITION as u32)).unwrap();
        let center = Position::from_i32(Position::MAX_POSITION, Position::MAX_POSITION);
        let aabb = circle.aabb(center);

        assert_eq!(aabb.max().raw()[0], 2 * Position::MAX_POSITION);
        assert!(aabb.max().raw()[0] > Position::MAX_POSITION);
    }
}
