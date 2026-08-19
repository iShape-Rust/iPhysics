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
        let r = self.radius.raw() as i64;
        let min_x = x as i64 - r;
        let max_x = x as i64 + r;
        let min_y = y as i64 - r;
        let max_y = y as i64 + r;
        debug_assert!(min_x >= Position::MIN_POINT as i64);
        debug_assert!(min_y >= Position::MIN_POINT as i64);
        debug_assert!(max_x <= Position::MAX_POINT as i64);
        debug_assert!(max_y <= Position::MAX_POINT as i64);
        Aabb::from_raw_unchecked(min_x as i32, max_x as i32, min_y as i32, max_y as i32)
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
