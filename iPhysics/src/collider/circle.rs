use crate::geometry::Aabb;
use crate::quantity::{Length, Position};

/// Circle centered at its collider transform origin.
///
/// Radius is limited to `Position::MAX_RAW` in Q16 so two radii, their square,
/// and all circle narrow-phase intermediates fit in 64 bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Circle {
    radius: Length,
}

impl Circle {
    #[inline]
    pub const fn new(radius: Length) -> Option<Self> {
        if radius.raw() == 0 || radius.raw() > Position::MAX_RAW as u32 {
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
    pub fn aabb(self, center: Position) -> Aabb {
        let [x, y] = center.raw();
        let r = self.radius.raw() as i64;
        Aabb::from_points(
            Position::from_wide_narrow(x as i64 - r, y as i64 - r),
            Position::from_wide_narrow(x as i64 + r, y as i64 + r),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radius_respects_world_limit() {
        assert!(Circle::new(Length::from_raw(Position::MAX_RAW as u32)).is_some());
        assert!(Circle::new(Length::from_raw(Position::MAX_RAW as u32 + 1)).is_none());
    }
}
