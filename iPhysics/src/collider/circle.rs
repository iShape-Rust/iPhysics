use crate::geometry::Aabb;
use crate::quantity::{Length, Position};

/// Circle centered at its collider transform origin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Circle {
    radius: Length,
}

impl Circle {
    #[inline]
    pub const fn new(radius: Length) -> Option<Self> {
        if radius.raw() == 0 {
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
    pub fn aabb(self, center: Position) -> Option<Aabb> {
        let [x, y] = center.raw();
        let r = self.radius.raw() as i64;
        let min_x = i32::try_from(x as i64 - r).ok()?;
        let max_x = i32::try_from(x as i64 + r).ok()?;
        let min_y = i32::try_from(y as i64 - r).ok()?;
        let max_y = i32::try_from(y as i64 + r).ok()?;

        Aabb::from_min_max(
            Position::from_raw(min_x, min_y),
            Position::from_raw(max_x, max_y),
        )
    }
}
