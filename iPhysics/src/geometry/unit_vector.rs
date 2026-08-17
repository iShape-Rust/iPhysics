/// Dimensionless normalized direction stored as signed Q30 components.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct UnitVector {
    x: i32,
    y: i32,
}

impl UnitVector {
    pub const FRACTION_BITS: u32 = 30;
    pub const X: Self = Self { x: 1 << 30, y: 0 };
    pub const Y: Self = Self { x: 0, y: 1 << 30 };

    #[inline(always)]
    pub(crate) const fn from_raw(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }
}
