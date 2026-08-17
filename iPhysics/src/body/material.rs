/// Minimal collision material. Friction will be added with polygon contacts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Material {
    restitution_q16: u32,
}

impl Material {
    pub const INELASTIC: Self = Self { restitution_q16: 0 };
    pub const ELASTIC: Self = Self {
        restitution_q16: 1 << 16,
    };

    /// Creates a material with restitution in the inclusive range `0..=1`.
    #[inline]
    pub fn new(restitution: f64) -> Option<Self> {
        if !restitution.is_finite() || !(0.0..=1.0).contains(&restitution) {
            return None;
        }

        Some(Self {
            restitution_q16: (restitution * 65_536.0 + 0.5) as u32,
        })
    }

    #[inline(always)]
    pub const fn restitution_raw(self) -> u32 {
        self.restitution_q16
    }
}

impl Default for Material {
    fn default() -> Self {
        Self::INELASTIC
    }
}
