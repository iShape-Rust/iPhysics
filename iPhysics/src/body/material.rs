use crate::ops::quantize::Quantize;

/// Collision material with Q16 restitution and Coulomb friction coefficients.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Material {
    restitution_q16: u32,
    friction_q16: u32,
}

impl Material {
    const SCALE: u32 = 1 << 16;
    const DEFAULT_FRICTION_Q16: u32 = Self::SCALE / 2;

    pub const INELASTIC: Self = Self {
        restitution_q16: 0,
        friction_q16: Self::DEFAULT_FRICTION_Q16,
    };
    pub const ELASTIC: Self = Self {
        restitution_q16: Self::SCALE,
        friction_q16: Self::DEFAULT_FRICTION_Q16,
    };

    /// Creates a material with restitution in `0..=1` and non-negative
    /// friction. Friction coefficients greater than one are allowed.
    #[inline]
    pub fn new(restitution: f64, friction: f64) -> Option<Self> {
        if !restitution.is_finite() || !(0.0..=1.0).contains(&restitution) {
            return None;
        }

        Some(Self {
            restitution_q16: restitution.quantize(16)?,
            friction_q16: friction.quantize(16)?,
        })
    }

    #[inline(always)]
    pub const fn restitution_raw(self) -> u32 {
        self.restitution_q16
    }

    #[inline(always)]
    pub const fn friction_raw(self) -> u32 {
        self.friction_q16
    }

    #[inline(always)]
    pub(crate) const fn combined_restitution_raw(self, other: Self) -> u32 {
        if self.restitution_q16 > other.restitution_q16 {
            self.restitution_q16
        } else {
            other.restitution_q16
        }
    }

    #[inline(always)]
    pub(crate) const fn combined_friction_raw(self, other: Self) -> u32 {
        (self.friction_q16 as u64 + other.friction_q16 as u64).div_ceil(2) as u32
    }
}

impl Default for Material {
    fn default() -> Self {
        Self::INELASTIC
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn material_quantizes_both_coefficients() {
        let material = Material::new(0.25, 1.5).unwrap();

        assert_eq!(material.restitution_raw(), 1 << 14);
        assert_eq!(material.friction_raw(), 3 << 15);
    }

    #[test]
    fn material_rejects_invalid_coefficients() {
        assert!(Material::new(-0.1, 0.5).is_none());
        assert!(Material::new(1.1, 0.5).is_none());
        assert!(Material::new(0.5, -0.1).is_none());
        assert!(Material::new(0.5, f64::INFINITY).is_none());
    }

    #[test]
    fn combines_contact_coefficients() {
        let a = Material::new(0.25, 0.5).unwrap();
        let b = Material::new(0.75, 1.5).unwrap();

        assert_eq!(a.combined_restitution_raw(b), 3 << 14);
        assert_eq!(a.combined_friction_raw(b), 1 << 16);
    }
}
