use crate::quantity::DiffVec2;

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

    /// Normalizes a non-zero raw vector into a deterministic Q30 direction.
    #[inline]
    pub fn normalized(vector: DiffVec2) -> Option<Self> {
        Self::normalized_with_length(vector, vector.squared_magnitude().isqrt())
    }

    /// Normalizes using a magnitude already computed by the caller.
    /// Circle collision uses this after its exact squared-distance test.
    #[inline]
    pub(crate) fn normalized_with_length(vector: DiffVec2, length: u64) -> Option<Self> {
        if length == 0 {
            return None;
        }

        let [x, y] = vector.raw();
        let scale = 1_i64 << Self::FRACTION_BITS;
        Some(Self {
            x: i32::try_from(x as i64 * scale / length as i64).ok()?,
            y: i32::try_from(y as i64 * scale / length as i64).ok()?,
        })
    }

    #[inline(always)]
    pub(crate) const fn from_raw(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub const fn raw(self) -> [i32; 2] {
        [self.x, self.y]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalizes_raw_vector_to_q30() {
        let direction = UnitVector::normalized(DiffVec2::from_raw(3, 4)).unwrap();

        assert_eq!(direction.raw(), [644_245_094, 858_993_459]);
        assert!(UnitVector::normalized(DiffVec2::ZERO).is_none());
    }
}
