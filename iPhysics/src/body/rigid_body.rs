use super::{BodyId, BodyState, Material};
use crate::collider::Collider;
use crate::quantity::Mass;

/// Dynamic rigid body with one inline collider.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Body {
    id: BodyId,
    collider: Collider,
    material: Material,
    inverse_mass_q24: u32,
    state: BodyState,
}

impl Body {
    #[inline(always)]
    pub fn dynamic(
        id: BodyId,
        collider: impl Into<Collider>,
        mass: Mass,
        material: Material,
        state: BodyState,
    ) -> Self {
        Self {
            id,
            collider: collider.into(),
            material,
            inverse_mass_q24: mass.inverse_q24(),
            state,
        }
    }

    #[inline(always)]
    pub const fn id(&self) -> BodyId {
        self.id
    }

    #[inline(always)]
    pub const fn collider(&self) -> Collider {
        self.collider
    }

    #[inline(always)]
    pub const fn material(&self) -> Material {
        self.material
    }

    #[inline(always)]
    pub const fn state(&self) -> &BodyState {
        &self.state
    }

    #[inline(always)]
    pub fn state_mut(&mut self) -> &mut BodyState {
        &mut self.state
    }

    #[inline(always)]
    pub(crate) const fn inverse_mass_q24(&self) -> u32 {
        if self.state.is_sleeping() {
            0
        } else {
            self.inverse_mass_q24
        }
    }
}
