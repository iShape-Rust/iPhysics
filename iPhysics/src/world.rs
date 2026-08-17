mod settings;
mod simulation;

use crate::body::{Body, BodyId, StaticBody};
use crate::collision::Contact;
use alloc::vec::Vec;

pub use settings::WorldSettings;
pub use simulation::{StepError, StepStats};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddBodyError {
    DuplicateId(BodyId),
}

/// Deterministic collection of classical rigid bodies.
///
/// Bodies remain sorted by their stable ID, which defines the canonical
/// collision-pair order used by the simulation.
#[derive(Debug, Clone)]
pub struct World {
    settings: WorldSettings,
    bodies: Vec<Body>,
    static_bodies: Vec<StaticBody>,
    contacts: Vec<Contact>,
    contact_pairs: Vec<ContactPair>,
}

/// Solver-only body lookup kept parallel to `contacts`; it intentionally has
/// no collider-part identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ContactPair {
    pub(super) a: usize,
    pub(super) b: ContactBodyIndex,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ContactBodyIndex {
    Dynamic(usize),
    Static(usize),
}

impl World {
    #[inline(always)]
    pub const fn new(settings: WorldSettings) -> Self {
        Self {
            settings,
            bodies: Vec::new(),
            static_bodies: Vec::new(),
            contacts: Vec::new(),
            contact_pairs: Vec::new(),
        }
    }

    /// Returns bodies in ascending ID order.
    #[inline(always)]
    pub fn bodies(&self) -> &[Body] {
        &self.bodies
    }

    #[inline(always)]
    pub const fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Returns static bodies in ascending ID order.
    #[inline(always)]
    pub fn static_bodies(&self) -> &[StaticBody] {
        &self.static_bodies
    }

    #[inline(always)]
    pub const fn static_body_count(&self) -> usize {
        self.static_bodies.len()
    }

    #[inline(always)]
    pub fn contacts(&self) -> &[Contact] {
        &self.contacts
    }

    #[inline]
    pub fn body(&self, id: BodyId) -> Option<&Body> {
        let index = self.bodies.binary_search_by_key(&id, Body::id).ok()?;
        Some(&self.bodies[index])
    }

    #[inline]
    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut Body> {
        let index = self.bodies.binary_search_by_key(&id, Body::id).ok()?;
        Some(&mut self.bodies[index])
    }

    #[inline]
    pub fn static_body(&self, id: BodyId) -> Option<&StaticBody> {
        let index = self
            .static_bodies
            .binary_search_by_key(&id, StaticBody::id)
            .ok()?;
        Some(&self.static_bodies[index])
    }

    /// Inserts a body while preserving ascending ID order.
    pub fn add_body(&mut self, body: Body) -> Result<(), AddBodyError> {
        if self.static_body(body.id()).is_some() {
            return Err(AddBodyError::DuplicateId(body.id()));
        }
        match self.bodies.binary_search_by_key(&body.id(), Body::id) {
            Ok(_) => Err(AddBodyError::DuplicateId(body.id())),
            Err(index) => {
                self.bodies.insert(index, body);
                self.clear_contacts();
                Ok(())
            }
        }
    }

    /// Inserts a static body while preserving ascending ID order. IDs share
    /// one namespace with dynamic bodies because contacts expose only BodyId.
    pub fn add_static_body(&mut self, body: StaticBody) -> Result<(), AddBodyError> {
        if self.body(body.id()).is_some() {
            return Err(AddBodyError::DuplicateId(body.id()));
        }
        match self
            .static_bodies
            .binary_search_by_key(&body.id(), StaticBody::id)
        {
            Ok(_) => Err(AddBodyError::DuplicateId(body.id())),
            Err(index) => {
                self.static_bodies.insert(index, body);
                self.clear_contacts();
                Ok(())
            }
        }
    }

    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let index = self.bodies.binary_search_by_key(&id, Body::id).ok()?;
        let body = self.bodies.remove(index);
        self.clear_contacts();
        Some(body)
    }

    pub fn remove_static_body(&mut self, id: BodyId) -> Option<StaticBody> {
        let index = self
            .static_bodies
            .binary_search_by_key(&id, StaticBody::id)
            .ok()?;
        let body = self.static_bodies.remove(index);
        self.clear_contacts();
        Some(body)
    }

    fn clear_contacts(&mut self) {
        self.contacts.clear();
        self.contact_pairs.clear();
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new(WorldSettings::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyState, Material};
    use crate::collider::{Circle, CompositeCollider};
    use crate::quantity::{
        Angle, AngularVelocity, Length, LinearAcceleration, LinearVelocity, Mass, Position,
    };
    use crate::transform::Transform;

    fn circle_body(id: u64) -> Body {
        Body::dynamic(
            BodyId::new(id),
            Circle::new(Length::from_meters(0.5).unwrap()).unwrap(),
            Mass::ONE,
            Material::INELASTIC,
            BodyState::new(
                Transform::new(Position::ZERO, Angle::ZERO),
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
            ),
        )
    }

    fn world() -> World {
        World::new(WorldSettings::new(LinearAcceleration::ZERO))
    }

    #[test]
    fn bodies_are_sorted_and_found_by_stable_id() {
        let mut world = world();
        world.add_body(circle_body(9)).unwrap();
        world.add_body(circle_body(2)).unwrap();

        assert_eq!(world.bodies()[0].id(), BodyId::new(2));
        assert_eq!(world.bodies()[1].id(), BodyId::new(9));
        assert_eq!(world.body(BodyId::new(9)).unwrap().id(), BodyId::new(9));

        assert_eq!(
            world.remove_body(BodyId::new(2)).unwrap().id(),
            BodyId::new(2)
        );
        assert!(world.body(BodyId::new(2)).is_none());
    }

    #[test]
    fn duplicate_body_id_is_rejected() {
        let mut world = world();
        world.add_body(circle_body(7)).unwrap();

        assert_eq!(
            world.add_body(circle_body(7)),
            Err(AddBodyError::DuplicateId(BodyId::new(7)))
        );
    }

    #[test]
    fn dynamic_and_static_bodies_share_id_namespace() {
        let mut world = world();
        world.add_body(circle_body(7)).unwrap();
        let static_body = StaticBody::new(
            BodyId::new(7),
            Transform::IDENTITY,
            CompositeCollider::single(
                Circle::new(Length::from_meters(1.0).unwrap())
                    .unwrap()
                    .into(),
            )
            .unwrap(),
            Material::INELASTIC,
        )
        .unwrap();

        assert_eq!(
            world.add_static_body(static_body),
            Err(AddBodyError::DuplicateId(BodyId::new(7)))
        );
    }
}
