use crate::collision::CollisionSolver;
use crate::world::simulation::constraint::relative_speed_along;
use crate::world::{BodyRef, SleepSupport};
use crate::{Body, BodyId, LinearAcceleration, RopeJoint, StepStats, UnitVector, World};
use alloc::{vec, vec::Vec};
use core::mem;

use super::IMPACT_SPEED_RAW;

const WAKE_PENETRATION_RAW: u32 = 655; // approximately 0.01 m in Q16

impl World {
    pub(crate) fn update_sleep_states(&mut self, stats: &mut StepStats) {
        let gravity = self.settings.gravity;
        let mut has_support_contact = vec![false; self.bodies.len()];
        for contact in self.active_static_contacts.iter().copied() {
            if gravity.is_zero() || normal_supports_body_a(contact.normal, gravity) {
                has_support_contact[contact.body_a] = true;
            }
        }
        for contact in self.active_dynamic_contacts.iter().copied() {
            if gravity.is_zero() {
                has_support_contact[contact.body_a] = true;
                has_support_contact[contact.body_b] = true;
            } else if normal_supports_body_a(contact.normal, gravity) {
                has_support_contact[contact.body_a] = true;
            } else if normal_supports_body_a(-contact.normal, gravity) {
                has_support_contact[contact.body_b] = true;
            }
        }

        let mut has_mouse_joint = vec![false; self.bodies.len()];
        for joint in &self.mouse_joints {
            if let Ok(index) = self.bodies.binary_search_by_key(&joint.body(), Body::id) {
                has_mouse_joint[index] = true;
            }
        }

        let mut has_joint_constraint = vec![false; self.bodies.len()];
        self.mark_sleep_constraints(&mut has_joint_constraint);

        for (index, body) in self.bodies.iter_mut().enumerate() {
            body.state_mut().update_sleep(
                (has_support_contact[index] || has_joint_constraint[index])
                    && !has_mouse_joint[index],
                self.settings.sleep,
            );
            if body.state().is_sleeping() {
                stats.sleeping_bodies += 1;
            }
        }
        self.update_sleep_supports();
    }

    /// Wakes only sleeping bodies whose last gravity-opposing support no
    /// longer exists. A support that merely woke but still touches remains
    /// valid, so neighbouring bodies can continue sleeping independently.
    pub(crate) fn wake_unsupported_sleeping_bodies(&mut self) {
        if self.sleep_supports.is_empty() {
            return;
        }

        let previous = mem::take(&mut self.sleep_supports);
        let mut dependents = Vec::with_capacity(previous.len());
        let mut valid = Vec::with_capacity(previous.len());
        let mut collision_solver = CollisionSolver::new();
        for support in previous {
            let Some(dependent) = self.body(support.dependent) else {
                continue;
            };
            if !dependent.state().is_sleeping() {
                continue;
            }

            dependents.push(support.dependent);
            if self.sleep_support_is_valid(support, &mut collision_solver) {
                valid.push(support);
            }
        }

        valid.sort_unstable();
        valid.dedup();
        dependents.sort_unstable();
        dependents.dedup();
        for dependent in dependents {
            if !valid.iter().any(|support| support.dependent == dependent) {
                self.wake_body(dependent);
            }
        }
        self.sleep_supports = valid;
    }

    fn sleep_support_is_valid(
        &self,
        support: SleepSupport,
        collision_solver: &mut CollisionSolver,
    ) -> bool {
        let Some(dependent) = self.body_by_id(support.dependent) else {
            return false;
        };
        let Some(support_body) = self.body_by_id(support.support) else {
            return false;
        };

        match support_body {
            BodyRef::Static(_) => true,
            BodyRef::Dynamic(body) if body.state().is_sleeping() => true,
            BodyRef::Dynamic(_) => bodies_have_gravity_support(
                collision_solver,
                dependent,
                support_body,
                self.settings.gravity,
            ),
        }
    }

    fn update_sleep_supports(&mut self) {
        if self.settings.gravity.is_zero() {
            self.sleep_supports.clear();
            return;
        }

        let bodies = &self.bodies;
        self.sleep_supports.retain(|support| {
            bodies
                .binary_search_by_key(&support.dependent, Body::id)
                .ok()
                .is_some_and(|index| bodies[index].state().is_sleeping())
        });

        for contact in self.active_static_contacts.iter().copied() {
            let body = &self.bodies[contact.body_a];
            if body.state().is_sleeping()
                && normal_supports_body_a(contact.normal, self.settings.gravity)
            {
                self.sleep_supports.push(SleepSupport {
                    dependent: body.id(),
                    support: self.static_bodies[contact.body_b].id(),
                });
            }
        }
        for contact in self.active_dynamic_contacts.iter().copied() {
            let body_a = &self.bodies[contact.body_a];
            let body_b = &self.bodies[contact.body_b];
            if normal_supports_body_a(contact.normal, self.settings.gravity) {
                if body_a.state().is_sleeping() {
                    self.sleep_supports.push(SleepSupport {
                        dependent: body_a.id(),
                        support: body_b.id(),
                    });
                }
            } else if normal_supports_body_a(-contact.normal, self.settings.gravity)
                && body_b.state().is_sleeping()
            {
                self.sleep_supports.push(SleepSupport {
                    dependent: body_b.id(),
                    support: body_a.id(),
                });
            }
        }
        self.sleep_supports.sort_unstable();
        self.sleep_supports.dedup();
    }

    pub(crate) fn wake_impacted_bodies(&mut self) {
        for contact in self.active_static_contacts.iter().copied() {
            let normal_speed = relative_speed_along(
                &self.bodies[contact.body_a],
                None,
                contact.point,
                contact.normal,
            );
            let strong = normal_speed < -IMPACT_SPEED_RAW
                || contact.penetration.raw() > WAKE_PENETRATION_RAW;
            if strong {
                self.bodies[contact.body_a].state_mut().wake();
            }
        }

        for contact in self.active_dynamic_contacts.iter().copied() {
            let normal_speed = relative_speed_along(
                &self.bodies[contact.body_a],
                Some(&self.bodies[contact.body_b]),
                contact.point,
                contact.normal,
            );
            let strong = normal_speed < -IMPACT_SPEED_RAW
                || contact.penetration.raw() > WAKE_PENETRATION_RAW;
            if strong {
                self.bodies[contact.body_a].state_mut().wake();
                self.bodies[contact.body_b].state_mut().wake();
            }
        }
    }

    pub(crate) fn wake_mouse_joints_bodies(&mut self) {
        for joint in &self.mouse_joints {
            if let Ok(index) = self.bodies.binary_search_by_key(&joint.body(), Body::id) {
                self.bodies[index].state_mut().wake();
            }
        }
    }
    pub(super) fn wake_distance_and_rope_joints_bodies(&mut self) {
        // Propagate wake state through a whole joint island. Repeating to a fixed
        // point avoids making the result depend on joint storage order.
        loop {
            let mut changed = false;
            for index in 0..self.distance_joints.len() {
                let joint = self.distance_joints[index];
                changed |= self.wake_dynamic_pair(joint.body_a(), joint.body_b());
            }
            for index in 0..self.rope_joints.len() {
                let joint = self.rope_joints[index];
                if self.rope_is_taut(joint) {
                    changed |= self.wake_dynamic_pair(joint.body_a(), joint.body_b());
                }
            }
            if !changed {
                break;
            }
        }
    }

    fn wake_dynamic_pair(&mut self, body_a: BodyId, body_b: BodyId) -> bool {
        let Ok(index_a) = self.bodies.binary_search_by_key(&body_a, Body::id) else {
            return false;
        };
        let Ok(index_b) = self.bodies.binary_search_by_key(&body_b, Body::id) else {
            return false;
        };
        let sleeping_a = self.bodies[index_a].state().is_sleeping();
        let sleeping_b = self.bodies[index_b].state().is_sleeping();
        if sleeping_a == sleeping_b {
            return false;
        }
        let sleeping_index = if sleeping_a { index_a } else { index_b };
        self.bodies[sleeping_index].state_mut().wake();
        true
    }

    pub(super) fn mark_sleep_constraints(&self, constrained: &mut [bool]) {
        debug_assert_eq!(constrained.len(), self.bodies.len());
        for joint in &self.distance_joints {
            self.mark_dynamic_endpoint(constrained, joint.body_a());
            self.mark_dynamic_endpoint(constrained, joint.body_b());
        }
        for joint in &self.rope_joints {
            if self.rope_is_taut(*joint) {
                self.mark_dynamic_endpoint(constrained, joint.body_a());
                self.mark_dynamic_endpoint(constrained, joint.body_b());
            }
        }
    }

    fn rope_is_taut(&self, joint: RopeJoint) -> bool {
        let Some(endpoint_a) = self.resolve_endpoint(joint.body_a()) else {
            return false;
        };
        let Some(endpoint_b) = self.resolve_endpoint(joint.body_b()) else {
            return false;
        };
        let anchor_a = self.endpoint_anchor(endpoint_a, joint.local_anchor_a());
        let anchor_b = self.endpoint_anchor(endpoint_b, joint.local_anchor_b());

        (anchor_b - anchor_a).squared_magnitude() >= joint.max_length().sqr_length()
    }

    fn mark_dynamic_endpoint(&self, constrained: &mut [bool], id: BodyId) {
        if let Ok(index) = self.bodies.binary_search_by_key(&id, Body::id) {
            constrained[index] = true;
        }
    }
}

fn bodies_have_gravity_support(
    collision_solver: &mut CollisionSolver,
    dependent: BodyRef<'_>,
    support: BodyRef<'_>,
    gravity: LinearAcceleration,
) -> bool {
    if gravity.is_zero()
        || !dependent
            .collider()
            .aabb(dependent.transform())
            .intersects(support.collider().aabb(support.transform()))
    {
        return false;
    }

    let mut supported = false;
    collision_solver.collide(
        dependent.id(),
        dependent.collider(),
        dependent.transform(),
        support.id(),
        support.collider(),
        support.transform(),
        |manifold| {
            supported |= manifold
                .into_contacts()
                .any(|contact| normal_supports_body_a(contact.normal, gravity));
        },
    );
    supported
}

#[inline(always)]
fn normal_supports_body_a(normal: UnitVector, gravity: LinearAcceleration) -> bool {
    let [normal_x, normal_y] = normal.raw();
    let [gravity_x, gravity_y] = gravity.raw();
    normal_x as i64 * gravity_x as i64 + normal_y as i64 * gravity_y as i64 > 0
}
