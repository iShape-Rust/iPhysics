use crate::world::simulation::constraint::{relative_speed_along, two_bodies_mut};
use crate::{Body, BodyId, RopeJoint, StepStats, World};
use alloc::vec;

use super::IMPACT_SPEED_RAW;

const WAKE_PENETRATION_RAW: u32 = 655; // approximately 0.01 m in Q16

impl World {
    pub(crate) fn update_sleep_states(&mut self, stats: &mut StepStats) {
        let mut has_contact = vec![false; self.bodies.len()];
        for contact in self.active_static_contacts.iter().copied() {
            has_contact[contact.body_a] = true;
        }
        for contact in self.active_dynamic_contacts.iter().copied() {
            has_contact[contact.body_a] = true;
            has_contact[contact.body_b] = true;
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
                (has_contact[index] || has_joint_constraint[index]) && !has_mouse_joint[index],
                self.settings.sleep,
            );
            if body.state().is_sleeping() {
                stats.sleeping_bodies += 1;
            }
        }
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
                let (a, b) = two_bodies_mut(&mut self.bodies, contact.body_a, contact.body_b);
                a.state_mut().wake();
                b.state_mut().wake();
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
