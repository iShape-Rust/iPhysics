use super::World;

#[derive(Debug, Clone, Copy, Default)]
pub(in crate::world::simulation) struct DistanceImpulseState {
    /// Accumulated bilateral scalar impulse in Q10 kg*m/s.
    pub(super) accumulated_impulse_q10: i64,
}

impl World {
    pub(in crate::world::simulation) fn solve_distance_joints_velocities(
        &mut self,
        states: &mut [DistanceImpulseState],
        reverse: bool,
    ) {
        debug_assert_eq!(states.len(), self.distance_joints.len());
        if reverse {
            for index in (0..self.distance_joints.len()).rev() {
                self.solve_distance_velocity(index, &mut states[index]);
            }
        } else {
            for (index, state) in states.iter_mut().enumerate() {
                self.solve_distance_velocity(index, state);
            }
        }
    }

    pub(super) fn solve_distance_velocity(
        &mut self,
        joint_index: usize,
        state: &mut DistanceImpulseState,
    ) {
        let joint = self.distance_joints[joint_index];
        self.solve_scalar_constraint(
            joint.body_a(),
            joint.local_anchor_a(),
            joint.body_b(),
            joint.local_anchor_b(),
            joint.length(),
            joint.max_force().impulse_per_tick_q10(),
            joint.response_raw(),
            false,
            &mut state.accumulated_impulse_q10,
        );
    }
}
