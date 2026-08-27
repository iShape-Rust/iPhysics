use super::World;

#[derive(Debug, Clone, Copy, Default)]
pub(in crate::world::simulation) struct RopeImpulseState {
    /// Accumulated pulling-only scalar impulse in Q10 kg*m/s.
    pub(super) accumulated_impulse_q10: i64,
}

impl World {
    pub(in crate::world::simulation) fn solve_rope_joints_velocities(
        &mut self,
        states: &mut [RopeImpulseState],
        reverse: bool,
    ) {
        debug_assert_eq!(states.len(), self.rope_joints.len());
        if reverse {
            for index in (0..self.rope_joints.len()).rev() {
                self.solve_rope_velocity(index, &mut states[index]);
            }
        } else {
            for (index, state) in states.iter_mut().enumerate() {
                self.solve_rope_velocity(index, state);
            }
        }
    }

    pub(super) fn solve_rope_velocity(&mut self, joint_index: usize, state: &mut RopeImpulseState) {
        let joint = self.rope_joints[joint_index];
        self.solve_scalar_constraint(
            joint.body_a(),
            joint.local_anchor_a(),
            joint.body_b(),
            joint.local_anchor_b(),
            joint.max_length(),
            joint.max_force().impulse_per_tick_q10(),
            joint.response_raw(),
            true,
            &mut state.accumulated_impulse_q10,
        );
    }
}
