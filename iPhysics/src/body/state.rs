use crate::quantity::{
    AngularAcceleration, AngularVelocity, LinearAcceleration, LinearVelocity, checked_integrate,
    checked_integrate_angular,
};
use crate::transform::Transform;

/// Aggressive sleeping thresholds intended for short-lived gameplay effects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SleepConfig {
    linear_speed_raw: u32,
    angular_speed_raw: u32,
    required_ticks: u8,
}

impl SleepConfig {
    /// `0.05 m/s`, `0.1 rad/s`, and 16 ticks (`0.25 s` at 64 Hz).
    pub const FAST_EFFECTS: Self = Self {
        linear_speed_raw: 838_861,
        angular_speed_raw: 1_677_722,
        required_ticks: 16,
    };

    #[inline(always)]
    pub const fn from_raw(
        linear_speed_raw: u32,
        angular_speed_raw: u32,
        required_ticks: u8,
    ) -> Self {
        Self {
            linear_speed_raw,
            angular_speed_raw,
            required_ticks,
        }
    }

    #[inline(always)]
    pub const fn linear_speed_raw(self) -> u32 {
        self.linear_speed_raw
    }

    #[inline(always)]
    pub const fn angular_speed_raw(self) -> u32 {
        self.angular_speed_raw
    }

    #[inline(always)]
    pub const fn required_ticks(self) -> u8 {
        self.required_ticks
    }
}

impl Default for SleepConfig {
    fn default() -> Self {
        Self::FAST_EFFECTS
    }
}

/// Complete mutable body state required by deterministic rollback.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BodyState {
    transform: Transform,
    linear_velocity: LinearVelocity,
    angular_velocity: AngularVelocity,
    sleep_ticks: u8,
    sleeping: bool,
}

impl BodyState {
    #[inline(always)]
    pub const fn new(
        transform: Transform,
        linear_velocity: LinearVelocity,
        angular_velocity: AngularVelocity,
    ) -> Self {
        Self {
            transform,
            linear_velocity,
            angular_velocity,
            sleep_ticks: 0,
            sleeping: false,
        }
    }

    #[inline(always)]
    pub const fn is_sleeping(&self) -> bool {
        self.sleeping
    }

    #[inline(always)]
    pub const fn transform(&self) -> Transform {
        self.transform
    }

    #[inline(always)]
    pub const fn linear_velocity(&self) -> LinearVelocity {
        self.linear_velocity
    }

    #[inline(always)]
    pub const fn angular_velocity(&self) -> AngularVelocity {
        self.angular_velocity
    }

    #[inline(always)]
    pub const fn sleep_ticks(&self) -> u8 {
        self.sleep_ticks
    }

    /// Wakes the body after an external force, transform change, or impact.
    #[inline(always)]
    pub fn wake(&mut self) {
        self.sleep_ticks = 0;
        self.sleeping = false;
    }

    #[inline(always)]
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        self.wake();
    }

    #[inline(always)]
    pub fn set_linear_velocity(&mut self, velocity: LinearVelocity) {
        self.linear_velocity = velocity;
        self.wake();
    }

    #[inline(always)]
    pub fn set_angular_velocity(&mut self, velocity: AngularVelocity) {
        self.angular_velocity = velocity;
        self.wake();
    }

    /// Integrates one fixed tick. Sleeping bodies deliberately ignore gravity;
    /// external changes must call [`Self::wake`] first.
    #[inline]
    pub fn checked_integrate(
        &mut self,
        linear_acceleration: LinearAcceleration,
        angular_acceleration: AngularAcceleration,
    ) -> bool {
        if self.sleeping {
            return true;
        }

        let Some((position, linear_velocity)) = checked_integrate(
            self.transform.position,
            self.linear_velocity,
            linear_acceleration,
        ) else {
            return false;
        };
        let Some((angle, angular_velocity)) = checked_integrate_angular(
            self.transform.angle,
            self.angular_velocity,
            angular_acceleration,
        ) else {
            return false;
        };

        self.transform = Transform::new(position, angle);
        self.linear_velocity = linear_velocity;
        self.angular_velocity = angular_velocity;
        true
    }

    /// Updates the explicit rollback-safe sleep counter after collision solving.
    /// Returns `true` when the body transitions to sleeping on this call.
    #[inline]
    pub fn update_sleep(&mut self, has_contact: bool, config: SleepConfig) -> bool {
        if self.sleeping {
            return false;
        }

        let linear_limit = config.linear_speed_raw as u64;
        let slow_linear =
            self.linear_velocity.raw_sqr_magnitude() <= linear_limit.saturating_mul(linear_limit);
        let slow_angular = self.angular_velocity.raw_magnitude() <= config.angular_speed_raw;

        if !has_contact || !slow_linear || !slow_angular {
            self.sleep_ticks = 0;
            return false;
        }

        self.sleep_ticks = self.sleep_ticks.saturating_add(1);
        if self.sleep_ticks < config.required_ticks.max(1) {
            return false;
        }

        self.linear_velocity = LinearVelocity::ZERO;
        self.angular_velocity = AngularVelocity::ZERO;
        self.sleeping = true;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quantity::{Angle, Position};

    #[test]
    fn integrates_awake_body_transactionally() {
        let mut state = BodyState::new(
            Transform::new(Position::ZERO, Angle::ZERO),
            LinearVelocity::ZERO,
            AngularVelocity::ZERO,
        );
        let acceleration = LinearAcceleration::from_meters_per_second_squared(64.0, 0.0).unwrap();

        assert!(state.checked_integrate(acceleration, AngularAcceleration::ZERO));
        assert_eq!(state.linear_velocity().to_meters_per_second(), [1.0, 0.0]);
        assert_eq!(state.transform().position.to_meters(), [0.015625, 0.0]);
    }

    #[test]
    fn sleeps_after_configured_contact_ticks() {
        let config = SleepConfig::from_raw(u32::MAX, u32::MAX, 3);
        let mut state = BodyState::new(
            Transform::IDENTITY,
            LinearVelocity::from_raw(10, -10),
            AngularVelocity::from_raw(10),
        );

        assert!(!state.update_sleep(true, config));
        assert!(!state.update_sleep(true, config));
        assert!(state.update_sleep(true, config));
        assert!(state.is_sleeping());
        assert_eq!(state.linear_velocity(), LinearVelocity::ZERO);
        assert_eq!(state.angular_velocity(), AngularVelocity::ZERO);
    }

    #[test]
    fn body_without_contact_does_not_sleep() {
        let config = SleepConfig::from_raw(u32::MAX, u32::MAX, 1);
        let mut state = BodyState::default();

        assert!(!state.update_sleep(false, config));
        assert!(!state.is_sleeping());
    }

    #[test]
    fn wake_resets_explicit_sleep_state() {
        let config = SleepConfig::from_raw(u32::MAX, u32::MAX, 1);
        let mut state = BodyState::default();
        assert!(state.update_sleep(true, config));

        state.wake();

        assert!(!state.is_sleeping());
        assert_eq!(state.sleep_ticks(), 0);
    }

    #[test]
    fn sleeping_body_ignores_gravity() {
        let config = SleepConfig::from_raw(u32::MAX, u32::MAX, 1);
        let mut state = BodyState::default();
        assert!(state.update_sleep(true, config));
        let before = state;

        assert!(state.checked_integrate(
            LinearAcceleration::from_meters_per_second_squared(0.0, -10.0).unwrap(),
            AngularAcceleration::ZERO,
        ));
        assert_eq!(state, before);
    }
}
