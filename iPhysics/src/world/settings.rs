use crate::body::SleepConfig;
use crate::quantity::{Damping, LinearAcceleration};

const DEFAULT_DAMPING_RAW: u32 = 66;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorldSettings {
    pub gravity: LinearAcceleration,
    /// Fraction of linear velocity lost during each fixed simulation tick.
    pub linear_damping: Damping,
    /// Fraction of angular velocity lost during each fixed simulation tick.
    pub angular_damping: Damping,
    pub velocity_iterations: u8,
    pub sleep: SleepConfig,
}

impl WorldSettings {
    #[inline(always)]
    pub const fn new(gravity: LinearAcceleration) -> Self {
        Self {
            gravity,
            linear_damping: match Damping::from_raw(DEFAULT_DAMPING_RAW) {
                Some(value) => value,
                None => unreachable!(),
            },
            angular_damping: match Damping::from_raw(DEFAULT_DAMPING_RAW) {
                Some(value) => value,
                None => unreachable!(),
            },
            velocity_iterations: 6,
            sleep: SleepConfig::FAST_EFFECTS,
        }
    }
}

impl Default for WorldSettings {
    fn default() -> Self {
        Self::new(
            LinearAcceleration::from_meters_per_second_squared(0.0, -10.0)
                .expect("default gravity must fit Q4"),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_damping_is_about_one_thousandth_per_tick() {
        let settings = WorldSettings::default();

        assert_eq!(settings.linear_damping.raw(), DEFAULT_DAMPING_RAW);
        assert_eq!(settings.angular_damping.raw(), DEFAULT_DAMPING_RAW);
        assert!((settings.linear_damping.coefficient() - 0.001).abs() < 0.000_01);
    }

    #[test]
    fn default_solver_uses_six_velocity_iterations() {
        assert_eq!(WorldSettings::default().velocity_iterations, 6);
    }
}
