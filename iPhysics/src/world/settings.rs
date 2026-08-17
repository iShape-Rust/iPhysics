use crate::body::SleepConfig;
use crate::quantity::LinearAcceleration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorldSettings {
    pub gravity: LinearAcceleration,
    pub velocity_iterations: u8,
    pub sleep: SleepConfig,
}

impl WorldSettings {
    #[inline(always)]
    pub const fn new(gravity: LinearAcceleration) -> Self {
        Self {
            gravity,
            velocity_iterations: 4,
            sleep: SleepConfig::FAST_EFFECTS,
        }
    }
}

impl Default for WorldSettings {
    fn default() -> Self {
        Self::new(
            LinearAcceleration::from_meters_per_second_squared(0.0, -10.0)
                .expect("default gravity must fit Q24"),
        )
    }
}
