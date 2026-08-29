use crate::body::SleepConfig;
use crate::quantity::{Damping, LinearAcceleration};

const DEFAULT_DAMPING_RAW: u32 = 32;
const DEFAULT_COLUMN_WIDTH_POWER: u8 = 5;

/// Fixed-width X-axis grid used by the broad phase.
///
/// A power of `P` selects columns `2^P` metres wide. The bounded geometry
/// domain is at most `2^15` metres wide, so larger powers provide no useful
/// additional layouts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridBroadPhase {
    column_width_power: u8,
}

impl GridBroadPhase {
    pub const MAX_COLUMN_WIDTH_POWER: u8 = 15;

    #[inline(always)]
    pub const fn new(column_width_power: u8) -> Option<Self> {
        if column_width_power <= Self::MAX_COLUMN_WIDTH_POWER {
            Some(Self { column_width_power })
        } else {
            None
        }
    }

    #[inline(always)]
    pub const fn column_width_power(self) -> u8 {
        self.column_width_power
    }
}

impl Default for GridBroadPhase {
    fn default() -> Self {
        Self {
            column_width_power: DEFAULT_COLUMN_WIDTH_POWER,
        }
    }
}

/// Candidate-pair search performed before exact collision detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BroadPhase {
    /// Tests every eligible body pair.
    BruteForce,
    /// Always distributes AABBs into fixed-width X-axis columns.
    Grid(GridBroadPhase),
    /// Uses brute force for small worlds and the grid otherwise.
    Auto(GridBroadPhase),
}

impl Default for BroadPhase {
    fn default() -> Self {
        Self::Auto(GridBroadPhase::default())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorldSettings {
    pub gravity: LinearAcceleration,
    pub broad_phase: BroadPhase,
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
            broad_phase: BroadPhase::Auto(GridBroadPhase {
                column_width_power: DEFAULT_COLUMN_WIDTH_POWER,
            }),
            linear_damping: match Damping::from_raw(DEFAULT_DAMPING_RAW) {
                Some(value) => value,
                None => unreachable!(),
            },
            angular_damping: match Damping::from_raw(DEFAULT_DAMPING_RAW) {
                Some(value) => value,
                None => unreachable!(),
            },
            velocity_iterations: 50,
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

    #[test]
    fn default_broad_phase_is_auto_with_32_meter_columns() {
        assert_eq!(
            WorldSettings::default().broad_phase,
            BroadPhase::Auto(GridBroadPhase::new(5).unwrap())
        );
    }

    #[test]
    fn grid_width_power_is_bounded_by_the_geometry_domain() {
        assert!(GridBroadPhase::new(15).is_some());
        assert!(GridBroadPhase::new(16).is_none());
    }
}
