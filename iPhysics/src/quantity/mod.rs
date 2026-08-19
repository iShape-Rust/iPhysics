//! Fixed-point physical quantities used by the simulation.
//!
//! Each physical dimension has its own public type even when two quantities
//! share the same storage format. This prevents, for example, accidentally
//! adding acceleration directly to position.

mod angle;
mod angular_acceleration;
mod angular_velocity;
mod integration;
mod length;
mod linear_acceleration;
mod linear_velocity;
mod mass;
mod position;

pub(crate) use crate::geometry::vec::RawVec2;
pub use angle::{Angle, AngleDelta};
pub use angular_acceleration::AngularAcceleration;
pub use angular_velocity::AngularVelocity;
pub use integration::{integrate, integrate_angular};
pub use length::Length;
pub use linear_acceleration::LinearAcceleration;
pub use linear_velocity::LinearVelocity;
pub use mass::Mass;
pub use position::Position;

/// Number of fixed simulation ticks in one second.
pub const TICKS_PER_SECOND: u32 = 64;

pub(crate) const POSITION_FRACTION_BITS: u32 = 16;
pub(crate) const KINEMATIC_FRACTION_BITS: u32 = 24;
pub(crate) const ACCELERATION_TO_VELOCITY_SHIFT: u32 = 6;
pub(crate) const VELOCITY_TO_POSITION_SHIFT: u32 =
    KINEMATIC_FRACTION_BITS + ACCELERATION_TO_VELOCITY_SHIFT - POSITION_FRACTION_BITS;

#[cfg(test)]
mod tests;
