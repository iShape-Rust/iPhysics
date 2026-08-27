#![no_std]
extern crate alloc;

pub mod body;
pub mod collider;
pub mod collision;
pub mod geometry;
pub mod joint;
pub(crate) mod ops;
pub mod quantity;
pub mod transform;
pub mod world;

pub use body::{Body, BodyId, BodyState, Material, SleepConfig, StaticBody};
pub use collider::{
    Circle, Collider, CompositeCollider, Convex, ConvexError, RECOMMENDED_MAX_COMPOSITE_PARTS,
    SimpleCollider,
};
pub use collision::Contact;
pub use geometry::{Aabb, GeometryPoint, UnitVector};
pub use joint::{DistanceJoint, MouseJoint, RopeJoint};
pub use quantity::{
    Angle, AngleDelta, AngularAcceleration, AngularVelocity, Damping, Force, Length,
    LinearAcceleration, LinearVelocity, Mass, Position,
};
pub use transform::Transform;
pub use world::{
    AddBodyError, AddJointError, AddMouseJointError, BodyRef, BroadPhase, Contacts, GridBroadPhase,
    StepStats, World, WorldSettings,
};
