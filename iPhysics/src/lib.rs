#![no_std]
extern crate alloc;

pub mod body;
pub mod collider;
pub mod collision;
pub(crate) mod fix;
pub mod geometry;
pub mod quantity;
pub mod transform;
pub mod world;

pub use body::{Body, BodyId, BodyState, Material, SleepConfig, StaticBody};
pub use collider::{
    Circle, Collider, ColliderPart, CompositeCollider, CompositeColliderError, Convex, ConvexError,
    TransformedVertices,
};
pub use collision::Contact;
pub use geometry::{Aabb, GeometryPoint, UnitVector};
pub use quantity::{
    Angle, AngleDelta, AngularAcceleration, AngularVelocity, DiffVec2, Length, LinearAcceleration,
    LinearVelocity, Mass, Position,
};
pub use transform::Transform;
pub use world::{AddBodyError, StepStats, World, WorldSettings};
