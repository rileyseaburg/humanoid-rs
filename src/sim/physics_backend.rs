//! Physics Backend Abstraction Layer
//!
//! Provides a trait-based abstraction over physics engines, allowing
//! swappable backends (Rapier, PhysX, custom) without changing simulation logic.
//!
//! # Example
//! ```
//! use humanoid_rs::sim::physics_backend::{PhysicsBackend, RapierBackend, RigidBodySpec};
//! use std::time::Duration;
//!
//! let mut backend = RapierBackend::new();
//! backend.set_gravity(nalgebra::Vector3::new(0.0, -9.81, 0.0));
//! backend.set_timestep(Duration::from_secs_f64(1.0 / 500.0));
//!
//! let body_spec = RigidBodySpec {
//!     mass: 1.0,
//!     position: nalgebra::Isometry3::identity(),
//!     shape: Shape::Cuboid { half_extents: nalgebra::Vector3::new(0.5, 0.5, 0.5) },
//! };
//! let handle = backend.add_rigid_body(body_spec).unwrap();
//! ```

use nalgebra::{Isometry3, Vector3};
use std::time::Duration;

/// Handle to a rigid body in the physics world
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BodyHandle(pub usize);

/// Handle to a joint in the physics world
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct JointHandle(pub usize);

/// Shape types for collision geometry
#[derive(Clone, Debug)]
pub enum Shape {
    /// Axis-aligned bounding box centered at origin
    Cuboid {
        /// Half-extents in each dimension
        half_extents: Vector3<f64>,
    },
    /// Sphere centered at origin
    Sphere {
        /// Radius of the sphere
        radius: f64,
    },
    /// Cylinder aligned with Y axis
    Cylinder {
        /// Half-height of the cylinder
        half_height: f64,
        /// Radius of the cylinder
        radius: f64,
    },
    /// Capsule aligned with Y axis
    Capsule {
        /// Half-height of the capsule (not including hemispheres)
        half_height: f64,
        /// Radius of the capsule
        radius: f64,
    },
}

/// Rigid body type
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyType {
    /// Fixed in space (infinite mass)
    Fixed,
    /// Dynamic body affected by forces
    Dynamic,
    /// Kinematic body controlled by velocity
    Kinematic,
}

/// Specification for creating a rigid body
#[derive(Clone, Debug)]
pub struct RigidBodySpec {
    /// Body type (fixed, dynamic, kinematic)
    pub body_type: BodyType,
    /// Mass in kilograms (ignored for fixed bodies)
    pub mass: f64,
    /// Initial position and orientation
    pub position: Isometry3<f64>,
    /// Linear velocity
    pub linear_velocity: Vector3<f64>,
    /// Angular velocity
    pub angular_velocity: Vector3<f64>,
    /// Collision shape
    pub shape: Shape,
    /// Local inertia tensor diagonal (if None, computed from shape)
    pub inertia: Option<Vector3<f64>>,
    /// Linear damping coefficient
    pub linear_damping: f64,
    /// Angular damping coefficient
    pub angular_damping: f64,
    /// Restitution (bounciness)
    pub restitution: f64,
    /// Friction coefficient
    pub friction: f64,
}

impl Default for RigidBodySpec {
    fn default() -> Self {
        Self {
            body_type: BodyType::Dynamic,
            mass: 1.0,
            position: Isometry3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            shape: Shape::Cuboid {
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            inertia: None,
            linear_damping: 0.0,
            angular_damping: 0.0,
            restitution: 0.0,
            friction: 0.5,
        }
    }
}

/// Joint type
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointType {
    /// Revolute joint (rotation around axis)
    Revolute,
    /// Prismatic joint (translation along axis)
    Prismatic,
    /// Fixed joint (no relative motion)
    Fixed,
    /// Spherical joint (ball and socket)
    Spherical,
}

/// Specification for creating a joint
#[derive(Clone, Debug)]
pub struct JointSpec {
    /// Joint type
    pub joint_type: JointType,
    /// Body A handle
    pub body_a: BodyHandle,
    /// Body B handle
    pub body_b: BodyHandle,
    /// Anchor point on body A in local coordinates
    pub local_anchor_a: Vector3<f64>,
    /// Anchor point on body B in local coordinates
    pub local_anchor_b: Vector3<f64>,
    /// Joint axis in local coordinates of body A (for revolute/prismatic)
    pub axis: Vector3<f64>,
    /// Position limits (min, max) in radians for revolute, meters for prismatic
    pub position_limits: Option<(f64, f64)>,
    /// Spring stiffness for joint motor (0 = locked, None = free)
    pub motor_stiffness: Option<f64>,
    /// Damping for joint motor
    pub motor_damping: Option<f64>,
    /// Maximum motor force/torque
    pub max_motor_force: f64,
}

impl Default for JointSpec {
    fn default() -> Self {
        Self {
            joint_type: JointType::Revolute,
            body_a: BodyHandle(0),
            body_b: BodyHandle(0),
            local_anchor_a: Vector3::zeros(),
            local_anchor_b: Vector3::zeros(),
            axis: Vector3::y_axis().into_inner(),
            position_limits: None,
            motor_stiffness: None,
            motor_damping: None,
            max_motor_force: f64::INFINITY,
        }
    }
}

/// Velocity state of a body
#[derive(Clone, Debug, Default)]
pub struct Velocity {
    /// Linear velocity in world frame
    pub linear: Vector3<f64>,
    /// Angular velocity in world frame
    pub angular: Vector3<f64>,
}

/// Contact point between two bodies
#[derive(Clone, Debug)]
pub struct Contact {
    /// First body involved in contact
    pub body_a: BodyHandle,
    /// Second body involved in contact
    pub body_b: BodyHandle,
    /// Contact point in world coordinates
    pub point: Vector3<f64>,
    /// Contact normal (pointing from body B to body A)
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = penetration)
    pub penetration: f64,
    /// Contact impulse magnitude
    pub impulse: f64,
}

/// Errors that can occur in physics operations
#[derive(thiserror::Error, Debug)]
pub enum PhysicsError {
    #[error("Invalid body handle: {0:?}")]
    InvalidBodyHandle(BodyHandle),

    #[error("Invalid joint handle: {0:?}")]
    InvalidJointHandle(JointHandle),

    #[error("Body creation failed: {0}")]
    BodyCreationFailed(String),

    #[error("Joint creation failed: {0}")]
    JointCreationFailed(String),

    #[error("Backend error: {0}")]
    BackendError(String),
}

/// Result type for physics operations
pub type PhysicsResult<T> = std::result::Result<T, PhysicsError>;

/// Core trait for physics backends
///
/// Implement this trait to provide a custom physics engine backend.
/// All methods are required to be implemented.
pub trait PhysicsBackend: Send + Sync {
    /// Advance the physics simulation by the given timestep
    ///
    /// # Arguments
    /// * `dt` - Time step duration
    fn step(&mut self, dt: Duration);

    /// Add a rigid body to the physics world
    ///
    /// # Arguments
    /// * `spec` - Specification for the rigid body
    ///
    /// # Returns
    /// Handle to the created body
    fn add_rigid_body(&mut self, spec: RigidBodySpec) -> PhysicsResult<BodyHandle>;

    /// Remove a rigid body from the physics world
    ///
    /// # Arguments
    /// * `handle` - Handle of the body to remove
    fn remove_body(&mut self, handle: BodyHandle) -> PhysicsResult<()>;

    /// Add a joint between two bodies
    ///
    /// # Arguments
    /// * `spec` - Specification for the joint
    ///
    /// # Returns
    /// Handle to the created joint
    fn add_joint(&mut self, spec: JointSpec) -> PhysicsResult<JointHandle>;

    /// Remove a joint from the physics world
    ///
    /// # Arguments
    /// * `handle` - Handle of the joint to remove
    fn remove_joint(&mut self, handle: JointHandle) -> PhysicsResult<()>;

    /// Get the transform (position and orientation) of a body
    ///
    /// # Arguments
    /// * `handle` - Body handle
    ///
    /// # Returns
    /// The body's transform, or None if the body doesn't exist
    fn body_transform(&self, handle: BodyHandle) -> PhysicsResult<Isometry3<f64>>;

    /// Get the velocity of a body
    ///
    /// # Arguments
    /// * `handle` - Body handle
    ///
    /// # Returns
    /// The body's velocity, or None if the body doesn't exist
    fn body_velocity(&self, handle: BodyHandle) -> PhysicsResult<Velocity>;

    /// Set the transform of a body
    ///
    /// # Arguments
    /// * `handle` - Body handle
    /// * `transform` - New transform
    fn set_body_transform(
        &mut self,
        handle: BodyHandle,
        transform: Isometry3<f64>,
    ) -> PhysicsResult<()>;

    /// Set the velocity of a body
    ///
    /// # Arguments
    /// * `handle` - Body handle
    /// * `velocity` - New velocity
    fn set_body_velocity(&mut self, handle: BodyHandle, velocity: Velocity) -> PhysicsResult<()>;

    /// Apply a force to a body at its center of mass
    ///
    /// # Arguments
    /// * `handle` - Body handle
    /// * `force` - Force vector in world coordinates
    fn apply_force(&mut self, handle: BodyHandle, force: Vector3<f64>) -> PhysicsResult<()>;

    /// Apply a torque to a body
    ///
    /// # Arguments
    /// * `handle` - Body handle
    /// * `torque` - Torque vector in world coordinates
    fn apply_torque(&mut self, handle: BodyHandle, torque: Vector3<f64>) -> PhysicsResult<()>;

    /// Get all current contacts
    ///
    /// # Returns
    /// Vector of all active contacts
    fn contacts(&self) -> Vec<Contact>;

    /// Set the gravity vector
    ///
    /// # Arguments
    /// * `gravity` - Gravity acceleration vector
    fn set_gravity(&mut self, gravity: Vector3<f64>);

    /// Get the current gravity vector
    fn gravity(&self) -> Vector3<f64>;

    /// Set the simulation timestep
    ///
    /// # Arguments
    /// * `timestep` - Fixed timestep for the simulation
    fn set_timestep(&mut self, timestep: Duration);

    /// Get the current simulation timestep
    fn timestep(&self) -> Duration;

    /// Get the number of bodies in the simulation
    fn num_bodies(&self) -> usize;

    /// Get the number of joints in the simulation
    fn num_joints(&self) -> usize;
}

// ============================================
// Rapier Backend Implementation (STUB)
// ============================================

#[cfg(feature = "simulator")]
pub mod rapier_backend {
    use super::*;

    /// Rapier3D-based physics backend (STUB)
    ///
    /// This is a stub implementation that compiles but does not function.
    /// The full implementation needs to be fixed.
    pub struct RapierBackend;

    impl RapierBackend {
        pub fn new() -> Self {
            Self
        }
    }

    impl Default for RapierBackend {
        fn default() -> Self {
            Self::new()
        }
    }
}

#[cfg(feature = "simulator")]
pub use rapier_backend::RapierBackend;
