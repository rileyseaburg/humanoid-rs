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
// Rapier Backend Implementation
// ============================================

#[cfg(feature = "simulator")]
pub mod rapier_backend {
    use super::*;
    use rapier3d::prelude::*;
    use rapier3d::prelude::*;

    /// Rapier3D-based physics backend
    ///
    /// Implements the PhysicsBackend trait using the Rapier3D physics engine.
    pub struct RapierBackend {
        /// The physics pipeline
        pipeline: PhysicsPipeline,
        /// The integration parameters
        integration_parameters: IntegrationParameters,
        /// The broad phase
        broad_phase: DefaultBroadPhase,
        /// The narrow phase
        narrow_phase: NarrowPhase,
        /// The island manager
        island_manager: IslandManager,
        /// The impulse joint set
        impulse_joints: ImpulseJointSet,
        /// The multibody joint set
        multibody_joints: MultibodyJointSet,
        /// The rigid body set
        bodies: RigidBodySet,
        /// The collider set
        colliders: ColliderSet,
        /// The CCD solver
        ccd_solver: CCDSolver,
        /// Gravity vector
        gravity: Vector3<f64>,
        /// Timestep
        timestep: Duration,
        /// Body handle mapping (our BodyHandle -> Rapier RigidBodyHandle)
        body_handles: Vec<RigidBodyHandle>,
        /// Joint handle mapping (our JointHandle -> Rapier JointHandle)
        joint_handles: Vec<JointHandle>,
        /// Inverse body handle mapping (Rapier -> ours)
        rapier_to_body: Vec<Option<BodyHandle>>,
        /// Inverse joint handle mapping (Rapier -> ours)
        rapier_to_joint: Vec<Option<JointHandle>>,
    }

    impl RapierBackend {
        /// Create a new Rapier backend with default settings
        pub fn new() -> Self {
            Self::with_timestep(Duration::from_secs_f64(1.0 / 60.0))
        }

        /// Create a new Rapier backend with a specific timestep
        pub fn with_timestep(timestep: Duration) -> Self {
            let mut integration_parameters = IntegrationParameters::default();
            integration_parameters.dt = timestep.as_secs_f64() as f32;

            Self {
                pipeline: PhysicsPipeline::new(),
                integration_parameters,
                broad_phase: DefaultBroadPhase::new(),
                narrow_phase: NarrowPhase::new(),
                island_manager: IslandManager::new(),
                impulse_joints: ImpulseJointSet::new(),
                multibody_joints: MultibodyJointSet::new(),
                bodies: RigidBodySet::new(),
                colliders: ColliderSet::new(),
                ccd_solver: CCDSolver::new(),
                gravity: Vector3::new(0.0, -9.81, 0.0),
                timestep,
                body_handles: Vec::new(),
                joint_handles: Vec::new(),
                rapier_to_body: Vec::new(),
                rapier_to_joint: Vec::new(),
            }
        }

        /// Convert our BodyHandle to Rapier's RigidBodyHandle
        fn to_rapier_body(&self, handle: BodyHandle) -> PhysicsResult<RigidBodyHandle> {
            self.body_handles
                .get(handle.0)
                .copied()
                .ok_or(PhysicsError::InvalidBodyHandle(handle))
        }

        /// Convert our JointHandle to Rapier's JointHandle
        fn to_rapier_joint(&self, handle: JointHandle) -> PhysicsResult<JointHandle> {
            self.joint_handles
                .get(handle.0)
                .copied()
                .ok_or(PhysicsError::InvalidJointHandle(handle))
        }

        /// Convert Rapier RigidBodyHandle to our BodyHandle
        fn from_rapier_body(&self, rapier_handle: RigidBodyHandle) -> Option<BodyHandle> {
            let index = rapier_handle.index();
            self.rapier_to_body.get(index).copied().flatten()
        }

        /// Convert Rapier JointHandle to our JointHandle
        fn from_rapier_joint(&self, rapier_handle: JointHandle) -> Option<JointHandle> {
            let index = rapier_handle.into_raw_parts().0;
            self.rapier_to_joint.get(index).copied().flatten()
        }

        /// Convert our Shape to Rapier's SharedShape
        fn to_rapier_shape(&self, shape: &Shape) -> SharedShape {
            match shape {
                Shape::Cuboid { half_extents } => {
                    SharedShape::cuboid(half_extents.x, half_extents.y, half_extents.z)
                }
                Shape::Sphere { radius } => SharedShape::ball(*radius),
                Shape::Cylinder {
                    half_height,
                    radius,
                } => SharedShape::cylinder(*half_height, *radius),
                Shape::Capsule {
                    half_height,
                    radius,
                } => SharedShape::capsule_y(*half_height, *radius),
            }
        }

        /// Convert our BodyType to Rapier's RigidBodyType
        fn to_rapier_body_type(&self, body_type: BodyType) -> RigidBodyType {
            match body_type {
                BodyType::Fixed => RigidBodyType::Fixed,
                BodyType::Dynamic => RigidBodyType::Dynamic,
                BodyType::Kinematic => RigidBodyType::KinematicVelocityBased,
            }
        }

        /// Convert our JointType to Rapier's JointBuilder
        fn to_rapier_joint(&self, spec: &JointSpec) -> Box<dyn Joint> {
            match spec.joint_type {
                JointType::Revolute => {
                    let mut builder = RevoluteJointBuilder::new(spec.axis)
                        .local_anchor1(point![
                            spec.local_anchor_a.x,
                            spec.local_anchor_a.y,
                            spec.local_anchor_a.z
                        ])
                        .local_anchor2(point![
                            spec.local_anchor_b.x,
                            spec.local_anchor_b.y,
                            spec.local_anchor_b.z
                        ]);

                    if let Some((min, max)) = spec.position_limits {
                        builder = builder.limits([min as f32, max as f32]);
                    }

                    Box::new(builder)
                }
                JointType::Prismatic => {
                    let mut builder = PrismaticJointBuilder::new(spec.axis)
                        .local_anchor1(point![
                            spec.local_anchor_a.x,
                            spec.local_anchor_a.y,
                            spec.local_anchor_a.z
                        ])
                        .local_anchor2(point![
                            spec.local_anchor_b.x,
                            spec.local_anchor_b.y,
                            spec.local_anchor_b.z
                        ]);

                    if let Some((min, max)) = spec.position_limits {
                        builder = builder.limits([min as f32, max as f32]);
                    }

                    Box::new(builder)
                }
                JointType::Fixed => Box::new(
                    FixedJointBuilder::new()
                        .local_anchor1(point![
                            spec.local_anchor_a.x,
                            spec.local_anchor_a.y,
                            spec.local_anchor_a.z
                        ])
                        .local_anchor2(point![
                            spec.local_anchor_b.x,
                            spec.local_anchor_b.y,
                            spec.local_anchor_b.z
                        ]),
                ),
                JointType::Spherical => Box::new(
                    SphericalJointBuilder::new()
                        .local_anchor1(point![
                            spec.local_anchor_a.x,
                            spec.local_anchor_a.y,
                            spec.local_anchor_a.z
                        ])
                        .local_anchor2(point![
                            spec.local_anchor_b.x,
                            spec.local_anchor_b.y,
                            spec.local_anchor_b.z
                        ]),
                ),
            }
        }

        /// Convert Rapier Isometry to nalgebra Isometry3
        fn to_nalgebra_isometry(iso: &rapier3d::math::Isometry<f64>) -> Isometry3<f64> {
            let translation = iso.translation;
            let rotation = iso.rotation;
            Isometry3::from_parts(
                nalgebra::Translation3::new(translation.x, translation.y, translation.z),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    rotation.coords[3],
                    rotation.coords[0],
                    rotation.coords[1],
                    rotation.coords[2],
                )),
            )
        }

        /// Convert nalgebra Isometry3 to Rapier Isometry
        fn to_rapier_isometry(iso: &Isometry3<f64>) -> rapier3d::math::Isometry<f64> {
            let translation = iso.translation;
            let rotation = iso.rotation;
            rapier3d::math::Isometry::from_parts(
                vector![translation.x, translation.y, translation.z],
                rapier3d::na::UnitQuaternion::from_quaternion(rapier3d::na::Quaternion::new(
                    rotation.coords[3],
                    rotation.coords[0],
                    rotation.coords[1],
                    rotation.coords[2],
                )),
            )
        }

        /// Convert Rapier velocity to our Velocity
        fn to_velocity(vel: &rapier3d::dynamics::RigidBodyVelocity) -> Velocity {
            Velocity {
                linear: Vector3::new(vel.linvel.x, vel.linvel.y, vel.linvel.z),
                angular: Vector3::new(vel.angvel.x, vel.angvel.y, vel.angvel.z),
            }
        }
    }

    impl Default for RapierBackend {
        fn default() -> Self {
            Self::new()
        }
    }

    impl PhysicsBackend for RapierBackend {
        fn step(&mut self, dt: Duration) {
            self.integration_parameters.dt = dt.as_secs_f64() as f32;

            self.pipeline.step(
                &vector![self.gravity.x, self.gravity.y, self.gravity.z],
                &self.integration_parameters,
                &mut self.island_manager,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.bodies,
                &mut self.colliders,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                &mut self.ccd_solver,
                None,
                &(),
                &(),
            );
        }

        fn add_rigid_body(&mut self, spec: RigidBodySpec) -> PhysicsResult<BodyHandle> {
            let body_type = self.to_rapier_body_type(spec.body_type);

            let mut builder = RigidBodyBuilder::new(body_type)
                .position(Self::to_rapier_isometry(&spec.position))
                .linvel(vector![
                    spec.linear_velocity.x,
                    spec.linear_velocity.y,
                    spec.linear_velocity.z
                ])
                .angvel(vector![
                    spec.angular_velocity.x,
                    spec.angular_velocity.y,
                    spec.angular_velocity.z
                ])
                .linear_damping(spec.linear_damping as f32)
                .angular_damping(spec.angular_damping as f32);

            if spec.body_type == BodyType::Dynamic {
                builder = builder.mass(spec.mass);
                if let Some(inertia) = spec.inertia {
                    builder = builder.additional_mass_properties(MassProperties::new(
                        point![0.0, 0.0, 0.0],
                        spec.mass as f32,
                        vector![inertia.x as f32, inertia.y as f32, inertia.z as f32],
                    ));
                }
            }

            let body_handle = self.bodies.insert(builder);

            // Add collider
            let shape = self.to_rapier_shape(&spec.shape);
            let collider = ColliderBuilder::new(shape)
                .restitution(spec.restitution as f32)
                .friction(spec.friction as f32);

            self.colliders
                .insert_with_parent(collider, body_handle, &mut self.bodies);

            // Create our handle
            let our_handle = BodyHandle(self.body_handles.len());
            let rapier_index = body_handle.index();

            // Extend mapping vectors if needed
            if rapier_index >= self.rapier_to_body.len() {
                self.rapier_to_body.resize(rapier_index + 1, None);
            }

            self.body_handles.push(body_handle);
            self.rapier_to_body[rapier_index] = Some(our_handle);

            Ok(our_handle)
        }

        fn remove_body(&mut self, handle: BodyHandle) -> PhysicsResult<()> {
            let rapier_handle = self.to_rapier_body(handle)?;

            // Remove colliders attached to this body
            let colliders_to_remove: Vec<_> = self
                .colliders
                .iter()
                .filter(|(_, c)| c.parent() == Some(rapier_handle))
                .map(|(h, _)| h)
                .collect();

            for collider_handle in colliders_to_remove {
                self.colliders.remove(
                    collider_handle,
                    &mut self.island_manager,
                    &mut self.bodies,
                    true,
                );
            }

            // Remove the body
            self.bodies.remove(
                rapier_handle,
                &mut self.island_manager,
                &mut self.colliders,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );

            // Update mapping
            self.rapier_to_body[rapier_handle.index()] = None;
            self.body_handles[handle.0] = RigidBodyHandle::invalid();

            Ok(())
        }

        fn add_joint(&mut self, spec: JointSpec) -> PhysicsResult<JointHandle> {
            let body_a = self.to_rapier_body(spec.body_a)?;
            let body_b = self.to_rapier_body(spec.body_b)?;

            let joint = self.to_rapier_joint(&spec);

            let handle = self.impulse_joints.insert(body_a, body_b, *joint, true);

            let our_handle = JointHandle(self.joint_handles.len());
            let rapier_index = handle.into_raw_parts().0;

            if rapier_index >= self.rapier_to_joint.len() {
                self.rapier_to_joint.resize(rapier_index + 1, None);
            }

            self.joint_handles.push(handle);
            self.rapier_to_joint[rapier_index] = Some(our_handle);

            Ok(our_handle)
        }

        fn remove_joint(&mut self, handle: JointHandle) -> PhysicsResult<()> {
            let rapier_handle = self.to_rapier_joint(handle)?;

            self.impulse_joints.remove(rapier_handle, true);

            self.rapier_to_joint[rapier_handle.into_raw_parts().0] = None;
            self.joint_handles[handle.0] = JointHandle::invalid();

            Ok(())
        }

        fn body_transform(&self, handle: BodyHandle) -> PhysicsResult<Isometry3<f64>> {
            let rapier_handle = self.to_rapier_body(handle)?;
            let body = self
                .bodies
                .get(rapier_handle)
                .ok_or(PhysicsError::InvalidBodyHandle(handle))?;

            Ok(Self::to_nalgebra_isometry(&body.position()))
        }

        fn body_velocity(&self, handle: BodyHandle) -> PhysicsResult<Velocity> {
            let rapier_handle = self.to_rapier_body(handle)?;
            let body = self
                .bodies
                .get(rapier_handle)
                .ok_or(PhysicsError::InvalidBodyHandle(handle))?;

            Ok(Self::to_velocity(&body.velocity()))
        }

        fn set_body_transform(
            &mut self,
            handle: BodyHandle,
            transform: Isometry3<f64>,
        ) -> PhysicsResult<()> {
            let rapier_handle = self.to_rapier_body(handle)?;
            let body = self
                .bodies
                .get_mut(rapier_handle)
                .ok_or(PhysicsError::InvalidBodyHandle(handle))?;

            body.set_position(Self::to_rapier_isometry(&transform), true);
            Ok(())
        }

        fn set_body_velocity(
            &mut self,
            handle: BodyHandle,
            velocity: Velocity,
        ) -> PhysicsResult<()> {
            let rapier_handle = self.to_rapier_body(handle)?;
            let body = self
                .bodies
                .get_mut(rapier_handle)
                .ok_or(PhysicsError::InvalidBodyHandle(handle))?;

            body.set_linvel(
                vector![velocity.linear.x, velocity.linear.y, velocity.linear.z],
                true,
            );
            body.set_angvel(
                vector![velocity.angular.x, velocity.angular.y, velocity.angular.z],
                true,
            );
            Ok(())
        }

        fn apply_force(&mut self, handle: BodyHandle, force: Vector3<f64>) -> PhysicsResult<()> {
            let rapier_handle = self.to_rapier_body(handle)?;
            let body = self
                .bodies
                .get_mut(rapier_handle)
                .ok_or(PhysicsError::InvalidBodyHandle(handle))?;

            body.add_force(vector![force.x, force.y, force.z], true);
            Ok(())
        }

        fn apply_torque(&mut self, handle: BodyHandle, torque: Vector3<f64>) -> PhysicsResult<()> {
            let rapier_handle = self.to_rapier_body(handle)?;
            let body = self
                .bodies
                .get_mut(rapier_handle)
                .ok_or(PhysicsError::InvalidBodyHandle(handle))?;

            body.add_torque(vector![torque.x, torque.y, torque.z], true);
            Ok(())
        }

        fn contacts(&self) -> Vec<super::Contact> {
            let mut contacts = Vec::new();

            for contact_pair in self.narrow_phase.contacts_with(..) {
                let body_a = self
                    .colliders
                    .get(contact_pair.collider1)
                    .and_then(|c| c.parent())
                    .and_then(|b| self.from_rapier_body(b));

                let body_b = self
                    .colliders
                    .get(contact_pair.collider2)
                    .and_then(|c| c.parent())
                    .and_then(|b| self.from_rapier_body(b));

                if let (Some(ba), Some(bb)) = (body_a, body_b) {
                    for manifold in contact_pair.manifolds() {
                        for point in manifold.points() {
                            contacts.push(super::Contact {
                                body_a: ba,
                                body_b: bb,
                                point: Vector3::new(
                                    manifold.solver_contact(point).point.x as f64,
                                    manifold.solver_contact(point).point.y as f64,
                                    manifold.solver_contact(point).point.z as f64,
                                ),
                                normal: Vector3::new(
                                    manifold.normal().x as f64,
                                    manifold.normal().y as f64,
                                    manifold.normal().z as f64,
                                ),
                                penetration: point.dist() as f64,
                                impulse: point.data.impulse() as f64,
                            });
                        }
                    }
                }
            }

            contacts
        }

        fn set_gravity(&mut self, gravity: Vector3<f64>) {
            self.gravity = gravity;
        }

        fn gravity(&self) -> Vector3<f64> {
            self.gravity
        }

        fn set_timestep(&mut self, timestep: Duration) {
            self.timestep = timestep;
            self.integration_parameters.dt = timestep.as_secs_f64() as f32;
        }

        fn timestep(&self) -> Duration {
            self.timestep
        }

        fn num_bodies(&self) -> usize {
            self.bodies.len()
        }

        fn num_joints(&self) -> usize {
            self.impulse_joints.len()
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_backend_creation() {
            let backend = RapierBackend::new();
            assert_eq!(backend.num_bodies(), 0);
            assert_eq!(backend.num_joints(), 0);
        }

        #[test]
        fn test_add_rigid_body() {
            let mut backend = RapierBackend::new();

            let spec = RigidBodySpec {
                body_type: BodyType::Dynamic,
                mass: 1.0,
                position: Isometry3::identity(),
                shape: Shape::Cuboid {
                    half_extents: Vector3::new(0.5, 0.5, 0.5),
                },
                ..Default::default()
            };

            let handle = backend.add_rigid_body(spec).unwrap();
            assert_eq!(backend.num_bodies(), 1);

            let transform = backend.body_transform(handle).unwrap();
            assert!((transform.translation.vector - Vector3::zeros()).norm() < 1e-6);
        }

        #[test]
        fn test_add_fixed_body() {
            let mut backend = RapierBackend::new();

            let spec = RigidBodySpec {
                body_type: BodyType::Fixed,
                mass: 0.0, // Ignored for fixed bodies
                position: Isometry3::identity(),
                shape: Shape::Cuboid {
                    half_extents: Vector3::new(10.0, 0.1, 10.0),
                },
                ..Default::default()
            };

            let handle = backend.add_rigid_body(spec).unwrap();
            assert_eq!(backend.num_bodies(), 1);

            // Fixed bodies should have identity transform
            let transform = backend.body_transform(handle).unwrap();
            assert!((transform.translation.vector - Vector3::zeros()).norm() < 1e-6);
        }

        #[test]
        fn test_add_joint() {
            let mut backend = RapierBackend::new();

            // Create two bodies
            let body_a = backend
                .add_rigid_body(RigidBodySpec {
                    body_type: BodyType::Dynamic,
                    mass: 1.0,
                    position: Isometry3::identity(),
                    shape: Shape::Cuboid {
                        half_extents: Vector3::new(0.5, 0.5, 0.5),
                    },
                    ..Default::default()
                })
                .unwrap();

            let body_b = backend
                .add_rigid_body(RigidBodySpec {
                    body_type: BodyType::Dynamic,
                    mass: 1.0,
                    position: Isometry3::from_parts(
                        nalgebra::Translation3::new(1.0, 0.0, 0.0),
                        nalgebra::UnitQuaternion::identity(),
                    ),
                    shape: Shape::Cuboid {
                        half_extents: Vector3::new(0.5, 0.5, 0.5),
                    },
                    ..Default::default()
                })
                .unwrap();

            // Create joint
            let joint_spec = JointSpec {
                joint_type: JointType::Revolute,
                body_a,
                body_b,
                local_anchor_a: Vector3::new(0.5, 0.0, 0.0),
                local_anchor_b: Vector3::new(-0.5, 0.0, 0.0),
                axis: Vector3::y_axis().into_inner(),
                ..Default::default()
            };

            let joint_handle = backend.add_joint(joint_spec).unwrap();
            assert_eq!(backend.num_joints(), 1);

            // Remove joint
            backend.remove_joint(joint_handle).unwrap();
            assert_eq!(backend.num_joints(), 0);
        }

        #[test]
        fn test_step_simulation() {
            let mut backend = RapierBackend::new();

            // Add a dynamic body above ground
            let body = backend
                .add_rigid_body(RigidBodySpec {
                    body_type: BodyType::Dynamic,
                    mass: 1.0,
                    position: Isometry3::from_parts(
                        nalgebra::Translation3::new(0.0, 10.0, 0.0),
                        nalgebra::UnitQuaternion::identity(),
                    ),
                    shape: Shape::Cuboid {
                        half_extents: Vector3::new(0.5, 0.5, 0.5),
                    },
                    ..Default::default()
                })
                .unwrap();

            // Step simulation
            let initial_pos = backend.body_transform(body).unwrap().translation.vector.y;
            backend.step(Duration::from_secs_f64(1.0 / 60.0));
            let new_pos = backend.body_transform(body).unwrap().translation.vector.y;

            // Body should have fallen due to gravity
            assert!(new_pos < initial_pos);
        }

        #[test]
        fn test_gravity_setting() {
            let mut backend = RapierBackend::new();

            assert!((backend.gravity() - Vector3::new(0.0, -9.81, 0.0)).norm() < 1e-6);

            backend.set_gravity(Vector3::new(0.0, -3.71, 0.0)); // Mars gravity
            assert!((backend.gravity() - Vector3::new(0.0, -3.71, 0.0)).norm() < 1e-6);
        }

        #[test]
        fn test_timestep_setting() {
            let mut backend = RapierBackend::new();

            let new_timestep = Duration::from_secs_f64(1.0 / 500.0);
            backend.set_timestep(new_timestep);
            assert_eq!(backend.timestep(), new_timestep);
        }

        #[test]
        fn test_invalid_body_handle() {
            let backend = RapierBackend::new();
            let invalid_handle = BodyHandle(999);

            assert!(matches!(
                backend.body_transform(invalid_handle),
                Err(PhysicsError::InvalidBodyHandle(_))
            ));
        }

        #[test]
        fn test_apply_force() {
            let mut backend = RapierBackend::new();

            let body = backend
                .add_rigid_body(RigidBodySpec {
                    body_type: BodyType::Dynamic,
                    mass: 1.0,
                    position: Isometry3::identity(),
                    shape: Shape::Cuboid {
                        half_extents: Vector3::new(0.5, 0.5, 0.5),
                    },
                    ..Default::default()
                })
                .unwrap();

            // Apply force and step
            backend
                .apply_force(body, Vector3::new(100.0, 0.0, 0.0))
                .unwrap();
            backend.step(Duration::from_secs_f64(1.0 / 60.0));

            // Check velocity
            let velocity = backend.body_velocity(body).unwrap();
            assert!(velocity.linear.x > 0.0);
        }

        #[test]
        fn test_contacts() {
            let mut backend = RapierBackend::new();

            // Create ground
            let _ground = backend
                .add_rigid_body(RigidBodySpec {
                    body_type: BodyType::Fixed,
                    mass: 0.0,
                    position: Isometry3::from_parts(
                        nalgebra::Translation3::new(0.0, -0.5, 0.0),
                        nalgebra::UnitQuaternion::identity(),
                    ),
                    shape: Shape::Cuboid {
                        half_extents: Vector3::new(10.0, 0.1, 10.0),
                    },
                    ..Default::default()
                })
                .unwrap();

            // Create a body sitting on ground
            let body = backend
                .add_rigid_body(RigidBodySpec {
                    body_type: BodyType::Dynamic,
                    mass: 1.0,
                    position: Isometry3::from_parts(
                        nalgebra::Translation3::new(0.0, 0.5, 0.0),
                        nalgebra::UnitQuaternion::identity(),
                    ),
                    shape: Shape::Cuboid {
                        half_extents: Vector3::new(0.5, 0.5, 0.5),
                    },
                    ..Default::default()
                })
                .unwrap();

            // Step to let them settle and generate contacts
            for _ in 0..10 {
                backend.step(Duration::from_secs_f64(1.0 / 60.0));
            }

            let contacts = backend.contacts();
            // Should have at least some contacts
            assert!(!contacts.is_empty());
        }
    }
}

#[cfg(feature = "simulator")]
pub use rapier_backend::RapierBackend;

// ============================================
// Backend Configuration and Selection
// ============================================

/// Configuration for physics backend selection
#[derive(Clone, Debug)]
pub struct BackendConfig {
    /// Backend type to use
    pub backend_type: BackendType,
    /// Simulation timestep
    pub timestep: Duration,
    /// Gravity vector
    pub gravity: Vector3<f64>,
}

impl Default for BackendConfig {
    fn default() -> Self {
        Self {
            backend_type: BackendType::default(),
            timestep: Duration::from_secs_f64(1.0 / 60.0),
            gravity: Vector3::new(0.0, -9.81, 0.0),
        }
    }
}

/// Available physics backend types
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BackendType {
    /// Rapier3D physics engine
    #[cfg(feature = "simulator")]
    Rapier,
    /// PhysX physics engine (placeholder for future implementation)
    #[cfg(feature = "physics-physx")]
    PhysX,
    /// Custom physics implementation (placeholder for future implementation)
    #[cfg(feature = "physics-custom")]
    Custom,
}

impl Default for BackendType {
    fn default() -> Self {
        #[cfg(feature = "simulator")]
        return BackendType::Rapier;

        #[cfg(not(feature = "simulator"))]
        panic!("No physics backend available. Enable simulator feature for Rapier support.")
    }
}

impl std::fmt::Display for BackendType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            #[cfg(feature = "simulator")]
            BackendType::Rapier => write!(f, "Rapier3D"),
            #[cfg(feature = "physics-physx")]
            BackendType::PhysX => write!(f, "PhysX"),
            #[cfg(feature = "physics-custom")]
            BackendType::Custom => write!(f, "Custom"),
            #[allow(unreachable_patterns)]
            _ => write!(f, "Unknown"),
        }
    }
}

/// Factory for creating physics backends
pub struct BackendFactory;

impl BackendFactory {
    /// Create a physics backend from configuration
    ///
    /// # Arguments
    /// * `config` - Backend configuration
    ///
    /// # Returns
    /// A boxed physics backend trait object
    pub fn create(config: &BackendConfig) -> Box<dyn PhysicsBackend> {
        match config.backend_type {
            #[cfg(feature = "simulator")]
            BackendType::Rapier => {
                let mut backend = RapierBackend::with_timestep(config.timestep);
                backend.set_gravity(config.gravity);
                Box::new(backend)
            }
            #[cfg(feature = "physics-physx")]
            BackendType::PhysX => {
                unimplemented!("PhysX backend not yet implemented")
            }
            #[cfg(feature = "physics-custom")]
            BackendType::Custom => {
                unimplemented!("Custom backend not yet implemented")
            }
            #[allow(unreachable_patterns)]
            _ => {
                panic!("No physics backend available. Enable simulator feature for Rapier support.")
            }
        }
    }

    /// Create a default Rapier backend
    #[cfg(feature = "simulator")]
    pub fn create_rapier() -> RapierBackend {
        RapierBackend::new()
    }

    /// Create a Rapier backend with custom timestep
    #[cfg(feature = "simulator")]
    pub fn create_rapier_with_timestep(timestep: Duration) -> RapierBackend {
        RapierBackend::with_timestep(timestep)
    }

    /// Get list of available backend types
    pub fn available_backends() -> Vec<BackendType> {
        let mut backends = Vec::new();

        #[cfg(feature = "simulator")]
        backends.push(BackendType::Rapier);

        #[cfg(feature = "physics-physx")]
        backends.push(BackendType::PhysX);

        #[cfg(feature = "physics-custom")]
        backends.push(BackendType::Custom);

        backends
    }
}

// ============================================
// Backend Abstraction Tests
// ============================================

#[cfg(test)]
mod abstraction_tests {
    use super::*;

    /// Test that demonstrates the backend abstraction works correctly
    /// This test should pass regardless of which backend is used
    fn test_backend_abstraction<B: PhysicsBackend>(backend: &mut B) {
        // Add a dynamic body
        let body = backend
            .add_rigid_body(RigidBodySpec {
                body_type: BodyType::Dynamic,
                mass: 1.0,
                position: Isometry3::from_parts(
                    nalgebra::Translation3::new(0.0, 10.0, 0.0),
                    nalgebra::UnitQuaternion::identity(),
                ),
                shape: Shape::Cuboid {
                    half_extents: Vector3::new(0.5, 0.5, 0.5),
                },
                ..Default::default()
            })
            .unwrap();

        // Step simulation
        let initial_pos = backend.body_transform(body).unwrap().translation.vector.y;
        backend.step(Duration::from_secs_f64(1.0 / 60.0));
        let new_pos = backend.body_transform(body).unwrap().translation.vector.y;

        // Body should have fallen due to gravity
        assert!(new_pos < initial_pos, "Body should fall due to gravity");

        // Test velocity query
        let velocity = backend.body_velocity(body).unwrap();
        assert!(
            velocity.linear.y < 0.0,
            "Body should have negative vertical velocity"
        );

        // Test applying force
        backend
            .apply_force(body, Vector3::new(10.0, 0.0, 0.0))
            .unwrap();
        backend.step(Duration::from_secs_f64(1.0 / 60.0));

        let velocity_after_force = backend.body_velocity(body).unwrap();
        assert!(
            velocity_after_force.linear.x > velocity.linear.x,
            "Force should increase horizontal velocity"
        );
    }

    /// Test body manipulation through the trait interface
    fn test_body_manipulation<B: PhysicsBackend>(backend: &mut B) {
        // Add body at origin
        let body = backend
            .add_rigid_body(RigidBodySpec {
                body_type: BodyType::Dynamic,
                mass: 1.0,
                position: Isometry3::identity(),
                shape: Shape::Sphere { radius: 1.0 },
                ..Default::default()
            })
            .unwrap();

        // Set transform
        let new_transform = Isometry3::from_parts(
            nalgebra::Translation3::new(5.0, 3.0, 2.0),
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 1.0),
        );
        backend.set_body_transform(body, new_transform).unwrap();

        let retrieved_transform = backend.body_transform(body).unwrap();
        assert!(
            (retrieved_transform.translation.vector - Vector3::new(5.0, 3.0, 2.0)).norm() < 1e-5,
            "Transform position should be set correctly"
        );

        // Set velocity
        let new_velocity = Velocity {
            linear: Vector3::new(1.0, 2.0, 3.0),
            angular: Vector3::new(0.1, 0.2, 0.3),
        };
        backend
            .set_body_velocity(body, new_velocity.clone())
            .unwrap();

        let retrieved_velocity = backend.body_velocity(body).unwrap();
        assert!(
            (retrieved_velocity.linear - new_velocity.linear).norm() < 1e-5,
            "Linear velocity should be set correctly"
        );
    }

    /// Test joint creation and manipulation
    fn test_joint_operations<B: PhysicsBackend>(backend: &mut B) {
        // Create two bodies
        let body_a = backend
            .add_rigid_body(RigidBodySpec {
                body_type: BodyType::Fixed,
                mass: 0.0,
                position: Isometry3::identity(),
                shape: Shape::Cuboid {
                    half_extents: Vector3::new(1.0, 0.1, 1.0),
                },
                ..Default::default()
            })
            .unwrap();

        let body_b = backend
            .add_rigid_body(RigidBodySpec {
                body_type: BodyType::Dynamic,
                mass: 1.0,
                position: Isometry3::from_parts(
                    nalgebra::Translation3::new(2.0, 0.0, 0.0),
                    nalgebra::UnitQuaternion::identity(),
                ),
                shape: Shape::Cuboid {
                    half_extents: Vector3::new(0.5, 0.5, 0.5),
                },
                ..Default::default()
            })
            .unwrap();

        // Create revolute joint
        let joint = backend
            .add_joint(JointSpec {
                joint_type: JointType::Revolute,
                body_a,
                body_b,
                local_anchor_a: Vector3::new(1.0, 0.0, 0.0),
                local_anchor_b: Vector3::new(-0.5, 0.0, 0.0),
                axis: Vector3::y_axis().into_inner(),
                position_limits: Some((-1.0, 1.0)),
                ..Default::default()
            })
            .unwrap();

        assert_eq!(backend.num_joints(), 1, "Should have one joint");

        // Remove joint
        backend.remove_joint(joint).unwrap();
        assert_eq!(
            backend.num_joints(),
            0,
            "Should have no joints after removal"
        );
    }

    #[cfg(feature = "simulator")]
    mod rapier_tests {
        use super::*;

        #[test]
        fn test_rapier_backend_trait() {
            let mut backend = RapierBackend::new();
            test_backend_abstraction(&mut backend);
        }

        #[test]
        fn test_rapier_body_manipulation() {
            let mut backend = RapierBackend::new();
            test_body_manipulation(&mut backend);
        }

        #[test]
        fn test_rapier_joint_operations() {
            let mut backend = RapierBackend::new();
            test_joint_operations(&mut backend);
        }

        #[test]
        fn test_backend_factory() {
            let config = BackendConfig {
                backend_type: BackendType::Rapier,
                timestep: Duration::from_secs_f64(1.0 / 500.0),
                gravity: Vector3::new(0.0, -3.71, 0.0),
            };

            let mut backend = BackendFactory::create(&config);

            assert_eq!(backend.timestep(), config.timestep);
            assert!((backend.gravity() - config.gravity).norm() < 1e-6);

            // Test that the trait object works
            test_backend_abstraction(&mut *backend);
        }

        #[test]
        fn test_available_backends() {
            let backends = BackendFactory::available_backends();
            assert!(
                !backends.is_empty(),
                "Should have at least one backend available"
            );
            assert!(
                backends.contains(&BackendType::Rapier),
                "Should have Rapier backend"
            );
        }

        #[test]
        fn test_backend_type_display() {
            assert_eq!(format!("{}", BackendType::Rapier), "Rapier3D");
        }
    }

    #[test]
    fn test_backend_config_default() {
        let config = BackendConfig::default();
        assert_eq!(config.timestep, Duration::from_secs_f64(1.0 / 60.0));
        assert!((config.gravity - Vector3::new(0.0, -9.81, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn test_error_handling() {
        let backend = RapierBackend::new();
        let invalid_handle = BodyHandle(99999);

        // Should return error for invalid handle
        match backend.body_transform(invalid_handle) {
            Err(PhysicsError::InvalidBodyHandle(h)) => assert_eq!(h, invalid_handle),
            _ => panic!("Should return InvalidBodyHandle error"),
        }

        match backend.body_velocity(invalid_handle) {
            Err(PhysicsError::InvalidBodyHandle(h)) => assert_eq!(h, invalid_handle),
            _ => panic!("Should return InvalidBodyHandle error"),
        }
    }
}
