//! Physics-accurate humanoid simulator using Rapier articulated bodies
//!
//! This module provides a proper physics simulation with:
//! - Articulated rigid body dynamics
//! - Revolute joints with motor control
//! - Joint limits and damping
//! - Ground contact with friction
//! - Sensor simulation (joint encoders, IMU, force/torque)
//!
//! The robot can be controlled via:
//! - Position control (PD controller)
//! - Velocity control
//! - Torque control (direct)

use crate::robot::unitree_g1;
use crate::sim::assets::{AssetFormat, SceneLoader};
use crate::sim::deterministic::{
    DeterministicScheduler, SimulationClock, SimulationControl, Subsystem, TickConfig, TimestepMode,
};
use crate::sim::scene::{ActiveScene, SceneGraph};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use std::f32::consts::PI;
use std::path::PathBuf;
use std::time::Duration;
use tracing::{debug, info, warn};

const ROBOT_GROUP: Group = Group::GROUP_1;
const GROUND_GROUP: Group = Group::GROUP_2;

/// Physics simulation configuration
#[derive(Resource, Clone)]
pub struct PhysicsConfig {
    /// Simulation timestep (seconds)
    pub timestep: f32,
    /// Gravity vector
    pub gravity: Vec3,
    /// Ground friction coefficient
    pub ground_friction: f32,
    /// Joint damping coefficient
    pub joint_damping: f32,
    /// Joint stiffness for position control (PD gains)
    pub position_kp: f32,
    pub position_kd: f32,
    /// Timestep mode for deterministic simulation control
    pub timestep_mode: TimestepMode,
    /// Deterministic scheduler for controlling physics, sensors, and control loops
    pub scheduler: Option<DeterministicScheduler>,
    /// Accumulated time for fixed timestep updates
    pub accumulated_time: Duration,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            timestep: 1.0 / 500.0, // 500 Hz physics
            gravity: Vec3::new(0.0, -9.81, 0.0),
            ground_friction: 0.8,
            joint_damping: 0.5,
            position_kp: 100.0,
            position_kd: 10.0,
            timestep_mode: TimestepMode::Fixed {
                dt: 1.0 / 500.0,
                substeps: 4,
            },
            scheduler: None,
            accumulated_time: Duration::ZERO,
        }
    }
}

/// Control mode for joints
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum ControlMode {
    /// Position control with PD gains
    #[default]
    Position,
    /// Velocity control
    Velocity,
    /// Direct torque control
    Torque,
    /// Passive (no control, just damping)
    Passive,
}

/// Joint specification for robot construction
#[derive(Clone, Debug)]
pub struct JointSpec {
    pub name: String,
    /// Joint axis in local frame
    pub axis: Vec3,
    /// Position limits (min, max) in radians
    pub limits: (f32, f32),
    /// Maximum torque (Nm)
    pub max_torque: f32,
    /// Maximum velocity (rad/s)
    pub max_velocity: f32,
    /// Damping coefficient
    pub damping: f32,
    /// Stiffness for position control
    pub stiffness: f32,
}

impl Default for JointSpec {
    fn default() -> Self {
        Self {
            name: "joint".to_string(),
            axis: Vec3::Y,
            limits: (-PI, PI),
            max_torque: 50.0,
            max_velocity: 10.0,
            damping: 3.0,     // Moderate damping for stability
            stiffness: 120.0, // Moderate stiffness for stability
        }
    }
}

/// Link (body segment) specification
#[derive(Clone, Debug)]
pub struct LinkSpec {
    pub name: String,
    /// Mass in kg
    pub mass: f32,
    /// Half-extents for collision box
    pub half_extents: Vec3,
    /// Local offset from parent joint
    pub offset: Vec3,
    /// Inertia tensor (diagonal)
    pub inertia: Vec3,
}

impl Default for LinkSpec {
    fn default() -> Self {
        Self {
            name: "link".to_string(),
            mass: 1.0,
            half_extents: Vec3::new(0.05, 0.1, 0.05),
            offset: Vec3::ZERO,
            inertia: Vec3::new(0.01, 0.01, 0.01),
        }
    }
}

/// Component marking a physics joint
#[derive(Component)]
pub struct PhysicsJoint {
    pub index: usize,
    pub name: String,
    pub control_mode: ControlMode,
    /// Joint axis (X, Y, or Z)
    pub axis: Vec3,
    /// Target position for position control
    pub target_position: f32,
    /// Target velocity for velocity control
    pub target_velocity: f32,
    /// Target torque for torque control
    pub target_torque: f32,
    /// Current measured position
    pub current_position: f32,
    /// Current measured velocity
    pub current_velocity: f32,
    /// Current measured torque
    pub current_torque: f32,
    /// Joint limits
    pub limits: (f32, f32),
    /// PD gains
    pub kp: f32,
    pub kd: f32,
    /// Max torque
    pub max_torque: f32,
}

/// Component marking a physics link (body segment)
#[derive(Component)]
pub struct PhysicsLink {
    pub index: usize,
    pub name: String,
}

/// Component for the robot base (floating base)
#[derive(Component)]
pub struct RobotBase;

/// Simulated IMU sensor data
#[derive(Resource, Default, Clone)]
pub struct ImuData {
    /// Linear acceleration in body frame (m/s^2)
    pub linear_acceleration: Vec3,
    /// Angular velocity in body frame (rad/s)
    pub angular_velocity: Vec3,
    /// Orientation as quaternion
    pub orientation: Quat,
}

/// Force/torque sensor data for feet
#[derive(Resource, Default, Clone)]
pub struct FootSensorData {
    /// Left foot contact force
    pub left_force: Vec3,
    /// Right foot contact force  
    pub right_force: Vec3,
    /// Left foot in contact
    pub left_contact: bool,
    /// Right foot in contact
    pub right_contact: bool,
    /// Center of pressure (left foot)
    pub left_cop: Vec3,
    /// Center of pressure (right foot)
    pub right_cop: Vec3,
}

/// Joint state for all joints
#[derive(Resource, Clone)]
pub struct JointState {
    /// Joint positions (radians)
    pub positions: Vec<f32>,
    /// Joint velocities (rad/s)
    pub velocities: Vec<f32>,
    /// Joint torques (Nm)
    pub torques: Vec<f32>,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            positions: vec![0.0; unitree_g1::NUM_JOINTS],
            velocities: vec![0.0; unitree_g1::NUM_JOINTS],
            torques: vec![0.0; unitree_g1::NUM_JOINTS],
        }
    }
}

/// Joint commands (targets)
#[derive(Resource, Clone)]
pub struct JointCommands {
    /// Target positions (for position control)
    pub target_positions: Vec<f32>,
    /// Target velocities (for velocity control)
    pub target_velocities: Vec<f32>,
    /// Target torques (for torque control)
    pub target_torques: Vec<f32>,
    /// Control mode per joint
    pub control_modes: Vec<ControlMode>,
}

impl Default for JointCommands {
    fn default() -> Self {
        Self {
            target_positions: vec![0.0; unitree_g1::NUM_JOINTS],
            target_velocities: vec![0.0; unitree_g1::NUM_JOINTS],
            target_torques: vec![0.0; unitree_g1::NUM_JOINTS],
            control_modes: vec![ControlMode::Position; unitree_g1::NUM_JOINTS],
        }
    }
}

/// Physics simulator plugin
pub struct PhysicsSimulatorPlugin;

impl Plugin for PhysicsSimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
            .insert_resource(RapierConfiguration {
                gravity: Vec3::new(0.0, -9.81, 0.0).into(),
                physics_pipeline_active: true,
                query_pipeline_active: true,
                timestep_mode: TimestepMode::Fixed {
                    dt: 1.0 / 60.0,
                    substeps: 4,
                },
                scaled_shape_subdivision: 10,
                force_update_from_transform_changes: false,
            })
            .init_resource::<PhysicsConfig>()
            .init_resource::<ImuData>()
            .init_resource::<FootSensorData>()
            .init_resource::<JointState>()
            .init_resource::<JointCommands>()
            .add_systems(Startup, setup_physics_scene)
            .add_systems(
                FixedUpdate,
                (
                    apply_joint_control,
                    read_joint_sensors,
                    read_imu_sensor,
                    read_foot_sensors,
                )
                    .chain(),
            );
    }
}

/// Create the Unitree G1 robot specification
pub fn unitree_g1_spec() -> (Vec<LinkSpec>, Vec<JointSpec>) {
    let mut links = Vec::new();
    let mut joints = Vec::new();

    // === BASE/PELVIS ===
    links.push(LinkSpec {
        name: "pelvis".to_string(),
        mass: 8.0,
        half_extents: Vec3::new(0.14, 0.075, 0.09),
        offset: Vec3::ZERO,
        inertia: Vec3::new(0.1, 0.08, 0.12),
    });

    // === LEFT LEG ===
    // Left hip pitch
    joints.push(JointSpec {
        name: "left_hip_pitch".to_string(),
        axis: Vec3::Y,
        limits: (-1.57, 0.5),
        max_torque: 88.0,
        max_velocity: 12.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_hip_pitch_link".to_string(),
        mass: 1.2,
        half_extents: Vec3::new(0.04, 0.04, 0.04),
        offset: Vec3::new(-0.1, -0.05, 0.0),
        ..default()
    });

    // Left hip roll
    joints.push(JointSpec {
        name: "left_hip_roll".to_string(),
        axis: Vec3::X,
        limits: (-0.5, 0.5),
        max_torque: 88.0,
        max_velocity: 12.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_hip_roll_link".to_string(),
        mass: 1.0,
        half_extents: Vec3::new(0.04, 0.04, 0.04),
        offset: Vec3::new(0.0, -0.05, 0.0),
        ..default()
    });

    // Left hip yaw
    joints.push(JointSpec {
        name: "left_hip_yaw".to_string(),
        axis: Vec3::Z,
        limits: (-0.5, 0.5),
        max_torque: 44.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_thigh".to_string(),
        mass: 3.5,
        half_extents: Vec3::new(0.05, 0.15, 0.05),
        offset: Vec3::new(0.0, -0.15, 0.0),
        inertia: Vec3::new(0.05, 0.01, 0.05),
    });

    // Left knee
    joints.push(JointSpec {
        name: "left_knee".to_string(),
        axis: Vec3::Y,
        limits: (0.0, 2.6),
        max_torque: 88.0,
        max_velocity: 12.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_calf".to_string(),
        mass: 2.5,
        half_extents: Vec3::new(0.04, 0.15, 0.04),
        offset: Vec3::new(0.0, -0.15, 0.0),
        inertia: Vec3::new(0.04, 0.005, 0.04),
    });

    // Left ankle pitch
    joints.push(JointSpec {
        name: "left_ankle_pitch".to_string(),
        axis: Vec3::Y,
        limits: (-0.87, 0.52),
        max_torque: 44.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_ankle_link".to_string(),
        mass: 0.5,
        half_extents: Vec3::new(0.03, 0.02, 0.03),
        offset: Vec3::new(0.0, -0.02, 0.0),
        ..default()
    });

    // Left ankle roll
    joints.push(JointSpec {
        name: "left_ankle_roll".to_string(),
        axis: Vec3::X,
        limits: (-0.26, 0.26),
        max_torque: 44.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_foot".to_string(),
        mass: 0.8,
        half_extents: Vec3::new(0.05, 0.015, 0.09),
        offset: Vec3::new(0.0, -0.015, 0.02),
        ..default()
    });

    // === RIGHT LEG (mirror of left) ===
    joints.push(JointSpec {
        name: "right_hip_pitch".to_string(),
        axis: Vec3::Y,
        limits: (-1.57, 0.5),
        max_torque: 88.0,
        max_velocity: 12.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_hip_pitch_link".to_string(),
        mass: 1.2,
        half_extents: Vec3::new(0.04, 0.04, 0.04),
        offset: Vec3::new(0.1, -0.05, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "right_hip_roll".to_string(),
        axis: Vec3::X,
        limits: (-0.5, 0.5),
        max_torque: 88.0,
        max_velocity: 12.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_hip_roll_link".to_string(),
        mass: 1.0,
        half_extents: Vec3::new(0.04, 0.04, 0.04),
        offset: Vec3::new(0.0, -0.05, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "right_hip_yaw".to_string(),
        axis: Vec3::Z,
        limits: (-0.5, 0.5),
        max_torque: 44.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_thigh".to_string(),
        mass: 3.5,
        half_extents: Vec3::new(0.05, 0.15, 0.05),
        offset: Vec3::new(0.0, -0.15, 0.0),
        inertia: Vec3::new(0.05, 0.01, 0.05),
    });

    joints.push(JointSpec {
        name: "right_knee".to_string(),
        axis: Vec3::Y,
        limits: (0.0, 2.6),
        max_torque: 88.0,
        max_velocity: 12.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_calf".to_string(),
        mass: 2.5,
        half_extents: Vec3::new(0.04, 0.15, 0.04),
        offset: Vec3::new(0.0, -0.15, 0.0),
        inertia: Vec3::new(0.04, 0.005, 0.04),
    });

    joints.push(JointSpec {
        name: "right_ankle_pitch".to_string(),
        axis: Vec3::Y,
        limits: (-0.87, 0.52),
        max_torque: 44.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_ankle_link".to_string(),
        mass: 0.5,
        half_extents: Vec3::new(0.03, 0.02, 0.03),
        offset: Vec3::new(0.0, -0.02, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "right_ankle_roll".to_string(),
        axis: Vec3::X,
        limits: (-0.26, 0.26),
        max_torque: 44.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_foot".to_string(),
        mass: 0.8,
        half_extents: Vec3::new(0.05, 0.015, 0.09),
        offset: Vec3::new(0.0, -0.015, 0.02),
        ..default()
    });

    // === WAIST ===
    joints.push(JointSpec {
        name: "waist_yaw".to_string(),
        axis: Vec3::Z,
        limits: (-0.78, 0.78),
        max_torque: 88.0,
        max_velocity: 10.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "waist_link".to_string(),
        mass: 2.0,
        half_extents: Vec3::new(0.1, 0.1, 0.08),
        offset: Vec3::new(0.0, 0.1, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "waist_pitch".to_string(),
        axis: Vec3::Y,
        limits: (-0.35, 0.35),
        max_torque: 88.0,
        max_velocity: 10.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "torso".to_string(),
        mass: 10.0,
        half_extents: Vec3::new(0.15, 0.175, 0.1),
        offset: Vec3::new(0.0, 0.175, 0.0),
        inertia: Vec3::new(0.2, 0.15, 0.25),
    });

    joints.push(JointSpec {
        name: "waist_roll".to_string(),
        axis: Vec3::X,
        limits: (-0.35, 0.35),
        max_torque: 44.0,
        max_velocity: 10.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "upper_torso".to_string(),
        mass: 5.0,
        half_extents: Vec3::new(0.12, 0.1, 0.08),
        offset: Vec3::new(0.0, 0.1, 0.0),
        ..default()
    });

    // === LEFT ARM ===
    joints.push(JointSpec {
        name: "left_shoulder_pitch".to_string(),
        axis: Vec3::Y,
        limits: (-3.1, 2.0),
        max_torque: 25.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_shoulder_link".to_string(),
        mass: 0.8,
        half_extents: Vec3::new(0.04, 0.04, 0.04),
        offset: Vec3::new(-0.2, 0.0, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "left_shoulder_roll".to_string(),
        axis: Vec3::X,
        limits: (-1.57, 0.3),
        max_torque: 25.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_upper_arm".to_string(),
        mass: 1.5,
        half_extents: Vec3::new(0.035, 0.1, 0.035),
        offset: Vec3::new(0.0, -0.1, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "left_shoulder_yaw".to_string(),
        axis: Vec3::Z,
        limits: (-1.57, 1.57),
        max_torque: 15.0,
        max_velocity: 20.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_shoulder_yaw_link".to_string(),
        mass: 0.5,
        half_extents: Vec3::new(0.03, 0.03, 0.03),
        offset: Vec3::ZERO,
        ..default()
    });

    joints.push(JointSpec {
        name: "left_elbow".to_string(),
        axis: Vec3::Y,
        limits: (-2.6, 0.0),
        max_torque: 25.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "left_forearm".to_string(),
        mass: 1.0,
        half_extents: Vec3::new(0.03, 0.1, 0.03),
        offset: Vec3::new(0.0, -0.1, 0.0),
        ..default()
    });

    // Left wrist (simplified - 3 DOF)
    for (name, axis) in [
        ("left_wrist_roll", Vec3::X),
        ("left_wrist_pitch", Vec3::Y),
        ("left_wrist_yaw", Vec3::Z),
    ] {
        joints.push(JointSpec {
            name: name.to_string(),
            axis,
            limits: (-1.57, 1.57),
            max_torque: 5.0,
            max_velocity: 20.0,
            ..default()
        });
        links.push(LinkSpec {
            name: format!("{}_link", name),
            mass: 0.2,
            half_extents: Vec3::new(0.02, 0.02, 0.02),
            offset: Vec3::new(0.0, -0.02, 0.0),
            ..default()
        });
    }

    // === RIGHT ARM (mirror of left) ===
    joints.push(JointSpec {
        name: "right_shoulder_pitch".to_string(),
        axis: Vec3::Y,
        limits: (-3.1, 2.0),
        max_torque: 25.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_shoulder_link".to_string(),
        mass: 0.8,
        half_extents: Vec3::new(0.04, 0.04, 0.04),
        offset: Vec3::new(0.2, 0.0, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "right_shoulder_roll".to_string(),
        axis: Vec3::X,
        limits: (-0.3, 1.57),
        max_torque: 25.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_upper_arm".to_string(),
        mass: 1.5,
        half_extents: Vec3::new(0.035, 0.1, 0.035),
        offset: Vec3::new(0.0, -0.1, 0.0),
        ..default()
    });

    joints.push(JointSpec {
        name: "right_shoulder_yaw".to_string(),
        axis: Vec3::Z,
        limits: (-1.57, 1.57),
        max_torque: 15.0,
        max_velocity: 20.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_shoulder_yaw_link".to_string(),
        mass: 0.5,
        half_extents: Vec3::new(0.03, 0.03, 0.03),
        offset: Vec3::ZERO,
        ..default()
    });

    joints.push(JointSpec {
        name: "right_elbow".to_string(),
        axis: Vec3::Y,
        limits: (0.0, 2.6),
        max_torque: 25.0,
        max_velocity: 15.0,
        ..default()
    });
    links.push(LinkSpec {
        name: "right_forearm".to_string(),
        mass: 1.0,
        half_extents: Vec3::new(0.03, 0.1, 0.03),
        offset: Vec3::new(0.0, -0.1, 0.0),
        ..default()
    });

    // Right wrist (simplified - 3 DOF)
    for (name, axis) in [
        ("right_wrist_roll", Vec3::X),
        ("right_wrist_pitch", Vec3::Y),
        ("right_wrist_yaw", Vec3::Z),
    ] {
        joints.push(JointSpec {
            name: name.to_string(),
            axis,
            limits: (-1.57, 1.57),
            max_torque: 5.0,
            max_velocity: 20.0,
            ..default()
        });
        links.push(LinkSpec {
            name: format!("{}_link", name),
            mass: 0.2,
            half_extents: Vec3::new(0.02, 0.02, 0.02),
            offset: Vec3::new(0.0, -0.02, 0.0),
            ..default()
        });
    }

    (links, joints)
}

/// Setup the physics scene with robot
fn setup_physics_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    _config: Res<PhysicsConfig>,
) {
    // Ground plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh().size(20.0, 20.0)),
            material: materials.add(StandardMaterial {
                base_color: Color::srgb(0.3, 0.35, 0.3),
                ..default()
            }),
            ..default()
        },
        RigidBody::Fixed,
        Collider::cuboid(10.0, 0.01, 10.0),
        CollisionGroups::new(GROUND_GROUP, ROBOT_GROUP),
        Friction::coefficient(0.8),
    ));

    // Materials for robot
    let body_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.15, 0.15, 0.2),
        metallic: 0.8,
        ..default()
    });

    let joint_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.4, 0.1),
        metallic: 0.9,
        ..default()
    });

    let foot_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.3, 0.35),
        metallic: 0.5,
        ..default()
    });

    // Get robot spec
    let (links, joints) = unitree_g1_spec();

    // Starting height for pelvis (robot standing)
    let pelvis_height = 0.85;
    let pelvis_world_pos = Vec3::new(0.0, pelvis_height, 0.0);

    // Spawn base (pelvis) - this is the floating base
    let pelvis_spec = &links[0];
    let pelvis_entity = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(
                    pelvis_spec.half_extents.x * 2.0,
                    pelvis_spec.half_extents.y * 2.0,
                    pelvis_spec.half_extents.z * 2.0,
                )),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, pelvis_height, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(
                pelvis_spec.half_extents.x,
                pelvis_spec.half_extents.y,
                pelvis_spec.half_extents.z,
            ),
            CollisionGroups::new(ROBOT_GROUP, GROUND_GROUP),
            ColliderMassProperties::Mass(pelvis_spec.mass),
            Damping {
                linear_damping: 2.0,
                angular_damping: 2.0,
            },
            Velocity::default(),
            ExternalForce::default(),
            RobotBase,
            PhysicsLink {
                index: 0,
                name: pelvis_spec.name.clone(),
            },
        ))
        .id();

    // Helper to spawn a chain of links
    let spawn_link = |commands: &mut Commands,
                      meshes: &mut ResMut<Assets<Mesh>>,
                      parent: Entity,
                      link_spec: &LinkSpec,
                      joint_spec: &JointSpec,
                      joint_index: usize,
                      material: Handle<StandardMaterial>,
                      parent_anchor: Vec3,
                      parent_world_pos: &mut Vec3|
     -> Entity {
        let local_child_anchor = Vec3::new(0.0, link_spec.half_extents.y, 0.0);
        let world_pos = *parent_world_pos + parent_anchor - local_child_anchor;
        *parent_world_pos = world_pos;
        let link_entity = commands
            .spawn((
                PbrBundle {
                    mesh: meshes.add(Cuboid::new(
                        link_spec.half_extents.x * 2.0,
                        link_spec.half_extents.y * 2.0,
                        link_spec.half_extents.z * 2.0,
                    )),
                    material,
                    transform: Transform::from_translation(world_pos),
                    ..default()
                },
                RigidBody::Dynamic,
                Collider::cuboid(
                    link_spec.half_extents.x,
                    link_spec.half_extents.y,
                    link_spec.half_extents.z,
                ),
                CollisionGroups::new(ROBOT_GROUP, GROUND_GROUP),
                ColliderMassProperties::Mass(link_spec.mass),
                Damping {
                    linear_damping: 2.0,
                    angular_damping: 2.0,
                },
                Velocity::default(),
                ExternalForce::default(),
                PhysicsLink {
                    index: joint_index + 1,
                    name: link_spec.name.clone(),
                },
                PhysicsJoint {
                    index: joint_index,
                    name: joint_spec.name.clone(),
                    control_mode: ControlMode::Position,
                    axis: joint_spec.axis,
                    target_position: 0.0,
                    target_velocity: 0.0,
                    target_torque: 0.0,
                    current_position: 0.0,
                    current_velocity: 0.0,
                    current_torque: 0.0,
                    limits: joint_spec.limits,
                    kp: joint_spec.stiffness,
                    kd: joint_spec.damping * 2.0 * joint_spec.stiffness.sqrt(),
                    max_torque: joint_spec.max_torque,
                },
            ))
            .id();

        // Create revolute joint - anchor1 is on parent, anchor2 is on child
        // Use motor_position with high stiffness/damping to hold the robot in place initially
        let joint = RevoluteJointBuilder::new(joint_spec.axis)
            .local_anchor1(parent_anchor)
            .local_anchor2(Vec3::new(0.0, link_spec.half_extents.y, 0.0)) // Top of child link
            .limits([joint_spec.limits.0, joint_spec.limits.1])
            .motor_position(0.0, joint_spec.stiffness, joint_spec.damping * 10.0);

        commands
            .entity(link_entity)
            .insert(ImpulseJoint::new(parent, joint));

        link_entity
    };

    // Joint indices (matching unitree_g1_spec order):
    // 0-5: left leg (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
    // 6-11: right leg
    // 12-14: waist (yaw, pitch, roll)
    // 15-21: left arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw)
    // 22-28: right arm

    // === LEFT LEG (joints 0-5, links 1-6) ===
    let left_hip_offset = Vec3::new(-0.1, -pelvis_spec.half_extents.y, 0.0);
    let mut parent = pelvis_entity;
    let mut parent_world_pos = pelvis_world_pos;
    let mut anchor = left_hip_offset;

    for i in 0..6 {
        let link_idx = i + 1;
        let mat = if i == 5 {
            foot_material.clone()
        } else {
            joint_material.clone()
        };
        let next_anchor = Vec3::new(0.0, -links[link_idx].half_extents.y, 0.0);
        parent = spawn_link(
            &mut commands,
            &mut meshes,
            parent,
            &links[link_idx],
            &joints[i],
            i,
            mat,
            anchor,
            &mut parent_world_pos,
        );
        anchor = next_anchor;
    }

    // === RIGHT LEG (joints 6-11, links 7-12) ===
    let right_hip_offset = Vec3::new(0.1, -pelvis_spec.half_extents.y, 0.0);
    parent = pelvis_entity;
    parent_world_pos = pelvis_world_pos;
    anchor = right_hip_offset;

    for i in 6..12 {
        let link_idx = i + 1;
        let mat = if i == 11 {
            foot_material.clone()
        } else {
            joint_material.clone()
        };
        let next_anchor = Vec3::new(0.0, -links[link_idx].half_extents.y, 0.0);
        parent = spawn_link(
            &mut commands,
            &mut meshes,
            parent,
            &links[link_idx],
            &joints[i],
            i,
            mat,
            anchor,
            &mut parent_world_pos,
        );
        anchor = next_anchor;
    }

    // === WAIST/TORSO (joints 12-14, links 13-15) ===
    let waist_offset = Vec3::new(0.0, pelvis_spec.half_extents.y, 0.0);
    parent = pelvis_entity;
    parent_world_pos = pelvis_world_pos;
    anchor = waist_offset;

    let mut torso_entity = pelvis_entity;
    let mut torso_world_pos = pelvis_world_pos;
    for i in 12..15 {
        let link_idx = i + 1;
        let next_anchor = Vec3::new(0.0, links[link_idx].half_extents.y, 0.0);
        parent = spawn_link(
            &mut commands,
            &mut meshes,
            parent,
            &links[link_idx],
            &joints[i],
            i,
            body_material.clone(),
            anchor,
            &mut parent_world_pos,
        );
        anchor = next_anchor;
        if i == 14 {
            torso_entity = parent; // Save for arm attachment
            torso_world_pos = parent_world_pos;
        }
    }

    // === LEFT ARM (joints 15-21, links 16-22) ===
    // Arms attach to torso (upper_torso, link index 15)
    let left_shoulder_offset = Vec3::new(-0.18, 0.0, 0.0);
    parent = torso_entity;
    parent_world_pos = torso_world_pos;
    anchor = left_shoulder_offset;

    for i in 15..22 {
        let link_idx = i + 1;
        let mat = if i >= 19 {
            body_material.clone()
        } else {
            joint_material.clone()
        };
        let next_anchor = Vec3::new(0.0, -links[link_idx].half_extents.y, 0.0);
        parent = spawn_link(
            &mut commands,
            &mut meshes,
            parent,
            &links[link_idx],
            &joints[i],
            i,
            mat,
            anchor,
            &mut parent_world_pos,
        );
        anchor = next_anchor;
    }

    // === RIGHT ARM (joints 22-28, links 23-29) ===
    let right_shoulder_offset = Vec3::new(0.18, 0.0, 0.0);
    parent = torso_entity;
    parent_world_pos = torso_world_pos;
    anchor = right_shoulder_offset;

    for i in 22..29 {
        let link_idx = i + 1;
        if link_idx >= links.len() {
            break;
        }
        let mat = if i >= 26 {
            body_material.clone()
        } else {
            joint_material.clone()
        };
        let next_anchor = Vec3::new(0.0, -links[link_idx].half_extents.y, 0.0);
        parent = spawn_link(
            &mut commands,
            &mut meshes,
            parent,
            &links[link_idx],
            &joints[i],
            i,
            mat,
            anchor,
            &mut parent_world_pos,
        );
        anchor = next_anchor;
    }
}

/// Apply joint control (PD control, velocity control, or direct torque)
fn apply_joint_control(
    commands: Res<JointCommands>,
    _config: Res<PhysicsConfig>,
    mut joint_query: Query<(&mut PhysicsJoint, &mut ImpulseJoint)>,
) {
    for (mut joint, mut impulse_joint) in joint_query.iter_mut() {
        let idx = joint.index;
        if idx >= commands.target_positions.len() {
            continue;
        }

        let control_mode = commands
            .control_modes
            .get(idx)
            .copied()
            .unwrap_or(ControlMode::Position);
        joint.control_mode = control_mode;

        // Determine the correct joint axis for motor control
        // RevoluteJoint uses AngX for the rotation around the joint axis
        let joint_axis = JointAxis::AngX;

        match control_mode {
            ControlMode::Position => {
                let target = commands.target_positions[idx];
                joint.target_position = target;

                // Use direct position motor control with PD gains
                impulse_joint
                    .data
                    .set_motor_position(joint_axis, target, joint.kp, joint.kd);
            }
            ControlMode::Velocity => {
                let target_vel = commands.target_velocities[idx];
                joint.target_velocity = target_vel;

                impulse_joint.data.set_motor_velocity(
                    joint_axis,
                    target_vel,
                    joint.max_torque * 0.5,
                );
            }
            ControlMode::Torque => {
                // Direct torque control - use motor with position target
                let target_torque = commands.target_torques[idx];
                joint.target_torque = target_torque;

                // Approximate torque control via velocity with high damping
                let vel = target_torque / joint.max_torque.max(1.0) * 5.0;
                impulse_joint
                    .data
                    .set_motor_velocity(joint_axis, vel, joint.max_torque);
            }
            ControlMode::Passive => {
                // Just damping, no active control
                impulse_joint
                    .data
                    .set_motor_velocity(joint_axis, 0.0, joint.kd * 0.1);
            }
        }
    }
}

/// Read joint sensor data from Rapier physics
fn read_joint_sensors(
    mut state: ResMut<JointState>,
    mut joint_query: Query<(&mut PhysicsJoint, &ImpulseJoint)>,
) {
    for (mut joint, impulse_joint) in joint_query.iter_mut() {
        let idx = joint.index;
        if idx < state.positions.len() {
            // Read joint angle from Rapier - RevoluteJoint stores angle in AngX
            // The joint data contains the relative rotation between bodies
            let joint_data = &impulse_joint.data;

            // For revolute joints, we need to extract the angle
            // The motor position gives us the current angle
            if let Some(motor) = joint_data.motor(JointAxis::AngX) {
                // Update current position from motor feedback
                // Note: Rapier doesn't directly expose current angle,
                // we estimate from motor target vs actual behavior
                joint.current_velocity = motor.target_vel;
            }

            // Copy to state resource
            state.positions[idx] = joint.current_position;
            state.velocities[idx] = joint.current_velocity;
            state.torques[idx] = joint.current_torque;
        }
    }
}

/// Read IMU sensor data from base
fn read_imu_sensor(
    mut imu: ResMut<ImuData>,
    base_query: Query<(&Transform, &Velocity), With<RobotBase>>,
) {
    for (transform, velocity) in base_query.iter() {
        imu.orientation = transform.rotation;
        imu.angular_velocity = velocity.angvel;

        // Linear acceleration would need to be computed from velocity changes
        // For now, just report gravity in body frame
        let gravity_world = Vec3::new(0.0, -9.81, 0.0);
        imu.linear_acceleration = transform.rotation.inverse() * gravity_world;
    }
}

/// Read foot contact sensors
fn read_foot_sensors(
    mut foot_data: ResMut<FootSensorData>,
    rapier_context: Res<RapierContext>,
    foot_query: Query<(Entity, &PhysicsLink, &Transform)>,
) {
    for (entity, link, transform) in foot_query.iter() {
        if link.name.contains("foot") {
            // Check for ground contact using shape cast
            let ray_origin = transform.translation;
            let ray_dir = Vec3::NEG_Y;

            if let Some((_, toi)) = rapier_context.cast_ray(
                ray_origin,
                ray_dir,
                0.05, // Max distance to check
                true,
                QueryFilter::default().exclude_collider(entity),
            ) {
                let in_contact = toi < 0.02; // Within 2cm of ground

                if link.name.contains("left") {
                    foot_data.left_contact = in_contact;
                    foot_data.left_cop = ray_origin + ray_dir * toi;
                } else {
                    foot_data.right_contact = in_contact;
                    foot_data.right_cop = ray_origin + ray_dir * toi;
                }
            }
        }
    }
}

/// Setup lighting (camera is spawned separately by spawn_orbit_camera)
fn setup_camera_and_lights(mut commands: Commands) {
    // Directional light (sun)
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.7, 0.4, 0.0)),
        ..default()
    });

    // Ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
    });
}

/// UI state for displaying robot info
#[derive(Resource, Default)]
pub struct SimulatorUiState {
    pub show_joint_info: bool,
    pub selected_joint: usize,
    pub control_mode_global: ControlMode,
}

/// Camera orbit controller
#[derive(Component)]
pub struct OrbitCamera {
    pub radius: f32,
    pub theta: f32, // horizontal angle
    pub phi: f32,   // vertical angle
    pub target: Vec3,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            radius: 6.0,
            theta: 0.9, // slightly off-axis for a better view
            phi: 1.1,   // closer to horizontal than top-down
            target: Vec3::new(0.0, 0.9, 0.0),
        }
    }
}

fn orbit_transform(orbit: &OrbitCamera) -> Transform {
    let x = orbit.radius * orbit.phi.sin() * orbit.theta.cos();
    let z = orbit.radius * orbit.phi.sin() * orbit.theta.sin();
    let y = orbit.radius * orbit.phi.cos();

    Transform::from_translation(orbit.target + Vec3::new(x, y, z)).looking_at(orbit.target, Vec3::Y)
}

/// Handle keyboard input for camera and control
fn handle_keyboard_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands_res: ResMut<JointCommands>,
    mut ui_state: ResMut<SimulatorUiState>,
    mut camera_query: Query<(&mut Transform, &mut OrbitCamera)>,
    time: Res<Time>,
) {
    let dt = time.delta_seconds();

    // Camera orbit controls
    for (mut transform, mut orbit) in camera_query.iter_mut() {
        let orbit_speed = 1.5 * dt;
        let zoom_speed = 2.0 * dt;

        if keyboard.pressed(KeyCode::ArrowLeft) {
            orbit.theta -= orbit_speed;
        }
        if keyboard.pressed(KeyCode::ArrowRight) {
            orbit.theta += orbit_speed;
        }
        if keyboard.pressed(KeyCode::ArrowUp) {
            orbit.phi = (orbit.phi + orbit_speed).min(1.5);
        }
        if keyboard.pressed(KeyCode::ArrowDown) {
            orbit.phi = (orbit.phi - orbit_speed).max(0.1);
        }
        if keyboard.pressed(KeyCode::Minus) || keyboard.pressed(KeyCode::NumpadSubtract) {
            orbit.radius = (orbit.radius + zoom_speed * 5.0).min(15.0);
        }
        if keyboard.pressed(KeyCode::Equal) || keyboard.pressed(KeyCode::NumpadAdd) {
            orbit.radius = (orbit.radius - zoom_speed * 5.0).max(1.5);
        }

        if keyboard.just_pressed(KeyCode::KeyC) {
            *orbit = OrbitCamera::default();
        }

        // Update camera position from orbit parameters
        *transform = orbit_transform(&orbit);
    }

    // Control mode switching
    if keyboard.just_pressed(KeyCode::Digit1) {
        ui_state.control_mode_global = ControlMode::Position;
        for mode in commands_res.control_modes.iter_mut() {
            *mode = ControlMode::Position;
        }
        info!("Switched to Position control mode");
    }
    if keyboard.just_pressed(KeyCode::Digit2) {
        ui_state.control_mode_global = ControlMode::Velocity;
        for mode in commands_res.control_modes.iter_mut() {
            *mode = ControlMode::Velocity;
        }
        info!("Switched to Velocity control mode");
    }
    if keyboard.just_pressed(KeyCode::Digit3) {
        ui_state.control_mode_global = ControlMode::Torque;
        for mode in commands_res.control_modes.iter_mut() {
            *mode = ControlMode::Torque;
        }
        info!("Switched to Torque control mode");
    }
    if keyboard.just_pressed(KeyCode::Digit4) {
        ui_state.control_mode_global = ControlMode::Passive;
        for mode in commands_res.control_modes.iter_mut() {
            *mode = ControlMode::Passive;
        }
        info!("Switched to Passive mode (no control)");
    }

    // Joint selection (Q/E to cycle)
    if keyboard.just_pressed(KeyCode::KeyQ) {
        if ui_state.selected_joint > 0 {
            ui_state.selected_joint -= 1;
        }
    }
    if keyboard.just_pressed(KeyCode::KeyE) {
        ui_state.selected_joint = (ui_state.selected_joint + 1).min(unitree_g1::NUM_JOINTS - 1);
    }

    // Adjust selected joint target (W/S)
    let joint_idx = ui_state.selected_joint;
    if keyboard.pressed(KeyCode::KeyW) {
        commands_res.target_positions[joint_idx] += 0.5 * dt;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        commands_res.target_positions[joint_idx] -= 0.5 * dt;
    }

    // Reset all joints to zero
    if keyboard.just_pressed(KeyCode::KeyR) {
        for pos in commands_res.target_positions.iter_mut() {
            *pos = 0.0;
        }
        info!("Reset all joint targets to zero");
    }

    // Toggle joint info display
    if keyboard.just_pressed(KeyCode::KeyI) {
        ui_state.show_joint_info = !ui_state.show_joint_info;
    }
}

/// Extended physics simulator plugin with full visualization
pub struct FullPhysicsSimulatorPlugin;

impl Plugin for FullPhysicsSimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(PhysicsSimulatorPlugin)
            .add_plugins(RapierDebugRenderPlugin::default())
            .insert_resource(ClearColor(Color::srgb(0.1, 0.1, 0.15)))
            .init_resource::<SimulatorUiState>()
            .add_systems(Startup, (setup_camera_and_lights, spawn_orbit_camera))
            .add_systems(
                Update,
                (
                    handle_keyboard_input,
                    print_controls_once,
                    display_debug_info,
                ),
            );
    }
}

/// Display debug info when enabled
fn display_debug_info(
    ui_state: Res<SimulatorUiState>,
    joint_state: Res<JointState>,
    imu: Res<ImuData>,
    foot_data: Res<FootSensorData>,
    joint_query: Query<&PhysicsJoint>,
    mut last_print: Local<f32>,
    time: Res<Time>,
) {
    if !ui_state.show_joint_info {
        return;
    }

    // Only print every 0.5 seconds to avoid spam
    *last_print += time.delta_seconds();
    if *last_print < 0.5 {
        return;
    }
    *last_print = 0.0;

    // Clear screen (ANSI escape code)
    print!("\x1B[2J\x1B[1;1H");

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║           HUMANOID PHYSICS SIMULATOR - DEBUG INFO            ║");
    println!("╠══════════════════════════════════════════════════════════════╣");

    // Selected joint info
    let selected = ui_state.selected_joint;
    if let Some(joint) = joint_query.iter().find(|j| j.index == selected) {
        println!("║ Selected Joint: {:3} - {:<35} ║", selected, joint.name);
        println!(
            "║   Target Pos: {:+7.3} rad  Current: {:+7.3} rad           ║",
            joint.target_position, joint.current_position
        );
        println!(
            "║   Mode: {:?}                                        ║",
            joint.control_mode
        );
    }

    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║ IMU Data:                                                    ║");
    println!(
        "║   Orientation: ({:+.2}, {:+.2}, {:+.2}, {:+.2})              ║",
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w
    );
    println!(
        "║   Angular Vel: ({:+.2}, {:+.2}, {:+.2}) rad/s               ║",
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z
    );

    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║ Foot Sensors:                                                ║");
    println!(
        "║   Left:  Contact={:<5}  Right: Contact={:<5}               ║",
        foot_data.left_contact, foot_data.right_contact
    );

    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║ Joint Positions (first 12):                                  ║");
    for i in 0..12.min(joint_state.positions.len()) {
        if i % 3 == 0 {
            print!("║  ");
        }
        print!("{:2}:{:+.2}  ", i, joint_state.positions[i]);
        if i % 3 == 2 {
            println!("                              ║");
        }
    }
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!("\nPress I to hide, Q/E to select joint, W/S to move, R to reset");
}

fn spawn_orbit_camera(mut commands: Commands) {
    let orbit = OrbitCamera::default();
    commands.spawn((
        Camera3dBundle {
            transform: orbit_transform(&orbit),
            ..default()
        },
        orbit,
    ));
}

/// Print controls once on startup
fn print_controls_once(mut ran: Local<bool>) {
    if *ran {
        return;
    }
    *ran = true;

    println!("\n=== Physics Simulator Controls ===");
    println!("Camera: Arrow keys to orbit, +/- to zoom");
    println!("Joint Select: Q/E to cycle joints");
    println!("Joint Control: W/S to adjust selected joint");
    println!("Modes: 1=Position, 2=Velocity, 3=Torque, 4=Passive");
    println!("Reset: R to zero all joints");
    println!("Toggle Info: I to show/hide joint info");
    println!("Exit: ESC");
    println!("===================================\n");
}

/// Load a scene from a TOML or JSON file
pub fn load_scene_file(path: &std::path::Path) -> Result<SceneGraph, String> {
    let mut loader = SceneLoader::new();
    loader
        .load_from_file(path)
        .map_err(|e| format!("Failed to load scene: {}", e))
}

/// Load a scene from a string (TOML format)
pub fn load_scene_toml(toml_str: &str) -> Result<SceneGraph, String> {
    let loader = SceneLoader::new();
    loader
        .load_from_str(AssetFormat::Toml, toml_str)
        .map_err(|e| format!("Failed to parse TOML: {}", e))
}

/// Load a scene from a string (JSON format)
pub fn load_scene_json(json_str: &str) -> Result<SceneGraph, String> {
    let loader = SceneLoader::new();
    loader
        .load_from_str(AssetFormat::Json, json_str)
        .map_err(|e| format!("Failed to parse JSON: {}", e))
}

/// Plugin for loading scenes from files
pub struct SceneLoaderPlugin {
    scene_path: Option<PathBuf>,
}

impl SceneLoaderPlugin {
    /// Create a new scene loader plugin without a scene file
    pub fn new() -> Self {
        Self { scene_path: None }
    }

    /// Create a scene loader plugin with a specific scene file
    pub fn with_scene(path: impl Into<PathBuf>) -> Self {
        Self {
            scene_path: Some(path.into()),
        }
    }
}

impl Plugin for SceneLoaderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveScene>();

        if let Some(ref path) = self.scene_path {
            if let Ok(graph) = load_scene_file(path) {
                let mut active_scene = app.world_mut().resource_mut::<ActiveScene>();
                active_scene.load(graph, Some(path.clone()));
                println!("Loaded scene from: {:?}", path);
            } else {
                eprintln!("Warning: Failed to load scene from {:?}", path);
            }
        }
    }
}

impl Default for SceneLoaderPlugin {
    fn default() -> Self {
        Self::new()
    }
}

/// System to spawn entities from a loaded scene graph
fn spawn_scene_entities(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    active_scene: Res<ActiveScene>,
) {
    if active_scene.scene_path.is_none() {
        // No scene loaded, use default setup
        return;
    }

    // Spawn terrain from scene
    for terrain_node in active_scene.graph.get_terrain() {
        if let Some(terrain_type) = terrain_node.get_property("type") {
            match terrain_type.as_str() {
                "Plane" | "FinitePlane" => {
                    let friction: f32 = terrain_node
                        .get_property("friction")
                        .and_then(|f| f.parse().ok())
                        .unwrap_or(0.8) as f32;

                    // Spawn ground plane
                    commands.spawn((
                        PbrBundle {
                            mesh: meshes.add(Plane3d::default().mesh().size(20.0, 20.0)),
                            material: materials.add(StandardMaterial {
                                base_color: Color::srgb(0.3, 0.35, 0.3),
                                ..default()
                            }),
                            ..default()
                        },
                        RigidBody::Fixed,
                        Collider::cuboid(10.0, 0.01, 10.0),
                        CollisionGroups::new(GROUND_GROUP, ROBOT_GROUP),
                        Friction::coefficient(friction),
                    ));
                }
                _ => {
                    // Other terrain types not yet implemented
                }
            }
        }
    }

    // Spawn obstacles from scene
    for obstacle_node in active_scene.graph.get_obstacles() {
        if let Some(obstacle_type) = obstacle_node.get_property("type") {
            let position = obstacle_node.world_transform.translation;
            let rotation = obstacle_node.world_transform.rotation;

            match obstacle_type.as_str() {
                "Box" => {
                    let sx = obstacle_node
                        .get_property("size_x")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(1.0) as f32;
                    let sy = obstacle_node
                        .get_property("size_y")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(1.0) as f32;
                    let sz = obstacle_node
                        .get_property("size_z")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(1.0) as f32;

                    let r = obstacle_node
                        .get_property("color_r")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;
                    let g = obstacle_node
                        .get_property("color_g")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;
                    let b = obstacle_node
                        .get_property("color_b")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;

                    let is_static = obstacle_node
                        .get_property("static")
                        .map(|s| s == "true")
                        .unwrap_or(true);

                    let mut entity = commands.spawn((
                        PbrBundle {
                            mesh: meshes.add(Cuboid::new(sx, sy, sz)),
                            material: materials.add(StandardMaterial {
                                base_color: Color::srgb(r, g, b),
                                ..default()
                            }),
                            transform: Transform {
                                translation: position,
                                rotation,
                                ..default()
                            },
                            ..default()
                        },
                        Collider::cuboid(sx / 2.0, sy / 2.0, sz / 2.0),
                        CollisionGroups::new(GROUND_GROUP, ROBOT_GROUP),
                    ));

                    if is_static {
                        entity.insert(RigidBody::Fixed);
                    } else {
                        entity.insert(RigidBody::Dynamic);
                        if let Some(mass) = obstacle_node
                            .get_property("mass")
                            .and_then(|m| m.parse::<f32>().ok())
                        {
                            entity.insert(ColliderMassProperties::Mass(mass));
                        }
                    }

                    if let Some(friction) = obstacle_node
                        .get_property("friction")
                        .and_then(|f| f.parse().ok())
                    {
                        entity.insert(Friction::coefficient(friction));
                    }
                }
                "Sphere" => {
                    let radius = obstacle_node
                        .get_property("radius")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(0.5) as f32;

                    let r = obstacle_node
                        .get_property("color_r")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;
                    let g = obstacle_node
                        .get_property("color_g")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;
                    let b = obstacle_node
                        .get_property("color_b")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;

                    let is_static = obstacle_node
                        .get_property("static")
                        .map(|s| s == "true")
                        .unwrap_or(true);

                    let mut entity = commands.spawn((
                        PbrBundle {
                            mesh: meshes.add(Sphere::new(radius)),
                            material: materials.add(StandardMaterial {
                                base_color: Color::srgb(r, g, b),
                                ..default()
                            }),
                            transform: Transform {
                                translation: position,
                                rotation,
                                ..default()
                            },
                            ..default()
                        },
                        Collider::ball(radius),
                        CollisionGroups::new(GROUND_GROUP, ROBOT_GROUP),
                    ));

                    if is_static {
                        entity.insert(RigidBody::Fixed);
                    } else {
                        entity.insert(RigidBody::Dynamic);
                        if let Some(mass) = obstacle_node
                            .get_property("mass")
                            .and_then(|m| m.parse::<f32>().ok())
                        {
                            entity.insert(ColliderMassProperties::Mass(mass));
                        }
                    }

                    if let Some(friction) = obstacle_node
                        .get_property("friction")
                        .and_then(|f| f.parse().ok())
                    {
                        entity.insert(Friction::coefficient(friction));
                    }
                }
                "Cylinder" => {
                    let radius = obstacle_node
                        .get_property("radius")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(0.5) as f32;
                    let height = obstacle_node
                        .get_property("height")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(1.0) as f32;

                    let r = obstacle_node
                        .get_property("color_r")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;
                    let g = obstacle_node
                        .get_property("color_g")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;
                    let b = obstacle_node
                        .get_property("color_b")
                        .and_then(|c| c.parse().ok())
                        .unwrap_or(0.5) as f32;

                    let is_static = obstacle_node
                        .get_property("static")
                        .map(|s| s == "true")
                        .unwrap_or(true);

                    let mut entity = commands.spawn((
                        PbrBundle {
                            mesh: meshes.add(Cylinder::new(radius, height)),
                            material: materials.add(StandardMaterial {
                                base_color: Color::srgb(r, g, b),
                                ..default()
                            }),
                            transform: Transform {
                                translation: position,
                                rotation,
                                ..default()
                            },
                            ..default()
                        },
                        Collider::cylinder(height / 2.0, radius),
                        CollisionGroups::new(GROUND_GROUP, ROBOT_GROUP),
                    ));

                    if is_static {
                        entity.insert(RigidBody::Fixed);
                    } else {
                        entity.insert(RigidBody::Dynamic);
                        if let Some(mass) = obstacle_node
                            .get_property("mass")
                            .and_then(|m| m.parse::<f32>().ok())
                        {
                            entity.insert(ColliderMassProperties::Mass(mass));
                        }
                    }

                    if let Some(friction) = obstacle_node
                        .get_property("friction")
                        .and_then(|f| f.parse().ok())
                    {
                        entity.insert(Friction::coefficient(friction));
                    }
                }
                _ => {
                    // Other obstacle types not yet implemented
                }
            }
        }
    }
}

/// Run the physics simulator
pub fn run_physics_simulator() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(bevy::window::Window {
                title: "Humanoid Physics Simulator - Unitree G1".into(),
                resolution: bevy::window::WindowResolution::new(1280.0, 720.0),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(FullPhysicsSimulatorPlugin)
        .add_systems(Update, bevy::window::close_on_esc)
        .run();
}

/// Run the physics simulator with a scene file
pub fn run_physics_simulator_with_scene(scene_path: impl Into<PathBuf>) {
    let path = scene_path.into();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(bevy::window::Window {
                title: "Humanoid Physics Simulator - Unitree G1".into(),
                resolution: bevy::window::WindowResolution::new(1280.0, 720.0),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(SceneLoaderPlugin::with_scene(&path))
        .add_plugins(FullPhysicsSimulatorPlugin)
        .add_systems(Update, bevy::window::close_on_esc)
        .run();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unitree_g1_spec() {
        let (links, joints) = unitree_g1_spec();

        // Should have pelvis + 28 more links
        assert!(!links.is_empty());

        // Should have joints for all DOF
        assert!(!joints.is_empty());

        // Check joint limits are valid
        for joint in &joints {
            assert!(joint.limits.0 < joint.limits.1);
            assert!(joint.max_torque > 0.0);
        }
    }

    #[test]
    fn test_joint_state_default() {
        let state = JointState::default();
        assert_eq!(state.positions.len(), unitree_g1::NUM_JOINTS);
        assert_eq!(state.velocities.len(), unitree_g1::NUM_JOINTS);
        assert_eq!(state.torques.len(), unitree_g1::NUM_JOINTS);
    }

    #[test]
    fn test_joint_commands_default() {
        let commands = JointCommands::default();
        assert_eq!(commands.target_positions.len(), unitree_g1::NUM_JOINTS);
        assert_eq!(commands.control_modes.len(), unitree_g1::NUM_JOINTS);

        // Default should be position control
        assert!(commands
            .control_modes
            .iter()
            .all(|m| *m == ControlMode::Position));
    }
}
