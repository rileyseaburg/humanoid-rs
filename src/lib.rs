//! Humanoid Robotics Framework in Rust
//!
//! A memory-safe, real-time framework for controlling humanoid robots.
//! Designed for Unitree G1/H1 but extensible to other platforms.

pub mod control;
pub mod ik;
pub mod logging;
pub mod mpc;
pub mod robot;
pub mod sdk;
pub mod sim;
pub mod whole_body;

#[cfg(feature = "simulator")]
pub mod simulator;

#[cfg(feature = "simulator")]
pub mod physics_sim;

#[cfg(feature = "unitree")]
pub mod unitree;

#[cfg(feature = "ros2")]
pub mod ros2;

#[cfg(feature = "rl")]
pub mod rl;

#[cfg(feature = "mcp")]
pub mod mcp;

pub use control::safety::{
    CommunicationMonitor, ControlMode, EStopGuard, JointLimitGuard, SafetyLevel, SafetyManager,
    TorqueRampGuard,
};
pub use control::{
    CommandBuffer, ControlCommand, ControlState, Controller, ControllerError, MotionPlanner,
    Trajectory,
};
pub use ik::{IkSolver, IkSolverConfig, PositionTarget, WholeBodyIk};
pub use logging::{ExportFormat, SimulationLogger};
pub use mpc::{FootstepPlanner, LipmState, MpcController};
pub use robot::{HumanoidRobot, Joint, JointSet, Pose};
pub use sim::observability::{DebugHooks, ObservabilitySystem, SimulationControl};
pub use whole_body::{BalanceRecovery, ComState, WholeBodyController};

// Re-export SDK types at crate root for convenience
pub use sdk::robot::{RobotCapabilities, RobotProfile};
pub use sdk::{HumanoidSDK, RobotConfig, SDKConfig, SimulationConfig};

// Re-export physics backend types at crate root
pub use sim::physics_backend::{
    BodyHandle, BodyType, Contact, JointHandle, JointSpec, JointType, PhysicsBackend, PhysicsError,
    PhysicsResult, RigidBodySpec, Shape, Velocity as PhysicsVelocity,
};

// Re-export sensor framework types at crate root
pub use sim::sensors::{
    CameraConfig, CameraData, CameraSensor, ContactState, ForceTorqueConfig, ForceTorqueData,
    ForceTorqueSensor, ImuConfig, ImuData, ImuSensor, JointEncoderConfig, JointEncoderData,
    JointEncoderSensor, JointState as PhysicsJointState, NoiseModel, Sensor, SensorData,
    SensorRegistry, SensorSync, SensorType, SingleJointData, Timestamped, Transform3D,
    Velocity as SensorVelocity,
};

#[cfg(feature = "simulator")]
pub use sim::RapierBackend;

#[cfg(feature = "simulator")]
pub use sim::deterministic::{
    DeterministicScheduler, SimulationClock, SimulationControl, Subsystem, TickConfig, TimestepMode,
};

// Re-export scene and asset types at crate root (simulator only)
#[cfg(feature = "simulator")]
pub use sim::assets::config::{
    ControlModeConfig, DirectionalLightConfig, LightingConfig, ObstacleConfig, ObstacleType,
    PointLightConfig, RobotConfig, SimulationConfig, TerrainConfig, TerrainType,
};
#[cfg(feature = "simulator")]
pub use sim::assets::{AssetError, AssetFormat, SceneConfig, SceneLoader};
#[cfg(feature = "simulator")]
pub use sim::scene::{ActiveScene, NodeType, SceneError, SceneGraph, SceneMember, SceneNode};

use thiserror::Error;

/// Errors that can occur in the humanoid framework
#[derive(Error, Debug)]
pub enum HumanoidError {
    #[error("Joint limit exceeded: {joint_name} requested {requested}, limits [{min}, {max}]")]
    JointLimitExceeded {
        joint_name: String,
        requested: f64,
        min: f64,
        max: f64,
    },

    #[error("Communication error: {0}")]
    CommunicationError(String),

    #[error("Simulator error: {0}")]
    SimulatorError(String),

    #[error("Control error: {0}")]
    ControlError(String),

    #[error("Safety stop triggered: {reason}")]
    SafetyStop { reason: String },
}

pub type Result<T> = std::result::Result<T, HumanoidError>;

/// Version of the framework
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_framework_loads() {
        // Version should match Cargo.toml (0.2.0)
        assert!(!VERSION.is_empty());
        assert!(VERSION.contains('.'));
        assert_eq!(VERSION, "0.2.0");
    }
}
