//! Humanoid Robotics Framework in Rust
//! 
//! A memory-safe, real-time framework for controlling humanoid robots.
//! Designed for Unitree G1/H1 but extensible to other platforms.

pub mod robot;
pub mod control;
pub mod ik;
pub mod whole_body;
pub mod mpc;

#[cfg(feature = "simulator")]
pub mod simulator;

#[cfg(feature = "unitree")]
pub mod unitree;

#[cfg(feature = "ros2")]
pub mod ros2;

#[cfg(feature = "rl")]
pub mod rl;

#[cfg(feature = "mcp")]
pub mod mcp;

pub use robot::{HumanoidRobot, Joint, JointSet, Pose};
pub use control::{Controller, Trajectory, MotionPlanner};
pub use ik::{IkSolver, WholeBodyIk, PositionTarget, IkSolverConfig};
pub use whole_body::{WholeBodyController, BalanceRecovery, ComState};
pub use mpc::{MpcController, FootstepPlanner, LipmState};

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
        assert_eq!(VERSION, "0.1.0");
    }
}
