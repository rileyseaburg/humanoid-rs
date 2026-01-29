//! Humanoid SDK - Public API Layer
//!
//! This module provides a clean, high-level interface for controlling
//! humanoid robots with support for both simulation and hardware.

use std::path::Path;
use tracing::{debug, info};

pub mod config;
pub mod robot;

use crate::robot::HumanoidRobot;
pub use config::{
    ControlConfig, LoggingConfig, PhysicsConfig, RobotConfig, SDKConfig, SimulationConfig,
};
pub use robot::{RobotCapabilities, RobotProfile};

/// Re-export commonly used types for convenience
pub mod prelude {
    pub use crate::sdk::config::SDKConfig;
    pub use crate::sdk::robot::{RobotCapabilities, RobotProfile};
    pub use crate::sdk::{HumanoidSDK, RobotConfig, SimulationConfig};
}

/// Handle to a running simulation instance
#[derive(Debug, Clone)]
pub struct SimulationHandle {
    pub id: u64,
    pub is_running: bool,
}

/// Handle to a connected robot
#[derive(Debug, Clone)]
pub struct RobotHandle {
    pub id: u64,
    pub profile: RobotProfile,
    pub is_connected: bool,
}

impl RobotHandle {
    /// Command the robot to stand up
    pub fn stand_up(&self) -> crate::Result<()> {
        if !self.is_connected {
            return Err(crate::HumanoidError::CommunicationError(
                "Robot not connected".to_string(),
            ));
        }
        info!("Robot {}: Standing up", self.id);
        Ok(())
    }

    /// Command the robot to sit down
    pub fn sit_down(&self) -> crate::Result<()> {
        if !self.is_connected {
            return Err(crate::HumanoidError::CommunicationError(
                "Robot not connected".to_string(),
            ));
        }
        info!("Robot {}: Sitting down", self.id);
        Ok(())
    }
}

/// Main SDK interface for controlling humanoid robots
///
/// This struct provides a unified interface for both simulated and
/// real robot control. It manages the lifecycle of connections and
/// ensures proper cleanup on shutdown.
///
/// # Example
///
/// ```rust
/// use humanoid_rs::sdk::prelude::*;
///
/// let config = SDKConfig::default();
/// let mut sdk = HumanoidSDK::with_config(config);
/// ```
pub struct HumanoidSDK {
    simulation: Option<SimulationHandle>,
    robot: Option<Box<dyn HumanoidRobot>>,
    config: SDKConfig,
    next_id: u64,
}

impl HumanoidSDK {
    /// Create a new SDK instance with default configuration
    ///
    /// # Example
    ///
    /// ```rust
    /// use humanoid_rs::sdk::HumanoidSDK;
    ///
    /// let sdk = HumanoidSDK::new();
    /// ```
    pub fn new() -> Self {
        Self {
            simulation: None,
            robot: None,
            config: SDKConfig::default(),
            next_id: 1,
        }
    }

    /// Create a new SDK instance with custom configuration
    ///
    /// # Example
    ///
    /// ```rust
    /// use humanoid_rs::sdk::{HumanoidSDK, SDKConfig};
    ///
    /// let config = SDKConfig::default();
    /// let sdk = HumanoidSDK::with_config(config);
    /// ```
    pub fn with_config(config: SDKConfig) -> Self {
        info!("Initializing HumanoidSDK with custom config");
        Self {
            simulation: None,
            robot: None,
            config,
            next_id: 1,
        }
    }

    /// Connect to a simulator
    ///
    /// # Arguments
    ///
    /// * `config` - Simulation configuration including timestep, rendering options, etc.
    ///
    /// # Returns
    ///
    /// A handle to the running simulation
    ///
    /// # Errors
    ///
    /// Returns an error if the simulation fails to start
    pub fn connect_simulator(
        &mut self,
        config: SimulationConfig,
    ) -> crate::Result<SimulationHandle> {
        debug!("Connecting to simulator with config: {:?}", config);

        config
            .validate()
            .map_err(|e| crate::HumanoidError::SimulatorError(e.to_string()))?;

        let handle = SimulationHandle {
            id: self.next_id,
            is_running: true,
        };
        self.next_id += 1;
        self.simulation = Some(handle.clone());

        info!("Connected to simulator with handle {}", handle.id);
        Ok(handle)
    }

    /// Connect to a physical robot
    ///
    /// # Arguments
    ///
    /// * `profile` - Robot profile describing the robot's capabilities
    ///
    /// # Returns
    ///
    /// A handle to the connected robot
    ///
    /// # Errors
    ///
    /// Returns an error if connection fails or profile validation fails
    pub fn connect_robot(&mut self, profile: RobotProfile) -> crate::Result<RobotHandle> {
        debug!("Connecting to robot with profile: {}", profile.name);

        profile
            .validate()
            .map_err(|e| crate::HumanoidError::CommunicationError(e.to_string()))?;

        let handle = RobotHandle {
            id: self.next_id,
            profile,
            is_connected: true,
        };
        self.next_id += 1;

        info!("Connected to robot with handle {}", handle.id);
        Ok(handle)
    }

    /// Load configuration from a file
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the configuration file (TOML, JSON, or YAML)
    ///
    /// # Returns
    ///
    /// A new SDK instance with the loaded configuration
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be read or parsed
    pub fn from_file(path: &Path) -> crate::Result<Self> {
        let config = SDKConfig::from_file(path)?;
        Ok(Self::with_config(config))
    }

    /// Get the current configuration
    pub fn config(&self) -> &SDKConfig {
        &self.config
    }

    /// Get mutable access to configuration
    pub fn config_mut(&mut self) -> &mut SDKConfig {
        &mut self.config
    }

    /// Shutdown the SDK and cleanup resources
    ///
    /// This should be called when finished to ensure proper cleanup
    /// of robot connections and simulation instances.
    pub fn shutdown(self) {
        info!("Shutting down HumanoidSDK");

        if let Some(ref _robot) = self.robot {
            debug!("Disconnecting robot");
            // Robot cleanup happens via Drop
        }

        if let Some(ref sim) = self.simulation {
            debug!("Stopping simulation {}", sim.id);
            // Simulation cleanup happens via Drop
        }

        info!("HumanoidSDK shutdown complete");
    }

    /// Check if connected to a simulation
    pub fn has_simulation(&self) -> bool {
        self.simulation.is_some()
    }

    /// Check if connected to a robot
    pub fn has_robot(&self) -> bool {
        self.robot.is_some()
    }
}

impl Default for HumanoidSDK {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_sdk_new() {
        let sdk = HumanoidSDK::new();
        assert!(!sdk.has_simulation());
        assert!(!sdk.has_robot());
    }

    #[test]
    fn test_sdk_default() {
        let sdk: HumanoidSDK = Default::default();
        assert!(!sdk.has_simulation());
        assert!(!sdk.has_robot());
    }

    #[test]
    fn test_sdk_with_config() {
        let config = SDKConfig::default();
        let sdk = HumanoidSDK::with_config(config);
        assert!(!sdk.has_simulation());
        assert!(!sdk.has_robot());
    }

    #[test]
    fn test_connect_simulator() {
        let mut sdk = HumanoidSDK::new();
        let sim_config = SimulationConfig {
            timestep: Duration::from_millis(2),
            real_time_factor: 1.0,
            headless: true,
            scene_path: None,
        };

        let handle = sdk.connect_simulator(sim_config).unwrap();
        assert_eq!(handle.id, 1);
        assert!(handle.is_running);
        assert!(sdk.has_simulation());
    }

    #[test]
    fn test_connect_robot() {
        let mut sdk = HumanoidSDK::new();
        let profile = RobotProfile::unitree_g1();

        let handle = sdk.connect_robot(profile.clone()).unwrap();
        assert_eq!(handle.id, 1);
        assert!(handle.is_connected);
        assert!(!sdk.has_robot()); // Robot stored separately from handle
    }

    #[test]
    fn test_simulation_config_validation() {
        let mut sdk = HumanoidSDK::new();
        let invalid_config = SimulationConfig {
            timestep: Duration::from_millis(20), // > 0.01s, should fail
            real_time_factor: 1.0,
            headless: true,
            scene_path: None,
        };

        assert!(sdk.connect_simulator(invalid_config).is_err());
    }
}
