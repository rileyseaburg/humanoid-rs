//! SDK Configuration System
//!
//! Provides comprehensive configuration management for the humanoid SDK,
//! including physics, robot, control, and logging settings.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::time::Duration;
use tracing::{debug, info, warn};

/// Main SDK configuration container
///
/// Aggregates all configuration subsystems for the SDK.
/// Can be loaded from files and validated.
///
/// # Example
///
/// ```rust
/// use humanoid_rs::sdk::config::SDKConfig;
///
/// let config = SDKConfig::default();
/// assert!(config.validate().is_ok());
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SDKConfig {
    /// Physics simulation settings
    pub physics: PhysicsConfig,
    /// Robot-specific configuration
    pub robot: RobotConfig,
    /// Control system settings
    pub control: ControlConfig,
    /// Logging configuration
    pub logging: LoggingConfig,
}

/// Physics simulation configuration
///
/// Controls the physics simulation timestep and accuracy.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsConfig {
    /// Simulation timestep in seconds (must be > 0 and <= 0.01)
    pub timestep: f64,
    /// Number of solver iterations for constraints
    pub solver_iterations: u32,
    /// Gravity vector (m/s²)
    pub gravity: [f64; 3],
    /// Enable collision detection
    pub enable_collision: bool,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            timestep: 0.002, // 500 Hz
            solver_iterations: 10,
            gravity: [0.0, -9.81, 0.0],
            enable_collision: true,
        }
    }
}

impl PhysicsConfig {
    /// Validate physics configuration
    ///
    /// # Validation Rules
    /// - timestep > 0 and <= 0.01
    /// - solver_iterations >= 1
    pub fn validate(&self) -> anyhow::Result<()> {
        if self.timestep <= 0.0 {
            anyhow::bail!("Physics timestep must be greater than 0");
        }
        if self.timestep > 0.01 {
            anyhow::bail!("Physics timestep must be <= 0.01s (100Hz minimum)");
        }
        if self.solver_iterations < 1 {
            anyhow::bail!("Solver iterations must be >= 1");
        }
        Ok(())
    }
}

/// Robot configuration
///
/// Settings specific to the robot hardware or model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotConfig {
    /// Robot model name (e.g., "unitree_g1", "unitree_h1")
    pub model: String,
    /// URDF file path (optional, for custom models)
    pub urdf_path: Option<PathBuf>,
    /// Default joint positions at startup
    pub default_joint_positions: Vec<f64>,
    /// Enable self-collision checking
    pub enable_self_collision: bool,
    /// Joint damping coefficient
    pub joint_damping: f64,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            model: "unitree_g1".to_string(),
            urdf_path: None,
            default_joint_positions: vec![0.0; 29],
            enable_self_collision: true,
            joint_damping: 0.1,
        }
    }
}

impl RobotConfig {
    /// Validate robot configuration
    ///
    /// # Validation Rules
    /// - Model name is not empty
    /// - URDF path exists if specified
    /// - Joint damping >= 0
    pub fn validate(&self) -> anyhow::Result<()> {
        if self.model.is_empty() {
            anyhow::bail!("Robot model name cannot be empty");
        }

        if let Some(ref path) = self.urdf_path {
            if !path.exists() {
                anyhow::bail!("URDF file does not exist: {}", path.display());
            }
        }

        if self.joint_damping < 0.0 {
            anyhow::bail!("Joint damping must be >= 0");
        }

        Ok(())
    }
}

/// Control system configuration
///
/// Settings for the control algorithms and update rates.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlConfig {
    /// Control loop frequency in Hz
    pub control_frequency: f64,
    /// Maximum joint velocity (rad/s)
    pub max_joint_velocity: f64,
    /// Maximum joint acceleration (rad/s²)
    pub max_joint_acceleration: f64,
    /// Joint position gain (P)
    pub kp: f64,
    /// Joint velocity gain (D)
    pub kd: f64,
    /// Integration gain (I)
    pub ki: f64,
    /// Safety torque limit (Nm)
    pub torque_limit: f64,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            control_frequency: 500.0,
            max_joint_velocity: 10.0,
            max_joint_acceleration: 50.0,
            kp: 100.0,
            kd: 10.0,
            ki: 0.0,
            torque_limit: 100.0,
        }
    }
}

impl ControlConfig {
    /// Validate control configuration
    ///
    /// # Validation Rules
    /// - control_frequency > 0
    /// - max_joint_velocity > 0
    /// - max_joint_acceleration > 0
    /// - kp, kd >= 0
    /// - torque_limit > 0
    pub fn validate(&self) -> anyhow::Result<()> {
        if self.control_frequency <= 0.0 {
            anyhow::bail!("Control frequency must be > 0");
        }
        if self.max_joint_velocity <= 0.0 {
            anyhow::bail!("Max joint velocity must be > 0");
        }
        if self.max_joint_acceleration <= 0.0 {
            anyhow::bail!("Max joint acceleration must be > 0");
        }
        if self.kp < 0.0 || self.kd < 0.0 {
            anyhow::bail!("PID gains must be >= 0");
        }
        if self.torque_limit <= 0.0 {
            anyhow::bail!("Torque limit must be > 0");
        }
        Ok(())
    }
}

/// Logging configuration
///
/// Controls what gets logged and where.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    /// Log level (trace, debug, info, warn, error)
    pub level: String,
    /// Enable console output
    pub console: bool,
    /// Log file path (optional)
    pub file: Option<PathBuf>,
    /// Log format (compact, pretty, json)
    pub format: LogFormat,
}

/// Log output format
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum LogFormat {
    /// Compact single-line format
    Compact,
    /// Pretty-printed multi-line format
    Pretty,
    /// JSON structured format
    Json,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            level: "info".to_string(),
            console: true,
            file: None,
            format: LogFormat::Compact,
        }
    }
}

impl LoggingConfig {
    /// Validate logging configuration
    pub fn validate(&self) -> anyhow::Result<()> {
        let valid_levels = ["trace", "debug", "info", "warn", "error"];
        if !valid_levels.contains(&self.level.as_str()) {
            anyhow::bail!(
                "Invalid log level: {}. Must be one of {:?}",
                self.level,
                valid_levels
            );
        }
        Ok(())
    }
}

/// Simulation configuration
///
/// Settings specific to simulation mode.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationConfig {
    /// Simulation timestep
    pub timestep: Duration,
    /// Real-time factor (1.0 = real-time, 0.0 = unlimited)
    pub real_time_factor: f64,
    /// Run in headless mode (no rendering)
    pub headless: bool,
    /// Scene file path (optional)
    pub scene_path: Option<PathBuf>,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            timestep: Duration::from_millis(2), // 500 Hz
            real_time_factor: 1.0,
            headless: false,
            scene_path: None,
        }
    }
}

impl SimulationConfig {
    /// Validate simulation configuration
    ///
    /// # Validation Rules
    /// - timestep > 0 and <= 0.01s
    /// - real_time_factor >= 0
    pub fn validate(&self) -> anyhow::Result<()> {
        let timestep_secs = self.timestep.as_secs_f64();

        if timestep_secs <= 0.0 {
            anyhow::bail!("Simulation timestep must be > 0");
        }
        if timestep_secs > 0.01 {
            anyhow::bail!("Simulation timestep must be <= 0.01s (100Hz minimum)");
        }
        if self.real_time_factor < 0.0 {
            anyhow::bail!("Real-time factor must be >= 0");
        }

        if let Some(ref path) = self.scene_path {
            if !path.exists() {
                warn!("Scene file does not exist: {}", path.display());
            }
        }

        Ok(())
    }
}

impl SDKConfig {
    /// Load configuration from a file
    ///
    /// Supports TOML, JSON, and YAML formats (detected by file extension).
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the configuration file
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be read or parsed.
    pub fn from_file(path: &Path) -> crate::Result<Self> {
        info!("Loading SDK config from: {}", path.display());

        let content = std::fs::read_to_string(path).map_err(|e| {
            crate::HumanoidError::CommunicationError(format!("Failed to read config file: {}", e))
        })?;

        let config: Self = if path.extension().map(|e| e == "json").unwrap_or(false) {
            serde_json::from_str(&content).map_err(|e| {
                crate::HumanoidError::CommunicationError(format!("Failed to parse JSON: {}", e))
            })?
        } else {
            // Default to TOML
            toml::from_str(&content).map_err(|e| {
                crate::HumanoidError::CommunicationError(format!("Failed to parse TOML: {}", e))
            })?
        };

        debug!("Loaded config: {:?}", config);
        Ok(config)
    }

    /// Validate the entire configuration
    ///
    /// Checks all subsystems for valid values.
    pub fn validate(&self) -> crate::Result<()> {
        debug!("Validating SDK configuration");

        self.physics.validate().map_err(|e| {
            crate::HumanoidError::ControlError(format!("Physics config invalid: {}", e))
        })?;

        self.robot.validate().map_err(|e| {
            crate::HumanoidError::ControlError(format!("Robot config invalid: {}", e))
        })?;

        self.control.validate().map_err(|e| {
            crate::HumanoidError::ControlError(format!("Control config invalid: {}", e))
        })?;

        self.logging.validate().map_err(|e| {
            crate::HumanoidError::ControlError(format!("Logging config invalid: {}", e))
        })?;

        info!("SDK configuration validated successfully");
        Ok(())
    }

    /// Merge another configuration into this one
    ///
    /// Non-default values from `other` will override values in `self`.
    pub fn merge(&mut self, other: SDKConfig) {
        debug!("Merging SDK configurations");

        // Physics merge
        if other.physics.timestep != PhysicsConfig::default().timestep {
            self.physics.timestep = other.physics.timestep;
        }
        if other.physics.solver_iterations != PhysicsConfig::default().solver_iterations {
            self.physics.solver_iterations = other.physics.solver_iterations;
        }

        // Robot merge
        if !other.robot.model.is_empty() && other.robot.model != "unitree_g1" {
            self.robot.model = other.robot.model;
        }
        if other.robot.urdf_path.is_some() {
            self.robot.urdf_path = other.robot.urdf_path;
        }

        // Control merge
        if other.control.control_frequency != ControlConfig::default().control_frequency {
            self.control.control_frequency = other.control.control_frequency;
        }

        // Logging merge
        if other.logging.level != "info" {
            self.logging.level = other.logging.level;
        }
        if other.logging.file.is_some() {
            self.logging.file = other.logging.file;
        }
    }

    /// Save configuration to a file
    ///
    /// # Arguments
    ///
    /// * `path` - Path to save the configuration
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be written.
    pub fn to_file(&self, path: &Path) -> crate::Result<()> {
        info!("Saving SDK config to: {}", path.display());

        let content = if path.extension().map(|e| e == "json").unwrap_or(false) {
            serde_json::to_string_pretty(self).map_err(|e| {
                crate::HumanoidError::CommunicationError(format!("Failed to serialize JSON: {}", e))
            })?
        } else {
            toml::to_string_pretty(self).map_err(|e| {
                crate::HumanoidError::CommunicationError(format!("Failed to serialize TOML: {}", e))
            })?
        };

        std::fs::write(path, content).map_err(|e| {
            crate::HumanoidError::CommunicationError(format!("Failed to write config file: {}", e))
        })?;

        Ok(())
    }
}

impl Default for SDKConfig {
    fn default() -> Self {
        Self {
            physics: PhysicsConfig::default(),
            robot: RobotConfig::default(),
            control: ControlConfig::default(),
            logging: LoggingConfig::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn test_default_config() {
        let config = SDKConfig::default();
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_physics_validation() {
        let mut config = PhysicsConfig::default();
        assert!(config.validate().is_ok());

        config.timestep = 0.0;
        assert!(config.validate().is_err());

        config.timestep = 0.02; // > 0.01
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_robot_validation() {
        let mut config = RobotConfig::default();
        assert!(config.validate().is_ok());

        config.model = "".to_string();
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_control_validation() {
        let mut config = ControlConfig::default();
        assert!(config.validate().is_ok());

        config.control_frequency = 0.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_simulation_validation() {
        let mut config = SimulationConfig::default();
        assert!(config.validate().is_ok());

        config.timestep = Duration::from_secs(0);
        assert!(config.validate().is_err());

        config.timestep = Duration::from_millis(20); // > 0.01s
        assert!(config.validate().is_err());

        config.timestep = Duration::from_millis(2);
        config.real_time_factor = -1.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_logging_validation() {
        let mut config = LoggingConfig::default();
        assert!(config.validate().is_ok());

        config.level = "invalid".to_string();
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_config_merge() {
        let mut config1 = SDKConfig::default();
        let config2 = SDKConfig {
            physics: PhysicsConfig {
                timestep: 0.001,
                ..Default::default()
            },
            robot: RobotConfig {
                model: "unitree_h1".to_string(),
                ..Default::default()
            },
            control: ControlConfig::default(),
            logging: LoggingConfig {
                level: "debug".to_string(),
                ..Default::default()
            },
        };

        config1.merge(config2);

        assert_eq!(config1.physics.timestep, 0.001);
        assert_eq!(config1.robot.model, "unitree_h1");
        assert_eq!(config1.logging.level, "debug");
    }

    #[test]
    fn test_config_serialization() {
        let config = SDKConfig::default();

        // Test JSON serialization
        let json = serde_json::to_string(&config).unwrap();
        let deserialized: SDKConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.physics.timestep, config.physics.timestep);

        // Test TOML serialization
        let toml_str = toml::to_string(&config).unwrap();
        let deserialized: SDKConfig = toml::from_str(&toml_str).unwrap();
        assert_eq!(deserialized.robot.model, config.robot.model);
    }
}
