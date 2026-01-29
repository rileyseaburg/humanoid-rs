//! Unified logging system for humanoid robot simulation
//!
//! Provides structured logging with tracing integration, circular buffer state history,
//! and multiple export formats (JSON, CSV, binary).

use crate::robot::RobotState;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::io::Write;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;
use tracing::{debug, error, info, span, Level};

/// Errors that can occur during logging operations
#[derive(Error, Debug)]
pub enum LoggingError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    Serialization(String),

    #[error("Export format not supported: {0}")]
    UnsupportedFormat(String),
}

/// A single logged state snapshot
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggedState {
    /// Simulation step number
    pub step: u64,
    /// Timestamp since simulation start
    pub timestamp: Duration,
    /// Robot state at this step
    pub robot_state: RobotState,
    /// Sensor data readings
    pub sensor_data: Vec<SensorData>,
    /// Control commands issued
    pub control_commands: Vec<ControlCommand>,
}

/// Sensor data types that can be logged
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorData {
    /// IMU sensor data
    Imu {
        linear_acceleration: [f64; 3],
        angular_velocity: [f64; 3],
        orientation: [f64; 4], // quaternion
    },
    /// Joint sensor data
    Joint {
        name: String,
        position: f64,
        velocity: f64,
        effort: f64,
    },
    /// Force/torque sensor data
    ForceTorque {
        location: String,
        force: [f64; 3],
        torque: [f64; 3],
    },
    /// Contact sensor data
    Contact {
        location: String,
        in_contact: bool,
        contact_force: Option<[f64; 3]>,
    },
    /// Custom sensor data
    Custom { name: String, value: f64 },
}

/// Control command types for logging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogControlCommand {
    pub joint_name: String,
    pub command_type: LogCommandType,
    pub value: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LogCommandType {
    Position,
    Velocity,
    Torque,
}

// Type alias for backward compatibility
pub type ControlCommand = LogControlCommand;

/// Contact event for logging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactEvent {
    pub step: u64,
    pub timestamp: Duration,
    pub location: String,
    pub in_contact: bool,
    pub contact_force: Option<[f64; 3]>,
    pub contact_normal: Option<[f64; 3]>,
}

/// Command event for logging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandEvent {
    pub step: u64,
    pub timestamp: Duration,
    pub command_type: String,
    pub description: String,
    pub data: Option<serde_json::Value>,
}

/// Export format for logged data
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExportFormat {
    Json,
    Csv,
    Binary,
}

/// Unified simulation logger with circular buffer history
pub struct SimulationLogger {
    state_history: VecDeque<LoggedState>,
    contact_history: VecDeque<ContactEvent>,
    command_history: VecDeque<CommandEvent>,
    max_history: usize,
    current_step: u64,
    start_time: std::time::Instant,
    trace_spans_enabled: bool,
}

impl SimulationLogger {
    /// Create a new logger with the specified maximum history size
    pub fn new(max_history: usize) -> Self {
        let _span = span!(Level::INFO, "SimulationLogger::new").entered();
        info!("Creating SimulationLogger with max_history={}", max_history);

        Self {
            state_history: VecDeque::with_capacity(max_history),
            contact_history: VecDeque::with_capacity(max_history / 10),
            command_history: VecDeque::with_capacity(max_history / 10),
            max_history,
            current_step: 0,
            start_time: std::time::Instant::now(),
            trace_spans_enabled: true,
        }
    }

    /// Log a state snapshot
    pub fn log_state(
        &mut self,
        robot_state: RobotState,
        sensor_data: Vec<SensorData>,
        control_commands: Vec<ControlCommand>,
    ) {
        let _span = if self.trace_spans_enabled {
            Some(span!(Level::DEBUG, "log_state", step = self.current_step).entered())
        } else {
            None
        };

        let state = LoggedState {
            step: self.current_step,
            timestamp: self.start_time.elapsed(),
            robot_state,
            sensor_data,
            control_commands,
        };

        if self.state_history.len() >= self.max_history {
            self.state_history.pop_front();
        }
        self.state_history.push_back(state);

        debug!(step = self.current_step, "State logged");
    }

    /// Log a contact event
    pub fn log_contact(
        &mut self,
        location: String,
        in_contact: bool,
        contact_force: Option<[f64; 3]>,
        contact_normal: Option<[f64; 3]>,
    ) {
        let event = ContactEvent {
            step: self.current_step,
            timestamp: self.start_time.elapsed(),
            location,
            in_contact,
            contact_force,
            contact_normal,
        };

        if in_contact {
            info!(step = self.current_step, location = %event.location, "Contact detected");
        }

        if self.contact_history.len() >= self.max_history {
            self.contact_history.pop_front();
        }
        self.contact_history.push_back(event);
    }

    /// Log a command event
    pub fn log_command(
        &mut self,
        command_type: String,
        description: String,
        data: Option<serde_json::Value>,
    ) {
        let _span = if self.trace_spans_enabled {
            Some(span!(Level::INFO, "log_command", command_type = %command_type).entered())
        } else {
            None
        };

        info!(command_type = %command_type, description = %description, "Command logged");

        let event = CommandEvent {
            step: self.current_step,
            timestamp: self.start_time.elapsed(),
            command_type,
            description,
            data,
        };

        if self.command_history.len() >= self.max_history / 10 {
            self.command_history.pop_front();
        }
        self.command_history.push_back(event);
    }

    /// Increment the simulation step counter
    pub fn step(&mut self) {
        self.current_step += 1;
    }

    /// Get the current step number
    pub fn current_step(&self) -> u64 {
        self.current_step
    }

    /// Get elapsed time since start
    pub fn elapsed(&self) -> Duration {
        self.start_time.elapsed()
    }

    /// Get the number of logged states
    pub fn state_count(&self) -> usize {
        self.state_history.len()
    }

    /// Get all logged states
    pub fn states(&self) -> &VecDeque<LoggedState> {
        &self.state_history
    }

    /// Get all contact events
    pub fn contacts(&self) -> &VecDeque<ContactEvent> {
        &self.contact_history
    }

    /// Get all command events
    pub fn commands(&self) -> &VecDeque<CommandEvent> {
        &self.command_history
    }

    /// Clear all history
    pub fn clear(&mut self) {
        self.state_history.clear();
        self.contact_history.clear();
        self.command_history.clear();
        self.current_step = 0;
        self.start_time = std::time::Instant::now();
        info!("Logger history cleared");
    }

    /// Export logged data to a file
    pub fn export(&self, path: &Path, format: ExportFormat) -> Result<(), LoggingError> {
        let _span =
            span!(Level::INFO, "export", path = %path.display(), format = ?format).entered();
        info!("Exporting simulation log");

        match format {
            ExportFormat::Json => self.export_json(path),
            ExportFormat::Csv => self.export_csv(path),
            ExportFormat::Binary => self.export_binary(path),
        }
    }

    /// Export to JSON format
    pub fn export_json(&self, path: &Path) -> Result<(), LoggingError> {
        #[derive(Serialize)]
        struct ExportData<'a> {
            metadata: ExportMetadata,
            states: &'a VecDeque<LoggedState>,
            contacts: &'a VecDeque<ContactEvent>,
            commands: &'a VecDeque<CommandEvent>,
        }

        #[derive(Serialize)]
        struct ExportMetadata {
            total_steps: u64,
            state_count: usize,
            contact_count: usize,
            command_count: usize,
            duration_secs: f64,
        }

        let data = ExportData {
            metadata: ExportMetadata {
                total_steps: self.current_step,
                state_count: self.state_history.len(),
                contact_count: self.contact_history.len(),
                command_count: self.command_history.len(),
                duration_secs: self.start_time.elapsed().as_secs_f64(),
            },
            states: &self.state_history,
            contacts: &self.contact_history,
            commands: &self.command_history,
        };

        let json = serde_json::to_string_pretty(&data)
            .map_err(|e| LoggingError::Serialization(e.to_string()))?;

        let mut file = std::fs::File::create(path)?;
        file.write_all(json.as_bytes())?;

        info!(path = %path.display(), "Exported to JSON");
        Ok(())
    }

    /// Export to CSV format (states only)
    pub fn export_csv(&self, path: &Path) -> Result<(), LoggingError> {
        let mut file = std::fs::File::create(path)?;

        // Write header
        writeln!(
            file,
            "step,timestamp_secs,is_standing,is_balanced,left_contact,right_contact"
        )?;

        // Write data rows
        for state in &self.state_history {
            writeln!(
                file,
                "{},{},{},{},{},{}",
                state.step,
                state.timestamp.as_secs_f64(),
                state.robot_state.is_standing,
                state.robot_state.is_balanced,
                state.robot_state.foot_contacts[0],
                state.robot_state.foot_contacts[1]
            )?;
        }

        info!(path = %path.display(), "Exported to CSV");
        Ok(())
    }

    /// Export to binary format
    pub fn export_binary(&self, path: &Path) -> Result<(), LoggingError> {
        let data = (
            self.state_history.clone(),
            self.contact_history.clone(),
            self.command_history.clone(),
        );
        let encoded =
            bincode::serialize(&data).map_err(|e| LoggingError::Serialization(e.to_string()))?;

        let mut file = std::fs::File::create(path)?;
        file.write_all(&encoded)?;

        info!(path = %path.display(), "Exported to binary");
        Ok(())
    }

    /// Load from binary format
    pub fn load_binary(&mut self, path: &Path) -> Result<(), LoggingError> {
        let bytes = std::fs::read(path)?;
        let (states, contacts, commands): (
            VecDeque<LoggedState>,
            VecDeque<ContactEvent>,
            VecDeque<CommandEvent>,
        ) = bincode::deserialize(&bytes).map_err(|e| LoggingError::Serialization(e.to_string()))?;

        self.state_history = states;
        self.contact_history = contacts;
        self.command_history = commands;
        self.current_step = self.state_history.back().map(|s| s.step).unwrap_or(0);

        info!(path = %path.display(), "Loaded from binary");
        Ok(())
    }

    /// Enable or disable trace spans
    pub fn set_trace_spans(&mut self, enabled: bool) {
        self.trace_spans_enabled = enabled;
    }
}

impl Default for SimulationLogger {
    fn default() -> Self {
        Self::new(10000)
    }
}

/// Extension trait for converting physics data to sensor data
#[cfg(feature = "simulator")]
pub mod physics_conversion {
    use super::*;
    use crate::physics_sim::{FootSensorData, ImuData, JointState, PhysicsJoint};
    use bevy::prelude::*;

    /// Convert IMU data to SensorData
    pub fn imu_to_sensor_data(imu: &ImuData) -> SensorData {
        SensorData::Imu {
            linear_acceleration: [
                imu.linear_acceleration.x as f64,
                imu.linear_acceleration.y as f64,
                imu.linear_acceleration.z as f64,
            ],
            angular_velocity: [
                imu.angular_velocity.x as f64,
                imu.angular_velocity.y as f64,
                imu.angular_velocity.z as f64,
            ],
            orientation: [
                imu.orientation.x as f64,
                imu.orientation.y as f64,
                imu.orientation.z as f64,
                imu.orientation.w as f64,
            ],
        }
    }

    /// Convert foot sensor data to SensorData
    pub fn foot_sensor_to_sensor_data(foot: &FootSensorData) -> Vec<SensorData> {
        vec![
            SensorData::Contact {
                location: "left_foot".to_string(),
                in_contact: foot.left_contact,
                contact_force: if foot.left_contact {
                    Some([
                        foot.left_force.x as f64,
                        foot.left_force.y as f64,
                        foot.left_force.z as f64,
                    ])
                } else {
                    None
                },
            },
            SensorData::Contact {
                location: "right_foot".to_string(),
                in_contact: foot.right_contact,
                contact_force: if foot.right_contact {
                    Some([
                        foot.right_force.x as f64,
                        foot.right_force.y as f64,
                        foot.right_force.z as f64,
                    ])
                } else {
                    None
                },
            },
        ]
    }

    /// Convert joint state to SensorData
    pub fn joint_state_to_sensor_data(
        joint_state: &JointState,
        joint_specs: &[String],
    ) -> Vec<SensorData> {
        joint_state
            .positions
            .iter()
            .zip(joint_state.velocities.iter())
            .zip(joint_state.torques.iter())
            .enumerate()
            .map(|(idx, ((pos, vel), torque))| SensorData::Joint {
                name: joint_specs
                    .get(idx)
                    .map(|s| s.clone())
                    .unwrap_or_else(|| format!("joint_{}", idx)),
                position: *pos as f64,
                velocity: *vel as f64,
                effort: *torque as f64,
            })
            .collect()
    }

    /// Convert PhysicsJoint to SensorData
    pub fn physics_joint_to_sensor_data(joint: &PhysicsJoint) -> SensorData {
        SensorData::Joint {
            name: joint.name.clone(),
            position: joint.current_position as f64,
            velocity: joint.current_velocity as f64,
            effort: joint.current_torque as f64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::{JointSet, Pose};
    use nalgebra::Vector3;

    fn create_test_state() -> RobotState {
        RobotState {
            timestamp: Duration::from_secs(0),
            base_pose: Pose::new(0.0, 0.85, 0.0, 0.0, 0.0, 0.0),
            joints: JointSet::new(),
            is_standing: true,
            is_balanced: true,
            foot_contacts: [true, true],
        }
    }

    #[test]
    fn test_logger_creation() {
        let logger = SimulationLogger::new(100);
        assert_eq!(logger.state_count(), 0);
        assert_eq!(logger.current_step(), 0);
    }

    #[test]
    fn test_log_state() {
        let mut logger = SimulationLogger::new(10);
        let state = create_test_state();

        logger.log_state(state.clone(), vec![], vec![]);
        assert_eq!(logger.state_count(), 1);

        // Fill the buffer
        for _ in 0..20 {
            logger.log_state(state.clone(), vec![], vec![]);
        }

        // Should only have max_history states
        assert_eq!(logger.state_count(), 10);
    }

    #[test]
    fn test_step_counter() {
        let mut logger = SimulationLogger::new(10);
        assert_eq!(logger.current_step(), 0);

        logger.step();
        assert_eq!(logger.current_step(), 1);

        logger.step();
        logger.step();
        assert_eq!(logger.current_step(), 3);
    }

    #[test]
    fn test_clear() {
        let mut logger = SimulationLogger::new(10);
        let state = create_test_state();

        logger.log_state(state.clone(), vec![], vec![]);
        logger.log_contact("left_foot".to_string(), true, None, None);
        logger.log_command("test".to_string(), "test command".to_string(), None);

        assert_eq!(logger.state_count(), 1);
        assert_eq!(logger.contacts().len(), 1);
        assert_eq!(logger.commands().len(), 1);
        assert_eq!(logger.current_step(), 0);

        logger.clear();

        assert_eq!(logger.state_count(), 0);
        assert_eq!(logger.contacts().len(), 0);
        assert_eq!(logger.commands().len(), 0);
        assert_eq!(logger.current_step(), 0);
    }

    #[test]
    fn test_export_csv() {
        let mut logger = SimulationLogger::new(10);
        let state = create_test_state();

        logger.log_state(state.clone(), vec![], vec![]);
        logger.step();
        logger.log_state(state.clone(), vec![], vec![]);

        let temp_dir = std::env::temp_dir();
        let path = temp_dir.join("test_log.csv");

        logger.export_csv(&path).unwrap();

        assert!(path.exists());
        let content = std::fs::read_to_string(&path).unwrap();
        assert!(content.contains("step,timestamp_secs"));
        assert!(content.contains("true")); // is_standing

        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn test_export_json() {
        let mut logger = SimulationLogger::new(10);
        let state = create_test_state();

        logger.log_state(state.clone(), vec![], vec![]);
        logger.log_contact("left_foot".to_string(), true, None, None);

        let temp_dir = std::env::temp_dir();
        let path = temp_dir.join("test_log.json");

        logger.export_json(&path).unwrap();

        assert!(path.exists());
        let content = std::fs::read_to_string(&path).unwrap();
        assert!(content.contains("states"));
        assert!(content.contains("contacts"));
        assert!(content.contains("commands"));

        std::fs::remove_file(&path).unwrap();
    }
}
