//! Observability and debug infrastructure for simulation
//!
//! Provides debug hooks, replay system, visualization sync, and tracing integration.

use crate::logging::{ContactEvent, ControlCommand, LoggedState, SensorData, SimulationLogger};
use crate::robot::{JointSet, Pose, RobotState};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::path::Path;
use std::time::Duration;
use thiserror::Error;
use tracing::{error, info, span, warn, Level};

/// Errors that can occur in observability operations
#[derive(Error, Debug)]
pub enum ObservabilityError {
    #[error("Logging error: {0}")]
    Logging(String),

    #[error("Replay error: {0}")]
    Replay(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

/// Debug hooks for simulation events
pub struct DebugHooks {
    /// Called on each simulation step
    pub on_step: Option<Box<dyn Fn(&LoggedState) + Send + Sync>>,
    /// Called on contact events
    pub on_contact: Option<Box<dyn Fn(&ContactEvent) + Send + Sync>>,
    /// Called when a limit is exceeded
    pub on_limit_exceeded: Option<Box<dyn Fn(&str, f64) + Send + Sync>>,
    /// Called on errors
    pub on_error: Option<Box<dyn Fn(&str) + Send + Sync>>,
}

impl DebugHooks {
    /// Create new debug hooks with no callbacks
    pub fn new() -> Self {
        Self {
            on_step: None,
            on_contact: None,
            on_limit_exceeded: None,
            on_error: None,
        }
    }

    /// Set the on_step callback
    pub fn with_on_step<F>(mut self, callback: F) -> Self
    where
        F: Fn(&LoggedState) + Send + Sync + 'static,
    {
        self.on_step = Some(Box::new(callback));
        self
    }

    /// Set the on_contact callback
    pub fn with_on_contact<F>(mut self, callback: F) -> Self
    where
        F: Fn(&ContactEvent) + Send + Sync + 'static,
    {
        self.on_contact = Some(Box::new(callback));
        self
    }

    /// Set the on_limit_exceeded callback
    pub fn with_on_limit_exceeded<F>(mut self, callback: F) -> Self
    where
        F: Fn(&str, f64) + Send + Sync + 'static,
    {
        self.on_limit_exceeded = Some(Box::new(callback));
        self
    }

    /// Set the on_error callback
    pub fn with_on_error<F>(mut self, callback: F) -> Self
    where
        F: Fn(&str) + Send + Sync + 'static,
    {
        self.on_error = Some(Box::new(callback));
        self
    }

    /// Trigger the on_step hook
    pub fn trigger_step(&self, state: &LoggedState) {
        if let Some(ref hook) = self.on_step {
            let _span = span!(Level::DEBUG, "debug_hook", hook = "on_step");
            hook(state);
        }
    }

    /// Trigger the on_contact hook
    pub fn trigger_contact(&self, event: &ContactEvent) {
        if let Some(ref hook) = self.on_contact {
            let _span = span!(Level::DEBUG, "debug_hook", hook = "on_contact");
            hook(event);
        }
    }

    /// Trigger the on_limit_exceeded hook
    pub fn trigger_limit_exceeded(&self, limit_name: &str, value: f64) {
        if let Some(ref hook) = self.on_limit_exceeded {
            let _span = span!(Level::DEBUG, "debug_hook", hook = "on_limit_exceeded");
            hook(limit_name, value);
        }
    }

    /// Trigger the on_error hook
    pub fn trigger_error(&self, error: &str) {
        if let Some(ref hook) = self.on_error {
            let _span = span!(Level::DEBUG, "debug_hook", hook = "on_error");
            hook(error);
        }
    }
}

impl Default for DebugHooks {
    fn default() -> Self {
        Self::new()
    }
}

/// Configuration for tracing/logging levels per subsystem
#[derive(Debug, Clone)]
pub struct LogConfig {
    pub physics_level: Level,
    pub control_level: Level,
    pub robot_level: Level,
    pub simulation_level: Level,
}

impl Default for LogConfig {
    fn default() -> Self {
        Self {
            physics_level: Level::INFO,
            control_level: Level::DEBUG,
            robot_level: Level::DEBUG,
            simulation_level: Level::INFO,
        }
    }
}

/// Replay system for deterministic simulation playback
pub struct ReplaySystem {
    /// Recorded states for replay
    states: VecDeque<LoggedState>,
    /// Current replay position
    current_index: usize,
    /// Whether recording or playing
    mode: ReplayMode,
    /// Recording metadata
    metadata: ReplayMetadata,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReplayMode {
    Recording,
    Playing,
    Paused,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayMetadata {
    pub total_steps: u64,
    pub duration_secs: f64,
    pub initial_state: Option<RobotState>,
    pub seed: Option<u64>,
}

impl ReplaySystem {
    /// Create a new replay system in recording mode
    pub fn new_recording(initial_state: Option<RobotState>) -> Self {
        Self {
            states: VecDeque::new(),
            current_index: 0,
            mode: ReplayMode::Recording,
            metadata: ReplayMetadata {
                total_steps: 0,
                duration_secs: 0.0,
                initial_state,
                seed: None,
            },
        }
    }

    /// Create a new replay system for playback
    pub fn new_playback(states: VecDeque<LoggedState>, metadata: ReplayMetadata) -> Self {
        Self {
            states,
            current_index: 0,
            mode: ReplayMode::Paused,
            metadata,
        }
    }

    /// Record a state
    pub fn record(&mut self, state: LoggedState) {
        if self.mode == ReplayMode::Recording {
            self.states.push_back(state);
            self.metadata.total_steps += 1;
        }
    }

    /// Get the next state in playback mode
    pub fn next_state(&mut self) -> Option<&LoggedState> {
        if self.mode == ReplayMode::Playing {
            if self.current_index < self.states.len() {
                let state = self.states.get(self.current_index);
                self.current_index += 1;
                state
            } else {
                self.mode = ReplayMode::Paused;
                None
            }
        } else {
            self.states.get(self.current_index)
        }
    }

    /// Step forward one frame in playback
    pub fn step_forward(&mut self) -> Option<&LoggedState> {
        if self.current_index < self.states.len().saturating_sub(1) {
            self.current_index += 1;
            self.states.get(self.current_index)
        } else {
            None
        }
    }

    /// Step backward one frame in playback
    pub fn step_backward(&mut self) -> Option<&LoggedState> {
        if self.current_index > 0 {
            self.current_index -= 1;
            self.states.get(self.current_index)
        } else {
            None
        }
    }

    /// Jump to a specific frame
    pub fn goto_frame(&mut self, frame: usize) -> Option<&LoggedState> {
        if frame < self.states.len() {
            self.current_index = frame;
            self.states.get(self.current_index)
        } else {
            None
        }
    }

    /// Start playback
    pub fn play(&mut self) {
        self.mode = ReplayMode::Playing;
        info!("Replay playback started");
    }

    /// Pause playback
    pub fn pause(&mut self) {
        self.mode = ReplayMode::Paused;
        info!("Replay playback paused");
    }

    /// Stop and reset playback
    pub fn stop(&mut self) {
        self.mode = ReplayMode::Paused;
        self.current_index = 0;
        info!("Replay playback stopped");
    }

    /// Get current playback mode
    pub fn mode(&self) -> ReplayMode {
        self.mode
    }

    /// Get current frame index
    pub fn current_frame(&self) -> usize {
        self.current_index
    }

    /// Get total number of frames
    pub fn total_frames(&self) -> usize {
        self.states.len()
    }

    /// Get progress as a percentage
    pub fn progress(&self) -> f64 {
        if self.states.is_empty() {
            0.0
        } else {
            self.current_index as f64 / self.states.len() as f64 * 100.0
        }
    }

    /// Get current state
    pub fn current_state(&self) -> Option<&LoggedState> {
        self.states.get(self.current_index)
    }

    /// Save replay to file
    pub fn save(&self, path: &Path) -> Result<(), ObservabilityError> {
        if self.mode == ReplayMode::Recording {
            let data = ReplayData {
                metadata: self.metadata.clone(),
                states: self.states.iter().cloned().collect(),
            };

            let json = serde_json::to_string_pretty(&data)
                .map_err(|e| ObservabilityError::Logging(e.to_string()))?;

            std::fs::write(path, json)?;
            info!(path = %path.display(), "Replay saved");
            Ok(())
        } else {
            Err(ObservabilityError::Replay(
                "Cannot save in playback mode".to_string(),
            ))
        }
    }

    /// Load replay from file
    pub fn load(path: &Path) -> Result<Self, ObservabilityError> {
        let json = std::fs::read_to_string(path)?;
        let data: ReplayData =
            serde_json::from_str(&json).map_err(|e| ObservabilityError::Replay(e.to_string()))?;

        info!(path = %path.display(), frames = data.states.len(), "Replay loaded");

        Ok(Self::new_playback(data.states.into(), data.metadata))
    }
}

#[derive(Serialize, Deserialize)]
struct ReplayData {
    metadata: ReplayMetadata,
    states: Vec<LoggedState>,
}

/// Visualization sync for external tools
pub struct VisualizationSync {
    /// Data export queue
    export_queue: VecDeque<VisualizationFrame>,
    /// Live plotting hooks
    plot_hooks: Vec<Box<dyn Fn(&VisualizationFrame) + Send + Sync>>,
    /// CSV file handle
    csv_writer: Option<csv::Writer<std::fs::File>>,
    /// Export configuration
    config: VizConfig,
}

#[derive(Debug, Clone)]
pub struct VizConfig {
    pub export_csv: bool,
    pub live_plotting: bool,
    pub update_rate_hz: f64,
    pub output_dir: std::path::PathBuf,
}

impl Default for VizConfig {
    fn default() -> Self {
        Self {
            export_csv: false,
            live_plotting: false,
            update_rate_hz: 30.0,
            output_dir: std::path::PathBuf::from("./viz_output"),
        }
    }
}

/// Frame data for visualization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualizationFrame {
    pub step: u64,
    pub timestamp: Duration,
    pub joint_positions: Vec<f64>,
    pub joint_velocities: Vec<f64>,
    pub joint_torques: Vec<f64>,
    pub base_position: [f64; 3],
    pub base_orientation: [f64; 4], // quaternion
    pub contacts: Vec<ContactData>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactData {
    pub location: String,
    pub position: [f64; 3],
    pub force: [f64; 3],
}

impl VisualizationSync {
    /// Create a new visualization sync
    pub fn new(config: VizConfig) -> Self {
        let csv_writer = if config.export_csv {
            std::fs::create_dir_all(&config.output_dir).ok();
            let csv_path = config.output_dir.join("visualization.csv");
            let writer = csv::Writer::from_path(&csv_path).ok();
            if writer.is_some() {
                info!("CSV export enabled: {}", csv_path.display());
            }
            writer
        } else {
            None
        };

        Self {
            export_queue: VecDeque::with_capacity(100),
            plot_hooks: Vec::new(),
            csv_writer,
            config,
        }
    }

    /// Add a live plotting hook
    pub fn add_plot_hook<F>(&mut self, hook: F)
    where
        F: Fn(&VisualizationFrame) + Send + Sync + 'static,
    {
        self.plot_hooks.push(Box::new(hook));
    }

    /// Publish a frame to all visualization outputs
    pub fn publish_frame(&mut self, frame: VisualizationFrame) {
        // Write to CSV if enabled
        if let Some(ref mut writer) = self.csv_writer {
            let _ = writer.serialize(&frame);
        }

        // Call live plotting hooks
        if self.config.live_plotting {
            for hook in &self.plot_hooks {
                hook(&frame);
            }
        }

        // Add to queue
        if self.export_queue.len() >= 100 {
            self.export_queue.pop_front();
        }
        self.export_queue.push_back(frame);
    }

    /// Export queued frames to JSON
    pub fn export_json(&self, path: &Path) -> Result<(), ObservabilityError> {
        let frames: Vec<_> = self.export_queue.iter().cloned().collect();
        let json = serde_json::to_string_pretty(&frames)
            .map_err(|e| ObservabilityError::Logging(e.to_string()))?;

        std::fs::write(path, json)?;
        info!(path = %path.display(), frames = frames.len(), "Exported visualization frames");
        Ok(())
    }

    /// Flush CSV writer
    pub fn flush(&mut self) -> Result<(), ObservabilityError> {
        if let Some(ref mut writer) = self.csv_writer {
            writer.flush()?;
        }
        Ok(())
    }
}

/// Complete observability system combining all features
pub struct ObservabilitySystem {
    logger: SimulationLogger,
    debug_hooks: DebugHooks,
    replay: Option<ReplaySystem>,
    viz_sync: Option<VisualizationSync>,
    log_config: LogConfig,
}

impl ObservabilitySystem {
    /// Create a new observability system
    pub fn new(logger: SimulationLogger) -> Self {
        Self {
            logger,
            debug_hooks: DebugHooks::default(),
            replay: None,
            viz_sync: None,
            log_config: LogConfig::default(),
        }
    }

    /// Set debug hooks
    pub fn set_debug_hooks(&mut self, hooks: DebugHooks) {
        self.debug_hooks = hooks;
    }

    /// Enable replay recording
    pub fn enable_replay_recording(&mut self, initial_state: Option<RobotState>) {
        self.replay = Some(ReplaySystem::new_recording(initial_state));
        info!("Replay recording enabled");
    }

    /// Enable visualization sync
    pub fn enable_visualization(&mut self, config: VizConfig) {
        self.viz_sync = Some(VisualizationSync::new(config));
        info!("Visualization sync enabled");
    }

    /// Log a complete simulation step
    pub fn log_step(
        &mut self,
        robot_state: RobotState,
        sensor_data: Vec<SensorData>,
        control_commands: Vec<ControlCommand>,
    ) {
        let _span = span!(Level::DEBUG, "log_step", step = self.logger.current_step()).entered();

        // Log to the logger
        self.logger.log_state(
            robot_state.clone(),
            sensor_data.clone(),
            control_commands.clone(),
        );

        // Record to replay system if active
        if let Some(ref mut replay) = self.replay {
            let state = LoggedState {
                step: self.logger.current_step(),
                timestamp: self.logger.elapsed(),
                robot_state: robot_state.clone(),
                sensor_data: sensor_data.clone(),
                control_commands: control_commands.clone(),
            };
            replay.record(state);
        }

        // Publish to visualization if active
        if let Some(ref mut viz) = self.viz_sync {
            let frame = VisualizationFrame {
                step: self.logger.current_step(),
                timestamp: self.logger.elapsed(),
                joint_positions: robot_state.joints.positions(),
                joint_velocities: robot_state.joints.iter().map(|j| j.velocity).collect(),
                joint_torques: robot_state.joints.iter().map(|j| j.effort).collect(),
                base_position: [
                    robot_state.base_pose.position.x,
                    robot_state.base_pose.position.y,
                    robot_state.base_pose.position.z,
                ],
                base_orientation: [
                    robot_state.base_pose.orientation.i,
                    robot_state.base_pose.orientation.j,
                    robot_state.base_pose.orientation.k,
                    robot_state.base_pose.orientation.w,
                ],
                contacts: robot_state
                    .foot_contacts
                    .iter()
                    .enumerate()
                    .map(|(i, contact)| {
                        ContactData {
                            location: if i == 0 {
                                "left".to_string()
                            } else {
                                "right".to_string()
                            },
                            position: [0.0, 0.0, 0.0], // Would need actual CoP
                            force: if *contact {
                                [0.0, 100.0, 0.0]
                            } else {
                                [0.0, 0.0, 0.0]
                            },
                        }
                    })
                    .collect(),
            };
            viz.publish_frame(frame);
        }

        // Trigger debug hooks
        let logged_state = LoggedState {
            step: self.logger.current_step(),
            timestamp: self.logger.elapsed(),
            robot_state,
            sensor_data,
            control_commands,
        };
        self.debug_hooks.trigger_step(&logged_state);

        // Increment step
        self.logger.step();
    }

    /// Log a contact event
    pub fn log_contact(
        &mut self,
        location: String,
        in_contact: bool,
        contact_force: Option<[f64; 3]>,
        contact_normal: Option<[f64; 3]>,
    ) {
        self.logger
            .log_contact(location.clone(), in_contact, contact_force, contact_normal);

        let event = ContactEvent {
            step: self.logger.current_step(),
            timestamp: self.logger.elapsed(),
            location,
            in_contact,
            contact_force,
            contact_normal,
        };

        self.debug_hooks.trigger_contact(&event);
    }

    /// Log a limit exceeded event
    pub fn log_limit_exceeded(&mut self, limit_name: &str, value: f64) {
        warn!(limit = limit_name, value = value, "Limit exceeded");
        self.logger.log_command(
            "limit_exceeded".to_string(),
            format!("{} exceeded with value {}", limit_name, value),
            None,
        );
        self.debug_hooks.trigger_limit_exceeded(limit_name, value);
    }

    /// Log an error
    pub fn log_error(&mut self, error: &str) {
        error!("{}", error);
        self.logger
            .log_command("error".to_string(), error.to_string(), None);
        self.debug_hooks.trigger_error(error);
    }

    /// Get the logger
    pub fn logger(&self) -> &SimulationLogger {
        &self.logger
    }

    /// Get the logger mutably
    pub fn logger_mut(&mut self) -> &mut SimulationLogger {
        &mut self.logger
    }

    /// Get the replay system
    pub fn replay(&self) -> Option<&ReplaySystem> {
        self.replay.as_ref()
    }

    /// Get the replay system mutably
    pub fn replay_mut(&mut self) -> Option<&mut ReplaySystem> {
        self.replay.as_mut()
    }

    /// Get the visualization sync
    pub fn viz_sync(&self) -> Option<&VisualizationSync> {
        self.viz_sync.as_ref()
    }

    /// Save replay to file
    pub fn save_replay(&self, path: &Path) -> Result<(), ObservabilityError> {
        if let Some(ref replay) = self.replay {
            replay.save(path)
        } else {
            Err(ObservabilityError::Replay("Replay not enabled".to_string()))
        }
    }

    /// Export visualization data
    pub fn export_visualization(&self, path: &Path) -> Result<(), ObservabilityError> {
        if let Some(ref viz) = self.viz_sync {
            viz.export_json(path)
        } else {
            Err(ObservabilityError::Logging(
                "Visualization not enabled".to_string(),
            ))
        }
    }

    /// Flush all outputs
    pub fn flush(&mut self) -> Result<(), ObservabilityError> {
        if let Some(ref mut viz) = self.viz_sync {
            viz.flush()?;
        }
        Ok(())
    }
}

/// Trait for simulation control with observability
pub trait SimulationControl {
    /// Set debug hooks for the simulation
    fn set_debug_hooks(&mut self, hooks: DebugHooks);

    /// Get the observability system
    fn observability(&self) -> Option<&ObservabilitySystem>;

    /// Get the observability system mutably
    fn observability_mut(&mut self) -> Option<&mut ObservabilitySystem>;
}

/// Comparison result for regression testing
#[derive(Debug, Clone)]
pub struct StateComparison {
    pub differences: Vec<StateDifference>,
    pub max_joint_error: f64,
    pub max_position_error: f64,
    pub is_match: bool,
}

#[derive(Debug, Clone)]
pub struct StateDifference {
    pub field: String,
    pub expected: f64,
    pub actual: f64,
    pub error: f64,
}

/// Compare two logged states for regression testing
pub fn compare_states(
    expected: &LoggedState,
    actual: &LoggedState,
    tolerance: f64,
) -> StateComparison {
    let mut differences = Vec::new();
    let mut max_joint_error: f64 = 0.0;
    let mut max_position_error: f64 = 0.0;

    // Compare joint positions
    let expected_positions: Vec<_> = expected.robot_state.joints.positions();
    let actual_positions: Vec<_> = actual.robot_state.joints.positions();

    for (i, (exp, act)) in expected_positions
        .iter()
        .zip(actual_positions.iter())
        .enumerate()
    {
        let error = (exp - act).abs();
        if error > tolerance {
            differences.push(StateDifference {
                field: format!("joint_{}_position", i),
                expected: *exp,
                actual: *act,
                error,
            });
        }
        max_joint_error = max_joint_error.max(error);
    }

    // Compare base position
    let pos_err =
        (expected.robot_state.base_pose.position - actual.robot_state.base_pose.position).norm();
    if pos_err > tolerance {
        differences.push(StateDifference {
            field: "base_position".to_string(),
            expected: expected.robot_state.base_pose.position.norm(),
            actual: actual.robot_state.base_pose.position.norm(),
            error: pos_err,
        });
    }
    max_position_error = max_position_error.max(pos_err);

    StateComparison {
        differences,
        max_joint_error,
        max_position_error,
        is_match: max_joint_error < tolerance && max_position_error < tolerance,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::unitree_g1;

    #[test]
    fn test_debug_hooks() {
        let mut step_called = false;
        let mut contact_called = false;

        let hooks = DebugHooks::new()
            .with_on_step(|_| {
                step_called = true;
            })
            .with_on_contact(|_| {
                contact_called = true;
            });

        let state = LoggedState {
            step: 0,
            timestamp: Duration::from_secs(0),
            robot_state: RobotState {
                timestamp: Duration::from_secs(0),
                base_pose: Pose::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                joints: JointSet::new(),
                is_standing: false,
                is_balanced: false,
                foot_contacts: [false, false],
            },
            sensor_data: vec![],
            control_commands: vec![],
        };

        hooks.trigger_step(&state);
        assert!(step_called);

        let contact = ContactEvent {
            step: 0,
            timestamp: Duration::from_secs(0),
            location: "left_foot".to_string(),
            in_contact: true,
            contact_force: None,
            contact_normal: None,
        };

        hooks.trigger_contact(&contact);
        assert!(contact_called);
    }

    #[test]
    fn test_replay_system() {
        let mut replay = ReplaySystem::new_recording(None);

        let state = LoggedState {
            step: 0,
            timestamp: Duration::from_secs(0),
            robot_state: RobotState {
                timestamp: Duration::from_secs(0),
                base_pose: Pose::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                joints: JointSet::new(),
                is_standing: false,
                is_balanced: false,
                foot_contacts: [false, false],
            },
            sensor_data: vec![],
            control_commands: vec![],
        };

        replay.record(state.clone());
        replay.record(state.clone());
        replay.record(state.clone());

        assert_eq!(replay.total_frames(), 3);

        // Save and load
        let temp_dir = std::env::temp_dir();
        let path = temp_dir.join("test_replay.json");

        replay.save(&path).unwrap();
        let loaded = ReplaySystem::load(&path).unwrap();

        assert_eq!(loaded.total_frames(), 3);

        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn test_state_comparison() {
        let state1 = LoggedState {
            step: 0,
            timestamp: Duration::from_secs(0),
            robot_state: RobotState {
                timestamp: Duration::from_secs(0),
                base_pose: Pose::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                joints: unitree_g1::create_default_joints(),
                is_standing: false,
                is_balanced: false,
                foot_contacts: [false, false],
            },
            sensor_data: vec![],
            control_commands: vec![],
        };

        let mut state2 = state1.clone();

        // States should match
        let comparison = compare_states(&state1, &state2, 0.01);
        assert!(comparison.is_match);

        // Modify state2
        state2.robot_state.base_pose.position.x = 0.1;

        let comparison = compare_states(&state1, &state2, 0.01);
        assert!(!comparison.is_match);
        assert!(comparison.max_position_error > 0.0);
    }
}
