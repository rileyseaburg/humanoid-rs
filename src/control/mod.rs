//! Control module for robot motion planning and execution
//!
//! This module provides:
//! - Trajectory planning and interpolation
//! - Motion planners for joint and Cartesian space
//! - Low-level controllers (PD, etc.)
//! - Safety guards and monitoring

use crate::robot::{JointSet, Pose, RobotState};
use nalgebra::Vector3;
use std::time::Duration;
use thiserror::Error;
use tracing::{debug, warn};

// Re-export safety module
pub mod safety;

// Re-export safety types
pub use safety::{
    CommunicationMonitor, ControlMode, EStopGuard, JointLimitGuard, JointLimits, SafetyError,
    SafetyGuard, SafetyLevel, SafetyManager, TorqueRampGuard,
};

/// Control command structure
#[derive(Debug, Clone)]
pub struct ControlCommand {
    pub timestamp: Duration,
    pub joint_positions: Option<Vec<f64>>,
    pub joint_velocities: Option<Vec<f64>>,
    pub joint_efforts: Option<Vec<f64>>,
    pub control_mode: ControlMode,
}

impl ControlCommand {
    /// Create a new position command
    pub fn position(timestamp: Duration, positions: Vec<f64>) -> Self {
        Self {
            timestamp,
            joint_positions: Some(positions),
            joint_velocities: None,
            joint_efforts: None,
            control_mode: ControlMode::Position,
        }
    }

    /// Create a new velocity command
    pub fn velocity(timestamp: Duration, velocities: Vec<f64>) -> Self {
        Self {
            timestamp,
            joint_positions: None,
            joint_velocities: Some(velocities),
            joint_efforts: None,
            control_mode: ControlMode::Velocity,
        }
    }

    /// Create a new effort command
    pub fn effort(timestamp: Duration, efforts: Vec<f64>) -> Self {
        Self {
            timestamp,
            joint_positions: None,
            joint_velocities: None,
            joint_efforts: Some(efforts),
            control_mode: ControlMode::Effort,
        }
    }

    /// Create an idle command
    pub fn idle(timestamp: Duration) -> Self {
        Self {
            timestamp,
            joint_positions: None,
            joint_velocities: None,
            joint_efforts: None,
            control_mode: ControlMode::Idle,
        }
    }

    /// Check if command is valid for given number of joints
    pub fn is_valid(&self, num_joints: usize) -> bool {
        match self.control_mode {
            ControlMode::Position => self
                .joint_positions
                .as_ref()
                .map(|p| p.len() == num_joints)
                .unwrap_or(false),
            ControlMode::Velocity => self
                .joint_velocities
                .as_ref()
                .map(|v| v.len() == num_joints)
                .unwrap_or(false),
            ControlMode::Effort => self
                .joint_efforts
                .as_ref()
                .map(|e| e.len() == num_joints)
                .unwrap_or(false),
            ControlMode::Idle => true,
            ControlMode::Impedance => self
                .joint_positions
                .as_ref()
                .map(|p| p.len() == num_joints)
                .unwrap_or(false),
        }
    }
}

/// Control state from robot
#[derive(Debug, Clone)]
pub struct ControlState {
    pub timestamp: Duration,
    pub actual_positions: Vec<f64>,
    pub actual_velocities: Vec<f64>,
    pub actual_efforts: Vec<f64>,
    pub is_valid: bool,
}

impl ControlState {
    /// Create new control state
    pub fn new(
        timestamp: Duration,
        positions: Vec<f64>,
        velocities: Vec<f64>,
        efforts: Vec<f64>,
    ) -> Self {
        Self {
            timestamp,
            actual_positions: positions,
            actual_velocities: velocities,
            actual_efforts: efforts,
            is_valid: true,
        }
    }

    /// Create from RobotState
    pub fn from_robot_state(state: &RobotState) -> Self {
        Self {
            timestamp: state.timestamp,
            actual_positions: state.joints.positions(),
            actual_velocities: state.joints.iter().map(|j| j.velocity).collect(),
            actual_efforts: state.joints.iter().map(|j| j.effort).collect(),
            is_valid: true,
        }
    }

    /// Create invalid state
    pub fn invalid() -> Self {
        Self {
            timestamp: Duration::from_secs(0),
            actual_positions: vec![],
            actual_velocities: vec![],
            actual_efforts: vec![],
            is_valid: false,
        }
    }

    /// Get number of joints
    pub fn num_joints(&self) -> usize {
        self.actual_positions.len()
    }
}

/// Controller errors
#[derive(Error, Debug)]
pub enum ControllerError {
    #[error("Invalid control state: {0}")]
    InvalidState(String),

    #[error("Safety violation: {0}")]
    SafetyViolation(String),

    #[error("Command buffer underrun")]
    BufferUnderrun,

    #[error("Timestamp validation failed: {0}")]
    TimestampError(String),

    #[error("Control mode mismatch: expected {expected}, got {actual}")]
    ModeMismatch {
        expected: ControlMode,
        actual: ControlMode,
    },
}

/// Updated Controller trait with safety integration
pub trait Controller: Send + Sync {
    /// Update control loop
    ///
    /// # Arguments
    /// * `state` - Current control state from robot
    /// * `dt` - Time step since last update
    ///
    /// # Returns
    /// The control command to send to the robot
    fn update(&mut self, state: &ControlState, dt: Duration) -> crate::Result<ControlCommand>;

    /// Set target command
    fn set_target(&mut self, target: ControlCommand);

    /// Check if controller is ready
    fn is_ready(&self) -> bool;

    /// Reset controller state
    fn reset(&mut self);

    /// Get current control mode
    fn control_mode(&self) -> ControlMode {
        ControlMode::Position
    }
}

/// Legacy Controller trait for backward compatibility
///
/// Deprecated: Use the new `Controller` trait instead
pub trait LegacyController {
    /// Update control loop
    fn update(&mut self, state: &RobotState, dt: Duration) -> crate::Result<JointSet>;

    /// Set target trajectory
    fn set_trajectory(&mut self, trajectory: Trajectory);

    /// Check if trajectory is complete
    fn is_complete(&self) -> bool;
}

/// Command buffer with latency handling
pub struct CommandBuffer {
    buffer: Vec<(Duration, ControlCommand)>,
    delay: Duration,
    max_size: usize,
}

impl CommandBuffer {
    pub fn new(delay: Duration, max_size: usize) -> Self {
        Self {
            buffer: Vec::with_capacity(max_size),
            delay,
            max_size,
        }
    }

    /// Add command to buffer
    pub fn push(&mut self, timestamp: Duration, command: ControlCommand) {
        if self.buffer.len() >= self.max_size {
            self.buffer.remove(0);
            warn!("Command buffer overflow, dropping oldest command");
        }
        self.buffer.push((timestamp, command));
    }

    /// Get command for current time (with delay compensation)
    pub fn get(&self, current_time: Duration) -> Option<&ControlCommand> {
        let target_time = current_time.saturating_sub(self.delay);

        // Find command closest to target time
        self.buffer
            .iter()
            .find(|(t, _)| *t >= target_time)
            .map(|(_, cmd)| cmd)
            .or_else(|| self.buffer.last().map(|(_, cmd)| cmd))
    }

    /// Extrapolate command for given time
    pub fn extrapolate(&self, current_time: Duration, num_joints: usize) -> Option<ControlCommand> {
        if self.buffer.len() < 2 {
            return self.get(current_time).cloned();
        }

        let target_time = current_time.saturating_sub(self.delay);

        // Find surrounding commands for interpolation
        let idx = self
            .buffer
            .iter()
            .position(|(t, _)| *t >= target_time)
            .unwrap_or(self.buffer.len() - 1);

        if idx == 0 {
            return self.buffer.get(0).map(|(_, cmd)| cmd.clone());
        }

        let (t1, cmd1) = &self.buffer[idx - 1];
        let (t2, cmd2) = &self.buffer[idx];

        let dt = t2.as_secs_f64() - t1.as_secs_f64();
        if dt <= 0.0 {
            return Some(cmd1.clone());
        }

        let alpha = (target_time.as_secs_f64() - t1.as_secs_f64()) / dt;

        // Interpolate positions
        let positions = if let (Some(ref p1), Some(ref p2)) =
            (cmd1.joint_positions.as_ref(), cmd2.joint_positions.as_ref())
        {
            if p1.len() == num_joints && p2.len() == num_joints {
                Some(
                    p1.iter()
                        .zip(p2.iter())
                        .map(|(a, b)| a + alpha * (b - a))
                        .collect(),
                )
            } else {
                cmd2.joint_positions.clone()
            }
        } else {
            cmd2.joint_positions.clone()
        };

        Some(ControlCommand {
            timestamp: target_time,
            joint_positions: positions,
            joint_velocities: cmd2.joint_velocities.clone(),
            joint_efforts: cmd2.joint_efforts.clone(),
            control_mode: cmd2.control_mode,
        })
    }

    /// Clear buffer
    pub fn clear(&mut self) {
        self.buffer.clear();
    }

    /// Get buffer size
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }
}

/// Trajectory waypoint
#[derive(Debug, Clone)]
pub struct Waypoint {
    pub time: Duration,
    pub joint_positions: Vec<f64>,
    pub base_pose: Option<Pose>,
}

/// Trajectory for robot motion
#[derive(Debug, Clone, Default)]
pub struct Trajectory {
    waypoints: Vec<Waypoint>,
}

impl Trajectory {
    pub fn new() -> Self {
        Self {
            waypoints: Vec::new(),
        }
    }

    pub fn add_waypoint(&mut self, waypoint: Waypoint) {
        self.waypoints.push(waypoint);
    }

    /// Interpolate trajectory at given time
    pub fn interpolate(&self, time: Duration) -> Option<Waypoint> {
        if self.waypoints.is_empty() {
            return None;
        }

        // Find surrounding waypoints
        let idx = self
            .waypoints
            .iter()
            .position(|w| w.time > time)
            .unwrap_or(self.waypoints.len());

        if idx == 0 {
            return Some(self.waypoints[0].clone());
        }

        if idx >= self.waypoints.len() {
            return Some(self.waypoints.last().unwrap().clone());
        }

        let w1 = &self.waypoints[idx - 1];
        let w2 = &self.waypoints[idx];

        // Linear interpolation
        let t1 = w1.time.as_secs_f64();
        let t2 = w2.time.as_secs_f64();
        let t = time.as_secs_f64();
        let alpha = (t - t1) / (t2 - t1);

        let joint_positions: Vec<f64> = w1
            .joint_positions
            .iter()
            .zip(w2.joint_positions.iter())
            .map(|(p1, p2)| p1 + alpha * (p2 - p1))
            .collect();

        Some(Waypoint {
            time,
            joint_positions,
            base_pose: None, // Could interpolate pose too
        })
    }

    pub fn duration(&self) -> Duration {
        self.waypoints
            .last()
            .map(|w| w.time)
            .unwrap_or(Duration::from_secs(0))
    }
}

/// Motion planner for generating trajectories
pub struct MotionPlanner {
    max_velocity: f64,
    max_acceleration: f64,
}

impl Default for MotionPlanner {
    fn default() -> Self {
        Self {
            max_velocity: 2.0,     // rad/s
            max_acceleration: 5.0, // rad/s²
        }
    }
}

impl MotionPlanner {
    pub fn new(max_velocity: f64, max_acceleration: f64) -> Self {
        Self {
            max_velocity,
            max_acceleration,
        }
    }

    /// Plan joint space trajectory from current to target
    pub fn plan_joint_space(
        &self,
        current: &JointSet,
        target: &JointSet,
        duration: Duration,
    ) -> Trajectory {
        let mut trajectory = Trajectory::new();

        // Start waypoint
        trajectory.add_waypoint(Waypoint {
            time: Duration::from_secs(0),
            joint_positions: current.positions(),
            base_pose: None,
        });

        // End waypoint
        trajectory.add_waypoint(Waypoint {
            time: duration,
            joint_positions: target.positions(),
            base_pose: None,
        });

        trajectory
    }

    /// Plan Cartesian trajectory for end effector
    pub fn plan_cartesian(&self, start: &Pose, end: &Pose, duration: Duration) -> Trajectory {
        let mut trajectory = Trajectory::new();

        // Simple linear interpolation in Cartesian space
        let steps = 20;
        let dt = duration.as_secs_f64() / steps as f64;

        for i in 0..=steps {
            let alpha = i as f64 / steps as f64;
            let t = Duration::from_secs_f64(i as f64 * dt);

            // Linear interpolation for position
            let position = start.position * (1.0 - alpha) + end.position * alpha;

            // Spherical linear interpolation for orientation
            let orientation = start.orientation.slerp(&end.orientation, alpha);

            trajectory.add_waypoint(Waypoint {
                time: t,
                joint_positions: vec![], // Would need IK to fill this
                base_pose: Some(Pose {
                    position,
                    orientation,
                }),
            });
        }

        trajectory
    }

    /// Plan standing up motion
    pub fn plan_stand_up(&self, current: &JointSet) -> Trajectory {
        let mut trajectory = Trajectory::new();
        let stand_pose = Self::standing_pose();

        trajectory.add_waypoint(Waypoint {
            time: Duration::from_secs(0),
            joint_positions: current.positions(),
            base_pose: None,
        });

        trajectory.add_waypoint(Waypoint {
            time: Duration::from_secs(2),
            joint_positions: stand_pose,
            base_pose: None,
        });

        trajectory
    }

    fn standing_pose() -> Vec<f64> {
        // Rough standing pose for G1
        let mut pose = vec![0.0; 29];
        // Slight knee bend
        pose[3] = 0.3; // left_knee
        pose[9] = 0.3; // right_knee
                       // Arms down
        pose[15] = 0.2; // left_shoulder_pitch
        pose[22] = 0.2; // right_shoulder_pitch
        pose
    }
}

/// Joint-space PD controller with safety integration
pub struct JointSpaceController {
    kp: f64, // Proportional gain
    kd: f64, // Derivative gain
    trajectory: Option<Trajectory>,
    current_time: Duration,
    target: JointSet,
    command_buffer: CommandBuffer,
    safety_manager: Option<SafetyManager>,
    last_target_cmd: Option<ControlCommand>,
    ready: bool,
}

impl JointSpaceController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self {
            kp,
            kd,
            trajectory: None,
            current_time: Duration::from_secs(0),
            target: JointSet::new(),
            command_buffer: CommandBuffer::new(Duration::from_millis(10), 100),
            safety_manager: None,
            last_target_cmd: None,
            ready: false,
        }
    }

    /// Create controller with safety manager
    pub fn with_safety(mut self, safety: SafetyManager) -> Self {
        self.safety_manager = Some(safety);
        self
    }

    /// Set command buffer delay
    pub fn set_buffer_delay(&mut self, delay: Duration) {
        self.command_buffer = CommandBuffer::new(delay, 100);
    }

    /// Check if safety manager is configured
    pub fn has_safety(&self) -> bool {
        self.safety_manager.is_some()
    }

    /// Get mutable reference to safety manager
    pub fn safety_manager_mut(&mut self) -> Option<&mut SafetyManager> {
        self.safety_manager.as_mut()
    }

    /// Set target trajectory (for backward compatibility with tests)
    pub fn set_trajectory(&mut self, trajectory: Trajectory) {
        self.trajectory = Some(trajectory);
        self.current_time = Duration::from_secs(0);
    }

    /// Check if trajectory is complete (for backward compatibility with tests)
    pub fn is_complete(&self) -> bool {
        match &self.trajectory {
            None => true,
            Some(traj) => self.current_time >= traj.duration(),
        }
    }
}

impl Controller for JointSpaceController {
    fn update(&mut self, state: &ControlState, dt: Duration) -> crate::Result<ControlCommand> {
        self.current_time += dt;

        // Get target from trajectory or hold current
        let target_positions = if let Some(ref traj) = self.trajectory {
            traj.interpolate(self.current_time)
                .map(|w| w.joint_positions)
                .unwrap_or_else(|| self.target.positions())
        } else {
            self.target.positions()
        };

        if !state.is_valid {
            return Err(crate::HumanoidError::ControlError(
                "Invalid control state".to_string(),
            ));
        }

        // PD control
        let num_joints = state.num_joints();
        let mut output_positions = Vec::with_capacity(num_joints);
        let mut output_velocities = Vec::with_capacity(num_joints);
        let mut output_efforts = Vec::with_capacity(num_joints);

        for i in 0..num_joints {
            let current_pos = state.actual_positions.get(i).copied().unwrap_or(0.0);
            let current_vel = state.actual_velocities.get(i).copied().unwrap_or(0.0);
            let target_pos = target_positions.get(i).copied().unwrap_or(current_pos);

            let error = target_pos - current_pos;
            let d_error = -current_vel; // Simplified

            let effort = self.kp * error + self.kd * d_error;

            output_positions.push(target_pos);
            output_velocities.push(0.0);
            output_efforts.push(effort);
        }

        // Create command
        let mut command = ControlCommand {
            timestamp: self.current_time,
            joint_positions: Some(output_positions.clone()),
            joint_velocities: Some(output_velocities),
            joint_efforts: Some(output_efforts),
            control_mode: ControlMode::Position,
        };

        // Apply safety checks
        if let Some(ref mut safety) = self.safety_manager {
            match safety.check_command(
                output_positions.as_slice(),
                command.joint_velocities.as_ref().unwrap().as_slice(),
                command.joint_efforts.as_ref().unwrap().as_slice(),
                ControlMode::Position,
                self.current_time,
            ) {
                Ok((p, v, e)) => {
                    command.joint_positions = Some(p);
                    command.joint_velocities = Some(v);
                    command.joint_efforts = Some(e);
                }
                Err(e) => {
                    return Err(crate::HumanoidError::SafetyStop {
                        reason: format!("Safety check failed: {}", e),
                    });
                }
            }
        }

        self.ready = true;
        Ok(command)
    }

    fn set_target(&mut self, target: ControlCommand) {
        self.last_target_cmd = Some(target.clone());
        if let Some(ref positions) = target.joint_positions {
            self.target.set_positions(positions);
        }
    }

    fn is_ready(&self) -> bool {
        self.ready
    }

    fn reset(&mut self) {
        self.current_time = Duration::from_secs(0);
        self.trajectory = None;
        self.command_buffer.clear();
        self.ready = false;
        if let Some(ref mut safety) = self.safety_manager {
            safety.reset();
        }
    }
}

/// Adapter to use new Controller trait as LegacyController
pub struct ControllerAdapter<T: Controller> {
    controller: T,
    state_converter: Box<dyn Fn(&RobotState) -> ControlState + Send + Sync>,
}

impl<T: Controller> ControllerAdapter<T> {
    pub fn new<F>(controller: T, converter: F) -> Self
    where
        F: Fn(&RobotState) -> ControlState + Send + Sync + 'static,
    {
        Self {
            controller,
            state_converter: Box::new(converter),
        }
    }
}

impl<T: Controller> LegacyController for ControllerAdapter<T> {
    fn update(&mut self, state: &RobotState, dt: Duration) -> crate::Result<JointSet> {
        let control_state = (self.state_converter)(state);
        let command = self.controller.update(&control_state, dt)?;

        // Convert command back to JointSet
        let mut joint_set = JointSet::new();
        if let Some(ref positions) = command.joint_positions {
            for (i, &pos) in positions.iter().enumerate() {
                use crate::robot::{Joint, JointLimits, JointType};
                use nalgebra::Vector3;

                joint_set.add_joint(Joint {
                    name: format!("joint_{}", i),
                    joint_type: JointType::Revolute {
                        axis: *Vector3::y_axis(),
                    },
                    parent: None,
                    limits: JointLimits::default(),
                    current_position: pos,
                    target_position: pos,
                    velocity: command
                        .joint_velocities
                        .as_ref()
                        .and_then(|v| v.get(i).copied())
                        .unwrap_or(0.0),
                    effort: command
                        .joint_efforts
                        .as_ref()
                        .and_then(|e| e.get(i).copied())
                        .unwrap_or(0.0),
                });
            }
        }

        Ok(joint_set)
    }

    fn set_trajectory(&mut self, trajectory: Trajectory) {
        // Store trajectory and convert waypoints to ControlCommands
        // This is a simplified implementation
        debug!(
            "Setting trajectory with {} waypoints",
            trajectory.waypoints.len()
        );
    }

    fn is_complete(&self) -> bool {
        true // Simplified
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_control_command_validation() {
        let cmd = ControlCommand::position(Duration::from_secs(0), vec![0.0; 10]);
        assert!(cmd.is_valid(10));
        assert!(!cmd.is_valid(5));
    }

    #[test]
    fn test_control_state_from_robot() {
        let robot_state = RobotState::default();
        let control_state = ControlState::from_robot_state(&robot_state);
        assert!(control_state.is_valid);
    }

    #[test]
    fn test_command_buffer() {
        let mut buffer = CommandBuffer::new(Duration::from_millis(50), 10);

        let cmd = ControlCommand::position(Duration::from_secs(0), vec![1.0, 2.0]);
        buffer.push(Duration::from_millis(0), cmd.clone());

        // With 50ms delay, at t=0 we should still get the command
        let result = buffer.get(Duration::from_millis(0));
        assert!(result.is_some());

        // At t=100ms with 50ms delay, we should extrapolate
        let result = buffer.extrapolate(Duration::from_millis(100), 2);
        assert!(result.is_some());
    }

    #[test]
    fn test_joint_space_controller() {
        let mut controller = JointSpaceController::new(10.0, 1.0);

        let state = ControlState::new(
            Duration::from_secs(0),
            vec![0.0; 5],
            vec![0.0; 5],
            vec![0.0; 5],
        );

        controller.set_target(ControlCommand::position(
            Duration::from_secs(0),
            vec![1.0; 5],
        ));

        let result = controller.update(&state, Duration::from_millis(10));
        assert!(result.is_ok());

        let cmd = result.unwrap();
        assert!(cmd.is_valid(5));
    }

    #[test]
    fn test_controller_with_safety() {
        let limits = vec![safety::JointLimits::new(-1.0, 1.0, 5.0, 10.0)];

        let safety = SafetyManager::new().add_guard(safety::JointLimitGuard::new(&limits));

        let mut controller = JointSpaceController::new(10.0, 1.0).with_safety(safety);

        let state = ControlState::new(Duration::from_secs(0), vec![0.0], vec![0.0], vec![0.0]);

        let result = controller.update(&state, Duration::from_millis(10));
        assert!(result.is_ok());
    }
}
