//! Control safety and guardrails
//!
//! Provides safety mechanisms for robot control including:
//! - Joint limit enforcement
//! - Emergency stop handling
//! - Communication monitoring
//! - Torque rate limiting

use std::time::{Duration, Instant};
use thiserror::Error;
use tracing::{debug, error, info, warn};

/// Safety-related errors
#[derive(Error, Debug, Clone)]
pub enum SafetyError {
    #[error("Joint limit exceeded: joint {joint_index}, value {value}, limits [{min}, {max}]")]
    JointLimitExceeded {
        joint_index: usize,
        value: f64,
        min: f64,
        max: f64,
    },

    #[error("Velocity limit exceeded: joint {joint_index}, value {value}, max {max}")]
    VelocityLimitExceeded {
        joint_index: usize,
        value: f64,
        max: f64,
    },

    #[error("Effort limit exceeded: joint {joint_index}, value {value}, max {max}")]
    EffortLimitExceeded {
        joint_index: usize,
        value: f64,
        max: f64,
    },

    #[error("Emergency stop triggered: {reason}")]
    EmergencyStop { reason: String },

    #[error("Communication timeout: no command received for {elapsed:?}")]
    CommunicationTimeout { elapsed: Duration },

    #[error("Torque ramp rate exceeded: {rate} Nm/s, max {max_rate} Nm/s")]
    TorqueRampExceeded { rate: f64, max_rate: f64 },

    #[error("Invalid control mode transition from {from:?} to {to:?}")]
    InvalidModeTransition { from: ControlMode, to: ControlMode },

    #[error("Guard chain violated: {guard}")]
    GuardViolation { guard: String },
}

/// Control mode enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ControlMode {
    Idle,
    Position,
    Velocity,
    Effort,
    Impedance,
}

impl std::fmt::Display for ControlMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ControlMode::Idle => write!(f, "Idle"),
            ControlMode::Position => write!(f, "Position"),
            ControlMode::Velocity => write!(f, "Velocity"),
            ControlMode::Effort => write!(f, "Effort"),
            ControlMode::Impedance => write!(f, "Impedance"),
        }
    }
}

/// Joint limits for safety checking
#[derive(Debug, Clone)]
pub struct JointLimits {
    pub min_position: f64,
    pub max_position: f64,
    pub max_velocity: f64,
    pub max_effort: f64,
}

impl JointLimits {
    pub fn new(min_position: f64, max_position: f64, max_velocity: f64, max_effort: f64) -> Self {
        Self {
            min_position,
            max_position,
            max_velocity,
            max_effort,
        }
    }
}

/// Trait for safety guards
pub trait SafetyGuard: Send + Sync + AsAny {
    /// Check and optionally modify a command
    fn check_command(
        &mut self,
        positions: &[f64],
        velocities: &[f64],
        efforts: &[f64],
        mode: ControlMode,
        timestamp: Duration,
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), SafetyError>;

    /// Get guard name
    fn name(&self) -> &str;

    /// Reset guard state
    fn reset(&mut self);

    /// Check if guard is active/enabled
    fn is_active(&self) -> bool;
}

/// Guard that enforces joint position, velocity, and effort limits
pub struct JointLimitGuard {
    limits: Vec<JointLimits>,
    name: String,
    active: bool,
}

impl JointLimitGuard {
    pub fn new(limits: &[JointLimits]) -> Self {
        Self {
            limits: limits.to_vec(),
            name: "JointLimitGuard".to_string(),
            active: true,
        }
    }

    /// Create with default symmetric limits for n joints
    pub fn new_symmetric(n: usize, pos_limit: f64, vel_limit: f64, effort_limit: f64) -> Self {
        let limits = (0..n)
            .map(|_| JointLimits::new(-pos_limit, pos_limit, vel_limit, effort_limit))
            .collect();
        Self {
            limits,
            name: "JointLimitGuard".to_string(),
            active: true,
        }
    }
}

impl SafetyGuard for JointLimitGuard {
    fn check_command(
        &mut self,
        positions: &[f64],
        velocities: &[f64],
        efforts: &[f64],
        _mode: ControlMode,
        _timestamp: Duration,
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), SafetyError> {
        if !self.active {
            return Ok((positions.to_vec(), velocities.to_vec(), efforts.to_vec()));
        }

        let mut clamped_positions = positions.to_vec();
        let mut clamped_velocities = velocities.to_vec();
        let mut clamped_efforts = efforts.to_vec();

        for (i, limit) in self.limits.iter().enumerate() {
            // Check position limits
            if i < positions.len() {
                let pos = positions[i];
                if pos < limit.min_position || pos > limit.max_position {
                    warn!(
                        "Joint {} position {} outside limits [{}, {}], clamping",
                        i, pos, limit.min_position, limit.max_position
                    );
                    clamped_positions[i] = pos.clamp(limit.min_position, limit.max_position);
                }
            }

            // Check velocity limits
            if i < velocities.len() {
                let vel = velocities[i].abs();
                if vel > limit.max_velocity {
                    warn!(
                        "Joint {} velocity {} exceeds limit {}, clamping",
                        i, velocities[i], limit.max_velocity
                    );
                    clamped_velocities[i] = velocities[i].signum() * limit.max_velocity;
                }
            }

            // Check effort limits
            if i < efforts.len() {
                let effort = efforts[i].abs();
                if effort > limit.max_effort {
                    warn!(
                        "Joint {} effort {} exceeds limit {}, clamping",
                        i, efforts[i], limit.max_effort
                    );
                    clamped_efforts[i] = efforts[i].signum() * limit.max_effort;
                }
            }
        }

        Ok((clamped_positions, clamped_velocities, clamped_efforts))
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn reset(&mut self) {
        // No state to reset
    }

    fn is_active(&self) -> bool {
        self.active
    }
}

/// Guard for emergency stop handling
pub struct EStopGuard {
    estopped: bool,
    name: String,
    active: bool,
    stop_reason: Option<String>,
}

impl EStopGuard {
    pub fn new() -> Self {
        Self {
            estopped: false,
            name: "EStopGuard".to_string(),
            active: true,
            stop_reason: None,
        }
    }

    /// Trigger emergency stop
    pub fn trigger(&mut self, reason: impl Into<String>) {
        if !self.estopped {
            let reason_str: String = reason.into();
            error!("EMERGENCY STOP TRIGGERED: {}", &reason_str);
            self.estopped = true;
            self.stop_reason = Some(reason_str);
        }
    }

    /// Reset emergency stop
    pub fn reset(&mut self) {
        if self.estopped {
            info!("Emergency stop reset");
            self.estopped = false;
            self.stop_reason = None;
        }
    }

    /// Check if emergency stop is active
    pub fn is_estopped(&self) -> bool {
        self.estopped
    }
}

impl SafetyGuard for EStopGuard {
    fn check_command(
        &mut self,
        _positions: &[f64],
        _velocities: &[f64],
        _efforts: &[f64],
        _mode: ControlMode,
        _timestamp: Duration,
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), SafetyError> {
        if self.estopped {
            return Err(SafetyError::EmergencyStop {
                reason: self
                    .stop_reason
                    .clone()
                    .unwrap_or_else(|| "Unknown".to_string()),
            });
        }
        Ok((vec![], vec![], vec![]))
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn reset(&mut self) {
        if self.estopped {
            info!("Emergency stop reset");
            self.estopped = false;
            self.stop_reason = None;
        }
    }

    fn is_active(&self) -> bool {
        self.active
    }
}

impl Default for EStopGuard {
    fn default() -> Self {
        Self::new()
    }
}

/// Guard that monitors communication health
pub struct CommunicationMonitor {
    timeout: Duration,
    last_command_time: Option<Instant>,
    name: String,
    active: bool,
}

impl CommunicationMonitor {
    pub fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            last_command_time: None,
            name: "CommunicationMonitor".to_string(),
            active: true,
        }
    }

    /// Record that a command was received
    pub fn record_command(&mut self) {
        self.last_command_time = Some(Instant::now());
        debug!("Communication monitor: command recorded");
    }

    /// Check if communication is healthy
    pub fn is_healthy(&self) -> bool {
        match self.last_command_time {
            None => true, // No commands yet, assume healthy
            Some(last) => last.elapsed() < self.timeout,
        }
    }

    /// Get time since last command
    pub fn time_since_last(&self) -> Option<Duration> {
        self.last_command_time.map(|t| t.elapsed())
    }
}

impl SafetyGuard for CommunicationMonitor {
    fn check_command(
        &mut self,
        positions: &[f64],
        velocities: &[f64],
        efforts: &[f64],
        _mode: ControlMode,
        _timestamp: Duration,
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), SafetyError> {
        if !self.active {
            return Ok((positions.to_vec(), velocities.to_vec(), efforts.to_vec()));
        }

        if let Some(last) = self.last_command_time {
            let elapsed = last.elapsed();
            if elapsed > self.timeout {
                error!(
                    "Communication timeout: no command for {:?} (timeout: {:?})",
                    elapsed, self.timeout
                );
                return Err(SafetyError::CommunicationTimeout { elapsed });
            }
        }

        self.record_command();
        Ok((positions.to_vec(), velocities.to_vec(), efforts.to_vec()))
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn reset(&mut self) {
        self.last_command_time = None;
    }

    fn is_active(&self) -> bool {
        self.active
    }
}

/// Guard that limits torque ramp rates
pub struct TorqueRampGuard {
    max_rate: f64, // Nm/s
    last_efforts: Option<Vec<f64>>,
    last_time: Option<Instant>,
    name: String,
    active: bool,
}

impl TorqueRampGuard {
    pub fn new(max_rate: f64) -> Self {
        Self {
            max_rate,
            last_efforts: None,
            last_time: None,
            name: "TorqueRampGuard".to_string(),
            active: true,
        }
    }

    /// Set maximum rate in Nm/s
    pub fn set_max_rate(&mut self, rate: f64) {
        self.max_rate = rate;
    }
}

impl SafetyGuard for TorqueRampGuard {
    fn check_command(
        &mut self,
        positions: &[f64],
        velocities: &[f64],
        efforts: &[f64],
        _mode: ControlMode,
        _timestamp: Duration,
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), SafetyError> {
        if !self.active {
            return Ok((positions.to_vec(), velocities.to_vec(), efforts.to_vec()));
        }

        let mut ramped_efforts = efforts.to_vec();

        if let (Some(ref last), Some(last_time)) = (&self.last_efforts, self.last_time) {
            let dt = last_time.elapsed().as_secs_f64();
            if dt > 0.0 {
                for (i, (&current, &last)) in efforts.iter().zip(last.iter()).enumerate() {
                    let delta = current - last;
                    let rate = delta.abs() / dt;

                    if rate > self.max_rate {
                        warn!(
                            "Torque ramp rate {} Nm/s exceeds limit {} Nm/s for joint {}",
                            rate, self.max_rate, i
                        );
                        let max_delta = self.max_rate * dt;
                        ramped_efforts[i] = last + delta.signum() * max_delta;
                    }
                }
            }
        }

        self.last_efforts = Some(ramped_efforts.clone());
        self.last_time = Some(Instant::now());

        Ok((positions.to_vec(), velocities.to_vec(), ramped_efforts))
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn reset(&mut self) {
        self.last_efforts = None;
        self.last_time = None;
    }

    fn is_active(&self) -> bool {
        self.active
    }
}

/// Safety level for configuring guard behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyLevel {
    /// Simulation - relaxed limits
    Simulation,
    /// Real robot with safety margins
    Production,
    /// Critical operation - strictest limits
    Critical,
}

/// Manager for chaining multiple safety guards
pub struct SafetyManager {
    guards: Vec<Box<dyn SafetyGuard>>,
    level: SafetyLevel,
    estop_guard_idx: Option<usize>,
}

impl SafetyManager {
    pub fn new() -> Self {
        Self {
            guards: Vec::new(),
            level: SafetyLevel::Production,
            estop_guard_idx: None,
        }
    }

    /// Create manager with specific safety level
    pub fn with_level(level: SafetyLevel) -> Self {
        Self {
            guards: Vec::new(),
            level,
            estop_guard_idx: None,
        }
    }

    /// Add a guard to the chain
    pub fn add_guard<G: SafetyGuard + 'static>(mut self, guard: G) -> Self {
        if guard.name() == "EStopGuard" {
            self.estop_guard_idx = Some(self.guards.len());
        }
        self.guards.push(Box::new(guard));
        self
    }

    /// Check a command through all guards
    pub fn check_command(
        &mut self,
        positions: &[f64],
        velocities: &[f64],
        efforts: &[f64],
        mode: ControlMode,
        timestamp: Duration,
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), SafetyError> {
        let mut current_positions = positions.to_vec();
        let mut current_velocities = velocities.to_vec();
        let mut current_efforts = efforts.to_vec();

        for guard in &mut self.guards {
            match guard.check_command(
                &current_positions,
                &current_velocities,
                &current_efforts,
                mode,
                timestamp,
            ) {
                Ok((p, v, e)) => {
                    current_positions = p;
                    current_velocities = v;
                    current_efforts = e;
                }
                Err(e) => {
                    error!("Safety guard '{}' rejected command: {}", guard.name(), e);
                    return Err(e);
                }
            }
        }

        Ok((current_positions, current_velocities, current_efforts))
    }

    /// Trigger emergency stop
    pub fn trigger_estop(&mut self, reason: impl Into<String>) {
        if let Some(idx) = self.estop_guard_idx {
            if let Some(guard) = self.guards.get_mut(idx) {
                if let Some(estop) = guard.as_any_mut().downcast_mut::<EStopGuard>() {
                    estop.trigger(reason);
                }
            }
        }
    }

    /// Reset emergency stop
    pub fn reset_estop(&mut self) {
        if let Some(idx) = self.estop_guard_idx {
            if let Some(guard) = self.guards.get_mut(idx) {
                if let Some(estop) = guard.as_any_mut().downcast_mut::<EStopGuard>() {
                    estop.reset();
                }
            }
        }
    }

    /// Check if emergency stop is active
    pub fn is_estopped(&self) -> bool {
        if let Some(idx) = self.estop_guard_idx {
            if let Some(guard) = self.guards.get(idx) {
                if let Some(estop) = guard.as_any().downcast_ref::<EStopGuard>() {
                    return estop.is_estopped();
                }
            }
        }
        false
    }

    /// Get safety level
    pub fn level(&self) -> SafetyLevel {
        self.level
    }

    /// Set safety level
    pub fn set_level(&mut self, level: SafetyLevel) {
        self.level = level;
    }

    /// Reset all guards
    pub fn reset(&mut self) {
        for guard in &mut self.guards {
            guard.reset();
        }
    }

    /// Get number of guards
    pub fn num_guards(&self) -> usize {
        self.guards.len()
    }
}

impl Default for SafetyManager {
    fn default() -> Self {
        Self::new()
    }
}

// Helper trait for downcasting
pub trait AsAny {
    fn as_any(&self) -> &dyn std::any::Any;
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

impl<T: 'static> AsAny for T {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joint_limit_guard_clamping() {
        let limits = vec![
            JointLimits::new(-1.0, 1.0, 5.0, 10.0),
            JointLimits::new(-2.0, 2.0, 3.0, 15.0),
        ];

        let mut guard = JointLimitGuard::new(&limits);

        // Test position clamping
        let positions = vec![1.5, -2.5]; // Outside limits
        let velocities = vec![0.0, 0.0];
        let efforts = vec![0.0, 0.0];

        let result = guard.check_command(
            &positions,
            &velocities,
            &efforts,
            ControlMode::Position,
            Duration::from_secs(0),
        );

        assert!(result.is_ok());
        let (p, _, _) = result.unwrap();
        assert_eq!(p[0], 1.0); // Clamped to max
        assert_eq!(p[1], -2.0); // Clamped to min
    }

    #[test]
    fn test_estop_guard() {
        let mut guard = EStopGuard::new();
        assert!(!guard.is_estopped());

        guard.trigger("Test stop");
        assert!(guard.is_estopped());

        let result =
            guard.check_command(&[], &[], &[], ControlMode::Position, Duration::from_secs(0));

        assert!(matches!(result, Err(SafetyError::EmergencyStop { .. })));

        guard.reset();
        assert!(!guard.is_estopped());
    }

    #[test]
    fn test_communication_monitor() {
        let mut guard = CommunicationMonitor::new(Duration::from_millis(100));

        // First command should succeed
        let result =
            guard.check_command(&[], &[], &[], ControlMode::Position, Duration::from_secs(0));
        assert!(result.is_ok());

        // Should still be healthy immediately after
        assert!(guard.is_healthy());
    }

    #[test]
    fn test_torque_ramp_guard() {
        let mut guard = TorqueRampGuard::new(100.0); // 100 Nm/s limit

        // First command sets baseline
        let result = guard.check_command(
            &[],
            &[],
            &vec![0.0],
            ControlMode::Effort,
            Duration::from_secs(0),
        );
        assert!(result.is_ok());

        // Small change should be fine
        std::thread::sleep(Duration::from_millis(10));
        let result = guard.check_command(
            &[],
            &[],
            &vec![1.0], // 1 Nm change in ~10ms = 100 Nm/s
            ControlMode::Effort,
            Duration::from_millis(0),
        );
        assert!(result.is_ok());
    }

    #[test]
    fn test_safety_manager_chain() {
        let limits = vec![JointLimits::new(-1.0, 1.0, 5.0, 10.0)];

        let mut manager = SafetyManager::new()
            .add_guard(JointLimitGuard::new(&limits))
            .add_guard(EStopGuard::new());

        assert_eq!(manager.num_guards(), 2);

        // Normal command should pass
        let result = manager.check_command(
            &vec![0.5],
            &vec![0.0],
            &vec![0.0],
            ControlMode::Position,
            Duration::from_secs(0),
        );
        assert!(result.is_ok());
    }
}
