//! Common test utilities for humanoid-rs
//!
//! This module provides shared test fixtures, mock implementations,
//! and helper functions for deterministic testing.

use humanoid_rs::control::{Controller, JointSpaceController, Trajectory, Waypoint};
use humanoid_rs::ik::{IkSolver, IkSolverConfig};
use humanoid_rs::mpc::{LipmState, MpcConfig, MpcController, ZmpReference};
use humanoid_rs::robot::{unitree_g1, Joint, JointLimits, JointSet, JointType, Pose, RobotState};
use nalgebra::{UnitQuaternion, Vector3};
use std::time::Duration;

/// Seed for deterministic random number generation
pub const TEST_SEED: u64 = 42;

/// Default timestep for simulation tests
pub const DEFAULT_TIMESTEP: Duration = Duration::from_millis(1);

/// Maximum allowed difference for floating-point comparisons
pub const FLOAT_EPSILON: f64 = 1e-9;

/// Maximum allowed difference for physics state comparisons
pub const PHYSICS_EPSILON: f64 = 1e-6;

/// Creates a deterministic test simulation with a fixed seed
pub fn create_test_sim() -> TestSimulation {
    TestSimulation::new(TEST_SEED)
}

/// Creates a test simulation with a specific seed
pub fn create_test_sim_with_seed(seed: u64) -> TestSimulation {
    TestSimulation::new(seed)
}

/// Test simulation wrapper for deterministic testing
pub struct TestSimulation {
    seed: u64,
    state: RobotState,
    timestep: Duration,
    step_count: u64,
    sensor_data: SensorData,
}

/// Simulated sensor readings
#[derive(Debug, Clone, PartialEq)]
pub struct SensorData {
    pub joint_positions: Vec<f64>,
    pub joint_velocities: Vec<f64>,
    pub joint_torques: Vec<f64>,
    pub imu_acceleration: Vector3<f64>,
    pub imu_gyroscope: Vector3<f64>,
    pub foot_force_left: f64,
    pub foot_force_right: f64,
}

impl Default for SensorData {
    fn default() -> Self {
        Self {
            joint_positions: vec![0.0; unitree_g1::NUM_JOINTS],
            joint_velocities: vec![0.0; unitree_g1::NUM_JOINTS],
            joint_torques: vec![0.0; unitree_g1::NUM_JOINTS],
            imu_acceleration: Vector3::new(0.0, -9.81, 0.0),
            imu_gyroscope: Vector3::zeros(),
            foot_force_left: 0.0,
            foot_force_right: 0.0,
        }
    }
}

impl TestSimulation {
    pub fn new(seed: u64) -> Self {
        let mut state = RobotState::default();
        state.joints = unitree_g1::create_default_joints();

        Self {
            seed,
            state,
            timestep: DEFAULT_TIMESTEP,
            step_count: 0,
            sensor_data: SensorData::default(),
        }
    }

    /// Advance simulation by one timestep
    pub fn step(&mut self, dt: Duration) {
        self.step_count += 1;
        self.state.timestamp += dt;

        // Deterministic physics update based on seed and step count
        let deterministic_noise = self.compute_deterministic_noise();

        // Update joint positions with simple integration
        for (i, joint) in self.state.joints.iter_mut().enumerate() {
            let target = joint.target_position;
            let error = target - joint.current_position;
            let velocity = error * 10.0; // Simple P control
            joint.velocity = velocity + deterministic_noise[i % deterministic_noise.len()];
            joint.current_position += joint.velocity * dt.as_secs_f64();
            joint.effort = error * 100.0; // Simple P control effort
        }

        // Update sensor data
        self.update_sensors();
    }

    /// Run simulation for multiple steps
    pub fn run_steps(&mut self, steps: usize) {
        for _ in 0..steps {
            self.step(self.timestep);
        }
    }

    /// Get current simulation state
    pub fn state(&self) -> &RobotState {
        &self.state
    }

    /// Get sensor data
    pub fn sensor_data(&self) -> &SensorData {
        &self.sensor_data
    }

    /// Set target positions for all joints
    pub fn set_target_positions(&mut self, positions: &[f64]) {
        for (joint, &pos) in self.state.joints.iter_mut().zip(positions.iter()) {
            joint.target_position = pos;
        }
    }

    /// Apply deterministic perturbation for testing robustness
    pub fn apply_perturbation(&mut self, magnitude: f64) {
        let perturbation = self.compute_deterministic_noise();
        for (joint, &pert) in self.state.joints.iter_mut().zip(perturbation.iter()) {
            joint.current_position += pert * magnitude;
        }
    }

    fn compute_deterministic_noise(&self) -> Vec<f64> {
        // Simple deterministic pseudo-random based on seed and step count
        let mut noise = Vec::with_capacity(unitree_g1::NUM_JOINTS);
        for i in 0..unitree_g1::NUM_JOINTS {
            let hash = self.seed.wrapping_add(self.step_count * 1000 + i as u64);
            let normalized = (hash % 1000) as f64 / 1000.0;
            noise.push((normalized - 0.5) * 2.0); // Range: [-1, 1]
        }
        noise
    }

    fn update_sensors(&mut self) {
        self.sensor_data.joint_positions = self.state.joints.positions();
        self.sensor_data.joint_velocities = self.state.joints.iter().map(|j| j.velocity).collect();
        self.sensor_data.joint_torques = self.state.joints.iter().map(|j| j.effort).collect();
    }
}

/// Creates a mock robot with default configuration
pub fn create_mock_robot() -> MockRobot {
    MockRobot::new()
}

/// Mock robot implementation for testing
pub struct MockRobot {
    state: RobotState,
    joint_limits: Vec<JointLimits>,
    emergency_stop_active: bool,
    command_history: Vec<Vec<f64>>,
}

impl MockRobot {
    pub fn new() -> Self {
        let mut state = RobotState::default();
        state.joints = unitree_g1::create_default_joints();

        let joint_limits: Vec<JointLimits> = (0..unitree_g1::NUM_JOINTS)
            .map(|_| JointLimits::default())
            .collect();

        Self {
            state,
            joint_limits,
            emergency_stop_active: false,
            command_history: Vec::new(),
        }
    }

    pub fn state(&self) -> &RobotState {
        &self.state
    }

    pub fn set_joint_positions(&mut self, positions: &[f64]) -> Result<(), String> {
        if self.emergency_stop_active {
            return Err("Emergency stop active".to_string());
        }

        if positions.len() != self.state.joints.iter().count() {
            return Err("Position count mismatch".to_string());
        }

        // Apply joint limits
        for (i, (joint, &pos)) in self
            .state
            .joints
            .iter_mut()
            .zip(positions.iter())
            .enumerate()
        {
            let limits = &self.joint_limits[i];
            if pos < limits.min || pos > limits.max {
                return Err(format!(
                    "Joint {} position {} exceeds limits [{}, {}]",
                    joint.name, pos, limits.min, limits.max
                ));
            }
            joint.target_position = pos;
        }

        self.command_history.push(positions.to_vec());
        Ok(())
    }

    pub fn emergency_stop(&mut self) {
        self.emergency_stop_active = true;
        // Zero all commands
        for joint in self.state.joints.iter_mut() {
            joint.target_position = joint.current_position;
            joint.velocity = 0.0;
            joint.effort = 0.0;
        }
    }

    pub fn reset_emergency_stop(&mut self) {
        self.emergency_stop_active = false;
    }

    pub fn is_emergency_stop_active(&self) -> bool {
        self.emergency_stop_active
    }

    pub fn command_history(&self) -> &[Vec<f64>] {
        &self.command_history
    }
}

/// Creates a test trajectory for validation
pub fn create_test_trajectory(duration_secs: f64) -> Trajectory {
    let mut trajectory = Trajectory::new();
    let steps = 10;
    let dt = duration_secs / steps as f64;

    for i in 0..=steps {
        let t = Duration::from_secs_f64(i as f64 * dt);
        let progress = i as f64 / steps as f64;

        // Simple trajectory: move all joints from 0 to 0.5 and back
        let target_pos = if progress < 0.5 {
            progress * 2.0 * 0.5
        } else {
            (1.0 - progress) * 2.0 * 0.5
        };

        let positions = vec![target_pos; unitree_g1::NUM_JOINTS];

        trajectory.add_waypoint(Waypoint {
            time: t,
            joint_positions: positions,
            base_pose: None,
        });
    }

    trajectory
}

/// Creates a PD controller with specified gains for testing
pub fn create_pd_controller(kp: f64, kd: f64) -> JointSpaceController {
    JointSpaceController::new(kp, kd)
}

/// Creates a standard test controller (kp=50, kd=5)
pub fn create_test_controller() -> JointSpaceController {
    create_pd_controller(50.0, 5.0)
}

/// State comparison helpers
pub fn assert_states_equal(sim1: &TestSimulation, sim2: &TestSimulation) {
    let state1 = sim1.state();
    let state2 = sim2.state();

    assert_eq!(
        state1.timestamp, state2.timestamp,
        "Timestamps should match"
    );

    let pos1 = state1.joints.positions();
    let pos2 = state2.joints.positions();

    assert_eq!(pos1.len(), pos2.len(), "Joint count should match");

    for (i, (p1, p2)) in pos1.iter().zip(pos2.iter()).enumerate() {
        assert!(
            (p1 - p2).abs() < FLOAT_EPSILON,
            "Joint {} position mismatch: {} vs {}",
            i,
            p1,
            p2
        );
    }
}

/// Compare robot states with tolerance
pub fn assert_robot_states_approx_equal(state1: &RobotState, state2: &RobotState, epsilon: f64) {
    let pos1 = state1.joints.positions();
    let pos2 = state2.joints.positions();

    assert_eq!(pos1.len(), pos2.len(), "Joint count should match");

    for (i, (p1, p2)) in pos1.iter().zip(pos2.iter()).enumerate() {
        assert!(
            (p1 - p2).abs() < epsilon,
            "Joint {} position mismatch: {} vs {} (epsilon: {})",
            i,
            p1,
            p2,
            epsilon
        );
    }
}

/// Compare sensor data
pub fn assert_sensor_data_equal(data1: &SensorData, data2: &SensorData) {
    assert_eq!(data1.joint_positions.len(), data2.joint_positions.len());

    for (i, (p1, p2)) in data1
        .joint_positions
        .iter()
        .zip(data2.joint_positions.iter())
        .enumerate()
    {
        assert!(
            (p1 - p2).abs() < FLOAT_EPSILON,
            "Joint position {} mismatch: {} vs {}",
            i,
            p1,
            p2
        );
    }
}

/// Creates an IK solver with default configuration
pub fn create_test_ik_solver() -> IkSolver {
    IkSolver::with_default_config()
}

/// Creates an MPC controller for testing
pub fn create_test_mpc_controller() -> MpcController {
    let config = MpcConfig::default();
    MpcController::new(config)
}

/// Creates a standing ZMP reference
pub fn create_standing_reference(duration: f64) -> ZmpReference {
    ZmpReference::standing(duration, 0.02, Vector3::zeros())
}

/// Golden file helpers
pub fn save_golden_state(test_name: &str, state: &RobotState) -> Result<(), std::io::Error> {
    let path = format!("tests/golden/{}.json", test_name);
    let json = serde_json::to_string_pretty(state)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
    std::fs::write(&path, json)
}

pub fn load_golden_state(test_name: &str) -> Result<RobotState, Box<dyn std::error::Error>> {
    let path = format!("tests/golden/{}.json", test_name);
    let json = std::fs::read_to_string(&path)?;
    let state = serde_json::from_str(&json)?;
    Ok(state)
}

/// Check if golden file exists
pub fn golden_file_exists(test_name: &str) -> bool {
    let path = format!("tests/golden/{}.json", test_name);
    std::path::Path::new(&path).exists()
}

/// Performance measurement helpers
pub struct PerformanceMetrics {
    pub iterations: usize,
    pub total_duration: Duration,
    pub min_duration: Duration,
    pub max_duration: Duration,
}

impl PerformanceMetrics {
    pub fn average_duration(&self) -> Duration {
        self.total_duration / self.iterations as u32
    }

    pub fn average_hz(&self) -> f64 {
        1.0 / self.average_duration().as_secs_f64()
    }
}

/// Measure function performance
pub fn measure_performance<F>(iterations: usize, mut f: F) -> PerformanceMetrics
where
    F: FnMut(),
{
    let mut total = Duration::from_secs(0);
    let mut min = Duration::from_secs(u64::MAX);
    let mut max = Duration::from_secs(0);

    for _ in 0..iterations {
        let start = std::time::Instant::now();
        f();
        let duration = start.elapsed();

        total += duration;
        min = min.min(duration);
        max = max.max(duration);
    }

    PerformanceMetrics {
        iterations,
        total_duration: total,
        min_duration: min,
        max_duration: max,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_robot_creation() {
        let robot = create_mock_robot();
        assert_eq!(robot.state().joints.iter().count(), unitree_g1::NUM_JOINTS);
    }

    #[test]
    fn test_mock_robot_commands() {
        let mut robot = create_mock_robot();
        let positions = vec![0.1; unitree_g1::NUM_JOINTS];
        assert!(robot.set_joint_positions(&positions).is_ok());
        assert_eq!(robot.command_history().len(), 1);
    }

    #[test]
    fn test_mock_robot_emergency_stop() {
        let mut robot = create_mock_robot();
        robot.emergency_stop();
        assert!(robot.is_emergency_stop_active());

        let positions = vec![0.1; unitree_g1::NUM_JOINTS];
        assert!(robot.set_joint_positions(&positions).is_err());
    }

    #[test]
    fn test_deterministic_simulation() {
        let mut sim1 = create_test_sim();
        let mut sim2 = create_test_sim();

        sim1.run_steps(100);
        sim2.run_steps(100);

        assert_states_equal(&sim1, &sim2);
    }

    #[test]
    fn test_trajectory_creation() {
        let trajectory = create_test_trajectory(2.0);
        assert!(trajectory.duration().as_secs_f64() > 0.0);
    }
}
