//! Reinforcement Learning Training Environment for Humanoid Robots
//!
//! Provides a Gym-like interface for training RL policies on humanoid control tasks.
//! Supports walking, balance, reaching, and other locomotion/manipulation tasks.

use crate::control::{Controller, Trajectory};
use crate::robot::{unitree_g1, HumanoidRobot, JointSet, Pose, RobotState};
use crate::whole_body::ComState;
use nalgebra::{DVector, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Duration;

/// Observation space for the RL environment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Observation {
    /// Joint positions (radians)
    pub joint_positions: Vec<f64>,
    /// Joint velocities (rad/s)
    pub joint_velocities: Vec<f64>,
    /// Base position (x, y, z)
    pub base_position: [f64; 3],
    /// Base orientation as quaternion (w, x, y, z)
    pub base_orientation: [f64; 4],
    /// Base linear velocity
    pub base_linear_velocity: [f64; 3],
    /// Base angular velocity
    pub base_angular_velocity: [f64; 3],
    /// Foot contact states [left, right]
    pub foot_contacts: [bool; 2],
    /// Center of mass position
    pub com_position: [f64; 3],
    /// Target velocity command (forward, lateral, yaw_rate)
    pub velocity_command: [f64; 3],
    /// Time since episode start
    pub time: f64,
}

impl Default for Observation {
    fn default() -> Self {
        Self {
            joint_positions: vec![0.0; unitree_g1::NUM_JOINTS],
            joint_velocities: vec![0.0; unitree_g1::NUM_JOINTS],
            base_position: [0.0, 0.9, 0.0],
            base_orientation: [1.0, 0.0, 0.0, 0.0],
            base_linear_velocity: [0.0, 0.0, 0.0],
            base_angular_velocity: [0.0, 0.0, 0.0],
            foot_contacts: [true, true],
            com_position: [0.0, 0.9, 0.0],
            velocity_command: [0.0, 0.0, 0.0],
            time: 0.0,
        }
    }
}

impl Observation {
    /// Convert to flat vector for neural network input
    pub fn to_vector(&self) -> Vec<f64> {
        let mut vec = Vec::with_capacity(self.dimension());

        vec.extend(&self.joint_positions);
        vec.extend(&self.joint_velocities);
        vec.extend(&self.base_position);
        vec.extend(&self.base_orientation);
        vec.extend(&self.base_linear_velocity);
        vec.extend(&self.base_angular_velocity);
        vec.push(if self.foot_contacts[0] { 1.0 } else { 0.0 });
        vec.push(if self.foot_contacts[1] { 1.0 } else { 0.0 });
        vec.extend(&self.com_position);
        vec.extend(&self.velocity_command);
        vec.push(self.time);

        vec
    }

    /// Get observation space dimension
    pub fn dimension(&self) -> usize {
        self.joint_positions.len() * 2  // positions + velocities
            + 3  // base position
            + 4  // base orientation
            + 3  // base linear velocity
            + 3  // base angular velocity
            + 2  // foot contacts
            + 3  // com position
            + 3  // velocity command
            + 1 // time
    }
}

/// Action space for the RL environment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Action {
    /// Target joint positions (or deltas depending on action mode)
    pub joint_targets: Vec<f64>,
}

impl Default for Action {
    fn default() -> Self {
        Self {
            joint_targets: vec![0.0; unitree_g1::NUM_JOINTS],
        }
    }
}

impl Action {
    /// Create from flat vector
    pub fn from_vector(vec: &[f64]) -> Self {
        Self {
            joint_targets: vec.to_vec(),
        }
    }

    /// Get action space dimension
    pub fn dimension() -> usize {
        unitree_g1::NUM_JOINTS
    }
}

/// Action mode - how actions are interpreted
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActionMode {
    /// Actions are absolute target positions
    Position,
    /// Actions are velocity commands
    Velocity,
    /// Actions are position deltas from current position
    PositionDelta,
    /// Actions are torque commands
    Torque,
}

/// Task type for the RL environment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TaskType {
    /// Stand still and maintain balance
    Stand,
    /// Walk forward at target velocity
    Walk,
    /// Turn in place
    Turn,
    /// Reach target position with hand
    Reach,
    /// Get up from ground
    GetUp,
    /// Follow velocity command (omnidirectional)
    VelocityTracking,
    /// Custom task with user-defined reward
    Custom,
}

/// Configuration for the RL environment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvConfig {
    /// Task type
    pub task: TaskType,
    /// Action mode
    pub action_mode: ActionMode,
    /// Simulation timestep (seconds)
    pub dt: f64,
    /// Maximum episode length (seconds)
    pub max_episode_time: f64,
    /// Control frequency relative to sim (e.g., 4 = control every 4 sim steps)
    pub control_decimation: usize,
    /// Action scaling factor
    pub action_scale: f64,
    /// Whether to add action noise
    pub add_action_noise: bool,
    /// Action noise standard deviation
    pub action_noise_std: f64,
    /// Whether to add observation noise
    pub add_observation_noise: bool,
    /// Observation noise standard deviation
    pub observation_noise_std: f64,
    /// Random initial pose variance
    pub initial_pose_noise: f64,
    /// Target velocity for walking task [forward, lateral, yaw_rate]
    pub target_velocity: [f64; 3],
    /// Reward weights
    pub reward_weights: RewardWeights,
    /// Early termination conditions
    pub termination: TerminationConfig,
}

impl Default for EnvConfig {
    fn default() -> Self {
        Self {
            task: TaskType::Walk,
            action_mode: ActionMode::Position,
            dt: 0.002, // 500 Hz simulation
            max_episode_time: 20.0,
            control_decimation: 4, // 125 Hz control
            action_scale: 0.5,
            add_action_noise: false,
            action_noise_std: 0.01,
            add_observation_noise: false,
            observation_noise_std: 0.01,
            initial_pose_noise: 0.0,
            target_velocity: [1.0, 0.0, 0.0], // 1 m/s forward
            reward_weights: RewardWeights::default(),
            termination: TerminationConfig::default(),
        }
    }
}

/// Reward weights for different components
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RewardWeights {
    /// Weight for tracking target velocity
    pub velocity_tracking: f64,
    /// Weight for base height tracking
    pub height_tracking: f64,
    /// Weight for base orientation (upright)
    pub orientation: f64,
    /// Weight for smooth actions (penalize large changes)
    pub action_smoothness: f64,
    /// Weight for joint limit proximity penalty
    pub joint_limits: f64,
    /// Weight for energy usage penalty
    pub energy: f64,
    /// Weight for foot contact pattern (gait)
    pub gait_pattern: f64,
    /// Weight for symmetric motion
    pub symmetry: f64,
    /// Weight for surviving (per step bonus)
    pub alive_bonus: f64,
    /// Penalty for falling
    pub fall_penalty: f64,
}

impl Default for RewardWeights {
    fn default() -> Self {
        Self {
            velocity_tracking: 1.0,
            height_tracking: 0.5,
            orientation: 0.5,
            action_smoothness: 0.1,
            joint_limits: 0.1,
            energy: 0.01,
            gait_pattern: 0.2,
            symmetry: 0.1,
            alive_bonus: 0.01,
            fall_penalty: 10.0,
        }
    }
}

/// Termination conditions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerminationConfig {
    /// Terminate if base height falls below this (meters)
    pub min_height: f64,
    /// Terminate if base height exceeds this (meters)
    pub max_height: f64,
    /// Terminate if base roll exceeds this (radians)
    pub max_roll: f64,
    /// Terminate if base pitch exceeds this (radians)
    pub max_pitch: f64,
    /// Terminate if any joint limit exceeded by this margin
    pub joint_limit_margin: f64,
}

impl Default for TerminationConfig {
    fn default() -> Self {
        Self {
            min_height: 0.3,
            max_height: 1.5,
            max_roll: 1.0,
            max_pitch: 1.0,
            joint_limit_margin: 0.1,
        }
    }
}

/// Step result from the environment
#[derive(Debug, Clone)]
pub struct StepResult {
    /// New observation
    pub observation: Observation,
    /// Reward received
    pub reward: f64,
    /// Whether episode terminated
    pub terminated: bool,
    /// Whether episode truncated (time limit)
    pub truncated: bool,
    /// Additional info
    pub info: StepInfo,
}

/// Additional information from a step
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StepInfo {
    /// Individual reward components
    pub reward_components: HashMap<String, f64>,
    /// Current velocity
    pub current_velocity: [f64; 3],
    /// Target velocity
    pub target_velocity: [f64; 3],
    /// Episode length so far
    pub episode_length: f64,
    /// Termination reason if terminated
    pub termination_reason: Option<String>,
}

/// RL Environment for humanoid robot training
pub struct RlEnvironment {
    config: EnvConfig,

    // Internal state
    state: RobotState,
    last_action: Action,
    episode_time: f64,
    step_count: usize,

    // Physics state (simplified, would connect to simulator)
    base_linear_velocity: Vector3<f64>,
    base_angular_velocity: Vector3<f64>,

    // Target tracking
    target_velocity: Vector3<f64>,

    // Random number generator seed
    rng_seed: u64,
}

impl RlEnvironment {
    /// Create a new RL environment
    pub fn new(config: EnvConfig) -> Self {
        let target_velocity = Vector3::new(
            config.target_velocity[0],
            config.target_velocity[1],
            config.target_velocity[2],
        );

        Self {
            config,
            state: RobotState::default(),
            last_action: Action::default(),
            episode_time: 0.0,
            step_count: 0,
            base_linear_velocity: Vector3::zeros(),
            base_angular_velocity: Vector3::zeros(),
            target_velocity,
            rng_seed: 42,
        }
    }

    /// Reset the environment to initial state
    pub fn reset(&mut self, seed: Option<u64>) -> Observation {
        if let Some(s) = seed {
            self.rng_seed = s;
        }

        // Reset to default standing pose
        self.state = RobotState::default();
        self.state.joints = unitree_g1::create_default_joints();
        self.state.base_pose = Pose::new(0.0, 0.9, 0.0, 0.0, 0.0, 0.0);
        self.state.is_standing = true;
        self.state.is_balanced = true;
        self.state.foot_contacts = [true, true];

        // Apply initial pose noise if configured
        if self.config.initial_pose_noise > 0.0 {
            // Add small random perturbations to joint positions
            // Generate noise values first to avoid borrow issues
            let noise = self.config.initial_pose_noise;
            let num_joints = self.state.joints.iter().count();
            let perturbations: Vec<f64> = (0..num_joints)
                .map(|_| (self.pseudo_random() - 0.5) * 2.0 * noise)
                .collect();

            for (joint, perturbation) in self.state.joints.iter_mut().zip(perturbations.iter()) {
                joint.current_position += perturbation;
            }
        }

        self.base_linear_velocity = Vector3::zeros();
        self.base_angular_velocity = Vector3::zeros();
        self.episode_time = 0.0;
        self.step_count = 0;
        self.last_action = Action::default();

        self.get_observation()
    }

    /// Take a step in the environment
    pub fn step(&mut self, action: &Action) -> StepResult {
        // Apply action scaling and noise
        let mut scaled_action = action.clone();
        for target in &mut scaled_action.joint_targets {
            *target *= self.config.action_scale;

            if self.config.add_action_noise {
                *target += (self.pseudo_random() - 0.5) * 2.0 * self.config.action_noise_std;
            }
        }

        // Apply action based on mode
        match self.config.action_mode {
            ActionMode::Position => {
                self.state
                    .joints
                    .set_positions(&scaled_action.joint_targets);
            }
            ActionMode::PositionDelta => {
                let current = self.state.joints.positions();
                let new_positions: Vec<f64> = current
                    .iter()
                    .zip(scaled_action.joint_targets.iter())
                    .map(|(c, d)| c + d)
                    .collect();
                self.state.joints.set_positions(&new_positions);
            }
            ActionMode::Velocity => {
                // Update positions based on velocity
                let dt = self.config.dt * self.config.control_decimation as f64;
                let current = self.state.joints.positions();
                let new_positions: Vec<f64> = current
                    .iter()
                    .zip(scaled_action.joint_targets.iter())
                    .map(|(p, v)| p + v * dt)
                    .collect();
                self.state.joints.set_positions(&new_positions);
            }
            ActionMode::Torque => {
                // Would apply torques through dynamics simulation
                // Simplified: convert torques to accelerations
            }
        }

        // Simulate physics (simplified - would use actual physics engine)
        self.simulate_physics();

        // Update time
        let dt = self.config.dt * self.config.control_decimation as f64;
        self.episode_time += dt;
        self.step_count += 1;

        // Get observation
        let observation = self.get_observation();

        // Compute reward
        let (reward, reward_components) = self.compute_reward(&scaled_action);

        // Check termination
        let (terminated, termination_reason) = self.check_termination();
        let truncated = self.episode_time >= self.config.max_episode_time;

        // Store last action
        self.last_action = action.clone();

        let info = StepInfo {
            reward_components,
            current_velocity: [
                self.base_linear_velocity.x,
                self.base_linear_velocity.y,
                self.base_angular_velocity.z,
            ],
            target_velocity: self.config.target_velocity,
            episode_length: self.episode_time,
            termination_reason,
        };

        StepResult {
            observation,
            reward,
            terminated,
            truncated,
            info,
        }
    }

    /// Get current observation
    fn get_observation(&mut self) -> Observation {
        let positions = self.state.joints.positions();
        let velocities: Vec<f64> = self.state.joints.iter().map(|j| j.velocity).collect();

        let base_quat = self.state.base_pose.orientation;

        let mut obs = Observation {
            joint_positions: positions,
            joint_velocities: velocities,
            base_position: [
                self.state.base_pose.position.x,
                self.state.base_pose.position.y,
                self.state.base_pose.position.z,
            ],
            base_orientation: [base_quat.w, base_quat.i, base_quat.j, base_quat.k],
            base_linear_velocity: [
                self.base_linear_velocity.x,
                self.base_linear_velocity.y,
                self.base_linear_velocity.z,
            ],
            base_angular_velocity: [
                self.base_angular_velocity.x,
                self.base_angular_velocity.y,
                self.base_angular_velocity.z,
            ],
            foot_contacts: self.state.foot_contacts,
            com_position: [
                self.state.base_pose.position.x,
                self.state.base_pose.position.y,
                self.state.base_pose.position.z,
            ],
            velocity_command: self.config.target_velocity,
            time: self.episode_time,
        };

        // Add observation noise if configured
        if self.config.add_observation_noise {
            let noise_std = self.config.observation_noise_std;
            let num_positions = obs.joint_positions.len();
            let num_velocities = obs.joint_velocities.len();

            // Generate noise values first
            let pos_noise: Vec<f64> = (0..num_positions)
                .map(|_| (self.pseudo_random() - 0.5) * 2.0 * noise_std)
                .collect();
            let vel_noise: Vec<f64> = (0..num_velocities)
                .map(|_| (self.pseudo_random() - 0.5) * 2.0 * noise_std * 10.0)
                .collect();

            for (pos, noise) in obs.joint_positions.iter_mut().zip(pos_noise.iter()) {
                *pos += noise;
            }
            for (vel, noise) in obs.joint_velocities.iter_mut().zip(vel_noise.iter()) {
                *vel += noise;
            }
        }

        obs
    }

    /// Compute reward based on current state and action
    fn compute_reward(&self, action: &Action) -> (f64, HashMap<String, f64>) {
        let weights = &self.config.reward_weights;
        let mut components = HashMap::new();
        let mut total_reward = 0.0;

        // Velocity tracking reward
        let velocity_error = (self.target_velocity - self.base_linear_velocity).norm();
        let velocity_reward = (-velocity_error).exp();
        components.insert(
            "velocity_tracking".to_string(),
            velocity_reward * weights.velocity_tracking,
        );
        total_reward += velocity_reward * weights.velocity_tracking;

        // Height tracking reward
        let target_height = 0.9; // Standing height
        let height_error = (self.state.base_pose.position.y - target_height).abs();
        let height_reward = (-5.0 * height_error).exp();
        components.insert(
            "height_tracking".to_string(),
            height_reward * weights.height_tracking,
        );
        total_reward += height_reward * weights.height_tracking;

        // Orientation reward (upright)
        let euler = self.state.base_pose.orientation.euler_angles();
        let orientation_error = euler.0.abs() + euler.1.abs(); // roll + pitch
        let orientation_reward = (-orientation_error).exp();
        components.insert(
            "orientation".to_string(),
            orientation_reward * weights.orientation,
        );
        total_reward += orientation_reward * weights.orientation;

        // Action smoothness (penalize large action changes)
        let action_diff: f64 = action
            .joint_targets
            .iter()
            .zip(self.last_action.joint_targets.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum();
        let smoothness_reward = -action_diff.sqrt();
        components.insert(
            "action_smoothness".to_string(),
            smoothness_reward * weights.action_smoothness,
        );
        total_reward += smoothness_reward * weights.action_smoothness;

        // Energy penalty
        let energy: f64 = action.joint_targets.iter().map(|a| a.abs()).sum();
        let energy_penalty = -energy / action.joint_targets.len() as f64;
        components.insert("energy".to_string(), energy_penalty * weights.energy);
        total_reward += energy_penalty * weights.energy;

        // Alive bonus
        components.insert("alive_bonus".to_string(), weights.alive_bonus);
        total_reward += weights.alive_bonus;

        // Gait pattern reward (alternating foot contacts for walking)
        if self.config.task == TaskType::Walk {
            let phase = (self.episode_time * 2.0 * std::f64::consts::PI).sin();
            let expected_left = phase > 0.0;
            let expected_right = !expected_left;

            let gait_match = (self.state.foot_contacts[0] == expected_left) as i32
                + (self.state.foot_contacts[1] == expected_right) as i32;
            let gait_reward = gait_match as f64 / 2.0;
            components.insert(
                "gait_pattern".to_string(),
                gait_reward * weights.gait_pattern,
            );
            total_reward += gait_reward * weights.gait_pattern;
        }

        (total_reward, components)
    }

    /// Check for termination conditions
    fn check_termination(&self) -> (bool, Option<String>) {
        let config = &self.config.termination;

        // Height check
        let height = self.state.base_pose.position.y;
        if height < config.min_height {
            return (true, Some("Fell down (height too low)".to_string()));
        }
        if height > config.max_height {
            return (true, Some("Height exceeded maximum".to_string()));
        }

        // Orientation check
        let euler = self.state.base_pose.orientation.euler_angles();
        if euler.0.abs() > config.max_roll {
            return (true, Some("Roll limit exceeded".to_string()));
        }
        if euler.1.abs() > config.max_pitch {
            return (true, Some("Pitch limit exceeded".to_string()));
        }

        (false, None)
    }

    /// Simplified physics simulation
    fn simulate_physics(&mut self) {
        let dt = self.config.dt * self.config.control_decimation as f64;

        // Simple gravity simulation
        if !self.state.foot_contacts[0] && !self.state.foot_contacts[1] {
            self.base_linear_velocity.y -= 9.81 * dt;
        }

        // Update base position based on velocity
        self.state.base_pose.position.x += self.base_linear_velocity.x * dt;
        self.state.base_pose.position.y += self.base_linear_velocity.y * dt;
        self.state.base_pose.position.z += self.base_linear_velocity.z * dt;

        // Ground collision
        if self.state.base_pose.position.y < 0.5 {
            self.state.foot_contacts = [true, true];
            self.base_linear_velocity.y = 0.0;
        }

        // Walking forward based on target velocity (simplified)
        if self.config.task == TaskType::Walk
            && (self.state.foot_contacts[0] || self.state.foot_contacts[1])
        {
            let target_vx = self.config.target_velocity[0];
            self.base_linear_velocity.x = self.base_linear_velocity.x * 0.9 + target_vx * 0.1;
        }
    }

    /// Pseudo-random number generator (simple LCG)
    fn pseudo_random(&mut self) -> f64 {
        self.rng_seed = self
            .rng_seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.rng_seed as f64) / (u64::MAX as f64)
    }

    /// Get environment configuration
    pub fn config(&self) -> &EnvConfig {
        &self.config
    }

    /// Get observation dimension
    pub fn observation_dim(&self) -> usize {
        Observation::default().dimension()
    }

    /// Get action dimension
    pub fn action_dim(&self) -> usize {
        Action::dimension()
    }

    /// Set target velocity
    pub fn set_target_velocity(&mut self, forward: f64, lateral: f64, yaw_rate: f64) {
        self.target_velocity = Vector3::new(forward, lateral, yaw_rate);
        self.config.target_velocity = [forward, lateral, yaw_rate];
    }
}

/// Vectorized environment for parallel training
pub struct VecEnv {
    envs: Vec<RlEnvironment>,
}

impl VecEnv {
    /// Create a vectorized environment
    pub fn new(config: EnvConfig, num_envs: usize) -> Self {
        let envs = (0..num_envs)
            .map(|i| {
                let mut env = RlEnvironment::new(config.clone());
                env.rng_seed = 42 + i as u64;
                env
            })
            .collect();

        Self { envs }
    }

    /// Reset all environments
    pub fn reset(&mut self) -> Vec<Observation> {
        self.envs
            .iter_mut()
            .enumerate()
            .map(|(i, env)| env.reset(Some(42 + i as u64)))
            .collect()
    }

    /// Step all environments
    pub fn step(&mut self, actions: &[Action]) -> Vec<StepResult> {
        self.envs
            .iter_mut()
            .zip(actions.iter())
            .map(|(env, action)| {
                let result = env.step(action);
                // Auto-reset on termination
                if result.terminated || result.truncated {
                    env.reset(None);
                }
                result
            })
            .collect()
    }

    /// Get number of environments
    pub fn num_envs(&self) -> usize {
        self.envs.len()
    }
}

/// Trait for RL policy implementations
pub trait Policy {
    /// Get action from observation
    fn get_action(&mut self, observation: &Observation) -> Action;

    /// Update policy (for training)
    fn update(&mut self, batch: &ExperienceBatch) -> f64;

    /// Save policy to file
    fn save(&self, path: &str) -> std::io::Result<()>;

    /// Load policy from file
    fn load(&mut self, path: &str) -> std::io::Result<()>;
}

/// Experience tuple for replay buffer
#[derive(Debug, Clone)]
pub struct Experience {
    pub observation: Observation,
    pub action: Action,
    pub reward: f64,
    pub next_observation: Observation,
    pub terminated: bool,
}

/// Batch of experiences for training
#[derive(Debug, Clone)]
pub struct ExperienceBatch {
    pub experiences: Vec<Experience>,
}

impl ExperienceBatch {
    pub fn new() -> Self {
        Self {
            experiences: Vec::new(),
        }
    }

    pub fn add(&mut self, exp: Experience) {
        self.experiences.push(exp);
    }

    pub fn len(&self) -> usize {
        self.experiences.len()
    }

    pub fn is_empty(&self) -> bool {
        self.experiences.is_empty()
    }

    pub fn clear(&mut self) {
        self.experiences.clear();
    }
}

impl Default for ExperienceBatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Simple random policy for testing
pub struct RandomPolicy {
    action_scale: f64,
    seed: u64,
}

impl RandomPolicy {
    pub fn new(action_scale: f64) -> Self {
        Self {
            action_scale,
            seed: 42,
        }
    }

    fn pseudo_random(&mut self) -> f64 {
        self.seed = self
            .seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.seed as f64) / (u64::MAX as f64) * 2.0 - 1.0
    }
}

impl Policy for RandomPolicy {
    fn get_action(&mut self, _observation: &Observation) -> Action {
        Action {
            joint_targets: (0..unitree_g1::NUM_JOINTS)
                .map(|_| self.pseudo_random() * self.action_scale)
                .collect(),
        }
    }

    fn update(&mut self, _batch: &ExperienceBatch) -> f64 {
        0.0 // No learning
    }

    fn save(&self, _path: &str) -> std::io::Result<()> {
        Ok(())
    }

    fn load(&mut self, _path: &str) -> std::io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_environment_creation() {
        let config = EnvConfig::default();
        let mut env = RlEnvironment::new(config);

        let obs = env.reset(Some(42));
        assert_eq!(obs.joint_positions.len(), unitree_g1::NUM_JOINTS);
        assert_eq!(obs.dimension(), env.observation_dim());
    }

    #[test]
    fn test_environment_step() {
        let config = EnvConfig::default();
        let mut env = RlEnvironment::new(config);

        env.reset(Some(42));

        let action = Action::default();
        let result = env.step(&action);

        assert!(!result.observation.joint_positions.is_empty());
        assert!(result.reward.is_finite());
    }

    #[test]
    fn test_vec_env() {
        let config = EnvConfig::default();
        let mut vec_env = VecEnv::new(config, 4);

        let observations = vec_env.reset();
        assert_eq!(observations.len(), 4);

        let actions: Vec<Action> = (0..4).map(|_| Action::default()).collect();
        let results = vec_env.step(&actions);
        assert_eq!(results.len(), 4);
    }

    #[test]
    fn test_observation_vector() {
        let obs = Observation::default();
        let vec = obs.to_vector();

        assert_eq!(vec.len(), obs.dimension());
    }

    #[test]
    fn test_random_policy() {
        let mut policy = RandomPolicy::new(0.1);
        let obs = Observation::default();

        let action = policy.get_action(&obs);
        assert_eq!(action.joint_targets.len(), unitree_g1::NUM_JOINTS);

        // Check action is within scale
        for target in &action.joint_targets {
            assert!(target.abs() <= 0.1);
        }
    }
}
