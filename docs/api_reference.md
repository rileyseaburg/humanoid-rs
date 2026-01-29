# API Reference

Complete reference for all public types, traits, and functions in Humanoid-RS.

## Core Module (`humanoid_rs`)

### Re-exports

```rust
pub use robot::{HumanoidRobot, Joint, JointSet, Pose};
pub use control::{Controller, Trajectory, MotionPlanner};
pub use ik::{IkSolver, WholeBodyIk, PositionTarget, IkSolverConfig};
pub use whole_body::{WholeBodyController, BalanceRecovery, ComState};
pub use mpc::{MpcController, FootstepPlanner, LipmState};
```

### Error Types

#### `HumanoidError`

```rust
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
```

**Example:**
```rust
use humanoid_rs::{HumanoidError, Result};

fn move_joint(position: f64) -> Result<()> {
    if position > 3.14 {
        return Err(HumanoidError::JointLimitExceeded {
            joint_name: "shoulder".to_string(),
            requested: position,
            min: -3.14,
            max: 3.14,
        });
    }
    Ok(())
}
```

### Type Aliases

- `Result<T>` = `std::result::Result<T, HumanoidError>`

### Constants

- `VERSION: &str` - Framework version from Cargo.toml

## Robot Module (`humanoid_rs::robot`)

### `HumanoidRobot` Trait

Main abstraction for any humanoid robot.

```rust
pub trait HumanoidRobot {
    /// Get current robot state (immutable)
    fn state(&self) -> &RobotState;
    
    /// Send joint position commands
    fn set_joint_positions(&mut self, positions: &[f64]) -> Result<()>;
    
    /// Send joint velocity commands
    fn set_joint_velocities(&mut self, velocities: &[f64]) -> Result<()>;
    
    /// Send joint effort/torque commands
    fn set_joint_efforts(&mut self, efforts: &[f64]) -> Result<()>;
    
    /// Compute forward kinematics for a link
    fn forward_kinematics(&self, link_name: &str) -> Result<Pose>;
    
    /// Get number of actuated joints
    fn num_joints(&self) -> usize;
    
    /// Emergency stop - immediately stop all motion
    fn emergency_stop(&mut self) -> Result<()>;
    
    /// Reset emergency stop condition
    fn reset_emergency_stop(&mut self) -> Result<()>;
}
```

**Example:**
```rust
use humanoid_rs::robot::HumanoidRobot;
use humanoid_rs::unitree::UnitreeRobot;

#[tokio::main]
async fn main() -> Result<()> {
    let mut robot = UnitreeRobot::connect("192.168.123.104:8080").await?;
    
    // Get current state
    let state = robot.state();
    println!("Current positions: {:?}", state.joints.positions());
    
    // Send position command
    let positions = vec![0.0; robot.num_joints()];
    robot.set_joint_positions(&positions)?;
    
    Ok(())
}
```

### `Joint`

Represents a single robot joint.

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub joint_type: JointType,
    pub parent: Option<String>,
    pub limits: JointLimits,
    pub current_position: f64,
    pub target_position: f64,
    pub velocity: f64,
    pub effort: f64,
}
```

### `JointType`

```rust
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum JointType {
    Revolute { axis: Vector3<f64> },
    Prismatic { axis: Vector3<f64> },
    Fixed,
    Spherical,
}
```

### `JointLimits`

```rust
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointLimits {
    pub min: f64,
    pub max: f64,
    pub max_velocity: f64,
    pub max_effort: f64,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self {
            min: -3.14,
            max: 3.14,
            max_velocity: 10.0,
            max_effort: 100.0,
        }
    }
}
```

### `JointSet`

Collection of joints with utility methods.

```rust
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct JointSet {
    joints: Vec<Joint>,
}

impl JointSet {
    pub fn new() -> Self;
    pub fn add_joint(&mut self, joint: Joint);
    pub fn get_joint(&self, name: &str) -> Option<&Joint>;
    pub fn get_joint_mut(&mut self, name: &str) -> Option<&mut Joint>;
    pub fn iter(&self) -> impl Iterator<Item = &Joint>;
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Joint>;
    pub fn positions(&self) -> Vec<f64>;
    pub fn set_positions(&mut self, positions: &[f64]);
}
```

### `Pose`

3D position and orientation.

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
}

impl Pose {
    pub fn new(x: f64, y: f64, z: f64, roll: f64, pitch: f64, yaw: f64) -> Self;
    pub fn to_isometry(&self) -> Isometry3<f64>;
}
```

**Example:**
```rust
use humanoid_rs::robot::Pose;

// Create pose at position (1.0, 2.0, 3.0) with 90° yaw
let pose = Pose::new(1.0, 2.0, 3.0, 0.0, 0.0, std::f64::consts::PI / 2.0);
```

### `RobotState`

Complete robot state snapshot.

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotState {
    pub timestamp: Duration,
    pub base_pose: Pose,
    pub joints: JointSet,
    pub is_standing: bool,
    pub is_balanced: bool,
    pub foot_contacts: [bool; 2],  // [left, right]
}
```

### Unitree G1 Configuration

```rust
pub mod unitree_g1 {
    pub const NUM_JOINTS: usize = 29;
    
    pub fn default_joint_names() -> Vec<String>;
    pub fn create_default_joints() -> JointSet;
}
```

## Control Module (`humanoid_rs::control`)

### `Trajectory`

Time-parameterized motion path.

```rust
#[derive(Debug, Clone, Default)]
pub struct Trajectory {
    waypoints: Vec<Waypoint>,
}

impl Trajectory {
    pub fn new() -> Self;
    pub fn add_waypoint(&mut self, waypoint: Waypoint);
    pub fn interpolate(&self, time: Duration) -> Option<Waypoint>;
    pub fn duration(&self) -> Duration;
}
```

### `Waypoint`

```rust
#[derive(Debug, Clone)]
pub struct Waypoint {
    pub time: Duration,
    pub joint_positions: Vec<f64>,
    pub base_pose: Option<Pose>,
}
```

### `MotionPlanner`

Generates smooth trajectories.

```rust
pub struct MotionPlanner {
    max_velocity: f64,
    max_acceleration: f64,
}

impl MotionPlanner {
    pub fn new(max_velocity: f64, max_acceleration: f64) -> Self;
    
    /// Plan joint space trajectory
    pub fn plan_joint_space(
        &self,
        current: &JointSet,
        target: &JointSet,
        duration: Duration,
    ) -> Trajectory;
    
    /// Plan Cartesian trajectory
    pub fn plan_cartesian(
        &self,
        start: &Pose,
        end: &Pose,
        duration: Duration,
    ) -> Trajectory;
    
    /// Plan standing up motion
    pub fn plan_stand_up(&self, current: &JointSet) -> Trajectory;
}
```

**Example:**
```rust
use humanoid_rs::control::MotionPlanner;
use std::time::Duration;

let planner = MotionPlanner::new(2.0, 5.0);
let trajectory = planner.plan_stand_up(&current_joints);
println!("Trajectory duration: {:?}", trajectory.duration());
```

### `Controller` Trait

```rust
pub trait Controller {
    /// Update control loop - called at control frequency
    fn update(&mut self, state: &RobotState, dt: Duration) -> Result<JointSet>;
    
    /// Set target trajectory
    fn set_trajectory(&mut self, trajectory: Trajectory);
    
    /// Check if trajectory is complete
    fn is_complete(&self) -> bool;
}
```

### `JointSpaceController`

PD controller for joint space tracking.

```rust
pub struct JointSpaceController {
    kp: f64,  // Proportional gain
    kd: f64,  // Derivative gain
}

impl JointSpaceController {
    pub fn new(kp: f64, kd: f64) -> Self;
}

impl Controller for JointSpaceController {
    // ...
}
```

## MPC Module (`humanoid_rs::mpc`)

### `MpcController`

Model Predictive Control for locomotion.

```rust
pub struct MpcController {
    // ...
}

impl MpcController {
    pub fn new(config: MpcConfig) -> Self;
    pub fn set_reference(&mut self, zmp_ref: ZmpReference);
    pub fn solve(&mut self, state: &LipmState) -> (Vector3<f64>, Vector3<f64>);
    pub fn capture_point(&self, state: &LipmState) -> Vector3<f64>;
    pub fn reset(&mut self);
}
```

### `MpcConfig`

```rust
#[derive(Debug, Clone)]
pub struct MpcConfig {
    pub horizon: usize,           // Prediction steps
    pub dt: f64,                  // Control timestep
    pub com_height: f64,          // Center of mass height
    pub weight_com_pos: f64,      // CoM position weight
    pub weight_com_vel: f64,      // CoM velocity weight
    pub weight_zmp: f64,          // ZMP tracking weight
    pub weight_jerk: f64,         // Control smoothness weight
    pub zmp_margin: f64,          // Stability margin
    pub gravity: f64,             // Gravity constant
}

impl Default for MpcConfig {
    fn default() -> Self {
        Self {
            horizon: 20,
            dt: 0.02,
            com_height: 0.85,
            weight_com_pos: 1.0,
            weight_com_vel: 0.1,
            weight_zmp: 10.0,
            weight_jerk: 0.001,
            zmp_margin: 0.08,
            gravity: 9.81,
        }
    }
}
```

### `LipmState`

Linear Inverted Pendulum Model state.

```rust
#[derive(Debug, Clone, Default)]
pub struct LipmState {
    pub x: f64,
    pub x_dot: f64,
    pub y: f64,
    pub y_dot: f64,
}

impl LipmState {
    pub fn new(x: f64, x_dot: f64, y: f64, y_dot: f64) -> Self;
    pub fn com_position(&self) -> Vector3<f64>;
    pub fn com_velocity(&self) -> Vector3<f64>;
}
```

### `FootstepPlanner`

```rust
pub struct FootstepPlanner {
    pub step_length: f64,
    pub step_width: f64,
    pub step_duration: f64,
    pub double_support_ratio: f64,
}

impl FootstepPlanner {
    pub fn plan_walking(&self, velocity: Vector3<f64>, duration: f64) -> Vec<Footstep>;
    pub fn footsteps_to_zmp(&self, footsteps: &[Footstep], dt: f64) -> ZmpReference;
}
```

## Whole-Body Control Module (`humanoid_rs::whole_body`)

### `WholeBodyController`

Coordinated whole-body motion and balance.

```rust
pub struct WholeBodyController {
    // ...
}

impl WholeBodyController {
    pub fn new(config: WholeBodyConfig) -> Self;
    pub fn set_com_target(&mut self, position: Vector3<f64>, velocity: Vector3<f64>);
    pub fn set_foot_contacts(&mut self, left: FootContact, right: FootContact);
    pub fn compute(&mut self, state: &RobotState, dt: Duration) -> Vec<f64>;
    pub fn is_balanced(&self, state: &RobotState) -> bool;
    pub fn stability_margin(&self, state: &RobotState) -> f64;
}
```

### `ComState`

Center of mass state.

```rust
#[derive(Debug, Clone, Default)]
pub struct ComState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}
```

### `FootContact`

```rust
#[derive(Debug, Clone)]
pub struct FootContact {
    pub position: Vector3<f64>,
    pub normal: Vector3<f64>,
    pub in_contact: bool,
    pub force: Vector3<f64>,
}
```

### `BalanceRecovery`

```rust
pub struct BalanceRecovery {
    // ...
}

impl BalanceRecovery {
    pub fn compute(&self, com: &ComState, support: &SupportPolygon) -> RecoveryAction;
}

pub enum RecoveryAction {
    None,
    Ankle { torque: f64 },
    Hip { angle: f64 },
    Step { target: Vector3<f64> },
}
```

## IK Module (`humanoid_rs::ik`)

### `IkSolver`

```rust
pub struct IkSolver {
    config: IkSolverConfig,
}

impl IkSolver {
    pub fn new(config: IkSolverConfig) -> Self;
    pub fn with_default_config() -> Self;
    pub fn solve_arm_2dof(&self, l1: f64, l2: f64, target: Vector3<f64>) -> Option<(f64, f64)>;
    pub fn solve_arm_reach(&self, target: Vector3<f64>, shoulder_pos: Vector3<f64>, 
                          arm_lengths: (f64, f64, f64)) -> Vec<f64>;
    pub fn solve_ccd(&self, joints: &mut JointSet, end_effector_pos: Vector3<f64>,
                     joint_indices: &[usize], joint_axes: &[Vector3<f64>]) -> bool;
}
```

### `IkSolverConfig`

```rust
#[derive(Debug, Clone)]
pub struct IkSolverConfig {
    pub max_iterations: usize,
    pub learning_rate: f64,
    pub joint_limits_penalty: f64,
}

impl Default for IkSolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            learning_rate: 0.1,
            joint_limits_penalty: 10.0,
        }
    }
}
```

### `PositionTarget`

```rust
#[derive(Debug, Clone)]
pub struct PositionTarget {
    pub position: Vector3<f64>,
    pub tolerance: f64,
}
```

## Simulator Module (`humanoid_rs::simulator`)

*Requires `simulator` feature*

### `SimulatedRobot`

```rust
pub struct SimulatedRobot {
    pub state: RobotState,
    pub controller: Option<Box<dyn Controller + Send + Sync>>,
    pub commands: Vec<f64>,
}

impl SimulatedRobot {
    pub fn new() -> Self;
    pub fn set_controller(&mut self, controller: Box<dyn Controller + Send + Sync>);
    pub fn set_target_positions(&mut self, positions: Vec<f64>);
    pub fn get_state_json(&self) -> String;
    pub fn load_trajectory(&mut self, trajectory: Trajectory);
}
```

### `run_simulator`

```rust
pub fn run_simulator();
```

Launches the 3D Bevy + Rapier simulator.

**Example:**
```rust
use humanoid_rs::simulator::run_simulator;

fn main() {
    run_simulator(); // Launches interactive 3D simulator
}
```

## Unitree Module (`humanoid_rs::unitree`)

*Requires `unitree` feature*

### `UnitreeRobot`

```rust
pub struct UnitreeRobot {
    // ...
}

impl UnitreeRobot {
    pub async fn connect(addr: &str) -> Result<Self>;
    pub fn mock() -> Self;
    pub fn enable_low_level_control(&mut self) -> Result<()>;
    pub fn disable_low_level_control(&mut self) -> Result<()>;
    pub fn stand_up(&mut self) -> Result<()>;
    pub fn sit_down(&mut self) -> Result<()>;
    pub fn walk(&mut self, vx: f64, vy: f64, vyaw: f64) -> Result<()>;
    pub fn stop(&mut self) -> Result<()>;
    pub fn poll_state(&mut self) -> Result<bool>;
    pub fn is_connected(&self) -> bool;
    pub fn mode(&self) -> RobotMode;
}

impl HumanoidRobot for UnitreeRobot {
    // ...
}
```

### `RobotMode`

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RobotMode {
    Idle,
    Standing,
    Walking,
    Running,
    Climbing,
    LowLevel,
}
```

**Example:**
```rust
use humanoid_rs::unitree::UnitreeRobot;

#[tokio::main]
async fn main() -> Result<()> {
    let mut robot = UnitreeRobot::connect("192.168.123.104:8080").await?;
    
    // High-level command
    robot.stand_up()?;
    
    // Enable low-level control
    robot.enable_low_level_control()?;
    
    // Send joint positions
    let positions = vec![0.0; robot.num_joints()];
    robot.set_joint_positions(&positions)?;
    
    Ok(())
}
```

## ROS2 Module (`humanoid_rs::ros2`)

*Requires `ros2` feature*

### `Ros2Bridge`

```rust
pub struct Ros2Bridge {
    // ...
}

impl Ros2Bridge {
    pub fn new(node_name: &str, namespace: &str) -> Self;
    pub fn init(&self) -> Result<()>;
    pub fn publish_state(&self, state: &RobotState) -> Result<()>;
    pub fn broadcast_tf(&self, state: &RobotState) -> Result<()>;
    pub fn get_cmd_vel(&self) -> msg::Twist;
    pub fn take_trajectory(&self) -> Option<msg::JointTrajectory>;
    pub fn spin_once(&self, timeout: Duration) -> Result<()>;
}
```

### Message Types

```rust
pub mod msg {
    pub struct JointState {
        pub header: Header,
        pub name: Vec<String>,
        pub position: Vec<f64>,
        pub velocity: Vec<f64>,
        pub effort: Vec<f64>,
    }
    
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }
    
    pub struct JointTrajectory {
        pub header: Header,
        pub joint_names: Vec<String>,
        pub points: Vec<JointTrajectoryPoint>,
    }
    
    pub struct JointTrajectoryPoint {
        pub positions: Vec<f64>,
        pub velocities: Vec<f64>,
        pub accelerations: Vec<f64>,
        pub effort: Vec<f64>,
        pub time_from_start: Duration,
    }
}
```

### Standard Topics

```rust
pub mod topics {
    pub const JOINT_STATES: &str = "/joint_states";
    pub const CMD_VEL: &str = "/cmd_vel";
    pub const ODOM: &str = "/odom";
    pub const IMU: &str = "/imu/data";
    pub const JOINT_TRAJECTORY: &str = "/joint_trajectory_controller/joint_trajectory";
}
```

**Example:**
```rust
use humanoid_rs::ros2::{Ros2Bridge, topics};

fn main() -> Result<()> {
    let bridge = Ros2Bridge::new("humanoid_node", "/humanoid");
    bridge.init()?;
    
    loop {
        // Publish current state
        bridge.publish_state(&robot_state)?;
        
        // Check for commands
        let cmd_vel = bridge.get_cmd_vel();
        
        // Process callbacks
        bridge.spin_once(Duration::from_millis(1))?;
    }
}
```

## Feature Flags

| Feature | Description | Dependencies |
|---------|-------------|--------------|
| `simulator` | Enable 3D simulator | bevy, bevy_rapier3d |
| `unitree` | Enable Unitree robot support | - |
| `ros2` | Enable ROS2 bridge | - |
| `rl` | Enable reinforcement learning env | - |
| `mcp` | Enable LLM/MCP integration | - |

Enable features in `Cargo.toml`:

```toml
[dependencies]
humanoid-rs = { version = "0.1.0", features = ["simulator", "unitree"] }
```

Or on command line:

```bash
cargo run --example stand_up --features "simulator unitree"
```
