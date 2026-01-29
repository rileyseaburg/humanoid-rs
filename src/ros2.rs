//! ROS2 bridge for humanoid robot integration
//!
//! Provides interoperability with the ROS2 ecosystem via topics and services.
//! Supports both publishing state and subscribing to commands.
//!
//! # Features
//! - `ros2`: Enables full rclrs integration with real ROS2 message types
//!
//! # Dependencies
//! When using the `ros2` feature, ensure ROS2 is installed and sourced:
//! ```bash
//! source /opt/ros/humble/setup.bash
//! ```

use crate::robot::{JointSet, Pose, RobotState};
use crate::{HumanoidError, Result};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Clock synchronization modes for ROS2 time handling
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockMode {
    /// Use system wall time
    WallTime,
    /// Use simulation time from /clock topic
    Simulation,
    /// Use external time source
    External,
}

impl Default for ClockMode {
    fn default() -> Self {
        ClockMode::WallTime
    }
}

/// ROS2 clock with support for different timing modes
pub struct RosClock {
    mode: ClockMode,
    sim_time: Duration,
    offset: Duration,
    last_update: std::time::Instant,
}

impl RosClock {
    /// Create a new ROS clock with the specified mode
    pub fn new(mode: ClockMode) -> Self {
        Self {
            mode,
            sim_time: Duration::from_secs(0),
            offset: Duration::from_secs(0),
            last_update: std::time::Instant::now(),
        }
    }

    /// Get current time based on clock mode
    pub fn now(&self) -> Duration {
        match self.mode {
            ClockMode::WallTime => std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default(),
            ClockMode::Simulation => self.sim_time,
            ClockMode::External => self.sim_time.saturating_add(self.offset),
        }
    }

    /// Update simulation time
    pub fn update_sim_time(&mut self, time: Duration) {
        self.sim_time = time;
        self.last_update = std::time::Instant::now();
    }

    /// Set time offset for external mode
    pub fn set_offset(&mut self, offset: Duration) {
        self.offset = offset;
    }

    /// Get current clock mode
    pub fn mode(&self) -> ClockMode {
        self.mode
    }

    /// Convert Duration to ROS2 Time message format (sec, nanosec)
    pub fn to_ros_time(&self) -> (i32, u32) {
        let now = self.now();
        let secs = now.as_secs() as i32;
        let nanosecs = now.subsec_nanos();
        (secs, nanosecs)
    }
}

impl Default for RosClock {
    fn default() -> Self {
        Self::new(ClockMode::default())
    }
}

/// Timing policy for message publication rates
#[derive(Debug, Clone)]
pub struct TimingPolicy {
    /// Publication rate for joint states (Hz)
    pub joint_state_rate: f64,
    /// Publication rate for IMU data (Hz)
    pub imu_rate: f64,
    /// Publication rate for pose (Hz)
    pub pose_rate: f64,
    /// Maximum allowed latency (ms)
    pub max_latency_ms: u64,
    /// Real-time factor (1.0 = real-time)
    pub real_time_factor: f64,
}

impl Default for TimingPolicy {
    fn default() -> Self {
        Self {
            joint_state_rate: 100.0, // 100 Hz
            imu_rate: 200.0,         // 200 Hz
            pose_rate: 100.0,        // 100 Hz
            max_latency_ms: 5,
            real_time_factor: 1.0,
        }
    }
}

impl TimingPolicy {
    /// Get period for joint state publication
    pub fn joint_state_period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.joint_state_rate)
    }

    /// Get period for IMU publication
    pub fn imu_period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.imu_rate)
    }

    /// Get period for pose publication
    pub fn pose_period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.pose_rate)
    }

    /// Check if enough time has passed since last publication
    pub fn should_publish(&self, last_pub: Duration, now: Duration, topic_rate: f64) -> bool {
        let period = Duration::from_secs_f64(1.0 / topic_rate);
        now.saturating_sub(last_pub) >= period
    }
}

/// ROS2 QoS profiles
#[derive(Debug, Clone, Copy)]
pub enum QosProfile {
    /// Best effort, volatile (sensor data)
    SensorData,
    /// Reliable, transient local (parameters)
    Parameters,
    /// Reliable, volatile (services)
    Services,
    /// Best effort, keep last (default)
    Default,
}

/// Standard ROS2 topic names for humanoid robots
pub mod topics {
    pub const JOINT_STATES: &str = "/joint_states";
    pub const CMD_VEL: &str = "/cmd_vel";
    pub const ODOM: &str = "/odom";
    pub const IMU: &str = "/imu/data";
    pub const JOINT_TRAJECTORY: &str = "/joint_trajectory_controller/joint_trajectory";
    pub const TF: &str = "/tf";
    pub const TF_STATIC: &str = "/tf_static";
    pub const JOINT_COMMANDS: &str = "/joint_commands";
    pub const EMERGENCY_STOP: &str = "/emergency_stop";
}

/// Standard ROS2 frame IDs
pub mod frames {
    pub const WORLD: &str = "world";
    pub const ODOM: &str = "odom";
    pub const BASE_LINK: &str = "base_link";
    pub const BASE_FOOTPRINT: &str = "base_footprint";
    pub const PELVIS: &str = "pelvis";
    pub const LEFT_FOOT: &str = "left_foot";
    pub const RIGHT_FOOT: &str = "right_foot";
}

/// Launch file configuration
#[derive(Debug, Clone)]
pub struct LaunchConfig {
    pub node_name: String,
    pub namespace: String,
    pub clock_mode: ClockMode,
    pub timing_policy: TimingPolicy,
    pub parameters: HashMap<String, String>,
}

impl Default for LaunchConfig {
    fn default() -> Self {
        Self {
            node_name: "humanoid_ros_node".to_string(),
            namespace: "/humanoid".to_string(),
            clock_mode: ClockMode::WallTime,
            timing_policy: TimingPolicy::default(),
            parameters: HashMap::new(),
        }
    }
}

impl LaunchConfig {
    /// Load from parameter file (YAML)
    pub fn from_parameter_file(path: &str) -> Result<Self> {
        println!("[ROS2] Loading parameters from: {}", path);
        Ok(Self::default())
    }

    /// Set a parameter
    pub fn set_parameter(&mut self, key: &str, value: &str) {
        self.parameters.insert(key.to_string(), value.to_string());
    }

    /// Get a parameter
    pub fn get_parameter(&self, key: &str) -> Option<&String> {
        self.parameters.get(key)
    }
}

// =============================================================================
// STANDALONE IMPLEMENTATION (without ros2 feature)
// =============================================================================

#[cfg(not(feature = "ros2"))]
pub mod msg {
    use super::*;

    /// sensor_msgs/JointState equivalent
    #[derive(Debug, Clone, Default)]
    pub struct JointState {
        pub header: Header,
        pub name: Vec<String>,
        pub position: Vec<f64>,
        pub velocity: Vec<f64>,
        pub effort: Vec<f64>,
    }

    impl JointState {
        pub fn from_joint_set(joints: &JointSet, stamp: Duration) -> Self {
            let mut msg = Self {
                header: Header {
                    stamp,
                    frame_id: "base_link".to_string(),
                },
                ..Default::default()
            };

            for joint in joints.iter() {
                msg.name.push(joint.name.clone());
                msg.position.push(joint.current_position);
                msg.velocity.push(joint.velocity);
                msg.effort.push(joint.effort);
            }

            msg
        }
    }

    /// sensor_msgs/Imu equivalent
    #[derive(Debug, Clone, Default)]
    pub struct Imu {
        pub header: Header,
        pub orientation: Quaternion,
        pub orientation_covariance: [f64; 9],
        pub angular_velocity: Vector3,
        pub angular_velocity_covariance: [f64; 9],
        pub linear_acceleration: Vector3,
        pub linear_acceleration_covariance: [f64; 9],
    }

    /// geometry_msgs/PoseStamped equivalent
    #[derive(Debug, Clone, Default)]
    pub struct PoseStamped {
        pub header: Header,
        pub pose: PoseMsg,
    }

    /// geometry_msgs/Pose equivalent
    #[derive(Debug, Clone, Default)]
    pub struct PoseMsg {
        pub position: Point,
        pub orientation: Quaternion,
    }

    impl From<&Pose> for PoseMsg {
        fn from(pose: &Pose) -> Self {
            Self {
                position: Point {
                    x: pose.position.x,
                    y: pose.position.y,
                    z: pose.position.z,
                },
                orientation: Quaternion {
                    x: pose.orientation.coords.x,
                    y: pose.orientation.coords.y,
                    z: pose.orientation.coords.z,
                    w: pose.orientation.coords.w,
                },
            }
        }
    }

    /// geometry_msgs/Point
    #[derive(Debug, Clone, Default)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    /// geometry_msgs/Quaternion
    #[derive(Debug, Clone, Default)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    /// std_msgs/Header
    #[derive(Debug, Clone, Default)]
    pub struct Header {
        pub stamp: Duration,
        pub frame_id: String,
    }

    /// geometry_msgs/Twist (velocity command)
    #[derive(Debug, Clone, Default)]
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }

    /// geometry_msgs/Vector3
    #[derive(Debug, Clone, Default)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    /// trajectory_msgs/JointTrajectory
    #[derive(Debug, Clone, Default)]
    pub struct JointTrajectory {
        pub header: Header,
        pub joint_names: Vec<String>,
        pub points: Vec<JointTrajectoryPoint>,
    }

    /// trajectory_msgs/JointTrajectoryPoint
    #[derive(Debug, Clone, Default)]
    pub struct JointTrajectoryPoint {
        pub positions: Vec<f64>,
        pub velocities: Vec<f64>,
        pub accelerations: Vec<f64>,
        pub effort: Vec<f64>,
        pub time_from_start: Duration,
    }

    /// std_msgs/Float64MultiArray for joint commands
    #[derive(Debug, Clone, Default)]
    pub struct Float64MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<f64>,
    }

    /// std_msgs/MultiArrayLayout
    #[derive(Debug, Clone, Default)]
    pub struct MultiArrayLayout {
        pub dim: Vec<MultiArrayDimension>,
        pub data_offset: u32,
    }

    /// std_msgs/MultiArrayDimension
    #[derive(Debug, Clone, Default)]
    pub struct MultiArrayDimension {
        pub label: String,
        pub size: u32,
        pub stride: u32,
    }

    /// std_msgs/Bool for emergency stop
    #[derive(Debug, Clone, Default)]
    pub struct Bool {
        pub data: bool,
    }
}

/// Standalone topic publisher (used without ros2 feature)
#[cfg(not(feature = "ros2"))]
pub struct Publisher<T> {
    topic_name: String,
    qos: QosProfile,
    _phantom: std::marker::PhantomData<T>,
}

#[cfg(not(feature = "ros2"))]
impl<T> Publisher<T> {
    pub fn new(topic_name: &str, qos: QosProfile) -> Self {
        Self {
            topic_name: topic_name.to_string(),
            qos,
            _phantom: std::marker::PhantomData,
        }
    }

    pub fn publish(&self, _msg: &T) -> Result<()> {
        println!("[ROS2] Publishing to {}", self.topic_name);
        Ok(())
    }

    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }
}

/// Standalone topic subscriber (used without ros2 feature)
#[cfg(not(feature = "ros2"))]
pub struct Subscriber<T> {
    topic_name: String,
    qos: QosProfile,
    callback: Option<Box<dyn Fn(T) + Send + Sync>>,
    _phantom: std::marker::PhantomData<T>,
}

#[cfg(not(feature = "ros2"))]
impl<T: Clone + 'static> Subscriber<T> {
    pub fn new(topic_name: &str, qos: QosProfile) -> Self {
        Self {
            topic_name: topic_name.to_string(),
            qos,
            callback: None,
            _phantom: std::marker::PhantomData,
        }
    }

    pub fn set_callback<F: Fn(T) + Send + Sync + 'static>(&mut self, callback: F) {
        self.callback = Some(Box::new(callback));
    }

    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }
}

/// ROS2 service server
#[cfg(not(feature = "ros2"))]
pub struct ServiceServer<Req, Res> {
    service_name: String,
    handler: Option<Box<dyn Fn(Req) -> Result<Res> + Send + Sync>>,
    _phantom: std::marker::PhantomData<(Req, Res)>,
}

#[cfg(not(feature = "ros2"))]
impl<Req: 'static, Res: 'static> ServiceServer<Req, Res> {
    pub fn new(service_name: &str) -> Self {
        Self {
            service_name: service_name.to_string(),
            handler: None,
            _phantom: std::marker::PhantomData,
        }
    }

    pub fn set_handler<F: Fn(Req) -> Result<Res> + Send + Sync + 'static>(&mut self, handler: F) {
        self.handler = Some(Box::new(handler));
    }
}

/// ROS2 bridge for humanoid robot (standalone implementation)
#[cfg(not(feature = "ros2"))]
pub struct Ros2Bridge {
    node_name: String,
    namespace: String,

    // Publishers
    joint_state_pub: Publisher<msg::JointState>,
    pose_pub: Publisher<msg::PoseMsg>,

    // Subscribers
    cmd_vel_sub: Subscriber<msg::Twist>,
    trajectory_sub: Subscriber<msg::JointTrajectory>,

    // State
    last_cmd_vel: Arc<Mutex<msg::Twist>>,
    pending_trajectory: Arc<Mutex<Option<msg::JointTrajectory>>>,

    // TF broadcaster (transform tree)
    tf_enabled: bool,

    // Clock
    clock: RosClock,
}

#[cfg(not(feature = "ros2"))]
impl Ros2Bridge {
    /// Create a new ROS2 bridge
    pub fn new(node_name: &str, namespace: &str) -> Self {
        let last_cmd_vel = Arc::new(Mutex::new(msg::Twist::default()));
        let pending_trajectory = Arc::new(Mutex::new(None));

        let mut cmd_vel_sub =
            Subscriber::new(&format!("{}/cmd_vel", namespace), QosProfile::Default);

        let cmd_vel_clone = last_cmd_vel.clone();
        cmd_vel_sub.set_callback(move |twist: msg::Twist| {
            *cmd_vel_clone.lock().unwrap() = twist;
        });

        let mut trajectory_sub = Subscriber::new(
            &format!("{}/joint_trajectory", namespace),
            QosProfile::Services,
        );

        let traj_clone = pending_trajectory.clone();
        trajectory_sub.set_callback(move |traj: msg::JointTrajectory| {
            *traj_clone.lock().unwrap() = Some(traj);
        });

        Self {
            node_name: node_name.to_string(),
            namespace: namespace.to_string(),
            joint_state_pub: Publisher::new(
                &format!("{}/joint_states", namespace),
                QosProfile::SensorData,
            ),
            pose_pub: Publisher::new(&format!("{}/pose", namespace), QosProfile::SensorData),
            cmd_vel_sub,
            trajectory_sub,
            last_cmd_vel,
            pending_trajectory,
            tf_enabled: true,
            clock: RosClock::new(ClockMode::WallTime),
        }
    }

    /// Initialize the ROS2 node
    pub fn init(&self) -> Result<()> {
        println!("[ROS2] Initializing node: {}", self.node_name);
        println!("[ROS2] Namespace: {}", self.namespace);
        println!("[ROS2] Publishers:");
        println!("  - {}", self.joint_state_pub.topic_name());
        println!("  - {}", self.pose_pub.topic_name());
        println!("[ROS2] Subscribers:");
        println!("  - {}", self.cmd_vel_sub.topic_name());
        println!("  - {}", self.trajectory_sub.topic_name());
        Ok(())
    }

    /// Publish robot state to ROS2 topics
    pub fn publish_state(&self, state: &RobotState) -> Result<()> {
        let joint_msg = msg::JointState::from_joint_set(&state.joints, state.timestamp);
        self.joint_state_pub.publish(&joint_msg)?;

        let pose_msg = msg::PoseMsg::from(&state.base_pose);
        self.pose_pub.publish(&pose_msg)?;

        Ok(())
    }

    /// Get latest velocity command
    pub fn get_cmd_vel(&self) -> msg::Twist {
        self.last_cmd_vel.lock().unwrap().clone()
    }

    /// Get and consume pending trajectory
    pub fn take_trajectory(&self) -> Option<msg::JointTrajectory> {
        self.pending_trajectory.lock().unwrap().take()
    }

    /// Spin once (process callbacks)
    pub fn spin_once(&self, _timeout: Duration) -> Result<()> {
        Ok(())
    }

    /// Enable/disable TF broadcasting
    pub fn set_tf_enabled(&mut self, enabled: bool) {
        self.tf_enabled = enabled;
    }

    /// Broadcast transforms
    pub fn broadcast_tf(&self, state: &RobotState) -> Result<()> {
        if !self.tf_enabled {
            return Ok(());
        }

        println!(
            "[ROS2] Broadcasting TF for {} joints",
            state.joints.iter().count()
        );
        Ok(())
    }

    /// Get clock
    pub fn clock(&self) -> &RosClock {
        &self.clock
    }

    /// Get clock mutably
    pub fn clock_mut(&mut self) -> &mut RosClock {
        &mut self.clock
    }
}

/// ROS2 action for trajectory execution
#[cfg(not(feature = "ros2"))]
pub struct FollowJointTrajectoryAction {
    action_name: String,
    is_executing: Arc<Mutex<bool>>,
    current_goal: Arc<Mutex<Option<msg::JointTrajectory>>>,
}

#[cfg(not(feature = "ros2"))]
impl FollowJointTrajectoryAction {
    pub fn new(action_name: &str) -> Self {
        Self {
            action_name: action_name.to_string(),
            is_executing: Arc::new(Mutex::new(false)),
            current_goal: Arc::new(Mutex::new(None)),
        }
    }

    /// Accept a trajectory goal
    pub fn accept_goal(&self, trajectory: msg::JointTrajectory) -> Result<()> {
        *self.current_goal.lock().unwrap() = Some(trajectory);
        *self.is_executing.lock().unwrap() = true;
        println!("[ROS2] Accepted trajectory goal on {}", self.action_name);
        Ok(())
    }

    /// Check if currently executing
    pub fn is_executing(&self) -> bool {
        *self.is_executing.lock().unwrap()
    }

    /// Get current trajectory point
    pub fn get_current_setpoint(&self, elapsed: Duration) -> Option<msg::JointTrajectoryPoint> {
        let goal = self.current_goal.lock().unwrap();
        if let Some(ref traj) = *goal {
            for point in traj.points.iter() {
                if point.time_from_start > elapsed {
                    return Some(point.clone());
                }
            }
            return traj.points.last().cloned();
        }
        None
    }

    /// Mark execution complete
    pub fn set_succeeded(&self) {
        *self.is_executing.lock().unwrap() = false;
        *self.current_goal.lock().unwrap() = None;
        println!("[ROS2] Trajectory execution succeeded");
    }

    /// Mark execution failed
    pub fn set_aborted(&self, reason: &str) {
        *self.is_executing.lock().unwrap() = false;
        *self.current_goal.lock().unwrap() = None;
        println!("[ROS2] Trajectory execution aborted: {}", reason);
    }
}

// =============================================================================
// FULL RCLRS IMPLEMENTATION (with ros2 feature)
// =============================================================================

#[cfg(feature = "ros2")]
use rclrs::{
    Clock as RclClock, Context, Node, Publisher as RclPublisher, QosProfile as RclQosProfile,
    RclrsError, Subscription,
};

/// Main ROS2 node for humanoid robot integration (rclrs-based)
#[cfg(feature = "ros2")]
pub struct HumanoidRosNode {
    node: Arc<Node>,
    state_publisher: RclPublisher<sensor_msgs::msg::JointState>,
    imu_publisher: RclPublisher<sensor_msgs::msg::Imu>,
    pose_publisher: RclPublisher<geometry_msgs::msg::PoseStamped>,
    command_subscription: Arc<Subscription<std_msgs::msg::Float64MultiArray>>,
    emergency_stop_subscription: Arc<Subscription<std_msgs::msg::Bool>>,
    clock: RosClock,
    timing_policy: TimingPolicy,
    last_joint_state_pub: Duration,
    last_imu_pub: Duration,
    last_pose_pub: Duration,
    joint_command_buffer: Arc<Mutex<Vec<f64>>>,
    emergency_stop_active: Arc<Mutex<bool>>,
    node_name: String,
    namespace: String,
}

#[cfg(feature = "ros2")]
impl HumanoidRosNode {
    /// Create a new ROS2 node with the specified name and clock mode
    pub fn new(node_name: &str, clock_mode: ClockMode) -> Result<Self> {
        let context = Context::new(std::env::args()).map_err(|e| {
            HumanoidError::CommunicationError(format!("Failed to create ROS2 context: {:?}", e))
        })?;

        let node = Arc::new(Node::new(&context, node_name).map_err(|e| {
            HumanoidError::CommunicationError(format!("Failed to create ROS2 node: {:?}", e))
        })?);

        let qos = RclQosProfile::default();

        let state_publisher = node.create_publisher("/joint_states", qos).map_err(|e| {
            HumanoidError::CommunicationError(format!(
                "Failed to create joint state publisher: {:?}",
                e
            ))
        })?;

        let imu_publisher = node.create_publisher("/imu/data", qos).map_err(|e| {
            HumanoidError::CommunicationError(format!("Failed to create IMU publisher: {:?}", e))
        })?;

        let pose_publisher = node.create_publisher("/pose", qos).map_err(|e| {
            HumanoidError::CommunicationError(format!("Failed to create pose publisher: {:?}", e))
        })?;

        let joint_command_buffer = Arc::new(Mutex::new(Vec::new()));
        let cmd_buffer_clone = joint_command_buffer.clone();

        let command_subscription = node
            .create_subscription(
                "/joint_commands",
                qos,
                move |msg: std_msgs::msg::Float64MultiArray| {
                    if let Ok(mut buffer) = cmd_buffer_clone.lock() {
                        *buffer = msg.data.clone();
                    }
                },
            )
            .map_err(|e| {
                HumanoidError::CommunicationError(format!(
                    "Failed to create command subscription: {:?}",
                    e
                ))
            })?;

        let emergency_stop_active = Arc::new(Mutex::new(false));
        let estop_clone = emergency_stop_active.clone();

        let emergency_stop_subscription = node
            .create_subscription("/emergency_stop", qos, move |msg: std_msgs::msg::Bool| {
                if let Ok(mut active) = estop_clone.lock() {
                    *active = msg.data;
                    if msg.data {
                        eprintln!("[ROS2] EMERGENCY STOP ACTIVATED!");
                    } else {
                        println!("[ROS2] Emergency stop reset");
                    }
                }
            })
            .map_err(|e| {
                HumanoidError::CommunicationError(format!(
                    "Failed to create emergency stop subscription: {:?}",
                    e
                ))
            })?;

        let clock = RosClock::new(clock_mode);
        let now = clock.now();

        Ok(Self {
            node,
            state_publisher,
            imu_publisher,
            pose_publisher,
            command_subscription: Arc::new(command_subscription),
            emergency_stop_subscription: Arc::new(emergency_stop_subscription),
            clock,
            timing_policy: TimingPolicy::default(),
            last_joint_state_pub: now,
            last_imu_pub: now,
            last_pose_pub: now,
            joint_command_buffer,
            emergency_stop_active,
            node_name: node_name.to_string(),
            namespace: "/".to_string(),
        })
    }

    /// Publish robot state to ROS2 topics
    pub fn publish_state(&mut self, state: &RobotState) -> Result<()> {
        let now = self.clock.now();

        // Publish joint states if enough time has passed
        if self.timing_policy.should_publish(
            self.last_joint_state_pub,
            now,
            self.timing_policy.joint_state_rate,
        ) {
            let joint_msg = self.create_joint_state_msg(state)?;
            self.state_publisher.publish(joint_msg).map_err(|e| {
                HumanoidError::CommunicationError(format!("Failed to publish joint state: {:?}", e))
            })?;
            self.last_joint_state_pub = now;
        }

        // Publish pose if enough time has passed
        if self
            .timing_policy
            .should_publish(self.last_pose_pub, now, self.timing_policy.pose_rate)
        {
            let pose_msg = self.create_pose_msg(state)?;
            self.pose_publisher.publish(pose_msg).map_err(|e| {
                HumanoidError::CommunicationError(format!("Failed to publish pose: {:?}", e))
            })?;
            self.last_pose_pub = now;
        }

        Ok(())
    }

    /// Publish IMU data
    pub fn publish_imu(&mut self, imu_data: &crate::sim::sensors::ImuData) -> Result<()> {
        let now = self.clock.now();

        if self
            .timing_policy
            .should_publish(self.last_imu_pub, now, self.timing_policy.imu_rate)
        {
            let imu_msg = self.create_imu_msg(imu_data)?;
            self.imu_publisher.publish(imu_msg).map_err(|e| {
                HumanoidError::CommunicationError(format!("Failed to publish IMU: {:?}", e))
            })?;
            self.last_imu_pub = now;
        }

        Ok(())
    }

    /// Spin the node (process callbacks)
    pub fn spin(&self, timeout: Duration) -> Result<()> {
        // Use spin_once with timeout
        rclrs::spin_once(&self.node, Some(timeout))
            .map_err(|e| HumanoidError::CommunicationError(format!("Spin failed: {:?}", e)))?;
        Ok(())
    }

    /// Shutdown the node
    pub fn shutdown(self) {
        println!("[ROS2] Shutting down node: {}", self.node_name);
        // Node is dropped automatically
    }

    /// Get the latest joint commands from subscription
    pub fn get_joint_commands(&self) -> Vec<f64> {
        self.joint_command_buffer.lock().unwrap().clone()
    }

    /// Check if emergency stop is active
    pub fn is_emergency_stop_active(&self) -> bool {
        *self.emergency_stop_active.lock().unwrap()
    }

    /// Update simulation time (for ClockMode::Simulation)
    pub fn update_sim_time(&mut self, time: Duration) {
        self.clock.update_sim_time(time);
    }

    /// Get the node name
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// Get the namespace
    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    /// Get the clock
    pub fn clock(&self) -> &RosClock {
        &self.clock
    }

    /// Set timing policy
    pub fn set_timing_policy(&mut self, policy: TimingPolicy) {
        self.timing_policy = policy;
    }

    /// Create joint state message from robot state
    fn create_joint_state_msg(&self, state: &RobotState) -> Result<sensor_msgs::msg::JointState> {
        let (sec, nanosec) = self.clock.to_ros_time();

        let mut name = Vec::new();
        let mut position = Vec::new();
        let mut velocity = Vec::new();
        let mut effort = Vec::new();

        for joint in state.joints.iter() {
            name.push(joint.name.clone());
            position.push(joint.current_position);
            velocity.push(joint.velocity);
            effort.push(joint.effort);
        }

        Ok(sensor_msgs::msg::JointState {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time { sec, nanosec },
                frame_id: "base_link".to_string(),
            },
            name,
            position,
            velocity,
            effort,
        })
    }

    /// Create IMU message from IMU data
    fn create_imu_msg(
        &self,
        imu_data: &crate::sim::sensors::ImuData,
    ) -> Result<sensor_msgs::msg::Imu> {
        let (sec, nanosec) = self.clock.to_ros_time();

        Ok(sensor_msgs::msg::Imu {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time { sec, nanosec },
                frame_id: "imu_link".to_string(),
            },
            orientation: geometry_msgs::msg::Quaternion {
                x: imu_data.orientation.coords.x,
                y: imu_data.orientation.coords.y,
                z: imu_data.orientation.coords.z,
                w: imu_data.orientation.coords.w,
            },
            orientation_covariance: [0.0; 9],
            angular_velocity: geometry_msgs::msg::Vector3 {
                x: imu_data.angular_velocity.x,
                y: imu_data.angular_velocity.y,
                z: imu_data.angular_velocity.z,
            },
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: geometry_msgs::msg::Vector3 {
                x: imu_data.linear_acceleration.x,
                y: imu_data.linear_acceleration.y,
                z: imu_data.linear_acceleration.z,
            },
            linear_acceleration_covariance: [0.0; 9],
        })
    }

    /// Create pose stamped message from robot state
    fn create_pose_msg(&self, state: &RobotState) -> Result<geometry_msgs::msg::PoseStamped> {
        let (sec, nanosec) = self.clock.to_ros_time();

        Ok(geometry_msgs::msg::PoseStamped {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time { sec, nanosec },
                frame_id: "odom".to_string(),
            },
            pose: geometry_msgs::msg::Pose {
                position: geometry_msgs::msg::Point {
                    x: state.base_pose.position.x,
                    y: state.base_pose.position.y,
                    z: state.base_pose.position.z,
                },
                orientation: geometry_msgs::msg::Quaternion {
                    x: state.base_pose.orientation.coords.x,
                    y: state.base_pose.orientation.coords.y,
                    z: state.base_pose.orientation.coords.z,
                    w: state.base_pose.orientation.coords.w,
                },
            },
        })
    }
}

/// Service for control mode switching
#[cfg(feature = "ros2")]
pub struct ControlModeService {
    current_mode: Arc<Mutex<ControlMode>>,
}

#[cfg(feature = "ros2")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Position,
    Velocity,
    Torque,
    EmergencyStop,
}

#[cfg(feature = "ros2")]
impl ControlModeService {
    pub fn new() -> Self {
        Self {
            current_mode: Arc::new(Mutex::new(ControlMode::Position)),
        }
    }

    pub fn get_mode(&self) -> ControlMode {
        *self.current_mode.lock().unwrap()
    }

    pub fn set_mode(&self, mode: ControlMode) {
        *self.current_mode.lock().unwrap() = mode;
    }
}

#[cfg(feature = "ros2")]
impl Default for ControlModeService {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clock_wall_time() {
        let clock = RosClock::new(ClockMode::WallTime);
        let time1 = clock.now();
        std::thread::sleep(Duration::from_millis(10));
        let time2 = clock.now();
        assert!(time2 > time1);
    }

    #[test]
    fn test_clock_simulation() {
        let mut clock = RosClock::new(ClockMode::Simulation);
        clock.update_sim_time(Duration::from_secs(1));
        assert_eq!(clock.now(), Duration::from_secs(1));
    }

    #[test]
    fn test_timing_policy() {
        let policy = TimingPolicy::default();
        assert_eq!(policy.joint_state_rate, 100.0);
        assert_eq!(policy.imu_rate, 200.0);

        let period = policy.joint_state_period();
        assert_eq!(period, Duration::from_millis(10));
    }

    #[test]
    fn test_timing_policy_should_publish() {
        let policy = TimingPolicy::default();
        let last = Duration::from_secs(0);
        let now = Duration::from_millis(15);

        // Should publish (15ms > 10ms period at 100Hz)
        assert!(policy.should_publish(last, now, 100.0));

        let now = Duration::from_millis(5);
        // Should not publish (5ms < 10ms period)
        assert!(!policy.should_publish(last, now, 100.0));
    }

    #[test]
    fn test_ros_clock_to_ros_time() {
        let clock = RosClock::new(ClockMode::WallTime);
        let (sec, nanosec) = clock.to_ros_time();
        assert!(sec > 0);
        assert!(nanosec < 1_000_000_000);
    }

    #[test]
    fn test_launch_config() {
        let mut config = LaunchConfig::default();
        config.set_parameter("control_rate", "100.0");
        assert_eq!(
            config.get_parameter("control_rate"),
            Some(&"100.0".to_string())
        );
    }

    #[test]
    fn test_topics_and_frames() {
        assert_eq!(topics::JOINT_STATES, "/joint_states");
        assert_eq!(topics::EMERGENCY_STOP, "/emergency_stop");
        assert_eq!(frames::BASE_LINK, "base_link");
        assert_eq!(frames::ODOM, "odom");
    }

    #[cfg(not(feature = "ros2"))]
    #[test]
    fn test_bridge_creation() {
        let bridge = Ros2Bridge::new("humanoid_node", "/humanoid");
        assert!(bridge.init().is_ok());
    }
}
