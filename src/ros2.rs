//! ROS2 bridge for humanoid robot integration
//!
//! Provides interoperability with the ROS2 ecosystem via topics and services.
//! Supports both publishing state and subscribing to commands.

use crate::robot::{HumanoidRobot, JointSet, Pose, RobotState};
use crate::{HumanoidError, Result};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

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

/// ROS2 message types we support
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
}

/// ROS2 topic publisher
pub struct Publisher<T> {
    topic_name: String,
    qos: QosProfile,
    _phantom: std::marker::PhantomData<T>,
}

impl<T> Publisher<T> {
    pub fn new(topic_name: &str, qos: QosProfile) -> Self {
        Self {
            topic_name: topic_name.to_string(),
            qos,
            _phantom: std::marker::PhantomData,
        }
    }

    pub fn publish(&self, _msg: &T) -> Result<()> {
        // In a real implementation, this would use rclrs or similar
        // For now, we just log
        println!("[ROS2] Publishing to {}", self.topic_name);
        Ok(())
    }

    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }
}

/// ROS2 topic subscriber
pub struct Subscriber<T> {
    topic_name: String,
    qos: QosProfile,
    callback: Option<Box<dyn Fn(T) + Send + Sync>>,
    _phantom: std::marker::PhantomData<T>,
}

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
pub struct ServiceServer<Req, Res> {
    service_name: String,
    handler: Option<Box<dyn Fn(Req) -> Result<Res> + Send + Sync>>,
    _phantom: std::marker::PhantomData<(Req, Res)>,
}

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

/// ROS2 bridge for humanoid robot
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
}

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
        // Publish joint states
        let joint_msg = msg::JointState::from_joint_set(&state.joints, state.timestamp);
        self.joint_state_pub.publish(&joint_msg)?;

        // Publish pose
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
        // In real implementation, would call rclrs spin_once
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

        // Would publish to /tf topic
        println!(
            "[ROS2] Broadcasting TF for {} joints",
            state.joints.iter().count()
        );
        Ok(())
    }
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

/// ROS2 action for trajectory execution
pub struct FollowJointTrajectoryAction {
    action_name: String,
    is_executing: Arc<Mutex<bool>>,
    current_goal: Arc<Mutex<Option<msg::JointTrajectory>>>,
}

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
            // Find appropriate point based on elapsed time
            for (i, point) in traj.points.iter().enumerate() {
                if point.time_from_start > elapsed {
                    return Some(point.clone());
                }
            }
            // Return last point if past end
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::unitree_g1;

    #[test]
    fn test_bridge_creation() {
        let bridge = Ros2Bridge::new("humanoid_node", "/humanoid");
        assert_eq!(bridge.node_name, "humanoid_node");
    }

    #[test]
    fn test_joint_state_conversion() {
        let joints = unitree_g1::create_default_joints();
        let msg = msg::JointState::from_joint_set(&joints, Duration::from_secs(1));
        assert_eq!(msg.name.len(), unitree_g1::NUM_JOINTS);
        assert_eq!(msg.position.len(), unitree_g1::NUM_JOINTS);
    }

    #[test]
    fn test_action_server() {
        let action = FollowJointTrajectoryAction::new("/joint_trajectory_action");
        assert!(!action.is_executing());

        let traj = msg::JointTrajectory {
            points: vec![msg::JointTrajectoryPoint {
                positions: vec![0.0; 29],
                time_from_start: Duration::from_secs(1),
                ..Default::default()
            }],
            ..Default::default()
        };

        action.accept_goal(traj).unwrap();
        assert!(action.is_executing());

        action.set_succeeded();
        assert!(!action.is_executing());
    }
}
