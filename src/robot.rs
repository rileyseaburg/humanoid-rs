use nalgebra::{Isometry3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// A joint in the robot's kinematic chain
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum JointType {
    Revolute { axis: Vector3<f64> },
    Prismatic { axis: Vector3<f64> },
    Fixed,
    Spherical,
}

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

/// A collection of joints
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct JointSet {
    joints: Vec<Joint>,
}

impl JointSet {
    pub fn new() -> Self {
        Self { joints: Vec::new() }
    }

    pub fn add_joint(&mut self, joint: Joint) {
        self.joints.push(joint);
    }

    pub fn get_joint(&self, name: &str) -> Option<&Joint> {
        self.joints.iter().find(|j| j.name == name)
    }

    pub fn get_joint_mut(&mut self, name: &str) -> Option<&mut Joint> {
        self.joints.iter_mut().find(|j| j.name == name)
    }

    pub fn iter(&self) -> impl Iterator<Item = &Joint> {
        self.joints.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Joint> {
        self.joints.iter_mut()
    }

    /// Get positions as vector
    pub fn positions(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.current_position).collect()
    }

    /// Set positions from vector
    pub fn set_positions(&mut self, positions: &[f64]) {
        for (joint, &pos) in self.joints.iter_mut().zip(positions.iter()) {
            joint.current_position = pos;
        }
    }
}

/// 3D pose (position + orientation)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        }
    }
}

impl Pose {
    pub fn new(x: f64, y: f64, z: f64, roll: f64, pitch: f64, yaw: f64) -> Self {
        Self {
            position: Vector3::new(x, y, z),
            orientation: UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        }
    }

    pub fn to_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(self.position.into(), self.orientation)
    }
}

/// Complete robot state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotState {
    pub timestamp: Duration,
    pub base_pose: Pose,
    pub joints: JointSet,
    pub is_standing: bool,
    pub is_balanced: bool,
    pub foot_contacts: [bool; 2], // [left, right]
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            timestamp: Duration::from_secs(0),
            base_pose: Pose::default(),
            joints: JointSet::new(),
            is_standing: false,
            is_balanced: false,
            foot_contacts: [false, false],
        }
    }
}

/// Trait for any humanoid robot
pub trait HumanoidRobot {
    /// Get current state
    fn state(&self) -> &RobotState;

    /// Send joint position commands
    fn set_joint_positions(&mut self, positions: &[f64]) -> crate::Result<()>;

    /// Send joint velocity commands
    fn set_joint_velocities(&mut self, velocities: &[f64]) -> crate::Result<()>;

    /// Send joint effort commands
    fn set_joint_efforts(&mut self, efforts: &[f64]) -> crate::Result<()>;

    /// Get forward kinematics for a link
    fn forward_kinematics(&self, link_name: &str) -> crate::Result<Pose>;

    /// Get number of actuated joints
    fn num_joints(&self) -> usize;

    /// Emergency stop
    fn emergency_stop(&mut self) -> crate::Result<()>;

    /// Reset emergency stop
    fn reset_emergency_stop(&mut self) -> crate::Result<()>;
}

/// Unitree G1 specific configuration
pub mod unitree_g1 {
    use super::*;

    pub const NUM_JOINTS: usize = 29;

    pub fn default_joint_names() -> Vec<String> {
        vec![
            // Left leg
            "left_hip_pitch",
            "left_hip_roll",
            "left_hip_yaw",
            "left_knee",
            "left_ankle_pitch",
            "left_ankle_roll",
            // Right leg
            "right_hip_pitch",
            "right_hip_roll",
            "right_hip_yaw",
            "right_knee",
            "right_ankle_pitch",
            "right_ankle_roll",
            // Waist
            "waist_yaw",
            "waist_pitch",
            "waist_roll",
            // Left arm
            "left_shoulder_pitch",
            "left_shoulder_roll",
            "left_shoulder_yaw",
            "left_elbow",
            "left_wrist_roll",
            "left_wrist_pitch",
            "left_wrist_yaw",
            // Right arm
            "right_shoulder_pitch",
            "right_shoulder_roll",
            "right_shoulder_yaw",
            "right_elbow",
            "right_wrist_roll",
            "right_wrist_pitch",
            "right_wrist_yaw",
        ]
        .into_iter()
        .map(String::from)
        .collect()
    }

    pub fn create_default_joints() -> JointSet {
        let mut joints = JointSet::new();

        for name in default_joint_names() {
            let limits = JointLimits::default();
            joints.add_joint(Joint {
                name,
                joint_type: JointType::Revolute {
                    axis: *Vector3::y_axis(),
                },
                parent: None,
                limits,
                current_position: 0.0,
                target_position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            });
        }

        joints
    }
}
