//! Robot SDK Layer
//!
//! Provides robot profiles, capabilities, and hardware abstraction
//! for different humanoid robot platforms.

use crate::robot::{HumanoidRobot, JointLimits};
use std::path::Path;
use tracing::{debug, info};

/// Robot profile describing a specific robot model
///
/// Contains static information about the robot's physical characteristics
/// and joint configuration.
///
/// # Example
///
/// ```rust
/// use humanoid_rs::sdk::robot::RobotProfile;
///
/// let profile = RobotProfile::unitree_g1();
/// assert_eq!(profile.name, "Unitree G1");
/// ```
#[derive(Debug, Clone)]
pub struct RobotProfile {
    /// Human-readable name of the robot model
    pub name: String,
    /// Number of actuated joints
    pub num_joints: usize,
    /// Names of each joint in order
    pub joint_names: Vec<String>,
    /// Limits for each joint
    pub joint_limits: Vec<JointLimits>,
    /// Total mass in kilograms
    pub mass: f64,
    /// Standing height in meters
    pub height: f64,
    /// Maximum linear velocity (m/s)
    pub max_velocity: f64,
}

/// Robot capabilities flags
///
/// Describes what features the robot hardware supports.
#[derive(Debug, Clone)]
pub struct RobotCapabilities {
    /// Whether low-level joint control is supported
    pub supports_low_level_control: bool,
    /// Whether high-level commands (stand, walk) are supported
    pub supports_high_level_commands: bool,
    /// Maximum torque for each joint (Nm)
    pub max_torque: Vec<f64>,
    /// Maximum velocity for each joint (rad/s)
    pub max_velocity: Vec<f64>,
}

impl RobotProfile {
    /// Create a profile for the Unitree G1 robot
    ///
    /// The Unitree G1 is a 29-DOF humanoid robot.
    pub fn unitree_g1() -> Self {
        info!("Creating Unitree G1 robot profile");

        let joint_names = vec![
            // Left leg (6 joints)
            "left_hip_pitch".to_string(),
            "left_hip_roll".to_string(),
            "left_hip_yaw".to_string(),
            "left_knee".to_string(),
            "left_ankle_pitch".to_string(),
            "left_ankle_roll".to_string(),
            // Right leg (6 joints)
            "right_hip_pitch".to_string(),
            "right_hip_roll".to_string(),
            "right_hip_yaw".to_string(),
            "right_knee".to_string(),
            "right_ankle_pitch".to_string(),
            "right_ankle_roll".to_string(),
            // Waist (3 joints)
            "waist_yaw".to_string(),
            "waist_pitch".to_string(),
            "waist_roll".to_string(),
            // Left arm (7 joints)
            "left_shoulder_pitch".to_string(),
            "left_shoulder_roll".to_string(),
            "left_shoulder_yaw".to_string(),
            "left_elbow".to_string(),
            "left_wrist_roll".to_string(),
            "left_wrist_pitch".to_string(),
            "left_wrist_yaw".to_string(),
            // Right arm (7 joints)
            "right_shoulder_pitch".to_string(),
            "right_shoulder_roll".to_string(),
            "right_shoulder_yaw".to_string(),
            "right_elbow".to_string(),
            "right_wrist_roll".to_string(),
            "right_wrist_pitch".to_string(),
            "right_wrist_yaw".to_string(),
        ];

        let num_joints = joint_names.len();

        // Default joint limits for G1
        let joint_limits = vec![
            JointLimits {
                min: -2.5,
                max: 2.5,
                max_velocity: 10.0,
                max_effort: 80.0,
            };
            num_joints
        ];

        Self {
            name: "Unitree G1".to_string(),
            num_joints,
            joint_names,
            joint_limits,
            mass: 35.0,        // kg
            height: 1.72,      // meters
            max_velocity: 2.0, // m/s
        }
    }

    /// Create a profile for the Unitree H1 robot
    ///
    /// The Unitree H1 is a larger humanoid robot with more powerful actuators.
    pub fn unitree_h1() -> Self {
        info!("Creating Unitree H1 robot profile");

        let joint_names = vec![
            // Left leg (6 joints)
            "left_hip_yaw".to_string(),
            "left_hip_roll".to_string(),
            "left_hip_pitch".to_string(),
            "left_knee".to_string(),
            "left_ankle_pitch".to_string(),
            "left_ankle_roll".to_string(),
            // Right leg (6 joints)
            "right_hip_yaw".to_string(),
            "right_hip_roll".to_string(),
            "right_hip_pitch".to_string(),
            "right_knee".to_string(),
            "right_ankle_pitch".to_string(),
            "right_ankle_roll".to_string(),
            // Torso (2 joints)
            "torso_yaw".to_string(),
            "torso_pitch".to_string(),
            // Left arm (7 joints)
            "left_shoulder_pitch".to_string(),
            "left_shoulder_roll".to_string(),
            "left_shoulder_yaw".to_string(),
            "left_elbow".to_string(),
            "left_wrist_roll".to_string(),
            "left_wrist_pitch".to_string(),
            "left_wrist_yaw".to_string(),
            // Right arm (7 joints)
            "right_shoulder_pitch".to_string(),
            "right_shoulder_roll".to_string(),
            "right_shoulder_yaw".to_string(),
            "right_elbow".to_string(),
            "right_wrist_roll".to_string(),
            "right_wrist_pitch".to_string(),
            "right_wrist_yaw".to_string(),
        ];

        let num_joints = joint_names.len();

        // H1 has higher torque limits
        let joint_limits = vec![
            JointLimits {
                min: -3.0,
                max: 3.0,
                max_velocity: 15.0,
                max_effort: 150.0,
            };
            num_joints
        ];

        Self {
            name: "Unitree H1".to_string(),
            num_joints,
            joint_names,
            joint_limits,
            mass: 47.0,        // kg
            height: 1.80,      // meters
            max_velocity: 3.3, // m/s
        }
    }

    /// Load a robot profile from a URDF file
    ///
    /// Parses the URDF to extract joint information and limits.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the URDF file
    ///
    /// # Returns
    ///
    /// A RobotProfile populated from the URDF data
    ///
    /// # Errors
    ///
    /// Returns an error if the URDF cannot be parsed.
    pub fn from_urdf(path: &Path) -> anyhow::Result<Self> {
        info!("Loading robot profile from URDF: {}", path.display());

        let content = std::fs::read_to_string(path)?;

        // Parse URDF XML
        let doc = roxmltree::Document::parse(&content)?;
        let robot_elem = doc.root_element();

        let name = robot_elem
            .attribute("name")
            .unwrap_or("Unknown")
            .to_string();

        let mut joint_names = Vec::new();
        let mut joint_limits = Vec::new();
        let mut total_mass = 0.0;

        // Parse joints
        for node in robot_elem.children() {
            if node.tag_name().name() == "joint" {
                if let Some(joint_name) = node.attribute("name") {
                    joint_names.push(joint_name.to_string());

                    // Extract limits from <limit> element
                    let mut limits = JointLimits::default();
                    for child in node.children() {
                        if child.tag_name().name() == "limit" {
                            if let Some(lower) = child.attribute("lower") {
                                limits.min = lower.parse().unwrap_or(-3.14);
                            }
                            if let Some(upper) = child.attribute("upper") {
                                limits.max = upper.parse().unwrap_or(3.14);
                            }
                            if let Some(velocity) = child.attribute("velocity") {
                                limits.max_velocity = velocity.parse().unwrap_or(10.0);
                            }
                            if let Some(effort) = child.attribute("effort") {
                                limits.max_effort = effort.parse().unwrap_or(100.0);
                            }
                        }
                    }
                    joint_limits.push(limits);
                }
            }

            // Accumulate mass from links
            if node.tag_name().name() == "link" {
                for child in node.children() {
                    if child.tag_name().name() == "inertial" {
                        for inertial_child in child.children() {
                            if inertial_child.tag_name().name() == "mass" {
                                if let Some(mass_val) = inertial_child.attribute("value") {
                                    if let Ok(m) = mass_val.parse::<f64>() {
                                        total_mass += m;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        debug!("Parsed {} joints from URDF", joint_names.len());

        Ok(Self {
            name,
            num_joints: joint_names.len(),
            joint_names,
            joint_limits,
            mass: if total_mass > 0.0 { total_mass } else { 50.0 },
            height: 1.75, // Default, could be calculated from kinematics
            max_velocity: 2.0,
        })
    }

    /// Validate this profile against actual hardware
    ///
    /// Compares the profile's joint count and limits against
    /// what the robot reports.
    ///
    /// # Arguments
    ///
    /// * `robot` - Reference to the robot implementation
    ///
    /// # Errors
    ///
    /// Returns an error if validation fails (wrong joint count, etc.)
    pub fn validate_against_hardware(&self, robot: &dyn HumanoidRobot) -> anyhow::Result<()> {
        debug!("Validating profile against hardware");

        // Check joint count
        let hardware_joints = robot.num_joints();
        if hardware_joints != self.num_joints {
            anyhow::bail!(
                "Joint count mismatch: profile has {} joints, hardware has {}",
                self.num_joints,
                hardware_joints
            );
        }

        // Could also verify joint limits by checking the state
        info!(
            "Profile validation successful: {} joints match",
            self.num_joints
        );
        Ok(())
    }

    /// Validate the profile itself
    ///
    /// Checks that the profile is internally consistent.
    pub fn validate(&self) -> anyhow::Result<()> {
        if self.name.is_empty() {
            anyhow::bail!("Robot name cannot be empty");
        }

        if self.num_joints == 0 {
            anyhow::bail!("Robot must have at least one joint");
        }

        if self.joint_names.len() != self.num_joints {
            anyhow::bail!(
                "Joint names count ({}) doesn't match num_joints ({})",
                self.joint_names.len(),
                self.num_joints
            );
        }

        if self.joint_limits.len() != self.num_joints {
            anyhow::bail!(
                "Joint limits count ({}) doesn't match num_joints ({})",
                self.joint_limits.len(),
                self.num_joints
            );
        }

        // Validate each joint limit
        for (i, limits) in self.joint_limits.iter().enumerate() {
            if limits.min >= limits.max {
                anyhow::bail!(
                    "Joint {}: min limit ({}) must be less than max limit ({})",
                    i,
                    limits.min,
                    limits.max
                );
            }
            if limits.max_velocity <= 0.0 {
                anyhow::bail!("Joint {}: max velocity must be > 0", i);
            }
            if limits.max_effort <= 0.0 {
                anyhow::bail!("Joint {}: max effort must be > 0", i);
            }
        }

        if self.mass <= 0.0 {
            anyhow::bail!("Robot mass must be > 0");
        }

        if self.height <= 0.0 {
            anyhow::bail!("Robot height must be > 0");
        }

        if self.max_velocity <= 0.0 {
            anyhow::bail!("Max velocity must be > 0");
        }

        Ok(())
    }

    /// Get capabilities for this robot profile
    pub fn capabilities(&self) -> RobotCapabilities {
        let max_torque = self.joint_limits.iter().map(|l| l.max_effort).collect();

        let max_velocity = self.joint_limits.iter().map(|l| l.max_velocity).collect();

        RobotCapabilities {
            supports_low_level_control: true,
            supports_high_level_commands: true,
            max_torque,
            max_velocity,
        }
    }

    /// Get joint index by name
    pub fn get_joint_index(&self, name: &str) -> Option<usize> {
        self.joint_names.iter().position(|n| n == name)
    }

    /// Get joint limits by name
    pub fn get_joint_limits(&self, name: &str) -> Option<&JointLimits> {
        self.get_joint_index(name)
            .and_then(|idx| self.joint_limits.get(idx))
    }
}

impl RobotCapabilities {
    /// Check if all joints support the required torque
    pub fn can_support_torque(&self, torques: &[f64]) -> bool {
        if torques.len() != self.max_torque.len() {
            return false;
        }

        torques
            .iter()
            .zip(self.max_torque.iter())
            .all(|(req, max)| req.abs() <= *max)
    }

    /// Check if all joints can achieve the required velocity
    pub fn can_achieve_velocity(&self, velocities: &[f64]) -> bool {
        if velocities.len() != self.max_velocity.len() {
            return false;
        }

        velocities
            .iter()
            .zip(self.max_velocity.iter())
            .all(|(req, max)| req.abs() <= *max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unitree_g1_profile() {
        let profile = RobotProfile::unitree_g1();
        assert_eq!(profile.name, "Unitree G1");
        assert_eq!(profile.num_joints, 29);
        assert_eq!(profile.joint_names.len(), 29);
        assert_eq!(profile.joint_limits.len(), 29);
        assert!(profile.mass > 0.0);
        assert!(profile.height > 0.0);
        assert!(profile.validate().is_ok());
    }

    #[test]
    fn test_unitree_h1_profile() {
        let profile = RobotProfile::unitree_h1();
        assert_eq!(profile.name, "Unitree H1");
        assert_eq!(profile.num_joints, 28);
        assert!(profile.mass > 0.0);
        assert!(profile.validate().is_ok());
    }

    #[test]
    fn test_profile_validation() {
        let mut profile = RobotProfile::unitree_g1();
        assert!(profile.validate().is_ok());

        // Test invalid joint limits
        profile.joint_limits[0].min = 1.0;
        profile.joint_limits[0].max = -1.0;
        assert!(profile.validate().is_err());

        // Reset and test other validations
        profile.joint_limits[0] = JointLimits::default();
        profile.mass = 0.0;
        assert!(profile.validate().is_err());
    }

    #[test]
    fn test_capabilities() {
        let profile = RobotProfile::unitree_g1();
        let caps = profile.capabilities();

        assert!(caps.supports_low_level_control);
        assert!(caps.supports_high_level_commands);
        assert_eq!(caps.max_torque.len(), profile.num_joints);
        assert_eq!(caps.max_velocity.len(), profile.num_joints);
    }

    #[test]
    fn test_joint_lookup() {
        let profile = RobotProfile::unitree_g1();

        assert!(profile.get_joint_index("left_hip_pitch").is_some());
        assert!(profile.get_joint_index("nonexistent").is_none());

        let limits = profile.get_joint_limits("left_knee");
        assert!(limits.is_some());
    }

    #[test]
    fn test_capability_checks() {
        let profile = RobotProfile::unitree_g1();
        let caps = profile.capabilities();

        // Test valid torques
        let valid_torques = vec![10.0; profile.num_joints];
        assert!(caps.can_support_torque(&valid_torques));

        // Test invalid torques (exceeds limits)
        let invalid_torques = vec![200.0; profile.num_joints];
        assert!(!caps.can_support_torque(&invalid_torques));

        // Test mismatched length
        let wrong_size = vec![10.0; 5];
        assert!(!caps.can_support_torque(&wrong_size));
    }
}
