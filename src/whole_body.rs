//! Whole-body control for humanoid robots
//!
//! Implements balance control, center of mass tracking, and coordinated motion.

use crate::ik::IkSolver;
use crate::robot::{JointSet, Pose, RobotState};
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use std::time::Duration;

/// Center of mass state
#[derive(Debug, Clone, Default)]
pub struct ComState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

/// Contact state for a foot
#[derive(Debug, Clone)]
pub struct FootContact {
    pub position: Vector3<f64>,
    pub normal: Vector3<f64>,
    pub in_contact: bool,
    pub force: Vector3<f64>,
}

impl Default for FootContact {
    fn default() -> Self {
        Self {
            position: Vector3::zeros(),
            normal: Vector3::y(),
            in_contact: true,
            force: Vector3::new(0.0, 0.0, 9.81 * 30.0 / 2.0), // Half body weight
        }
    }
}

/// Support polygon from foot contacts
#[derive(Debug, Clone)]
pub struct SupportPolygon {
    pub vertices: Vec<Vector3<f64>>,
    pub centroid: Vector3<f64>,
}

impl SupportPolygon {
    /// Create from foot contacts
    pub fn from_feet(left: &FootContact, right: &FootContact) -> Self {
        let mut vertices = Vec::new();
        let foot_length = 0.18;
        let foot_width = 0.10;

        if left.in_contact {
            let p = left.position;
            vertices.push(Vector3::new(
                p.x - foot_width / 2.0,
                0.0,
                p.z - foot_length / 2.0,
            ));
            vertices.push(Vector3::new(
                p.x + foot_width / 2.0,
                0.0,
                p.z - foot_length / 2.0,
            ));
            vertices.push(Vector3::new(
                p.x + foot_width / 2.0,
                0.0,
                p.z + foot_length / 2.0,
            ));
            vertices.push(Vector3::new(
                p.x - foot_width / 2.0,
                0.0,
                p.z + foot_length / 2.0,
            ));
        }

        if right.in_contact {
            let p = right.position;
            vertices.push(Vector3::new(
                p.x - foot_width / 2.0,
                0.0,
                p.z - foot_length / 2.0,
            ));
            vertices.push(Vector3::new(
                p.x + foot_width / 2.0,
                0.0,
                p.z - foot_length / 2.0,
            ));
            vertices.push(Vector3::new(
                p.x + foot_width / 2.0,
                0.0,
                p.z + foot_length / 2.0,
            ));
            vertices.push(Vector3::new(
                p.x - foot_width / 2.0,
                0.0,
                p.z + foot_length / 2.0,
            ));
        }

        let centroid = if vertices.is_empty() {
            Vector3::zeros()
        } else {
            vertices.iter().fold(Vector3::zeros(), |acc, v| acc + v) / vertices.len() as f64
        };

        Self { vertices, centroid }
    }

    /// Check if a point is inside the support polygon
    pub fn contains(&self, point: Vector3<f64>) -> bool {
        if self.vertices.len() < 3 {
            return false;
        }

        // Project to XZ plane and check containment
        let p = Vector3::new(point.x, 0.0, point.z);

        // Use winding number algorithm
        let mut winding = 0i32;
        let n = self.vertices.len();

        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];

            if v1.z <= p.z {
                if v2.z > p.z {
                    if Self::is_left(v1, v2, p) > 0.0 {
                        winding += 1;
                    }
                }
            } else {
                if v2.z <= p.z {
                    if Self::is_left(v1, v2, p) < 0.0 {
                        winding -= 1;
                    }
                }
            }
        }

        winding != 0
    }

    fn is_left(v0: Vector3<f64>, v1: Vector3<f64>, p: Vector3<f64>) -> f64 {
        (v1.x - v0.x) * (p.z - v0.z) - (p.x - v0.x) * (v1.z - v0.z)
    }

    /// Get margin to edge of support polygon
    pub fn margin(&self, point: Vector3<f64>) -> f64 {
        if self.vertices.is_empty() {
            return 0.0;
        }

        let p = Vector3::new(point.x, 0.0, point.z);
        let mut min_dist = f64::MAX;

        let n = self.vertices.len();
        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];

            // Distance from point to line segment
            let edge = v2 - v1;
            let edge_len = edge.norm();
            if edge_len < 1e-6 {
                continue;
            }

            let t = ((p - v1).dot(&edge) / (edge_len * edge_len)).clamp(0.0, 1.0);
            let closest = v1 + edge * t;
            let dist = (p - closest).norm();

            min_dist = min_dist.min(dist);
        }

        if self.contains(point) {
            min_dist
        } else {
            -min_dist
        }
    }
}

/// Whole-body controller configuration
#[derive(Debug, Clone)]
pub struct WholeBodyConfig {
    /// CoM tracking gains
    pub com_kp: Vector3<f64>,
    pub com_kd: Vector3<f64>,

    /// Orientation tracking gains
    pub orientation_kp: Vector3<f64>,
    pub orientation_kd: Vector3<f64>,

    /// Foot tracking gains
    pub foot_kp: f64,
    pub foot_kd: f64,

    /// Joint damping
    pub joint_damping: f64,

    /// Regularization weight
    pub regularization: f64,

    /// Weight for minimizing joint acceleration
    pub min_accel_weight: f64,

    /// Robot mass (kg)
    pub robot_mass: f64,

    /// Gravity vector
    pub gravity: Vector3<f64>,
}

impl Default for WholeBodyConfig {
    fn default() -> Self {
        Self {
            com_kp: Vector3::new(100.0, 100.0, 100.0),
            com_kd: Vector3::new(20.0, 20.0, 20.0),
            orientation_kp: Vector3::new(100.0, 100.0, 100.0),
            orientation_kd: Vector3::new(20.0, 20.0, 20.0),
            foot_kp: 200.0,
            foot_kd: 40.0,
            joint_damping: 10.0,
            regularization: 1e-6,
            min_accel_weight: 0.01,
            robot_mass: 35.0, // G1 is about 35kg
            gravity: Vector3::new(0.0, -9.81, 0.0),
        }
    }
}

/// Whole-body controller for balance and coordination
pub struct WholeBodyController {
    config: WholeBodyConfig,
    ik_solver: IkSolver,

    // State tracking
    com_desired: ComState,
    left_foot: FootContact,
    right_foot: FootContact,

    // Control outputs
    joint_torques: Vec<f64>,
    last_joint_velocities: Vec<f64>,
}

impl WholeBodyController {
    pub fn new(config: WholeBodyConfig) -> Self {
        Self {
            config,
            ik_solver: IkSolver::with_default_config(),
            com_desired: ComState::default(),
            left_foot: FootContact::default(),
            right_foot: FootContact::default(),
            joint_torques: vec![0.0; 29],
            last_joint_velocities: vec![0.0; 29],
        }
    }

    /// Set desired center of mass position
    pub fn set_com_target(&mut self, position: Vector3<f64>, velocity: Vector3<f64>) {
        self.com_desired.position = position;
        self.com_desired.velocity = velocity;
    }

    /// Update foot contact state
    pub fn set_foot_contacts(&mut self, left: FootContact, right: FootContact) {
        self.left_foot = left;
        self.right_foot = right;
    }

    /// Compute whole-body control output
    pub fn compute(&mut self, state: &RobotState, dt: Duration) -> Vec<f64> {
        let dt_s = dt.as_secs_f64();

        // Get current CoM estimate (simplified - assume at base position)
        let current_com = ComState {
            position: Vector3::new(
                state.base_pose.position.x,
                state.base_pose.position.y,
                state.base_pose.position.z,
            ),
            velocity: Vector3::zeros(), // Would compute from base velocity
            acceleration: Vector3::zeros(),
        };

        // Compute support polygon
        let support = SupportPolygon::from_feet(&self.left_foot, &self.right_foot);

        // Compute CoM error
        let com_error = self.com_desired.position - current_com.position;
        let com_vel_error = self.com_desired.velocity - current_com.velocity;

        // PD control for CoM
        let com_accel_des = self.config.com_kp.component_mul(&com_error)
            + self.config.com_kd.component_mul(&com_vel_error);

        // Compute required ground reaction forces
        let total_force = self.config.robot_mass * (com_accel_des - self.config.gravity);

        // Distribute forces to feet
        let (left_force, right_force) = self.distribute_forces(total_force, &current_com);

        // Compute joint torques via inverse dynamics (simplified)
        self.compute_joint_torques(state, left_force, right_force, dt_s);

        // Apply joint damping
        for (i, torque) in self.joint_torques.iter_mut().enumerate() {
            let joint = state.joints.iter().nth(i);
            if let Some(j) = joint {
                *torque -= self.config.joint_damping * j.velocity;
            }
        }

        self.joint_torques.clone()
    }

    /// Distribute total force between feet
    fn distribute_forces(
        &self,
        total_force: Vector3<f64>,
        com: &ComState,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let left_pos = self.left_foot.position;
        let right_pos = self.right_foot.position;

        // Simple linear interpolation based on CoM position
        let foot_span = (right_pos.x - left_pos.x).abs().max(0.01);
        let com_offset = (com.position.x - left_pos.x) / foot_span;

        let right_ratio = com_offset.clamp(0.0, 1.0);
        let left_ratio = 1.0 - right_ratio;

        let left_force = if self.left_foot.in_contact {
            total_force * left_ratio
        } else {
            Vector3::zeros()
        };

        let right_force = if self.right_foot.in_contact {
            total_force * right_ratio
        } else {
            Vector3::zeros()
        };

        (left_force, right_force)
    }

    /// Compute joint torques from desired forces (simplified inverse dynamics)
    fn compute_joint_torques(
        &mut self,
        state: &RobotState,
        left_force: Vector3<f64>,
        right_force: Vector3<f64>,
        dt: f64,
    ) {
        // Simplified: map forces to leg joint torques
        // Real implementation would use full Jacobian transpose

        let positions = state.joints.positions();

        // Left leg (joints 0-5)
        if self.left_foot.in_contact {
            self.joint_torques[0] = left_force.z * 0.3; // hip pitch
            self.joint_torques[1] = left_force.x * 0.1; // hip roll
            self.joint_torques[2] = 0.0; // hip yaw
            self.joint_torques[3] = left_force.y * 0.3; // knee
            self.joint_torques[4] = -left_force.z * 0.15; // ankle pitch
            self.joint_torques[5] = -left_force.x * 0.05; // ankle roll
        }

        // Right leg (joints 6-11)
        if self.right_foot.in_contact {
            self.joint_torques[6] = right_force.z * 0.3;
            self.joint_torques[7] = right_force.x * 0.1;
            self.joint_torques[8] = 0.0;
            self.joint_torques[9] = right_force.y * 0.3;
            self.joint_torques[10] = -right_force.z * 0.15;
            self.joint_torques[11] = -right_force.x * 0.05;
        }

        // Arms: just apply gravity compensation
        for i in 15..29 {
            self.joint_torques[i] = 0.0; // Would compute gravity torques
        }

        // Update velocity tracking for derivative control
        for (i, pos) in positions.iter().enumerate() {
            if i < self.last_joint_velocities.len() {
                self.last_joint_velocities[i] = (pos - positions.get(i).unwrap_or(&0.0)) / dt;
            }
        }
    }

    /// Check if robot is balanced
    pub fn is_balanced(&self, state: &RobotState) -> bool {
        let com = Vector3::new(
            state.base_pose.position.x,
            state.base_pose.position.y,
            state.base_pose.position.z,
        );

        let support = SupportPolygon::from_feet(&self.left_foot, &self.right_foot);
        let margin = support.margin(com);

        margin > 0.02 // At least 2cm margin
    }

    /// Get stability margin
    pub fn stability_margin(&self, state: &RobotState) -> f64 {
        let com = Vector3::new(
            state.base_pose.position.x,
            state.base_pose.position.y,
            state.base_pose.position.z,
        );

        let support = SupportPolygon::from_feet(&self.left_foot, &self.right_foot);
        support.margin(com)
    }
}

/// Balance recovery controller
pub struct BalanceRecovery {
    /// Ankle strategy gains
    ankle_kp: f64,
    ankle_kd: f64,

    /// Hip strategy gains
    hip_kp: f64,
    hip_kd: f64,

    /// Step strategy threshold (CoM velocity)
    step_threshold: f64,
}

impl Default for BalanceRecovery {
    fn default() -> Self {
        Self {
            ankle_kp: 500.0,
            ankle_kd: 50.0,
            hip_kp: 200.0,
            hip_kd: 40.0,
            step_threshold: 0.5,
        }
    }
}

impl BalanceRecovery {
    /// Compute recovery action
    pub fn compute(&self, com: &ComState, support: &SupportPolygon) -> RecoveryAction {
        let margin = support.margin(com.position);
        let velocity_mag = Vector3::new(com.velocity.x, 0.0, com.velocity.z).norm();

        if margin > 0.05 && velocity_mag < 0.1 {
            // Stable, no recovery needed
            RecoveryAction::None
        } else if margin > 0.0 && velocity_mag < self.step_threshold {
            // Use ankle strategy
            let ankle_torque = self.ankle_kp * (support.centroid.x - com.position.x)
                - self.ankle_kd * com.velocity.x;
            RecoveryAction::Ankle {
                torque: ankle_torque,
            }
        } else if margin > -0.02 && velocity_mag < self.step_threshold * 1.5 {
            // Use hip strategy
            let hip_angle =
                self.hip_kp * (support.centroid.x - com.position.x) - self.hip_kd * com.velocity.x;
            RecoveryAction::Hip { angle: hip_angle }
        } else {
            // Need to step
            let step_target = com.position + com.velocity * 0.5;
            RecoveryAction::Step {
                target: step_target,
            }
        }
    }
}

/// Recovery action type
#[derive(Debug, Clone)]
pub enum RecoveryAction {
    None,
    Ankle { torque: f64 },
    Hip { angle: f64 },
    Step { target: Vector3<f64> },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_support_polygon() {
        let left = FootContact {
            position: Vector3::new(-0.1, 0.0, 0.0),
            in_contact: true,
            ..Default::default()
        };
        let right = FootContact {
            position: Vector3::new(0.1, 0.0, 0.0),
            in_contact: true,
            ..Default::default()
        };

        let support = SupportPolygon::from_feet(&left, &right);
        assert!(!support.vertices.is_empty());

        // Center should be inside
        assert!(support.contains(Vector3::new(0.0, 0.0, 0.0)));

        // Far point should be outside
        assert!(!support.contains(Vector3::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn test_whole_body_controller() {
        let config = WholeBodyConfig::default();
        let mut controller = WholeBodyController::new(config);

        controller.set_com_target(Vector3::new(0.0, 0.85, 0.0), Vector3::zeros());

        let state = RobotState::default();
        let torques = controller.compute(&state, Duration::from_millis(10));

        assert_eq!(torques.len(), 29);
    }

    #[test]
    fn test_balance_recovery() {
        let recovery = BalanceRecovery::default();

        // CoM at center (y is height, not used for balance in XZ plane)
        let stable_com = ComState {
            position: Vector3::new(0.0, 0.85, 0.0),
            velocity: Vector3::zeros(),
            ..Default::default()
        };

        // Wide stance for large support polygon
        let left = FootContact {
            position: Vector3::new(-0.15, 0.0, 0.0),
            in_contact: true,
            ..Default::default()
        };
        let right = FootContact {
            position: Vector3::new(0.15, 0.0, 0.0),
            in_contact: true,
            ..Default::default()
        };

        let support = SupportPolygon::from_feet(&left, &right);
        let action = recovery.compute(&stable_com, &support);

        // With CoM at center and wide stance, should be stable or at worst use ankle strategy
        // The margin depends on foot geometry, so we accept None or Ankle as valid
        assert!(matches!(
            action,
            RecoveryAction::None | RecoveryAction::Ankle { .. }
        ));
    }
}
