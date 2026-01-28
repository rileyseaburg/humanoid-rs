//! Inverse Kinematics solver for humanoid robots
//!
//! Allows reaching motions by computing joint angles from desired end-effector positions

use crate::robot::JointSet;
use nalgebra::Vector3;
use std::f64::consts::PI;

/// 3D position target
#[derive(Debug, Clone)]
pub struct PositionTarget {
    pub position: Vector3<f64>,
    pub tolerance: f64,
}

/// IK solver configuration
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

/// Inverse Kinematics solver using Jacobian pseudoinverse (CCD-style)
pub struct IkSolver {
    config: IkSolverConfig,
}

impl IkSolver {
    pub fn new(config: IkSolverConfig) -> Self {
        Self { config }
    }

    pub fn with_default_config() -> Self {
        Self::new(IkSolverConfig::default())
    }

    /// Solve IK for a simple 2-link arm (simplified for demo)
    pub fn solve_arm_2dof(&self, l1: f64, l2: f64, target: Vector3<f64>) -> Option<(f64, f64)> {
        let distance = target.norm();

        // Check if target is reachable
        if distance > l1 + l2 || distance < (l1 - l2).abs() {
            return None;
        }

        // Law of cosines for elbow angle
        let cos_elbow = (distance.powi(2) - l1.powi(2) - l2.powi(2)) / (2.0 * l1 * l2);
        let elbow_angle = if cos_elbow.abs() <= 1.0 {
            cos_elbow.acos()
        } else {
            return None;
        };

        // Shoulder angle using atan2 + offset
        let alpha = target.y.atan2(target.x);
        let beta = ((l1.powi(2) + distance.powi(2) - l2.powi(2)) / (2.0 * l1 * distance)).acos();
        let shoulder_angle = alpha + beta;

        Some((shoulder_angle, elbow_angle))
    }

    /// Solve IK for arm reaching (3D, simplified)
    pub fn solve_arm_reach(
        &self,
        target: Vector3<f64>,
        shoulder_pos: Vector3<f64>,
        arm_lengths: (f64, f64, f64),
    ) -> Vec<f64> {
        let (upper_arm, forearm, hand) = arm_lengths;
        let total_length = upper_arm + forearm + hand;

        let rel_target = target - shoulder_pos;
        let distance = rel_target.norm();

        if distance > total_length {
            // Target out of reach, extend fully toward target
            let direction = rel_target / distance;
            return vec![direction.y.atan2(direction.x), 0.0, 0.0];
        }

        // Simple analytic solution for demonstration
        let shoulder_pitch = rel_target.y.atan2(rel_target.z);
        let shoulder_yaw = rel_target.x.atan2(rel_target.z);
        let elbow = if distance < upper_arm + forearm {
            // Bend elbow to reach
            let cos_elbow = (distance.powi(2) - upper_arm.powi(2) - forearm.powi(2))
                / (2.0 * upper_arm * forearm);
            if cos_elbow.abs() <= 1.0 {
                cos_elbow.acos()
            } else {
                0.0
            }
        } else {
            0.0
        };

        vec![shoulder_pitch, shoulder_yaw, elbow]
    }

    /// Solve leg IK for stepping
    pub fn solve_leg_step(
        &self,
        foot_target: Vector3<f64>,
        hip_pos: Vector3<f64>,
        leg_lengths: (f64, f64, f64),
    ) -> Vec<f64> {
        let (thigh, shin, _foot) = leg_lengths;
        let rel_target = foot_target - hip_pos;
        let distance = rel_target.norm();

        if distance > thigh + shin {
            // Target out of reach, extend fully
            return vec![0.0, 0.0, 0.0];
        }

        // Hip pitch (forward/back)
        let hip_pitch = (rel_target.z.atan2(rel_target.y)).max(-1.0).min(1.0);

        // Hip yaw (left/right)
        let hip_yaw = (rel_target.x.atan2(rel_target.z)).max(-0.5).min(0.5);

        // Knee bend
        let desired_knee = if distance < thigh {
            // Need to bend knee to reach lower target
            let height = rel_target.y.abs();
            let knee_bend = ((thigh - height) / shin).acos().max(-1.4).min(0.0);
            knee_bend
        } else {
            0.0
        };

        vec![hip_pitch, hip_yaw, desired_knee]
    }

    /// CCD (Cyclic Coordinate Descent) IK solver for articulated chains
    pub fn solve_ccd(
        &self,
        joints: &mut JointSet,
        end_effector_pos: Vector3<f64>,
        joint_indices: &[usize],
        joint_axes: &[Vector3<f64>],
    ) -> bool {
        let mut converged = false;
        let tolerance = self.config.learning_rate;

        for _iteration in 0..self.config.max_iterations {
            let mut positions = joints.positions();

            // Iterate from end effector backward to base
            for (idx, &joint_idx) in joint_indices.iter().enumerate().rev() {
                if idx == 0 {
                    continue;
                }

                let current_end = self.forward_kinematics(&positions, joint_indices, joint_axes);
                let to_end = current_end - end_effector_pos;

                if to_end.norm() < tolerance {
                    converged = true;
                    break;
                }

                let default_axis = Vector3::x();
                let axis = joint_axes.get(idx).unwrap_or(&default_axis);

                // Rotate around axis to minimize distance
                let current_angle = positions.get(joint_idx).copied().unwrap_or(0.0);
                let delta = to_end.dot(axis) * self.config.learning_rate;
                positions[joint_idx] = (current_angle - delta).clamp(-PI, PI);
            }

            joints.set_positions(&positions);

            if converged {
                break;
            }
        }

        converged
    }

    /// Simple forward kinematics for IK chain
    fn forward_kinematics(
        &self,
        joint_angles: &[f64],
        joint_indices: &[usize],
        joint_axes: &[Vector3<f64>],
    ) -> Vector3<f64> {
        let mut position = Vector3::zeros();
        let mut rotation = nalgebra::Rotation3::identity();

        let segment_length = 0.3;

        for (i, &joint_idx) in joint_indices.iter().enumerate() {
            let angle = joint_angles.get(joint_idx).copied().unwrap_or(0.0);
            let default_axis = Vector3::x();
            let axis = joint_axes.get(i).unwrap_or(&default_axis);

            // Apply rotation
            let joint_rot =
                nalgebra::Rotation3::from_axis_angle(&nalgebra::Unit::new_normalize(*axis), angle);
            rotation = joint_rot * rotation;

            // Translate along local Z
            let translation = rotation * Vector3::new(0.0, 0.0, segment_length);
            position += translation;
        }

        position
    }
}

/// Whole-body IK solver for balancing and reaching
pub struct WholeBodyIk {
    solver: IkSolver,
    balance_weight: f64,
    reach_weight: f64,
}

impl WholeBodyIk {
    pub fn new(balance_weight: f64, reach_weight: f64) -> Self {
        Self {
            solver: IkSolver::with_default_config(),
            balance_weight,
            reach_weight,
        }
    }

    /// Solve for balanced standing with arm reaching
    pub fn solve_balanced_reach(
        &self,
        joints: &mut JointSet,
        hand_target: Option<Vector3<f64>>,
        _target_com: Vector3<f64>,
    ) -> bool {
        // This is a simplified approach - in practice you'd use QP or similar
        // For demo, we just solve arm IK while keeping legs stable

        if let Some(target) = hand_target {
            let arm_solution =
                self.solver
                    .solve_arm_reach(target, Vector3::new(0.2, 1.2, 0.0), (0.3, 0.25, 0.1));

            // Apply to right arm joints (indices 22-26)
            for (i, &angle) in arm_solution.iter().enumerate() {
                if let Some(joint_idx) = [22, 23, 24, 25].get(i) {
                    joints.set_positions(&{
                        let mut pos = joints.positions();
                        pos[*joint_idx] = angle;
                        pos
                    });
                }
            }
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arm_2dof_reachable() {
        let solver = IkSolver::with_default_config();
        let target = Vector3::new(1.0, 1.0, 0.0);
        let result = solver.solve_arm_2dof(1.0, 1.0, target);
        assert!(result.is_some());
    }

    #[test]
    fn test_arm_2dof_unreachable() {
        let solver = IkSolver::with_default_config();
        let target = Vector3::new(3.0, 3.0, 0.0);
        let result = solver.solve_arm_2dof(1.0, 1.0, target);
        assert!(result.is_none());
    }

    #[test]
    fn test_leg_step() {
        let solver = IkSolver::with_default_config();
        let foot_target = Vector3::new(0.2, -0.8, 0.3);
        let hip_pos = Vector3::new(0.0, 0.85, 0.0);
        let result = solver.solve_leg_step(foot_target, hip_pos, (0.3, 0.3, 0.1));
        assert_eq!(result.len(), 3);
    }
}
