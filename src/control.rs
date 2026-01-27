use crate::robot::{JointSet, Pose, RobotState};
use std::time::Duration;
use nalgebra::{Vector3, UnitQuaternion};

/// Trajectory waypoint
#[derive(Debug, Clone)]
pub struct Waypoint {
    pub time: Duration,
    pub joint_positions: Vec<f64>,
    pub base_pose: Option<Pose>,
}

/// Trajectory for robot motion
#[derive(Debug, Clone, Default)]
pub struct Trajectory {
    waypoints: Vec<Waypoint>,
}

impl Trajectory {
    pub fn new() -> Self {
        Self { waypoints: Vec::new() }
    }
    
    pub fn add_waypoint(&mut self, waypoint: Waypoint) {
        self.waypoints.push(waypoint);
    }
    
    /// Interpolate trajectory at given time
    pub fn interpolate(&self, time: Duration) -> Option<Waypoint> {
        if self.waypoints.is_empty() {
            return None;
        }
        
        // Find surrounding waypoints
        let idx = self.waypoints.iter()
            .position(|w| w.time > time)
            .unwrap_or(self.waypoints.len());
            
        if idx == 0 {
            return Some(self.waypoints[0].clone());
        }
        
        if idx >= self.waypoints.len() {
            return Some(self.waypoints.last().unwrap().clone());
        }
        
        let w1 = &self.waypoints[idx - 1];
        let w2 = &self.waypoints[idx];
        
        // Linear interpolation
        let t1 = w1.time.as_secs_f64();
        let t2 = w2.time.as_secs_f64();
        let t = time.as_secs_f64();
        let alpha = (t - t1) / (t2 - t1);
        
        let joint_positions: Vec<f64> = w1.joint_positions.iter()
            .zip(w2.joint_positions.iter())
            .map(|(p1, p2)| p1 + alpha * (p2 - p1))
            .collect();
            
        Some(Waypoint {
            time,
            joint_positions,
            base_pose: None,  // Could interpolate pose too
        })
    }
    
    pub fn duration(&self) -> Duration {
        self.waypoints.last()
            .map(|w| w.time)
            .unwrap_or(Duration::from_secs(0))
    }
}

/// Motion planner for generating trajectories
pub struct MotionPlanner {
    max_velocity: f64,
    max_acceleration: f64,
}

impl Default for MotionPlanner {
    fn default() -> Self {
        Self {
            max_velocity: 2.0,      // rad/s
            max_acceleration: 5.0,   // rad/s²
        }
    }
}

impl MotionPlanner {
    pub fn new(max_velocity: f64, max_acceleration: f64) -> Self {
        Self {
            max_velocity,
            max_acceleration,
        }
    }
    
    /// Plan joint space trajectory from current to target
    pub fn plan_joint_space(
        &self,
        current: &JointSet,
        target: &JointSet,
        duration: Duration,
    ) -> Trajectory {
        let mut trajectory = Trajectory::new();
        
        // Start waypoint
        trajectory.add_waypoint(Waypoint {
            time: Duration::from_secs(0),
            joint_positions: current.positions(),
            base_pose: None,
        });
        
        // End waypoint
        trajectory.add_waypoint(Waypoint {
            time: duration,
            joint_positions: target.positions(),
            base_pose: None,
        });
        
        trajectory
    }
    
    /// Plan Cartesian trajectory for end effector
    pub fn plan_cartesian(
        &self,
        start: &Pose,
        end: &Pose,
        duration: Duration,
    ) -> Trajectory {
        let mut trajectory = Trajectory::new();
        
        // Simple linear interpolation in Cartesian space
        let steps = 20;
        let dt = duration.as_secs_f64() / steps as f64;
        
        for i in 0..=steps {
            let alpha = i as f64 / steps as f64;
            let t = Duration::from_secs_f64(i as f64 * dt);
            
            // Linear interpolation for position
            let position = start.position * (1.0 - alpha) + end.position * alpha;
            
            // Spherical linear interpolation for orientation
            let orientation = start.orientation.slerp(&end.orientation, alpha);
            
            trajectory.add_waypoint(Waypoint {
                time: t,
                joint_positions: vec![],  // Would need IK to fill this
                base_pose: Some(Pose { position, orientation }),
            });
        }
        
        trajectory
    }
    
    /// Plan standing up motion
    pub fn plan_stand_up(&self, current: &JointSet) -> Trajectory {
        let mut trajectory = Trajectory::new();
        let stand_pose = Self::standing_pose();
        
        trajectory.add_waypoint(Waypoint {
            time: Duration::from_secs(0),
            joint_positions: current.positions(),
            base_pose: None,
        });
        
        trajectory.add_waypoint(Waypoint {
            time: Duration::from_secs(2),
            joint_positions: stand_pose,
            base_pose: None,
        });
        
        trajectory
    }
    
    fn standing_pose() -> Vec<f64> {
        // Rough standing pose for G1
        let mut pose = vec![0.0; 29];
        // Slight knee bend
        pose[3] = 0.3;   // left_knee
        pose[9] = 0.3;   // right_knee
        // Arms down
        pose[15] = 0.2;  // left_shoulder_pitch
        pose[22] = 0.2;  // right_shoulder_pitch
        pose
    }
}

/// Low-level controller interface
pub trait Controller {
    /// Update control loop
    fn update(&mut self, state: &RobotState, dt: Duration) -> crate::Result<JointSet>;
    
    /// Set target trajectory
    fn set_trajectory(&mut self, trajectory: Trajectory);
    
    /// Check if trajectory is complete
    fn is_complete(&self) -> bool;
}

/// Joint-space PD controller
pub struct JointSpaceController {
    kp: f64,  // Proportional gain
    kd: f64,  // Derivative gain
    trajectory: Option<Trajectory>,
    current_time: Duration,
    target: JointSet,
}

impl JointSpaceController {
    pub fn new(kp: f64, kd: f64) -> Self {
        Self {
            kp,
            kd,
            trajectory: None,
            current_time: Duration::from_secs(0),
            target: JointSet::new(),
        }
    }
}

impl Controller for JointSpaceController {
    fn update(&mut self, state: &RobotState, dt: Duration) -> crate::Result<JointSet> {
        self.current_time += dt;
        
        // Get target from trajectory or hold current
        let target_positions = if let Some(ref traj) = self.trajectory {
            traj.interpolate(self.current_time)
                .map(|w| w.joint_positions)
                .unwrap_or_else(|| self.target.positions())
        } else {
            self.target.positions()
        };
        
        // PD control
        let mut output = JointSet::new();
        for (joint, &target_pos) in state.joints.iter().zip(target_positions.iter()) {
            let error = target_pos - joint.current_position;
            let d_error = -joint.velocity;  // Simplified
            
            let effort = self.kp * error + self.kd * d_error;
            
            let mut out_joint = joint.clone();
            out_joint.effort = effort;
            output.add_joint(out_joint);
        }
        
        Ok(output)
    }
    
    fn set_trajectory(&mut self, trajectory: Trajectory) {
        self.trajectory = Some(trajectory);
        self.current_time = Duration::from_secs(0);
    }
    
    fn is_complete(&self) -> bool {
        self.trajectory.as_ref()
            .map(|t| self.current_time >= t.duration())
            .unwrap_or(true)
    }
}
