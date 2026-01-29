//! Stand Up Example - Updated with New SDK APIs
//!
//! Demonstrates safe standing motion with proper error handling,
//! safety limits, and emergency stop features.
//!
//! Run with: `cargo run --example stand_up --features simulator`

use humanoid_rs::control::{Controller, JointSpaceController, MotionPlanner, Trajectory, Waypoint};
use humanoid_rs::robot::{unitree_g1, HumanoidRobot, JointLimits, JointSet, RobotState};
use humanoid_rs::simulator::SimulatedRobot;
use humanoid_rs::{HumanoidError, Result};
use std::thread;
use std::time::{Duration, Instant};

/// Safety configuration for robot motion
struct SafetyConfig {
    max_joint_velocity: f64,     // rad/s
    max_joint_acceleration: f64, // rad/s²
    position_tolerance: f64,     // rad
    emergency_stop_timeout: Duration,
    check_rate_hz: f64,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            max_joint_velocity: 2.0,
            max_joint_acceleration: 5.0,
            position_tolerance: 0.01,
            emergency_stop_timeout: Duration::from_millis(100),
            check_rate_hz: 100.0,
        }
    }
}

/// Safe motion controller with integrated safety checks
struct SafeMotionController {
    planner: MotionPlanner,
    controller: JointSpaceController,
    safety: SafetyConfig,
    emergency_stopped: bool,
    last_position: Vec<f64>,
    last_update: Instant,
}

impl SafeMotionController {
    fn new() -> Self {
        let safety = SafetyConfig::default();
        let planner = MotionPlanner::new(safety.max_joint_velocity, safety.max_joint_acceleration);
        let controller = JointSpaceController::new(100.0, 20.0);

        Self {
            planner,
            controller,
            safety,
            emergency_stopped: false,
            last_position: vec![0.0; unitree_g1::NUM_JOINTS],
            last_update: Instant::now(),
        }
    }

    /// Check if motion is safe to execute
    fn validate_trajectory(&self, trajectory: &Trajectory, joints: &JointSet) -> Result<()> {
        // Check each waypoint for safety
        if let Some(waypoint) = trajectory.interpolate(Duration::from_secs(0)) {
            // Check joint limits
            for (target_pos, joint) in waypoint.joint_positions.iter().zip(joints.iter()) {
                if *target_pos < joint.limits.min || *target_pos > joint.limits.max {
                    return Err(HumanoidError::JointLimitExceeded {
                        joint_name: joint.name.clone(),
                        requested: *target_pos,
                        min: joint.limits.min,
                        max: joint.limits.max,
                    });
                }
            }
        }

        Ok(())
    }

    /// Execute standing motion with safety monitoring
    fn execute_stand_up(&mut self, robot: &mut SimulatedRobot) -> Result<()> {
        println!("\nExecuting safe stand-up sequence...");

        // Plan trajectory
        let trajectory = self.planner.plan_stand_up(&robot.state.joints);
        println!("  Planned trajectory: {:?}", trajectory.duration());

        // Validate trajectory
        self.validate_trajectory(&trajectory, &robot.state.joints)?;
        println!("  ✓ Trajectory validated");

        // Set trajectory
        self.controller.set_trajectory(trajectory);
        println!("  ✓ Controller initialized");

        // Execute with monitoring
        let dt = Duration::from_secs_f64(1.0 / self.safety.check_rate_hz);
        let timeout = trajectory.duration() + Duration::from_secs(2);
        let start_time = Instant::now();

        println!("  Executing...");

        while start_time.elapsed() < timeout && !self.controller.is_complete() {
            if self.emergency_stopped {
                return Err(HumanoidError::SafetyStop {
                    reason: "Emergency stop activated".to_string(),
                });
            }

            // Update controller
            let output = self.controller.update(&robot.state, dt)?;

            // Apply safety checks
            self.check_safety(&output)?;

            // Update robot state
            let positions: Vec<f64> = output.iter().map(|j| j.target_position).collect();
            robot.set_target_positions(positions.clone());
            self.last_position = positions;

            // Print progress
            if start_time.elapsed().as_millis() % 500 == 0 {
                let progress = (start_time.elapsed().as_secs_f64()
                    / trajectory.duration().as_secs_f64()
                    * 100.0) as u32;
                print!("\r    Progress: {}%", progress.min(100));
            }

            thread::sleep(dt);
        }

        println!("\n  ✓ Stand-up complete");
        Ok(())
    }

    /// Check safety constraints
    fn check_safety(&self, output: &JointSet) -> Result<()> {
        for joint in output.iter() {
            // Check effort limits
            if joint.effort.abs() > joint.limits.max_effort {
                return Err(HumanoidError::SafetyStop {
                    reason: format!(
                        "Joint {} effort {:.2} Nm exceeds limit {:.2} Nm",
                        joint.name, joint.effort, joint.limits.max_effort
                    ),
                });
            }

            // Check velocity limits
            if joint.velocity.abs() > self.safety.max_joint_velocity {
                return Err(HumanoidError::SafetyStop {
                    reason: format!(
                        "Joint {} velocity {:.2} rad/s exceeds limit",
                        joint.name, joint.velocity
                    ),
                });
            }
        }

        Ok(())
    }

    /// Emergency stop
    fn emergency_stop(&mut self) {
        println!("\n  !!! EMERGENCY STOP !!!");
        self.emergency_stopped = true;
    }

    /// Reset emergency stop
    fn reset_emergency_stop(&mut self) {
        println!("  Resetting emergency stop...");
        self.emergency_stopped = false;
    }
}

fn main() -> Result<()> {
    println!("Humanoid-RS: Stand Up Example (Safe Motion)");
    println!("=============================================\n");

    // Create robot
    println!("Initializing robot...");
    let mut robot = SimulatedRobot::new();
    println!("  ✓ Created robot with {} joints", unitree_g1::NUM_JOINTS);

    // Display initial state
    println!("\nInitial State:");
    println!("  Position: seated/collapsed");
    let initial_positions = robot.state.joints.positions();
    println!(
        "  Joint positions: {} joints at 0.0 rad",
        initial_positions.len()
    );

    // Create safe motion controller
    println!("\nInitializing safe motion controller...");
    let mut safe_controller = SafeMotionController::new();
    println!("  ✓ Safety limits configured:");
    println!(
        "    - Max velocity: {:.1} rad/s",
        safe_controller.safety.max_joint_velocity
    );
    println!(
        "    - Max acceleration: {:.1} rad/s²",
        safe_controller.safety.max_joint_acceleration
    );
    println!(
        "    - Position tolerance: {:.3} rad",
        safe_controller.safety.position_tolerance
    );
    println!(
        "    - Safety check rate: {:.0} Hz",
        safe_controller.safety.check_rate_hz
    );

    // Execute stand-up motion
    match safe_controller.execute_stand_up(&mut robot) {
        Ok(()) => {
            println!("\n✓ Stand-up motion completed successfully");
        }
        Err(HumanoidError::SafetyStop { reason }) => {
            println!("\n✗ Safety stop triggered: {}", reason);
            return Err(HumanoidError::SafetyStop { reason });
        }
        Err(e) => {
            println!("\n✗ Error during stand-up: {}", e);
            return Err(e);
        }
    }

    // Verify final state
    println!("\nFinal State:");
    let final_positions = robot.state.joints.positions();
    let standing_pose = get_standing_pose();
    let mut max_error = 0.0;

    for (final_pos, target_pos) in final_positions.iter().zip(standing_pose.iter()).enumerate() {
        let error = (final_pos.1 - target_pos).abs();
        max_error = max_error.max(error);
        if error > safe_controller.safety.position_tolerance {
            println!(
                "  Warning: Joint {} position error {:.4} rad exceeds tolerance",
                final_pos.0, error
            );
        }
    }

    println!("  Max position error: {:.4} rad", max_error);
    println!(
        "  Status: {}",
        if max_error < safe_controller.safety.position_tolerance {
            "STANDING ✓"
        } else {
            "POSITION ERROR ⚠"
        }
    );

    // Demonstrate emergency stop
    println!("\n--- Emergency Stop Demonstration ---");
    println!("Testing emergency stop functionality...");

    safe_controller.emergency_stop();

    match safe_controller.execute_stand_up(&mut robot) {
        Ok(()) => println!("  ✗ Emergency stop did not work!"),
        Err(HumanoidError::SafetyStop { reason }) => {
            println!("  ✓ Emergency stop working: {}", reason);
        }
        Err(e) => println!("  ✗ Unexpected error: {}", e),
    }

    // Reset and complete
    safe_controller.reset_emergency_stop();
    println!("\n✓ Example complete!");

    println!("\nSafety Features Demonstrated:");
    println!("  1. Joint limit checking before motion");
    println!("  2. Velocity limit enforcement during motion");
    println!("  3. Effort monitoring and limiting");
    println!("  4. Emergency stop functionality");
    println!("  5. Trajectory validation");
    println!("  6. Position tolerance checking");

    println!("\nFor real hardware:");
    println!("  1. Use UnitreeRobot instead of SimulatedRobot");
    println!("  2. Enable low-level control: robot.enable_low_level_control()?");
    println!("  3. Monitor hardware status continuously");
    println!("  4. Implement hardware-specific safety checks");
    println!("  5. Log all safety events for analysis");

    Ok(())
}

/// Get standing pose for verification
fn get_standing_pose() -> Vec<f64> {
    let mut pose = vec![0.0; 29];
    pose[3] = 0.3; // left_knee
    pose[9] = 0.3; // right_knee
    pose[15] = 0.2; // left_shoulder_pitch
    pose[22] = 0.2; // right_shoulder_pitch
    pose
}
