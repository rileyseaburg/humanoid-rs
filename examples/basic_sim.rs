//! Basic Simulation Example
//!
//! Demonstrates minimal usage of the humanoid simulator.
//!
//! Run with: `cargo run --example basic_sim --features simulator`

use humanoid_rs::control::{Controller, JointSpaceController, MotionPlanner};
use humanoid_rs::robot::{unitree_g1, JointSet, RobotState};
use humanoid_rs::simulator::{run_simulator, SimulatedRobot};
use std::time::Duration;

fn main() {
    println!("Humanoid-RS: Basic Simulation Example");
    println!("========================================\n");

    // Create a simulated robot
    let mut robot = SimulatedRobot::new();
    println!(
        "Created simulated robot with {} joints",
        unitree_g1::NUM_JOINTS
    );

    // Create a motion planner for standing up
    let planner = MotionPlanner::new(2.0, 5.0);
    println!("Created motion planner with max velocity: 2.0 rad/s, max acceleration: 5.0 rad/s²");

    // Plan a standing motion
    let standing_trajectory = planner.plan_stand_up(&robot.state.joints);
    println!(
        "Planned standing trajectory with duration: {:?}",
        standing_trajectory.duration()
    );

    // Create a joint-space controller with PD gains
    let mut controller = JointSpaceController::new(50.0, 10.0);
    controller.set_trajectory(standing_trajectory);
    robot.set_controller(Box::new(controller));

    // Simulate a control loop for 5 seconds
    println!("\nSimulating control loop for 5 seconds...\n");

    let dt = Duration::from_millis(10); // 100 Hz control rate
    let total_duration = Duration::from_secs(5);
    let steps = total_duration.as_millis() / dt.as_millis();

    for step in 0..steps {
        // Update controller
        if let Some(ref mut ctrl) = robot.controller {
            match ctrl.update(&robot.state, dt) {
                Ok(output) => {
                    // Apply control output to robot
                    let positions: Vec<f64> = output.iter().map(|j| j.current_position).collect();
                    robot.set_target_positions(positions);
                }
                Err(e) => {
                    eprintln!("Control error at step {}: {}", step, e);
                    break;
                }
            }
        }

        // Print progress every second
        if step % 100 == 0 {
            let elapsed = step as f64 * dt.as_secs_f64();
            println!("  Time: {:.1}s - Controller active", elapsed);
        }
    }

    println!("\nSimulation complete!");
    println!("\nIn a real scenario, this would:");
    println!("  1. Launch the 3D Bevy + Rapier simulator");
    println!("  2. Visualize the robot executing the motion");
    println!("  3. Allow interactive control via keyboard/mouse");
    println!("\nTo run the full 3D simulator, use:");
    println!("  cargo run --bin humanoid-sim --features simulator");
}
