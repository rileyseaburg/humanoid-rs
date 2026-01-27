//! Example: Control simulated robot
//! 
//! Run with: cargo run --example stand_up --features simulator

use humanoid_rs::robot::{unitree_g1, JointSet};
use humanoid_rs::control::{MotionPlanner, JointSpaceController};

fn main() {
    println!("Humanoid Robot Example");
    println!("======================\n");
    
    // Create default G1 robot
    let mut joints = unitree_g1::create_default_joints();
    
    println!("Created robot with {} joints", unitree_g1::NUM_JOINTS);
    println!("Joint names:");
    for joint in joints.iter().take(5) {
        println!("  - {}", joint.name);
    }
    println!("  ... and {} more\n", unitree_g1::NUM_JOINTS - 5);
    
    // Create motion planner
    let planner = MotionPlanner::new(2.0, 5.0);
    
    // Plan standing motion
    println!("Planning standing trajectory...");
    let trajectory = planner.plan_stand_up(&joints);
    println!("Trajectory duration: {:?}\n", trajectory.duration());
    
    // Create controller
    let mut controller = JointSpaceController::new(10.0, 1.0);
    controller.set_trajectory(trajectory);
    
    // Simulate control loop
    println!("Simulating control loop...");
    use std::time::Duration;
    
    let dt = Duration::from_millis(10);
    let state = humanoid_rs::robot::RobotState {
        joints: joints.clone(),
        ..Default::default()
    };
    
    for i in 0..10 {
        match controller.update(&state, dt) {
            Ok(output) => {
                println!("  Step {}: Generated {} joint efforts", i, output.iter().count());
            }
            Err(e) => {
                eprintln!("  Step {}: Error - {}", i, e);
            }
        }
    }
    
    println!("\nSimulation complete!");
    println!("In real usage, this would:");
    println!("  1. Connect to robot via UDP");
    println!("  2. Stream joint commands at 100-1000Hz");
    println!("  3. Monitor safety limits");
    println!("  4. Handle emergency stops");
}
