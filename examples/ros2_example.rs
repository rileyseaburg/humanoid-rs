//! Example: ROS2 Integration
//!
//! This example demonstrates how to use the ROS2 node with the humanoid framework.
//!
//! To run:
//! ```bash
//! # Terminal 1: Start ROS2
//! source /opt/ros/humble/setup.bash
//! ros2 run humanoid_rs ros2_example
//!
//! # Terminal 2: Monitor topics
//! ros2 topic echo /joint_states
//! ros2 topic echo /imu/data
//! ```

#[cfg(feature = "ros2")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use humanoid::robot::{unitree_g1, JointSet, Pose, RobotState};
    use humanoid::ros2::{ClockMode, HumanoidRosNode, TimingPolicy};
    use std::thread;
    use std::time::Duration;

    println!("Starting ROS2 example...");

    // Create ROS node with simulation clock
    let ros_node = HumanoidRosNode::new("humanoid_example", ClockMode::WallTime)?;
    println!("ROS node created: {}", ros_node.node_name());

    // Configure timing policy
    let mut policy = TimingPolicy::default();
    policy.joint_state_rate = 50.0; // 50 Hz for demo
    policy.imu_rate = 100.0;
    policy.pose_rate = 50.0;

    ros_node.set_timing_policy(policy);
    println!("Timing policy configured");

    // Create initial robot state
    let mut robot_state = RobotState::default();
    robot_state.joints = unitree_g1::create_default_joints();
    robot_state.base_pose = Pose::new(0.0, 0.0, 0.8, 0.0, 0.0, 0.0);
    robot_state.is_standing = true;
    robot_state.is_balanced = true;

    println!(
        "Robot state initialized with {} joints",
        unitree_g1::NUM_JOINTS
    );
    println!("Running for 10 seconds... Press Ctrl+C to stop early");

    let start_time = std::time::Instant::now();
    let mut iteration = 0;

    // Main control loop
    loop {
        let elapsed = start_time.elapsed();

        // Update timestamp
        robot_state.timestamp = elapsed;

        // Simulate some motion (simple sine wave on hip joints)
        if let Some(joint) = robot_state.joints.get_joint_mut("left_hip_pitch") {
            joint.current_position = 0.1 * (elapsed.as_secs_f64() * 2.0).sin();
            joint.velocity = 0.2 * (elapsed.as_secs_f64() * 2.0).cos();
        }
        if let Some(joint) = robot_state.joints.get_joint_mut("right_hip_pitch") {
            joint.current_position = -0.1 * (elapsed.as_secs_f64() * 2.0).sin();
            joint.velocity = -0.2 * (elapsed.as_secs_f64() * 2.0).cos();
        }

        // Publish state to ROS2
        ros_node.publish_state(&robot_state)?;

        // Process callbacks (spin)
        ros_node.spin(Duration::from_millis(10))?;

        // Check for emergency stop
        if ros_node.is_emergency_stop_active() {
            println!("Emergency stop activated!");
            break;
        }

        // Check for commands
        let commands = ros_node.get_latest_commands();
        if !commands.is_empty() {
            println!("Received {} joint commands", commands.len());
        }

        // Print status every second
        iteration += 1;
        if iteration % 100 == 0 {
            println!(
                "Running... {:.1}s elapsed, hip pitch: {:.2} rad",
                elapsed.as_secs_f64(),
                robot_state
                    .joints
                    .get_joint("left_hip_pitch")
                    .map(|j| j.current_position)
                    .unwrap_or(0.0)
            );
        }

        // Run for 10 seconds
        if elapsed > Duration::from_secs(10) {
            println!("Demo complete!");
            break;
        }

        // Small delay to maintain ~100Hz
        thread::sleep(Duration::from_millis(10));
    }

    // Graceful shutdown
    ros_node.shutdown();
    println!("ROS node shutdown complete");

    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {
    eprintln!("This example requires the 'ros2' feature to be enabled.");
    eprintln!("Build with: cargo build --example ros2_example --features ros2");
    std::process::exit(1);
}
