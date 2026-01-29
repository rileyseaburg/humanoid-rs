//! ROS2 Bridge Example
//!
//! Demonstrates ROS2 integration with the humanoid robot framework.
//! Shows how to publish joint states, subscribe to commands, and synchronize clocks.
//!
//! Run with: `cargo run --example ros2_bridge --features ros2`
//!
//! Prerequisites:
//!   - ROS2 Humble or Iron installed
//!   - Source ROS2 environment: source /opt/ros/humble/setup.bash

use humanoid_rs::robot::{unitree_g1, JointSet, Pose, RobotState};
use humanoid_rs::ros2::{
    frames,
    msg::{JointState, JointTrajectory, JointTrajectoryPoint, Twist},
    topics, FollowJointTrajectoryAction, QosProfile, Ros2Bridge,
};
use std::thread;
use std::time::{Duration, Instant};

/// ROS2 node configuration
struct Ros2Config {
    node_name: String,
    namespace: String,
    publish_rate_hz: f64,
    cmd_timeout_ms: u64,
}

impl Default for Ros2Config {
    fn default() -> Self {
        Self {
            node_name: "humanoid_robot".to_string(),
            namespace: "/humanoid".to_string(),
            publish_rate_hz: 100.0,
            cmd_timeout_ms: 500,
        }
    }
}

/// Example ROS2 node that bridges humanoid robot with ROS2 ecosystem
struct HumanoidRos2Node {
    bridge: Ros2Bridge,
    config: Ros2Config,
    robot_state: RobotState,
    last_cmd_time: Instant,
    trajectory_action: FollowJointTrajectoryAction,
}

impl HumanoidRos2Node {
    fn new(config: Ros2Config) -> Self {
        println!("Creating ROS2 bridge...");
        let bridge = Ros2Bridge::new(&config.node_name, &config.namespace);
        let trajectory_action = FollowJointTrajectoryAction::new(&format!(
            "{}/follow_joint_trajectory",
            config.namespace
        ));

        Self {
            bridge,
            config,
            robot_state: RobotState {
                joints: unitree_g1::create_default_joints(),
                ..Default::default()
            },
            last_cmd_time: Instant::now(),
            trajectory_action,
        }
    }

    fn initialize(&self) -> Result<(), Box<dyn std::error::Error>> {
        println!("\nInitializing ROS2 node...");
        self.bridge.init()?;

        println!("\nROS2 Bridge Configuration:");
        println!("  Node name: {}", self.config.node_name);
        println!("  Namespace: {}", self.config.namespace);
        println!("  Publishing rate: {:.0} Hz", self.config.publish_rate_hz);

        println!("\nPublished Topics:");
        println!(
            "  - {}/joint_states (sensor_msgs/JointState)",
            self.config.namespace
        );
        println!("  - {}/pose (geometry_msgs/Pose)", self.config.namespace);
        println!("  - {}/tf (tf2_msgs/TFMessage)", self.config.namespace);

        println!("\nSubscribed Topics:");
        println!(
            "  - {}/cmd_vel (geometry_msgs/Twist)",
            self.config.namespace
        );
        println!(
            "  - {}/joint_trajectory (trajectory_msgs/JointTrajectory)",
            self.config.namespace
        );

        println!("\nActions:");
        println!(
            "  - {}/follow_joint_trajectory (control_msgs/FollowJointTrajectory)",
            self.config.namespace
        );

        Ok(())
    }

    fn run(&mut self, duration: Duration) -> Result<(), Box<dyn std::error::Error>> {
        println!("\nRunning ROS2 bridge for {:?}...\n", duration);

        let publish_period = Duration::from_secs_f64(1.0 / self.config.publish_rate_hz);
        let start_time = Instant::now();
        let mut last_publish = Instant::now();
        let mut publish_count = 0;

        while start_time.elapsed() < duration {
            let loop_start = Instant::now();

            // 1. Update robot state (in real usage, this would come from hardware)
            self.update_robot_state();

            // 2. Publish state to ROS2 topics
            if last_publish.elapsed() >= publish_period {
                self.bridge.publish_state(&self.robot_state)?;
                self.bridge.broadcast_tf(&self.robot_state)?;
                publish_count += 1;
                last_publish = Instant::now();

                if publish_count % 100 == 0 {
                    println!("  Published {} messages", publish_count);
                }
            }

            // 3. Process incoming commands
            self.process_commands()?;

            // 4. Handle trajectory action
            self.process_trajectory_action()?;

            // 5. Spin ROS2 (process callbacks)
            self.bridge.spin_once(Duration::from_millis(1))?;

            // Maintain loop rate
            let elapsed = loop_start.elapsed();
            if elapsed < publish_period {
                thread::sleep(publish_period - elapsed);
            }
        }

        println!("\nPublished {} messages over {:?}", publish_count, duration);
        println!(
            "Actual publish rate: {:.1} Hz",
            publish_count as f64 / duration.as_secs_f64()
        );

        Ok(())
    }

    fn update_robot_state(&mut self) {
        // Simulate robot state changes
        // In real usage, this would read from hardware
        let t = self.robot_state.timestamp.as_secs_f64();

        // Simulate slight swaying motion
        self.robot_state.base_pose.position.x = 0.01 * (2.0 * std::f64::consts::PI * 0.5 * t).sin();
        self.robot_state.base_pose.position.z =
            0.005 * (2.0 * std::f64::consts::PI * 0.3 * t).cos();

        // Simulate joint motion
        for (i, joint) in self.robot_state.joints.iter_mut().enumerate() {
            let freq = 0.2 + (i as f64 * 0.02);
            joint.current_position = 0.05 * (2.0 * std::f64::consts::PI * freq * t).sin();
            joint.velocity = 0.05
                * 2.0
                * std::f64::consts::PI
                * freq
                * (2.0 * std::f64::consts::PI * freq * t).cos();
        }

        self.robot_state.timestamp += Duration::from_millis(10);
    }

    fn process_commands(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Check for velocity commands
        let cmd_vel = self.bridge.get_cmd_vel();

        if cmd_vel.linear.x != 0.0 || cmd_vel.linear.y != 0.0 || cmd_vel.angular.z != 0.0 {
            println!(
                "  Received cmd_vel: linear=[{:.2}, {:.2}, {:.2}], angular=[{:.2}, {:.2}, {:.2}]",
                cmd_vel.linear.x,
                cmd_vel.linear.y,
                cmd_vel.linear.z,
                cmd_vel.angular.x,
                cmd_vel.angular.y,
                cmd_vel.angular.z
            );
            self.last_cmd_time = Instant::now();
        }

        // Check for trajectory commands
        if let Some(trajectory) = self.bridge.take_trajectory() {
            println!(
                "  Received trajectory with {} points for joints: {:?}",
                trajectory.points.len(),
                trajectory.joint_names
            );
            self.trajectory_action.accept_goal(trajectory)?;
        }

        Ok(())
    }

    fn process_trajectory_action(&self) -> Result<(), Box<dyn std::error::Error>> {
        if self.trajectory_action.is_executing() {
            let elapsed = self.robot_state.timestamp;
            if let Some(setpoint) = self.trajectory_action.get_current_setpoint(elapsed) {
                // Apply trajectory setpoint to robot
                // In real usage, this would command the hardware
                if elapsed.as_secs_f64() % 1.0 < 0.01 {
                    println!(
                        "  Executing trajectory, {} points remaining",
                        setpoint.positions.len()
                    );
                }
            } else {
                // Trajectory complete
                println!("  Trajectory execution complete");
            }
        }

        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Humanoid-RS: ROS2 Bridge Example");
    println!("==================================\n");

    // Configuration
    let config = Ros2Config::default();

    // Create and initialize ROS2 node
    let mut node = HumanoidRos2Node::new(config);
    node.initialize()?;

    // Run the bridge
    let duration = Duration::from_secs(10);
    node.run(duration)?;

    println!("\n\nROS2 Integration Summary:");
    println!("  The ROS2 bridge provides:");
    println!("    1. Joint state publishing (position, velocity, effort)");
    println!("    2. Base pose publishing (position, orientation)");
    println!("    3. TF tree broadcasting (link transforms)");
    println!("    4. Velocity command subscription (cmd_vel)");
    println!("    5. Trajectory command subscription (joint_trajectory)");
    println!("    6. Action server for trajectory following");

    println!("\nROS2 Ecosystem Integration:");
    println!("  - rviz2: Visualize robot state and TF tree");
    println!("  - rqt_plot: Plot joint positions and velocities");
    println!("  - rosbag2: Record and playback sensor data");
    println!("  - moveit2: Motion planning and collision checking");
    println!("  - nav2: Navigation and path planning");

    println!("\nClock Synchronization:");
    println!("  Use ROS2 time for synchronized logging:");
    println!("    let ros_time = bridge.get_ros_time();");
    println!("    robot_state.timestamp = ros_time;");

    println!("\nFor real hardware integration:");
    println!("  1. Connect to robot: UnitreeRobot::connect(\"192.168.123.104:8080\").await?");
    println!("  2. Create ROS2 bridge: Ros2Bridge::new(\"unitree_g1\", \"/robot\")");
    println!("  3. Map joint names between robot and ROS2");
    println!("  4. Convert coordinate frames if needed");
    println!("  5. Run publisher loop at 100-500 Hz");

    Ok(())
}
