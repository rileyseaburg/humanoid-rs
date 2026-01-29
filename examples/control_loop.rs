//! Control Loop Example
//!
//! Demonstrates PD controller usage, trajectory following, and real-time control rates.
//!
//! Run with: `cargo run --example control_loop --features simulator`

use humanoid_rs::control::{Controller, JointSpaceController, MotionPlanner, Trajectory, Waypoint};
use humanoid_rs::robot::{unitree_g1, JointSet, Pose, RobotState};
use std::thread;
use std::time::Duration;

/// Real-time control loop statistics
struct ControlStats {
    target_rate_hz: f64,
    actual_rate_hz: f64,
    max_latency_ms: f64,
    min_latency_ms: f64,
    avg_latency_ms: f64,
    control_errors: Vec<f64>,
}

impl ControlStats {
    fn new(target_rate_hz: f64) -> Self {
        Self {
            target_rate_hz,
            actual_rate_hz: 0.0,
            max_latency_ms: 0.0,
            min_latency_ms: f64::MAX,
            avg_latency_ms: 0.0,
            control_errors: Vec::new(),
        }
    }

    fn print_summary(&self) {
        println!("\nControl Loop Statistics:");
        println!("  Target rate:     {:.1} Hz", self.target_rate_hz);
        println!("  Actual rate:     {:.1} Hz", self.actual_rate_hz);
        println!("  Max latency:     {:.3} ms", self.max_latency_ms);
        println!("  Min latency:     {:.3} ms", self.min_latency_ms);
        println!("  Avg latency:     {:.3} ms", self.avg_latency_ms);
        if !self.control_errors.is_empty() {
            let avg_error: f64 =
                self.control_errors.iter().sum::<f64>() / self.control_errors.len() as f64;
            let max_error: f64 = self.control_errors.iter().cloned().fold(0.0, f64::max);
            println!("  Avg position error: {:.4} rad", avg_error);
            println!("  Max position error: {:.4} rad", max_error);
        }
    }
}

fn main() {
    println!("Humanoid-RS: Control Loop Example");
    println!("===================================\n");

    // Create initial robot state
    let mut robot_state = RobotState {
        joints: unitree_g1::create_default_joints(),
        ..Default::default()
    };

    // Create a sinusoidal trajectory for smooth motion
    println!("Generating sinusoidal trajectory...");
    let trajectory = create_sinusoidal_trajectory(&robot_state.joints, Duration::from_secs(5));
    println!("  Duration: {:?}", trajectory.duration());
    println!("  Waypoints: generated at 50Hz\n");

    // Create PD controller with gains
    // Kp = 100.0 (proportional gain - stiffness)
    // Kd = 20.0 (derivative gain - damping)
    println!("Creating PD controller:");
    println!("  Kp = 100.0 (stiffness)");
    println!("  Kd = 20.0 (damping)");
    let mut controller = JointSpaceController::new(100.0, 20.0);
    controller.set_trajectory(trajectory);

    // Run control loop at different rates
    let control_rates = vec![50.0, 100.0, 200.0, 500.0, 1000.0];

    for &target_rate_hz in &control_rates {
        println!("\n--- Running at {:.0} Hz ---", target_rate_hz);

        let dt = Duration::from_secs_f64(1.0 / target_rate_hz);
        let duration = Duration::from_secs(3); // 3 seconds per rate test
        let num_steps = (duration.as_secs_f64() * target_rate_hz) as usize;

        let mut stats = ControlStats::new(target_rate_hz);
        let mut latencies = Vec::new();

        // Reset controller
        controller.set_trajectory(create_sinusoidal_trajectory(
            &robot_state.joints,
            Duration::from_secs(3),
        ));

        let start_time = std::time::Instant::now();

        for step in 0..num_steps {
            let loop_start = std::time::Instant::now();

            // Update controller with current state
            match controller.update(&robot_state, dt) {
                Ok(output) => {
                    // Calculate tracking error for first joint
                    let target_pos =
                        get_target_position(&controller, step as f64 * dt.as_secs_f64());
                    let actual_pos = robot_state
                        .joints
                        .iter()
                        .next()
                        .map(|j| j.current_position)
                        .unwrap_or(0.0);
                    let error = (target_pos - actual_pos).abs();
                    stats.control_errors.push(error);

                    // Update robot state with control output (simulated)
                    for (joint, out_joint) in robot_state.joints.iter_mut().zip(output.iter()) {
                        // Simulate simple dynamics: position moves toward target
                        let alpha = 0.1; // Simple low-pass filter
                        joint.current_position = joint.current_position * (1.0 - alpha)
                            + out_joint.target_position * alpha;
                        joint.velocity = out_joint.effort / 100.0; // Simplified
                    }
                }
                Err(e) => {
                    eprintln!("Control error: {}", e);
                    break;
                }
            }

            // Measure latency
            let latency = loop_start.elapsed();
            let latency_ms = latency.as_secs_f64() * 1000.0;
            latencies.push(latency_ms);
            stats.max_latency_ms = stats.max_latency_ms.max(latency_ms);
            stats.min_latency_ms = stats.min_latency_ms.min(latency_ms);

            // Sleep to maintain control rate
            let elapsed = loop_start.elapsed();
            if elapsed < dt {
                thread::sleep(dt - elapsed);
            }
        }

        let total_elapsed = start_time.elapsed();
        stats.actual_rate_hz = num_steps as f64 / total_elapsed.as_secs_f64();
        stats.avg_latency_ms = latencies.iter().sum::<f64>() / latencies.len() as f64;

        stats.print_summary();
    }

    println!("\n\nControl Loop Best Practices:");
    println!("  1. Use high control rates (500-1000 Hz) for stability");
    println!("  2. Keep control loop jitter under 1 ms");
    println!("  3. Use real-time thread priority on Linux");
    println!("  4. Pre-allocate all memory in control loop");
    println!("  5. Minimize allocations and syscalls");
    println!("  6. Use lock-free data structures for inter-thread communication");
    println!("  7. Monitor control loop timing with histograms");
}

/// Create a sinusoidal trajectory for smooth joint motion
fn create_sinusoidal_trajectory(joints: &JointSet, duration: Duration) -> Trajectory {
    let mut trajectory = Trajectory::new();
    let num_points = (duration.as_secs_f64() * 50.0) as usize; // 50 Hz trajectory

    for i in 0..=num_points {
        let t = duration.as_secs_f64() * (i as f64 / num_points as f64);
        let time = Duration::from_secs_f64(t);

        // Create sinusoidal motion for each joint
        let joint_positions: Vec<f64> = joints
            .iter()
            .enumerate()
            .map(|(idx, _)| {
                let frequency = 0.5 + (idx as f64 * 0.1); // Different frequency per joint
                let amplitude = 0.3; // 0.3 rad amplitude
                amplitude * (2.0 * std::f64::consts::PI * frequency * t).sin()
            })
            .collect();

        trajectory.add_waypoint(Waypoint {
            time,
            joint_positions,
            base_pose: None,
        });
    }

    trajectory
}

/// Get target position from controller for error calculation
fn get_target_position(controller: &JointSpaceController, time: f64) -> f64 {
    // This is a simplified version - in practice you'd query the trajectory directly
    // For demo purposes, return a simple sinusoidal value
    0.3 * (2.0 * std::f64::consts::PI * 0.5 * time).sin()
}
