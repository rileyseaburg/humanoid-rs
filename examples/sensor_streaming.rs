//! Sensor Streaming Example
//!
//! Demonstrates accessing sensor data including IMU readings, joint encoders, and contact detection.
//!
//! Run with: `cargo run --example sensor_streaming --features unitree`

use humanoid_rs::robot::{unitree_g1, JointSet, Pose, RobotState};
use humanoid_rs::unitree::{RobotMode, UnitreeRobot};
use std::thread;
use std::time::{Duration, Instant};

/// Sensor data logger for recording and analyzing sensor streams
struct SensorLogger {
    imu_history: Vec<ImuReading>,
    joint_history: Vec<JointStateReading>,
    contact_history: Vec<ContactReading>,
    max_history: usize,
}

#[derive(Debug, Clone)]
struct ImuReading {
    timestamp: Duration,
    orientation: [f64; 4],         // Quaternion [w, x, y, z]
    angular_velocity: [f64; 3],    // rad/s
    linear_acceleration: [f64; 3], // m/s²
}

#[derive(Debug, Clone)]
struct JointStateReading {
    timestamp: Duration,
    positions: Vec<f64>,
    velocities: Vec<f64>,
    efforts: Vec<f64>,
}

#[derive(Debug, Clone)]
struct ContactReading {
    timestamp: Duration,
    left_foot_contact: bool,
    right_foot_contact: bool,
    left_foot_force: f64,
    right_foot_force: f64,
}

impl SensorLogger {
    fn new(max_history: usize) -> Self {
        Self {
            imu_history: Vec::with_capacity(max_history),
            joint_history: Vec::with_capacity(max_history),
            contact_history: Vec::with_capacity(max_history),
            max_history,
        }
    }

    fn log_imu(&mut self, reading: ImuReading) {
        if self.imu_history.len() >= self.max_history {
            self.imu_history.remove(0);
        }
        self.imu_history.push(reading);
    }

    fn log_joints(&mut self, reading: JointStateReading) {
        if self.joint_history.len() >= self.max_history {
            self.joint_history.remove(0);
        }
        self.joint_history.push(reading);
    }

    fn log_contacts(&mut self, reading: ContactReading) {
        if self.contact_history.len() >= self.max_history {
            self.contact_history.remove(0);
        }
        self.contact_history.push(reading);
    }

    fn print_statistics(&self) {
        println!("\n=== Sensor Statistics ===");

        // IMU Statistics
        if !self.imu_history.is_empty() {
            println!("\nIMU Data ({} samples):", self.imu_history.len());
            let latest = self.imu_history.last().unwrap();
            println!(
                "  Latest Orientation: [{:.3}, {:.3}, {:.3}, {:.3}]",
                latest.orientation[0],
                latest.orientation[1],
                latest.orientation[2],
                latest.orientation[3]
            );
            println!(
                "  Angular Velocity: [{:.3}, {:.3}, {:.3}] rad/s",
                latest.angular_velocity[0], latest.angular_velocity[1], latest.angular_velocity[2]
            );
            println!(
                "  Linear Acceleration: [{:.3}, {:.3}, {:.3}] m/s²",
                latest.linear_acceleration[0],
                latest.linear_acceleration[1],
                latest.linear_acceleration[2]
            );

            // Calculate angular velocity statistics
            let avg_ang_vel: [f64; 3] = [
                self.imu_history
                    .iter()
                    .map(|r| r.angular_velocity[0])
                    .sum::<f64>()
                    / self.imu_history.len() as f64,
                self.imu_history
                    .iter()
                    .map(|r| r.angular_velocity[1])
                    .sum::<f64>()
                    / self.imu_history.len() as f64,
                self.imu_history
                    .iter()
                    .map(|r| r.angular_velocity[2])
                    .sum::<f64>()
                    / self.imu_history.len() as f64,
            ];
            println!(
                "  Avg Angular Velocity: [{:.3}, {:.3}, {:.3}] rad/s",
                avg_ang_vel[0], avg_ang_vel[1], avg_ang_vel[2]
            );
        }

        // Joint Statistics
        if !self.joint_history.is_empty() {
            println!("\nJoint Data ({} samples):", self.joint_history.len());
            let latest = self.joint_history.last().unwrap();
            println!("  Number of joints: {}", latest.positions.len());

            // Find joints with largest position change
            if self.joint_history.len() >= 2 {
                let prev = &self.joint_history[self.joint_history.len() - 2];
                let mut max_change = 0.0;
                let mut max_change_idx = 0;
                for (i, (curr, prev_pos)) in latest
                    .positions
                    .iter()
                    .zip(prev.positions.iter())
                    .enumerate()
                {
                    let change = (curr - prev_pos).abs();
                    if change > max_change {
                        max_change = change;
                        max_change_idx = i;
                    }
                }
                println!(
                    "  Max position change: joint {} with {:.4} rad",
                    max_change_idx, max_change
                );
            }
        }

        // Contact Statistics
        if !self.contact_history.is_empty() {
            println!("\nContact Data ({} samples):", self.contact_history.len());
            let latest = self.contact_history.last().unwrap();
            println!(
                "  Left foot: {} (force: {:.1} N)",
                if latest.left_foot_contact {
                    "CONTACT"
                } else {
                    "AIR"
                },
                latest.left_foot_force
            );
            println!(
                "  Right foot: {} (force: {:.1} N)",
                if latest.right_foot_contact {
                    "CONTACT"
                } else {
                    "AIR"
                },
                latest.right_foot_force
            );

            // Calculate contact duty cycle
            let left_contact_ratio = self
                .contact_history
                .iter()
                .filter(|r| r.left_foot_contact)
                .count() as f64
                / self.contact_history.len() as f64;
            let right_contact_ratio = self
                .contact_history
                .iter()
                .filter(|r| r.right_foot_contact)
                .count() as f64
                / self.contact_history.len() as f64;
            println!("  Left foot duty cycle: {:.1}%", left_contact_ratio * 100.0);
            println!(
                "  Right foot duty cycle: {:.1}%",
                right_contact_ratio * 100.0
            );
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Humanoid-RS: Sensor Streaming Example");
    println!("======================================\n");

    // Create a mock robot for demonstration (no hardware required)
    println!("Creating mock robot for sensor demonstration...");
    let mut robot = UnitreeRobot::mock();
    println!("  Connected: {}", robot.is_connected());
    println!("  Mode: {:?}", robot.mode());
    println!("  Number of joints: {}\n", unitree_g1::NUM_JOINTS);

    // Initialize sensor logger
    let mut logger = SensorLogger::new(1000); // Keep last 1000 samples

    // Simulate sensor streaming at 100 Hz
    let sensor_rate_hz = 100.0;
    let dt = Duration::from_secs_f64(1.0 / sensor_rate_hz);
    let duration = Duration::from_secs(10);
    let num_samples = (duration.as_secs_f64() * sensor_rate_hz) as usize;

    println!(
        "Streaming sensors at {:.0} Hz for {} seconds...\n",
        sensor_rate_hz,
        duration.as_secs()
    );

    let start_time = Instant::now();

    for i in 0..num_samples {
        let timestamp = start_time.elapsed();

        // Simulate reading sensors from robot
        // In real usage, this would come from robot.poll_state() or similar

        // 1. Read IMU data
        let imu_reading = simulate_imu_reading(timestamp, i);
        logger.log_imu(imu_reading);

        // 2. Read joint encoders
        let joint_reading = simulate_joint_reading(&robot, timestamp);
        logger.log_joints(joint_reading);

        // 3. Read contact sensors
        let contact_reading = simulate_contact_reading(timestamp, i, num_samples);
        logger.log_contacts(contact_reading);

        // Print progress every second
        if i % sensor_rate_hz as usize == 0 && i > 0 {
            let elapsed_sec = i as f64 / sensor_rate_hz;
            println!("  Time: {:.0}s - Collected {} samples", elapsed_sec, i);

            // Print current sensor state
            if let Some(imu) = logger.imu_history.last() {
                println!(
                    "    IMU - Roll: {:.2}°, Pitch: {:.2}°",
                    quaternion_to_roll(imu.orientation) * 180.0 / std::f64::consts::PI,
                    quaternion_to_pitch(imu.orientation) * 180.0 / std::f64::consts::PI
                );
            }
        }

        // Maintain sensor rate
        thread::sleep(dt);
    }

    // Print final statistics
    logger.print_statistics();

    println!("\n\nSensor Streaming Summary:");
    println!("  Total samples collected: {}", num_samples);
    println!("  Data rates:");
    println!("    - IMU: Typically 200-1000 Hz (high-speed orientation)");
    println!("    - Joint encoders: 100-500 Hz (position/velocity)");
    println!("    - Contact sensors: 100-200 Hz (foot state)");
    println!("    - Force/torque: 500-1000 Hz (if available)");

    println!("\nReal Hardware Usage:");
    println!("  To connect to real Unitree robot:");
    println!("    let robot = UnitreeRobot::connect(\"192.168.123.104:8080\").await?;");
    println!("    robot.enable_low_level_control()?;");
    println!("    loop {{");
    println!("        robot.poll_state()?;");
    println!("        let state = robot.state();");
    println!("        // Process sensor data...");
    println!("    }}");

    Ok(())
}

/// Simulate IMU reading with synthetic data
fn simulate_imu_reading(timestamp: Duration, sample_idx: usize) -> ImuReading {
    let t = timestamp.as_secs_f64();
    let freq = 0.5; // 0.5 Hz oscillation

    // Simulate robot swaying slightly
    let roll = 0.05 * (2.0 * std::f64::consts::PI * freq * t).sin();
    let pitch = 0.03 * (2.0 * std::f64::consts::PI * freq * t).cos();
    let yaw = 0.01 * t; // Slow rotation

    let orientation = euler_to_quaternion(roll, pitch, yaw);

    ImuReading {
        timestamp,
        orientation,
        angular_velocity: [
            0.1 * (2.0 * std::f64::consts::PI * freq * t).cos(),
            -0.06 * (2.0 * std::f64::consts::PI * freq * t).sin(),
            0.01,
        ],
        linear_acceleration: [
            0.1 * (2.0 * std::f64::consts::PI * 2.0 * freq * t).sin(),
            -9.81, // Gravity
            0.05 * (2.0 * std::f64::consts::PI * freq * t).cos(),
        ],
    }
}

/// Simulate joint encoder reading
fn simulate_joint_reading(robot: &UnitreeRobot, timestamp: Duration) -> JointStateReading {
    let num_joints = unitree_g1::NUM_JOINTS;

    // Simulate slowly varying joint positions
    let t = timestamp.as_secs_f64();
    let positions: Vec<f64> = (0..num_joints)
        .map(|i| {
            let freq = 0.3 + (i as f64 * 0.05);
            let amp = 0.1 + (i as f64 * 0.01);
            amp * (2.0 * std::f64::consts::PI * freq * t + i as f64).sin()
        })
        .collect();

    let velocities: Vec<f64> = positions
        .iter()
        .enumerate()
        .map(|(i, _)| {
            let freq = 0.3 + (i as f64 * 0.05);
            let amp = 0.1 + (i as f64 * 0.01);
            2.0 * std::f64::consts::PI
                * freq
                * amp
                * (2.0 * std::f64::consts::PI * freq * t + i as f64).cos()
        })
        .collect();

    let efforts: Vec<f64> = vec![0.0; num_joints]; // Simplified

    JointStateReading {
        timestamp,
        positions,
        velocities,
        efforts,
    }
}

/// Simulate contact sensor reading
fn simulate_contact_reading(
    timestamp: Duration,
    sample_idx: usize,
    total_samples: usize,
) -> ContactReading {
    let t = timestamp.as_secs_f64();

    // Simulate walking gait - alternating contact
    let gait_period = 1.0; // 1 second gait period
    let phase = (t % gait_period) / gait_period;

    let left_contact = phase < 0.5;
    let right_contact = phase >= 0.5;

    // Simulate force based on contact state
    let left_force = if left_contact {
        200.0 + 50.0 * (phase * 2.0 * std::f64::consts::PI).sin()
    } else {
        0.0
    };

    let right_force = if right_contact {
        200.0 + 50.0 * ((phase - 0.5) * 2.0 * std::f64::consts::PI).sin()
    } else {
        0.0
    };

    ContactReading {
        timestamp,
        left_foot_contact: left_contact,
        right_foot_contact: right_contact,
        left_foot_force: left_force,
        right_foot_force: right_force,
    }
}

/// Convert Euler angles to quaternion [w, x, y, z]
fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();

    [
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    ]
}

/// Extract roll from quaternion
fn quaternion_to_roll(q: [f64; 4]) -> f64 {
    let sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]);
    let cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    sinr_cosp.atan2(cosr_cosp)
}

/// Extract pitch from quaternion
fn quaternion_to_pitch(q: [f64; 4]) -> f64 {
    let sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
    if sinp.abs() >= 1.0 {
        sinp.signum() * std::f64::consts::PI / 2.0
    } else {
        sinp.asin()
    }
}
