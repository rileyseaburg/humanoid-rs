//! Example demonstrating the sensor framework
//!
//! This example shows how to use the sensor framework with noise models
//! to simulate realistic sensor data for a humanoid robot.

use humanoid_rs::sim::sensors::*;
use humanoid_rs::{SensorPhysicsBackend, SensorRegistry};
use std::time::Duration;

fn main() {
    println!("=== Sensor Framework Example ===\n");

    // Create a sensor registry to manage all sensors
    let mut registry = SensorRegistry::new();

    // 1. Create an IMU sensor with realistic noise
    let imu_config = ImuConfig {
        accel_noise: NoiseModel::RandomWalk {
            bias: 0.0,
            std_dev: 0.005,    // 5 mg white noise
            walk_rate: 0.0001, // Bias drift
        },
        gyro_noise: NoiseModel::RandomWalk {
            bias: 0.0,
            std_dev: 0.0005,    // 0.5 deg/s white noise
            walk_rate: 0.00001, // Bias drift
        },
        sample_rate: Duration::from_millis(1), // 1 kHz
        processing_delay: Duration::from_micros(500),
        simulate_temperature: true,
        temperature_noise: NoiseModel::gaussian(0.5),
    };

    let imu = ImuSensor::new("pelvis_imu", imu_config);
    registry.add_sensor("pelvis_imu", Box::new(imu));
    println!("Added IMU sensor: pelvis_imu");

    // 2. Create joint encoders with quantization noise
    let encoder_config = JointEncoderConfig {
        position_noise: NoiseModel::Quantization {
            resolution: 0.0001, // High resolution encoder
            bias: 0.0,
        },
        velocity_noise: NoiseModel::gaussian(0.001),
        effort_noise: NoiseModel::gaussian(0.01),
        sample_rate: Duration::from_millis(1),
        processing_delay: Duration::from_micros(100),
        joint_names: vec![
            "left_hip_pitch".to_string(),
            "left_hip_roll".to_string(),
            "left_hip_yaw".to_string(),
            "left_knee".to_string(),
            "left_ankle_pitch".to_string(),
            "left_ankle_roll".to_string(),
        ],
    };

    let encoders = JointEncoderSensor::new("left_leg_encoders", encoder_config);
    registry.add_sensor("left_leg_encoders", Box::new(encoders));
    println!("Added joint encoder sensor: left_leg_encoders (6 joints)");

    // 3. Create force/torque sensors for feet
    let ft_config = ForceTorqueConfig {
        force_noise: NoiseModel::gaussian(0.5),    // 0.5 N noise
        torque_noise: NoiseModel::gaussian(0.025), // 0.025 Nm noise
        sample_rate: Duration::from_millis(1),
        processing_delay: Duration::from_micros(200),
        body_name: "left_foot".to_string(),
        offset: nalgebra::Vector3::zeros(),
    };

    let left_ft = ForceTorqueSensor::new("left_foot_ft", ft_config);
    registry.add_sensor("left_foot_ft", Box::new(left_ft));
    println!("Added F/T sensor: left_foot_ft");

    // 4. Create a camera sensor (placeholder)
    let camera_config = CameraConfig {
        width: 640,
        height: 480,
        intrinsics: [500.0, 500.0, 320.0, 240.0], // fx, fy, cx, cy
        sample_rate: Duration::from_millis(33),   // 30 Hz
        processing_delay: Duration::from_millis(5),
        enable_depth: true,
        depth_noise: NoiseModel::gaussian(0.001), // 1mm depth noise
    };

    let camera = CameraSensor::new("head_camera", camera_config);
    registry.add_sensor("head_camera", Box::new(camera));
    println!("Added camera sensor: head_camera (640x480 @ 30Hz)");

    // Print sensor registry summary
    println!("\n=== Sensor Registry Summary ===");
    println!("Total sensors: {}", registry.len());
    println!("Sensor names: {:?}", registry.sensor_names());
    println!("Sensor types: {:?}", registry.sensor_types());

    // Demonstrate noise models
    println!("\n=== Noise Model Examples ===");
    demonstrate_noise_models();

    println!("\n=== Example Complete ===");
    println!("The sensor framework provides:");
    println!("  - Multiple sensor types (IMU, encoders, F/T, cameras)");
    println!("  - Realistic noise models (Gaussian, random walk, quantization)");
    println!("  - Timestamped data with processing delays");
    println!("  - Sensor registry for centralized management");
    println!("  - Sensor synchronization support");
}

fn demonstrate_noise_models() {
    use rand::thread_rng;
    let mut rng = thread_rng();

    let value = 1.0;

    // Gaussian noise
    let gaussian = NoiseModel::gaussian(0.1);
    let noisy = gaussian.apply(value, &mut rng);
    println!("Gaussian noise: {} -> {:.4}", value, noisy);

    // Quantization
    let quant = NoiseModel::quantization(0.5);
    let quantized = quant.apply(value, &mut rng);
    println!("Quantization (step=0.5): {} -> {}", value, quantized);

    // Saturation
    let sat = NoiseModel::saturation(-0.5, 0.5, NoiseModel::gaussian(0.0));
    println!(
        "Saturation [-0.5, 0.5]: 0.3 -> {}",
        sat.apply(0.3, &mut rng)
    );
    println!(
        "Saturation [-0.5, 0.5]: 1.0 -> {}",
        sat.apply(1.0, &mut rng)
    );
    println!(
        "Saturation [-0.5, 0.5]: -1.0 -> {}",
        sat.apply(-1.0, &mut rng)
    );

    // Random walk (demonstrated through IMU)
    println!("\nRandom walk noise accumulates bias over time");
    println!("(see ImuSensor implementation for details)");
}
