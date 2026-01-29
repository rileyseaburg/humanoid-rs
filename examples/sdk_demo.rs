//! SDK Demo Example
//!
//! This example demonstrates the high-level SDK API for controlling
//! humanoid robots.

use humanoid_rs::sdk::prelude::*;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Humanoid SDK Demo");
    println!("=================\n");

    // Example 1: Create SDK with default configuration
    println!("1. Creating SDK with default config...");
    let sdk = HumanoidSDK::new();
    println!("   ✓ SDK created\n");

    // Example 2: Load configuration from file
    println!("2. Loading configuration from file...");
    let config_path = std::path::Path::new("config.toml");

    // Try to load from file, or use default if not found
    let mut sdk = if config_path.exists() {
        println!("   Loading from config.toml...");
        HumanoidSDK::from_file(config_path)?
    } else {
        println!("   Config file not found, using defaults...");
        HumanoidSDK::with_config(SDKConfig::default())
    };
    println!("   ✓ SDK initialized with configuration\n");

    // Example 3: Connect to simulator
    println!("3. Connecting to simulator...");
    let sim_config = SimulationConfig {
        timestep: Duration::from_millis(2), // 500 Hz
        real_time_factor: 1.0,
        headless: false,
        scene_path: None,
    };

    match sdk.connect_simulator(sim_config) {
        Ok(handle) => println!("   ✓ Connected to simulator (handle: {})\n", handle.id),
        Err(e) => println!("   ⚠ Could not connect to simulator: {}\n", e),
    }

    // Example 4: Connect to a Unitree G1 robot
    println!("4. Connecting to Unitree G1 robot...");
    let profile = RobotProfile::unitree_g1();
    println!("   Profile: {}", profile.name);
    println!("   Joints: {}", profile.num_joints);
    println!("   Mass: {} kg", profile.mass);
    println!("   Height: {} m", profile.height);

    match sdk.connect_robot(profile) {
        Ok(robot) => {
            println!("   ✓ Connected to robot (handle: {})\n", robot.id);

            // Example 5: Use high-level commands
            println!("5. Sending high-level commands...");
            match robot.stand_up() {
                Ok(_) => println!("   ✓ Robot standing up"),
                Err(e) => println!("   ⚠ Command failed: {}", e),
            }

            // In a real scenario, you would add delays or wait for completion
            // std::thread::sleep(Duration::from_secs(2));

            match robot.sit_down() {
                Ok(_) => println!("   ✓ Robot sitting down\n"),
                Err(e) => println!("   ⚠ Command failed: {}\n", e),
            }
        }
        Err(e) => println!("   ⚠ Could not connect to robot: {}\n", e),
    }

    // Example 6: Demonstrate configuration validation
    println!("6. Configuration validation...");
    let config = sdk.config();
    match config.validate() {
        Ok(_) => println!("   ✓ Configuration is valid\n"),
        Err(e) => println!("   ✗ Configuration invalid: {}\n", e),
    }

    // Example 7: Robot capabilities
    println!("7. Robot capabilities...");
    let profile = RobotProfile::unitree_g1();
    let caps = profile.capabilities();
    println!("   Low-level control: {}", caps.supports_low_level_control);
    println!(
        "   High-level commands: {}",
        caps.supports_high_level_commands
    );
    println!("   Max joints: {}\n", caps.max_torque.len());

    // Example 8: Joint lookup
    println!("8. Joint lookup...");
    if let Some(idx) = profile.get_joint_index("left_hip_pitch") {
        println!("   ✓ left_hip_pitch found at index {}", idx);
        if let Some(limits) = profile.get_joint_limits("left_hip_pitch") {
            println!("   Limits: [{:.2}, {:.2}] rad", limits.min, limits.max);
            println!("   Max velocity: {:.2} rad/s", limits.max_velocity);
            println!("   Max effort: {:.2} Nm\n", limits.max_effort);
        }
    }

    // Example 9: Create custom configuration
    println!("9. Creating custom configuration...");
    let custom_config = SDKConfig {
        physics: humanoid_rs::sdk::config::PhysicsConfig {
            timestep: 0.001, // 1000 Hz
            ..Default::default()
        },
        robot: humanoid_rs::sdk::config::RobotConfig {
            model: "custom_robot".to_string(),
            ..Default::default()
        },
        control: humanoid_rs::sdk::config::ControlConfig {
            control_frequency: 1000.0,
            kp: 200.0,
            kd: 20.0,
            ..Default::default()
        },
        logging: humanoid_rs::sdk::config::LoggingConfig {
            level: "debug".to_string(),
            ..Default::default()
        },
    };

    let mut custom_sdk = HumanoidSDK::with_config(custom_config);
    println!("   ✓ Custom SDK created");
    println!(
        "   Physics timestep: {} s",
        custom_sdk.config().physics.timestep
    );
    println!(
        "   Control frequency: {} Hz\n",
        custom_sdk.config().control.control_frequency
    );

    // Example 10: Shutdown
    println!("10. Shutting down SDK...");
    custom_sdk.shutdown();
    sdk.shutdown();
    println!("    ✓ SDK shutdown complete\n");

    println!("Demo completed successfully!");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sdk_creation() {
        let sdk = HumanoidSDK::new();
        assert!(!sdk.has_simulation());
        assert!(!sdk.has_robot());
    }

    #[test]
    fn test_robot_profile() {
        let profile = RobotProfile::unitree_g1();
        assert_eq!(profile.name, "Unitree G1");
        assert!(profile.num_joints > 0);
    }

    #[test]
    fn test_simulation_config() {
        let config = SimulationConfig::default();
        assert!(config.validate().is_ok());
        assert_eq!(config.timestep, Duration::from_millis(2));
    }
}
