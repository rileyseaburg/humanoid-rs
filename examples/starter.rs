//! 🚀 Starter Example - Complete SDK Demo
//!
//! This is the recommended starting point for new users.
//! It demonstrates all major SDK features in a simple, well-documented program.
//!
//! Run with: `cargo run --example starter --features simulator`

use humanoid_rs::control::safety::{EStopGuard, JointLimitGuard, SafetyLevel, SafetyManager};
use humanoid_rs::sdk::prelude::*;
use humanoid_rs::sim::deterministic::{SimulationClock, TimestepMode};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("╔══════════════════════════════════════════════════════════╗");
    println!("║     Humanoid-RS Starter Example                          ║");
    println!("║     Complete SDK Demonstration                           ║");
    println!("╚══════════════════════════════════════════════════════════╝\n");

    // =========================================================================
    // PART 1: SDK Initialization
    // =========================================================================
    println!("📦 Part 1: Initializing SDK");
    println!("   ─────────────────────────────────────────────────────────\n");

    // Create SDK with default configuration
    let mut sdk = HumanoidSDK::new();
    println!("   ✓ SDK instance created");

    // You can also load from a config file:
    // let sdk = HumanoidSDK::from_file("config.toml")?;

    // =========================================================================
    // PART 2: Simulator Setup
    // =========================================================================
    println!("\n🎮 Part 2: Connecting to Simulator");
    println!("   ─────────────────────────────────────────────────────────\n");

    let sim_config = SimulationConfig {
        timestep: Duration::from_millis(2), // 500 Hz physics
        real_time_factor: 1.0,              // Real-time simulation
        headless: false,                    // Show 3D window
        scene_path: None,                   // Use default scene
    };

    let sim = sdk.connect_simulator(sim_config)?;
    println!("   ✓ Connected to simulator");
    println!("     - Handle ID: {}", sim.id);
    println!("     - Running: {}", sim.is_running);
    println!("     - Timestep: 2ms (500 Hz)");

    // =========================================================================
    // PART 3: Robot Connection
    // =========================================================================
    println!("\n🤖 Part 3: Loading Robot Profile");
    println!("   ─────────────────────────────────────────────────────────\n");

    // Choose a robot profile
    let profile = RobotProfile::unitree_g1();
    // Alternative: RobotProfile::unitree_h1()
    // Alternative: RobotProfile::from_urdf("robot.urdf")?

    let robot = sdk.connect_robot(profile.clone())?;
    println!("   ✓ Robot connected: {}", robot.profile.name);
    println!("     - Joints: {}", robot.profile.num_joints);
    println!("     - Height: {:.2} m", robot.profile.height);
    println!("     - Mass: {:.1} kg", robot.profile.mass);
    println!("     - Max velocity: {:.1} m/s", robot.profile.max_velocity);

    // =========================================================================
    // PART 4: Safety Setup
    // =========================================================================
    println!("\n🛡️  Part 4: Configuring Safety");
    println!("   ─────────────────────────────────────────────────────────\n");

    // Create safety manager with guards
    let mut safety = SafetyManager::new(SafetyLevel::Simulation);

    // Add joint limit guard
    let joint_limits: Vec<_> = (0..robot.profile.num_joints)
        .map(|i| humanoid_rs::robot::JointLimits {
            name: format!("joint_{}", i),
            min_position: -3.14,
            max_position: 3.14,
            max_velocity: 10.0,
            max_effort: 100.0,
        })
        .collect();

    safety.add_guard(Box::new(JointLimitGuard::new(&joint_limits)));
    println!(
        "   ✓ Joint limit guard added ({} joints)",
        joint_limits.len()
    );

    // Add emergency stop guard
    safety.add_guard(Box::new(EStopGuard::new()));
    println!("   ✓ Emergency stop guard added");

    // =========================================================================
    // PART 5: Deterministic Timing
    // =========================================================================
    println!("\n⏱️  Part 5: Setting up Deterministic Timing");
    println!("   ─────────────────────────────────────────────────────────\n");

    // Create a simulation clock for reproducible behavior
    let mut clock = SimulationClock::new(Duration::from_millis(2));
    clock.set_mode(TimestepMode::Fixed);
    println!("   ✓ Simulation clock created");
    println!("     - Mode: Fixed timestep");
    println!("     - Timestep: 2ms");

    // =========================================================================
    // PART 6: Control Loop
    // =========================================================================
    println!("\n🎛️  Part 6: Running Control Loop");
    println!("   ─────────────────────────────────────────────────────────\n");

    println!("   Executing robot commands...\n");

    // Command 1: Stand up
    println!("   ▶ Standing up...");
    robot.stand_up()?;
    simulate_delay(Duration::from_secs(2));
    println!("     ✓ Standing complete\n");

    // Command 2: Check capabilities
    println!("   ▶ Robot capabilities:");
    if robot.profile.supports_low_level_control {
        println!("     • Low-level control: Yes");
    }
    if robot.profile.supports_high_level_commands {
        println!("     • High-level commands: Yes");
    }
    println!();

    // Simulate a simple control loop
    let duration = Duration::from_secs(3);
    let dt = Duration::from_millis(100);
    let steps = duration.as_millis() / dt.as_millis();

    println!(
        "   ▶ Running {}-step control loop ({:.1}s)...",
        steps,
        duration.as_secs_f64()
    );

    for step in 0..steps {
        // In a real scenario, this would:
        // 1. Read sensor data
        // 2. Compute control output
        // 3. Check safety limits
        // 4. Send commands to robot

        if step % 10 == 0 {
            let progress = (step as f64 / steps as f64) * 100.0;
            print!("\r     Progress: {:.0}%", progress);
        }

        // Simulate work
        std::thread::sleep(Duration::from_millis(1));
    }
    println!("\r     ✓ Control loop complete (100%)\n");

    // Command 3: Sit down
    println!("   ▶ Sitting down...");
    robot.sit_down()?;
    simulate_delay(Duration::from_secs(1));
    println!("     ✓ Sitting complete\n");

    // =========================================================================
    // PART 7: Configuration Demo
    // =========================================================================
    println!("\n⚙️  Part 7: Configuration Management");
    println!("   ─────────────────────────────────────────────────────────\n");

    // Access current configuration
    let config = sdk.config();
    println!("   Current configuration:");
    println!("     - Physics timestep: {:?}", config.physics.timestep);
    println!(
        "     - Real-time factor: {:.1}",
        config.simulation.real_time_factor
    );
    println!("     - Headless mode: {}", config.simulation.headless);

    // Modify configuration
    let mut new_config = config.clone();
    new_config.simulation.real_time_factor = 2.0; // 2x speed
    println!("\n   ✓ Modified real-time factor to 2.0x");

    // =========================================================================
    // PART 8: Cleanup
    // =========================================================================
    println!("\n🧹 Part 8: Cleanup");
    println!("   ─────────────────────────────────────────────────────────\n");

    println!("   Shutting down SDK...");
    sdk.shutdown();
    println!("   ✓ All connections closed");
    println!("   ✓ Resources freed");

    // =========================================================================
    // Summary
    // =========================================================================
    println!("\n╔══════════════════════════════════════════════════════════╗");
    println!("║     ✅ Starter Example Complete!                         ║");
    println!("╚══════════════════════════════════════════════════════════╝\n");

    println!("📚 What you learned:");
    println!("   • SDK initialization and configuration");
    println!("   • Simulator connection with custom settings");
    println!("   • Robot profile loading (Unitree G1)");
    println!("   • Safety guard configuration");
    println!("   • Deterministic timing setup");
    println!("   • Command execution and control loops");
    println!("   • Proper resource cleanup\n");

    println!("🚀 Next steps:");
    println!("   1. Try the other examples: cargo run --example <name>");
    println!("      - basic_sim       : Minimal simulation");
    println!("      - control_loop    : PD control demo");
    println!("      - sensor_streaming: Sensor data access");
    println!("      - walking_demo    : Walking simulation");
    println!("   2. Read the tutorials in docs/tutorials/");
    println!("   3. Check the API reference at docs/api_reference.md");
    println!("   4. Build your own robot application!\n");

    Ok(())
}

/// Helper function to simulate a delay with visual feedback
fn simulate_delay(duration: Duration) {
    let steps = 10;
    let step_duration = duration / steps;

    for i in 0..=steps {
        let progress = (i as f64 / steps as f64) * 100.0;
        print!("\r     [{:10}] {:.0}%", "█".repeat(i), progress);
        std::io::Write::flush(&mut std::io::stdout()).unwrap();

        if i < steps {
            std::thread::sleep(step_duration);
        }
    }
    println!();
}
