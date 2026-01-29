# Quick Start Guide

Get up and running with Humanoid-RS in 5 minutes.

## Installation

```bash
# Clone the repository
git clone https://github.com/rileyseaburg/humanoid-rs
cd humanoid-rs

# Build with simulator support
cargo build --release --features simulator

# Run the simulator
cargo run --bin humanoid-sim --features simulator
```

## Your First Robot Program

Create a new project:

```bash
cargo new my_first_robot
cd my_first_robot
```

Add Humanoid-RS to `Cargo.toml`:

```toml
[dependencies]
humanoid-rs = { path = "../humanoid-rs", features = ["simulator"] }
tokio = { version = "1", features = ["full"] }
```

Write `src/main.rs`:

```rust
//! My First Robot Program
//! 
//! This example demonstrates the basic Humanoid-RS SDK usage.

use humanoid_rs::sdk::prelude::*;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🤖 My First Robot Program");
    println!("==========================\n");

    // Step 1: Create SDK instance
    let mut sdk = HumanoidSDK::new();
    println!("✓ SDK initialized");

    // Step 2: Connect to simulator
    let sim_config = SimulationConfig {
        timestep: Duration::from_millis(2),  // 500 Hz physics
        real_time_factor: 1.0,
        headless: false,                      // Show 3D window
        scene_path: None,
    };
    
    let sim = sdk.connect_simulator(sim_config)?;
    println!("✓ Connected to simulator (ID: {})", sim.id);

    // Step 3: Connect to robot
    let robot = sdk.connect_robot(RobotProfile::unitree_g1())?;
    println!("✓ Connected to {} robot", robot.profile.name);
    println!("  - {} joints", robot.profile.num_joints);
    println!("  - Height: {:.2}m", robot.profile.height);
    println!("  - Mass: {:.1}kg", robot.profile.mass);

    // Step 4: Command robot
    println!("\n▶ Commanding robot to stand up...");
    robot.stand_up()?;

    // Simulate 5 seconds
    println!("▶ Running for 5 seconds...");
    tokio::time::sleep(Duration::from_secs(5)).await;

    // Step 5: Cleanup
    println!("\n✓ Shutting down...");
    sdk.shutdown();
    
    println!("\n🎉 Success! Robot simulation complete.");
    Ok(())
}
```

Run it:

```bash
cargo run
```

## What Just Happened?

1. **SDK Initialization** - Created a `HumanoidSDK` instance that manages all connections
2. **Simulator Connection** - Connected to the Bevy + Rapier physics simulator
3. **Robot Connection** - Loaded the Unitree G1 robot profile with 29 joints
4. **Command Execution** - Sent stand-up command to the robot
5. **Cleanup** - Properly shut down all connections

## Key Concepts

### Robot Profile

```rust
// Pre-defined profiles
let g1 = RobotProfile::unitree_g1();
let h1 = RobotProfile::unitree_h1();

// Custom profile from URDF
let custom = RobotProfile::from_urdf("path/to/robot.urdf")?;
```

### Configuration

```rust
// Load from file
let sdk = HumanoidSDK::from_file("config.toml")?;

// Or build programmatically
let config = SDKConfig {
    physics: PhysicsConfig {
        timestep: Duration::from_millis(2),
        gravity: Vector3::new(0.0, -9.81, 0.0),
    },
    robot: RobotConfig::default(),
    control: ControlConfig::default(),
    logging: LoggingConfig::default(),
};
```

### Safety Features

```rust
use humanoid_rs::control::safety::{SafetyManager, JointLimitGuard};

// Enable safety guards
let safety = SafetyManager::new()
    .add_guard(JointLimitGuard::new(&joint_limits))
    .add_guard(EStopGuard::new());

// Emergency stop
if emergency {
    safety.trigger_estop("User requested stop")?;
}
```

## Next Steps

- **Examples**: Check `examples/` directory for more code
- **Documentation**: Read the full API docs at `docs/api_reference.md`
- **Tutorials**: Follow the tutorial series starting at `docs/tutorials/01_getting_started.md`

## Common Issues

### Simulator window doesn't open

```bash
# Try software rendering
export WGPU_BACKEND=gl  # Linux/macOS
set WGPU_BACKEND=dx12   # Windows

# Or run headless
cargo run --example basic_sim
```

### Build fails on Linux

```bash
sudo apt-get install build-essential pkg-config libssl-dev
```

## Project Structure

```
my_first_robot/
├── Cargo.toml           # Dependencies
├── src/
│   └── main.rs         # Your code
├── config.toml         # Robot configuration
└── README.md          # Your documentation
```

---

**Ready to build?** Continue with [Tutorial 1: Getting Started](docs/tutorials/01_getting_started.md)
