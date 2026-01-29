# Getting Started with Humanoid-RS

Welcome! This guide will get you from zero to controlling a simulated robot in 10 minutes.

## Prerequisites

- **Rust 1.75+**: [Install from rustup.rs](https://rustup.rs/)
- **Git**: For cloning the repo

## 1. Installation

```bash
# Clone the repository
git clone https://github.com/rileyseaburg/humanoid-rs
cd humanoid-rs

# Build everything (takes a few minutes on first run)
cargo build --release --features simulator
```

## 2. Run Your First Simulation

```bash
# Launch the 3D simulator
cargo run --bin humanoid-sim --features simulator
```

You should see a window with a Unitree G1 robot. Try these controls:
- **Space**: Stand up
- **C**: Crouch
- **1-5**: Different poses
- **Arrows**: Move camera
- **ESC**: Exit

## 3. Create Your First Project

```bash
# Create new Rust project
cargo new my_robot
cd my_robot
```

Edit `Cargo.toml`:

```toml
[package]
name = "my_robot"
version = "0.1.0"
edition = "2021"

[dependencies]
# Add Humanoid-RS with simulator feature
humanoid-rs = { path = "../humanoid-rs", features = ["simulator"] }

# Async runtime
tokio = { version = "1", features = ["full"] }
```

Edit `src/main.rs`:

```rust
//! My First Robot Program
use humanoid_rs::sdk::prelude::*;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Create SDK
    let mut sdk = HumanoidSDK::new();
    
    // 2. Connect to simulator
    let sim_config = SimulationConfig::default();
    let _sim = sdk.connect_simulator(sim_config)?;
    println!("✓ Connected to simulator");
    
    // 3. Connect to robot
    let robot = sdk.connect_robot(RobotProfile::unitree_g1())?;
    println!("✓ Connected to {}", robot.profile.name);
    
    // 4. Command robot
    robot.stand_up()?;
    println!("✓ Robot standing");
    
    // 5. Run for 5 seconds
    std::thread::sleep(Duration::from_secs(5));
    
    // 6. Cleanup
    sdk.shutdown();
    println!("✓ Done!");
    
    Ok(())
}
```

Run it:

```bash
cargo run
```

## 4. Understanding the SDK

### Robot Profiles

```rust
// Pre-defined profiles
let g1 = RobotProfile::unitree_g1();
let h1 = RobotProfile::unitree_h1();

// Custom from URDF
let custom = RobotProfile::from_urdf("robot.urdf")?;

// Check capabilities
println!("Joints: {}", profile.num_joints);
println!("Height: {:.2}m", profile.height);
```

### Configuration

```rust
// Method 1: Default config
let sdk = HumanoidSDK::new();

// Method 2: Custom config
let config = SDKConfig {
    physics: PhysicsConfig {
        timestep: Duration::from_millis(2),
        ..Default::default()
    },
    ..Default::default()
};
let sdk = HumanoidSDK::with_config(config);

// Method 3: From file
let sdk = HumanoidSDK::from_file("config.toml")?;
```

### Safety

```rust
use humanoid_rs::control::safety::*;

// Enable safety guards
let mut safety = SafetyManager::new(SafetyLevel::Simulation);
safety.add_guard(Box::new(JointLimitGuard::new(&limits)));
safety.add_guard(Box::new(EStopGuard::new()));

// Emergency stop
safety.trigger_estop("User requested")?;
```

## 5. Examples to Try

```bash
# Basic simulation
cargo run --example starter --features simulator

# Control loop demo
cargo run --example control_loop --features simulator

# Sensor data
cargo run --example sensor_streaming --features simulator

# Walking simulation
cargo run --example walking_demo --features simulator
```

## 6. Troubleshooting

### Build Errors

**"feature `simulator` requires bevy"**
```bash
# Install graphics drivers or use software rendering
export WGPU_BACKEND=gl  # Linux/macOS
set WGPU_BACKEND=dx12   # Windows
```

**"linking with `cc` failed" (Linux)**
```bash
sudo apt-get install build-essential pkg-config libssl-dev
```

### Runtime Issues

**Simulator window doesn't appear**
```bash
# Check graphics support
vulkaninfo

# Or run headless
cargo run --example basic_sim  # No --features simulator
```

**Robot doesn't move**
- Check that simulator is running
- Verify robot is connected: `robot.is_connected`
- Check for safety stops: `safety.is_estopped()`

## 7. Project Structure

```
my_robot/
├── Cargo.toml          # Dependencies
├── config.toml         # Robot configuration
└── src/
    └── main.rs        # Your code
```

## 8. Next Steps

1. **Read the Tutorials**
   - Tutorial 2: Running Simulations
   - Tutorial 3: Controlling Robots
   - Tutorial 4: Sensors and Observability

2. **Explore Examples**
   - Check `examples/` directory
   - Each example demonstrates different features

3. **API Documentation**
   - Run: `cargo doc --open`
   - Or read: `docs/api_reference.md`

4. **Build Something!**
   - Try different robot profiles
   - Experiment with configurations
   - Add custom control logic

## Quick Reference

### Common Imports

```rust
use humanoid_rs::sdk::prelude::*;        // SDK types
use humanoid_rs::control::safety::*;     // Safety
use humanoid_rs::robot::{Joint, Pose};   // Robot types
```

### Creating a Config File

Copy `config.example.toml` to `config.toml`:

```bash
cp ../humanoid-rs/config.example.toml config.toml
```

### Running Tests

```bash
cd ../humanoid-rs
cargo test
```

---

**That's it!** You're ready to build with Humanoid-RS. Check the other tutorials for deeper dives into specific features.

Need help? Check:
- [API Reference](api_reference.md)
- [Examples](../../examples/)
- [GitHub Issues](https://github.com/rileyseaburg/humanoid-rs/issues)
