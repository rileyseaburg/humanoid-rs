# Humanoid-RS 🦾

[![CI](https://github.com/rileyseaburg/humanoid-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/rileyseaburg/humanoid-rs/actions/workflows/ci.yml)
[![Crates.io](https://img.shields.io/crates/v/humanoid-rs)](https://crates.io/crates/humanoid-rs)
[![Documentation](https://docs.rs/humanoid-rs/badge.svg)](https://docs.rs/humanoid-rs)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE)
[![Rust Version](https://img.shields.io/badge/rustc-1.75+-orange.svg)](https://blog.rust-lang.org/2023/12/28/Rust-1.75.0.html)

A memory-safe, real-time robotics framework in Rust for humanoid robots.

**Status:** Early development - Simulator working, Unitree integration in progress

## Why Rust for Robotics?

- **Memory safety** - No segfaults mid-operation
- **Zero-cost abstractions** - Fast as C++, safer than Python
- **Real-time guarantees** - No GC pauses
- **Fearless concurrency** - Parallel control loops without data races

## Target Robots

- [Unitree G1](https://www.unitree.com/products/g1) ($16k-$64k)
- [Unitree H1](https://www.unitree.com/products/h1) ($90k-$116k)
- Any humanoid with compatible API

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Application                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  LLM Agent   │  │  Motion Plan │  │  Teleop      │      │
│  │  (MCP)       │  │  Generator   │  │  (Vision Pro)│      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
└─────────┼─────────────────┼─────────────────┼───────────────┘
          │                 │                 │
          └─────────────────┼─────────────────┘
                            │
┌───────────────────────────▼───────────────────────────────┐
│              Humanoid-RS Framework                        │
│  ┌──────────────────┐  ┌──────────────────────────────┐  │
│  │  Robot API       │  │  Motion Planning             │  │
│  │  - Joint control │  │  - Trajectory generation     │  │
│  │  - State queries │  │  - IK/FK                     │  │
│  │  - Safety limits │  │  - Collision avoidance       │  │
│  └────────┬─────────┘  └──────────────┬───────────────┘  │
│           │                           │                   │
│  ┌────────▼─────────┐  ┌──────────────▼───────────────┐  │
│  │  Controllers     │  │  Simulators                  │  │
│  │  - Joint space   │  │  - Bevy + Rapier (3D)        │  │
│  │  - Cartesian     │  │  - MuJoCo (physics-accurate) │  │
│  │  - Whole-body    │  │  - Isaac Gym (GPU parallel)  │  │
│  └────────┬─────────┘  └──────────────┬───────────────┘  │
└───────────┼───────────────────────────┼───────────────────┘
            │                           │
    ┌───────▼───────┐           ┌───────▼───────┐
    │  Unitree SDK  │           │  Real Robot   │
    │  (C++ Bridge) │           │  (UDP/ROS2)   │
    └───────────────┘           └───────────────┘
```

## Quick Start

### 🚀 5-Minute Quick Start

```bash
# 1. Clone and build
git clone https://github.com/rileyseaburg/humanoid-rs
cd humanoid-rs
cargo build --release --features simulator

# 2. Run the starter example
cargo run --example starter --features simulator

# 3. Launch interactive simulator
cargo run --bin humanoid-sim --features simulator
```

### Create Your First Project

```bash
# Create new project
cargo new my_robot
cd my_robot

# Add dependency (edit Cargo.toml)
echo '[dependencies]
humanoid-rs = { path = "../humanoid-rs", features = ["simulator"] }
tokio = { version = "1", features = ["full"] }' >> Cargo.toml

# Write starter code
cat > src/main.rs << 'EOF'
use humanoid_rs::sdk::prelude::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut sdk = HumanoidSDK::new();
    let _sim = sdk.connect_simulator(SimulationConfig::default())?;
    let robot = sdk.connect_robot(RobotProfile::unitree_g1())?;
    
    robot.stand_up()?;
    tokio::time::sleep(std::time::Duration::from_secs(5)).await;
    
    sdk.shutdown();
    Ok(())
}
EOF

# Run it
cargo run
```

📖 **[Complete Quick Start Guide](QUICKSTART.md)** - Full walkthrough with explanations

### Simulator (No Hardware Required)

Launch the 3D visualization:

```bash
cargo run --bin humanoid-sim --features simulator
```

**Controls:**
- **Space** - Stand up
- **C** - Crouch
- **1-5** - Pose presets
- **W** - Walking mode
- **Arrows** - Camera
- **ESC** - Exit

### Connect to Real Robot (Unitree G1)

```bash
cargo run --bin unitree-bridge --features unitree -- 192.168.123.104:8080
```

## Usage Examples

### Basic Joint Control

```rust
use humanoid_rs::robot::{HumanoidRobot, unitree_g1};
use humanoid_rs::unitree::UnitreeRobot;

#[tokio::main]
async fn main() -> Result<()> {
    // Connect to robot
    let mut robot = UnitreeRobot::connect("192.168.123.104:8080").await?;
    
    // Stand up
    let standing_pose = vec![
        // Left leg
        0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
        // Right leg  
        0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
        // Waist
        0.0, 0.0, 0.0,
        // Arms
        0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];
    
    robot.set_joint_positions(&standing_pose)?;
    
    Ok(())
}
```

### Motion Planning

```rust
use humanoid_rs::control::{MotionPlanner, Trajectory};

let planner = MotionPlanner::new(2.0, 5.0); // max_vel, max_accel

// Plan standing motion
let current = robot.state().joints.clone();
let trajectory = planner.plan_stand_up(&current);

// Execute
controller.set_trajectory(trajectory);
```

### LLM Integration (MCP)

```rust
// Robot controlled via Claude/LLM
let mcp_tools = vec![
    MCPTool {
        name: "robot_stand".to_string(),
        handler: Box::new(|_| robot.set_joint_positions(&standing_pose)),
    },
    MCPTool {
        name: "robot_walk".to_string(), 
        handler: Box::new(|args| robot.walk(args.get("direction").unwrap())),
    },
];
```

## Project Structure

```
humanoid/
├── Cargo.toml              # Workspace manifest
├── src/
│   ├── lib.rs              # Core framework
│   ├── robot.rs            # Robot abstraction, kinematics
│   ├── control.rs          # Controllers, planners
│   ├── simulator.rs        # Bevy + Rapier simulation
│   ├── unitree.rs          # Unitree SDK integration
│   └── bin/
│       ├── simulator.rs    # Run 3D sim
│       └── unitree_bridge.rs # Connect to real robot
├── examples/               # Usage examples
└── docs/                   # Documentation
```

## Features

- [x] Core robot abstraction
- [x] Joint-space control
- [x] Motion planning (trajectory generation)
- [x] 3D simulator (Bevy + Rapier)
- [x] Inverse kinematics (IK) solver
- [x] Walking gait controller
- [x] Multiple preset animations
- [x] Unitree SDK integration
- [x] ROS2 bridge
- [x] Whole-body control
- [x] MPC (Model Predictive Control)
- [x] RL training environment
- [x] LLM integration (MCP)

## Why This vs Unitree's C++ SDK?

| Feature | Unitree C++ | Humanoid-RS |
|---------|-------------|-------------|
| Language | C++ | Rust |
| Memory safety | ❌ Manual | ✅ Compiler-checked |
| Concurrency | ❌ Risky | ✅ Fearless |
| Simulator | ❌ MuJoCo only | ✅ Multiple backends |
| LLM integration | ❌ Manual | ✅ MCP-native |
| Open source | ✅ Yes | ✅ Yes (MIT/Apache) |

## Roadmap

### Phase 1: Core (Complete)
- [x] Robot abstraction
- [x] Basic simulator
- [x] Joint control

### Phase 2: Real Hardware (Complete)
- [x] Unitree SDK bridge
- [x] ROS2 integration
- [ ] Sensor fusion

### Phase 3: Intelligence (Complete)
- [x] MPC controller
- [x] RL training env
- [x] LLM task planning (MCP)

### Phase 4: Production
- [ ] Safety certification
- [ ] Industry partnerships
- [ ] Commercial support

## Contributing

This is early-stage. Contributions welcome!

1. Fork the repo
2. Create feature branch
3. Write tests
4. Submit PR

## License

MIT OR Apache-2.0 - Dual licensed for maximum compatibility

## Acknowledgments

- Unitree Robotics for open-sourcing their SDK
- Bevy Engine for 3D visualization
- Rapier for physics simulation
- The Rust robotics community

---

**Built with 🦀 by robotics engineers for robotics engineers**


