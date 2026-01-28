# Humanoid-RS 🦾

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

### Simulator (No Hardware Required)

```bash
# Clone repo
git clone https://github.com/yourusername/humanoid-rs
cd humanoid-rs

# Run simulator
cargo run --bin humanoid-sim --features simulator
```

This launches a 3D visualization of a Unitree G1 robot using Bevy + Rapier physics.

#### Simulator Controls

- **Space** - Stand up
- **C** - Crouch
- **1** - Wave animation
- **2** - T-pose
- **3** - Lean forward
- **4** - Sit pose
- **5** - Jump pose
- **W** - Toggle walking mode
- **Q/E** - Rotate camera
- **ESC** - Exit

### Connect to Real Robot (Unitree G1)

```bash
# Connect to robot on local network
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


