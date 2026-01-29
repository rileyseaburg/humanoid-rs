# Humanoid-RS Architecture

This document describes the system architecture, data flow, and component interactions of the Humanoid-RS framework.

## Overview

Humanoid-RS is a memory-safe, real-time robotics framework for humanoid robots built in Rust. It provides abstractions for robot control, motion planning, simulation, and hardware integration.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Application Layer                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  LLM Agent   │  │  Motion Plan │  │  Teleop      │              │
│  │  (MCP)       │  │  Generator   │  │  (Vision Pro)│              │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘              │
└─────────┼─────────────────┼─────────────────┼──────────────────────┘
          │                 │                 │
          └─────────────────┼─────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────────┐
│                    Humanoid-RS Framework                            │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                     Robot API Layer                           │  │
│  │  • HumanoidRobot trait (hardware abstraction)                │  │
│  │  • RobotState (position, velocity, sensors)                  │  │
│  │  • Joint control (position/velocity/effort)                  │  │
│  └──────────────────────┬───────────────────────────────────────┘  │
│                         │                                           │
│  ┌──────────────────────▼───────────────────────────────────────┐  │
│  │                   Control Layer                               │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │  │
│  │  │   Motion     │  │   Joint      │  │   Whole-Body │       │  │
│  │  │   Planner    │  │   Controller │  │   Controller │       │  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘       │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │  │
│  │  │     MPC      │  │      IK      │  │    Balance   │       │  │
│  │  │  Controller  │  │   Solver     │  │   Recovery   │       │  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘       │  │
│  └──────────────────────┬───────────────────────────────────────┘  │
│                         │                                           │
│  ┌──────────────────────▼───────────────────────────────────────┐  │
│  │                   Hardware Layer                              │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │  │
│  │  │   Unitree    │  │    ROS2      │  │  Simulator   │       │  │
│  │  │   Robot      │  │    Bridge    │  │  (Bevy)      │       │  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘       │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## Data Flow

### Real-Time Control Loop

```
Time ───────────────────────────────────────────────────────────────►

┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│ Physics │───►│ Sensors │───►│ Control │───►│Actuation│
│  Step   │    │  Read   │    │ Update  │    │ Output  │
└─────────┘    └─────────┘    └─────────┘    └─────────┘
     │              │              │              │
     ▼              ▼              ▼              ▼
 1 kHz           100-500 Hz     500-1000 Hz    500-1000 Hz
 (sim/real)      (IMU/encoders) (control)      (motors)
```

### Timing Requirements

| Component | Frequency | Latency | Jitter |
|-----------|-----------|---------|--------|
| Physics Simulation | 1000 Hz | < 1 ms | < 0.1 ms |
| IMU Reading | 200-1000 Hz | < 2 ms | < 0.5 ms |
| Joint Encoders | 100-500 Hz | < 5 ms | < 1 ms |
| Control Loop | 500-1000 Hz | < 2 ms | < 0.5 ms |
| Motor Commands | 500-1000 Hz | < 1 ms | < 0.5 ms |
| Safety Monitor | 1000 Hz | < 1 ms | < 0.1 ms |

## Component Interactions

### 1. Robot API Layer

The `HumanoidRobot` trait provides the primary abstraction:

```rust
pub trait HumanoidRobot {
    fn state(&self) -> &RobotState;
    fn set_joint_positions(&mut self, positions: &[f64]) -> Result<()>;
    fn set_joint_velocities(&mut self, velocities: &[f64]) -> Result<()>;
    fn set_joint_efforts(&mut self, efforts: &[f64]) -> Result<()>;
    fn forward_kinematics(&self, link_name: &str) -> Result<Pose>;
    fn emergency_stop(&mut self) -> Result<()>;
}
```

**Implementations:**
- `UnitreeRobot` - Real hardware via UDP
- `SimulatedRobot` - Bevy + Rapier physics
- `Ros2Robot` - ROS2-based robot proxy

### 2. Control Layer

#### Motion Planner
- Generates smooth trajectories between poses
- Respects joint limits and velocity constraints
- Supports joint-space and Cartesian planning

#### Joint Space Controller
- PD control with feedforward
- Trajectory following
- Gravity compensation

#### MPC Controller
- Model Predictive Control for locomotion
- Uses Linear Inverted Pendulum Model (LIPM)
- Predictive footstep placement

#### Whole-Body Controller
- CoM tracking
- Force distribution
- Balance maintenance
- Contact planning

#### IK Solver
- Analytical solutions for simple chains
- CCD (Cyclic Coordinate Descent) for complex chains
- Position and orientation targets

### 3. Hardware Layer

#### Unitree Integration
- UDP communication protocol
- High-level commands (stand, walk, stop)
- Low-level joint control
- State streaming at 100-500 Hz

#### ROS2 Bridge
- Topic publishing (joint_states, pose, TF)
- Command subscription (cmd_vel, trajectory)
- Action servers (FollowJointTrajectory)
- Clock synchronization

#### Simulator
- Bevy for rendering
- Rapier3D for physics
- Real-time 3D visualization
- Interactive controls

## Threading Model

```
Main Thread:
  └── Application logic
      └── Control loop (spawns workers)

Control Thread (Real-time priority):
  └── 500-1000 Hz control loop
      ├── Read sensors
      ├── Update controller
      ├── Safety checks
      └── Send commands

Sensor Thread:
  └── 100-500 Hz sensor polling
      ├── Poll hardware
      └── Update shared state (lock-free)

Safety Thread (Highest priority):
  └── 1000 Hz safety monitor
      ├── Check limits
      ├── Monitor timing
      └── Emergency stop if needed

ROS2 Thread (Optional):
  └── Publish/subscribe
      ├── Publish state at 100 Hz
      └── Process callbacks
```

### Thread Safety

- **Control Loop**: Runs on dedicated thread with real-time priority
- **State Sharing**: Uses lock-free data structures where possible
- **Command Queue**: Lock-free SPSC (Single Producer Single Consumer) queue
- **Safety**: Atomic flags for emergency stop
- **ROS2**: Separate thread for ROS2 spinning

## Module Dependencies

```
lib.rs
├── robot
│   ├── Joint, JointSet
│   ├── RobotState, Pose
│   └── HumanoidRobot trait
├── control
│   ├── Trajectory, MotionPlanner
│   ├── Controller trait
│   └── JointSpaceController
├── mpc
│   ├── MpcController
│   ├── FootstepPlanner
│   └── LipmState
├── whole_body
│   ├── WholeBodyController
│   ├── BalanceRecovery
│   └── SupportPolygon
├── ik
│   ├── IkSolver
│   └── WholeBodyIk
├── simulator (optional)
│   ├── SimulatedRobot
│   └── HumanoidSimulatorPlugin
├── unitree (optional)
│   └── UnitreeRobot
└── ros2 (optional)
    ├── Ros2Bridge
    └── FollowJointTrajectoryAction
```

## Communication Protocols

### Unitree Protocol

```
┌─────────────────────────────────────┐
│  Magic (4 bytes): "UNIT"            │
├─────────────────────────────────────┤
│  Command ID (1 byte)                │
│  Data Length (1 byte)               │
│  Data (variable)                    │
│  Checksum (1 byte)                  │
└─────────────────────────────────────┘

Command IDs:
  0x01 - Stand Up
  0x02 - Sit Down
  0x03 - Walk (vx, vy, vyaw)
  0x10 - Joint Positions
  0x11 - Joint Velocities
  0x12 - Joint Torques
  0xFF - Emergency Stop
```

### ROS2 Topics

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| /joint_states | sensor_msgs/JointState | Publish | 100 Hz |
| /pose | geometry_msgs/Pose | Publish | 50 Hz |
| /tf | tf2_msgs/TFMessage | Publish | 100 Hz |
| /cmd_vel | geometry_msgs/Twist | Subscribe | - |
| /joint_trajectory | trajectory_msgs/JointTrajectory | Subscribe | - |

## Error Handling

### Error Types

```rust
pub enum HumanoidError {
    JointLimitExceeded { ... },
    CommunicationError(String),
    SimulatorError(String),
    ControlError(String),
    SafetyStop { reason: String },
}
```

### Error Handling Strategy

1. **Hardware Errors**: Retry with exponential backoff, then fail
2. **Safety Errors**: Immediate emergency stop, no retry
3. **Control Errors**: Log and continue with degraded performance
4. **Communication Errors**: Buffer and retry, alert if persistent

## Memory Management

### Allocation Strategy

- **Pre-allocation**: All control loop memory pre-allocated
- **No Heap in Hot Path**: Stack-only allocations in 1kHz loop
- **Object Pools**: Reusable trajectory and state objects
- **Arena Allocators**: Per-frame allocations grouped

### Memory Layout

```
Static Memory:
  ├── Robot configuration (joint limits, kinematics)
  └── Safety parameters

Pre-allocated:
  ├── Control buffers (positions, velocities, torques)
  ├── Trajectory waypoints
  └── Sensor history (circular buffer)

Per-frame (stack):
  ├── Temporary calculations
  └── Control outputs
```

## Performance Considerations

### Optimization Strategies

1. **SIMD**: Use packed SIMD for matrix operations
2. **Cache Friendly**: Structure of Arrays for joint data
3. **Branch Prediction**: Minimize branches in hot loops
4. **Inlining**: Force inline small hot functions
5. **Lock-free**: Avoid mutexes in control loop

### Benchmarking

```rust
// Control loop timing
criterion_group!(benches, bench_control_loop);
fn bench_control_loop(c: &mut Criterion) {
    c.bench_function("control_update", |b| {
        b.iter(|| controller.update(&state, dt))
    });
}
```

## Future Architecture

### Planned Improvements

1. **Distributed Control**: Multi-process control for safety
2. **Hardware Abstraction**: Support for more robot platforms
3. **Learning Integration**: RL training environment
4. **Cloud Integration**: Remote monitoring and control
5. **Safety Certification**: ISO 10218 compliance

### Scalability

- Support for multiple robots (swarm control)
- Distributed computing for complex planners
- Cloud-based trajectory optimization
