# ROS2 Integration for Humanoid-rs

This module provides full ROS2 integration for the humanoid robotics framework.

## Prerequisites

1. Install ROS2 Humble or later:
   ```bash
   # Follow official ROS2 installation guide
   # https://docs.ros.org/en/humble/Installation.html
   ```

2. Source ROS2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Install additional ROS2 packages:
   ```bash
   sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs
   ```

## Building

Build with ROS2 support:
```bash
cargo build --features ros2
```

## Usage

### Basic Example

```rust
#[cfg(feature = "ros2")]
{
    use humanoid::ros2::{HumanoidRosNode, ClockMode};
    use std::time::Duration;

    // Create ROS node
    let ros_node = HumanoidRosNode::new("humanoid_sim", ClockMode::Simulation)?;
    
    // In simulation loop
    loop {
        // Publish robot state
        ros_node.publish_state(&robot_state)?;
        
        // Process callbacks
        ros_node.spin(Duration::from_millis(1))?;
        
        // Check emergency stop
        if ros_node.is_emergency_stop_active() {
            break;
        }
    }
    
    // Shutdown
    ros_node.shutdown();
}
```

### Clock Modes

- **WallTime**: Use system clock (default for real robot)
- **Simulation**: Use simulation time from /clock topic
- **External**: Use external time source with offset

```rust
// Simulation mode (synchronized with physics sim)
let mut node = HumanoidRosNode::new("humanoid", ClockMode::Simulation)?;
node.update_simulation_time(simulation_time);

// External mode
let mut node = HumanoidRosNode::new("humanoid", ClockMode::External)?;
node.clock_mut().set_offset(Duration::from_millis(100));
```

### Timing Policy

Configure publication rates:

```rust
use humanoid::ros2::TimingPolicy;

let mut policy = TimingPolicy::default();
policy.joint_state_rate = 100.0;  // 100 Hz
policy.imu_rate = 200.0;          // 200 Hz
policy.pose_rate = 50.0;          // 50 Hz
policy.real_time_factor = 1.0;    // Real-time

ros_node.set_timing_policy(policy);
```

## Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | 100 Hz | Joint positions, velocities, efforts |
| `/imu/data` | sensor_msgs/Imu | 200 Hz | IMU orientation and acceleration |
| `/base_pose` | geometry_msgs/PoseStamped | 100 Hz | Base link pose |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_commands` | std_msgs/Float64MultiArray | Joint position commands |
| `/emergency_stop` | std_msgs/Bool | Emergency stop signal |

## Launch Files

### Python Launch

```bash
ros2 launch humanoid_rs humanoid.launch.py
```

### XML Launch

```bash
ros2 launch humanoid_rs humanoid.xml
```

## Parameters

The node accepts the following parameters:

- `clock_mode`: WallTime, Simulation, or External
- `joint_state_rate`: Publication rate for joint states (Hz)
- `imu_rate`: Publication rate for IMU (Hz)
- `pose_rate`: Publication rate for pose (Hz)
- `real_time_factor`: Real-time factor for simulation

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    HumanoidRosNode                          │
├─────────────────────────────────────────────────────────────┤
│  Clock      │  WallTime │ Simulation │ External             │
├─────────────┼───────────┼────────────┼──────────────────────┤
│ Publishers  │ JointState│ IMU        │ PoseStamped          │
├─────────────┼───────────┼────────────┼──────────────────────┤
│ Subscribers │ Commands  │ EmergencyStop                     │
├─────────────┼───────────┼────────────────────────────────────┤
│  Timing     │ Configurable publication rates per topic     │
└─────────────────────────────────────────────────────────────┘
```

## Error Handling

The ROS2 node handles errors gracefully:

- Context initialization failures
- Node creation failures
- Publisher/Subscription creation failures
- Message publication failures

All errors are converted to `HumanoidError::CommunicationError` with descriptive messages.

## Testing

Run tests:
```bash
cargo test --features ros2
```

## Best Practices

1. **Always check emergency stop**: Check `is_emergency_stop_active()` in control loop
2. **Use appropriate clock mode**: Simulation for sim, WallTime for real robot
3. **Configure rates**: Match publication rates to your control frequency
4. **Handle errors**: Always handle Result from publish methods
5. **Shutdown gracefully**: Call `shutdown()` before exiting
