# ROS2 Integration

This tutorial covers integrating Humanoid-RS with ROS2 (Robot Operating System 2).

## Overview

ROS2 provides:
- **Communication** - Topics, services, and actions
- **Tools** - rviz2, rqt, rosbag2
- **Ecosystem** - Navigation, manipulation, perception packages
- **Integration** - Connect with other robots and systems

## Prerequisites

### Install ROS2

**Ubuntu 22.04 (Humble Hawksbill):**

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base
sudo apt install ros-dev-tools

# Source ROS2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**Other platforms:** See [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### Verify Installation

```bash
# Check ROS2 version
ros2 --version

# Test installation
ros2 run demo_nodes_cpp talker  # Terminal 1
ros2 run demo_nodes_cpp listener  # Terminal 2
```

## Basic ROS2 Bridge

### Creating a Bridge Node

```rust
use humanoid_rs::ros2::{Ros2Bridge, QosProfile};
use humanoid_rs::robot::unitree_g1;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create ROS2 bridge
    let mut bridge = Ros2Bridge::new("humanoid_robot", "/humanoid");
    
    // Initialize the node
    bridge.init()?;
    
    println!("ROS2 bridge initialized!");
    println!("Node name: humanoid_robot");
    println!("Namespace: /humanoid");
    
    // Main loop
    loop {
        // Create robot state (in real usage, get from hardware)
        let state = create_robot_state();
        
        // Publish state to ROS2
        bridge.publish_state(&state)?;
        
        // Process ROS2 callbacks
        bridge.spin_once(Duration::from_millis(10))?;
        
        std::thread::sleep(Duration::from_millis(10));
    }
}

fn create_robot_state() -> RobotState {
    RobotState {
        joints: unitree_g1::create_default_joints(),
        ..Default::default()
    }
}
```

### Published Topics

The bridge automatically publishes:

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/humanoid/joint_states` | sensor_msgs/JointState | 100 Hz | Joint positions, velocities, efforts |
| `/humanoid/pose` | geometry_msgs/Pose | 50 Hz | Base position and orientation |
| `/humanoid/tf` | tf2_msgs/TFMessage | 100 Hz | Transform tree |

```bash
# View published topics
ros2 topic list

# Echo joint states
ros2 topic echo /humanoid/joint_states

# Check topic rate
ros2 topic hz /humanoid/joint_states
```

### Subscribed Topics

The bridge subscribes to:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/humanoid/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/humanoid/joint_trajectory` | trajectory_msgs/JointTrajectory | Joint trajectory |

```rust
// Get latest velocity command
let cmd_vel = bridge.get_cmd_vel();
println!("Received cmd_vel: linear=[{:.2}, {:.2}, {:.2}]",
    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);

// Get pending trajectory
if let Some(trajectory) = bridge.take_trajectory() {
    println!("Received trajectory with {} points", 
        trajectory.points.len());
    execute_trajectory(trajectory)?;
}
```

## Visualizing with RViz2

### Launch RViz2

```bash
# Terminal 1: Run the bridge
cargo run --example ros2_bridge --features ros2

# Terminal 2: Launch RViz2
ros2 run rviz2 rviz2
```

### RViz2 Configuration

1. **Add displays:**
   - **RobotModel**: Visualize the robot
     - Topic: `/humanoid/robot_description`
     - Fixed Frame: `base_link`
   
   - **TF**: Show coordinate frames
     - Check all frames
   
   - **Pose**: Show base pose
     - Topic: `/humanoid/pose`

2. **Set fixed frame:**
   - Fixed Frame: `world` or `base_link`

3. **Save configuration:**
   - File → Save Config As → `humanoid_config.rviz`

### Robot Description (URDF)

Create a URDF file for visualization:

```xml
<!-- humanoid.urdf -->
<?xml version="1.0"?>
<robot name="humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="body">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25"/>
  </joint>
  
  <!-- Add more links and joints for full robot -->
</robot>
```

Publish URDF:

```bash
ros2 topic pub /humanoid/robot_description std_msgs/msg/String \
  "data: $(cat humanoid.urdf | xargs)" --once
```

## ROS2 Actions

### FollowJointTrajectory Action

```rust
use humanoid_rs::ros2::FollowJointTrajectoryAction;

// Create action server
let action = FollowJointTrajectoryAction::new("/humanoid/follow_joint_trajectory");

// Accept a goal
let trajectory = create_trajectory();
action.accept_goal(trajectory)?;

// In control loop
if action.is_executing() {
    let elapsed = Duration::from_secs(1);  // Time since goal accepted
    
    if let Some(setpoint) = action.get_current_setpoint(elapsed) {
        // Send setpoint to robot
        robot.set_joint_positions(&setpoint.positions)?;
    } else {
        // Trajectory complete
        action.set_succeeded();
    }
}
```

### Sending Goals from Command Line

```bash
# Send trajectory goal
ros2 action send_goal /humanoid/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: ['joint1', 'joint2'], 
    points: [{positions: [0.5, 0.5], time_from_start: {sec: 2}}]}"
```

## Recording and Playback

### Recording with rosbag2

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /humanoid/joint_states /humanoid/pose /humanoid/cmd_vel

# Record with compression
ros2 bag record -a -s mcap --compression-mode file --compression-format zstd
```

### Playing Back

```bash
# List recordings
ros2 bag list

# Play recording
ros2 bag play recording_2024-01-15/

# Play at half speed
ros2 bag play recording_2024-01-15/ -r 0.5

# Play in loop
ros2 bag play recording_2024-01-15/ -l
```

### Analyzing Recordings

```bash
# Show recording info
ros2 bag info recording_2024-01-15/

# Convert to other formats
ros2 bag convert recording_2024-01-15/ -o output.mcap
```

## ROS2 Services

### Creating a Service

```rust
use humanoid_rs::ros2::ServiceServer;

// Create service for emergency stop
let mut estop_service = ServiceServer::<Empty, Empty>::new("/humanoid/emergency_stop");
estop_service.set_handler(|_req| {
    robot.emergency_stop()?;
    Ok(Empty {})
});

// Create service for getting robot info
let mut info_service = ServiceServer::<Empty, RobotInfo>::new("/humanoid/get_info");
info_service.set_handler(|_req| {
    Ok(RobotInfo {
        name: "Unitree G1".to_string(),
        num_joints: 29,
        is_connected: robot.is_connected(),
    })
});
```

### Calling Services

```bash
# Call emergency stop
ros2 service call /humanoid/emergency_stop std_srvs/srv/Empty

# Get robot info
ros2 service call /humanoid/get_info humanoid_msgs/srv/GetRobotInfo

# List services
ros2 service list

# Show service type
ros2 service type /humanoid/emergency_stop
```

## Integration Examples

### Navigation with Nav2

```bash
# Terminal 1: Launch robot bridge
cargo run --example ros2_bridge --features ros2

# Terminal 2: Launch Nav2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}}}'
```

### Motion Planning with MoveIt

```bash
# Terminal 1: Launch robot
ros2 launch humanoid_bringup robot.launch.py

# Terminal 2: Launch MoveIt
ros2 launch humanoid_moveit_config moveit.launch.py

# Terminal 3: Run RViz2 with MotionPlanning plugin
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix humanoid_moveit_config)/share/humanoid_moveit_config/config/humanoid.rviz
```

### Multi-Robot Setup

```rust
// Robot 1
let bridge1 = Ros2Bridge::new("robot_1", "/robot_1");

// Robot 2
let bridge2 = Ros2Bridge::new("robot_2", "/robot_2");

// Each robot has its own namespace:
// /robot_1/joint_states
// /robot_2/joint_states
```

## QoS (Quality of Service)

### QoS Profiles

```rust
use humanoid_rs::ros2::QosProfile;

// Sensor data (best effort, volatile)
let imu_qos = QosProfile::SensorData;

// Parameters (reliable, persistent)
let param_qos = QosProfile::Parameters;

// Services (reliable, volatile)
let service_qos = QosProfile::Services;

// Default (best effort, keep last)
let default_qos = QosProfile::Default;
```

### Custom QoS

```rust
// High-frequency, low-latency sensor data
let high_freq_qos = QosProfile::SensorData;

// Critical commands (reliable delivery)
let command_qos = QosProfile::Services;
```

## Launch Files

### Python Launch File

```python
# humanoid.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Humanoid bridge
        Node(
            package='humanoid_ros2',
            executable='bridge',
            name='humanoid_bridge',
            namespace='humanoid',
            parameters=[{
                'publish_rate': 100.0,
                'cmd_timeout': 0.5,
            }],
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='humanoid',
            arguments=['humanoid.urdf'],
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'humanoid_config.rviz'],
        ),
    ])
```

### Running Launch Files

```bash
# Run launch file
ros2 launch humanoid_bringup humanoid.launch.py

# With parameters
ros2 launch humanoid_bringup humanoid.launch.py \
  publish_rate:=200.0 \
  use_sim:=true
```

## Parameter Server

### Setting Parameters

```bash
# Set parameter
ros2 param set /humanoid/bridge publish_rate 200.0

# List parameters
ros2 param list /humanoid/bridge

# Get parameter
ros2 param get /humanoid/bridge publish_rate

# Dump all parameters
ros2 param dump /humanoid/bridge > bridge_params.yaml
```

### Loading Parameters

```bash
# Load from file
ros2 param load /humanoid/bridge bridge_params.yaml

# In launch file
Node(
    package='humanoid_ros2',
    executable='bridge',
    parameters=['bridge_params.yaml'],
)
```

## Best Practices

1. **Use appropriate QoS** - Sensor data = best effort, commands = reliable
2. **Set namespace** - Avoid topic collisions in multi-robot setups
3. **Monitor topic rates** - Use `ros2 topic hz` to verify publishing
4. **Log to bag files** - Record sessions for debugging
5. **Use RViz2** - Visualize robot state and TF tree
6. **Implement actions** - For long-running operations like trajectories
7. **Handle latencies** - Account for ROS2 communication delays
8. **Document topics** - List all published/subscribed topics
9. **Use standard messages** - Prefer standard ROS2 message types
10. **Test with playback** - Use rosbag2 for reproducible testing

## Troubleshooting

### Bridge Won't Start

```bash
# Check ROS2 environment
printenv | grep ROS

# Source ROS2
source /opt/ros/humble/setup.bash

# Verify ROS2 is working
ros2 topic list
```

### Topics Not Visible

```bash
# Check node is running
ros2 node list

# Check topics
ros2 topic list

# Echo topic with verbose
ros2 topic echo /humanoid/joint_states --verbose
```

### RViz2 Shows Nothing

1. **Check fixed frame** - Set to `base_link` or `world`
2. **Add RobotModel** - Set topic to `/humanoid/robot_description`
3. **Check TF** - View TF tree: `ros2 run tf2_tools view_frames.py`
4. **Verify messages** - `ros2 topic hz /humanoid/joint_states`

### High Latency

1. **Check network** - Use wired connection, not WiFi
2. **Reduce QoS reliability** - Use `best_effort` for sensors
3. **Limit topic rates** - Don't publish faster than needed
4. **Enable compression** - Use zstd compression in rosbag2
5. **Check CPU load** - Use `htop` to monitor system resources

## Next Steps

- Review the [API Reference](../api_reference.md)
- Learn about [Control Algorithms](03_control_robots.md)
- Explore [Sensors and Observability](04_sensors_and_observability.md)
- Join the [ROS2 community](https://discourse.ros.org/)

## Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [RViz2 User Guide](https://github.com/ros2/rviz/blob/ros2/README.md)
- [Nav2 Documentation](https://navigation.ros.org/)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
