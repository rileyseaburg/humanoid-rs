# Sensors and Observability

This tutorial covers how to access and use sensor data from humanoid robots.

> **Quick Start:** New to Humanoid-RS? See [QUICKSTART.md](../../QUICKSTART.md) or run `cargo run --example starter --features simulator`

## Overview

Humanoid robots typically have these sensors:

- **IMU** (Inertial Measurement Unit) - Orientation, angular velocity, acceleration
- **Joint Encoders** - Position, velocity, sometimes effort
- **Contact Sensors** - Foot contact detection
- **Force/Torque Sensors** - Contact forces (optional)
- **Cameras** - Visual perception (handled separately)

## Sensor Types

### IMU (Inertial Measurement Unit)

The IMU provides orientation and motion data:

```rust
use humanoid_rs::robot::RobotState;

fn read_imu(state: &RobotState) {
    // Orientation as quaternion [w, x, y, z]
    let orientation = state.base_pose.orientation;
    println!("Orientation: [{:.3}, {:.3}, {:.3}, {:.3}]",
        orientation.coords.w,
        orientation.coords.x,
        orientation.coords.y,
        orientation.coords.z);
    
    // Convert to Euler angles
    let (roll, pitch, yaw) = quaternion_to_euler(orientation);
    println!("Roll: {:.1}°, Pitch: {:.1}°, Yaw: {:.1}°",
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees());
}

fn quaternion_to_euler(q: UnitQuaternion<f64>) -> (f64, f64, f64) {
    let (roll, pitch, yaw) = q.euler_angles();
    (roll, pitch, yaw)
}
```

**Typical IMU Specifications:**
- Update rate: 200-1000 Hz
- Accelerometer range: ±16g
- Gyroscope range: ±2000°/s
- Orientation accuracy: ±1°

### Joint Encoders

Each joint has position and velocity feedback:

```rust
fn read_joint_states(state: &RobotState) {
    for joint in state.joints.iter() {
        println!("Joint: {}", joint.name);
        println!("  Position: {:.3} rad", joint.current_position);
        println!("  Velocity: {:.3} rad/s", joint.velocity);
        println!("  Effort: {:.3} Nm", joint.effort);
        println!("  Target: {:.3} rad", joint.target_position);
    }
}

// Get all positions as a vector
let positions = state.joints.positions();
println!("All positions: {:?}", positions);
```

**Typical Encoder Specifications:**
- Resolution: 12-19 bits
- Update rate: 100-500 Hz
- Accuracy: ±0.1°

### Contact Sensors

Foot contact detection for balance:

```rust
fn check_foot_contacts(state: &RobotState) {
    let [left_contact, right_contact] = state.foot_contacts;
    
    println!("Left foot: {}", 
        if left_contact { "CONTACT" } else { "AIR" });
    println!("Right foot: {}",
        if right_contact { "CONTACT" } else { "AIR" });
    
    // Determine support state
    match (left_contact, right_contact) {
        (true, true) => println!("Double support"),
        (true, false) => println!("Left single support"),
        (false, true) => println!("Right single support"),
        (false, false) => println!("Flying - both feet in air!"),
    }
}
```

## Streaming Sensor Data

### Polling Loop

```rust
use std::time::{Duration, Instant};

fn sensor_streaming_loop(robot: &mut impl HumanoidRobot) -> Result<()> {
    let sample_rate_hz = 100.0;
    let dt = Duration::from_secs_f64(1.0 / sample_rate_hz);
    
    let mut sensor_logger = SensorLogger::new();
    
    loop {
        let start = Instant::now();
        
        // Poll robot for state updates
        // (implementation depends on robot type)
        
        // Get current state
        let state = robot.state();
        
        // Log sensor data
        sensor_logger.log(&state);
        
        // Process sensors
        process_sensors(&state)?;
        
        // Maintain sample rate
        let elapsed = start.elapsed();
        if elapsed < dt {
            std::thread::sleep(dt - elapsed);
        }
    }
}
```

### Real Hardware (Unitree)

```rust
use humanoid_rs::unitree::UnitreeRobot;

#[tokio::main]
async fn main() -> Result<()> {
    let mut robot = UnitreeRobot::connect("192.168.123.104:8080").await?;
    
    loop {
        // Poll for state updates
        if robot.poll_state()? {
            // New state available
            let state = robot.state();
            
            // Access sensor data
            println!("IMU Orientation: {:?}", state.base_pose.orientation);
            println!("Joint positions: {} values", state.joints.positions().len());
            println!("Foot contacts: {:?}", state.foot_contacts);
        }
        
        std::thread::sleep(Duration::from_millis(2));
    }
}
```

## Sensor Fusion

### Complementary Filter

Combine IMU and encoder data:

```rust
struct SensorFusion {
    // Filter state
    alpha: f64,  // Complementary filter coefficient
    imu_weight: f64,
    encoder_weight: f64,
}

impl SensorFusion {
    fn new() -> Self {
        Self {
            alpha: 0.98,  // 98% IMU, 2% encoder integration
            imu_weight: 0.7,
            encoder_weight: 0.3,
        }
    }
    
    fn fuse_orientation(
        &self,
        imu_orientation: UnitQuaternion<f64>,
        encoder_based_orientation: UnitQuaternion<f64>,
        dt: Duration,
    ) -> UnitQuaternion<f64> {
        // Simple complementary filter
        let fused = imu_orientation 
            * self.imu_weight 
            + encoder_based_orientation 
            * self.encoder_weight;
        
        fused.normalize()
    }
}
```

### State Estimation

```rust
use nalgebra::{Vector3, Matrix3};

struct StateEstimator {
    // Kalman filter or similar
    com_position: Vector3<f64>,
    com_velocity: Vector3<f64>,
    
    // Covariance matrices
    p: Matrix3<f64>,  // State covariance
    q: Matrix3<f64>,  // Process noise
    r: Matrix3<f64>,  // Measurement noise
}

impl StateEstimator {
    fn predict(&mut self, dt: f64) {
        // Predict CoM state
        self.com_position += self.com_velocity * dt;
        
        // Update covariance
        self.p += self.q;
    }
    
    fn update(&mut self, measurement: Vector3<f64>) {
        // Kalman gain
        let s = self.p + self.r;
        let k = self.p * s.try_inverse().unwrap();
        
        // Update state
        let innovation = measurement - self.com_position;
        self.com_position += k * innovation;
        
        // Update covariance
        self.p = (Matrix3::identity() - k) * self.p;
    }
}
```

## Sensor Logging and Analysis

### Data Logging

```rust
use std::fs::File;
use std::io::Write;
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize)]
struct SensorLogEntry {
    timestamp: u64,  // milliseconds
    joint_positions: Vec<f64>,
    joint_velocities: Vec<f64>,
    orientation: [f64; 4],
    angular_velocity: [f64; 3],
    linear_acceleration: [f64; 3],
    foot_contacts: [bool; 2],
}

struct SensorLogger {
    file: File,
    buffer: Vec<SensorLogEntry>,
    buffer_size: usize,
}

impl SensorLogger {
    fn new(path: &str) -> Result<Self> {
        let file = File::create(path)?;
        Ok(Self {
            file,
            buffer: Vec::with_capacity(1000),
            buffer_size: 1000,
        })
    }
    
    fn log(&mut self, state: &RobotState) {
        let entry = SensorLogEntry {
            timestamp: state.timestamp.as_millis() as u64,
            joint_positions: state.joints.positions(),
            joint_velocities: state.joints.iter()
                .map(|j| j.velocity)
                .collect(),
            orientation: [
                state.base_pose.orientation.coords.w,
                state.base_pose.orientation.coords.x,
                state.base_pose.orientation.coords.y,
                state.base_pose.orientation.coords.z,
            ],
            angular_velocity: [0.0, 0.0, 0.0],  // Would come from IMU
            linear_acceleration: [0.0, -9.81, 0.0],  // Would come from IMU
            foot_contacts: state.foot_contacts,
        };
        
        self.buffer.push(entry);
        
        // Flush when buffer is full
        if self.buffer.len() >= self.buffer_size {
            self.flush()?;
        }
    }
    
    fn flush(&mut self) -> Result<()> {
        for entry in &self.buffer {
            let json = serde_json::to_string(entry)?;
            writeln!(self.file, "{}", json)?;
        }
        self.buffer.clear();
        Ok(())
    }
}
```

### Data Visualization

```rust
// Plot joint trajectories
fn plot_joint_trajectories(log: &[SensorLogEntry], joint_idx: usize) {
    use gnuplot::*;
    
    let times: Vec<f64> = log.iter()
        .map(|e| e.timestamp as f64 / 1000.0)
        .collect();
    
    let positions: Vec<f64> = log.iter()
        .map(|e| e.joint_positions[joint_idx])
        .collect();
    
    let mut fg = Figure::new();
    fg.axes2d()
        .lines(&times, &positions, &[Caption("Joint Position")])
        .set_x_label("Time (s)", &[])
        .set_y_label("Position (rad)", &[]);
    
    fg.show().unwrap();
}
```

## Sensor Monitoring

### Health Monitoring

```rust
struct SensorHealth {
    imu_ok: bool,
    encoder_ok: Vec<bool>,
    last_update: Instant,
    timeout: Duration,
}

impl SensorHealth {
    fn check(&self) -> Result<()> {
        // Check IMU
        if !self.imu_ok {
            return Err(HumanoidError::SafetyStop {
                reason: "IMU failure".to_string(),
            });
        }
        
        // Check encoders
        for (i, ok) in self.encoder_ok.iter().enumerate() {
            if !ok {
                return Err(HumanoidError::SafetyStop {
                    reason: format!("Encoder {} failure", i),
                });
            }
        }
        
        // Check timeout
        if self.last_update.elapsed() > self.timeout {
            return Err(HumanoidError::SafetyStop {
                reason: "Sensor timeout".to_string(),
            });
        }
        
        Ok(())
    }
}
```

### Statistics

```rust
struct SensorStatistics {
    // IMU statistics
    imu_rate_hz: f64,
    imu_jitter_ms: f64,
    
    // Encoder statistics
    encoder_rates: Vec<f64>,
    encoder_jitters: Vec<f64>,
    
    // Contact statistics
    left_duty_cycle: f64,
    right_duty_cycle: f64,
}

impl SensorStatistics {
    fn compute(log: &[SensorLogEntry]) -> Self {
        // Compute statistics from log data
        // ... implementation ...
        
        Self {
            imu_rate_hz: 200.0,
            imu_jitter_ms: 0.5,
            encoder_rates: vec![100.0; 29],
            encoder_jitters: vec![1.0; 29],
            left_duty_cycle: 0.6,
            right_duty_cycle: 0.6,
        }
    }
}
```

## Real-Time Considerations

### Lock-Free Data Structures

```rust
use crossbeam::channel::{bounded, Receiver, Sender};

struct SensorStream {
    sender: Sender<RobotState>,
    receiver: Receiver<RobotState>,
}

impl SensorStream {
    fn new(buffer_size: usize) -> Self {
        let (sender, receiver) = bounded(buffer_size);
        Self { sender, receiver }
    }
    
    fn publish(&self, state: RobotState) {
        // Non-blocking send, drops old data if buffer full
        let _ = self.sender.try_send(state);
    }
    
    fn subscribe(&self) -> Option<RobotState> {
        // Non-blocking receive
        self.receiver.try_recv().ok()
    }
}
```

### Memory Pre-allocation

```rust
struct PreallocatedBuffers {
    // Pre-allocated to avoid allocations in hot path
    joint_positions: Vec<f64>,
    joint_velocities: Vec<f64>,
    orientation: [f64; 4],
}

impl PreallocatedBuffers {
    fn new(num_joints: usize) -> Self {
        Self {
            joint_positions: vec![0.0; num_joints],
            joint_velocities: vec![0.0; num_joints],
            orientation: [1.0, 0.0, 0.0, 0.0],
        }
    }
    
    fn copy_from_state(&mut self, state: &RobotState) {
        self.joint_positions.clear();
        self.joint_positions.extend(state.joints.positions());
        
        self.joint_velocities.clear();
        self.joint_velocities.extend(
            state.joints.iter().map(|j| j.velocity)
        );
        
        self.orientation[0] = state.base_pose.orientation.coords.w;
        self.orientation[1] = state.base_pose.orientation.coords.x;
        self.orientation[2] = state.base_pose.orientation.coords.y;
        self.orientation[3] = state.base_pose.orientation.coords.z;
    }
}
```

## Best Practices

1. **Validate sensor data** - Check for NaN, infinity, out-of-range values
2. **Monitor update rates** - Log and alert on timing issues
3. **Use redundancy** - Multiple sensors for critical measurements
4. **Implement timeouts** - Detect sensor failures quickly
5. **Log raw data** - For debugging and analysis
6. **Pre-allocate memory** - No heap allocations in hot path
7. **Filter noise** - Apply appropriate filters
8. **Synchronize timestamps** - Use monotonic clock
9. **Handle packet loss** - Graceful degradation
10. **Test failure modes** - Simulate sensor failures

## Next Steps

- Explore [ROS2 Integration](05_ros2_integration.md)
- Read the [API Reference](../api_reference.md)
- Learn about [Control Algorithms](03_control_robots.md)
