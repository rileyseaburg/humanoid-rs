# Controlling Robots

This tutorial covers control algorithms for humanoid robots, from basic joint control to advanced whole-body coordination.

> **Quick Start:** New to Humanoid-RS? Start with [QUICKSTART.md](../../QUICKSTART.md) or the `starter.rs` example.

## Overview

Humanoid-RS provides several control approaches:

1. **Joint Space Control** - Direct joint position/velocity control
2. **Motion Planning** - Trajectory generation for smooth motions
3. **MPC** - Model Predictive Control for locomotion
4. **Whole-Body Control** - Coordinated multi-task control
5. **Balance Control** - Stability maintenance and recovery

## Joint Space Control

The simplest form of control - directly commanding joint positions.

### Basic PD Controller

```rust
use humanoid_rs::control::JointSpaceController;

// Create PD controller with gains
// Kp = 100.0 (stiffness)
// Kd = 20.0 (damping)
let mut controller = JointSpaceController::new(100.0, 20.0);

// Set target trajectory
controller.set_trajectory(trajectory);

// Run control loop
loop {
    let output = controller.update(&robot_state, dt)?;
    
    // Extract joint efforts
    for joint in output.iter() {
        println!("Joint {}: effort = {:.2} Nm", 
            joint.name, joint.effort);
    }
}
```

### Tuning PD Gains

| Gain | Effect | Too Low | Too High |
|------|--------|---------|----------|
| Kp | Stiffness | Slow response, steady error | Oscillation, instability |
| Kd | Damping | Overshoot, ringing | Sluggish response, noise |

**Tuning procedure:**
1. Set Kd = 0, increase Kp until oscillation
2. Reduce Kp by 50%
3. Increase Kd until oscillation stops
4. Fine-tune for desired response

### Position Control with Limits

```rust
use humanoid_rs::robot::{HumanoidRobot, JointLimits};

// Safety check before sending commands
fn safe_position_control(
    robot: &mut impl HumanoidRobot,
    positions: &[f64],
) -> Result<()> {
    // Get current state
    let state = robot.state();
    
    // Validate positions against limits
    for (i, (pos, joint)) in positions.iter()
        .zip(state.joints.iter())
        .enumerate() {
        if *pos < joint.limits.min || *pos > joint.limits.max {
            return Err(HumanoidError::JointLimitExceeded {
                joint_name: joint.name.clone(),
                requested: *pos,
                min: joint.limits.min,
                max: joint.limits.max,
            });
        }
        
        // Check velocity limit
        let velocity = (pos - joint.current_position).abs() / dt.as_secs_f64();
        if velocity > joint.limits.max_velocity {
            return Err(HumanoidError::SafetyStop {
                reason: format!(
                    "Joint {} velocity {:.2} exceeds limit {:.2}",
                    i, velocity, joint.limits.max_velocity
                ),
            });
        }
    }
    
    // Send validated command
    robot.set_joint_positions(positions)
}
```

## Motion Planning

### Trajectory Generation

```rust
use humanoid_rs::control::{MotionPlanner, Trajectory, Waypoint};
use std::time::Duration;

let planner = MotionPlanner::new(
    2.0,  // max_velocity (rad/s)
    5.0   // max_acceleration (rad/s²)
);

// Plan stand-up motion
let trajectory = planner.plan_stand_up(&current_joints);

// Plan custom trajectory
let mut trajectory = Trajectory::new();

// Start waypoint
trajectory.add_waypoint(Waypoint {
    time: Duration::from_secs(0),
    joint_positions: current_joints.positions(),
    base_pose: None,
});

// End waypoint
trajectory.add_waypoint(Waypoint {
    time: Duration::from_secs(3),
    joint_positions: target_positions,
    base_pose: Some(target_pose),
});
```

### Interpolation

```rust
// Get position at specific time
let time = Duration::from_millis(1500);
if let Some(waypoint) = trajectory.interpolate(time) {
    println!("At t={:?}, positions: {:?}", 
        time, waypoint.joint_positions);
}
```

### Trajectory Types

**Joint-space trajectory:**
```rust
let trajectory = planner.plan_joint_space(
    &current_joints,
    &target_joints,
    Duration::from_secs(5),
);
```

**Cartesian trajectory:**
```rust
let trajectory = planner.plan_cartesian(
    &start_pose,
    &end_pose,
    Duration::from_secs(3),
);
```

## Model Predictive Control (MPC)

MPC is used for locomotion and balance control.

### Basic MPC Setup

```rust
use humanoid_rs::mpc::{MpcController, MpcConfig, LipmState, ZmpReference};
use nalgebra::Vector3;

// Configure MPC
let config = MpcConfig {
    horizon: 20,           // 20-step prediction
    dt: 0.02,              // 50 Hz control
    com_height: 0.85,      // CoM height
    weight_com_pos: 1.0,   // Position tracking weight
    weight_com_vel: 0.1,   // Velocity tracking weight
    weight_zmp: 10.0,      // ZMP tracking weight
    weight_jerk: 0.001,    // Smoothness weight
    zmp_margin: 0.08,      // Stability margin
    gravity: 9.81,
};

let mut mpc = MpcController::new(config);

// Set ZMP reference (standing still)
let zmp_ref = ZmpReference::standing(5.0, 0.02, Vector3::zeros());
mpc.set_reference(zmp_ref);

// Control loop
loop {
    // Current CoM state
    let com_state = LipmState {
        x: robot_state.base_pose.position.x,
        x_dot: 0.0,
        y: robot_state.base_pose.position.z,
        y_dot: 0.0,
    };
    
    // Solve MPC
    let (zmp_cmd, com_vel_cmd) = mpc.solve(&com_state);
    
    // Use ZMP command for foot placement
    println!("ZMP command: [{:.3}, {:.3}]", zmp_cmd.x, zmp_cmd.z);
}
```

### Walking with MPC

```rust
use humanoid_rs::mpc::FootstepPlanner;

let footstep_planner = FootstepPlanner {
    step_length: 0.15,   // meters
    step_width: 0.20,    // meters
    step_duration: 0.4,  // seconds
    double_support_ratio: 0.2,
};

// Plan footsteps for walking forward at 0.3 m/s
let velocity = Vector3::new(0.3, 0.0, 0.0);
let footsteps = footstep_planner.plan_walking(velocity, 10.0);

// Generate ZMP reference from footsteps
let zmp_ref = footstep_planner.footsteps_to_zmp(&footsteps, 0.02);
mpc.set_reference(zmp_ref);
```

### Capture Point (DCM)

The capture point predicts where the CoM will stabilize:

```rust
let capture_point = mpc.capture_point(&com_state);
println!("Capture point: [{:.3}, {:.3}]", 
    capture_point.x, capture_point.z);

// If capture point is outside support polygon, robot is falling
```

## Whole-Body Control

Coordinated control of all joints for multiple objectives.

### Basic Setup

```rust
use humanoid_rs::whole_body::{
    WholeBodyController, WholeBodyConfig, ComState, FootContact
};

// Configure controller
let config = WholeBodyConfig {
    com_kp: Vector3::new(100.0, 100.0, 100.0),
    com_kd: Vector3::new(20.0, 20.0, 20.0),
    orientation_kp: Vector3::new(100.0, 100.0, 100.0),
    orientation_kd: Vector3::new(20.0, 20.0, 20.0),
    foot_kp: 200.0,
    foot_kd: 40.0,
    joint_damping: 10.0,
    regularization: 1e-6,
    min_accel_weight: 0.01,
    robot_mass: 35.0,
    gravity: Vector3::new(0.0, -9.81, 0.0),
};

let mut wbc = WholeBodyController::new(config);
```

### Setting Targets

```rust
// Set CoM target
wbc.set_com_target(
    Vector3::new(0.0, 0.85, 0.0),  // position
    Vector3::new(0.1, 0.0, 0.0),   // velocity
);

// Set foot contacts
let left_foot = FootContact {
    position: Vector3::new(-0.1, 0.0, 0.0),
    normal: Vector3::y(),
    in_contact: true,
    force: Vector3::new(0.0, 200.0, 0.0),
};

let right_foot = FootContact {
    position: Vector3::new(0.1, 0.0, 0.0),
    normal: Vector3::y(),
    in_contact: true,
    force: Vector3::new(0.0, 200.0, 0.0),
};

wbc.set_foot_contacts(left_foot, right_foot);
```

### Computing Control

```rust
// Compute joint torques
let torques = wbc.compute(&robot_state, dt);

// Check balance
if wbc.is_balanced(&robot_state) {
    println!("Robot is balanced");
} else {
    let margin = wbc.stability_margin(&robot_state);
    println!("Stability margin: {:.3} m", margin);
}
```

## Balance Recovery

When the robot is pushed or loses balance, use recovery strategies:

```rust
use humanoid_rs::whole_body::{BalanceRecovery, RecoveryAction};

let recovery = BalanceRecovery::default();

// Check if recovery is needed
let action = recovery.compute(&com_state, &support_polygon);

match action {
    RecoveryAction::None => {
        // Robot is stable, continue normal control
    }
    RecoveryAction::Ankle { torque } => {
        // Apply ankle torque to recover
        println!("Using ankle strategy: torque={:.2}", torque);
    }
    RecoveryAction::Hip { angle } => {
        // Use hip strategy
        println!("Using hip strategy: angle={:.2}", angle);
    }
    RecoveryAction::Step { target } => {
        // Take a step to recover
        println!("Using step strategy: target=[{:.2}, {:.2}]", 
            target.x, target.z);
        
        // Plan and execute step
        plan_recovery_step(target);
    }
}
```

## Control Loop Architecture

### Basic Control Loop

```rust
use std::time::{Duration, Instant};

fn control_loop(robot: &mut impl HumanoidRobot) -> Result<()> {
    let target_rate_hz = 500.0;
    let dt = Duration::from_secs_f64(1.0 / target_rate_hz);
    
    let mut controller = create_controller();
    let mut last_time = Instant::now();
    
    loop {
        let loop_start = Instant::now();
        
        // 1. Read state
        let state = robot.state();
        
        // 2. Update controller
        let output = controller.update(state, dt)?;
        
        // 3. Send commands
        let positions: Vec<f64> = output.iter()
            .map(|j| j.target_position)
            .collect();
        robot.set_joint_positions(&positions)?;
        
        // 4. Maintain timing
        let elapsed = loop_start.elapsed();
        if elapsed < dt {
            std::thread::sleep(dt - elapsed);
        }
        
        // Monitor timing
        let jitter = loop_start.elapsed().as_micros() as i64
            - dt.as_micros() as i64;
        if jitter.abs() > 100 { // 100 microseconds threshold
            eprintln!("Control loop jitter: {} µs", jitter);
        }
    }
}
```

### Multi-Rate Control

Different control tasks at different rates:

```rust
fn multi_rate_control(robot: &mut impl HumanoidRobot) -> Result<()> {
    let mut balance_controller = BalanceController::new();
    let mut joint_controller = JointSpaceController::new(100.0, 20.0);
    
    let fast_dt = Duration::from_millis(2);   // 500 Hz
    let slow_dt = Duration::from_millis(10);  // 100 Hz
    
    let mut last_slow = Instant::now();
    
    loop {
        let now = Instant::now();
        let state = robot.state();
        
        // Fast loop: Joint control (500 Hz)
        let joint_output = joint_controller.update(state, fast_dt)?;
        
        // Slow loop: Balance control (100 Hz)
        if now - last_slow >= slow_dt {
            balance_controller.update(state, slow_dt)?;
            last_slow = now;
        }
        
        // Combine outputs
        let combined = combine_outputs(&joint_output, &balance_controller);
        robot.set_joint_efforts(&combined)?;
        
        std::thread::sleep(fast_dt);
    }
}
```

## Real-World Considerations

### Communication Delays

```rust
// Compensate for network delay
const NETWORK_DELAY_MS: u64 = 10;

fn compensate_delay(
    current_state: &RobotState,
    commanded_positions: &[f64],
    delay: Duration,
) -> Vec<f64> {
    // Predict state at command arrival time
    let predicted_positions: Vec<f64> = current_state
        .joints
        .iter()
        .zip(commanded_positions.iter())
        .map(|(joint, cmd)| {
            // Simple linear prediction
            joint.current_position + joint.velocity * delay.as_secs_f64()
        })
        .collect();
    
    predicted_positions
}
```

### Sensor Fusion

```rust
// Combine IMU and encoder data
fn fuse_sensors(
    imu_orientation: &[f64; 4],
    encoder_positions: &[f64],
    dt: Duration,
) -> RobotState {
    // Kalman filter or complementary filter
    // ... implementation ...
}
```

### Safety Monitoring

```rust
fn safety_monitor(robot: &impl HumanoidRobot) -> Result<()> {
    let state = robot.state();
    
    // Check joint limits
    for joint in state.joints.iter() {
        if joint.current_position < joint.limits.min 
            || joint.current_position > joint.limits.max {
            return Err(HumanoidError::SafetyStop {
                reason: format!("Joint {} exceeded limits", joint.name),
            });
        }
    }
    
    // Check balance
    if !state.is_balanced {
        return Err(HumanoidError::SafetyStop {
            reason: "Robot lost balance".to_string(),
        });
    }
    
    Ok(())
}
```

## Best Practices

1. **Always validate commands** before sending to hardware
2. **Use safety limits** even in simulation
3. **Monitor control loop timing** and log jitter
4. **Start with low gains** and increase gradually
5. **Test in simulation** before trying on real robot
6. **Implement emergency stop** on separate thread
7. **Log all safety events** for analysis
8. **Use multi-rate control** for different tasks
9. **Compensate for delays** in real hardware
10. **Have a recovery plan** for balance loss

## Next Steps

- Learn about [Sensors and Observability](04_sensors_and_observability.md)
- Explore [ROS2 Integration](05_ros2_integration.md)
- Read the [API Reference](../api_reference.md)
