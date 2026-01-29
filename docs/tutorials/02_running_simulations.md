# Running Simulations

This tutorial covers how to use the 3D simulator for testing and development.

## Overview

The Humanoid-RS simulator uses:
- **Bevy** - Modern game engine for rendering
- **Rapier3D** - Physics engine for realistic dynamics
- **Real-time visualization** - Interactive 3D view

## Launching the Simulator

### Basic Launch

```bash
cargo run --bin humanoid-sim --features simulator
```

This opens a 3D window showing a Unitree G1 robot.

### With Custom Options

```bash
# Run at specific resolution
HUMANOID_RESOLUTION=1920x1080 cargo run --bin humanoid-sim --features simulator

# Run headless (no window, for testing)
cargo run --bin humanoid-sim --features simulator -- --headless

# Enable physics debug rendering
cargo run --bin humanoid-sim --features simulator -- --debug-physics
```

## Simulator Controls

### Keyboard Controls

| Key | Action |
|-----|--------|
| `Space` | Stand up |
| `C` | Crouch |
| `1` | Wave animation |
| `2` | T-pose |
| `3` | Lean forward |
| `4` | Sit pose |
| `5` | Jump pose |
| `W` | Toggle walking mode |
| `R` | Reset robot |
| `Q/E` | Rotate camera |
| `ESC` | Exit |

### Mouse Controls

- **Left Click + Drag** - Orbit camera
- **Right Click + Drag** - Pan camera
- **Scroll** - Zoom in/out

## Using the Simulator Programmatically

### Creating a Simulation

```rust
use humanoid_rs::simulator::{SimulatedRobot, run_simulator};
use humanoid_rs::control::{JointSpaceController, MotionPlanner};

fn main() {
    // Create robot and controller
    let mut robot = SimulatedRobot::new();
    let planner = MotionPlanner::new(2.0, 5.0);
    let controller = JointSpaceController::new(50.0, 10.0);
    
    // Set up controller
    let trajectory = planner.plan_stand_up(&robot.state.joints);
    controller.set_trajectory(trajectory);
    robot.set_controller(Box::new(controller));
    
    // Run simulator (this blocks until window closes)
    run_simulator();
}
```

### Custom Simulation Loop

For more control, create a custom Bevy app:

```rust
use bevy::prelude::*;
use humanoid_rs::simulator::HumanoidSimulatorPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(HumanoidSimulatorPlugin)
        .add_systems(Update, my_custom_system)
        .run();
}

fn my_custom_system(
    // Access Bevy resources and components
) {
    // Your custom logic here
}
```

## Simulation Parameters

### Physics Settings

```rust
// In your setup
app.insert_resource(RapierConfiguration {
    gravity: Vec3::new(0.0, -9.81, 0.0),
    timestep_mode: TimestepMode::Fixed {
        dt: 1.0 / 1000.0,  // 1000 Hz physics
        substeps: 1,
    },
    ..default()
});
```

### Robot Configuration

```rust
use humanoid_rs::robot::JointLimits;

// Customize joint limits for simulation
let mut joints = unitree_g1::create_default_joints();
for joint in joints.iter_mut() {
    joint.limits = JointLimits {
        min: -3.0,
        max: 3.0,
        max_velocity: 5.0,
        max_effort: 50.0,
    };
}
```

## Recording and Playback

### Recording Simulation

```rust
use std::fs::File;
use std::io::Write;

// In your system
fn record_state(robot: Res<SimulatedRobot>) {
    let state_json = robot.get_state_json();
    
    // Write to file
    let mut file = File::options()
        .append(true)
        .create(true)
        .open("simulation_log.jsonl")
        .unwrap();
    
    writeln!(file, "{}", state_json).unwrap();
}
```

### Playback Recording

```rust
use std::fs::File;
use std::io::{BufRead, BufReader};

fn playback_recording(path: &str) -> Vec<RobotState> {
    let file = File::open(path).unwrap();
    let reader = BufReader::new(file);
    
    reader
        .lines()
        .filter_map(|line| {
            let line = line.ok()?;
            serde_json::from_str(&line).ok()
        })
        .collect()
}
```

## Testing Control Algorithms

### Unit Testing with Simulator

```rust
#[test]
fn test_stand_up_controller() {
    let mut robot = SimulatedRobot::new();
    let planner = MotionPlanner::new(2.0, 5.0);
    let mut controller = JointSpaceController::new(50.0, 10.0);
    
    // Plan and execute
    let trajectory = planner.plan_stand_up(&robot.state.joints);
    controller.set_trajectory(trajectory);
    
    // Simulate for 3 seconds
    let dt = Duration::from_millis(10);
    for _ in 0..300 {
        let output = controller.update(&robot.state, dt).unwrap();
        let positions: Vec<f64> = output.iter()
            .map(|j| j.target_position)
            .collect();
        robot.set_target_positions(positions);
    }
    
    // Verify robot is standing
    assert!(robot.state.is_standing);
}
```

### Benchmarking

```rust
use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn bench_control_loop(c: &mut Criterion) {
    let mut robot = SimulatedRobot::new();
    let planner = MotionPlanner::new(2.0, 5.0);
    let mut controller = JointSpaceController::new(50.0, 10.0);
    
    let trajectory = planner.plan_stand_up(&robot.state.joints);
    controller.set_trajectory(trajectory);
    
    let dt = Duration::from_millis(2); // 500 Hz
    
    c.bench_function("control_update", |b| {
        b.iter(|| {
            let _ = controller.update(black_box(&robot.state), black_box(dt));
        });
    });
}

criterion_group!(benches, bench_control_loop);
criterion_main!(benches);
```

## Visualization Features

### Joint Visualization

The simulator shows:
- **Body parts** - Dark gray metallic
- **Joints** - Orange/red spheres
- **Joint axes** - Visible rotation axes
- **Accent colors** - Blue highlights

### Physics Debug

Enable debug rendering to see:
- Collision shapes
- Contact points
- Force vectors
- Center of mass

```bash
cargo run --bin humanoid-sim --features simulator -- --debug-physics
```

### Camera Views

Switch between camera modes:

```rust
// In your Bevy system
fn setup_cameras(mut commands: Commands) {
    // Orbit camera (default)
    commands.spawn((
        Camera3dBundle::default(),
        OrbitCamera {
            target: Vec3::new(0.0, 0.8, 0.0),
            distance: 4.0,
        },
    ));
    
    // First-person view from robot head
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 1.6, 0.0),
            ..default()
        },
        FirstPersonCamera,
    ));
}
```

## Advanced Topics

### Custom Animations

```rust
fn animate_wave(time: Res<Time>, mut joints: Query<&mut JointRotation>) {
    let t = time.elapsed_seconds();
    let wave_angle = (t * 5.0).sin() * 0.5;
    
    for mut joint in joints.iter_mut() {
        if joint.name == "right_shoulder" {
            joint.target_angle = -2.5; // Raise arm
        }
        if joint.name == "right_elbow" {
            joint.target_angle = -0.5 + wave_angle; // Wave
        }
    }
}
```

### Multiple Robots

```rust
fn spawn_multiple_robots(mut commands: Commands) {
    for i in 0..5 {
        let x = (i as f32 - 2.0) * 2.0;
        spawn_robot_at(&mut commands, Vec3::new(x, 0.0, 0.0));
    }
}
```

### Environment Setup

```rust
fn setup_environment(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh().size(50.0, 50.0)),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3)),
            ..default()
        },
        Collider::cuboid(25.0, 0.01, 25.0),
        RigidBody::Fixed,
    ));
    
    // Obstacles
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb(0.8, 0.2, 0.2)),
            transform: Transform::from_xyz(3.0, 0.5, 0.0),
            ..default()
        },
        Collider::cuboid(0.5, 0.5, 0.5),
        RigidBody::Fixed,
    ));
}
```

## Performance Optimization

### Reducing Physics Load

```rust
// Reduce physics precision for better performance
app.insert_resource(RapierConfiguration {
    timestep_mode: TimestepMode::Fixed {
        dt: 1.0 / 500.0,  // 500 Hz instead of 1000 Hz
        substeps: 1,
    },
    ..default()
});
```

### Optimizing Rendering

```rust
// Reduce shadow quality
commands.spawn(DirectionalLightBundle {
    directional_light: DirectionalLight {
        shadows_enabled: false,  // Disable shadows
        ..default()
    },
    ..default()
});
```

## Troubleshooting

### Low Frame Rate

1. **Reduce physics frequency**: Edit `timestep_mode` to 500 Hz
2. **Disable shadows**: Set `shadows_enabled: false`
3. **Reduce window resolution**: Use smaller window
4. **Close other applications**: Free up GPU resources

### Physics Glitches

1. **Increase substeps**: Set `substeps: 4` in timestep_mode
2. **Check for NaN values**: Validate all joint commands
3. **Reduce time step**: Use 1000 Hz physics
4. **Check collision shapes**: Ensure no overlapping at start

### Build Issues

**Error: "wgpu error"**

Try different graphics backends:
```bash
# Windows
set WGPU_BACKEND=dx12
cargo run --bin humanoid-sim --features simulator

# Linux
export WGPU_BACKEND=vulkan
cargo run --bin humanoid-sim --features simulator

# macOS
export WGPU_BACKEND=metal
cargo run --bin humanoid-sim --features simulator
```

**Error: "failed to initialize graphics"**

Use software rendering:
```bash
export WGPU_BACKEND=gl
export LIBGL_ALWAYS_SOFTWARE=1
cargo run --bin humanoid-sim --features simulator
```

## Next Steps

- Learn about [Control Algorithms](03_control_robots.md)
- Explore [Sensors and Observability](04_sensors_and_observability.md)
- Integrate with [ROS2](05_ros2_integration.md)
