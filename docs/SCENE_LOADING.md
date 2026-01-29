# Scene Loading and Asset Pipeline

This document describes the scene graph and asset loading system for the humanoid simulator.

## Overview

The scene loading system provides:

1. **SceneGraph**: Hierarchical organization of simulation entities
2. **SceneConfig**: TOML/JSON configuration format for scene definitions
3. **AssetLoader**: Support for URDF, SDF, GLTF, and configuration files
4. **Hot-reloading**: Automatic scene updates during development

## Scene Configuration Format

Scenes are defined in TOML or JSON files with the following structure:

```toml
[simulation]
timestep = 0.001
gravity = [0.0, -9.81, 0.0]
friction = 0.8

[[robots]]
name = "g1_robot"
model = "unitree_g1"
position = [0.0, 0.85, 0.0]
orientation = [0.0, 0.0, 0.0]
control_mode = "position"

[[terrain]]
type = "plane"
normal = [0.0, 1.0, 0.0]
friction = 0.8

[[obstacles]]
name = "box_1"
type = "box"
size = [0.5, 0.5, 0.5]
position = [2.0, 0.25, 0.0]
color = [0.8, 0.3, 0.3]
static = true
```

## Supported Asset Formats

### TOML/JSON Configuration
- **Extension**: `.toml`, `.json`
- **Purpose**: Define scenes with robots, terrain, and obstacles
- **Features**: Full scene specification with validation

### URDF (Universal Robot Description Format)
- **Extension**: `.urdf`
- **Purpose**: Load robot models from ROS/robotics formats
- **Status**: Basic support (links, joints, poses)

### SDF (Simulation Description Format)
- **Extension**: `.sdf`
- **Purpose**: Load Gazebo-style scenes
- **Status**: Basic support (models, worlds, poses)

### GLTF (GL Transmission Format)
- **Extension**: `.gltf`, `.glb`
- **Purpose**: Load 3D mesh models
- **Status**: Stub implementation (requires bevy_gltf for full support)

## API Usage

### Loading from File

```rust
use humanoid_rs::sim::{SceneLoader, AssetFormat};
use std::path::Path;

// Auto-detect format from extension
let mut loader = SceneLoader::new();
let graph = loader.load_from_file(Path::new("scene.toml"))?;

// Enable hot-reloading for development
let mut loader = SceneLoader::new().with_hot_reload(true);
```

### Loading from String

```rust
use humanoid_rs::sim::{SceneLoader, AssetFormat};

let loader = SceneLoader::new();
let graph = loader.load_from_str(AssetFormat::Toml, toml_str)?;
```

### Convenience Functions

```rust
use humanoid_rs::sim::util;

// Quick load functions
let graph = util::load_toml_file(Path::new("scene.toml"))?;
let graph = util::load_toml_str(toml_str)?;
let graph = util::load_json_file(Path::new("scene.json"))?;
let graph = util::load_json_str(json_str)?;
let graph = util::load_urdf_str(urdf_str)?;
let graph = util::load_sdf_str(sdf_str)?;
```

### Using with Physics Simulator

```rust
use humanoid_rs::physics_sim::run_physics_simulator_with_scene;
use std::path::Path;

// Run simulator with a scene file
run_physics_simulator_with_scene(Path::new("examples/scenes/basic_scene.toml"));
```

## SceneGraph API

### Querying the Graph

```rust
use humanoid_rs::sim::scene::SceneGraph;

// Get all robots
let robots = graph.get_robots();

// Get all terrain
let terrain = graph.get_terrain();

// Get all obstacles
let obstacles = graph.get_obstacles();

// Lookup by name
if let Some(node) = graph.get_node_by_name("g1_robot") {
    println!("Robot position: {:?}", node.world_transform.translation);
}
```

### Modifying the Graph

```rust
use humanoid_rs::sim::scene::{SceneGraph, SceneNode, NodeType};
use bevy::prelude::*;

let mut graph = SceneGraph::new();

// Add a robot
let robot = SceneNode::new("my_robot", NodeType::Robot)
    .with_position(Vec3::new(0.0, 0.85, 0.0));
graph.add_node(robot, None)?;

// Add an obstacle as a child
let obstacle = SceneNode::new("obstacle_1", NodeType::Obstacle)
    .with_position(Vec3::new(2.0, 0.5, 0.0));
graph.add_node(obstacle, Some("my_robot"))?;

// Update world transforms
graph.update_transforms();
```

## Configuration Reference

### Simulation Configuration

```toml
[simulation]
timestep = 0.001        # Physics timestep in seconds
gravity = [0.0, -9.81, 0.0]  # Gravity vector
duration = 60.0         # Simulation duration (0 = infinite)
friction = 0.8          # Default ground friction
seed = 12345           # Random seed for reproducibility
```

### Robot Configuration

```toml
[[robots]]
name = "robot_name"           # Unique identifier
model = "unitree_g1"          # Robot model type
position = [0.0, 0.85, 0.0]   # [x, y, z] in meters
orientation = [0.0, 0.0, 0.0] # [roll, pitch, yaw] in radians
control_mode = "position"     # "position", "velocity", "torque", "passive"
scale = 1.0                   # Model scale factor
model_path = "path/to.urdf"   # Optional: custom URDF file
```

### Terrain Configuration

```toml
[[terrain]]
type = "plane"                # "plane", "finite_plane", "heightmap", "stairs", "slope"
normal = [0.0, 1.0, 0.0]      # Surface normal (for planes)
height = 0.0                  # Height offset
friction = 0.8                # Friction coefficient
restitution = 0.0             # Bounciness [0, 1]
size = [10.0, 10.0]           # For finite planes: [width, depth]
heightmap_path = "terrain.png" # For heightmap type
heightmap_scale = 1.0         # Max height difference
resolution = [100, 100]       # Heightmap resolution
```

### Obstacle Configuration

```toml
[[obstacles]]
name = "obstacle_name"        # Unique identifier
type = "box"                  # "box", "sphere", "cylinder", "capsule", "mesh"
position = [2.0, 0.5, 0.0]    # [x, y, z] in meters
orientation = [0.0, 0.0, 0.0] # [roll, pitch, yaw] in radians

# For boxes
size = [0.5, 0.5, 0.5]        # [width, height, depth]

# For spheres
type = "sphere"
radius = 0.3

# For cylinders
type = "cylinder"
radius = 0.2
height = 1.0

# For meshes
type = "mesh"
mesh_path = "model.gltf"

# Common properties
color = [0.8, 0.3, 0.3]       # [r, g, b] values in [0, 1]
static = true                 # Is immovable?
mass = 10.0                   # Mass in kg (if not static)
friction = 0.8                # Friction coefficient
```

### Lighting Configuration

```toml
[lighting]
ambient = 0.3                 # Ambient light intensity

[[lighting.directional]]
direction = [-0.5, -1.0, -0.3]
intensity = 1.0
color = [1.0, 1.0, 1.0]
shadows = true

[[lighting.point]]
position = [0.0, 5.0, 0.0]
intensity = 1.0
color = [1.0, 1.0, 1.0]
radius = 10.0
```

## Validation

The configuration system validates scenes and reports errors:

- Duplicate robot/obstacle names
- Invalid timestep (must be positive, max 0.1s)
- Missing required fields
- Invalid numeric ranges
- Missing mesh files

## Error Handling

All loading functions return `Result<SceneGraph, AssetError>`:

```rust
use humanoid_rs::sim::assets::AssetError;

match loader.load_from_file(path) {
    Ok(graph) => println!("Loaded {} nodes", graph.len()),
    Err(AssetError::FileNotFound(path)) => eprintln!("File not found: {}", path),
    Err(AssetError::Toml(e)) => eprintln!("TOML error: {}", e),
    Err(AssetError::Validation(msg)) => eprintln!("Validation error: {}", msg),
    Err(e) => eprintln!("Error: {}", e),
}
```

## Examples

See the `examples/scenes/` directory for complete examples:

- `basic_scene.toml` - Simple scene with robot, plane, and obstacles
- `basic_scene.json` - Same scene in JSON format
- `obstacle_course.toml` - Complex scene with various obstacle types

Run examples:

```bash
# Load TOML scene
cargo run --example scene_loading --features simulator -- examples/scenes/basic_scene.toml

# Load JSON scene
cargo run --example scene_loading --features simulator -- examples/scenes/basic_scene.json

# Load obstacle course
cargo run --example scene_loading --features simulator -- examples/scenes/obstacle_course.toml
```
