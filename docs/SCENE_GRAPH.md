# Scene Graph and Asset Loading Pipeline

This module provides a comprehensive scene management system for the humanoid robotics simulator, including hierarchical scene graphs, configuration-based scene definition, and support for multiple file formats.

## Features

### 1. SceneGraph

A hierarchical node structure for organizing simulation entities:

- **Hierarchical nodes**: Parent-child relationships with transform inheritance
- **Named entity lookup**: Fast O(1) access to nodes by name
- **Transform hierarchy**: Local and world transforms with automatic updates
- **Node types**: Robot, Terrain, Obstacle, Sensor, Object, Root
- **Custom properties**: Key-value storage for node-specific data

```rust
use humanoid_rs::sim::scene::{SceneGraph, SceneNode, NodeType};

let mut graph = SceneGraph::new();

// Add a robot
let robot = SceneNode::new("unitree_g1", NodeType::Robot)
    .with_position([0.0, 0.85, 0.0].into())
    .with_property("model", "unitree_g1");
graph.add_node(robot, None)?;

// Lookup nodes
if let Some(node) = graph.get_node_by_name("unitree_g1") {
    println!("Robot at: {:?}", node.world_transform.translation);
}
```

### 2. SceneConfig

TOML/JSON-based scene definition with validation:

```toml
[simulation]
timestep = 0.001
gravity = [0.0, -9.81, 0.0]
friction = 0.8

[[robots]]
name = "g1_robot"
model = "unitree_g1"
position = [0.0, 0.85, 0.0]
control_mode = "position"

[[terrain]]
type = "plane"
friction = 0.8

[[obstacles]]
name = "box_1"
type = "box"
size = [0.5, 0.5, 0.5]
position = [2.0, 0.25, 0.0]
color = [0.8, 0.3, 0.3]
```

### 3. Asset Loaders

Support for multiple file formats:

- **TOML**: Human-readable configuration format
- **JSON**: Machine-readable interchange format  
- **URDF**: Universal Robot Description Format (basic support)
- **SDF**: Simulation Description Format (Gazebo-compatible)

### 4. SceneLoader API

Unified interface for loading scenes:

```rust
use humanoid_rs::sim::assets::{SceneLoader, AssetFormat};
use humanoid_rs::physics_sim::{load_scene_file, load_scene_toml};

// From file (auto-detects format)
let graph = load_scene_file(Path::new("scene.toml"))?;

// From string
let graph = load_scene_toml(toml_string)?;

// With hot-reloading
let mut loader = SceneLoader::new().with_hot_reload(true);
let graph = loader.load_from_file(path)?;
```

## Examples

### Basic Scene (TOML)

```toml
[simulation]
timestep = 0.001
gravity = [0.0, -9.81, 0.0]

[[robots]]
name = "g1_robot"
model = "unitree_g1"
position = [0.0, 0.85, 0.0]

[[terrain]]
type = "plane"
friction = 0.8

[[obstacles]]
name = "box"
type = "box"
size = [1.0, 1.0, 1.0]
position = [3.0, 0.5, 0.0]
```

### Obstacle Course

See `examples/scenes/obstacle_course.toml` for a complete example with multiple obstacles and walls.

### Running Examples

```bash
# Scene graph demo
cargo run --example scene_graph_demo --features simulator

# Load scene from file
cargo run --example scene_loading --features simulator -- examples/scenes/basic_scene.toml

# Load JSON scene
cargo run --example scene_loading --features simulator -- examples/scenes/basic_scene.json
```

## Configuration Reference

### Simulation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `timestep` | f64 | 0.001 | Physics timestep in seconds |
| `gravity` | [f64; 3] | [0, -9.81, 0] | Gravity vector |
| `friction` | f64 | 0.8 | Ground friction coefficient |
| `duration` | f64 | 0.0 | Simulation duration (0 = infinite) |
| `seed` | u64 | 0 | Random seed (0 = random) |

### Robot Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | String | required | Unique robot name |
| `model` | String | "" | Robot model identifier |
| `position` | [f64; 3] | [0, 0, 0] | Position in world coordinates |
| `orientation` | [f64; 3] | [0, 0, 0] | Euler angles (roll, pitch, yaw) |
| `control_mode` | String | "position" | Control mode: position, velocity, torque, passive |
| `model_path` | String | null | Path to URDF/SDF file |
| `scale` | f64 | 1.0 | Model scale factor |

### Terrain Types

- **Plane**: Infinite ground plane
- **FinitePlane**: Finite plane with size [width, depth]
- **Heightmap**: Heightmap terrain from image
- **Stairs**: Staircase terrain
- **Slope**: Inclined ramp

### Obstacle Types

- **Box**: Rectangular prism (requires `size: [x, y, z]`)
- **Sphere**: Sphere (requires `radius`)
- **Cylinder**: Cylinder (requires `radius`, `height`)
- **Capsule**: Capsule (requires `radius`, `height`)
- **Mesh**: Custom mesh (requires `mesh_path`)

## Hot-Reloading

Enable hot-reloading for development:

```rust
let mut loader = SceneLoader::new()
    .with_hot_reload(true);

// In game loop
if let Some(new_graph) = loader.check_hot_reload(path) {
    // Scene file changed, reload
    active_scene.load(new_graph, Some(path));
}
```

## Error Handling

Comprehensive error types for debugging:

- `AssetError::Io`: File I/O errors
- `AssetError::Toml`: TOML parsing errors
- `AssetError::Json`: JSON parsing errors
- `AssetError::Xml`: XML parsing errors (URDF/SDF)
- `AssetError::Validation`: Configuration validation errors
- `AssetError::Scene`: Scene graph errors (duplicate names, etc.)

## Integration with Physics Simulator

The scene loader integrates with the physics simulator:

```rust
use humanoid_rs::physics_sim::run_physics_simulator_with_scene;

// Run simulator with scene file
run_physics_simulator_with_scene("examples/scenes/basic_scene.toml");
```

## Future Enhancements

- GLTF/GLB mesh loading
- Material system integration
- Physics material properties
- Advanced terrain generation
- Procedural obstacle generation
- Multi-robot scene support
