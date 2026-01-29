//! Example: Scene Graph and Asset Loading
//!
//! This example demonstrates the scene graph system and asset loading capabilities.
//!
//! Usage:
//! ```bash
//! cargo run --example scene_graph_demo --features simulator
//! ```

use humanoid_rs::physics_sim::{load_scene_file, load_scene_toml};
use humanoid_rs::sim::assets::{AssetFormat, SceneConfig, SceneLoader};
use humanoid_rs::sim::scene::{NodeType, SceneGraph, SceneNode};

fn main() {
    println!("=== Scene Graph and Asset Loading Demo ===\n");

    // Demo 1: Create a scene graph programmatically
    demo_programmatic_scene();

    // Demo 2: Load scene from TOML string
    demo_toml_loading();

    // Demo 3: Scene configuration
    demo_scene_config();

    println!("\n=== Demo Complete ===");
}

fn demo_programmatic_scene() {
    println!("1. Creating Scene Graph Programmatically");
    println!("   -----------------------------------");

    let mut graph = SceneGraph::new();

    // Add a robot
    let robot = SceneNode::new("unitree_g1", NodeType::Robot)
        .with_position([0.0, 0.85, 0.0].into())
        .with_property("model", "unitree_g1");
    graph.add_node(robot, None).unwrap();

    // Add terrain
    let terrain = SceneNode::new("ground", NodeType::Terrain)
        .with_property("type", "Plane")
        .with_property("friction", "0.8");
    graph.add_node(terrain, None).unwrap();

    // Add an obstacle
    let obstacle = SceneNode::new("box_obstacle", NodeType::Obstacle)
        .with_position([2.0, 0.25, 0.0].into())
        .with_property("type", "Box")
        .with_property("size_x", "0.5")
        .with_property("size_y", "0.5")
        .with_property("size_z", "0.5");
    graph.add_node(obstacle, None).unwrap();

    // Update transforms
    graph.update_transforms();

    // Print scene info
    println!("   ✓ Created scene with {} nodes", graph.len());
    println!("   ✓ Robots: {}", graph.get_robots().len());
    println!("   ✓ Terrain: {}", graph.get_terrain().len());
    println!("   ✓ Obstacles: {}", graph.get_obstacles().len());

    // Lookup a node
    if let Some(node) = graph.get_node_by_name("unitree_g1") {
        println!(
            "   ✓ Found robot '{}' at position {:?}",
            node.name, node.world_transform.translation
        );
    }

    println!();
}

fn demo_toml_loading() {
    println!("2. Loading Scene from TOML");
    println!("   ------------------------");

    let toml_str = r#"
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
name = "red_box"
type = "box"
size = [0.5, 0.5, 0.5]
position = [2.0, 0.25, 0.0]
color = [0.8, 0.3, 0.3]
static = true

[[obstacles]]
name = "blue_sphere"
type = "sphere"
radius = 0.3
position = [-2.0, 0.3, 0.0]
color = [0.3, 0.3, 0.8]
static = true

[lighting]
ambient = 0.3

[[lighting.directional]]
direction = [-0.5, -1.0, -0.3]
intensity = 1.0
color = [1.0, 1.0, 1.0]
shadows = true
"#;

    match load_scene_toml(toml_str) {
        Ok(graph) => {
            println!("   ✓ Successfully loaded scene from TOML");
            println!("   ✓ Total nodes: {}", graph.len());
            println!("   ✓ Robots: {}", graph.get_robots().len());
            println!("   ✓ Terrain: {}", graph.get_terrain().len());
            println!("   ✓ Obstacles: {}", graph.get_obstacles().len());

            // Print robot info
            for robot in graph.get_robots() {
                println!(
                    "   ✓ Robot '{}' at {:?}",
                    robot.name, robot.world_transform.translation
                );
            }
        }
        Err(e) => {
            println!("   ✗ Failed to load TOML: {}", e);
        }
    }

    println!();
}

fn demo_scene_config() {
    println!("3. Scene Configuration API");
    println!("   ------------------------");

    // Create configuration programmatically
    let config = SceneConfig {
        simulation: humanoid_rs::sim::assets::config::SimulationConfig {
            timestep: 0.001,
            gravity: [0.0, -9.81, 0.0],
            friction: 0.8,
            duration: 0.0,
            seed: 42,
        },
        robots: vec![humanoid_rs::sim::assets::config::RobotConfig {
            name: "test_robot".to_string(),
            model: "unitree_g1".to_string(),
            position: [0.0, 0.85, 0.0],
            orientation: [0.0, 0.0, 0.0],
            initial_joints: vec![],
            control_mode: humanoid_rs::sim::assets::config::ControlModeConfig::Position,
            model_path: None,
            scale: 1.0,
            properties: std::collections::HashMap::new(),
        }],
        terrain: vec![humanoid_rs::sim::assets::config::TerrainConfig {
            terrain_type: humanoid_rs::sim::assets::config::TerrainType::Plane,
            normal: [0.0, 1.0, 0.0],
            height: 0.0,
            friction: Some(0.8),
            restitution: None,
            size: None,
            heightmap_path: None,
            heightmap_scale: 1.0,
            resolution: [100, 100],
        }],
        obstacles: vec![],
        lighting: humanoid_rs::sim::assets::config::LightingConfig::default(),
    };

    // Validate configuration
    match config.validate() {
        Ok(_) => {
            println!("   ✓ Configuration is valid");

            // Serialize to TOML
            match config.to_toml() {
                Ok(toml) => {
                    println!("   ✓ Serialized to TOML ({} bytes)", toml.len());
                }
                Err(e) => {
                    println!("   ✗ Failed to serialize: {}", e);
                }
            }

            // Serialize to JSON
            match config.to_json() {
                Ok(json) => {
                    println!("   ✓ Serialized to JSON ({} bytes)", json.len());
                }
                Err(e) => {
                    println!("   ✗ Failed to serialize: {}", e);
                }
            }
        }
        Err(e) => {
            println!("   ✗ Configuration invalid: {}", e);
        }
    }

    println!();
}
