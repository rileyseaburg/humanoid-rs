//! Example: Loading and using a scene file
//!
//! This example demonstrates how to load a scene from a TOML or JSON file
//! and run the physics simulator with it.
//!
//! Usage:
//! ```bash
//! cargo run --example scene_loading --features simulator -- examples/scenes/basic_scene.toml
//! ```

use humanoid_rs::physics_sim::run_physics_simulator_with_scene;
use humanoid_rs::sim::{load_scene_file, load_scene_toml, SceneGraph};
use std::env;
use std::path::Path;

fn main() {
    // Get scene file path from command line arguments
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        println!("Usage: cargo run --example scene_loading -- <scene_file>");
        println!("");
        println!("Examples:");
        println!("  cargo run --example scene_loading -- examples/scenes/basic_scene.toml");
        println!("  cargo run --example scene_loading -- examples/scenes/basic_scene.json");
        println!("");
        println!("Running with default scene...");

        // Create a simple default scene programmatically
        let toml_str = r#"
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
name = "box_1"
type = "box"
size = [0.5, 0.5, 0.5]
position = [2.0, 0.25, 0.0]
color = [0.8, 0.3, 0.3]
"#;

        // Parse the TOML and verify it works
        match load_scene_toml(toml_str) {
            Ok(graph) => {
                println!("✓ Successfully parsed default scene");
                println!("  Nodes: {}", graph.len());
                println!("  Robots: {}", graph.get_robots().len());
                println!("  Terrain: {}", graph.get_terrain().len());
                println!("  Obstacles: {}", graph.get_obstacles().len());
            }
            Err(e) => {
                eprintln!("✗ Failed to parse default scene: {}", e);
                std::process::exit(1);
            }
        }

        return;
    }

    let scene_path = Path::new(&args[1]);

    // Verify the scene file exists
    if !scene_path.exists() {
        eprintln!("Error: Scene file not found: {:?}", scene_path);
        std::process::exit(1);
    }

    // Load and validate the scene
    println!("Loading scene from: {:?}", scene_path);

    match load_scene_file(scene_path) {
        Ok(graph) => {
            println!("✓ Scene loaded successfully!");
            print_scene_info(&graph);

            // Run the simulator with the scene
            println!("\nStarting physics simulator...");
            run_physics_simulator_with_scene(scene_path);
        }
        Err(e) => {
            eprintln!("✗ Failed to load scene: {}", e);
            std::process::exit(1);
        }
    }
}

fn print_scene_info(graph: &SceneGraph) {
    println!("\n=== Scene Information ===");
    println!("Total nodes: {}", graph.len());

    // Print robots
    let robots = graph.get_robots();
    if !robots.is_empty() {
        println!("\nRobots ({}):", robots.len());
        for robot in robots {
            println!(
                "  - {} at {:?}",
                robot.name, robot.world_transform.translation
            );
        }
    }

    // Print terrain
    let terrain = graph.get_terrain();
    if !terrain.is_empty() {
        println!("\nTerrain ({}):", terrain.len());
        for t in terrain {
            if let Some(t_type) = t.get_property("type") {
                println!("  - {} (type: {:?})", t.name, t_type);
            } else {
                println!("  - {}", t.name);
            }
        }
    }

    // Print obstacles
    let obstacles = graph.get_obstacles();
    if !obstacles.is_empty() {
        println!("\nObstacles ({}):", obstacles.len());
        for obstacle in obstacles {
            if let Some(o_type) = obstacle.get_property("type") {
                println!(
                    "  - {} (type: {:?}) at {:?}",
                    obstacle.name, o_type, obstacle.world_transform.translation
                );
            } else {
                println!("  - {}", obstacle.name);
            }
        }
    }

    println!("\n========================\n");
}
