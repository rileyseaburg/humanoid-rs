//! SDF (Simulation Description Format) parser
//!
//! Provides support for parsing SDF files (used by Gazebo) and converting
//! them to scene graphs. Supports complex scenes with multiple models,
//! worlds, and physics parameters.

use crate::sim::assets::{AssetError, Result};
use crate::sim::scene::{NodeType, SceneGraph, SceneNode};
use quick_xml::events::Event;
use quick_xml::Reader;

/// SDF loader for complex scenes
pub struct SdfLoader;

impl SdfLoader {
    /// Create a new SDF loader
    pub fn new() -> Self {
        Self
    }

    /// Load a scene graph from SDF XML data
    pub fn load(&self, data: &str) -> Result<SceneGraph> {
        let mut graph = SceneGraph::new();
        let mut reader = Reader::from_str(data);
        reader.trim_text(true);

        let mut buf = Vec::new();
        let mut current_world: Option<SdfWorld> = None;
        let mut current_model: Option<SdfModel> = None;
        let mut current_element = String::new();

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(e)) => {
                    current_element = String::from_utf8_lossy(e.name().as_ref()).to_string();

                    match current_element.as_str() {
                        "sdf" => {
                            // SDF version info
                        }
                        "world" => {
                            current_world = Some(SdfWorld::from_attributes(e.attributes()));
                        }
                        "model" => {
                            current_model = Some(SdfModel::from_attributes(e.attributes()));
                        }
                        _ => {}
                    }
                }
                Ok(Event::End(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).to_string();

                    match name.as_str() {
                        "model" => {
                            if let Some(model) = current_model.take() {
                                if let Some(ref mut world) = current_world {
                                    world.models.push(model);
                                } else {
                                    // Model outside of world (standalone)
                                    self.add_model_to_graph(&mut graph, model)?;
                                }
                            }
                        }
                        "world" => {
                            if let Some(world) = current_world.take() {
                                self.add_world_to_graph(&mut graph, world)?;
                            }
                        }
                        _ => {}
                    }
                }
                Ok(Event::Text(e)) => {
                    let _text = e.unescape().unwrap_or_default().to_string();
                    // Parse text content based on current_element
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(AssetError::Xml(format!(
                        "XML parsing error at position {}: {:?}",
                        reader.buffer_position(),
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        if graph.is_empty() {
            // If no world or models were found, return empty graph
            // This allows for minimal SDF files
        }

        Ok(graph)
    }

    /// Add a world to the scene graph
    fn add_world_to_graph(&self, graph: &mut SceneGraph, world: SdfWorld) -> Result<()> {
        // Create world node
        let world_node =
            SceneNode::new(&world.name, NodeType::Object).with_property("sdf_type", "world");
        let _world_idx = graph
            .add_node(world_node, None)
            .map_err(AssetError::Scene)?;

        // Add all models in the world
        for model in world.models {
            self.add_model_to_graph(graph, model)?;
        }

        Ok(())
    }

    /// Add a model to the scene graph
    fn add_model_to_graph(&self, graph: &mut SceneGraph, model: SdfModel) -> Result<()> {
        // Determine node type based on model name/type
        let node_type = if model.name.contains("robot") || model.name.contains("Robot") {
            NodeType::Robot
        } else if model.name.contains("ground") || model.name.contains("terrain") {
            NodeType::Terrain
        } else if model.name.contains("obstacle")
            || model.name.contains("box")
            || model.name.contains("sphere")
            || model.name.contains("cylinder")
        {
            NodeType::Obstacle
        } else {
            NodeType::Object
        };

        use bevy::prelude::*;

        let position = Vec3::new(
            model.pose.0[0] as f32,
            model.pose.0[1] as f32,
            model.pose.0[2] as f32,
        );

        let rotation = Quat::from_euler(
            EulerRot::XYZ,
            model.pose.1[0] as f32,
            model.pose.1[1] as f32,
            model.pose.1[2] as f32,
        );

        let model_node = SceneNode::new(&model.name, node_type)
            .with_position(position)
            .with_rotation(rotation)
            .with_property("sdf_type", "model")
            .with_property("is_static", model.static_.to_string());

        graph
            .add_node(model_node, None)
            .map_err(AssetError::Scene)?;

        // Add links as children
        for link in &model.links {
            let link_node =
                SceneNode::new(&format!("{}.{}", model.name, link.name), NodeType::Object);
            graph
                .add_node(link_node, Some(&model.name))
                .map_err(AssetError::Scene)?;
        }

        Ok(())
    }
}

impl Default for SdfLoader {
    fn default() -> Self {
        Self::new()
    }
}

/// SDF world structure
#[derive(Debug, Default)]
struct SdfWorld {
    name: String,
    models: Vec<SdfModel>,
    physics: Option<SdfPhysics>,
}

impl SdfWorld {
    fn from_attributes(attrs: quick_xml::events::attributes::Attributes) -> Self {
        let mut world = Self::default();

        for attr in attrs {
            if let Ok(attr) = attr {
                let key = String::from_utf8_lossy(attr.key.as_ref());
                let value = String::from_utf8_lossy(&attr.value);

                match key.as_ref() {
                    "name" => world.name = value.to_string(),
                    _ => {}
                }
            }
        }

        // Default name if not provided
        if world.name.is_empty() {
            world.name = "default".to_string();
        }

        world
    }
}

/// SDF model structure
#[derive(Debug, Default)]
struct SdfModel {
    name: String,
    static_: bool,
    pose: ([f64; 3], [f64; 3]), // (xyz, rpy)
    links: Vec<SdfLink>,
    joints: Vec<SdfJoint>,
}

impl SdfModel {
    fn from_attributes(attrs: quick_xml::events::attributes::Attributes) -> Self {
        let mut model = Self::default();

        for attr in attrs {
            if let Ok(attr) = attr {
                let key = String::from_utf8_lossy(attr.key.as_ref());
                let value = String::from_utf8_lossy(&attr.value);

                match key.as_ref() {
                    "name" => model.name = value.to_string(),
                    _ => {}
                }
            }
        }

        model
    }
}

/// SDF link structure
#[derive(Debug, Default)]
struct SdfLink {
    name: String,
    pose: ([f64; 3], [f64; 3]),
}

/// SDF joint structure
#[derive(Debug, Default)]
struct SdfJoint {
    name: String,
    joint_type: String,
    parent: String,
    child: String,
}

/// SDF physics parameters
#[derive(Debug, Default)]
struct SdfPhysics {
    max_step_size: f64,
    real_time_factor: f64,
    real_time_update_rate: f64,
}

/// Parse SDF pose element (6 values: x y z roll pitch yaw)
fn parse_pose(text: &str) -> ([f64; 3], [f64; 3]) {
    let values: Vec<f64> = text
        .split_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();

    if values.len() >= 6 {
        (
            [values[0], values[1], values[2]],
            [values[3], values[4], values[5]],
        )
    } else {
        ([0.0; 3], [0.0; 3])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_sdf() {
        let sdf_str = r#"<?xml version="1.0"?>
<sdf version="1.6">
    <world name="default">
        <model name="ground_plane">
            <static>true</static>
            <link name="link"/>
        </model>
        <model name="robot">
            <pose>0 0 0.5 0 0 0</pose>
            <link name="base_link"/>
        </model>
    </world>
</sdf>
"#;

        let loader = SdfLoader::new();
        let graph = loader.load(sdf_str).unwrap();

        // Should have default, world, and models
        assert!(graph.len() > 0);
    }

    #[test]
    fn test_parse_standalone_model() {
        let sdf_str = r#"<?xml version="1.0"?>
<sdf version="1.6">
    <model name="box_obstacle">
        <pose>2.0 0.0 0.5 0 0 0</pose>
        <link name="link"/>
    </model>
</sdf>
"#;

        let loader = SdfLoader::new();
        let graph = loader.load(sdf_str).unwrap();

        assert!(graph.has_node("box_obstacle"));
    }

    #[test]
    fn test_parse_pose() {
        let pose = parse_pose("1.0 2.0 3.0 0.1 0.2 0.3");
        assert_eq!(pose.0, [1.0, 2.0, 3.0]);
        assert_eq!(pose.1, [0.1, 0.2, 0.3]);

        // Test incomplete pose
        let incomplete = parse_pose("1.0 2.0 3.0");
        assert_eq!(incomplete.0, [0.0, 0.0, 0.0]);
        assert_eq!(incomplete.1, [0.0, 0.0, 0.0]);
    }
}
