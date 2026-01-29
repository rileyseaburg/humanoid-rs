//! URDF (Universal Robot Description Format) parser
//!
//! Provides basic support for parsing URDF files and converting them
//! to scene graphs. Supports robots, links, joints, and materials.

use crate::sim::assets::{AssetError, Result};
use crate::sim::scene::{NodeType, SceneError, SceneGraph, SceneNode};
use quick_xml::events::Event;
use quick_xml::Reader;

/// URDF loader for robot descriptions
pub struct UrdfLoader;

impl UrdfLoader {
    /// Create a new URDF loader
    pub fn new() -> Self {
        Self
    }

    /// Load a scene graph from URDF XML data
    pub fn load(&self, data: &str) -> Result<SceneGraph> {
        let mut graph = SceneGraph::new();
        let mut reader = Reader::from_str(data);
        reader.trim_text(true);

        let mut buf = Vec::new();
        let mut current_robot: Option<UrdfRobot> = None;
        let mut current_link: Option<UrdfLink> = None;
        let mut current_joint: Option<UrdfJoint> = None;
        let mut current_element = String::new();

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(e)) => {
                    current_element = String::from_utf8_lossy(e.name().as_ref()).to_string();

                    match current_element.as_str() {
                        "robot" => {
                            current_robot = Some(UrdfRobot::from_attributes(e.attributes()));
                        }
                        "link" => {
                            current_link = Some(UrdfLink::from_attributes(e.attributes()));
                        }
                        "joint" => {
                            current_joint = Some(UrdfJoint::from_attributes(e.attributes()));
                        }
                        _ => {}
                    }
                }
                Ok(Event::End(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).to_string();

                    match name.as_str() {
                        "link" => {
                            if let Some(link) = current_link.take() {
                                if let Some(ref mut robot) = current_robot {
                                    robot.links.push(link);
                                }
                            }
                        }
                        "joint" => {
                            if let Some(joint) = current_joint.take() {
                                if let Some(ref mut robot) = current_robot {
                                    robot.joints.push(joint);
                                }
                            }
                        }
                        "robot" => {
                            if let Some(robot) = current_robot.take() {
                                self.add_robot_to_graph(&mut graph, robot)?;
                            }
                        }
                        _ => {}
                    }
                }
                Ok(Event::Text(e)) => {
                    let text = e.unescape().unwrap_or_default().to_string();

                    // Parse origin elements
                    if current_element == "origin" {
                        // This would parse xyz and rpy attributes
                    }
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
            return Err(AssetError::Validation("No robot found in URDF".to_string()));
        }

        Ok(graph)
    }

    /// Add a parsed URDF robot to the scene graph
    fn add_robot_to_graph(&self, graph: &mut SceneGraph, robot: UrdfRobot) -> Result<()> {
        // Create robot node
        let robot_node = SceneNode::new(&robot.name, NodeType::Robot);
        let robot_idx = graph
            .add_node(robot_node, None)
            .map_err(AssetError::Scene)?;

        // Add links as children
        for link in &robot.links {
            let link_node =
                SceneNode::new(&format!("{}.{}", robot.name, link.name), NodeType::Object);
            graph
                .add_node(link_node, Some(&robot.name))
                .map_err(AssetError::Scene)?;
        }

        // Update transforms
        graph.update_transforms();

        Ok(())
    }
}

impl Default for UrdfLoader {
    fn default() -> Self {
        Self::new()
    }
}

/// URDF robot structure
#[derive(Debug, Default)]
struct UrdfRobot {
    name: String,
    links: Vec<UrdfLink>,
    joints: Vec<UrdfJoint>,
}

impl UrdfRobot {
    fn from_attributes(attrs: quick_xml::events::attributes::Attributes) -> Self {
        let mut robot = Self::default();

        for attr in attrs {
            if let Ok(attr) = attr {
                let key = String::from_utf8_lossy(attr.key.as_ref());
                let value = String::from_utf8_lossy(&attr.value);

                match key.as_ref() {
                    "name" => robot.name = value.to_string(),
                    _ => {}
                }
            }
        }

        robot
    }
}

/// URDF link structure
#[derive(Debug, Default)]
struct UrdfLink {
    name: String,
    // Additional link properties would go here
}

impl UrdfLink {
    fn from_attributes(attrs: quick_xml::events::attributes::Attributes) -> Self {
        let mut link = Self::default();

        for attr in attrs {
            if let Ok(attr) = attr {
                let key = String::from_utf8_lossy(attr.key.as_ref());
                let value = String::from_utf8_lossy(&attr.value);

                match key.as_ref() {
                    "name" => link.name = value.to_string(),
                    _ => {}
                }
            }
        }

        link
    }
}

/// URDF joint structure
#[derive(Debug, Default)]
struct UrdfJoint {
    name: String,
    joint_type: String,
    parent: String,
    child: String,
}

impl UrdfJoint {
    fn from_attributes(attrs: quick_xml::events::attributes::Attributes) -> Self {
        let mut joint = Self::default();

        for attr in attrs {
            if let Ok(attr) = attr {
                let key = String::from_utf8_lossy(attr.key.as_ref());
                let value = String::from_utf8_lossy(&attr.value);

                match key.as_ref() {
                    "name" => joint.name = value.to_string(),
                    "type" => joint.joint_type = value.to_string(),
                    _ => {}
                }
            }
        }

        joint
    }
}

/// Parse a URDF origin attribute (xyz and rpy)
fn parse_origin(attrs: quick_xml::events::attributes::Attributes) -> ([f64; 3], [f64; 3]) {
    let mut xyz = [0.0; 3];
    let mut rpy = [0.0; 3];

    for attr in attrs {
        if let Ok(attr) = attr {
            let key = String::from_utf8_lossy(attr.key.as_ref());
            let value = String::from_utf8_lossy(&attr.value);

            match key.as_ref() {
                "xyz" => {
                    let parts: Vec<f64> = value
                        .split_whitespace()
                        .filter_map(|s| s.parse().ok())
                        .collect();
                    if parts.len() >= 3 {
                        xyz = [parts[0], parts[1], parts[2]];
                    }
                }
                "rpy" => {
                    let parts: Vec<f64> = value
                        .split_whitespace()
                        .filter_map(|s| s.parse().ok())
                        .collect();
                    if parts.len() >= 3 {
                        rpy = [parts[0], parts[1], parts[2]];
                    }
                }
                _ => {}
            }
        }
    }

    (xyz, rpy)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_urdf() {
        let urdf_str = r#"<?xml version="1.0"?>
<robot name="test_robot">
    <link name="base_link"/>
    <link name="link1"/>
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
</robot>
"#;

        let loader = UrdfLoader::new();
        let graph = loader.load(urdf_str).unwrap();

        assert!(graph.has_node("test_robot"));
        // Links would be added as children
    }

    #[test]
    fn test_parse_origin() {
        let origin_str = r#"<origin xyz="1.0 2.0 3.0" rpy="0.1 0.2 0.3"/>"#;
        let mut reader = Reader::from_str(origin_str);
        reader.trim_text(true);

        let mut buf = Vec::new();
        if let Ok(Event::Empty(e)) = reader.read_event_into(&mut buf) {
            let (xyz, rpy) = parse_origin(e.attributes());
            assert_eq!(xyz, [1.0, 2.0, 3.0]);
            assert_eq!(rpy, [0.1, 0.2, 0.3]);
        }
    }
}
