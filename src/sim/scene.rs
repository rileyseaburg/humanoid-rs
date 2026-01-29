//! Scene Graph for hierarchical 3D scene management
//!
//! Provides a hierarchical node structure for organizing entities in the simulation,
//! including robots, terrain, and obstacles. Supports named entity lookup and
//! transform hierarchy.

#![cfg(feature = "simulator")]

use bevy::prelude::*;
use std::collections::HashMap;
use thiserror::Error;

/// Errors that can occur in scene operations
#[derive(Error, Debug)]
pub enum SceneError {
    #[error("Node not found: {0}")]
    NodeNotFound(String),
    #[error("Duplicate node name: {0}")]
    DuplicateName(String),
    #[error("Invalid transform: {0}")]
    InvalidTransform(String),
    #[error("Cycle detected in hierarchy")]
    HierarchyCycle,
    #[error("Node has no parent")]
    NoParent,
}

/// Type of node in the scene graph
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeType {
    /// Root node of the scene
    Root,
    /// Robot entity
    Robot,
    /// Terrain/ground plane
    Terrain,
    /// Static obstacle
    Obstacle,
    /// Sensor or camera
    Sensor,
    /// Generic object
    Object,
}

/// A node in the scene graph
#[derive(Debug, Clone)]
pub struct SceneNode {
    /// Unique name for lookup
    pub name: String,
    /// Node type
    pub node_type: NodeType,
    /// Local transform relative to parent
    pub local_transform: Transform,
    /// World transform (computed)
    pub world_transform: Transform,
    /// Entity handle (if spawned in Bevy)
    pub entity: Option<Entity>,
    /// Parent node index (None for root)
    pub parent: Option<usize>,
    /// Child node indices
    pub children: Vec<usize>,
    /// Custom properties as key-value pairs
    pub properties: HashMap<String, String>,
}

impl SceneNode {
    /// Create a new scene node
    pub fn new(name: impl Into<String>, node_type: NodeType) -> Self {
        let name = name.into();
        Self {
            name,
            node_type,
            local_transform: Transform::IDENTITY,
            world_transform: Transform::IDENTITY,
            entity: None,
            parent: None,
            children: Vec::new(),
            properties: HashMap::new(),
        }
    }

    /// Set the local transform
    pub fn with_transform(mut self, transform: Transform) -> Self {
        self.local_transform = transform;
        self
    }

    /// Set the position
    pub fn with_position(mut self, position: Vec3) -> Self {
        self.local_transform.translation = position;
        self
    }

    /// Set the rotation
    pub fn with_rotation(mut self, rotation: Quat) -> Self {
        self.local_transform.rotation = rotation;
        self
    }

    /// Add a custom property
    pub fn with_property(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.properties.insert(key.into(), value.into());
        self
    }

    /// Get a property value
    pub fn get_property(&self, key: &str) -> Option<&String> {
        self.properties.get(key)
    }
}

/// Hierarchical scene graph for organizing simulation entities
#[derive(Debug, Clone, Default)]
pub struct SceneGraph {
    /// All nodes in the scene
    nodes: Vec<SceneNode>,
    /// Map from name to node index for fast lookup
    name_to_index: HashMap<String, usize>,
    /// Root node index (always 0)
    root_index: usize,
}

impl SceneGraph {
    /// Create a new empty scene graph
    pub fn new() -> Self {
        let mut graph = Self {
            nodes: Vec::new(),
            name_to_index: HashMap::new(),
            root_index: 0,
        };

        // Create root node
        let root = SceneNode::new("root", NodeType::Root);
        graph.nodes.push(root);
        graph.name_to_index.insert("root".to_string(), 0);

        graph
    }

    /// Add a node to the scene graph
    pub fn add_node(
        &mut self,
        mut node: SceneNode,
        parent_name: Option<&str>,
    ) -> Result<usize, SceneError> {
        // Check for duplicate names
        if self.name_to_index.contains_key(&node.name) {
            return Err(SceneError::DuplicateName(node.name.clone()));
        }

        // Determine parent
        let parent_idx = match parent_name {
            Some(name) => *self
                .name_to_index
                .get(name)
                .ok_or_else(|| SceneError::NodeNotFound(name.to_string()))?,
            None => self.root_index,
        };

        // Set parent
        node.parent = Some(parent_idx);

        // Add to nodes vector
        let index = self.nodes.len();
        self.nodes.push(node);
        self.name_to_index
            .insert(self.nodes[index].name.clone(), index);

        // Add as child to parent
        self.nodes[parent_idx].children.push(index);

        Ok(index)
    }

    /// Get a node by index
    pub fn get_node(&self, index: usize) -> Option<&SceneNode> {
        self.nodes.get(index)
    }

    /// Get a mutable node by index
    pub fn get_node_mut(&mut self, index: usize) -> Option<&mut SceneNode> {
        self.nodes.get_mut(index)
    }

    /// Get a node by name
    pub fn get_node_by_name(&self, name: &str) -> Option<&SceneNode> {
        self.name_to_index
            .get(name)
            .and_then(|&idx| self.nodes.get(idx))
    }

    /// Get a mutable node by name
    pub fn get_node_by_name_mut(&mut self, name: &str) -> Option<&mut SceneNode> {
        if let Some(&idx) = self.name_to_index.get(name) {
            self.nodes.get_mut(idx)
        } else {
            None
        }
    }

    /// Remove a node and all its children
    pub fn remove_node(&mut self, name: &str) -> Result<(), SceneError> {
        let index = *self
            .name_to_index
            .get(name)
            .ok_or_else(|| SceneError::NodeNotFound(name.to_string()))?;

        // Cannot remove root
        if index == self.root_index {
            return Err(SceneError::InvalidTransform(
                "Cannot remove root node".to_string(),
            ));
        }

        // Collect all descendants
        let mut to_remove = vec![index];
        let mut i = 0;
        while i < to_remove.len() {
            let current = to_remove[i];
            to_remove.extend(&self.nodes[current].children);
            i += 1;
        }

        // Remove from parent's children list
        if let Some(parent_idx) = self.nodes[index].parent {
            self.nodes[parent_idx].children.retain(|&c| c != index);
        }

        // Remove from name map and nodes (in reverse order to maintain indices)
        to_remove.sort_by(|a, b| b.cmp(a));
        for idx in to_remove {
            let name = self.nodes[idx].name.clone();
            self.name_to_index.remove(&name);
            self.nodes.remove(idx);
        }

        // Rebuild indices (simple but slow approach)
        self.rebuild_indices();

        Ok(())
    }

    /// Rebuild name-to-index mapping after removal
    fn rebuild_indices(&mut self) {
        self.name_to_index.clear();
        for (idx, node) in self.nodes.iter().enumerate() {
            self.name_to_index.insert(node.name.clone(), idx);
        }
    }

    /// Update world transforms for all nodes
    pub fn update_transforms(&mut self) {
        self.update_transform_recursive(self.root_index, Transform::IDENTITY);
    }

    /// Recursively update transforms
    fn update_transform_recursive(&mut self, index: usize, parent_world: Transform) {
        if index >= self.nodes.len() {
            return;
        }

        // Compute world transform
        let local = self.nodes[index].local_transform;
        let world = parent_world * local;
        self.nodes[index].world_transform = world;

        // Update children
        let children = self.nodes[index].children.clone();
        for child_idx in children {
            self.update_transform_recursive(child_idx, world);
        }
    }

    /// Get all nodes of a specific type
    pub fn get_nodes_by_type(&self, node_type: NodeType) -> Vec<&SceneNode> {
        self.nodes
            .iter()
            .filter(|n| n.node_type == node_type)
            .collect()
    }

    /// Get all robot nodes
    pub fn get_robots(&self) -> Vec<&SceneNode> {
        self.get_nodes_by_type(NodeType::Robot)
    }

    /// Get all terrain nodes
    pub fn get_terrain(&self) -> Vec<&SceneNode> {
        self.get_nodes_by_type(NodeType::Terrain)
    }

    /// Get all obstacle nodes
    pub fn get_obstacles(&self) -> Vec<&SceneNode> {
        self.get_nodes_by_type(NodeType::Obstacle)
    }

    /// Get the root node
    pub fn root(&self) -> &SceneNode {
        &self.nodes[self.root_index]
    }

    /// Get all node names
    pub fn get_all_names(&self) -> Vec<&String> {
        self.name_to_index.keys().collect()
    }

    /// Check if a name exists
    pub fn has_node(&self, name: &str) -> bool {
        self.name_to_index.contains_key(name)
    }

    /// Get node count
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// Check if empty (only root)
    pub fn is_empty(&self) -> bool {
        self.nodes.len() <= 1
    }

    /// Iterate over all nodes
    pub fn iter(&self) -> impl Iterator<Item = &SceneNode> {
        self.nodes.iter()
    }

    /// Iterate over all nodes mutably
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut SceneNode> {
        self.nodes.iter_mut()
    }
}

/// Component marking an entity as part of a scene
#[derive(Component, Debug, Clone)]
pub struct SceneMember {
    pub scene_name: String,
    pub node_name: String,
}

/// Resource holding the active scene graph
#[derive(Resource, Debug, Clone, Default)]
pub struct ActiveScene {
    pub graph: SceneGraph,
    pub scene_path: Option<std::path::PathBuf>,
}

impl ActiveScene {
    /// Create a new active scene
    pub fn new() -> Self {
        Self {
            graph: SceneGraph::new(),
            scene_path: None,
        }
    }

    /// Load a scene graph
    pub fn load(&mut self, graph: SceneGraph, path: Option<std::path::PathBuf>) {
        self.graph = graph;
        self.scene_path = path;
    }

    /// Clear the scene
    pub fn clear(&mut self) {
        self.graph = SceneGraph::new();
        self.scene_path = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scene_graph_creation() {
        let graph = SceneGraph::new();
        assert_eq!(graph.len(), 1); // Just root
        assert!(graph.has_node("root"));
    }

    #[test]
    fn test_add_node() {
        let mut graph = SceneGraph::new();
        let node =
            SceneNode::new("robot1", NodeType::Robot).with_position(Vec3::new(1.0, 2.0, 3.0));

        let idx = graph.add_node(node, None).unwrap();
        assert_eq!(idx, 1);
        assert!(graph.has_node("robot1"));
    }

    #[test]
    fn test_duplicate_name_error() {
        let mut graph = SceneGraph::new();
        let node1 = SceneNode::new("robot1", NodeType::Robot);
        let node2 = SceneNode::new("robot1", NodeType::Robot);

        graph.add_node(node1, None).unwrap();
        assert!(matches!(
            graph.add_node(node2, None),
            Err(SceneError::DuplicateName(_))
        ));
    }

    #[test]
    fn test_hierarchy() {
        let mut graph = SceneGraph::new();

        let parent = SceneNode::new("parent", NodeType::Object);
        let parent_idx = graph.add_node(parent, None).unwrap();

        let child = SceneNode::new("child", NodeType::Object);
        let child_idx = graph.add_node(child, Some("parent")).unwrap();

        assert_eq!(graph.get_node(child_idx).unwrap().parent, Some(parent_idx));
        assert!(graph
            .get_node(parent_idx)
            .unwrap()
            .children
            .contains(&child_idx));
    }

    #[test]
    fn test_transform_update() {
        let mut graph = SceneGraph::new();

        let parent =
            SceneNode::new("parent", NodeType::Object).with_position(Vec3::new(1.0, 0.0, 0.0));
        graph.add_node(parent, None).unwrap();

        let child =
            SceneNode::new("child", NodeType::Object).with_position(Vec3::new(0.0, 1.0, 0.0));
        graph.add_node(child, Some("parent")).unwrap();

        graph.update_transforms();

        let child_world = graph.get_node_by_name("child").unwrap().world_transform;
        assert_eq!(child_world.translation, Vec3::new(1.0, 1.0, 0.0));
    }

    #[test]
    fn test_node_lookup() {
        let mut graph = SceneGraph::new();
        let node = SceneNode::new("test_robot", NodeType::Robot);
        graph.add_node(node, None).unwrap();

        assert!(graph.get_node_by_name("test_robot").is_some());
        assert!(graph.get_node_by_name("nonexistent").is_none());
    }

    #[test]
    fn test_remove_node() {
        let mut graph = SceneGraph::new();
        let node = SceneNode::new("to_remove", NodeType::Robot);
        graph.add_node(node, None).unwrap();

        assert!(graph.has_node("to_remove"));
        graph.remove_node("to_remove").unwrap();
        assert!(!graph.has_node("to_remove"));
    }
}
