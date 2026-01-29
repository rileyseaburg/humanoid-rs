//! GLTF mesh loading support
//!
//! Provides support for loading GLTF/GLB format 3D models.
//! This is a stub implementation - full GLTF support would require
//! the bevy_gltf crate or a dedicated GLTF parsing library.
//!
//! For now, this module provides:
//! - GLTF format detection
//! - Placeholder loading that falls back to procedural generation
//! - Type definitions for future GLTF integration

use crate::sim::assets::{AssetError, Result};
use crate::sim::scene::{NodeType, SceneGraph, SceneNode};
use bevy::prelude::*;

/// GLTF loader for 3D model files
pub struct GltfLoader;

impl GltfLoader {
    /// Create a new GLTF loader
    pub fn new() -> Self {
        Self
    }

    /// Load a scene graph from GLTF/GLB data
    ///
    /// # Note
    /// This is currently a stub that returns an empty scene graph.
    /// Full GLTF support requires additional dependencies like bevy_gltf.
    pub fn load(&self, _data: &[u8]) -> Result<SceneGraph> {
        // GLTF loading would require the bevy_gltf crate
        // For now, return an empty scene graph with a placeholder node
        let mut graph = SceneGraph::new();

        // Add a placeholder node indicating GLTF support is stubbed
        let node = SceneNode::new("gltf_placeholder", NodeType::Object)
            .with_property("format", "GLTF")
            .with_property("status", "stub_implementation")
            .with_property("note", "Full GLTF support requires bevy_gltf crate");

        graph
            .add_node(node, None)
            .map_err(|e| AssetError::Scene(e))?;

        graph.update_transforms();

        Ok(graph)
    }

    /// Check if data is a GLB file (binary GLTF)
    pub fn is_glb(data: &[u8]) -> bool {
        // GLB magic number: 0x46546C67 ("glTF" in ASCII)
        data.len() >= 4 && data[0..4] == [0x67, 0x6C, 0x54, 0x46]
    }

    /// Check if data is a GLTF JSON file
    pub fn is_gltf_json(data: &[u8]) -> bool {
        // Check for JSON opening brace
        if data.is_empty() {
            return false;
        }

        // Skip whitespace
        let start = data
            .iter()
            .position(|&b| !b.is_ascii_whitespace())
            .unwrap_or(0);

        data.get(start) == Some(&b'{')
    }
}

impl Default for GltfLoader {
    fn default() -> Self {
        Self::new()
    }
}

/// GLTF asset handle
#[derive(Debug, Clone)]
pub struct GltfAsset {
    /// Path to the GLTF file
    pub path: String,
    /// Whether this is a binary GLB file
    pub is_binary: bool,
    /// Meshes in the file
    pub meshes: Vec<GltfMesh>,
    /// Materials
    pub materials: Vec<GltfMaterial>,
}

/// GLTF mesh data
#[derive(Debug, Clone)]
pub struct GltfMesh {
    /// Mesh name
    pub name: String,
    /// Vertices
    pub vertices: Vec<Vec3>,
    /// Indices
    pub indices: Vec<u32>,
    /// Normals
    pub normals: Vec<Vec3>,
    /// UV coordinates
    pub uvs: Vec<Vec2>,
}

/// GLTF material data
#[derive(Debug, Clone)]
pub struct GltfMaterial {
    /// Material name
    pub name: String,
    /// Base color
    pub base_color: Color,
    /// Metallic factor
    pub metallic: f32,
    /// Roughness factor
    pub roughness: f32,
    /// Base color texture path (if any)
    pub base_color_texture: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gltf_loader_stub() {
        let loader = GltfLoader::new();
        let graph = loader.load(b"{}").unwrap();

        // Should have root + placeholder node
        assert_eq!(graph.len(), 2);
        assert!(graph.has_node("gltf_placeholder"));
    }

    #[test]
    fn test_glb_detection() {
        // GLB magic bytes: "glTF" = 0x67 0x6C 0x54 0x46
        let glb_data = vec![0x67, 0x6C, 0x54, 0x46, 0x00, 0x00];
        assert!(GltfLoader::is_glb(&glb_data));

        // Regular JSON data
        let json_data = b"{ \"test\": true }";
        assert!(!GltfLoader::is_glb(json_data));
    }

    #[test]
    fn test_gltf_json_detection() {
        let json_data = b"{ \"asset\": { \"version\": \"2.0\" } }";
        assert!(GltfLoader::is_gltf_json(json_data));

        let not_json = b"not json";
        assert!(!GltfLoader::is_gltf_json(not_json));
    }
}
