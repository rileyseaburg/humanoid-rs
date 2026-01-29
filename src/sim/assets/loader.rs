//! Scene loading API with support for multiple formats
//!
//! Provides a unified interface for loading scenes from files or strings,
//! with support for URDF, SDF, JSON, and TOML formats.

use super::config::SceneConfig;
use super::sdf::SdfLoader;
use super::urdf::UrdfLoader;
use super::{AssetError, Result};
use crate::sim::scene::{NodeType, SceneError, SceneGraph, SceneNode};
use std::path::{Path, PathBuf};

/// Supported asset formats
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssetFormat {
    /// Universal Robot Description Format
    Urdf,
    /// Simulation Description Format
    Sdf,
    /// JSON configuration
    Json,
    /// TOML configuration
    Toml,
}

impl AssetFormat {
    /// Detect format from file extension
    pub fn from_extension(path: &Path) -> Option<Self> {
        match path.extension()?.to_str()?.to_lowercase().as_str() {
            "urdf" => Some(AssetFormat::Urdf),
            "sdf" => Some(AssetFormat::Sdf),
            "json" => Some(AssetFormat::Json),
            "toml" => Some(AssetFormat::Toml),
            _ => None,
        }
    }

    /// Get file extension for this format
    pub fn extension(&self) -> &'static str {
        match self {
            AssetFormat::Urdf => "urdf",
            AssetFormat::Sdf => "sdf",
            AssetFormat::Json => "json",
            AssetFormat::Toml => "toml",
        }
    }
}

/// Main scene loader with hot-reloading support
pub struct SceneLoader {
    /// Base path for resolving relative paths
    base_path: PathBuf,
    /// Cache of loaded scenes for hot-reloading
    cache: std::collections::HashMap<PathBuf, (SceneGraph, std::time::SystemTime)>,
    /// Enable hot-reloading
    hot_reload: bool,
}

impl SceneLoader {
    /// Create a new scene loader
    pub fn new() -> Self {
        Self {
            base_path: std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")),
            cache: std::collections::HashMap::new(),
            hot_reload: false,
        }
    }

    /// Create a loader with a specific base path
    pub fn with_base_path(base_path: impl AsRef<Path>) -> Self {
        Self {
            base_path: base_path.as_ref().to_path_buf(),
            cache: std::collections::HashMap::new(),
            hot_reload: false,
        }
    }

    /// Enable or disable hot-reloading
    pub fn with_hot_reload(mut self, enabled: bool) -> Self {
        self.hot_reload = enabled;
        self
    }

    /// Load a scene from a file (auto-detects format)
    pub fn load_from_file(&mut self, path: &Path) -> Result<SceneGraph> {
        let format = AssetFormat::from_extension(path).ok_or_else(|| {
            AssetError::UnsupportedFormat(format!("Unknown file extension: {:?}", path.extension()))
        })?;

        // Check cache for hot-reloading
        if self.hot_reload {
            if let Some((cached_graph, cached_time)) = self.cache.get(path) {
                let metadata = std::fs::metadata(path)?;
                let modified = metadata.modified()?;

                if &modified == cached_time {
                    return Ok(cached_graph.clone());
                }
            }
        }

        let full_path = if path.is_absolute() {
            path.to_path_buf()
        } else {
            self.base_path.join(path)
        };

        if !full_path.exists() {
            return Err(AssetError::FileNotFound(full_path.display().to_string()));
        }

        let content = std::fs::read_to_string(&full_path)?;
        let graph = self.load_from_str(format, &content)?;

        // Cache for hot-reloading
        if self.hot_reload {
            let metadata = std::fs::metadata(&full_path)?;
            let modified = metadata.modified()?;
            self.cache
                .insert(path.to_path_buf(), (graph.clone(), modified));
        }

        Ok(graph)
    }

    /// Load a scene from a string
    pub fn load_from_str(&self, format: AssetFormat, data: &str) -> Result<SceneGraph> {
        match format {
            AssetFormat::Toml => Self::load_toml(data),
            AssetFormat::Json => Self::load_json(data),
            AssetFormat::Urdf => Self::load_urdf(data),
            AssetFormat::Sdf => Self::load_sdf(data),
        }
    }

    /// Load from TOML configuration
    fn load_toml(data: &str) -> Result<SceneGraph> {
        let config = SceneConfig::from_toml(data).map_err(AssetError::Toml)?;

        config.validate().map_err(AssetError::Validation)?;

        Self::config_to_graph(&config)
    }

    /// Load from JSON configuration
    fn load_json(data: &str) -> Result<SceneGraph> {
        let config = SceneConfig::from_json(data).map_err(AssetError::Json)?;

        config.validate().map_err(AssetError::Validation)?;

        Self::config_to_graph(&config)
    }

    /// Load from URDF format
    fn load_urdf(data: &str) -> Result<SceneGraph> {
        let loader = UrdfLoader::new();
        loader.load(data)
    }

    /// Load from SDF format
    fn load_sdf(data: &str) -> Result<SceneGraph> {
        let loader = SdfLoader::new();
        loader.load(data)
    }

    /// Convert SceneConfig to SceneGraph
    fn config_to_graph(config: &SceneConfig) -> Result<SceneGraph> {
        use bevy::prelude::*;

        let mut graph = SceneGraph::new();

        // Add robots
        for robot in &config.robots {
            let position = Vec3::new(
                robot.position[0] as f32,
                robot.position[1] as f32,
                robot.position[2] as f32,
            );

            let rotation = Quat::from_euler(
                EulerRot::XYZ,
                robot.orientation[0] as f32,
                robot.orientation[1] as f32,
                robot.orientation[2] as f32,
            );

            let mut node = SceneNode::new(&robot.name, NodeType::Robot)
                .with_position(position)
                .with_rotation(rotation)
                .with_property("model", &robot.model);

            if let Some(ref path) = robot.model_path {
                node = node.with_property("model_path", path);
            }

            node = node.with_property("scale", robot.scale.to_string());
            node = node.with_property("control_mode", format!("{:?}", robot.control_mode));

            graph.add_node(node, None)?;
        }

        // Add terrain
        for (i, terrain) in config.terrain.iter().enumerate() {
            let name = format!("terrain_{}", i);
            let mut node = SceneNode::new(&name, NodeType::Terrain)
                .with_property("type", format!("{:?}", terrain.terrain_type));

            if let Some(friction) = terrain.friction {
                node = node.with_property("friction", friction.to_string());
            }

            if let Some(restitution) = terrain.restitution {
                node = node.with_property("restitution", restitution.to_string());
            }

            if let Some(ref path) = terrain.heightmap_path {
                node = node.with_property("heightmap_path", path);
                node = node.with_property("heightmap_scale", terrain.heightmap_scale.to_string());
            }

            graph.add_node(node, None)?;
        }

        // Add obstacles
        for obstacle in &config.obstacles {
            let name = obstacle
                .name
                .clone()
                .unwrap_or_else(|| format!("obstacle_{}", graph.len()));

            let position = Vec3::new(
                obstacle.position[0] as f32,
                obstacle.position[1] as f32,
                obstacle.position[2] as f32,
            );

            let rotation = Quat::from_euler(
                EulerRot::XYZ,
                obstacle.orientation[0] as f32,
                obstacle.orientation[1] as f32,
                obstacle.orientation[2] as f32,
            );

            let mut node = SceneNode::new(&name, NodeType::Obstacle)
                .with_position(position)
                .with_rotation(rotation)
                .with_property("type", format!("{:?}", obstacle.obstacle_type));

            // Add type-specific properties
            match obstacle.obstacle_type {
                super::config::ObstacleType::Box => {
                    node = node.with_property("size_x", obstacle.size[0].to_string());
                    node = node.with_property("size_y", obstacle.size[1].to_string());
                    node = node.with_property("size_z", obstacle.size[2].to_string());
                }
                super::config::ObstacleType::Sphere => {
                    if let Some(r) = obstacle.radius {
                        node = node.with_property("radius", r.to_string());
                    }
                }
                super::config::ObstacleType::Cylinder | super::config::ObstacleType::Capsule => {
                    if let Some(r) = obstacle.radius {
                        node = node.with_property("radius", r.to_string());
                    }
                    if let Some(h) = obstacle.height {
                        node = node.with_property("height", h.to_string());
                    }
                }
                super::config::ObstacleType::Mesh => {
                    if let Some(ref path) = obstacle.mesh_path {
                        node = node.with_property("mesh_path", path);
                    }
                }
            }

            node = node.with_property("color_r", obstacle.color[0].to_string());
            node = node.with_property("color_g", obstacle.color[1].to_string());
            node = node.with_property("color_b", obstacle.color[2].to_string());
            node = node.with_property("static", obstacle.static_.to_string());

            if let Some(mass) = obstacle.mass {
                node = node.with_property("mass", mass.to_string());
            }

            if let Some(friction) = obstacle.friction {
                node = node.with_property("friction", friction.to_string());
            }

            graph.add_node(node, None)?;
        }

        // Update transforms
        graph.update_transforms();

        Ok(graph)
    }

    /// Check if a cached file has been modified
    pub fn check_hot_reload(&mut self, path: &Path) -> Option<SceneGraph> {
        if !self.hot_reload {
            return None;
        }

        let full_path = if path.is_absolute() {
            path.to_path_buf()
        } else {
            self.base_path.join(path)
        };

        if let Ok(metadata) = std::fs::metadata(&full_path) {
            if let Ok(modified) = metadata.modified() {
                if let Some((_, cached_time)) = self.cache.get(path) {
                    if &modified != cached_time {
                        // File has changed, reload
                        if let Ok(graph) = self.load_from_file(path) {
                            return Some(graph);
                        }
                    }
                }
            }
        }

        None
    }

    /// Clear the cache
    pub fn clear_cache(&mut self) {
        self.cache.clear();
    }

    /// Get the cache size
    pub fn cache_size(&self) -> usize {
        self.cache.len()
    }
}

impl Default for SceneLoader {
    fn default() -> Self {
        Self::new()
    }
}

/// Helper functions for loading scenes
pub mod util {
    use super::*;

    /// Quick load from a TOML file
    pub fn load_toml_file(path: &Path) -> Result<SceneGraph> {
        let mut loader = SceneLoader::new();
        loader.load_from_file(path)
    }

    /// Quick load from a TOML string
    pub fn load_toml_str(data: &str) -> Result<SceneGraph> {
        SceneLoader::load_toml(data)
    }

    /// Quick load from a JSON file
    pub fn load_json_file(path: &Path) -> Result<SceneGraph> {
        let mut loader = SceneLoader::new();
        loader.load_from_file(path)
    }

    /// Quick load from a JSON string
    pub fn load_json_str(data: &str) -> Result<SceneGraph> {
        SceneLoader::load_json(data)
    }

    /// Quick load from a URDF string
    pub fn load_urdf_str(data: &str) -> Result<SceneGraph> {
        SceneLoader::load_urdf(data)
    }

    /// Quick load from an SDF string
    pub fn load_sdf_str(data: &str) -> Result<SceneGraph> {
        SceneLoader::load_sdf(data)
    }

    /// Quick load from a GLTF byte slice
    pub fn load_gltf_bytes(data: &[u8]) -> Result<SceneGraph> {
        SceneLoader::load_gltf(data)
    }
}

// Import the actual loader functions for internal use
use super::config::ObstacleType;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_detection() {
        assert_eq!(
            AssetFormat::from_extension(Path::new("test.urdf")),
            Some(AssetFormat::Urdf)
        );
        assert_eq!(
            AssetFormat::from_extension(Path::new("test.sdf")),
            Some(AssetFormat::Sdf)
        );
        assert_eq!(
            AssetFormat::from_extension(Path::new("test.json")),
            Some(AssetFormat::Json)
        );
        assert_eq!(
            AssetFormat::from_extension(Path::new("test.toml")),
            Some(AssetFormat::Toml)
        );
        assert_eq!(AssetFormat::from_extension(Path::new("test.unknown")), None);
    }

    #[test]
    fn test_load_toml_str() {
        let toml_str = r#"
[simulation]
timestep = 0.001
gravity = [0.0, -9.81, 0.0]

[[robots]]
name = "test_robot"
model = "unitree_g1"
position = [0.0, 0.85, 0.0]

[[terrain]]
type = "plane"
friction = 0.8
"#;

        let graph = util::load_toml_str(toml_str).unwrap();
        assert!(graph.has_node("test_robot"));
        assert!(graph.has_node("terrain_0"));
        assert_eq!(graph.get_robots().len(), 1);
    }

    #[test]
    fn test_load_json_str() {
        let json_str = r#"
        {
            "simulation": {
                "timestep": 0.001,
                "gravity": [0.0, -9.81, 0.0]
            },
            "robots": [
                {
                    "name": "json_robot",
                    "model": "unitree_g1",
                    "position": [0.0, 0.85, 0.0]
                }
            ]
        }
        "#;

        let graph = util::load_json_str(json_str).unwrap();
        assert!(graph.has_node("json_robot"));
    }

    #[test]
    fn test_invalid_config() {
        let invalid_toml = r#"
[simulation]
timestep = -0.001
"#;

        assert!(util::load_toml_str(invalid_toml).is_err());
    }
}
