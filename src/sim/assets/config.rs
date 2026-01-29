//! Scene configuration types for TOML/JSON serialization
//!
//! Defines the schema for scene files that specify robots, terrain,
//! obstacles, and simulation parameters.

use serde::{Deserialize, Serialize};

/// Complete scene configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SceneConfig {
    /// Simulation parameters
    #[serde(default)]
    pub simulation: SimulationConfig,

    /// Robots in the scene
    #[serde(default)]
    pub robots: Vec<RobotConfig>,

    /// Terrain/ground configuration
    #[serde(default)]
    pub terrain: Vec<TerrainConfig>,

    /// Obstacles in the scene
    #[serde(default)]
    pub obstacles: Vec<ObstacleConfig>,

    /// Lighting configuration
    #[serde(default)]
    pub lighting: LightingConfig,
}

impl SceneConfig {
    /// Validate the configuration
    pub fn validate(&self) -> Result<(), String> {
        // Validate simulation config
        self.simulation.validate()?;

        // Validate robots have unique names
        let mut robot_names = std::collections::HashSet::new();
        for robot in &self.robots {
            if !robot_names.insert(&robot.name) {
                return Err(format!("Duplicate robot name: {}", robot.name));
            }
            robot.validate()?;
        }

        // Validate terrain
        for (i, terrain) in self.terrain.iter().enumerate() {
            terrain
                .validate()
                .map_err(|e| format!("Terrain[{}]: {}", i, e))?;
        }

        // Validate obstacles have unique names
        let mut obstacle_names = std::collections::HashSet::new();
        for obstacle in &self.obstacles {
            if let Some(ref name) = obstacle.name {
                if !obstacle_names.insert(name) {
                    return Err(format!("Duplicate obstacle name: {}", name));
                }
            }
            obstacle.validate().map_err(|e| {
                format!(
                    "Obstacle '{}': {}",
                    obstacle.name.as_deref().unwrap_or("unnamed"),
                    e
                )
            })?;
        }

        Ok(())
    }

    /// Load from TOML string
    pub fn from_toml(toml_str: &str) -> Result<Self, toml::de::Error> {
        toml::from_str(toml_str)
    }

    /// Load from JSON string
    pub fn from_json(json_str: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json_str)
    }

    /// Serialize to TOML string
    pub fn to_toml(&self) -> Result<String, toml::ser::Error> {
        toml::to_string_pretty(self)
    }

    /// Serialize to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}

/// Simulation configuration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationConfig {
    /// Physics timestep in seconds
    #[serde(default = "default_timestep")]
    pub timestep: f64,

    /// Gravity vector [x, y, z]
    #[serde(default = "default_gravity")]
    pub gravity: [f64; 3],

    /// Ground friction coefficient
    #[serde(default = "default_friction")]
    pub friction: f64,

    /// Simulation duration (0 = infinite)
    #[serde(default)]
    pub duration: f64,

    /// Random seed for reproducibility (0 = random)
    #[serde(default)]
    pub seed: u64,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            timestep: default_timestep(),
            gravity: default_gravity(),
            friction: default_friction(),
            duration: 0.0,
            seed: 0,
        }
    }
}

impl SimulationConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.timestep <= 0.0 {
            return Err("Timestep must be positive".to_string());
        }
        if self.timestep > 0.1 {
            return Err("Timestep too large (max 0.1s)".to_string());
        }
        if self.friction < 0.0 {
            return Err("Friction cannot be negative".to_string());
        }
        Ok(())
    }
}

fn default_timestep() -> f64 {
    0.001 // 1ms = 1000 Hz
}

fn default_gravity() -> [f64; 3] {
    [0.0, -9.81, 0.0]
}

fn default_friction() -> f64 {
    0.8
}

/// Robot configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotConfig {
    /// Unique name for the robot
    pub name: String,

    /// Robot model identifier
    #[serde(default)]
    pub model: String,

    /// Position [x, y, z] in world coordinates
    #[serde(default = "default_position")]
    pub position: [f64; 3],

    /// Orientation as Euler angles [roll, pitch, yaw] in radians
    #[serde(default)]
    pub orientation: [f64; 3],

    /// Initial joint positions (if applicable)
    #[serde(default)]
    pub initial_joints: Vec<f64>,

    /// Control mode
    #[serde(default)]
    pub control_mode: ControlModeConfig,

    /// Path to URDF/SDF file (optional, uses model if not specified)
    #[serde(default)]
    pub model_path: Option<String>,

    /// Scale factor for the model
    #[serde(default = "default_scale")]
    pub scale: f64,

    /// Robot-specific properties
    #[serde(default)]
    pub properties: std::collections::HashMap<String, serde_json::Value>,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            name: String::new(),
            model: String::new(),
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0],
            initial_joints: Vec::new(),
            control_mode: ControlModeConfig::default(),
            model_path: None,
            scale: 1.0,
            properties: std::collections::HashMap::new(),
        }
    }
}

impl RobotConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.name.is_empty() {
            return Err("Robot name cannot be empty".to_string());
        }
        if self.scale <= 0.0 {
            return Err("Scale must be positive".to_string());
        }
        Ok(())
    }
}

fn default_position() -> [f64; 3] {
    [0.0, 0.0, 0.0]
}

fn default_scale() -> f64 {
    1.0
}

/// Control mode configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum ControlModeConfig {
    #[default]
    Position,
    Velocity,
    Torque,
    Passive,
}

/// Terrain configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerrainConfig {
    /// Terrain type
    #[serde(rename = "type")]
    pub terrain_type: TerrainType,

    /// Normal vector for planes [x, y, z]
    #[serde(default = "default_normal")]
    pub normal: [f64; 3],

    /// Height offset
    #[serde(default)]
    pub height: f64,

    /// Friction coefficient
    #[serde(default)]
    pub friction: Option<f64>,

    /// Restitution (bounciness)
    #[serde(default)]
    pub restitution: Option<f64>,

    /// Size for finite planes [width, depth]
    #[serde(default)]
    pub size: Option<[f64; 2]>,

    /// Path to heightmap image (for heightmap terrain)
    #[serde(default)]
    pub heightmap_path: Option<String>,

    /// Heightmap scale (max height difference)
    #[serde(default)]
    pub heightmap_scale: f64,

    /// Resolution for heightmap
    #[serde(default)]
    pub resolution: [usize; 2],
}

impl TerrainConfig {
    pub fn validate(&self) -> Result<(), String> {
        if let Some(friction) = self.friction {
            if friction < 0.0 {
                return Err("Friction cannot be negative".to_string());
            }
        }
        if let Some(restitution) = self.restitution {
            if restitution < 0.0 || restitution > 1.0 {
                return Err("Restitution must be in [0, 1]".to_string());
            }
        }

        if self.terrain_type == TerrainType::Heightmap {
            if self.heightmap_path.is_none() {
                return Err("Heightmap terrain requires heightmap_path".to_string());
            }
            if self.heightmap_scale <= 0.0 {
                return Err("Heightmap scale must be positive".to_string());
            }
            if self.resolution[0] == 0 || self.resolution[1] == 0 {
                return Err("Heightmap resolution must be non-zero".to_string());
            }
        }

        Ok(())
    }
}

impl Default for TerrainConfig {
    fn default() -> Self {
        Self {
            terrain_type: TerrainType::Plane,
            normal: default_normal(),
            height: 0.0,
            friction: None,
            restitution: None,
            size: None,
            heightmap_path: None,
            heightmap_scale: 1.0,
            resolution: [100, 100],
        }
    }
}

fn default_normal() -> [f64; 3] {
    [0.0, 1.0, 0.0]
}

/// Terrain type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TerrainType {
    /// Infinite plane
    Plane,
    /// Finite plane with size
    FinitePlane,
    /// Heightmap terrain
    Heightmap,
    /// Stairs
    Stairs,
    /// Slope/ramp
    Slope,
}

/// Obstacle configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObstacleConfig {
    /// Optional name for the obstacle
    #[serde(default)]
    pub name: Option<String>,

    /// Obstacle type
    #[serde(rename = "type")]
    pub obstacle_type: ObstacleType,

    /// Position [x, y, z]
    #[serde(default = "default_position")]
    pub position: [f64; 3],

    /// Orientation as Euler angles [roll, pitch, yaw]
    #[serde(default)]
    pub orientation: [f64; 3],

    /// Size parameters (meaning depends on type)
    #[serde(default)]
    pub size: Vec<f64>,

    /// Radius (for sphere, cylinder, capsule)
    #[serde(default)]
    pub radius: Option<f64>,

    /// Height (for cylinder, capsule)
    #[serde(default)]
    pub height: Option<f64>,

    /// Path to mesh file (for mesh type)
    #[serde(default)]
    pub mesh_path: Option<String>,

    /// Color [r, g, b] (0-1)
    #[serde(default = "default_color")]
    pub color: [f64; 3],

    /// Is this obstacle static (immovable)?
    #[serde(default = "default_true")]
    pub static_: bool,

    /// Mass (if not static)
    #[serde(default)]
    pub mass: Option<f64>,

    /// Friction
    #[serde(default)]
    pub friction: Option<f64>,
}

impl ObstacleConfig {
    pub fn validate(&self) -> Result<(), String> {
        match self.obstacle_type {
            ObstacleType::Box => {
                if self.size.len() < 3 {
                    return Err("Box obstacle requires size [x, y, z]".to_string());
                }
                if self.size.iter().any(|&s| s <= 0.0) {
                    return Err("Box dimensions must be positive".to_string());
                }
            }
            ObstacleType::Sphere => {
                if self.radius.is_none() || self.radius.unwrap() <= 0.0 {
                    return Err("Sphere obstacle requires positive radius".to_string());
                }
            }
            ObstacleType::Cylinder | ObstacleType::Capsule => {
                if self.radius.is_none() || self.radius.unwrap() <= 0.0 {
                    return Err("Cylinder/Capsule requires positive radius".to_string());
                }
                if self.height.is_none() || self.height.unwrap() <= 0.0 {
                    return Err("Cylinder/Capsule requires positive height".to_string());
                }
            }
            ObstacleType::Mesh => {
                if self.mesh_path.is_none() {
                    return Err("Mesh obstacle requires mesh_path".to_string());
                }
            }
        }

        if !self.static_ {
            if self.mass.is_none() || self.mass.unwrap() <= 0.0 {
                return Err("Non-static obstacles require positive mass".to_string());
            }
        }

        Ok(())
    }
}

impl Default for ObstacleConfig {
    fn default() -> Self {
        Self {
            name: None,
            obstacle_type: ObstacleType::Box,
            position: default_position(),
            orientation: [0.0; 3],
            size: vec![1.0, 1.0, 1.0],
            radius: None,
            height: None,
            mesh_path: None,
            color: default_color(),
            static_: true,
            mass: None,
            friction: None,
        }
    }
}

fn default_color() -> [f64; 3] {
    [0.5, 0.5, 0.5]
}

fn default_true() -> bool {
    true
}

/// Obstacle shape types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ObstacleType {
    Box,
    Sphere,
    Cylinder,
    Capsule,
    Mesh,
}

/// Lighting configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LightingConfig {
    /// Ambient light intensity
    #[serde(default = "default_ambient")]
    pub ambient: f64,

    /// Directional lights
    #[serde(default)]
    pub directional: Vec<DirectionalLightConfig>,

    /// Point lights
    #[serde(default)]
    pub point: Vec<PointLightConfig>,
}

impl LightingConfig {
    pub fn validate(&self) -> Result<(), String> {
        for (i, light) in self.directional.iter().enumerate() {
            light
                .validate()
                .map_err(|e| format!("DirectionalLight[{}]: {}", i, e))?;
        }
        for (i, light) in self.point.iter().enumerate() {
            light
                .validate()
                .map_err(|e| format!("PointLight[{}]: {}", i, e))?;
        }
        Ok(())
    }
}

fn default_ambient() -> f64 {
    0.3
}

/// Directional light configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DirectionalLightConfig {
    /// Direction vector [x, y, z]
    #[serde(default = "default_light_direction")]
    pub direction: [f64; 3],

    /// Light intensity
    #[serde(default = "default_intensity")]
    pub intensity: f64,

    /// Light color [r, g, b]
    #[serde(default = "default_white")]
    pub color: [f64; 3],

    /// Cast shadows
    #[serde(default = "default_true")]
    pub shadows: bool,
}

impl DirectionalLightConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.intensity <= 0.0 {
            return Err("Light intensity must be positive".to_string());
        }
        if self.color.iter().any(|&c| c < 0.0 || c > 1.0) {
            return Err("Color components must be in [0, 1]".to_string());
        }
        Ok(())
    }
}

impl Default for DirectionalLightConfig {
    fn default() -> Self {
        Self {
            direction: default_light_direction(),
            intensity: default_intensity(),
            color: default_white(),
            shadows: true,
        }
    }
}

fn default_light_direction() -> [f64; 3] {
    [-0.5, -1.0, -0.3]
}

fn default_intensity() -> f64 {
    1.0
}

fn default_white() -> [f64; 3] {
    [1.0, 1.0, 1.0]
}

/// Point light configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointLightConfig {
    /// Position [x, y, z]
    #[serde(default)]
    pub position: [f64; 3],

    /// Light intensity
    #[serde(default = "default_intensity")]
    pub intensity: f64,

    /// Light color [r, g, b]
    #[serde(default = "default_white")]
    pub color: [f64; 3],

    /// Attenuation radius
    #[serde(default = "default_radius")]
    pub radius: f64,
}

impl PointLightConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.intensity <= 0.0 {
            return Err("Light intensity must be positive".to_string());
        }
        if self.radius <= 0.0 {
            return Err("Light radius must be positive".to_string());
        }
        if self.color.iter().any(|&c| c < 0.0 || c > 1.0) {
            return Err("Color components must be in [0, 1]".to_string());
        }
        Ok(())
    }
}

impl Default for PointLightConfig {
    fn default() -> Self {
        Self {
            position: default_position(),
            intensity: default_intensity(),
            color: default_white(),
            radius: default_radius(),
        }
    }
}

fn default_radius() -> f64 {
    10.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scene_config_defaults() {
        let config = SceneConfig::default();
        assert_eq!(config.simulation.timestep, 0.001);
        assert!(config.robots.is_empty());
        assert!(config.terrain.is_empty());
        assert!(config.obstacles.is_empty());
    }

    #[test]
    fn test_toml_roundtrip() {
        let config = SceneConfig {
            simulation: SimulationConfig {
                timestep: 0.002,
                gravity: [0.0, -9.81, 0.0],
                friction: 0.9,
                duration: 10.0,
                seed: 42,
            },
            robots: vec![RobotConfig {
                name: "test_robot".to_string(),
                model: "unitree_g1".to_string(),
                position: [0.0, 0.85, 0.0],
                orientation: [0.0, 0.0, 0.0],
                initial_joints: vec![],
                control_mode: ControlModeConfig::Position,
                model_path: None,
                scale: 1.0,
                properties: std::collections::HashMap::new(),
            }],
            terrain: vec![TerrainConfig {
                terrain_type: TerrainType::Plane,
                normal: [0.0, 1.0, 0.0],
                height: 0.0,
                friction: Some(0.8),
                restitution: None,
                size: None,
                heightmap_path: None,
                heightmap_scale: 1.0,
                resolution: [100, 100],
            }],
            obstacles: vec![ObstacleConfig {
                name: Some("box_1".to_string()),
                obstacle_type: ObstacleType::Box,
                position: [2.0, 0.25, 0.0],
                orientation: [0.0, 0.0, 0.0],
                size: vec![0.5, 0.5, 0.5],
                radius: None,
                height: None,
                mesh_path: None,
                color: [0.8, 0.3, 0.3],
                static_: true,
                mass: None,
                friction: None,
            }],
            lighting: LightingConfig::default(),
        };

        let toml_str = config.to_toml().unwrap();
        let parsed = SceneConfig::from_toml(&toml_str).unwrap();

        assert_eq!(parsed.simulation.timestep, config.simulation.timestep);
        assert_eq!(parsed.robots.len(), 1);
        assert_eq!(parsed.robots[0].name, "test_robot");
    }

    #[test]
    fn test_validation() {
        let config = SceneConfig {
            simulation: SimulationConfig {
                timestep: -0.001, // Invalid
                ..Default::default()
            },
            ..Default::default()
        };

        assert!(config.validate().is_err());

        let valid_config = SceneConfig::default();
        assert!(valid_config.validate().is_ok());
    }

    #[test]
    fn test_duplicate_robot_names() {
        let config = SceneConfig {
            robots: vec![
                RobotConfig {
                    name: "robot".to_string(),
                    ..Default::default()
                },
                RobotConfig {
                    name: "robot".to_string(),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };

        assert!(config.validate().is_err());
    }

    #[test]
    fn test_obstacle_validation() {
        // Valid box
        let box_obstacle = ObstacleConfig {
            name: Some("test_box".to_string()),
            obstacle_type: ObstacleType::Box,
            size: vec![1.0, 1.0, 1.0],
            ..Default::default()
        };
        assert!(box_obstacle.validate().is_ok());

        // Invalid box - missing dimensions
        let invalid_box = ObstacleConfig {
            name: Some("invalid_box".to_string()),
            obstacle_type: ObstacleType::Box,
            size: vec![1.0, 1.0], // Missing z
            ..Default::default()
        };
        assert!(invalid_box.validate().is_err());

        // Valid sphere
        let sphere = ObstacleConfig {
            name: Some("test_sphere".to_string()),
            obstacle_type: ObstacleType::Sphere,
            radius: Some(0.5),
            ..Default::default()
        };
        assert!(sphere.validate().is_ok());

        // Invalid sphere - missing radius
        let invalid_sphere = ObstacleConfig {
            name: Some("invalid_sphere".to_string()),
            obstacle_type: ObstacleType::Sphere,
            ..Default::default()
        };
        assert!(invalid_sphere.validate().is_err());
    }
}
