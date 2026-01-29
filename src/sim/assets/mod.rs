//! Asset loading and configuration management
//!
//! Provides support for loading scenes from various formats including
//! TOML, JSON, URDF, SDF, and GLTF. Includes hot-reloading for development.

pub mod config;
pub mod gltf;
pub mod loader;
pub mod sdf;
pub mod urdf;

pub use config::{
    ControlModeConfig, ObstacleConfig, ObstacleType, RobotConfig, SceneConfig, SimulationConfig,
    TerrainConfig, TerrainType,
};
pub use gltf::{GltfAsset, GltfLoader, GltfMaterial, GltfMesh};
pub use loader::{AssetFormat, SceneLoader};

use crate::sim::scene::SceneError;
use thiserror::Error;

/// Errors that can occur during asset loading
#[derive(Error, Debug)]
pub enum AssetError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("TOML parsing error: {0}")]
    Toml(#[from] toml::de::Error),

    #[error("JSON parsing error: {0}")]
    Json(#[from] serde_json::Error),

    #[error("XML parsing error: {0}")]
    Xml(String),

    #[error("Scene error: {0}")]
    Scene(#[from] SceneError),

    #[error("Validation error: {0}")]
    Validation(String),

    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("File not found: {0}")]
    FileNotFound(String),
}

pub type Result<T> = std::result::Result<T, AssetError>;
