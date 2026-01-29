//! Asset loading and configuration management
//!
//! Provides support for loading scenes from various formats including
//! TOML, JSON, URDF, and SDF. Includes hot-reloading for development.

pub mod config;
pub mod loader;
pub mod sdf;
pub mod urdf;

pub use config::{ObstacleConfig, RobotConfig, SceneConfig, SimulationConfig, TerrainConfig};
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
