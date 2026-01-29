# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-01-28

### Added
- Deterministic simulation core with fixed-timestep physics
- Physics backend abstraction supporting multiple engines (Rapier, PhysX, Bullet)
- Scene graph system for hierarchical transforms and asset loading
- Sensor framework with configurable noise models (Gaussian, quantization, dropout)
- Safety guardrails with configurable limits and emergency stop
- ROS2 integration via rclrs for seamless robotics workflows
- SDK public API for external control and monitoring
- Comprehensive benchmarking suite with Criterion
- CI/CD pipeline with GitHub Actions

### Changed
- Refactored control interface for better ergonomics
- Improved error handling with `thiserror` and `anyhow`
- Updated to Bevy 0.13 and bevy_rapier3d 0.26
- Enhanced type safety throughout the codebase

### Fixed
- Memory leaks in physics simulation cleanup
- Race conditions in sensor data access
- Documentation examples now compile correctly

## [0.1.0] - 2025-12-01

### Added
- Initial release with core robotics framework
- Basic simulator using Bevy game engine
- Unitree G1 robot support
- gRPC-based control interface
- Real-time control foundation with RTIC
- Async runtime with Tokio

[Unreleased]: https://github.com/rileyseaburg/humanoid-rs/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/rileyseaburg/humanoid-rs/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/rileyseaburg/humanoid-rs/releases/tag/v0.1.0
