# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0-alpha.1] - 2026-01-29

### Added
- **Complete SDK Implementation**
  - Public SDK API with HumanoidSDK interface
  - Robot profiles (Unitree G1, H1) with capabilities system
  - Configuration management with TOML/JSON support
  - Simulation and robot connection handling

- **Simulation Engine**
  - Deterministic simulation core with fixed-timestep physics
  - SimulationClock for reproducible timing control
  - Physics backend abstraction trait (PhysicsBackend)
  - RapierBackend implementation for Rapier3D integration
  - Scene graph system for hierarchical entity management
  - Asset loading pipeline (URDF, SDF, GLTF support)

- **Sensor Framework**
  - Sensor trait with configurable noise models
  - IMU sensor with accelerometer and gyroscope
  - Joint encoder sensors for position/velocity/effort
  - Force/torque sensors for contact detection
  - Camera sensor placeholder for RGB-D
  - Noise models: Gaussian, Random Walk, Quantization, Saturation

- **Safety System**
  - SafetyManager with guard chain pattern
  - JointLimitGuard for position/velocity/effort limits
  - EStopGuard for emergency stop handling
  - CommunicationMonitor for watchdog timeouts
  - TorqueRampGuard for rate limiting
  - Configurable safety levels

- **Control Improvements**
  - Hardened control interface with ControlCommand/ControlState
  - Command buffer with latency handling
  - Controller trait with set_target method
  - Safety integration in control loop

- **Observability**
  - SimulationLogger for state history
  - Debug hooks for stepping and visualization
  - Replay system for deterministic reproduction
  - Export formats: CSV, JSON, binary

- **Documentation**
  - Complete API reference documentation
  - Architecture documentation with diagrams
  - Tutorial series (5 tutorials)
  - Quick start guide
  - Configuration examples

- **Examples**
  - starter.rs - Complete SDK demonstration
  - basic_sim.rs - Minimal simulation example
  - control_loop.rs - Control loop demonstration
  - sensor_streaming.rs - Sensor data access
  - walking_demo.rs - Walking simulation
  - sdk_demo.rs - SDK usage examples
  - ros2_bridge.rs - ROS2 integration example

- **Testing Infrastructure**
  - Deterministic regression tests
  - Control tests for safety scenarios
  - Safety tests for guard activation
  - Integration tests
  - Performance benchmarks (Criterion)
  - Golden file testing

- **CI/CD**
  - GitHub Actions workflows for CI
  - Benchmark tracking
  - Automated releases
  - Issue templates

### Changed
- Refactored control module structure
- Improved error handling with thiserror
- Enhanced type safety throughout
- Split control.rs into modular components

### Fixed
- All compiler warnings and errors
- Import resolution issues
- Type mismatches in sensor framework
- Physics backend compilation issues

## [0.1.0] - 2026-01-28

### Added
- Initial project structure
- Core robotics framework foundation
- Basic simulator with Bevy + Rapier3D
- Unitree G1 robot support
- gRPC interface with tonic
- Async runtime with Tokio
- Basic joint control and kinematics
- MCP (Model Context Protocol) integration structure
- RL (Reinforcement Learning) module structure

[Unreleased]: https://github.com/rileyseaburg/humanoid-rs/compare/v0.2.0-alpha.1...HEAD
[0.2.0-alpha.1]: https://github.com/rileyseaburg/humanoid-rs/compare/v0.1.0...v0.2.0-alpha.1
[0.1.0]: https://github.com/rileyseaburg/humanoid-rs/commit/805e4a75b152eede18a5d4b1119ee865dff0ff1f
