//! Sensor framework with realistic noise models for humanoid robot simulation
//!
//! This module provides a comprehensive sensor simulation system with:
//! - Multiple sensor types (IMU, joint encoders, force/torque, cameras)
//! - Realistic noise models (Gaussian, random walk, quantization, saturation)
//! - Timestamping with configurable processing delays
//! - Sensor registry for centralized management
//!
//! All sensor data includes proper timestamps and follows ROS2 conventions for units.

use nalgebra::{Const, Quaternion, UnitQuaternion, Vector3, Vector6, U1};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Sensor trait that all sensors must implement
pub trait Sensor: Send + Sync {
    /// Update sensor data from physics backend
    fn update(&mut self, physics: &dyn PhysicsBackend, dt: Duration);
    /// Get current sensor data
    fn data(&self) -> SensorData;
    /// Reset sensor to initial state
    fn reset(&mut self);
    /// Set noise model for this sensor
    fn set_noise_model(&mut self, model: NoiseModel);
    /// Get sensor name
    fn name(&self) -> &str;
    /// Get sensor type
    fn sensor_type(&self) -> SensorType;
}

/// Types of sensors supported
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorType {
    Imu,
    JointEncoder,
    ForceTorque,
    Camera,
}

impl std::fmt::Display for SensorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SensorType::Imu => write!(f, "IMU"),
            SensorType::JointEncoder => write!(f, "JointEncoder"),
            SensorType::ForceTorque => write!(f, "ForceTorque"),
            SensorType::Camera => write!(f, "Camera"),
        }
    }
}

/// Unified sensor data enum
#[derive(Debug, Clone)]
pub enum SensorData {
    Imu(ImuData),
    JointEncoder(JointEncoderData),
    ForceTorque(ForceTorqueData),
    Camera(CameraData),
}

/// Timestamped sensor data wrapper
#[derive(Debug, Clone)]
pub struct Timestamped<T> {
    pub data: T,
    pub timestamp: Instant,
    pub acquisition_delay: Duration,
}

impl<T> Timestamped<T> {
    pub fn new(data: T, timestamp: Instant, acquisition_delay: Duration) -> Self {
        Self {
            data,
            timestamp,
            acquisition_delay,
        }
    }

    /// Get the effective measurement time (accounting for processing delay)
    pub fn measurement_time(&self) -> Instant {
        self.timestamp - self.acquisition_delay
    }
}

/// IMU sensor data
#[derive(Debug, Clone)]
pub struct ImuData {
    /// Linear acceleration in body frame (m/s^2)
    pub linear_acceleration: Vector3<f64>,
    /// Angular velocity in body frame (rad/s)
    pub angular_velocity: Vector3<f64>,
    /// Orientation as unit quaternion (world to body)
    pub orientation: UnitQuaternion<f64>,
    /// Temperature (Celsius, optional)
    pub temperature: Option<f64>,
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            linear_acceleration: Vector3::new(0.0, 0.0, -9.81),
            angular_velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            temperature: None,
        }
    }
}

/// Joint encoder data for a single joint
#[derive(Debug, Clone, Copy)]
pub struct SingleJointData {
    /// Joint position (rad)
    pub position: f64,
    /// Joint velocity (rad/s)
    pub velocity: f64,
    /// Joint effort/torque (Nm)
    pub effort: f64,
}

impl Default for SingleJointData {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            effort: 0.0,
        }
    }
}

/// Joint encoder data for all joints
#[derive(Debug, Clone)]
pub struct JointEncoderData {
    /// Joint data indexed by joint ID
    pub joints: Vec<SingleJointData>,
    /// Joint names (optional, for identification)
    pub joint_names: Vec<String>,
}

impl JointEncoderData {
    pub fn new(num_joints: usize) -> Self {
        Self {
            joints: vec![SingleJointData::default(); num_joints],
            joint_names: (0..num_joints).map(|i| format!("joint_{}", i)).collect(),
        }
    }

    pub fn with_names(num_joints: usize, names: Vec<String>) -> Self {
        let mut data = Self::new(num_joints);
        if names.len() == num_joints {
            data.joint_names = names;
        }
        data
    }
}

/// Force/torque sensor data (6-axis)
#[derive(Debug, Clone)]
pub struct ForceTorqueData {
    /// Force vector (N) in sensor frame
    pub force: Vector3<f64>,
    /// Torque vector (Nm) in sensor frame
    pub torque: Vector3<f64>,
    /// Contact state (true if in contact)
    pub in_contact: bool,
    /// Center of pressure (m) relative to sensor origin
    pub center_of_pressure: Option<Vector3<f64>>,
}

impl Default for ForceTorqueData {
    fn default() -> Self {
        Self {
            force: Vector3::zeros(),
            torque: Vector3::zeros(),
            in_contact: false,
            center_of_pressure: None,
        }
    }
}

/// Camera sensor data (RGB-D)
#[derive(Debug, Clone)]
pub struct CameraData {
    /// RGB image data (flattened, row-major)
    pub rgb: Vec<u8>,
    /// Depth image data (meters, flattened, row-major)
    pub depth: Vec<f32>,
    /// Image width
    pub width: u32,
    /// Image height
    pub height: u32,
    /// Camera intrinsics (fx, fy, cx, cy)
    pub intrinsics: [f64; 4],
    /// Camera extrinsics (position, orientation)
    pub extrinsics: (Vector3<f64>, UnitQuaternion<f64>),
}

impl CameraData {
    pub fn new(width: u32, height: u32) -> Self {
        let num_pixels = (width * height) as usize;
        Self {
            rgb: vec![0; num_pixels * 3],
            depth: vec![0.0; num_pixels],
            width,
            height,
            intrinsics: [
                width as f64 * 0.5,
                height as f64 * 0.5,
                width as f64 * 0.5,
                height as f64 * 0.5,
            ],
            extrinsics: (Vector3::zeros(), UnitQuaternion::identity()),
        }
    }
}

/// Noise models for sensor simulation
#[derive(Debug, Clone)]
pub enum NoiseModel {
    /// No noise (ideal sensor)
    None,
    /// Gaussian noise with bias and standard deviation
    Gaussian {
        /// Bias offset
        bias: f64,
        /// Standard deviation (white noise)
        std_dev: f64,
    },
    /// Random walk (bias drift over time)
    RandomWalk {
        /// Initial bias
        bias: f64,
        /// White noise standard deviation
        std_dev: f64,
        /// Random walk rate (bias drift per second)
        walk_rate: f64,
    },
    /// Quantization noise (ADC simulation)
    Quantization {
        /// Resolution (step size)
        resolution: f64,
        /// Bias
        bias: f64,
    },
    /// Saturation with optional noise
    Saturation {
        /// Minimum value
        min: f64,
        /// Maximum value
        max: f64,
        /// Underlying noise model
        inner: Box<NoiseModel>,
    },
    /// Combined noise model
    Combined {
        /// Gaussian component
        gaussian: Box<NoiseModel>,
        /// Random walk component
        random_walk: Box<NoiseModel>,
        /// Quantization component
        quantization: Box<NoiseModel>,
    },
}

impl NoiseModel {
    /// Create a simple Gaussian noise model
    pub fn gaussian(std_dev: f64) -> Self {
        Self::Gaussian { bias: 0.0, std_dev }
    }

    /// Create a Gaussian noise model with bias
    pub fn gaussian_with_bias(bias: f64, std_dev: f64) -> Self {
        Self::Gaussian { bias, std_dev }
    }

    /// Create a random walk noise model
    pub fn random_walk(std_dev: f64, walk_rate: f64) -> Self {
        Self::RandomWalk {
            bias: 0.0,
            std_dev,
            walk_rate,
        }
    }

    /// Create a quantization noise model
    pub fn quantization(resolution: f64) -> Self {
        Self::Quantization {
            resolution,
            bias: 0.0,
        }
    }

    /// Create a saturation model wrapping another noise model
    pub fn saturation(min: f64, max: f64, inner: NoiseModel) -> Self {
        Self::Saturation {
            min,
            max,
            inner: Box::new(inner),
        }
    }

    /// Apply noise to a single value
    pub fn apply(&self, value: f64, rng: &mut impl Rng) -> f64 {
        match self {
            NoiseModel::None => value,
            NoiseModel::Gaussian { bias, std_dev } => {
                let normal = Normal::new(*bias, *std_dev).unwrap_or(Normal::new(0.0, 1.0).unwrap());
                value + normal.sample(rng)
            }
            NoiseModel::RandomWalk { bias, std_dev, .. } => {
                // Note: Random walk bias update is handled separately
                let normal = Normal::new(*bias, *std_dev).unwrap_or(Normal::new(0.0, 1.0).unwrap());
                value + normal.sample(rng)
            }
            NoiseModel::Quantization { resolution, bias } => {
                let quantized = (value / resolution).round() * resolution;
                quantized + bias
            }
            NoiseModel::Saturation { min, max, inner } => {
                let noisy = inner.apply(value, rng);
                noisy.clamp(*min, *max)
            }
            NoiseModel::Combined { .. } => {
                // Combined models need state, handled separately
                value
            }
        }
    }

    /// Apply noise to a 3D vector
    pub fn apply_vector(&self, vector: &Vector3<f64>, rng: &mut impl Rng) -> Vector3<f64> {
        Vector3::new(
            self.apply(vector.x, rng),
            self.apply(vector.y, rng),
            self.apply(vector.z, rng),
        )
    }

    /// Get whether this model requires state (for random walk)
    pub fn requires_state(&self) -> bool {
        matches!(
            self,
            NoiseModel::RandomWalk { .. } | NoiseModel::Combined { .. }
        )
    }
}

/// Physics backend trait for sensor queries
pub trait PhysicsBackend {
    /// Get the base (pelvis) transform and velocity
    fn get_base_state(&self) -> Option<(Transform3D, Velocity)>;
    /// Get joint state by index
    fn get_joint_state(&self, index: usize) -> Option<JointState>;
    /// Get contact state for a body
    fn get_contact_state(&self, body_name: &str) -> Option<ContactState>;
    /// Get current simulation time
    fn simulation_time(&self) -> Instant;
}

/// 3D transform (position + orientation)
#[derive(Debug, Clone, Copy)]
pub struct Transform3D {
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
}

impl Default for Transform3D {
    fn default() -> Self {
        Self {
            position: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        }
    }
}

/// Velocity (linear + angular)
#[derive(Debug, Clone, Copy)]
pub struct Velocity {
    pub linear: Vector3<f64>,
    pub angular: Vector3<f64>,
}

impl Default for Velocity {
    fn default() -> Self {
        Self {
            linear: Vector3::zeros(),
            angular: Vector3::zeros(),
        }
    }
}

/// Joint state from physics
#[derive(Debug, Clone, Copy)]
pub struct JointState {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            effort: 0.0,
        }
    }
}

/// Contact state from physics
#[derive(Debug, Clone, Copy)]
pub struct ContactState {
    pub in_contact: bool,
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
    pub contact_point: Option<Vector3<f64>>,
}

impl Default for ContactState {
    fn default() -> Self {
        Self {
            in_contact: false,
            force: Vector3::zeros(),
            torque: Vector3::zeros(),
            contact_point: None,
        }
    }
}

/// IMU sensor configuration
#[derive(Debug, Clone)]
pub struct ImuConfig {
    /// Noise model for accelerometer
    pub accel_noise: NoiseModel,
    /// Noise model for gyroscope
    pub gyro_noise: NoiseModel,
    /// Sample rate
    pub sample_rate: Duration,
    /// Processing delay
    pub processing_delay: Duration,
    /// Enable temperature simulation
    pub simulate_temperature: bool,
    /// Temperature noise
    pub temperature_noise: NoiseModel,
}

impl Default for ImuConfig {
    fn default() -> Self {
        // Default to typical MEMS IMU specs (e.g., MPU-9250-like)
        Self {
            accel_noise: NoiseModel::gaussian(0.01),      // 10 mg noise
            gyro_noise: NoiseModel::gaussian(0.001),      // 1 deg/s noise (in rad/s)
            sample_rate: Duration::from_millis(1),        // 1 kHz
            processing_delay: Duration::from_micros(500), // 500 us
            simulate_temperature: false,
            temperature_noise: NoiseModel::gaussian(0.5),
        }
    }
}

impl ImuConfig {
    /// Create a high-quality IMU config (e.g., VectorNav, XSens)
    pub fn high_quality() -> Self {
        Self {
            accel_noise: NoiseModel::gaussian(0.001), // 1 mg noise
            gyro_noise: NoiseModel::gaussian(0.0001), // 0.1 deg/s noise
            sample_rate: Duration::from_millis(1),
            processing_delay: Duration::from_micros(200),
            simulate_temperature: true,
            temperature_noise: NoiseModel::gaussian(0.1),
        }
    }

    /// Create a low-cost IMU config
    pub fn low_cost() -> Self {
        Self {
            accel_noise: NoiseModel::gaussian(0.05), // 50 mg noise
            gyro_noise: NoiseModel::gaussian(0.005), // 5 deg/s noise
            sample_rate: Duration::from_millis(2),   // 500 Hz
            processing_delay: Duration::from_millis(1),
            simulate_temperature: false,
            temperature_noise: NoiseModel::gaussian(1.0),
        }
    }

    /// Create a realistic IMU with bias drift
    pub fn with_bias_drift(accel_walk: f64, gyro_walk: f64) -> Self {
        let mut config = Self::default();
        config.accel_noise = NoiseModel::RandomWalk {
            bias: 0.0,
            std_dev: 0.005,
            walk_rate: accel_walk,
        };
        config.gyro_noise = NoiseModel::RandomWalk {
            bias: 0.0,
            std_dev: 0.0005,
            walk_rate: gyro_walk,
        };
        config
    }
}

/// IMU sensor implementation
pub struct ImuSensor {
    name: String,
    config: ImuConfig,
    data: Timestamped<ImuData>,
    accel_bias: Vector3<f64>,
    gyro_bias: Vector3<f64>,
    rng: rand::rngs::StdRng,
    last_update: Instant,
}

impl ImuSensor {
    pub fn new(name: impl Into<String>, config: ImuConfig) -> Self {
        let name = name.into();
        let now = Instant::now();
        Self {
            name,
            config,
            data: Timestamped::new(ImuData::default(), now, Duration::default()),
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            rng: rand::SeedableRng::from_entropy(),
            last_update: now,
        }
    }

    pub fn data(&self) -> &Timestamped<ImuData> {
        &self.data
    }

    fn update_bias(&mut self, dt: Duration) {
        let dt_secs = dt.as_secs_f64();

        // Update accelerometer bias if using random walk
        if let NoiseModel::RandomWalk { walk_rate, .. } = &self.config.accel_noise {
            let normal = Normal::new(0.0, *walk_rate * dt_secs).unwrap();
            self.accel_bias.x += normal.sample(&mut self.rng);
            self.accel_bias.y += normal.sample(&mut self.rng);
            self.accel_bias.z += normal.sample(&mut self.rng);
        }

        // Update gyroscope bias if using random walk
        if let NoiseModel::RandomWalk { walk_rate, .. } = &self.config.gyro_noise {
            let normal = Normal::new(0.0, *walk_rate * dt_secs).unwrap();
            self.gyro_bias.x += normal.sample(&mut self.rng);
            self.gyro_bias.y += normal.sample(&mut self.rng);
            self.gyro_bias.z += normal.sample(&mut self.rng);
        }
    }
}

impl Sensor for ImuSensor {
    fn update(&mut self, physics: &dyn PhysicsBackend, dt: Duration) {
        self.last_update = physics.simulation_time();
        self.update_bias(dt);

        if let Some((transform, velocity)) = physics.get_base_state() {
            // Compute gravity in body frame
            let gravity_world = Vector3::new(0.0, 0.0, -9.81);
            let gravity_body = transform.orientation.inverse() * gravity_world;

            // Linear acceleration (gravity + acceleration)
            let linear_accel_body =
                transform.orientation.inverse() * velocity.linear + gravity_body;

            // Apply accelerometer noise
            let accel_noisy = self
                .config
                .accel_noise
                .apply_vector(&(linear_accel_body + self.accel_bias), &mut self.rng);

            // Angular velocity (apply gyro noise)
            let gyro_noisy = self
                .config
                .gyro_noise
                .apply_vector(&(velocity.angular + self.gyro_bias), &mut self.rng);

            // Temperature simulation
            let temperature = if self.config.simulate_temperature {
                let base_temp = 25.0; // Room temperature
                Some(
                    self.config
                        .temperature_noise
                        .apply(base_temp, &mut self.rng),
                )
            } else {
                None
            };

            self.data = Timestamped::new(
                ImuData {
                    linear_acceleration: accel_noisy,
                    angular_velocity: gyro_noisy,
                    orientation: transform.orientation,
                    temperature,
                },
                self.last_update,
                self.config.processing_delay,
            );
        }
    }

    fn data(&self) -> SensorData {
        SensorData::Imu(self.data.data.clone())
    }

    fn reset(&mut self) {
        self.accel_bias = Vector3::zeros();
        self.gyro_bias = Vector3::zeros();
        self.data = Timestamped::new(ImuData::default(), Instant::now(), Duration::default());
        self.rng = rand::SeedableRng::from_entropy();
    }

    fn set_noise_model(&mut self, model: NoiseModel) {
        // IMU uses separate models for accel and gyro, so this sets both to the same
        self.config.accel_noise = model.clone();
        self.config.gyro_noise = model;
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Imu
    }
}

/// Joint encoder configuration
#[derive(Debug, Clone)]
pub struct JointEncoderConfig {
    /// Noise model for position
    pub position_noise: NoiseModel,
    /// Noise model for velocity
    pub velocity_noise: NoiseModel,
    /// Noise model for effort
    pub effort_noise: NoiseModel,
    /// Sample rate
    pub sample_rate: Duration,
    /// Processing delay
    pub processing_delay: Duration,
    /// Joint names
    pub joint_names: Vec<String>,
}

impl Default for JointEncoderConfig {
    fn default() -> Self {
        Self {
            position_noise: NoiseModel::Quantization {
                resolution: 0.0001, // ~0.006 degrees resolution
                bias: 0.0,
            },
            velocity_noise: NoiseModel::gaussian(0.001), // Small noise
            effort_noise: NoiseModel::gaussian(0.01),    // 0.01 Nm noise
            sample_rate: Duration::from_millis(1),       // 1 kHz
            processing_delay: Duration::from_micros(100),
            joint_names: Vec::new(),
        }
    }
}

impl JointEncoderConfig {
    /// Create config for high-resolution encoders
    pub fn high_resolution() -> Self {
        Self {
            position_noise: NoiseModel::Quantization {
                resolution: 0.00001, // Very high resolution
                bias: 0.0,
            },
            velocity_noise: NoiseModel::gaussian(0.0001),
            effort_noise: NoiseModel::gaussian(0.005),
            sample_rate: Duration::from_millis(1),
            processing_delay: Duration::from_micros(50),
            joint_names: Vec::new(),
        }
    }
}

/// Joint encoder sensor implementation
pub struct JointEncoderSensor {
    name: String,
    config: JointEncoderConfig,
    data: Timestamped<JointEncoderData>,
    rng: rand::rngs::StdRng,
    last_update: Instant,
}

impl JointEncoderSensor {
    pub fn new(name: impl Into<String>, config: JointEncoderConfig) -> Self {
        let name = name.into();
        let now = Instant::now();
        let num_joints = config.joint_names.len().max(1);
        let joint_names = config.joint_names.clone();
        Self {
            name,
            config,
            data: Timestamped::new(
                JointEncoderData::with_names(num_joints, joint_names),
                now,
                Duration::default(),
            ),
            rng: rand::SeedableRng::from_entropy(),
            last_update: now,
        }
    }

    pub fn data(&self) -> &Timestamped<JointEncoderData> {
        &self.data
    }
}

impl Sensor for JointEncoderSensor {
    fn update(&mut self, physics: &dyn PhysicsBackend, _dt: Duration) {
        self.last_update = physics.simulation_time();
        let mut joint_data = JointEncoderData::with_names(
            self.data.data.joints.len(),
            self.config.joint_names.clone(),
        );

        for (i, joint) in joint_data.joints.iter_mut().enumerate() {
            if let Some(state) = physics.get_joint_state(i) {
                joint.position = self
                    .config
                    .position_noise
                    .apply(state.position, &mut self.rng);
                joint.velocity = self
                    .config
                    .velocity_noise
                    .apply(state.velocity, &mut self.rng);
                joint.effort = self.config.effort_noise.apply(state.effort, &mut self.rng);
            }
        }

        self.data = Timestamped::new(joint_data, self.last_update, self.config.processing_delay);
    }

    fn data(&self) -> SensorData {
        SensorData::JointEncoder(self.data.data.clone())
    }

    fn reset(&mut self) {
        let num_joints = self.data.data.joints.len();
        self.data = Timestamped::new(
            JointEncoderData::with_names(num_joints, self.config.joint_names.clone()),
            Instant::now(),
            Duration::default(),
        );
        self.rng = rand::SeedableRng::from_entropy();
    }

    fn set_noise_model(&mut self, model: NoiseModel) {
        self.config.position_noise = model.clone();
        self.config.velocity_noise = model.clone();
        self.config.effort_noise = model;
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::JointEncoder
    }
}

/// Force/torque sensor configuration
#[derive(Debug, Clone)]
pub struct ForceTorqueConfig {
    /// Noise model for force
    pub force_noise: NoiseModel,
    /// Noise model for torque
    pub torque_noise: NoiseModel,
    /// Sample rate
    pub sample_rate: Duration,
    /// Processing delay
    pub processing_delay: Duration,
    /// Body name in physics engine
    pub body_name: String,
    /// Sensor offset from body origin
    pub offset: Vector3<f64>,
}

impl Default for ForceTorqueConfig {
    fn default() -> Self {
        Self {
            force_noise: NoiseModel::gaussian(0.5),    // 0.5 N noise
            torque_noise: NoiseModel::gaussian(0.025), // 0.025 Nm noise
            sample_rate: Duration::from_millis(1),
            processing_delay: Duration::from_micros(200),
            body_name: String::new(),
            offset: Vector3::zeros(),
        }
    }
}

/// Force/torque sensor implementation
pub struct ForceTorqueSensor {
    name: String,
    config: ForceTorqueConfig,
    data: Timestamped<ForceTorqueData>,
    rng: rand::rngs::StdRng,
    last_update: Instant,
}

impl ForceTorqueSensor {
    pub fn new(name: impl Into<String>, config: ForceTorqueConfig) -> Self {
        let name = name.into();
        let now = Instant::now();
        Self {
            name,
            config,
            data: Timestamped::new(ForceTorqueData::default(), now, Duration::default()),
            rng: rand::SeedableRng::from_entropy(),
            last_update: now,
        }
    }

    pub fn data(&self) -> &Timestamped<ForceTorqueData> {
        &self.data
    }
}

impl Sensor for ForceTorqueSensor {
    fn update(&mut self, physics: &dyn PhysicsBackend, _dt: Duration) {
        self.last_update = physics.simulation_time();

        if let Some(contact) = physics.get_contact_state(&self.config.body_name) {
            let force_noisy = self
                .config
                .force_noise
                .apply_vector(&contact.force, &mut self.rng);
            let torque_noisy = self
                .config
                .torque_noise
                .apply_vector(&contact.torque, &mut self.rng);

            self.data = Timestamped::new(
                ForceTorqueData {
                    force: force_noisy,
                    torque: torque_noisy,
                    in_contact: contact.in_contact,
                    center_of_pressure: contact.contact_point,
                },
                self.last_update,
                self.config.processing_delay,
            );
        }
    }

    fn data(&self) -> SensorData {
        SensorData::ForceTorque(self.data.data.clone())
    }

    fn reset(&mut self) {
        self.data = Timestamped::new(
            ForceTorqueData::default(),
            Instant::now(),
            Duration::default(),
        );
        self.rng = rand::SeedableRng::from_entropy();
    }

    fn set_noise_model(&mut self, model: NoiseModel) {
        self.config.force_noise = model.clone();
        self.config.torque_noise = model;
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::ForceTorque
    }
}

/// Camera sensor configuration
#[derive(Debug, Clone)]
pub struct CameraConfig {
    /// Image width
    pub width: u32,
    /// Image height
    pub height: u32,
    /// Camera intrinsics (fx, fy, cx, cy)
    pub intrinsics: [f64; 4],
    /// Sample rate (frame rate)
    pub sample_rate: Duration,
    /// Processing delay (capture + processing)
    pub processing_delay: Duration,
    /// Enable depth simulation
    pub enable_depth: bool,
    /// Depth noise model
    pub depth_noise: NoiseModel,
}

impl Default for CameraConfig {
    fn default() -> Self {
        let width = 640;
        let height = 480;
        Self {
            width,
            height,
            intrinsics: [
                width as f64 * 0.8,  // fx
                height as f64 * 0.8, // fy
                width as f64 * 0.5,  // cx
                height as f64 * 0.5, // cy
            ],
            sample_rate: Duration::from_millis(33), // 30 Hz
            processing_delay: Duration::from_millis(5),
            enable_depth: true,
            depth_noise: NoiseModel::gaussian(0.001), // 1mm noise
        }
    }
}

impl CameraConfig {
    /// Create a high-resolution camera config (1080p)
    pub fn high_resolution() -> Self {
        let width = 1920;
        let height = 1080;
        Self {
            width,
            height,
            intrinsics: [
                width as f64 * 0.8,
                height as f64 * 0.8,
                width as f64 * 0.5,
                height as f64 * 0.5,
            ],
            sample_rate: Duration::from_millis(33),
            processing_delay: Duration::from_millis(10),
            enable_depth: true,
            depth_noise: NoiseModel::gaussian(0.001),
        }
    }
}

/// Camera sensor implementation (placeholder)
pub struct CameraSensor {
    name: String,
    config: CameraConfig,
    data: Timestamped<CameraData>,
    rng: rand::rngs::StdRng,
    last_update: Instant,
    frame_count: u64,
}

impl CameraSensor {
    pub fn new(name: impl Into<String>, config: CameraConfig) -> Self {
        let name = name.into();
        let now = Instant::now();
        let width = config.width;
        let height = config.height;
        Self {
            name,
            config,
            data: Timestamped::new(CameraData::new(width, height), now, Duration::default()),
            rng: rand::SeedableRng::from_entropy(),
            last_update: now,
            frame_count: 0,
        }
    }

    pub fn data(&self) -> &Timestamped<CameraData> {
        &self.data
    }

    /// Set camera extrinsics (position and orientation)
    pub fn set_extrinsics(&mut self, position: Vector3<f64>, orientation: UnitQuaternion<f64>) {
        self.data.data.extrinsics = (position, orientation);
    }
}

impl Sensor for CameraSensor {
    fn update(&mut self, physics: &dyn PhysicsBackend, _dt: Duration) {
        self.last_update = physics.simulation_time();
        self.frame_count += 1;

        // Camera update is mostly a placeholder since rendering requires graphics pipeline
        // In a real implementation, this would capture from the render target
        let mut camera_data = CameraData::new(self.config.width, self.config.height);
        camera_data.intrinsics = self.config.intrinsics;
        camera_data.extrinsics = self.data.data.extrinsics;

        // Simulate some depth noise
        if self.config.enable_depth {
            for depth in &mut camera_data.depth {
                *depth = self.config.depth_noise.apply(*depth as f64, &mut self.rng) as f32;
            }
        }

        self.data = Timestamped::new(camera_data, self.last_update, self.config.processing_delay);
    }

    fn data(&self) -> SensorData {
        SensorData::Camera(self.data.data.clone())
    }

    fn reset(&mut self) {
        self.data = Timestamped::new(
            CameraData::new(self.config.width, self.config.height),
            Instant::now(),
            Duration::default(),
        );
        self.frame_count = 0;
        self.rng = rand::SeedableRng::from_entropy();
    }

    fn set_noise_model(&mut self, model: NoiseModel) {
        self.config.depth_noise = model;
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Camera
    }
}

/// Sensor registry for managing multiple sensors
pub struct SensorRegistry {
    sensors: Vec<Box<dyn Sensor>>,
    update_rates: HashMap<String, Duration>,
    last_updates: HashMap<String, Instant>,
}

impl Default for SensorRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorRegistry {
    /// Create a new empty sensor registry
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            update_rates: HashMap::new(),
            last_updates: HashMap::new(),
        }
    }

    /// Add a sensor to the registry
    pub fn add_sensor(&mut self, name: impl Into<String>, sensor: Box<dyn Sensor>) {
        let name = name.into();
        self.last_updates.insert(name.clone(), Instant::now());
        self.sensors.push(sensor);
    }

    /// Add a sensor with a specific update rate
    pub fn add_sensor_with_rate(
        &mut self,
        name: impl Into<String>,
        sensor: Box<dyn Sensor>,
        rate: Duration,
    ) {
        let name = name.into();
        self.update_rates.insert(name.clone(), rate);
        self.add_sensor(name, sensor);
    }

    /// Remove a sensor by name
    pub fn remove_sensor(&mut self, name: &str) -> Option<Box<dyn Sensor>> {
        if let Some(index) = self.sensors.iter().position(|s| s.name() == name) {
            self.update_rates.remove(name);
            self.last_updates.remove(name);
            Some(self.sensors.remove(index))
        } else {
            None
        }
    }

    /// Get a sensor by name
    pub fn get_sensor(&self, name: &str) -> Option<&dyn Sensor> {
        self.sensors
            .iter()
            .find(|s| s.name() == name)
            .map(|s| s.as_ref())
    }

    /// Get a mutable sensor by name
    pub fn get_sensor_mut(&mut self, name: &str) -> Option<&mut dyn Sensor> {
        // Use an explicit mutable iteration with early return to avoid lifetime inference issues
        for sensor in &mut self.sensors {
            if sensor.name() == name {
                return Some(sensor.as_mut());
            }
        }
        None
    }

    /// Get all sensors of a specific type
    pub fn get_sensors_by_type(&self, sensor_type: SensorType) -> Vec<&dyn Sensor> {
        self.sensors
            .iter()
            .filter(|s| s.sensor_type() == sensor_type)
            .map(|s| s.as_ref())
            .collect()
    }

    /// Update all sensors that need updating
    pub fn update(&mut self, physics: &dyn PhysicsBackend, dt: Duration) {
        let now = physics.simulation_time();

        for sensor in &mut self.sensors {
            let name = sensor.name().to_string();
            let last_update = self.last_updates.get(&name).copied().unwrap_or(now);

            // Check if we should update based on rate
            let should_update = if let Some(rate) = self.update_rates.get(&name) {
                now.duration_since(last_update) >= *rate
            } else {
                true // Update every tick if no specific rate
            };

            if should_update {
                sensor.update(physics, dt);
                self.last_updates.insert(name, now);
            }
        }
    }

    /// Update a specific sensor by name
    pub fn update_sensor(
        &mut self,
        name: &str,
        physics: &dyn PhysicsBackend,
        dt: Duration,
    ) -> bool {
        if let Some(sensor) = self.sensors.iter_mut().find(|s| s.name() == name) {
            sensor.update(physics, dt);
            self.last_updates
                .insert(name.to_string(), physics.simulation_time());
            true
        } else {
            false
        }
    }

    /// Get data from all sensors
    pub fn get_all_data(&self) -> Vec<(&str, SensorData)> {
        self.sensors.iter().map(|s| (s.name(), s.data())).collect()
    }

    /// Get data from a specific sensor
    pub fn get_data(&self, name: &str) -> Option<SensorData> {
        self.sensors
            .iter()
            .find(|s| s.name() == name)
            .map(|s| s.data())
    }

    /// Reset all sensors
    pub fn reset_all(&mut self) {
        for sensor in &mut self.sensors {
            sensor.reset();
        }
        self.last_updates.clear();
    }

    /// Reset a specific sensor
    pub fn reset_sensor(&mut self, name: &str) -> bool {
        if let Some(sensor) = self.sensors.iter_mut().find(|s| s.name() == name) {
            sensor.reset();
            self.last_updates.remove(name);
            true
        } else {
            false
        }
    }

    /// Get the number of sensors
    pub fn len(&self) -> usize {
        self.sensors.len()
    }

    /// Check if registry is empty
    pub fn is_empty(&self) -> bool {
        self.sensors.is_empty()
    }

    /// Get all sensor names
    pub fn sensor_names(&self) -> Vec<&str> {
        self.sensors.iter().map(|s| s.name()).collect()
    }

    /// Get all sensor types
    pub fn sensor_types(&self) -> Vec<SensorType> {
        self.sensors.iter().map(|s| s.sensor_type()).collect()
    }
}

/// Sensor synchronization configuration
#[derive(Debug, Clone)]
pub struct SensorSync {
    /// Enable hardware synchronization
    pub enabled: bool,
    /// Master sensor name
    pub master_sensor: String,
    /// Slave sensor names
    pub slave_sensors: Vec<String>,
    /// Sync tolerance
    pub tolerance: Duration,
}

impl Default for SensorSync {
    fn default() -> Self {
        Self {
            enabled: false,
            master_sensor: String::new(),
            slave_sensors: Vec::new(),
            tolerance: Duration::from_micros(100),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockPhysicsBackend {
        time: Instant,
        base_state: Option<(Transform3D, Velocity)>,
    }

    impl MockPhysicsBackend {
        fn new() -> Self {
            Self {
                time: Instant::now(),
                base_state: Some((Transform3D::default(), Velocity::default())),
            }
        }
    }

    impl PhysicsBackend for MockPhysicsBackend {
        fn get_base_state(&self) -> Option<(Transform3D, Velocity)> {
            self.base_state
        }

        fn get_joint_state(&self, _index: usize) -> Option<JointState> {
            Some(JointState::default())
        }

        fn get_contact_state(&self, _body_name: &str) -> Option<ContactState> {
            Some(ContactState::default())
        }

        fn simulation_time(&self) -> Instant {
            self.time
        }
    }

    #[test]
    fn test_noise_model_gaussian() {
        let model = NoiseModel::gaussian(0.1);
        let mut rng = rand::thread_rng();

        let value = 1.0;
        let noisy = model.apply(value, &mut rng);

        // Just verify it runs and produces different values
        assert!(noisy != value || model.apply(value, &mut rng) != value);
    }

    #[test]
    fn test_noise_model_quantization() {
        let model = NoiseModel::quantization(0.5);
        let mut rng = rand::thread_rng();

        let value = 1.3;
        let quantized = model.apply(value, &mut rng);

        // Should be rounded to nearest 0.5
        assert!((quantized - 1.5).abs() < 0.01 || (quantized - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_noise_model_saturation() {
        let inner = NoiseModel::gaussian(0.0);
        let model = NoiseModel::saturation(-1.0, 1.0, inner);
        let mut rng = rand::thread_rng();

        assert_eq!(model.apply(-2.0, &mut rng), -1.0);
        assert_eq!(model.apply(2.0, &mut rng), 1.0);
        assert_eq!(model.apply(0.5, &mut rng), 0.5);
    }

    #[test]
    fn test_imu_sensor() {
        let config = ImuConfig::default();
        let mut imu = ImuSensor::new("test_imu", config);
        let physics = MockPhysicsBackend::new();

        imu.update(&physics, Duration::from_millis(1));

        let data = imu.data();
        assert_eq!(data.data.linear_acceleration.z, -9.81);
    }

    #[test]
    fn test_joint_encoder() {
        let config = JointEncoderConfig {
            joint_names: vec!["j1".to_string(), "j2".to_string()],
            ..Default::default()
        };
        let mut encoder = JointEncoderSensor::new("test_encoder", config);
        let physics = MockPhysicsBackend::new();

        encoder.update(&physics, Duration::from_millis(1));

        let data = encoder.data();
        assert_eq!(data.data.joints.len(), 2);
        assert_eq!(data.data.joint_names[0], "j1");
    }

    #[test]
    fn test_sensor_registry() {
        let mut registry = SensorRegistry::new();

        let imu_config = ImuConfig::default();
        let imu = ImuSensor::new("pelvis_imu", imu_config);
        registry.add_sensor("pelvis_imu", Box::new(imu));

        assert_eq!(registry.len(), 1);
        assert!(registry.get_sensor("pelvis_imu").is_some());

        let physics = MockPhysicsBackend::new();
        registry.update(&physics, Duration::from_millis(1));

        let data = registry.get_data("pelvis_imu");
        assert!(data.is_some());
    }

    #[test]
    fn test_imu_configs() {
        let default = ImuConfig::default();
        let high_quality = ImuConfig::high_quality();
        let low_cost = ImuConfig::low_cost();

        // High quality should have lower noise
        match (&default.gyro_noise, &high_quality.gyro_noise) {
            (NoiseModel::Gaussian { std_dev: d, .. }, NoiseModel::Gaussian { std_dev: h, .. }) => {
                assert!(*h < *d);
            }
            _ => {}
        }

        // Low cost should have higher noise
        match (&default.gyro_noise, &low_cost.gyro_noise) {
            (NoiseModel::Gaussian { std_dev: d, .. }, NoiseModel::Gaussian { std_dev: l, .. }) => {
                assert!(*l > *d);
            }
            _ => {}
        }
    }

    #[test]
    fn test_sensor_reset() {
        let mut imu = ImuSensor::new("test", ImuConfig::default());
        let physics = MockPhysicsBackend::new();

        imu.update(&physics, Duration::from_millis(1));
        imu.reset();

        let data = imu.data();
        // After reset, orientation should be identity
        assert!(data.data.orientation.i().abs() < 0.001);
        assert!(data.data.orientation.j().abs() < 0.001);
        assert!(data.data.orientation.k().abs() < 0.001);
    }

    #[test]
    fn test_timestamped_data() {
        let now = Instant::now();
        let delay = Duration::from_millis(10);

        let data = Timestamped::new(42.0, now, delay);

        assert_eq!(data.data, 42.0);
        assert_eq!(data.timestamp, now);
        assert_eq!(data.acquisition_delay, delay);
        assert_eq!(data.measurement_time(), now - delay);
    }
}
