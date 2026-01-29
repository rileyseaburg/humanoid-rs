//! Deterministic simulation core for reproducible physics
//!
//! This module provides deterministic timing control for physics simulations,
//! ensuring that the same inputs produce the same outputs. This is critical for:
//! - Unit testing physics behavior
//! - Replay debugging
//! - Reinforcement learning training
//! - Regression testing
//!
//! # Timing Policies
//!
//! ## Fixed Timestep
//! The simulation advances by fixed time intervals (e.g., 1/500s). This ensures
//! deterministic behavior but may require interpolation for rendering.
//!
//! ## Variable Timestep  
//! The simulation advances by the actual elapsed time. This provides smooth
//! rendering but may produce slightly different results on different runs.
//!
//! ## Fixed-Update Variable-Render
//! Physics runs at fixed timestep while rendering interpolates between states.
//! This is the recommended approach for most applications.

use std::time::Duration;

use crate::{HumanoidError, Result};

/// Simulation clock with deterministic timing control
///
/// The clock manages simulation time independently of wall-clock time,
/// allowing for pause, resume, and deterministic replay.
///
/// # Example
/// ```
/// use std::time::Duration;
/// use humanoid_rs::sim::deterministic::SimulationClock;
///
/// let mut clock = SimulationClock::new(Duration::from_micros(2000)); // 500 Hz
/// clock.accumulate(Duration::from_micros(5000)); // 5ms of real time
///
/// while let Some(dt) = clock.step() {
///     // Advance physics by fixed timestep
///     // This will execute 2 times (5000us / 2000us = 2.5, truncated to 2)
/// }
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulationClock {
    /// Fixed timestep duration (e.g., 2ms for 500 Hz)
    timestep: Duration,
    /// Accumulated time waiting to be simulated
    accumulated_time: Duration,
    /// Current simulation step counter (for reproducibility)
    current_step: u64,
    /// Whether simulation is paused
    paused: bool,
    /// Total simulated time elapsed
    total_simulated_time: Duration,
}

impl SimulationClock {
    /// Create a new simulation clock with the specified timestep
    ///
    /// # Arguments
    /// * `timestep` - The fixed timestep for each simulation step
    ///
    /// # Example
    /// ```
    /// use std::time::Duration;
    /// use humanoid_rs::sim::deterministic::SimulationClock;
    ///
    /// let clock = SimulationClock::new(Duration::from_millis(2)); // 500 Hz
    /// ```
    pub fn new(timestep: Duration) -> Self {
        Self {
            timestep,
            accumulated_time: Duration::ZERO,
            current_step: 0,
            paused: false,
            total_simulated_time: Duration::ZERO,
        }
    }

    /// Create a new simulation clock with 500 Hz default timestep
    pub fn default_physics() -> Self {
        Self::new(Duration::from_micros(2000)) // 500 Hz = 2ms
    }

    /// Create a new simulation clock with 60 Hz default timestep
    pub fn default_render() -> Self {
        Self::new(Duration::from_micros(16667)) // ~60 Hz = 16.67ms
    }

    /// Accumulate time from the render loop
    ///
    /// Call this with the actual elapsed time between frames.
    /// The accumulated time will be consumed in fixed steps.
    ///
    /// # Arguments
    /// * `dt` - Real elapsed time to add to the accumulator
    pub fn accumulate(&mut self, dt: Duration) {
        if !self.paused {
            self.accumulated_time += dt;
        }
    }

    /// Get the next timestep if available
    ///
    /// Returns Some(timestep) if enough time has accumulated,
    /// None otherwise. Each call consumes one timestep from the accumulator.
    ///
    /// # Returns
    /// * `Some(Duration)` - The fixed timestep to advance simulation
    /// * `None` - Not enough accumulated time for a step
    pub fn step(&mut self) -> Option<Duration> {
        if self.accumulated_time >= self.timestep {
            self.accumulated_time -= self.timestep;
            self.current_step += 1;
            self.total_simulated_time += self.timestep;
            Some(self.timestep)
        } else {
            None
        }
    }

    /// Get the current simulation step number
    pub fn current_step(&self) -> u64 {
        self.current_step
    }

    /// Get the fixed timestep duration
    pub fn timestep(&self) -> Duration {
        self.timestep
    }

    /// Get the accumulated (unprocessed) time
    pub fn accumulated_time(&self) -> Duration {
        self.accumulated_time
    }

    /// Get the total simulated time elapsed
    pub fn total_simulated_time(&self) -> Duration {
        self.total_simulated_time
    }

    /// Check if simulation is paused
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Pause the simulation
    pub fn pause(&mut self) {
        self.paused = true;
    }

    /// Resume the simulation
    pub fn resume(&mut self) {
        self.paused = false;
    }

    /// Reset the clock to initial state
    pub fn reset(&mut self) {
        self.accumulated_time = Duration::ZERO;
        self.current_step = 0;
        self.total_simulated_time = Duration::ZERO;
        self.paused = false;
    }

    /// Rewind by a number of steps (for deterministic replay)
    ///
    /// Note: This only rewinds the clock, not the actual physics state.
    /// The caller must handle state restoration.
    ///
    /// # Arguments
    /// * `steps` - Number of steps to rewind
    ///
    /// # Returns
    /// * `Ok(())` - Successfully rewound
    /// * `Err(HumanoidError)` - Attempted to rewind past step 0
    pub fn rewind_clock(&mut self, steps: usize) -> Result<()> {
        let steps_u64 = steps as u64;
        if steps_u64 > self.current_step {
            return Err(HumanoidError::SimulatorError(format!(
                "Cannot rewind {} steps from step {}",
                steps, self.current_step
            )));
        }
        self.current_step -= steps_u64;
        let time_to_remove = self.timestep * steps as u32;
        self.total_simulated_time = self.total_simulated_time.saturating_sub(time_to_remove);
        Ok(())
    }

    /// Calculate interpolation factor for rendering
    ///
    /// Returns a value between 0.0 and 1.0 indicating how far into
    /// the next physics step we are. Use this for smooth interpolation
    /// between physics states.
    pub fn interpolation_factor(&self) -> f64 {
        if self.timestep.is_zero() {
            return 0.0;
        }
        let accumulated = self.accumulated_time.as_nanos() as f64;
        let timestep = self.timestep.as_nanos() as f64;
        (accumulated / timestep).min(1.0)
    }
}

/// Trait for deterministic simulation control
///
/// Implement this trait for any system that needs deterministic
/// simulation capabilities (physics, sensors, controllers, etc.)
///
/// # Example
/// ```
/// use std::time::Duration;
/// use humanoid_rs::sim::deterministic::{SimulationControl, SimulationClock};
/// use humanoid_rs::Result;
///
/// struct MyPhysicsSystem {
///     clock: SimulationClock,
///     state: Vec<f64>,
/// }
///
/// impl SimulationControl for MyPhysicsSystem {
///     fn step(&mut self, dt: Duration) -> Result<()> {
///         // Advance physics by dt
///         Ok(())
///     }
///
///     fn pause(&mut self) {
///         self.clock.pause();
///     }
///
///     fn resume(&mut self) {
///         self.clock.resume();
///     }
///
///     fn reset(&mut self) -> Result<()> {
///         self.clock.reset();
///         self.state.clear();
///         Ok(())
///     }
///
///     fn current_step(&self) -> u64 {
///         self.clock.current_step()
///     }
/// }
/// ```
pub trait SimulationControl {
    /// Advance simulation by one fixed timestep
    ///
    /// # Arguments
    /// * `dt` - The fixed timestep duration
    ///
    /// # Returns
    /// * `Ok(())` - Step completed successfully
    /// * `Err(HumanoidError)` - Error during simulation step
    fn step(&mut self, dt: Duration) -> Result<()>;

    /// Pause simulation execution
    fn pause(&mut self);

    /// Resume simulation execution
    fn resume(&mut self);

    /// Reset simulation to initial state
    ///
    /// # Returns
    /// * `Ok(())` - Reset completed successfully
    /// * `Err(HumanoidError)` - Error during reset
    fn reset(&mut self) -> Result<()>;

    /// Get the current simulation step number
    fn current_step(&self) -> u64;

    /// Rewind simulation by a number of steps (optional)
    ///
    /// Default implementation returns an error.
    /// Override if your system supports state rewinding.
    ///
    /// # Arguments
    /// * `steps` - Number of steps to rewind
    fn rewind(&mut self, _steps: usize) -> Result<()> {
        Err(HumanoidError::SimulatorError(
            "Rewind not supported by this system".to_string(),
        ))
    }

    /// Check if system is currently paused
    fn is_paused(&self) -> bool;
}

/// Subsystem types for scheduling
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Subsystem {
    /// Physics simulation (Rapier)
    Physics,
    /// Sensor reading and processing
    Sensors,
    /// Control algorithms (MPC, IK, etc.)
    Control,
    /// Actuator/motor control
    Actuation,
    /// State estimation and filtering
    StateEstimation,
    /// Visualization and rendering
    Visualization,
}

/// Configuration for a subsystem's tick rate
#[derive(Debug, Clone, Copy)]
pub struct TickConfig {
    /// Subsystem identifier
    pub subsystem: Subsystem,
    /// How often this subsystem updates (every N physics steps)
    pub tick_every_n_steps: usize,
    /// Whether this subsystem is enabled
    pub enabled: bool,
}

impl TickConfig {
    /// Create a new tick configuration
    pub fn new(subsystem: Subsystem, tick_every_n_steps: usize) -> Self {
        Self {
            subsystem,
            tick_every_n_steps,
            enabled: true,
        }
    }

    /// Create a configuration that ticks every physics step (1:1)
    pub fn every_step(subsystem: Subsystem) -> Self {
        Self::new(subsystem, 1)
    }

    /// Create a configuration for 500 Hz physics with 50 Hz subsystem
    pub fn at_50_hz(subsystem: Subsystem) -> Self {
        Self::new(subsystem, 10) // 500/50 = 10
    }

    /// Create a configuration for 500 Hz physics with 100 Hz subsystem
    pub fn at_100_hz(subsystem: Subsystem) -> Self {
        Self::new(subsystem, 5) // 500/100 = 5
    }

    /// Create a configuration for 500 Hz physics with 200 Hz subsystem
    pub fn at_200_hz(subsystem: Subsystem) -> Self {
        Self::new(subsystem, 2) // 500/200 = 2.5, round to 2
    }
}

/// Deterministic scheduler with fixed tick ordering
///
/// The scheduler ensures subsystems update in a deterministic order:
/// 1. Physics → 2. Sensors → 3. Control → 4. Actuation
///
/// This ordering ensures that:
/// - Sensors read the latest physics state
/// - Controllers use fresh sensor data
/// - Actuators apply control based on current decisions
/// - Physics uses the actuator commands for the next step
///
/// # Example
/// ```
/// use humanoid_rs::sim::deterministic::{DeterministicScheduler, TickConfig, Subsystem};
///
/// let scheduler = DeterministicScheduler::new(500.0) // 500 Hz physics
///     .with_config(TickConfig::at_100_hz(Subsystem::Sensors))
///     .with_config(TickConfig::at_50_hz(Subsystem::Control));
/// ```
#[derive(Debug, Clone)]
pub struct DeterministicScheduler {
    /// Physics clock (master clock)
    clock: SimulationClock,
    /// Configuration for each subsystem
    configs: Vec<TickConfig>,
    /// Step counters for each subsystem
    step_counters: Vec<usize>,
    /// Random seed for reproducibility
    random_seed: u64,
    /// Execution order: fixed as [Physics, Sensors, Control, Actuation]
    execution_order: Vec<Subsystem>,
}

impl DeterministicScheduler {
    /// Create a new deterministic scheduler
    ///
    /// # Arguments
    /// * `physics_hz` - Physics update rate in Hz (e.g., 500.0 for 500 Hz)
    pub fn new(physics_hz: f64) -> Self {
        let timestep = Duration::from_nanos((1_000_000_000.0 / physics_hz) as u64);

        Self {
            clock: SimulationClock::new(timestep),
            configs: Vec::new(),
            step_counters: Vec::new(),
            random_seed: 0,
            execution_order: vec![
                Subsystem::Physics,
                Subsystem::Sensors,
                Subsystem::Control,
                Subsystem::Actuation,
            ],
        }
    }

    /// Set the random seed for reproducibility
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.random_seed = seed;
        self
    }

    /// Add a subsystem configuration
    pub fn with_config(mut self, config: TickConfig) -> Self {
        self.configs.push(config);
        self.step_counters.push(0);
        self
    }

    /// Get the physics clock
    pub fn clock(&self) -> &SimulationClock {
        &self.clock
    }

    /// Get mutable access to the clock
    pub fn clock_mut(&mut self) -> &mut SimulationClock {
        &mut self.clock
    }

    /// Accumulate time and determine which subsystems need updates
    ///
    /// Returns the list of subsystems that should update this frame,
    /// in the correct execution order.
    ///
    /// # Arguments
    /// * `dt` - Real elapsed time
    ///
    /// # Returns
    /// Vector of subsystems that need to update, in order
    pub fn update(&mut self, dt: Duration) -> Vec<Subsystem> {
        self.clock.accumulate(dt);

        let mut subsystems_to_update = Vec::new();

        // Check if we should do a physics step
        if self.clock.step().is_some() {
            // Add subsystems in fixed order
            for subsystem in &self.execution_order {
                if self.should_tick(*subsystem) {
                    subsystems_to_update.push(*subsystem);
                }
            }

            // Increment step counters
            for i in 0..self.step_counters.len() {
                self.step_counters[i] += 1;
            }
        }

        subsystems_to_update
    }

    /// Check if a subsystem should tick on this step
    fn should_tick(&self, subsystem: Subsystem) -> bool {
        // Physics always ticks (it's the master clock)
        if subsystem == Subsystem::Physics {
            return true;
        }

        // Find config for this subsystem
        for (i, config) in self.configs.iter().enumerate() {
            if config.subsystem == subsystem && config.enabled {
                let counter = self.step_counters.get(i).copied().unwrap_or(0);
                return counter % config.tick_every_n_steps == 0;
            }
        }

        // Default: don't tick if no config
        false
    }

    /// Reset the scheduler
    pub fn reset(&mut self) {
        self.clock.reset();
        for counter in &mut self.step_counters {
            *counter = 0;
        }
    }

    /// Get the random seed
    pub fn random_seed(&self) -> u64 {
        self.random_seed
    }

    /// Set the execution order (default: Physics → Sensors → Control → Actuation)
    pub fn set_execution_order(&mut self, order: Vec<Subsystem>) {
        self.execution_order = order;
    }

    /// Get the current execution order
    pub fn execution_order(&self) -> &[Subsystem] {
        &self.execution_order
    }

    /// Enable/disable a subsystem
    pub fn set_subsystem_enabled(&mut self, subsystem: Subsystem, enabled: bool) {
        for config in &mut self.configs {
            if config.subsystem == subsystem {
                config.enabled = enabled;
            }
        }
    }

    /// Get the physics timestep
    pub fn physics_timestep(&self) -> Duration {
        self.clock.timestep()
    }

    /// Get physics rate in Hz
    pub fn physics_hz(&self) -> f64 {
        1.0 / (self.clock.timestep().as_nanos() as f64 / 1_000_000_000.0)
    }
}

/// Timestep mode configuration for physics simulation
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimestepMode {
    /// Fixed timestep with substeps (deterministic)
    ///
    /// Physics always advances by exactly `dt` seconds per step,
    /// performing `substeps` internal substeps for stability.
    Fixed {
        /// Base timestep in seconds
        dt: f64,
        /// Number of substeps per timestep
        substeps: u32,
    },

    /// Variable timestep (non-deterministic but smooth)
    ///
    /// Physics advances by the actual elapsed time.
    /// Good for rendering, bad for reproducibility.
    Variable,

    /// Fixed-Update Variable-Render (hybrid)
    ///
    /// Physics runs at fixed timestep while rendering interpolates.
    /// Best of both worlds: deterministic + smooth rendering.
    Interpolated {
        /// Physics timestep
        dt: f64,
        /// Maximum accumulated time before simulation catches up
        max_accumulated: Duration,
    },
}

impl Default for TimestepMode {
    fn default() -> Self {
        TimestepMode::Fixed {
            dt: 1.0 / 500.0, // 500 Hz
            substeps: 4,
        }
    }
}

impl TimestepMode {
    /// Create a fixed timestep configuration
    pub fn fixed(hz: f64, substeps: u32) -> Self {
        TimestepMode::Fixed {
            dt: 1.0 / hz,
            substeps,
        }
    }

    /// Create an interpolated timestep configuration
    pub fn interpolated(hz: f64, max_accumulated_ms: u64) -> Self {
        TimestepMode::Interpolated {
            dt: 1.0 / hz,
            max_accumulated: Duration::from_millis(max_accumulated_ms),
        }
    }

    /// Get the base timestep duration
    pub fn timestep(&self) -> Duration {
        match self {
            TimestepMode::Fixed { dt, .. } => Duration::from_nanos((dt * 1_000_000_000.0) as u64),
            TimestepMode::Variable => Duration::ZERO,
            TimestepMode::Interpolated { dt, .. } => {
                Duration::from_nanos((dt * 1_000_000_000.0) as u64)
            }
        }
    }

    /// Check if this mode is deterministic
    pub fn is_deterministic(&self) -> bool {
        matches!(
            self,
            TimestepMode::Fixed { .. } | TimestepMode::Interpolated { .. }
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_clock_creation() {
        let clock = SimulationClock::new(Duration::from_millis(2));
        assert_eq!(clock.timestep(), Duration::from_millis(2));
        assert_eq!(clock.current_step(), 0);
        assert!(!clock.is_paused());
    }

    #[test]
    fn test_simulation_clock_default_physics() {
        let clock = SimulationClock::default_physics();
        assert_eq!(clock.timestep(), Duration::from_micros(2000)); // 500 Hz
    }

    #[test]
    fn test_simulation_clock_accumulation() {
        let mut clock = SimulationClock::new(Duration::from_millis(2));

        clock.accumulate(Duration::from_millis(5));
        assert_eq!(clock.accumulated_time(), Duration::from_millis(5));

        // Should get 2 steps (5ms / 2ms = 2.5, truncated)
        assert!(clock.step().is_some());
        assert!(clock.step().is_some());
        assert!(clock.step().is_none()); // 1ms remaining

        assert_eq!(clock.current_step(), 2);
        assert_eq!(clock.accumulated_time(), Duration::from_millis(1));
    }

    #[test]
    fn test_simulation_clock_pause_resume() {
        let mut clock = SimulationClock::new(Duration::from_millis(2));

        clock.pause();
        assert!(clock.is_paused());

        clock.accumulate(Duration::from_millis(10));
        assert_eq!(clock.accumulated_time(), Duration::ZERO); // Should not accumulate while paused

        clock.resume();
        assert!(!clock.is_paused());

        clock.accumulate(Duration::from_millis(10));
        assert_eq!(clock.accumulated_time(), Duration::from_millis(10));
    }

    #[test]
    fn test_simulation_clock_reset() {
        let mut clock = SimulationClock::new(Duration::from_millis(2));

        clock.accumulate(Duration::from_millis(10));
        clock.step();
        clock.pause();

        clock.reset();

        assert_eq!(clock.current_step(), 0);
        assert_eq!(clock.accumulated_time(), Duration::ZERO);
        assert!(!clock.is_paused());
    }

    #[test]
    fn test_simulation_clock_rewind() {
        let mut clock = SimulationClock::new(Duration::from_millis(2));

        clock.accumulate(Duration::from_millis(10));
        while clock.step().is_some() {}

        assert_eq!(clock.current_step(), 5);

        clock.rewind_clock(2).unwrap();
        assert_eq!(clock.current_step(), 3);
    }

    #[test]
    fn test_simulation_clock_rewind_error() {
        let mut clock = SimulationClock::new(Duration::from_millis(2));

        let result = clock.rewind_clock(1);
        assert!(result.is_err());
    }

    #[test]
    fn test_simulation_clock_interpolation() {
        let mut clock = SimulationClock::new(Duration::from_millis(2));

        // Halfway through a timestep
        clock.accumulate(Duration::from_millis(1));
        let factor = clock.interpolation_factor();
        assert!(factor >= 0.0 && factor <= 1.0);
        assert!((factor - 0.5).abs() < 0.01); // Should be ~0.5
    }

    #[test]
    fn test_deterministic_scheduler_creation() {
        let scheduler = DeterministicScheduler::new(500.0);
        assert!((scheduler.physics_hz() - 500.0).abs() < 0.1);
        assert_eq!(scheduler.clock().current_step(), 0);
    }

    #[test]
    fn test_deterministic_scheduler_with_configs() {
        let mut scheduler = DeterministicScheduler::new(500.0)
            .with_config(TickConfig::at_100_hz(Subsystem::Sensors))
            .with_config(TickConfig::at_50_hz(Subsystem::Control));

        // Run for 20ms at 500 Hz = 10 physics steps
        let dt = Duration::from_millis(20);
        let mut physics_count = 0;
        let mut sensor_count = 0;
        let mut control_count = 0;

        for _ in 0..10 {
            let subsystems = scheduler.update(dt / 10);
            for sub in subsystems {
                match sub {
                    Subsystem::Physics => physics_count += 1,
                    Subsystem::Sensors => sensor_count += 1,
                    Subsystem::Control => control_count += 1,
                    _ => {}
                }
            }
        }

        assert_eq!(physics_count, 10); // Every step
        assert_eq!(sensor_count, 2); // 100 Hz = every 5th step
        assert_eq!(control_count, 1); // 50 Hz = every 10th step
    }

    #[test]
    fn test_deterministic_scheduler_ordering() {
        let scheduler = DeterministicScheduler::new(500.0);
        let order = scheduler.execution_order();

        assert_eq!(order[0], Subsystem::Physics);
        assert_eq!(order[1], Subsystem::Sensors);
        assert_eq!(order[2], Subsystem::Control);
        assert_eq!(order[3], Subsystem::Actuation);
    }

    #[test]
    fn test_deterministic_scheduler_seed() {
        let scheduler = DeterministicScheduler::new(500.0).with_seed(12345);

        assert_eq!(scheduler.random_seed(), 12345);
    }

    #[test]
    fn test_deterministic_scheduler_reset() {
        let mut scheduler = DeterministicScheduler::new(500.0)
            .with_config(TickConfig::at_100_hz(Subsystem::Sensors));

        // Run some steps
        for _ in 0..10 {
            scheduler.update(Duration::from_millis(2));
        }

        assert!(scheduler.clock().current_step() > 0);

        scheduler.reset();

        assert_eq!(scheduler.clock().current_step(), 0);
    }

    #[test]
    fn test_timestep_mode_fixed() {
        let mode = TimestepMode::fixed(500.0, 4);

        match mode {
            TimestepMode::Fixed { dt, substeps } => {
                assert!((dt - 0.002).abs() < 0.0001); // 1/500
                assert_eq!(substeps, 4);
            }
            _ => panic!("Expected Fixed mode"),
        }

        assert!(mode.is_deterministic());
        assert_eq!(mode.timestep(), Duration::from_micros(2000));
    }

    #[test]
    fn test_timestep_mode_interpolated() {
        let mode = TimestepMode::interpolated(500.0, 100);

        match mode {
            TimestepMode::Interpolated {
                dt,
                max_accumulated,
            } => {
                assert!((dt - 0.002).abs() < 0.0001);
                assert_eq!(max_accumulated, Duration::from_millis(100));
            }
            _ => panic!("Expected Interpolated mode"),
        }

        assert!(mode.is_deterministic());
    }

    #[test]
    fn test_timestep_mode_variable() {
        let mode = TimestepMode::Variable;
        assert!(!mode.is_deterministic());
        assert_eq!(mode.timestep(), Duration::ZERO);
    }

    #[test]
    fn test_tick_config() {
        let config = TickConfig::at_50_hz(Subsystem::Sensors);
        assert_eq!(config.subsystem, Subsystem::Sensors);
        assert_eq!(config.tick_every_n_steps, 10); // 500/50
        assert!(config.enabled);

        let config = TickConfig::at_100_hz(Subsystem::Control);
        assert_eq!(config.tick_every_n_steps, 5); // 500/100

        let config = TickConfig::every_step(Subsystem::Physics);
        assert_eq!(config.tick_every_n_steps, 1);
    }

    #[test]
    fn test_determinism_same_seed_same_result() {
        // This test verifies that the same sequence of operations
        // produces the same clock state

        let mut clock1 = SimulationClock::new(Duration::from_millis(2));
        let mut clock2 = SimulationClock::new(Duration::from_millis(2));

        // Perform identical operations
        for _ in 0..100 {
            clock1.accumulate(Duration::from_millis(2));
            clock2.accumulate(Duration::from_millis(2));

            while let Some(_) = clock1.step() {}
            while let Some(_) = clock2.step() {}
        }

        assert_eq!(clock1.current_step(), clock2.current_step());
        assert_eq!(clock1.accumulated_time(), clock2.accumulated_time());
        assert_eq!(clock1.total_simulated_time(), clock2.total_simulated_time());
    }

    #[test]
    fn test_scheduler_determinism() {
        // Verify scheduler produces deterministic results
        let mut scheduler1 = DeterministicScheduler::new(500.0)
            .with_seed(42)
            .with_config(TickConfig::at_100_hz(Subsystem::Sensors));

        let mut scheduler2 = DeterministicScheduler::new(500.0)
            .with_seed(42)
            .with_config(TickConfig::at_100_hz(Subsystem::Sensors));

        let dt = Duration::from_millis(2);

        for _ in 0..50 {
            let result1 = scheduler1.update(dt);
            let result2 = scheduler2.update(dt);
            assert_eq!(result1.len(), result2.len());
            assert_eq!(result1, result2);
        }

        assert_eq!(
            scheduler1.clock().current_step(),
            scheduler2.clock().current_step()
        );
    }
}
