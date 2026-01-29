// Simple test file to verify deterministic module compiles correctly
use std::time::Duration;

// Mock the HumanoidError for testing
#[derive(Debug)]
pub enum HumanoidError {
    SimulatorError(String),
}

pub type Result<T> = std::result::Result<T, HumanoidError>;

// Include the deterministic module
pub mod deterministic;

fn main() {
    println!("Testing deterministic simulation core...\n");

    // Test 1: SimulationClock
    println!("Test 1: SimulationClock creation");
    let clock = deterministic::SimulationClock::new(Duration::from_millis(2));
    assert_eq!(clock.timestep(), Duration::from_millis(2));
    assert_eq!(clock.current_step(), 0);
    println!("✓ SimulationClock created successfully\n");

    // Test 2: Accumulation and stepping
    println!("Test 2: Time accumulation");
    let mut clock = deterministic::SimulationClock::new(Duration::from_millis(2));
    clock.accumulate(Duration::from_millis(5));

    let mut steps = 0;
    while let Some(_) = clock.step() {
        steps += 1;
    }
    assert_eq!(steps, 2);
    assert_eq!(clock.current_step(), 2);
    println!("✓ Accumulated 5ms, executed {} steps\n", steps);

    // Test 3: Pause/Resume
    println!("Test 3: Pause and resume");
    let mut clock = deterministic::SimulationClock::new(Duration::from_millis(2));
    clock.pause();
    clock.accumulate(Duration::from_millis(10));
    assert_eq!(clock.accumulated_time(), Duration::ZERO);
    clock.resume();
    clock.accumulate(Duration::from_millis(10));
    assert_eq!(clock.accumulated_time(), Duration::from_millis(10));
    println!("✓ Pause/resume works correctly\n");

    // Test 4: Scheduler
    println!("Test 4: DeterministicScheduler");
    let scheduler = deterministic::DeterministicScheduler::new(500.0)
        .with_seed(42)
        .with_config(deterministic::TickConfig::at_100_hz(
            deterministic::Subsystem::Sensors,
        ));

    assert!((scheduler.physics_hz() - 500.0).abs() < 0.1);
    assert_eq!(scheduler.random_seed(), 42);
    println!("✓ Scheduler created with 500 Hz physics and 100 Hz sensors\n");

    // Test 5: TimestepMode
    println!("Test 5: TimestepMode configurations");
    let fixed = deterministic::TimestepMode::fixed(500.0, 4);
    assert!(fixed.is_deterministic());

    let interpolated = deterministic::TimestepMode::interpolated(500.0, 100);
    assert!(interpolated.is_deterministic());

    let variable = deterministic::TimestepMode::Variable;
    assert!(!variable.is_deterministic());
    println!("✓ All timestep modes work correctly\n");

    println!("========================================");
    println!("All tests passed! ✓");
    println!("========================================");
}
