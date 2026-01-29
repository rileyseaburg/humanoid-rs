# Test Suite Documentation

This directory contains comprehensive tests for the humanoid-rs framework, including deterministic regression tests, control system tests, safety tests, integration tests, and performance benchmarks.

## Directory Structure

```
tests/
├── common/
│   └── mod.rs              # Shared test utilities and fixtures
├── determinism.rs          # Deterministic regression tests
├── control_tests.rs        # PD controller and trajectory tests
├── safety_tests.rs         # Joint limits and emergency stop tests
├── integration_tests.rs    # End-to-end integration tests
├── performance_targets.md  # Performance requirements and thresholds
└── golden/                 # Known-good state snapshots
    ├── physics_golden_100steps.json
    └── standup_trajectory_golden.json

benches/
├── control_loop.rs         # Control system benchmarks
├── physics_step.rs         # Physics simulation benchmarks
└── ik_solver.rs            # Inverse kinematics benchmarks

.github/workflows/
└── test.yml                # CI/CD configuration
```

## Running Tests

### Run All Tests

```bash
cargo test --all-features
```

### Run Specific Test Categories

```bash
# Deterministic regression tests
cargo test --test determinism --features simulator

# Control system tests
cargo test --test control_tests --features simulator

# Safety tests
cargo test --test safety_tests --features simulator

# Integration tests
cargo test --test integration_tests --features simulator
```

### Run with Output

```bash
cargo test -- --nocapture
```

## Running Benchmarks

### Run All Benchmarks

```bash
cargo bench
```

### Run Specific Benchmarks

```bash
cargo bench --bench control_loop
cargo bench --bench physics_step
cargo bench --bench ik_solver
```

### Generate HTML Reports

```bash
cargo bench -- --noplot  # Console output only
cargo bench              # With HTML reports in target/criterion/
```

## Golden Files

Golden files are known-good state snapshots used for regression testing. They are stored in `tests/golden/`.

### Regenerate Golden Files

```bash
# Regenerate all golden files
./regenerate_golden.sh

# Regenerate specific golden file
./regenerate_golden.sh test_physics_golden
```

Or use the CI trigger by including `[regenerate-golden]` in your commit message.

## Test Categories

### Determinism Tests (`determinism.rs`)

Verify that the simulation produces identical results with the same initial conditions:

- **test_simulation_determinism**: Two simulations with same seed produce identical states
- **test_determinism_with_trajectories**: Determinism with active trajectories
- **test_sensor_stability**: Sensor outputs are deterministic across runs
- **test_physics_golden**: Compare against known-good snapshots
- **test_long_run_stability**: No floating-point drift over 10,000 steps

### Control Tests (`control_tests.rs`)

Test control system functionality:

- **test_pd_controller_convergence**: PD controller reaches target
- **test_pd_controller_gains**: Different gain values produce expected behavior
- **test_trajectory_following_accuracy**: Trajectory interpolation works correctly
- **test_cartesian_trajectory_planning**: Cartesian motion planning
- **test_stand_up_motion_planning**: Stand-up trajectory generation

### Safety Tests (`safety_tests.rs`)

Verify safety mechanisms:

- **test_joint_limit_enforcement**: Positions outside limits are rejected
- **test_emergency_stop_activation**: E-stop prevents motion
- **test_emergency_stop_reset**: System recovery after E-stop
- **test_velocity_limit_enforcement**: Velocity constraints respected
- **test_effort_limiting**: Torque limits applied correctly

### Integration Tests (`integration_tests.rs`)

End-to-end system validation:

- **test_end_to_end_simulation**: Full control loop with physics
- **test_scene_loading**: Robot configuration validation
- **test_walking_gait_sequence**: Multi-step gait execution
- **test_state_serialization**: Save/load state round-trip
- **test_system_recovery**: Recovery from fault conditions

## Performance Benchmarks

### Control Loop Benchmarks

- **control_update**: PD controller update latency
- **trajectory_interpolation**: Path interpolation performance
- **full_control_loop**: Complete control loop timing
- **joint_set_operations**: Joint data structure performance
- **memory_allocation**: Allocation patterns and overhead

### Physics Step Benchmarks

- **physics_step**: Simulation step timing
- **physics_joint_configs**: Performance with different DOF counts
- **collision_detection**: Self-collision and environment checks
- **sensor_updates**: Sensor reading overhead
- **integration_methods**: Euler vs semi-implicit integration

### IK Solver Benchmarks

- **ik_convergence**: Iteration count vs accuracy tradeoff
- **chain_lengths**: Performance across different chain lengths
- **accuracy_speed_tradeoff**: Fast vs accurate configurations
- **batch_ik**: Multiple targets in sequence

## CI/CD Integration

Tests run automatically on:
- Pull requests to `main` and `develop`
- Pushes to `main` and `develop`
- Nightly schedule for benchmarks

### CI Jobs

1. **test**: Run all tests on stable and beta Rust
2. **coverage**: Generate code coverage reports
3. **benchmark**: Performance regression detection
4. **determinism-check**: Verify deterministic behavior
5. **safety-checks**: Safety validation and security audit
6. **memory-check**: Memory leak detection with valgrind

## Performance Targets

See `tests/performance_targets.md` for detailed requirements:

- **Control Loop**: < 500 µs latency (target: 2 kHz)
- **Physics Step**: < 500 µs for 29 joints
- **IK Solver**: < 1 ms for 3-DOF chains
- **Memory**: < 500 MB total system usage

## Writing New Tests

### Test Utilities

Use the `common` module for shared functionality:

```rust
mod common;
use common::*;

#[test]
fn my_test() {
    let mut sim = create_test_sim();
    let controller = create_test_controller();
    
    // Test code here
    sim.run_steps(100);
    assert_states_equal(&sim1, &sim2);
}
```

### Mock Robot

```rust
let mut robot = create_mock_robot();
robot.set_joint_positions(&positions).unwrap();
robot.emergency_stop();
assert!(robot.is_emergency_stop_active());
```

### Deterministic Testing

Always use seeded simulations for deterministic tests:

```rust
let mut sim = create_test_sim(); // Uses TEST_SEED (42)
// or
let mut sim = create_test_sim_with_seed(12345);
```

### Golden File Tests

```rust
#[test]
fn test_my_feature_golden() {
    let test_name = "my_feature_golden";
    let mut sim = create_test_sim();
    
    // Run simulation
    sim.run_steps(100);
    
    if golden_file_exists(test_name) {
        let golden = load_golden_state(test_name).unwrap();
        assert_robot_states_approx_equal(sim.state(), &golden, PHYSICS_EPSILON);
    } else {
        save_golden_state(test_name, sim.state()).unwrap();
    }
}
```

## Debugging Test Failures

### Determinism Failures

If determinism tests fail:
1. Check for non-deterministic operations (RNG without seed, time-based code)
2. Verify floating-point operations are consistent
3. Check for race conditions in multi-threaded code

### Golden File Mismatches

If golden file tests fail:
1. Check if failure is expected (feature change)
2. Review differences with: `diff tests/golden/file.json <(cargo test ...)`
3. Regenerate if intentional: `./regenerate_golden.sh test_name`

### Performance Regressions

If benchmarks show regression:
1. Check recent commits for performance-critical changes
2. Profile with `cargo flamegraph`
3. Review allocation patterns with `cargo dhat`

## Notes

- Tests require `--features simulator` for full functionality
- Debug builds are 10-100x slower than release
- Golden files should be committed to version control
- Benchmark baselines are tracked in CI
- All timing measurements assume release mode

## License

These tests are part of humanoid-rs and share the same license (MIT OR Apache-2.0).
