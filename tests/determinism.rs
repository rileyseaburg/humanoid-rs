//! Deterministic Regression Tests
//!
//! These tests verify that the simulation produces identical results
//! when run with the same initial conditions and seed. This is critical
//! for debugging, reproducibility, and regression testing.

use humanoid_rs::robot::unitree_g1;
use std::time::Duration;

mod common;
use common::*;

/// Test that simulations with the same seed produce identical states
#[test]
fn test_simulation_determinism() {
    let mut sim1 = create_test_sim();
    let mut sim2 = create_test_sim();

    // Run both simulations with the same seed
    for step in 0..1000 {
        sim1.step(Duration::from_millis(1));
        sim2.step(Duration::from_millis(1));

        // Verify states match at every 100th step for early failure detection
        if step % 100 == 99 {
            assert_states_equal(&sim1, &sim2);
        }
    }

    // Final comprehensive comparison
    assert_states_equal(&sim1, &sim2);
}

/// Test determinism with different target trajectories
#[test]
fn test_determinism_with_trajectories() {
    let mut sim1 = create_test_sim();
    let mut sim2 = create_test_sim();

    // Set identical target positions
    let target_positions: Vec<f64> = (0..unitree_g1::NUM_JOINTS)
        .map(|i| 0.1 * (i as f64 + 1.0))
        .collect();

    sim1.set_target_positions(&target_positions);
    sim2.set_target_positions(&target_positions);

    // Run for 500 steps
    for _ in 0..500 {
        sim1.step(Duration::from_millis(1));
        sim2.step(Duration::from_millis(1));
    }

    assert_states_equal(&sim1, &sim2);
}

/// Test sensor output stability across multiple runs
#[test]
fn test_sensor_stability() {
    const NUM_RUNS: usize = 10;
    const STEPS_PER_RUN: usize = 100;

    let mut all_sensor_data: Vec<SensorData> = Vec::with_capacity(NUM_RUNS);

    for _ in 0..NUM_RUNS {
        let mut sim = create_test_sim();

        // Apply a deterministic perturbation
        sim.apply_perturbation(0.01);

        // Run simulation
        sim.run_steps(STEPS_PER_RUN);

        // Collect sensor data
        all_sensor_data.push(sim.sensor_data().clone());
    }

    // All runs should produce identical sensor data
    for i in 1..NUM_RUNS {
        assert_sensor_data_equal(&all_sensor_data[0], &all_sensor_data[i]);
    }
}

/// Test sensor determinism with varying update frequencies
#[test]
fn test_sensor_determinism_with_varying_frequency() {
    let mut sim1 = create_test_sim();
    let mut sim2 = create_test_sim();

    // Simulate different control loop frequencies
    let dt1 = Duration::from_micros(500); // 2kHz
    let dt2 = Duration::from_micros(1000); // 1kHz

    // Run for same total time (100ms)
    let steps1 = 200; // 200 * 0.5ms = 100ms
    let steps2 = 100; // 100 * 1ms = 100ms

    for _ in 0..steps1 {
        sim1.step(dt1);
    }

    for _ in 0..steps2 {
        sim2.step(dt2);
    }

    // States should be identical since total time is the same
    assert_states_equal(&sim1, &sim2);
}

/// Compare simulation against known-good (golden) state snapshots
#[test]
fn test_physics_golden() {
    let test_name = "physics_golden_100steps";
    let mut sim = create_test_sim();

    // Set up initial conditions
    let initial_positions: Vec<f64> = vec![0.05; unitree_g1::NUM_JOINTS];
    sim.set_target_positions(&initial_positions);

    // Run simulation
    sim.run_steps(100);

    let current_state = sim.state().clone();

    // Check if golden file exists
    if golden_file_exists(test_name) {
        // Compare against golden state
        let golden_state = load_golden_state(test_name).expect("Failed to load golden state");

        assert_robot_states_approx_equal(&current_state, &golden_state, PHYSICS_EPSILON);
    } else {
        // Create golden file for future comparisons
        save_golden_state(test_name, &current_state).expect("Failed to save golden state");
        println!("Created golden file for test: {}", test_name);
    }
}

/// Test golden state after stand-up trajectory
#[test]
fn test_standup_trajectory_golden() {
    let test_name = "standup_trajectory_golden";
    let mut sim = create_test_sim();

    // Set stand-up target positions
    let stand_positions = create_stand_pose();
    sim.set_target_positions(&stand_positions);

    // Run for 2 seconds
    for _ in 0..2000 {
        sim.step(Duration::from_millis(1));
    }

    let current_state = sim.state().clone();

    if golden_file_exists(test_name) {
        let golden_state = load_golden_state(test_name).expect("Failed to load golden state");
        assert_robot_states_approx_equal(&current_state, &golden_state, PHYSICS_EPSILON);
    } else {
        save_golden_state(test_name, &current_state).expect("Failed to save golden state");
        println!("Created golden file for test: {}", test_name);
    }
}

/// Test that different seeds produce different results
#[test]
fn test_different_seeds_produce_different_results() {
    let mut sim1 = create_test_sim_with_seed(42);
    let mut sim2 = create_test_sim_with_seed(43);

    // Apply perturbations
    sim1.apply_perturbation(0.1);
    sim2.apply_perturbation(0.1);

    // Run simulations
    sim1.run_steps(100);
    sim2.run_steps(100);

    // States should be different (with very high probability)
    let pos1 = sim1.state().joints.positions();
    let pos2 = sim2.state().joints.positions();

    let all_equal = pos1
        .iter()
        .zip(pos2.iter())
        .all(|(p1, p2)| (p1 - p2).abs() < FLOAT_EPSILON);
    assert!(
        !all_equal,
        "Different seeds should produce different results"
    );
}

/// Test determinism across multiple controller updates
#[test]
fn test_controller_determinism() {
    let mut sim1 = create_test_sim();
    let mut sim2 = create_test_sim();
    let mut controller1 = create_test_controller();
    let mut controller2 = create_test_controller();

    let trajectory = create_test_trajectory(1.0);
    controller1.set_trajectory(trajectory.clone());
    controller2.set_trajectory(trajectory);

    for _ in 0..100 {
        // Update controllers
        let dt = Duration::from_millis(10);
        let _ = controller1.update(sim1.state(), dt);
        let _ = controller2.update(sim2.state(), dt);

        // Step simulations
        sim1.step(dt);
        sim2.step(dt);
    }

    assert_states_equal(&sim1, &sim2);
}

/// Test determinism with IK solver
#[test]
fn test_ik_determinism() {
    let solver = create_test_ik_solver();

    // Test multiple identical queries
    let target1 = nalgebra::Vector3::new(0.5, 0.8, 0.3);
    let target2 = nalgebra::Vector3::new(0.5, 0.8, 0.3);

    let result1 = solver.solve_arm_2dof(1.0, 1.0, target1);
    let result2 = solver.solve_arm_2dof(1.0, 1.0, target2);

    assert_eq!(
        result1, result2,
        "IK solver should produce deterministic results"
    );
}

/// Test determinism with MPC controller
#[test]
fn test_mpc_determinism() {
    let mut controller1 = create_test_mpc_controller();
    let mut controller2 = create_test_mpc_controller();

    let reference = create_standing_reference(2.0);
    controller1.set_reference(reference.clone());
    controller2.set_reference(reference);

    let state = LipmState::new(0.01, 0.0, 0.0, 0.0);

    // Run multiple solves
    for _ in 0..50 {
        let (zmp1, vel1) = controller1.solve(&state);
        let (zmp2, vel2) = controller2.solve(&state);

        assert!((zmp1.x - zmp2.x).abs() < FLOAT_EPSILON);
        assert!((zmp1.z - zmp2.z).abs() < FLOAT_EPSILON);
        assert!((vel1.x - vel2.x).abs() < FLOAT_EPSILON);
        assert!((vel1.z - vel2.z).abs() < FLOAT_EPSILON);
    }
}

/// Helper function to create stand pose
fn create_stand_pose() -> Vec<f64> {
    let mut pose = vec![0.0; unitree_g1::NUM_JOINTS];
    // Slight knee bend
    pose[3] = 0.3; // left_knee
    pose[9] = 0.3; // right_knee
                   // Arms down
    pose[15] = 0.2; // left_shoulder_pitch
    pose[22] = 0.2; // right_shoulder_pitch
    pose
}

/// Regression test: Ensure no floating-point drift over long runs
#[test]
fn test_long_run_stability() {
    let mut sim = create_test_sim();

    // Set a target position
    let targets: Vec<f64> = vec![0.2; unitree_g1::NUM_JOINTS];
    sim.set_target_positions(&targets);

    // Run for 10,000 steps (10 seconds at 1kHz)
    let initial_positions = sim.state().joints.positions().clone();
    sim.run_steps(10000);

    let final_positions = sim.state().joints.positions();

    // Verify no NaN or infinite values
    for (i, pos) in final_positions.iter().enumerate() {
        assert!(pos.is_finite(), "Joint {} has non-finite value: {}", i, pos);
    }

    // Verify joints moved toward target
    for (i, (initial, final_pos)) in initial_positions
        .iter()
        .zip(final_positions.iter())
        .enumerate()
    {
        let target = 0.2;
        let initial_error = (initial - target).abs();
        let final_error = (final_pos - target).abs();

        assert!(
            final_error <= initial_error || final_error < 0.01,
            "Joint {} did not converge toward target: initial={}, final={}, target={}",
            i,
            initial,
            final_pos,
            target
        );
    }
}

/// Test that simulation is deterministic even with concurrent access simulation
#[test]
fn test_determinism_with_sequential_simulations() {
    const NUM_SIMULATIONS: usize = 5;
    const STEPS: usize = 500;

    let target_positions: Vec<f64> = vec![0.15; unitree_g1::NUM_JOINTS];
    let mut final_states: Vec<Vec<f64>> = Vec::with_capacity(NUM_SIMULATIONS);

    for _ in 0..NUM_SIMULATIONS {
        let mut sim = create_test_sim();
        sim.set_target_positions(&target_positions);
        sim.run_steps(STEPS);
        final_states.push(sim.state().joints.positions());
    }

    // All final states should be identical
    for i in 1..NUM_SIMULATIONS {
        for (j, (p1, p2)) in final_states[0]
            .iter()
            .zip(final_states[i].iter())
            .enumerate()
        {
            assert!(
                (p1 - p2).abs() < FLOAT_EPSILON,
                "Simulation {} differs at joint {}: {} vs {}",
                i,
                j,
                p1,
                p2
            );
        }
    }
}
