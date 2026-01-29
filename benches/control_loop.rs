//! Control Loop Benchmarks
//!
//! Performance benchmarks for control system operations using Criterion.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use humanoid_rs::control::{Controller, JointSpaceController, Trajectory};
use humanoid_rs::robot::{unitree_g1, RobotState};
use std::time::Duration;

/// Benchmark PD controller update performance
fn bench_control_update(c: &mut Criterion) {
    let mut group = c.benchmark_group("control_update");

    // Test with different controller gains
    for (kp, kd) in [(10.0, 1.0), (50.0, 5.0), (100.0, 10.0)] {
        let name = format!("kp{}_kd{}", kp, kd);
        let mut controller = JointSpaceController::new(kp, kd);
        let state = create_test_state();

        group.bench_with_input(
            BenchmarkId::new("pd_controller", name),
            &(kp, kd),
            |b, _| {
                b.iter(|| {
                    let _ = controller.update(black_box(&state), Duration::from_millis(1));
                });
            },
        );
    }

    group.finish();
}

/// Benchmark trajectory interpolation performance
fn bench_trajectory_interpolation(c: &mut Criterion) {
    let mut group = c.benchmark_group("trajectory_interpolation");

    // Create trajectory with varying numbers of waypoints
    for num_waypoints in [10, 50, 100, 500] {
        let trajectory = create_test_trajectory(num_waypoints);
        let test_time = Duration::from_millis(500);

        group.bench_with_input(
            BenchmarkId::new("interpolate", num_waypoints),
            &num_waypoints,
            |b, _| {
                b.iter(|| {
                    let _ = trajectory.interpolate(black_box(test_time));
                });
            },
        );
    }

    group.finish();
}

/// Benchmark full control loop including state reading
fn bench_full_control_loop(c: &mut Criterion) {
    let mut group = c.benchmark_group("full_control_loop");

    group.bench_function("control_loop_1khz", |b| {
        let mut controller = JointSpaceController::new(50.0, 5.0);
        let state = create_test_state();
        let dt = Duration::from_micros(1000); // 1kHz

        b.iter(|| {
            // Read state (simulated)
            let current_state = black_box(&state);

            // Compute control
            let output = controller.update(current_state, dt).unwrap();

            // Apply to system (black_box prevents optimization)
            black_box(output);
        });
    });

    group.bench_function("control_loop_2khz", |b| {
        let mut controller = JointSpaceController::new(50.0, 5.0);
        let state = create_test_state();
        let dt = Duration::from_micros(500); // 2kHz

        b.iter(|| {
            let output = controller.update(black_box(&state), dt).unwrap();
            black_box(output);
        });
    });

    group.finish();
}

/// Benchmark joint set operations
fn bench_joint_set_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("joint_set_operations");

    group.bench_function("get_positions", |b| {
        let state = create_test_state();
        b.iter(|| {
            let positions = black_box(&state).joints.positions();
            black_box(positions);
        });
    });

    group.bench_function("set_positions", |b| {
        let mut state = create_test_state();
        let positions: Vec<f64> = (0..unitree_g1::NUM_JOINTS)
            .map(|i| i as f64 * 0.01)
            .collect();

        b.iter(|| {
            black_box(&mut state).joints.set_positions(&positions);
        });
    });

    group.bench_function("iterate_joints", |b| {
        let state = create_test_state();
        b.iter(|| {
            let mut sum = 0.0;
            for joint in black_box(&state).joints.iter() {
                sum += joint.current_position;
            }
            black_box(sum);
        });
    });

    group.finish();
}

/// Benchmark state cloning (important for multi-threaded scenarios)
fn bench_state_cloning(c: &mut Criterion) {
    let mut group = c.benchmark_group("state_cloning");

    let state = create_test_state();

    group.bench_function("clone_state", |b| {
        b.iter(|| {
            let cloned = black_box(&state).clone();
            black_box(cloned);
        });
    });

    group.bench_function("clone_jointset", |b| {
        b.iter(|| {
            let cloned = black_box(&state).joints.clone();
            black_box(cloned);
        });
    });

    group.finish();
}

/// Benchmark memory allocation patterns
fn bench_memory_allocation(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_allocation");

    group.bench_function("allocate_joint_vec", |b| {
        b.iter(|| {
            let v: Vec<f64> = vec![0.0; unitree_g1::NUM_JOINTS];
            black_box(v);
        });
    });

    group.bench_function("reallocate_joint_vec", |b| {
        let mut v: Vec<f64> = Vec::with_capacity(unitree_g1::NUM_JOINTS);
        b.iter(|| {
            v.clear();
            for i in 0..unitree_g1::NUM_JOINTS {
                v.push(i as f64);
            }
            black_box(&v);
        });
    });

    group.bench_function("preallocated_joint_vec", |b| {
        let mut v: Vec<f64> = vec![0.0; unitree_g1::NUM_JOINTS];
        b.iter(|| {
            for i in 0..unitree_g1::NUM_JOINTS {
                v[i] = i as f64;
            }
            black_box(&v);
        });
    });

    group.finish();
}

/// Helper function to create test state
fn create_test_state() -> RobotState {
    let mut state = RobotState::default();
    state.joints = unitree_g1::create_default_joints();

    // Set some non-zero values
    for (i, joint) in state.joints.iter_mut().enumerate() {
        joint.current_position = i as f64 * 0.01;
        joint.velocity = i as f64 * 0.001;
        joint.effort = i as f64 * 0.1;
        joint.target_position = i as f64 * 0.01;
    }

    state
}

/// Helper function to create test trajectory
fn create_test_trajectory(num_waypoints: usize) -> Trajectory {
    let mut trajectory = Trajectory::new();
    let duration = Duration::from_secs(2);

    for i in 0..num_waypoints {
        let t = Duration::from_secs_f64((i as f64 / num_waypoints as f64) * duration.as_secs_f64());

        let positions: Vec<f64> = (0..unitree_g1::NUM_JOINTS)
            .map(|j| (i as f64 + j as f64) * 0.01)
            .collect();

        trajectory.add_waypoint(humanoid_rs::control::Waypoint {
            time: t,
            joint_positions: positions,
            base_pose: None,
        });
    }

    trajectory
}

criterion_group!(
    benches,
    bench_control_update,
    bench_trajectory_interpolation,
    bench_full_control_loop,
    bench_joint_set_operations,
    bench_state_cloning,
    bench_memory_allocation
);

criterion_main!(benches);
