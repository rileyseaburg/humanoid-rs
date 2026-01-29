//! IK Solver Benchmarks
//!
//! Performance benchmarks for inverse kinematics algorithms.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use humanoid_rs::ik::{IkSolver, IkSolverConfig};
use humanoid_rs::robot::JointSet;
use nalgebra::Vector3;

/// Benchmark IK solver convergence speed
fn bench_ik_convergence(c: &mut Criterion) {
    let mut group = c.benchmark_group("ik_convergence");

    // Test with different numbers of iterations
    for max_iter in [10, 50, 100, 200] {
        let config = IkSolverConfig {
            max_iterations: max_iter,
            learning_rate: 0.1,
            joint_limits_penalty: 10.0,
        };
        let solver = IkSolver::new(config);

        group.bench_with_input(
            BenchmarkId::new("iterations", max_iter),
            &max_iter,
            |b, _| {
                let target = Vector3::new(0.5, 0.8, 0.2);
                b.iter(|| {
                    let _ =
                        solver.solve_arm_2dof(black_box(1.0), black_box(1.0), black_box(target));
                });
            },
        );
    }

    group.finish();
}

/// Benchmark different chain lengths
fn bench_chain_lengths(c: &mut Criterion) {
    let mut group = c.benchmark_group("ik_chain_lengths");

    // 2-DOF arm
    group.bench_function("arm_2dof", |b| {
        let solver = IkSolver::with_default_config();
        let target = Vector3::new(1.0, 1.0, 0.0);

        b.iter(|| {
            let _ = solver.solve_arm_2dof(black_box(1.0), black_box(1.0), black_box(target));
        });
    });

    // 3-DOF arm reach
    group.bench_function("arm_3dof_reach", |b| {
        let solver = IkSolver::with_default_config();
        let target = Vector3::new(0.5, 0.8, 0.3);
        let shoulder_pos = Vector3::new(0.2, 1.2, 0.0);
        let lengths = (0.3, 0.25, 0.1);

        b.iter(|| {
            let _ = solver.solve_arm_reach(
                black_box(target),
                black_box(shoulder_pos),
                black_box(lengths),
            );
        });
    });

    // 3-DOF leg
    group.bench_function("leg_3dof", |b| {
        let solver = IkSolver::with_default_config();
        let foot_target = Vector3::new(0.2, -0.8, 0.3);
        let hip_pos = Vector3::new(0.0, 0.85, 0.0);
        let lengths = (0.3, 0.3, 0.1);

        b.iter(|| {
            let _ = solver.solve_leg_step(
                black_box(foot_target),
                black_box(hip_pos),
                black_box(lengths),
            );
        });
    });

    group.finish();
}

/// Benchmark accuracy vs speed tradeoffs
fn bench_accuracy_speed_tradeoff(c: &mut Criterion) {
    let mut group = c.benchmark_group("ik_accuracy_speed");

    // Fast but less accurate
    let fast_config = IkSolverConfig {
        max_iterations: 50,
        learning_rate: 0.2,
        joint_limits_penalty: 5.0,
    };
    let fast_solver = IkSolver::new(fast_config);

    // Slow but accurate
    let accurate_config = IkSolverConfig {
        max_iterations: 200,
        learning_rate: 0.05,
        joint_limits_penalty: 20.0,
    };
    let accurate_solver = IkSolver::new(accurate_config);

    let target = Vector3::new(0.6, 0.9, 0.15);

    group.bench_function("fast_config", |b| {
        b.iter(|| {
            let _ = fast_solver.solve_arm_2dof(black_box(1.0), black_box(1.0), black_box(target));
        });
    });

    group.bench_function("accurate_config", |b| {
        b.iter(|| {
            let _ =
                accurate_solver.solve_arm_2dof(black_box(1.0), black_box(1.0), black_box(target));
        });
    });

    group.finish();
}

/// Benchmark CCD solver for articulated chains
fn bench_ccd_solver(c: &mut Criterion) {
    let mut group = c.benchmark_group("ccd_solver");

    for num_joints in [3, 5, 7, 10] {
        group.bench_with_input(
            BenchmarkId::new("chain_size", num_joints),
            &num_joints,
            |b, &num_joints| {
                let solver = IkSolver::with_default_config();
                let mut joints = create_joint_chain(num_joints);
                let end_effector = Vector3::new(0.8, 0.5, 0.0);
                let joint_indices: Vec<usize> = (0..num_joints).collect();
                let joint_axes: Vec<Vector3<f64>> =
                    vec![Vector3::z_axis().into_inner(); num_joints];

                b.iter(|| {
                    let mut joints_copy = joints.clone();
                    let _ = solver.solve_ccd(
                        &mut joints_copy,
                        black_box(end_effector),
                        &joint_indices,
                        &joint_axes,
                    );
                });
            },
        );
    }

    group.finish();
}

/// Benchmark different target positions
fn bench_target_positions(c: &mut Criterion) {
    let mut group = c.benchmark_group("ik_targets");

    let solver = IkSolver::with_default_config();

    // Easy target (reachable)
    group.bench_function("reachable_target", |b| {
        let target = Vector3::new(1.0, 1.0, 0.0);
        b.iter(|| {
            let _ = solver.solve_arm_2dof(black_box(1.0), black_box(1.0), black_box(target));
        });
    });

    // Difficult target (near singular)
    group.bench_function("near_singular", |b| {
        let target = Vector3::new(0.0, 2.0, 0.0);
        b.iter(|| {
            let _ = solver.solve_arm_2dof(black_box(1.0), black_box(1.0), black_box(target));
        });
    });

    // Impossible target (out of reach)
    group.bench_function("out_of_reach", |b| {
        let target = Vector3::new(10.0, 10.0, 0.0);
        b.iter(|| {
            let _ = solver.solve_arm_2dof(black_box(1.0), black_box(1.0), black_box(target));
        });
    });

    group.finish();
}

/// Benchmark solution validation
fn bench_solution_validation(c: &mut Criterion) {
    let mut group = c.benchmark_group("ik_validation");

    group.bench_function("validate_solution", |b| {
        let solver = IkSolver::with_default_config();
        let target = Vector3::new(0.8, 0.8, 0.0);
        let solution = solver.solve_arm_2dof(1.0, 1.0, target);

        b.iter(|| {
            if let Some((shoulder, elbow)) = black_box(solution) {
                // Compute forward kinematics
                let x = shoulder.cos() + (shoulder + elbow).cos();
                let y = shoulder.sin() + (shoulder + elbow).sin();
                let computed = Vector3::new(x, y, 0.0);

                // Compute error
                let error = (computed - target).norm();
                black_box(error);
            }
        });
    });

    group.finish();
}

/// Benchmark batch IK solving
fn bench_batch_ik(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_ik");

    // Generate batch of targets
    let targets: Vec<Vector3<f64>> = (0..100)
        .map(|i| {
            let angle = i as f64 * 0.1;
            Vector3::new(angle.cos() * 0.8, angle.sin() * 0.8 + 0.5, 0.0)
        })
        .collect();

    group.bench_function("solve_100_targets", |b| {
        let solver = IkSolver::with_default_config();

        b.iter(|| {
            let results: Vec<_> = targets
                .iter()
                .map(|&target| solver.solve_arm_2dof(1.0, 1.0, target))
                .collect();
            black_box(results);
        });
    });

    group.bench_function("solve_single_target", |b| {
        let solver = IkSolver::with_default_config();
        let target = targets[0];

        b.iter(|| {
            let result = solver.solve_arm_2dof(1.0, 1.0, target);
            black_box(result);
        });
    });

    group.finish();
}

/// Benchmark memory usage patterns
fn bench_ik_memory_patterns(c: &mut Criterion) {
    let mut group = c.benchmark_group("ik_memory");

    group.bench_function("allocate_per_solve", |b| {
        let solver = IkSolver::with_default_config();
        let target = Vector3::new(0.5, 0.8, 0.0);

        b.iter(|| {
            let mut joints = create_joint_chain(5);
            let joint_indices: Vec<usize> = (0..5).collect();
            let joint_axes = vec![Vector3::z_axis().into_inner(); 5];

            let _ = solver.solve_ccd(&mut joints, black_box(target), &joint_indices, &joint_axes);
        });
    });

    group.bench_function("reuse_allocation", |b| {
        let solver = IkSolver::with_default_config();
        let target = Vector3::new(0.5, 0.8, 0.0);
        let mut joints = create_joint_chain(5);
        let joint_indices: Vec<usize> = (0..5).collect();
        let joint_axes = vec![Vector3::z_axis().into_inner(); 5];

        b.iter(|| {
            let _ = solver.solve_ccd(&mut joints, black_box(target), &joint_indices, &joint_axes);
        });
    });

    group.finish();
}

/// Helper function to create a joint chain
fn create_joint_chain(num_joints: usize) -> JointSet {
    let mut joints = JointSet::new();

    for i in 0..num_joints {
        joints.add_joint(humanoid_rs::robot::Joint {
            name: format!("joint_{}", i),
            joint_type: humanoid_rs::robot::JointType::Revolute {
                axis: Vector3::z_axis().into_inner(),
            },
            parent: if i == 0 {
                None
            } else {
                Some(format!("joint_{}", i - 1))
            },
            limits: Default::default(),
            current_position: 0.0,
            target_position: 0.0,
            velocity: 0.0,
            effort: 0.0,
        });
    }

    joints
}

criterion_group!(
    benches,
    bench_ik_convergence,
    bench_chain_lengths,
    bench_accuracy_speed_tradeoff,
    bench_ccd_solver,
    bench_target_positions,
    bench_solution_validation,
    bench_batch_ik,
    bench_ik_memory_patterns
);

criterion_main!(benches);
