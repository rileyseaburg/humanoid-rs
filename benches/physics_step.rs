//! Physics Step Benchmarks
//!
//! Performance benchmarks for physics simulation operations.

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use humanoid_rs::robot::{unitree_g1, JointSet, RobotState};
use std::time::Duration;

mod common;
use common::*;

/// Benchmark simulation step time for different robot configurations
fn bench_physics_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("physics_step");
    
    // Test with different timestep sizes
    for timestep_us in [500, 1000, 2000, 5000] {
        let dt = Duration::from_micros(timestep_us);
        let name = format!("{}us", timestep_us);
        
        group.bench_with_input(
            BenchmarkId::new("timestep", name),
            &dt,
            |b, dt| {
                let mut sim = create_test_sim();
                b.iter(|| {
                    sim.step(black_box(*dt));
                });
            },
        );
    }
    
    group.finish();
}

/// Benchmark physics with different numbers of active joints
fn bench_physics_joint_configurations(c: &mut Criterion) {
    let mut group = c.benchmark_group("physics_joint_configs");
    
    // Simulate different robot configurations
    for num_active_joints in [10, 20, 29] {
        let name = format!("{}_joints", num_active_joints);
        
        group.bench_with_input(
            BenchmarkId::new("active_joints", name),
            &num_active_joints,
            |b, &num_joints| {
                let mut sim = create_test_sim();
                // Set only some joints as active
                let targets: Vec<f64> = (0..unitree_g1::NUM_JOINTS)
                    .map(|i| if i < num_joints { 0.1 } else { 0.0 })
                    .collect();
                sim.set_target_positions(&targets);
                
                b.iter(|| {
                    sim.step(black_box(Duration::from_millis(1)));
                });
            },
        );
    }
    
    group.finish();
}

/// Benchmark collision detection cost (simulated)
fn bench_collision_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("collision_detection");
    
    group.bench_function("self_collision_check", |b| {
        let mut sim = create_test_sim();
        let positions = vec![0.5; unitree_g1::NUM_JOINTS];
        sim.set_target_positions(&positions);
        sim.run_steps(100); // Get into pose that might have collisions
        
        b.iter(|| {
            // Simulate collision check
            let state = black_box(sim.state());
            let mut collision_count = 0;
            for (i, pos) in state.joints.positions().iter().enumerate() {
                // Simple distance-based check
                for (j, other_pos) in state.joints.positions().iter().enumerate() {
                    if i != j && (pos - other_pos).abs() < 0.01 {
                        collision_count += 1;
                    }
                }
            }
            black_box(collision_count);
        });
    });
    
    group.bench_function("ground_contact_check", |b| {
        let sim = create_test_sim();
        
        b.iter(|| {
            // Simulate ground contact check for feet
            let state = black_box(sim.state());
            let foot_indices = [5, 11]; // Left and right ankle
            let mut contacts = 0;
            
            for &idx in &foot_indices {
                if let Some(joint) = state.joints.iter().nth(idx) {
                    // Check if foot is near ground
                    if joint.current_position.abs() < 0.1 {
                        contacts += 1;
                    }
                }
            }
            black_box(contacts);
        });
    });
    
    group.finish();
}

/// Benchmark sensor update overhead
fn bench_sensor_updates(c: &mut Criterion) {
    let mut group = c.benchmark_group("sensor_updates");
    
    group.bench_function("update_all_sensors", |b| {
        let mut sim = create_test_sim();
        
        b.iter(|| {
            // Simulate sensor readings
            let state = black_box(sim.state());
            
            // Joint encoders
            let positions: Vec<f64> = state.joints.positions();
            let velocities: Vec<f64> = state.joints.iter()
                .map(|j| j.velocity)
                .collect();
            let torques: Vec<f64> = state.joints.iter()
                .map(|j| j.effort)
                .collect();
            
            // IMU (simulated)
            let _imu_accel = nalgebra::Vector3::new(0.0, -9.81, 0.0);
            let _imu_gyro = nalgebra::Vector3::zeros();
            
            // Force sensors (simulated)
            let _foot_force_left = 0.0;
            let _foot_force_right = 0.0;
            
            black_box((positions, velocities, torques));
        });
    });
    
    group.bench_function("encoder_reading_only", |b| {
        let sim = create_test_sim();
        
        b.iter(|| {
            let state = black_box(sim.state());
            let positions = state.joints.positions();
            black_box(positions);
        });
    });
    
    group.finish();
}

/// Benchmark memory allocation during physics
fn bench_physics_memory(c: &mut Criterion) {
    let mut group = c.benchmark_group("physics_memory");
    
    group.bench_function("allocate_state_vector", |b| {
        b.iter(|| {
            let state: Vec<f64> = vec![0.0; unitree_g1::NUM_JOINTS * 3]; // pos, vel, acc
            black_box(state);
        });
    });
    
    group.bench_function("preallocated_update", |b| {
        let mut sim = create_test_sim();
        let mut state_buffer: Vec<f64> = vec![0.0; unitree_g1::NUM_JOINTS * 3];
        
        b.iter(|| {
            // Reuse preallocated buffer
            let state = black_box(sim.state());
            for (i, joint) in state.joints.iter().enumerate() {
                state_buffer[i * 3] = joint.current_position;
                state_buffer[i * 3 + 1] = joint.velocity;
                state_buffer[i * 3 + 2] = joint.effort;
            }
            black_box(&state_buffer);
        });
    });
    
    group.bench_function("allocate_per_step", |b| {
        let sim = create_test_sim();
        
        b.iter(|| {
            let state = black_box(sim.state());
            let mut buffer: Vec<f64> = Vec::with_capacity(unitree_g1::NUM_JOINTS * 3);
            for joint in state.joints.iter() {
                buffer.push(joint.current_position);
                buffer.push(joint.velocity);
                buffer.push(joint.effort);
            }
            black_box(buffer);
        });
    });
    
    group.finish();
}

/// Benchmark physics integration methods
fn bench_integration_methods(c: &mut Criterion) {
    let mut group = c.benchmark_group("integration_methods");
    
    group.bench_function("euler_integration", |b| {
        let dt = 0.001; // 1ms
        let mut pos = 0.0;
        let mut vel = 0.5;
        let acc = -9.81;
        
        b.iter(|| {
            // Euler integration
            pos += vel * dt;
            vel += acc * dt;
            black_box((pos, vel));
        });
    });
    
    group.bench_function("semi_implicit_euler", |b| {
        let dt = 0.001;
        let mut pos = 0.0;
        let mut vel = 0.5;
        let acc = -9.81;
        
        b.iter(|| {
            // Semi-implicit Euler (velocity first)
            vel += acc * dt;
            pos += vel * dt;
            black_box((pos, vel));
        });
    });
    
    group.finish();
}

/// Benchmark contact solving
fn bench_contact_solving(c: &mut Criterion) {
    let mut group = c.benchmark_group("contact_solving");
    
    group.bench_function("single_contact", |b| {
        let mut sim = create_test_sim();
        
        b.iter(|| {
            // Simulate single foot contact
            let state = black_box(sim.state());
            
            // Check if any joint is near limit
            let mut contacts = 0;
            for joint in state.joints.iter() {
                if joint.current_position.abs() > 3.0 {
                    contacts += 1;
                }
            }
            black_box(contacts);
        });
    });
    
    group.bench_function("double_support", |b| {
        let sim = create_test_sim();
        
        b.iter(|| {
            // Simulate both feet in contact
            let state = black_box(sim.state());
            
            // Check ankle positions
            let left_ankle = state.joints.iter().nth(5).map(|j| j.current_position);
            let right_ankle = state.joints.iter().nth(11).map(|j| j.current_position);
            
            let left_contact = left_ankle.map(|p| p.abs() < 0.1).unwrap_or(false);
            let right_contact = right_ankle.map(|p| p.abs() < 0.1).unwrap_or(false);
            
            black_box((left_contact, right_contact));
        });
    });
    
    group.finish();
}

/// Benchmark physics with different precision levels
fn bench_precision_levels(c: &mut Criterion) {
    let mut group = c.benchmark_group("precision_levels");
    
    group.bench_function("f32_simulation", |b| {
        let dt: f32 = 0.001;
        let mut pos: f32 = 0.0;
        let vel: f32 = 0.5;
        
        b.iter(|| {
            pos += vel * black_box(dt);
            black_box(pos);
        });
    });
    
    group.bench_function("f64_simulation", |b| {
        let dt: f64 = 0.001;
        let mut pos: f64 = 0.0;
        let vel: f64 = 0.5;
        
        b.iter(|| {
            pos += vel * black_box(dt);
            black_box(pos);
        });
    });
    
    group.finish();
}

/// Benchmark batch simulation steps
fn bench_batch_simulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_simulation");
    
    for batch_size in [10, 50, 100, 500] {
        group.bench_with_input(
            BenchmarkId::new("steps", batch_size),
            &batch_size,
            |b, &steps| {
                let mut sim = create_test_sim();
                let dt = Duration::from_millis(1);
                
                b.iter(|| {
                    for _ in 0..steps {
                        sim.step(black_box(dt));
                    }
                    black_box(sim.state());
                });
            },
        );
    }
    
    group.finish();
}

criterion_group!(
    benches,
    bench_physics_step,
    bench_physics_joint_configurations,
    bench_collision_detection,
    bench_sensor_updates,
    bench_physics_memory,
    bench_integration_methods,
    bench_contact_solving,
    bench_precision_levels,
    bench_batch_simulation
);

criterion_main!(benches);
