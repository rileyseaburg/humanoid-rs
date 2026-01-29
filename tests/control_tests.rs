//! Control System Tests
//!
//! Tests for PD controllers, trajectory following, and safety mechanisms.

use humanoid_rs::control::{Controller, JointSpaceController, MotionPlanner, Trajectory, Waypoint};
use humanoid_rs::robot::{unitree_g1, JointSet, Pose};
use std::time::Duration;

mod common;
use common::*;

/// Test PD controller convergence to target position
#[test]
fn test_pd_controller_convergence() {
    let kp = 50.0;
    let kd = 10.0;
    let mut controller = JointSpaceController::new(kp, kd);

    let mut state = RobotState::default();
    state.joints = unitree_g1::create_default_joints();

    // Set target trajectory to move all joints to 0.5
    let mut target = JointSet::new();
    for i in 0..unitree_g1::NUM_JOINTS {
        target.add_joint(humanoid_rs::robot::Joint {
            name: format!("joint_{}", i),
            joint_type: humanoid_rs::robot::JointType::Revolute {
                axis: nalgebra::Vector3::y_axis().into_inner(),
            },
            parent: None,
            limits: Default::default(),
            current_position: 0.5,
            target_position: 0.5,
            velocity: 0.0,
            effort: 0.0,
        });
    }

    let trajectory = create_trajectory_to_target(&target, Duration::from_secs(2));
    controller.set_trajectory(trajectory);

    // Simulate for 2 seconds
    let dt = Duration::from_millis(10);
    let steps = 200; // 2 seconds at 100Hz

    for _ in 0..steps {
        let result = controller.update(&state, dt);
        assert!(result.is_ok());

        let output = result.unwrap();

        // Simulate simple dynamics: update position based on effort
        for (joint_out, joint_state) in output.iter().zip(state.joints.iter_mut()) {
            joint_state.effort = joint_out.effort;
            joint_state.velocity = joint_out.effort * 0.01; // Simple integration
            joint_state.current_position += joint_state.velocity * dt.as_secs_f64();
        }
    }

    // Check that positions converged toward targets
    let target_positions = target.positions();
    let final_positions = state.joints.positions();

    for (i, (target_pos, final_pos)) in target_positions
        .iter()
        .zip(final_positions.iter())
        .enumerate()
    {
        let error = (target_pos - final_pos).abs();
        assert!(
            error < 0.1,
            "Joint {} did not converge: target={}, final={}, error={}",
            i,
            target_pos,
            final_pos,
            error
        );
    }
}

/// Test controller with different gain values
#[test]
fn test_pd_controller_gains() {
    let test_cases = vec![
        (10.0, 1.0, "low_gains"),
        (50.0, 5.0, "medium_gains"),
        (100.0, 10.0, "high_gains"),
    ];

    for (kp, kd, name) in test_cases {
        let mut controller = JointSpaceController::new(kp, kd);
        let mut state = RobotState::default();
        state.joints = unitree_g1::create_default_joints();

        // Set target
        let mut target = JointSet::new();
        for _ in 0..unitree_g1::NUM_JOINTS {
            target.add_joint(humanoid_rs::robot::Joint {
                name: "test".to_string(),
                joint_type: humanoid_rs::robot::JointType::Revolute {
                    axis: nalgebra::Vector3::y_axis().into_inner(),
                },
                parent: None,
                limits: Default::default(),
                current_position: 0.3,
                target_position: 0.3,
                velocity: 0.0,
                effort: 0.0,
            });
        }

        let trajectory = create_trajectory_to_target(&target, Duration::from_secs(1));
        controller.set_trajectory(trajectory);

        let dt = Duration::from_millis(10);
        let mut max_effort = 0.0;

        for _ in 0..100 {
            let result = controller.update(&state, dt).unwrap();

            // Track maximum effort
            for joint in result.iter() {
                max_effort = max_effort.max(joint.effort.abs());
            }

            // Simple state update
            for (out, state_joint) in result.iter().zip(state.joints.iter_mut()) {
                state_joint.current_position += out.effort * 0.001;
            }
        }

        // Higher gains should produce higher efforts
        println!("Test {}: max_effort={}", name, max_effort);
        assert!(
            max_effort > 0.0,
            "Controller should produce non-zero effort"
        );
    }
}

/// Test trajectory following accuracy
#[test]
fn test_trajectory_following_accuracy() {
    let mut planner = MotionPlanner::new(2.0, 5.0);

    let current = create_joint_set_at_positions(&vec![0.0; unitree_g1::NUM_JOINTS]);
    let target = create_joint_set_at_positions(&vec![0.5; unitree_g1::NUM_JOINTS]);

    let trajectory = planner.plan_joint_space(&current, &target, Duration::from_secs(2));

    // Verify trajectory has waypoints
    assert!(trajectory.duration().as_secs_f64() > 0.0);

    // Test interpolation at various times
    let test_times = vec![
        Duration::from_secs_f64(0.0),
        Duration::from_secs_f64(0.5),
        Duration::from_secs_f64(1.0),
        Duration::from_secs_f64(1.5),
        Duration::from_secs_f64(2.0),
    ];

    let mut prev_positions = vec![0.0; unitree_g1::NUM_JOINTS];

    for time in test_times {
        let waypoint = trajectory.interpolate(time);
        assert!(
            waypoint.is_some(),
            "Should have waypoint at time {:?}",
            time
        );

        let positions = waypoint.unwrap().joint_positions;
        assert_eq!(positions.len(), unitree_g1::NUM_JOINTS);

        // Positions should progress monotonically (roughly)
        let avg_pos = positions.iter().sum::<f64>() / positions.len() as f64;
        let prev_avg = prev_positions.iter().sum::<f64>() / prev_positions.len() as f64;

        if time > Duration::from_secs(0) {
            assert!(
                avg_pos >= prev_avg - 0.01,
                "Trajectory should generally progress forward: time={:?}, avg={}",
                time,
                avg_pos
            );
        }

        prev_positions = positions;
    }
}

/// Test Cartesian trajectory planning
#[test]
fn test_cartesian_trajectory_planning() {
    let planner = MotionPlanner::default();

    let start = Pose::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
    let end = Pose::new(0.5, 1.0, 0.3, 0.0, 0.0, 0.0);

    let trajectory = planner.plan_cartesian(&start, &end, Duration::from_secs(1));

    // Should have multiple waypoints
    assert!(trajectory.duration().as_secs_f64() >= 1.0);

    // Check start and end
    let start_wp = trajectory.interpolate(Duration::from_secs(0));
    let end_wp = trajectory.interpolate(Duration::from_secs(1));

    assert!(start_wp.is_some());
    assert!(end_wp.is_some());
}

/// Test trajectory interpolation accuracy
#[test]
fn test_trajectory_interpolation_accuracy() {
    let mut trajectory = Trajectory::new();

    // Create simple trajectory: 0 -> 1 over 1 second
    trajectory.add_waypoint(Waypoint {
        time: Duration::from_secs(0),
        joint_positions: vec![0.0; unitree_g1::NUM_JOINTS],
        base_pose: None,
    });

    trajectory.add_waypoint(Waypoint {
        time: Duration::from_secs(1),
        joint_positions: vec![1.0; unitree_g1::NUM_JOINTS],
        base_pose: None,
    });

    // Test midpoint
    let midpoint = trajectory.interpolate(Duration::from_millis(500));
    assert!(midpoint.is_some());

    let positions = midpoint.unwrap().joint_positions;
    for (i, &pos) in positions.iter().enumerate() {
        assert!(
            (pos - 0.5).abs() < 0.01,
            "Midpoint interpolation incorrect at joint {}: expected 0.5, got {}",
            i,
            pos
        );
    }
}

/// Test controller completion detection
#[test]
fn test_controller_is_complete() {
    let mut controller = JointSpaceController::new(50.0, 5.0);

    // Without trajectory, should be complete
    assert!(controller.is_complete());

    // With trajectory, should not be complete initially
    let mut target = JointSet::new();
    for _ in 0..unitree_g1::NUM_JOINTS {
        target.add_joint(humanoid_rs::robot::Joint {
            name: "test".to_string(),
            joint_type: humanoid_rs::robot::JointType::Revolute {
                axis: nalgebra::Vector3::y_axis().into_inner(),
            },
            parent: None,
            limits: Default::default(),
            current_position: 0.5,
            target_position: 0.5,
            velocity: 0.0,
            effort: 0.0,
        });
    }

    let trajectory = create_trajectory_to_target(&target, Duration::from_millis(100));
    controller.set_trajectory(trajectory);

    assert!(!controller.is_complete());

    // After running past duration, should be complete
    let mut state = RobotState::default();
    state.joints = unitree_g1::create_default_joints();

    for _ in 0..20 {
        // 200ms at 10ms steps
        let _ = controller.update(&state, Duration::from_millis(10));
    }

    assert!(controller.is_complete());
}

/// Test stand-up motion planning
#[test]
fn test_stand_up_motion_planning() {
    let planner = MotionPlanner::default();
    let current = create_joint_set_at_positions(&vec![0.0; unitree_g1::NUM_JOINTS]);

    let trajectory = planner.plan_stand_up(&current);

    assert!(trajectory.duration().as_secs_f64() > 0.0);

    // Final waypoint should have knee bend
    let final_wp = trajectory.interpolate(Duration::from_secs(2));
    assert!(final_wp.is_some());

    let positions = final_wp.unwrap().joint_positions;
    // Check knee positions (indices 3 and 9 for left and right knees)
    assert!(positions[3] > 0.0, "Left knee should be bent in stand pose");
    assert!(
        positions[9] > 0.0,
        "Right knee should be bent in stand pose"
    );
}

/// Test controller with empty joint set
#[test]
fn test_controller_empty_joints() {
    let mut controller = JointSpaceController::new(50.0, 5.0);
    let mut state = RobotState::default();
    // Empty joints
    state.joints = JointSet::new();

    let result = controller.update(&state, Duration::from_millis(10));
    assert!(result.is_ok());

    let output = result.unwrap();
    assert_eq!(output.iter().count(), 0);
}

/// Test trajectory with single waypoint
#[test]
fn test_trajectory_single_waypoint() {
    let mut trajectory = Trajectory::new();

    trajectory.add_waypoint(Waypoint {
        time: Duration::from_secs(0),
        joint_positions: vec![0.5; unitree_g1::NUM_JOINTS],
        base_pose: None,
    });

    // Interpolate at various times
    let wp_early = trajectory.interpolate(Duration::from_secs(0));
    let wp_late = trajectory.interpolate(Duration::from_secs(10));

    assert!(wp_early.is_some());
    assert!(wp_late.is_some());

    // Both should return the same single waypoint
    assert_eq!(
        wp_early.unwrap().joint_positions,
        vec![0.5; unitree_g1::NUM_JOINTS]
    );
    assert_eq!(
        wp_late.unwrap().joint_positions,
        vec![0.5; unitree_g1::NUM_JOINTS]
    );
}

/// Test controller stability with zero timestep
#[test]
fn test_controller_zero_timestep() {
    let mut controller = JointSpaceController::new(50.0, 5.0);
    let mut state = RobotState::default();
    state.joints = unitree_g1::create_default_joints();

    // Should handle zero timestep gracefully
    let result = controller.update(&state, Duration::from_secs(0));
    assert!(result.is_ok());
}

/// Test motion planner with zero duration
#[test]
fn test_motion_planner_zero_duration() {
    let planner = MotionPlanner::default();
    let current = create_joint_set_at_positions(&vec![0.0; unitree_g1::NUM_JOINTS]);
    let target = create_joint_set_at_positions(&vec![0.5; unitree_g1::NUM_JOINTS]);

    let trajectory = planner.plan_joint_space(&current, &target, Duration::from_secs(0));

    // Should still produce valid trajectory with start and end at t=0
    assert!(trajectory.duration().as_secs_f64() >= 0.0);
}

// Helper functions

fn create_trajectory_to_target(target: &JointSet, duration: Duration) -> Trajectory {
    let mut trajectory = Trajectory::new();

    trajectory.add_waypoint(Waypoint {
        time: Duration::from_secs(0),
        joint_positions: vec![0.0; unitree_g1::NUM_JOINTS],
        base_pose: None,
    });

    trajectory.add_waypoint(Waypoint {
        time: duration,
        joint_positions: target.positions(),
        base_pose: None,
    });

    trajectory
}

fn create_joint_set_at_positions(positions: &[f64]) -> JointSet {
    let mut joints = JointSet::new();

    for (i, &pos) in positions.iter().enumerate() {
        joints.add_joint(humanoid_rs::robot::Joint {
            name: format!("joint_{}", i),
            joint_type: humanoid_rs::robot::JointType::Revolute {
                axis: nalgebra::Vector3::y_axis().into_inner(),
            },
            parent: None,
            limits: Default::default(),
            current_position: pos,
            target_position: pos,
            velocity: 0.0,
            effort: 0.0,
        });
    }

    joints
}

use humanoid_rs::robot::RobotState;
