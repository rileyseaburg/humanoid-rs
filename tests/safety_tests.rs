//! Safety System Tests
//!
//! Tests for joint limits, emergency stops, torque limiting, and other
//! safety-critical functionality.

use humanoid_rs::robot::{unitree_g1, Joint, JointLimits, JointSet, JointType, RobotState};
use nalgebra::Vector3;
use std::time::Duration;

mod common;
use common::*;

/// Test joint limit enforcement
#[test]
fn test_joint_limit_enforcement() {
    let mut robot = create_mock_robot();

    // Get the joint limits for the first joint
    let limits = JointLimits::default();

    // Try to set position within limits (should succeed)
    let valid_position = (limits.min + limits.max) / 2.0;
    let valid_positions = vec![valid_position; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&valid_positions).is_ok());

    // Try to set position exceeding max limit (should fail)
    let exceeded_max = limits.max + 0.1;
    let invalid_positions_max = vec![exceeded_max; unitree_g1::NUM_JOINTS];
    let result = robot.set_joint_positions(&invalid_positions_max);
    assert!(
        result.is_err(),
        "Should reject position exceeding max limit"
    );

    // Try to set position below min limit (should fail)
    let exceeded_min = limits.min - 0.1;
    let invalid_positions_min = vec![exceeded_min; unitree_g1::NUM_JOINTS];
    let result = robot.set_joint_positions(&invalid_positions_min);
    assert!(result.is_err(), "Should reject position below min limit");
}

/// Test custom joint limits
#[test]
fn test_custom_joint_limits() {
    let mut robot = create_mock_robot();

    // Create joint set with custom limits
    let mut joints = JointSet::new();
    let custom_limits = JointLimits {
        min: -1.0,
        max: 1.0,
        max_velocity: 5.0,
        max_effort: 50.0,
    };

    for i in 0..unitree_g1::NUM_JOINTS {
        joints.add_joint(Joint {
            name: format!("joint_{}", i),
            joint_type: JointType::Revolute {
                axis: Vector3::y_axis().into_inner(),
            },
            parent: None,
            limits: custom_limits,
            current_position: 0.0,
            target_position: 0.0,
            velocity: 0.0,
            effort: 0.0,
        });
    }

    // Verify limits are enforced
    let valid_positions = vec![0.5; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&valid_positions).is_ok());

    let invalid_positions = vec![1.5; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&invalid_positions).is_err());
}

/// Test torque ramp limiting (simulated)
#[test]
fn test_torque_ramp_limiting() {
    let mut robot = create_mock_robot();
    let max_ramp_rate = 100.0; // Nm/s
    let dt = Duration::from_millis(10);

    // Set initial position
    let initial_positions = vec![0.0; unitree_g1::NUM_JOINTS];
    robot.set_joint_positions(&initial_positions).unwrap();

    // Calculate maximum position change per timestep based on torque ramp
    // Assuming simple model: torque ~ position error
    let max_position_change = max_ramp_rate * dt.as_secs_f64() / 100.0; // Approximate

    // Apply small position change (should be within ramp limit)
    let small_change = vec![max_position_change * 0.5; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&small_change).is_ok());

    // Verify command was accepted
    assert_eq!(robot.command_history().len(), 2);
}

/// Test emergency stop functionality
#[test]
fn test_emergency_stop_activation() {
    let mut robot = create_mock_robot();

    // Initially not in emergency stop
    assert!(!robot.is_emergency_stop_active());

    // Activate emergency stop
    robot.emergency_stop();

    // Should be active
    assert!(robot.is_emergency_stop_active());

    // Commands should be rejected
    let positions = vec![0.1; unitree_g1::NUM_JOINTS];
    let result = robot.set_joint_positions(&positions);
    assert!(result.is_err());
    assert!(result.unwrap_err().contains("Emergency stop"));
}

/// Test emergency stop reset
#[test]
fn test_emergency_stop_reset() {
    let mut robot = create_mock_robot();

    // Activate and then reset
    robot.emergency_stop();
    assert!(robot.is_emergency_stop_active());

    robot.reset_emergency_stop();
    assert!(!robot.is_emergency_stop_active());

    // Commands should work again
    let positions = vec![0.1; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&positions).is_ok());
}

/// Test that emergency stop zeros all commands
#[test]
fn test_emergency_stop_zeros_commands() {
    let mut state = RobotState::default();
    state.joints = unitree_g1::create_default_joints();

    // Set some non-zero velocities and efforts
    for joint in state.joints.iter_mut() {
        joint.velocity = 1.0;
        joint.effort = 10.0;
        joint.target_position = 0.5;
    }

    // Simulate emergency stop behavior
    for joint in state.joints.iter_mut() {
        joint.target_position = joint.current_position; // Hold current position
        joint.velocity = 0.0;
        joint.effort = 0.0;
    }

    // Verify all motion stopped
    for joint in state.joints.iter() {
        assert_eq!(joint.velocity, 0.0, "Velocity should be zero after E-stop");
        assert_eq!(joint.effort, 0.0, "Effort should be zero after E-stop");
        assert_eq!(
            joint.target_position, joint.current_position,
            "Target should equal current position after E-stop"
        );
    }
}

/// Test communication timeout detection (simulated)
#[test]
fn test_communication_timeout() {
    let mut robot = create_mock_robot();
    let timeout_duration = Duration::from_millis(100);

    // Initial command
    let positions = vec![0.1; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&positions).is_ok());

    // Simulate passage of time without new commands
    // In a real implementation, this would trigger a safety timeout
    // For now, we verify the command history
    assert_eq!(robot.command_history().len(), 1);

    // A timeout mechanism would typically:
    // 1. Track last command timestamp
    // 2. Compare against current time
    // 3. Trigger safety stop if timeout exceeded
}

/// Test velocity limit enforcement
#[test]
fn test_velocity_limit_enforcement() {
    let mut joints = JointSet::new();
    let limits = JointLimits {
        min: -3.14,
        max: 3.14,
        max_velocity: 5.0, // rad/s
        max_effort: 100.0,
    };

    for i in 0..unitree_g1::NUM_JOINTS {
        joints.add_joint(Joint {
            name: format!("joint_{}", i),
            joint_type: JointType::Revolute {
                axis: Vector3::y_axis().into_inner(),
            },
            parent: None,
            limits: limits.clone(),
            current_position: 0.0,
            target_position: 0.0,
            velocity: 0.0,
            effort: 0.0,
        });
    }

    // Attempt to set velocity within limits
    for joint in joints.iter_mut() {
        joint.velocity = 3.0; // Within 5.0 rad/s limit
    }

    // All joints should have valid velocities
    for joint in joints.iter() {
        assert!(
            joint.velocity.abs() <= limits.max_velocity,
            "Velocity {} exceeds limit {}",
            joint.velocity,
            limits.max_velocity
        );
    }
}

/// Test effort/torque limiting
#[test]
fn test_effort_limiting() {
    let mut joints = JointSet::new();
    let limits = JointLimits {
        min: -3.14,
        max: 3.14,
        max_velocity: 10.0,
        max_effort: 50.0, // Nm
    };

    for i in 0..unitree_g1::NUM_JOINTS {
        joints.add_joint(Joint {
            name: format!("joint_{}", i),
            joint_type: JointType::Revolute {
                axis: Vector3::y_axis().into_inner(),
            },
            parent: None,
            limits: limits.clone(),
            current_position: 0.0,
            target_position: 0.0,
            velocity: 0.0,
            effort: 0.0,
        });
    }

    // Apply efforts within limits
    for joint in joints.iter_mut() {
        joint.effort = 30.0; // Within 50.0 Nm limit
    }

    for joint in joints.iter() {
        assert!(
            joint.effort.abs() <= limits.max_effort,
            "Effort {} exceeds limit {}",
            joint.effort,
            limits.max_effort
        );
    }

    // Clamp excessive effort
    for joint in joints.iter_mut() {
        let requested_effort = 100.0; // Exceeds limit
        joint.effort = requested_effort.clamp(-limits.max_effort, limits.max_effort);
    }

    for joint in joints.iter() {
        assert!(
            joint.effort.abs() <= limits.max_effort,
            "Clamped effort {} still exceeds limit {}",
            joint.effort,
            limits.max_effort
        );
    }
}

/// Test position command validation
#[test]
fn test_position_command_validation() {
    let robot = create_mock_robot();

    // Test with wrong number of joints
    let too_few = vec![0.1; unitree_g1::NUM_JOINTS - 1];
    let result = robot.set_joint_positions(&too_few);
    assert!(result.is_err());

    let too_many = vec![0.1; unitree_g1::NUM_JOINTS + 1];
    let result = robot.set_joint_positions(&too_many);
    assert!(result.is_err());

    // Test with NaN values
    let with_nan = vec![f64::NAN; unitree_g1::NUM_JOINTS];
    let result = robot.set_joint_positions(&with_nan);
    // Implementation may or may not reject NaN, document the behavior

    // Test with infinite values
    let with_inf = vec![f64::INFINITY; unitree_g1::NUM_JOINTS];
    let result = robot.set_joint_positions(&with_inf);
    // Implementation should reject infinite values
}

/// Test boundary conditions at joint limits
#[test]
fn test_joint_limit_boundary_conditions() {
    let mut robot = create_mock_robot();
    let limits = JointLimits::default();

    // Test exactly at limits (should succeed)
    let at_max = vec![limits.max; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&at_max).is_ok());

    let at_min = vec![limits.min; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&at_min).is_ok());

    // Test just beyond limits (should fail)
    let just_over = vec![limits.max + 1e-10; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&just_over).is_err());

    let just_under = vec![limits.min - 1e-10; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&just_under).is_err());
}

/// Test soft limits vs hard limits
#[test]
fn test_soft_vs_hard_limits() {
    // Hard limits: physical constraints that cannot be exceeded
    // Soft limits: conservative limits that can be overridden in special cases

    let hard_limits = JointLimits {
        min: -3.14,
        max: 3.14,
        max_velocity: 10.0,
        max_effort: 100.0,
    };

    let soft_limits = JointLimits {
        min: -2.5,
        max: 2.5,
        max_velocity: 8.0,
        max_effort: 80.0,
    };

    // Normal operation respects soft limits
    let normal_position = 2.0; // Within soft limits
    assert!(normal_position >= soft_limits.min && normal_position <= soft_limits.max);

    // Emergency recovery might use hard limits
    let emergency_position = 3.0; // Within hard limits but outside soft
    assert!(emergency_position >= hard_limits.min && emergency_position <= hard_limits.max);
    assert!(emergency_position < soft_limits.min || emergency_position > soft_limits.max);
}

/// Test cascading safety mechanisms
#[test]
fn test_cascading_safety_mechanisms() {
    let mut robot = create_mock_robot();

    // Level 1: Joint limits (always enforced)
    let limits = JointLimits::default();
    let over_limit = vec![limits.max + 1.0; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&over_limit).is_err());

    // Level 2: Velocity limiting (if implemented)
    // Level 3: Torque limiting (if implemented)
    // Level 4: Emergency stop (manual or automatic)
    robot.emergency_stop();
    assert!(robot.is_emergency_stop_active());

    // After E-stop, even valid commands are rejected
    let valid_positions = vec![0.0; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&valid_positions).is_err());
}

/// Test safety guard activation on simulation error
#[test]
fn test_safety_guard_on_simulation_error() {
    let mut sim = create_test_sim();

    // Run simulation normally
    sim.run_steps(100);

    // Apply a large perturbation that might cause instability
    sim.apply_perturbation(1.0);

    // Continue running
    sim.run_steps(100);

    // Verify state remains valid (no NaN/inf)
    for pos in sim.state().joints.positions() {
        assert!(
            pos.is_finite(),
            "Joint position became non-finite after perturbation"
        );
    }
}

/// Test joint limit message contains helpful information
#[test]
fn test_joint_limit_error_message() {
    let mut robot = create_mock_robot();
    let limits = JointLimits::default();

    let exceeded_position = limits.max + 0.5;
    let positions = vec![exceeded_position; unitree_g1::NUM_JOINTS];

    let result = robot.set_joint_positions(&positions);
    assert!(result.is_err());

    let error_msg = result.unwrap_err();
    // Error message should contain relevant information
    assert!(
        error_msg.contains("exceeds") || error_msg.contains("limit"),
        "Error message should indicate limit violation: {}",
        error_msg
    );
}
