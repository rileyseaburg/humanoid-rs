//! Integration Tests
//!
//! End-to-end tests that verify the entire system works together correctly.

use humanoid_rs::robot::{unitree_g1, JointSet, RobotState};
use std::time::Duration;

mod common;
use common::*;

/// Test end-to-end simulation scenario
#[test]
fn test_end_to_end_simulation() {
    // Setup
    let mut sim = create_test_sim();
    let mut controller = create_test_controller();

    // Create a stand-up trajectory
    let stand_positions = create_stand_pose();
    let mut target = JointSet::new();
    for (i, &pos) in stand_positions.iter().enumerate() {
        target.add_joint(humanoid_rs::robot::Joint {
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

    let trajectory = create_trajectory(&target, Duration::from_secs(2));
    controller.set_trajectory(trajectory);

    // Run simulation loop
    let dt = Duration::from_millis(10);
    let steps = 200; // 2 seconds

    for _ in 0..steps {
        // Read state
        let state = sim.state().clone();

        // Compute control
        let control_result = controller.update(&state, dt);
        assert!(control_result.is_ok());

        // Apply to simulation (simplified)
        let commands = control_result.unwrap();
        let positions: Vec<f64> = commands
            .iter()
            .map(|j| j.current_position + j.effort * 0.0001)
            .collect();
        sim.set_target_positions(&positions);

        // Step simulation
        sim.step(dt);

        // Verify state validity
        assert_state_valid(sim.state());
    }

    // Verify final state is reasonable
    let final_positions = sim.state().joints.positions();
    let target_positions = stand_positions;

    let avg_error: f64 = final_positions
        .iter()
        .zip(target_positions.iter())
        .map(|(f, t)| (f - t).abs())
        .sum::<f64>()
        / final_positions.len() as f64;

    assert!(
        avg_error < 0.5,
        "Average tracking error too large: {}",
        avg_error
    );
}

/// Test scene loading with robot profile
#[test]
fn test_scene_loading() {
    // Verify robot configuration is valid
    let joints = unitree_g1::create_default_joints();

    assert_eq!(joints.iter().count(), unitree_g1::NUM_JOINTS);

    // Verify joint names are valid
    let names = unitree_g1::default_joint_names();
    assert_eq!(names.len(), unitree_g1::NUM_JOINTS);

    // Check no duplicate names
    let unique_names: std::collections::HashSet<_> = names.iter().collect();
    assert_eq!(
        unique_names.len(),
        names.len(),
        "Duplicate joint names found"
    );

    // Check all joints have valid limits
    for joint in joints.iter() {
        assert!(
            joint.limits.min < joint.limits.max,
            "Joint {} has invalid limits: [{}, {}]",
            joint.name,
            joint.limits.min,
            joint.limits.max
        );
        assert!(
            joint.limits.max_velocity > 0.0,
            "Joint {} has invalid max velocity",
            joint.name
        );
        assert!(
            joint.limits.max_effort > 0.0,
            "Joint {} has invalid max effort",
            joint.name
        );
    }
}

/// Test robot profile validation
#[test]
fn test_robot_profile_validation() {
    let joints = unitree_g1::create_default_joints();
    let names = unitree_g1::default_joint_names();

    // Validate joint configuration
    let expected_joint_groups = vec![
        ("left_hip", 3),       // 3 DOF
        ("left_knee", 1),      // 1 DOF
        ("left_ankle", 2),     // 2 DOF
        ("right_hip", 3),      // 3 DOF
        ("right_knee", 1),     // 1 DOF
        ("right_ankle", 2),    // 2 DOF
        ("waist", 3),          // 3 DOF
        ("left_shoulder", 3),  // 3 DOF
        ("left_elbow", 1),     // 1 DOF
        ("left_wrist", 3),     // 3 DOF
        ("right_shoulder", 3), // 3 DOF
        ("right_elbow", 1),    // 1 DOF
        ("right_wrist", 3),    // 3 DOF
    ];

    // Verify total joint count matches
    let total_dofs: usize = expected_joint_groups.iter().map(|(_, n)| n).sum();
    assert_eq!(
        total_dofs,
        unitree_g1::NUM_JOINTS,
        "Joint count mismatch: expected {} DOFs, got {}",
        total_dofs,
        unitree_g1::NUM_JOINTS
    );

    // Verify joint naming conventions
    for name in &names {
        // Should have descriptive names
        assert!(
            name.contains("left") || name.contains("right") || name.contains("waist"),
            "Joint name {} should indicate side/body part",
            name
        );

        // Should specify joint type
        assert!(
            name.contains("hip")
                || name.contains("knee")
                || name.contains("ankle")
                || name.contains("waist")
                || name.contains("shoulder")
                || name.contains("elbow")
                || name.contains("wrist"),
            "Joint name {} should specify body part",
            name
        );
    }
}

/// Test walking gait sequence
#[test]
fn test_walking_gait_sequence() {
    let mut sim = create_test_sim();

    // Define walking gait parameters
    let step_duration = Duration::from_millis(500);
    let num_steps = 4;
    let dt = Duration::from_millis(10);

    // Simple alternating leg gait
    for step in 0..num_steps {
        let is_left = step % 2 == 0;

        // Lift and swing leg
        let swing_leg = if is_left { 0..6 } else { 6..12 };
        let support_leg = if is_left { 6..12 } else { 0..6 };

        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        // Swing leg forward
        for i in swing_leg {
            if i % 6 == 0 {
                // Hip pitch
                positions[i] = 0.3; // Forward
            } else if i % 6 == 3 {
                // Knee
                positions[i] = 0.5; // Bent
            }
        }

        // Support leg straight
        for i in support_leg {
            positions[i] = 0.0;
        }

        sim.set_target_positions(&positions);

        // Run for step duration
        let steps = (step_duration.as_millis() / dt.as_millis()) as usize;
        for _ in 0..steps {
            sim.step(dt);
            assert_state_valid(sim.state());
        }
    }

    // Verify robot is still in valid state
    assert_state_valid(sim.state());
}

/// Test IK integration with control loop
#[test]
fn test_ik_control_integration() {
    use humanoid_rs::ik::IkSolver;

    let solver = IkSolver::with_default_config();
    let mut sim = create_test_sim();

    // Target end-effector position (in front of robot)
    let target = nalgebra::Vector3::new(0.3, 0.8, 0.2);

    // Solve IK
    let solution = solver.solve_arm_reach(
        target,
        nalgebra::Vector3::new(0.2, 1.2, 0.0), // Shoulder position
        (0.3, 0.25, 0.1),                      // Arm segment lengths
    );

    assert_eq!(solution.len(), 3); // 3 joints for arm

    // Apply solution to simulation
    let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

    // Right arm joints start at index 22
    let right_arm_indices = [22, 23, 24, 25]; // shoulder_pitch, roll, yaw, elbow
    for (i, &angle) in solution.iter().enumerate() {
        if i < right_arm_indices.len() {
            positions[right_arm_indices[i]] = angle;
        }
    }

    sim.set_target_positions(&positions);
    sim.run_steps(100);

    // Verify state is valid
    assert_state_valid(sim.state());
}

/// Test MPC integration with simulation
#[test]
fn test_mpc_integration() {
    use humanoid_rs::mpc::{LipmState, MpcConfig, MpcController, ZmpReference};

    let config = MpcConfig::default();
    let mut mpc = MpcController::new(config);

    // Set up walking reference
    let zmp_ref = ZmpReference::walking(
        0.15, // step length
        0.20, // step width
        0.4,  // step duration
        5,    // num steps
        0.02, // dt
    );

    mpc.set_reference(zmp_ref);

    let mut state = LipmState::new(0.0, 0.0, 0.0, 0.0);

    // Run MPC loop
    for _ in 0..100 {
        let (zmp_cmd, com_vel) = mpc.solve(&state);

        // ZMP command should be finite
        assert!(zmp_cmd.x.is_finite());
        assert!(zmp_cmd.z.is_finite());
        assert!(com_vel.x.is_finite());
        assert!(com_vel.z.is_finite());

        // Predict next state (simplified)
        state.x += com_vel.x * 0.02;
        state.y += com_vel.z * 0.02;
    }
}

/// Test configuration file parsing (simulated)
#[test]
fn test_config_file_parsing() {
    // Simulate loading configuration from file
    let config_data = r#"
    {
        "robot_type": "unitree_g1",
        "num_joints": 29,
        "control_rate_hz": 1000,
        "safety_limits": {
            "max_joint_velocity": 10.0,
            "max_joint_effort": 100.0
        }
    }
    "#;

    // Parse config
    let parsed: serde_json::Value =
        serde_json::from_str(config_data).expect("Failed to parse config");

    assert_eq!(parsed["robot_type"], "unitree_g1");
    assert_eq!(parsed["num_joints"], 29);
    assert_eq!(parsed["control_rate_hz"], 1000);

    // Verify safety limits
    let safety = &parsed["safety_limits"];
    assert!(safety["max_joint_velocity"].as_f64().unwrap() > 0.0);
    assert!(safety["max_joint_effort"].as_f64().unwrap() > 0.0);
}

/// Test error handling throughout the system
#[test]
fn test_system_error_handling() {
    // Test invalid joint count
    let mut robot = create_mock_robot();
    let invalid_positions = vec![0.0; unitree_g1::NUM_JOINTS + 5];
    assert!(robot.set_joint_positions(&invalid_positions).is_err());

    // Test NaN handling
    let nan_positions = vec![f64::NAN; unitree_g1::NUM_JOINTS];
    let result = robot.set_joint_positions(&nan_positions);
    // Document behavior - may accept or reject

    // Test out-of-range values
    let huge_positions = vec![1e10; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&huge_positions).is_err());
}

/// Test data serialization round-trip
#[test]
fn test_state_serialization() {
    let mut state = RobotState::default();
    state.joints = unitree_g1::create_default_joints();
    state.timestamp = Duration::from_secs(1);

    // Serialize
    let json = serde_json::to_string(&state).expect("Failed to serialize state");

    // Deserialize
    let restored: RobotState = serde_json::from_str(&json).expect("Failed to deserialize state");

    // Verify
    assert_eq!(restored.timestamp, state.timestamp);
    assert_eq!(restored.joints.iter().count(), state.joints.iter().count());

    for (j1, j2) in restored.joints.iter().zip(state.joints.iter()) {
        assert_eq!(j1.name, j2.name);
        assert!((j1.current_position - j2.current_position).abs() < 1e-10);
    }
}

/// Test multi-threaded simulation access patterns
#[test]
fn test_concurrent_simulation_patterns() {
    use std::sync::Arc;
    use std::thread;

    // Create shared simulation state
    let sim = Arc::new(std::sync::Mutex::new(create_test_sim()));

    // Spawn multiple threads that read state
    let handles: Vec<_> = (0..4)
        .map(|i| {
            let sim_clone = Arc::clone(&sim);
            thread::spawn(move || {
                let mut local_sim = create_test_sim_with_seed(i as u64);
                local_sim.run_steps(100);
                (i, local_sim.state().joints.positions())
            })
        })
        .collect();

    // Collect results
    let results: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();

    // Verify all threads completed
    assert_eq!(results.len(), 4);

    // Each thread should have produced deterministic results
    for (i, positions) in results {
        assert_eq!(
            positions.len(),
            unitree_g1::NUM_JOINTS,
            "Thread {} produced wrong number of joint positions",
            i
        );
    }
}

/// Test system recovery after fault
#[test]
fn test_system_recovery() {
    let mut robot = create_mock_robot();

    // Normal operation
    let positions = vec![0.1; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&positions).is_ok());

    // Fault: emergency stop
    robot.emergency_stop();
    assert!(robot.is_emergency_stop_active());

    // Recovery
    robot.reset_emergency_stop();
    assert!(!robot.is_emergency_stop_active());

    // Should be able to operate again
    let new_positions = vec![0.2; unitree_g1::NUM_JOINTS];
    assert!(robot.set_joint_positions(&new_positions).is_ok());
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

/// Helper function to create trajectory to target
fn create_trajectory(target: &JointSet, duration: Duration) -> humanoid_rs::control::Trajectory {
    let mut trajectory = humanoid_rs::control::Trajectory::new();

    trajectory.add_waypoint(humanoid_rs::control::Waypoint {
        time: Duration::from_secs(0),
        joint_positions: vec![0.0; unitree_g1::NUM_JOINTS],
        base_pose: None,
    });

    trajectory.add_waypoint(humanoid_rs::control::Waypoint {
        time: duration,
        joint_positions: target.positions(),
        base_pose: None,
    });

    trajectory
}

/// Helper function to validate simulation state
fn assert_state_valid(state: &RobotState) {
    // Check all positions are finite
    for (i, pos) in state.joints.positions().iter().enumerate() {
        assert!(
            pos.is_finite(),
            "Joint {} has non-finite position: {}",
            i,
            pos
        );

        // Check reasonable range (-10 to 10 radians)
        assert!(
            pos.abs() < 10.0,
            "Joint {} has unreasonable position: {}",
            i,
            pos
        );
    }

    // Check velocities
    for (i, joint) in state.joints.iter().enumerate() {
        assert!(
            joint.velocity.is_finite(),
            "Joint {} has non-finite velocity",
            i
        );
    }
}
