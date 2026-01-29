//! Walking Demo Example
//!
//! Demonstrates walking simulation using MPC (Model Predictive Control) controller,
//! gait generation, and footstep planning.
//!
//! Run with: `cargo run --example walking_demo --features simulator`

use humanoid_rs::control::{MotionPlanner, Trajectory};
use humanoid_rs::mpc::{FootstepPlanner, LipmState, MpcConfig, MpcController, ZmpReference};
use humanoid_rs::robot::{unitree_g1, JointSet, Pose, RobotState};
use humanoid_rs::whole_body::{ComState, FootContact, WholeBodyConfig, WholeBodyController};
use nalgebra::Vector3;
use std::thread;
use std::time::{Duration, Instant};

/// Walking gait parameters
struct GaitParameters {
    step_length: f64,   // meters
    step_width: f64,    // meters
    step_duration: f64, // seconds
    com_height: f64,    // meters
    walking_speed: f64, // m/s
}

impl Default for GaitParameters {
    fn default() -> Self {
        Self {
            step_length: 0.15,
            step_width: 0.20,
            step_duration: 0.4,
            com_height: 0.85,
            walking_speed: 0.3,
        }
    }
}

/// Walking controller combining MPC with whole-body control
struct WalkingController {
    mpc: MpcController,
    footstep_planner: FootstepPlanner,
    whole_body: WholeBodyController,
    gait_params: GaitParameters,
    current_phase: f64, // 0.0 to 1.0 within a gait cycle
    is_walking: bool,
}

impl WalkingController {
    fn new(gait_params: GaitParameters) -> Self {
        // Configure MPC
        let mpc_config = MpcConfig {
            horizon: 20,
            dt: 0.02,
            com_height: gait_params.com_height,
            weight_com_pos: 1.0,
            weight_com_vel: 0.1,
            weight_zmp: 10.0,
            weight_jerk: 0.001,
            zmp_margin: 0.08,
            gravity: 9.81,
        };

        let mpc = MpcController::new(mpc_config);

        // Configure footstep planner
        let footstep_planner = FootstepPlanner {
            step_length: gait_params.step_length,
            step_width: gait_params.step_width,
            step_duration: gait_params.step_duration,
            double_support_ratio: 0.2,
        };

        // Configure whole-body controller
        let whole_body_config = WholeBodyConfig::default();
        let whole_body = WholeBodyController::new(whole_body_config);

        Self {
            mpc,
            footstep_planner,
            whole_body,
            gait_params,
            current_phase: 0.0,
            is_walking: false,
        }
    }

    fn start_walking(&mut self, velocity: Vector3<f64>) {
        // Plan footsteps for walking
        let duration = 10.0; // Plan 10 seconds ahead
        let footsteps = self.footstep_planner.plan_walking(velocity, duration);

        // Generate ZMP reference from footsteps
        let zmp_ref = self.footstep_planner.footsteps_to_zmp(&footsteps, 0.02);

        // Set MPC reference
        self.mpc.set_reference(zmp_ref);

        self.is_walking = true;
        println!(
            "  Started walking with velocity: [{:.2}, {:.2}, {:.2}] m/s",
            velocity.x, velocity.y, velocity.z
        );
    }

    fn stop_walking(&mut self) {
        self.is_walking = false;
        self.current_phase = 0.0;

        // Set standing ZMP reference
        let zmp_ref = ZmpReference::standing(5.0, 0.02, Vector3::zeros());
        self.mpc.set_reference(zmp_ref);

        println!("  Stopped walking");
    }

    fn update(&mut self, state: &RobotState, dt: Duration) -> Vec<f64> {
        if !self.is_walking {
            // Return zero torques when not walking (gravity compensation would go here)
            return vec![0.0; unitree_g1::NUM_JOINTS];
        }

        // Update gait phase
        self.current_phase += dt.as_secs_f64() / self.gait_params.step_duration;
        if self.current_phase >= 1.0 {
            self.current_phase -= 1.0;
        }

        // Get current CoM state
        let com_state = LipmState {
            x: state.base_pose.position.x,
            x_dot: 0.0, // Would compute from velocity
            y: state.base_pose.position.z,
            y_dot: 0.0,
        };

        // Solve MPC to get ZMP command
        let (zmp_cmd, _com_vel_cmd) = self.mpc.solve(&com_state);

        // Update foot contacts based on gait phase
        let left_contact = FootContact {
            position: Vector3::new(
                -self.gait_params.step_width / 2.0,
                0.0,
                if self.current_phase < 0.5 {
                    0.0
                } else {
                    self.gait_params.step_length * 0.5
                },
            ),
            normal: Vector3::y(),
            in_contact: self.current_phase < 0.5 || self.current_phase > 0.9,
            force: if self.current_phase < 0.5 {
                Vector3::new(0.0, 200.0, 0.0)
            } else {
                Vector3::zeros()
            },
        };

        let right_contact = FootContact {
            position: Vector3::new(
                self.gait_params.step_width / 2.0,
                0.0,
                if self.current_phase >= 0.5 {
                    0.0
                } else {
                    self.gait_params.step_length * 0.5
                },
            ),
            normal: Vector3::y(),
            in_contact: self.current_phase >= 0.5 && self.current_phase < 0.9,
            force: if self.current_phase >= 0.5 && self.current_phase < 0.9 {
                Vector3::new(0.0, 200.0, 0.0)
            } else {
                Vector3::zeros()
            },
        };

        // Update whole-body controller
        self.whole_body
            .set_foot_contacts(left_contact, right_contact);

        // Set CoM target based on ZMP command
        let com_target = ComState {
            position: Vector3::new(zmp_cmd.x, self.gait_params.com_height, zmp_cmd.z),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        self.whole_body
            .set_com_target(com_target.position, com_target.velocity);

        // Compute whole-body control
        self.whole_body.compute(state, dt)
    }

    fn get_capture_point(&self, state: &RobotState) -> Vector3<f64> {
        let com_state = LipmState {
            x: state.base_pose.position.x,
            x_dot: 0.0,
            y: state.base_pose.position.z,
            y_dot: 0.0,
        };
        self.mpc.capture_point(&com_state)
    }
}

fn main() {
    println!("Humanoid-RS: Walking Demo");
    println!("=========================\n");

    // Create initial robot state
    let mut robot_state = RobotState {
        joints: unitree_g1::create_default_joints(),
        base_pose: Pose::new(0.0, 0.85, 0.0, 0.0, 0.0, 0.0),
        is_standing: true,
        is_balanced: true,
        foot_contacts: [true, true],
        ..Default::default()
    };

    // Initialize walking controller
    let gait_params = GaitParameters::default();
    println!("Initializing walking controller with parameters:");
    println!("  Step length: {:.2} m", gait_params.step_length);
    println!("  Step width: {:.2} m", gait_params.step_width);
    println!("  Step duration: {:.2} s", gait_params.step_duration);
    println!("  CoM height: {:.2} m", gait_params.com_height);
    println!("  Walking speed: {:.2} m/s\n", gait_params.walking_speed);

    let mut walking_controller = WalkingController::new(gait_params);

    // Simulation parameters
    let control_rate_hz = 500.0;
    let dt = Duration::from_secs_f64(1.0 / control_rate_hz);

    // Walking sequence
    let walking_sequence = vec![
        ("Standing", Duration::from_secs(2), Vector3::zeros()),
        (
            "Walking Forward",
            Duration::from_secs(5),
            Vector3::new(0.3, 0.0, 0.0),
        ),
        ("Standing", Duration::from_secs(2), Vector3::zeros()),
        (
            "Walking Backward",
            Duration::from_secs(5),
            Vector3::new(-0.2, 0.0, 0.0),
        ),
        ("Standing", Duration::from_secs(2), Vector3::zeros()),
        (
            "Walking Left",
            Duration::from_secs(5),
            Vector3::new(0.0, 0.0, 0.15),
        ),
        ("Standing", Duration::from_secs(2), Vector3::zeros()),
        (
            "Walking Right",
            Duration::from_secs(5),
            Vector3::new(0.0, 0.0, -0.15),
        ),
        ("Standing", Duration::from_secs(2), Vector3::zeros()),
    ];

    println!("Starting walking simulation sequence...\n");

    let mut total_steps = 0;

    for (phase_name, duration, velocity) in walking_sequence {
        println!("Phase: {} (duration: {:?})", phase_name, duration);

        if velocity.norm() > 0.01 {
            walking_controller.start_walking(velocity);
        } else {
            walking_controller.stop_walking();
        }

        let steps = (duration.as_secs_f64() * control_rate_hz) as usize;

        for step in 0..steps {
            // Update walking controller
            let torques = walking_controller.update(&robot_state, dt);

            // Simulate robot dynamics (simplified)
            simulate_dynamics(&mut robot_state, &torques, dt);

            // Print status periodically
            if step % (control_rate_hz as usize) == 0 && velocity.norm() > 0.01 {
                let capture_point = walking_controller.get_capture_point(&robot_state);
                println!(
                    "  t={:.1}s - CoM: [{:.2}, {:.2}], Capture Point: [{:.2}, {:.2}]",
                    total_steps as f64 / control_rate_hz,
                    robot_state.base_pose.position.x,
                    robot_state.base_pose.position.z,
                    capture_point.x,
                    capture_point.z
                );
            }

            total_steps += 1;

            // Maintain control rate
            thread::sleep(dt);
        }
    }

    println!("\nWalking simulation complete!");
    println!("\nKey Concepts Demonstrated:");
    println!("  1. MPC-based CoM trajectory tracking");
    println!("  2. Footstep planning for desired velocity");
    println!("  3. Gait phase management for alternating legs");
    println!("  4. Capture point (DCM) for stability prediction");
    println!("  5. Whole-body control for force distribution");

    println!("\nTo visualize walking in 3D simulator:");
    println!("  cargo run --bin humanoid-sim --features simulator");
    println!("  Then press 'W' to start walking animation");
}

/// Simulate simplified robot dynamics
fn simulate_dynamics(state: &mut RobotState, torques: &[f64], dt: Duration) {
    // Very simplified dynamics - real implementation would use full physics
    let dt_s = dt.as_secs_f64();

    // Update joint positions based on torques (PD-like behavior)
    for (i, joint) in state.joints.iter_mut().enumerate() {
        if i < torques.len() {
            // Simple integration: torque -> acceleration -> velocity -> position
            let accel = torques[i] / 10.0; // Assume inertia of 10 kg*m²
            joint.velocity += accel * dt_s;
            joint.velocity *= 0.99; // Damping
            joint.current_position += joint.velocity * dt_s;

            // Clamp to limits
            joint.current_position = joint
                .current_position
                .clamp(joint.limits.min, joint.limits.max);
        }
    }

    // Update CoM position slightly based on walking
    // Real implementation would compute from full kinematics
    state.base_pose.position.x += 0.001 * (state.timestamp.as_secs_f64() * 0.5).sin();
    state.base_pose.position.z += 0.0005 * (state.timestamp.as_secs_f64() * 0.5).cos();

    // Update timestamp
    state.timestamp += dt;
}
