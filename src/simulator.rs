//! Physics simulator for humanoid robots
//! 
//! Uses Bevy for rendering and Rapier for physics

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use crate::robot::{HumanoidRobot, JointSet, RobotState, Pose, Joint, unitree_g1};
use crate::control::{Controller, Trajectory};
use std::time::Duration;

/// Plugin for the humanoid simulator
pub struct HumanoidSimulatorPlugin;

impl Plugin for HumanoidSimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
            .add_plugins(RapierDebugRenderPlugin::default())
            .add_systems(Startup, setup_simulator)
            .add_systems(Update, (
                update_robot_physics,
                apply_joint_commands,
                update_visualization,
            ));
    }
}

/// Resource holding the simulated robot
#[derive(Resource)]
pub struct SimulatedRobot {
    pub state: RobotState,
    pub controller: Option<Box<dyn Controller + Send + Sync>>,
    pub commands: Vec<f64>,
    pub entity: Option<Entity>,
}

impl SimulatedRobot {
    pub fn new() -> Self {
        Self {
            state: RobotState {
                joints: unitree_g1::create_default_joints(),
                ..Default::default()
            },
            controller: None,
            commands: vec![0.0; unitree_g1::NUM_JOINTS],
            entity: None,
        }
    }
    
    pub fn set_controller(&mut self, controller: Box<dyn Controller + Send + Sync>) {
        self.controller = Some(controller);
    }
}

/// Component for robot parts
#[derive(Component)]
pub struct RobotPart {
    pub joint_name: String,
    pub joint_index: usize,
}

/// Setup the simulator scene
fn setup_simulator(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut sim_robot: ResMut<SimulatedRobot>,
) {
    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-3.0, 3.0, 5.0)
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
    
    // Lights
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            ..default()
        },
        transform: Transform::from_xyz(-5.0, 10.0, 5.0)
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
    
    // Ground plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(Plane3d::default())),
            material: materials.add(Color::srgb(0.3, 0.5, 0.3)),
            transform: Transform::from_xyz(0.0, 0.0, 0.0)
                .with_scale(Vec3::splat(10.0)),
            ..default()
        },
        Collider::cuboid(5.0, 0.1, 5.0),
        RigidBody::Fixed,
    ));
    
    // Spawn robot base
    let robot_entity = spawn_robot_base(&mut commands, &mut meshes, &mut materials);
    sim_robot.entity = Some(robot_entity);
}

fn spawn_robot_base(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> Entity {
    // Simple box for pelvis/torso
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(0.3, 0.5, 0.2)),
            material: materials.add(Color::srgb(0.8, 0.2, 0.2)),
            transform: Transform::from_xyz(0.0, 1.0, 0.0),
            ..default()
        },
        RigidBody::Dynamic,
        Collider::cuboid(0.15, 0.25, 0.1),
        RobotPart {
            joint_name: "pelvis".to_string(),
            joint_index: 0,
        },
    )).id()
}

/// Update physics simulation
fn update_robot_physics(
    time: Res<Time>,
    mut sim_robot: ResMut<SimulatedRobot>,
) {
    let dt = Duration::from_secs_f64(time.delta_secs());
    
    // Update controller if present
    if let Some(ref mut controller) = sim_robot.controller {
        match controller.update(&sim_robot.state, dt) {
            Ok(output) => {
                // Extract commands from controller output
                sim_robot.commands = output.iter()
                    .map(|j| j.effort)
                    .collect();
            }
            Err(e) => {
                eprintln!("Controller error: {}", e);
            }
        }
    }
    
    // Update simulation time
    sim_robot.state.timestamp += dt;
}

/// Apply joint commands to physics bodies
fn apply_joint_commands(
    sim_robot: Res<SimulatedRobot>,
    mut query: Query<&mut ExternalForce, With<RobotPart>>,
) {
    // Apply forces from controller output
    for (mut force, i) in query.iter_mut().zip(0..) {
        if i < sim_robot.commands.len() {
            // Simplified - just apply torque around Y axis
            force.torque = Vec3::new(0.0, sim_robot.commands[i] as f32, 0.0);
        }
    }
}

/// Update visualization meshes
fn update_visualization(
    sim_robot: Res<SimulatedRobot>,
    mut query: Query<(&RobotPart, &mut Transform)>,
) {
    for (part, mut transform) in query.iter_mut() {
        if let Some(joint) = sim_robot.state.joints.get_joint(&part.joint_name) {
            // Update rotation based on joint position
            transform.rotation = Quat::from_rotation_y(joint.current_position as f32);
        }
    }
}

/// Run the simulator
pub fn run_simulator() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(SimulatedRobot::new())
        .add_plugins(HumanoidSimulatorPlugin)
        .run();
}

/// API for controlling the simulator from external code
impl SimulatedRobot {
    /// Set target joint positions
    pub fn set_target_positions(&mut self, positions: Vec<f64>) {
        if positions.len() == self.state.joints.iter().count() {
            for (joint, &pos) in self.state.joints.iter_mut().zip(positions.iter()) {
                joint.target_position = pos;
            }
        }
    }
    
    /// Get current state as JSON
    pub fn get_state_json(&self) -> String {
        serde_json::to_string(&self.state).unwrap_or_default()
    }
    
    /// Load trajectory
    pub fn load_trajectory(&mut self, trajectory: Trajectory) {
        if let Some(ref mut controller) = self.controller {
            controller.set_trajectory(trajectory);
        }
    }
}
