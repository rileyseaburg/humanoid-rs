//! Physics simulator for humanoid robots
//!
//! Uses Bevy for rendering and Rapier for physics
//!
//! Controls:
//! - WASD: Move camera
//! - Space: Stand up
//! - C: Crouch
//! - 1: Wave animation
//! - 2: T-pose
//! - 3: Lean forward
//! - 4: Sit pose
//! - 5: Jump pose
//! - W: Start/stop walking
//! - R: Reset robot
//! - Q/E: Rotate camera
//! - ESC: Exit

use crate::control::{Controller, Trajectory};
use crate::robot::{unitree_g1, RobotState};
use bevy::app::AppExit;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// Plugin for the humanoid simulator
pub struct HumanoidSimulatorPlugin;

impl Plugin for HumanoidSimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
            .add_plugins(RapierDebugRenderPlugin::default())
            .init_resource::<RobotCommands>()
            .add_systems(Startup, (setup_scene, setup_ui))
            .add_systems(
                Update,
                (
                    handle_keyboard_input,
                    update_robot_joints,
                    update_animations,
                    update_ui_text,
                    camera_controller,
                ),
            );
    }
}

/// Resource holding robot commands and state
#[derive(Resource, Default)]
pub struct RobotCommands {
    pub target_positions: Vec<f32>,
    pub current_animation: Option<Animation>,
    pub animation_time: f32,
}

/// Available animations
#[derive(Clone, Copy, PartialEq)]
pub enum Animation {
    StandUp,
    Crouch,
    Wave,
    TPose,
    LeanForward,
    SitPose,
    JumpPose,
    Walking,
    Reset,
}

/// Walking gait state
#[derive(Resource, Default)]
pub struct WalkingGait {
    pub is_walking: bool,
    pub gait_phase: f32,
    pub step_length: f32,
    pub step_height: f32,
}

/// Component marking the robot root
#[derive(Component)]
pub struct RobotRoot;

/// Component for robot body parts
#[derive(Component)]
pub struct RobotPart {
    pub name: String,
    pub joint_index: usize,
    pub is_joint: bool,
}

/// Component for joint rotation
#[derive(Component)]
pub struct JointRotation {
    pub axis: Vec3,
    pub current_angle: f32,
    pub target_angle: f32,
    pub speed: f32,
}

/// UI text component
#[derive(Component)]
pub struct UiText;

/// Camera controller component  
#[derive(Component)]
pub struct CameraController {
    pub distance: f32,
    pub angle_x: f32,
    pub angle_y: f32,
}

/// Setup the 3D scene
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut robot_commands: ResMut<RobotCommands>,
) {
    // Initialize joint positions (29 joints)
    robot_commands.target_positions = vec![0.0; unitree_g1::NUM_JOINTS];

    // Camera with orbit controller
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 1.5, 4.0)
                .looking_at(Vec3::new(0.0, 0.8, 0.0), Vec3::Y),
            ..default()
        },
        CameraController {
            distance: 4.0,
            angle_x: 0.0,
            angle_y: 0.3,
        },
    ));

    // Ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 300.0,
    });

    // Directional light (sun)
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 15000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Ground plane with grid pattern
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh().size(20.0, 20.0)),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.2, 0.25, 0.2),
                perceptual_roughness: 0.9,
                ..default()
            }),
            ..default()
        },
        Collider::cuboid(10.0, 0.01, 10.0),
        RigidBody::Fixed,
    ));

    // Grid lines on ground
    for i in -10..=10 {
        let color = if i == 0 {
            Color::rgb(0.5, 0.5, 0.5)
        } else {
            Color::rgb(0.3, 0.35, 0.3)
        };
        let mat = materials.add(StandardMaterial {
            base_color: color,
            unlit: true,
            ..default()
        });

        // X lines
        commands.spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(20.0, 0.005, 0.02)),
            material: mat.clone(),
            transform: Transform::from_xyz(0.0, 0.001, i as f32),
            ..default()
        });

        // Z lines
        commands.spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(0.02, 0.005, 20.0)),
            material: mat,
            transform: Transform::from_xyz(i as f32, 0.001, 0.0),
            ..default()
        });
    }

    // Spawn the humanoid robot
    spawn_humanoid(&mut commands, &mut meshes, &mut materials);
}

/// Spawn the full humanoid robot
fn spawn_humanoid(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    // Materials
    let body_material = materials.add(StandardMaterial {
        base_color: Color::rgb(0.15, 0.15, 0.18),
        metallic: 0.8,
        perceptual_roughness: 0.3,
        ..default()
    });

    let joint_material = materials.add(StandardMaterial {
        base_color: Color::rgb(0.8, 0.3, 0.1),
        metallic: 0.9,
        perceptual_roughness: 0.2,
        ..default()
    });

    let accent_material = materials.add(StandardMaterial {
        base_color: Color::rgb(0.1, 0.5, 0.9),
        metallic: 0.7,
        perceptual_roughness: 0.3,
        ..default()
    });

    // Robot root entity
    let robot_root = commands
        .spawn((
            SpatialBundle {
                transform: Transform::from_xyz(0.0, 0.0, 0.0),
                ..default()
            },
            RobotRoot,
        ))
        .id();

    // === PELVIS (root body part) ===
    let pelvis = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.28, 0.15, 0.18)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, 0.85, 0.0),
                ..default()
            },
            RobotPart {
                name: "pelvis".to_string(),
                joint_index: 0,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(robot_root).add_child(pelvis);

    // === TORSO ===
    let torso = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.30, 0.35, 0.20)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, 0.25, 0.0),
                ..default()
            },
            RobotPart {
                name: "torso".to_string(),
                joint_index: 12,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(pelvis).add_child(torso);

    // Chest accent
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(0.20, 0.08, 0.01)),
            material: accent_material.clone(),
            transform: Transform::from_xyz(0.0, 0.08, 0.11),
            ..default()
        })
        .set_parent(torso);

    // === HEAD ===
    let neck = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cylinder::new(0.05, 0.08)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.0, 0.22, 0.0),
                ..default()
            },
            RobotPart {
                name: "neck".to_string(),
                joint_index: 14,
                is_joint: true,
            },
        ))
        .id();
    commands.entity(torso).add_child(neck);

    let head = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.16, 0.18, 0.16)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, 0.14, 0.0),
                ..default()
            },
            RobotPart {
                name: "head".to_string(),
                joint_index: 14,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(neck).add_child(head);

    // Eyes (visor style)
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(0.12, 0.03, 0.01)),
            material: accent_material.clone(),
            transform: Transform::from_xyz(0.0, 0.02, 0.085),
            ..default()
        })
        .set_parent(head);

    // === LEFT LEG ===
    // Left hip joint
    let left_hip = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.06)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(-0.1, -0.08, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_hip".to_string(),
                joint_index: 0,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 3.0,
            },
        ))
        .id();
    commands.entity(pelvis).add_child(left_hip);

    // Left thigh
    let left_thigh = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.10, 0.28, 0.10)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.16, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_thigh".to_string(),
                joint_index: 0,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(left_hip).add_child(left_thigh);

    // Left knee joint
    let left_knee = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.055)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.0, -0.16, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_knee".to_string(),
                joint_index: 3,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 3.0,
            },
        ))
        .id();
    commands.entity(left_thigh).add_child(left_knee);

    // Left shin
    let left_shin = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.08, 0.26, 0.08)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.15, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_shin".to_string(),
                joint_index: 3,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(left_knee).add_child(left_shin);

    // Left ankle
    let left_ankle = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.04)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.0, -0.15, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_ankle".to_string(),
                joint_index: 4,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 3.0,
            },
        ))
        .id();
    commands.entity(left_shin).add_child(left_ankle);

    // Left foot
    commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.10, 0.04, 0.18)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.03, 0.04),
                ..default()
            },
            RobotPart {
                name: "left_foot".to_string(),
                joint_index: 4,
                is_joint: false,
            },
        ))
        .set_parent(left_ankle);

    // === RIGHT LEG ===
    // Right hip joint
    let right_hip = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.06)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.1, -0.08, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_hip".to_string(),
                joint_index: 6,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 3.0,
            },
        ))
        .id();
    commands.entity(pelvis).add_child(right_hip);

    // Right thigh
    let right_thigh = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.10, 0.28, 0.10)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.16, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_thigh".to_string(),
                joint_index: 6,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(right_hip).add_child(right_thigh);

    // Right knee joint
    let right_knee = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.055)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.0, -0.16, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_knee".to_string(),
                joint_index: 9,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 3.0,
            },
        ))
        .id();
    commands.entity(right_thigh).add_child(right_knee);

    // Right shin
    let right_shin = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.08, 0.26, 0.08)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.15, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_shin".to_string(),
                joint_index: 9,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(right_knee).add_child(right_shin);

    // Right ankle
    let right_ankle = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.04)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.0, -0.15, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_ankle".to_string(),
                joint_index: 10,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 3.0,
            },
        ))
        .id();
    commands.entity(right_shin).add_child(right_ankle);

    // Right foot
    commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.10, 0.04, 0.18)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.03, 0.04),
                ..default()
            },
            RobotPart {
                name: "right_foot".to_string(),
                joint_index: 10,
                is_joint: false,
            },
        ))
        .set_parent(right_ankle);

    // === LEFT ARM ===
    // Left shoulder
    let left_shoulder = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.055)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(-0.19, 0.12, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_shoulder".to_string(),
                joint_index: 15,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 4.0,
            },
        ))
        .id();
    commands.entity(torso).add_child(left_shoulder);

    // Left upper arm
    let left_upper_arm = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.07, 0.22, 0.07)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.13, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_upper_arm".to_string(),
                joint_index: 15,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(left_shoulder).add_child(left_upper_arm);

    // Left elbow
    let left_elbow = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.045)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.0, -0.13, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_elbow".to_string(),
                joint_index: 18,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 4.0,
            },
        ))
        .id();
    commands.entity(left_upper_arm).add_child(left_elbow);

    // Left forearm
    let left_forearm = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.06, 0.20, 0.06)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.12, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_forearm".to_string(),
                joint_index: 18,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(left_elbow).add_child(left_forearm);

    // Left hand
    commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.06, 0.10, 0.03)),
                material: accent_material.clone(),
                transform: Transform::from_xyz(0.0, -0.15, 0.0),
                ..default()
            },
            RobotPart {
                name: "left_hand".to_string(),
                joint_index: 19,
                is_joint: false,
            },
        ))
        .set_parent(left_forearm);

    // === RIGHT ARM ===
    // Right shoulder
    let right_shoulder = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.055)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.19, 0.12, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_shoulder".to_string(),
                joint_index: 22,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 4.0,
            },
        ))
        .id();
    commands.entity(torso).add_child(right_shoulder);

    // Right upper arm
    let right_upper_arm = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.07, 0.22, 0.07)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.13, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_upper_arm".to_string(),
                joint_index: 22,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(right_shoulder).add_child(right_upper_arm);

    // Right elbow
    let right_elbow = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.045)),
                material: joint_material.clone(),
                transform: Transform::from_xyz(0.0, -0.13, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_elbow".to_string(),
                joint_index: 25,
                is_joint: true,
            },
            JointRotation {
                axis: Vec3::X,
                current_angle: 0.0,
                target_angle: 0.0,
                speed: 4.0,
            },
        ))
        .id();
    commands.entity(right_upper_arm).add_child(right_elbow);

    // Right forearm
    let right_forearm = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.06, 0.20, 0.06)),
                material: body_material.clone(),
                transform: Transform::from_xyz(0.0, -0.12, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_forearm".to_string(),
                joint_index: 25,
                is_joint: false,
            },
        ))
        .id();
    commands.entity(right_elbow).add_child(right_forearm);

    // Right hand
    commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(0.06, 0.10, 0.03)),
                material: accent_material.clone(),
                transform: Transform::from_xyz(0.0, -0.15, 0.0),
                ..default()
            },
            RobotPart {
                name: "right_hand".to_string(),
                joint_index: 26,
                is_joint: false,
            },
        ))
        .set_parent(right_forearm);
}

/// Setup UI overlay
fn setup_ui(mut commands: Commands) {
    // Controls help text
    commands.spawn((
        TextBundle::from_sections([
            TextSection::new(
                "HUMANOID-RS SIMULATOR\n\n",
                TextStyle {
                    font_size: 24.0,
                    color: Color::rgb(0.1, 0.5, 0.9),
                    ..default()
                },
            ),
            TextSection::new(
                "Controls:\n",
                TextStyle {
                    font_size: 18.0,
                    color: Color::WHITE,
                    ..default()
                },
            ),
            TextSection::new(
                "[Space] Stand Up\n[C] Crouch\n[1] Wave\n[2] T-Pose\n[3] Lean Forward\n[4] Sit\n[5] Jump\n[W] Walk (toggle)\n[R] Reset\n[Q/E] Rotate Camera\n[ESC] Exit\n\n",
                TextStyle {
                    font_size: 16.0,
                    color: Color::rgb(0.8, 0.8, 0.8),
                    ..default()
                },
            ),
            TextSection::new(
                "Status: ",
                TextStyle {
                    font_size: 16.0,
                    color: Color::WHITE,
                    ..default()
                },
            ),
            TextSection::new(
                "Ready",
                TextStyle {
                    font_size: 16.0,
                    color: Color::rgb(0.2, 0.9, 0.2),
                    ..default()
                },
            ),
        ])
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        }),
        UiText,
    ));
}

/// Handle keyboard input
fn handle_keyboard_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut robot_commands: ResMut<RobotCommands>,
    mut exit: EventWriter<AppExit>,
) {
    // Exit
    if keyboard.just_pressed(KeyCode::Escape) {
        exit.send(AppExit::default());
    }

    // Animations
    if keyboard.just_pressed(KeyCode::Space) {
        robot_commands.current_animation = Some(Animation::StandUp);
        robot_commands.animation_time = 0.0;
    }

    if keyboard.just_pressed(KeyCode::KeyC) {
        robot_commands.current_animation = Some(Animation::Crouch);
        robot_commands.animation_time = 0.0;
    }

    if keyboard.just_pressed(KeyCode::Digit1) {
        robot_commands.current_animation = Some(Animation::Wave);
        robot_commands.animation_time = 0.0;
    }

    if keyboard.just_pressed(KeyCode::Digit2) {
        robot_commands.current_animation = Some(Animation::TPose);
        robot_commands.animation_time = 0.0;
    }

    if keyboard.just_pressed(KeyCode::Digit3) {
        robot_commands.current_animation = Some(Animation::LeanForward);
        robot_commands.animation_time = 0.0;
    }

    if keyboard.just_pressed(KeyCode::Digit4) {
        robot_commands.current_animation = Some(Animation::SitPose);
        robot_commands.animation_time = 0.0;
    }

    if keyboard.just_pressed(KeyCode::Digit5) {
        robot_commands.current_animation = Some(Animation::JumpPose);
        robot_commands.animation_time = 0.0;
    }

    if keyboard.just_pressed(KeyCode::KeyW) {
        if robot_commands.current_animation != Some(Animation::Walking) {
            robot_commands.current_animation = Some(Animation::Walking);
            robot_commands.animation_time = 0.0;
        } else {
            robot_commands.current_animation = Some(Animation::StandUp);
            robot_commands.animation_time = 0.0;
        }
    }

    if keyboard.just_pressed(KeyCode::KeyR) {
        robot_commands.current_animation = Some(Animation::Reset);
        robot_commands.animation_time = 0.0;
    }
}

/// Update animations
fn update_animations(
    time: Res<Time>,
    mut robot_commands: ResMut<RobotCommands>,
    mut joints: Query<(&RobotPart, &mut JointRotation)>,
) {
    let Some(animation) = robot_commands.current_animation else {
        return;
    };

    robot_commands.animation_time += time.delta_seconds();
    let t = robot_commands.animation_time;

    match animation {
        Animation::StandUp => {
            // Standing pose - slight arm position, straight legs
            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "left_hip" | "right_hip" => joint.target_angle = 0.0,
                    "left_knee" | "right_knee" => joint.target_angle = 0.0,
                    "left_ankle" | "right_ankle" => joint.target_angle = 0.0,
                    "left_shoulder" | "right_shoulder" => joint.target_angle = 0.15,
                    "left_elbow" | "right_elbow" => joint.target_angle = 0.1,
                    _ => {}
                }
            }
            if t > 1.0 {
                robot_commands.current_animation = None;
            }
        }

        Animation::Crouch => {
            // Crouching pose
            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "left_hip" | "right_hip" => joint.target_angle = -0.8,
                    "left_knee" | "right_knee" => joint.target_angle = 1.4,
                    "left_ankle" | "right_ankle" => joint.target_angle = -0.6,
                    "left_shoulder" | "right_shoulder" => joint.target_angle = 0.3,
                    "left_elbow" | "right_elbow" => joint.target_angle = 0.5,
                    _ => {}
                }
            }
            if t > 1.0 {
                robot_commands.current_animation = None;
            }
        }

        Animation::Wave => {
            // Wave animation - cycles the right arm
            let wave_angle = (t * 6.0).sin() * 0.4;

            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "right_shoulder" => joint.target_angle = -2.5, // Arm up
                    "right_elbow" => joint.target_angle = -0.5 + wave_angle, // Wave motion
                    "left_shoulder" => joint.target_angle = 0.15,
                    "left_elbow" => joint.target_angle = 0.1,
                    "left_hip" | "right_hip" => joint.target_angle = 0.0,
                    "left_knee" | "right_knee" => joint.target_angle = 0.0,
                    _ => {}
                }
            }

            if t > 3.0 {
                robot_commands.current_animation = Some(Animation::StandUp);
                robot_commands.animation_time = 0.0;
            }
        }

        Animation::TPose => {
            // T-pose - arms straight out
            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "left_shoulder" => {
                        joint.target_angle = 0.0;
                        joint.axis = Vec3::Z; // Rotate around Z for T-pose
                    }
                    "right_shoulder" => {
                        joint.target_angle = 0.0;
                        joint.axis = Vec3::Z;
                    }
                    "left_elbow" | "right_elbow" => joint.target_angle = 0.0,
                    "left_hip" | "right_hip" => joint.target_angle = 0.0,
                    "left_knee" | "right_knee" => joint.target_angle = 0.0,
                    _ => {}
                }
            }
            if t > 1.0 {
                robot_commands.current_animation = None;
            }
        }

        Animation::Reset => {
            // Reset all joints to zero
            for (_, mut joint) in joints.iter_mut() {
                joint.target_angle = 0.0;
                joint.axis = Vec3::X; // Reset axis
            }
            if t > 1.0 {
                robot_commands.current_animation = None;
            }
        }

        Animation::LeanForward => {
            // Leaning forward pose
            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "left_hip" | "right_hip" => joint.target_angle = 0.6,
                    "left_knee" | "right_knee" => joint.target_angle = 0.1,
                    "left_ankle" | "right_ankle" => joint.target_angle = -0.5,
                    "left_shoulder" | "right_shoulder" => joint.target_angle = 1.2,
                    "left_elbow" | "right_elbow" => joint.target_angle = 0.3,
                    _ => {}
                }
            }
            if t > 1.0 {
                robot_commands.current_animation = None;
            }
        }

        Animation::SitPose => {
            // Sitting pose
            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "left_hip" | "right_hip" => joint.target_angle = -1.2,
                    "left_knee" | "right_knee" => joint.target_angle = 1.6,
                    "left_ankle" | "right_ankle" => joint.target_angle = -1.0,
                    "left_shoulder" | "right_shoulder" => joint.target_angle = 0.2,
                    "left_elbow" | "right_elbow" => joint.target_angle = 0.8,
                    _ => {}
                }
            }
            if t > 1.0 {
                robot_commands.current_animation = None;
            }
        }

        Animation::JumpPose => {
            // Jump pose (mid-jump)
            let jump_phase = (t * 8.0).sin();

            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "left_hip" | "right_hip" => joint.target_angle = -0.3 * jump_phase,
                    "left_knee" | "right_knee" => joint.target_angle = 0.5 * jump_phase,
                    "left_ankle" | "right_ankle" => joint.target_angle = -0.3,
                    "left_shoulder" | "right_shoulder" => joint.target_angle = 1.5,
                    "left_elbow" | "right_elbow" => joint.target_angle = 0.8,
                    _ => {}
                }
            }
            if t > 2.0 {
                robot_commands.current_animation = Some(Animation::StandUp);
                robot_commands.animation_time = 0.0;
            }
        }

        Animation::Walking => {
            // Walking gait animation
            let gait_speed = 4.0;
            let phase = t * gait_speed;

            for (part, mut joint) in joints.iter_mut() {
                match part.name.as_str() {
                    "left_hip" => {
                        joint.target_angle = (phase).sin() * 0.5;
                    }
                    "right_hip" => {
                        joint.target_angle = (phase + std::f32::consts::PI).sin() * 0.5;
                    }
                    "left_knee" => {
                        let knee_bend = if (phase).sin() > 0.0 {
                            (phase).sin() * 0.8
                        } else {
                            0.1
                        };
                        joint.target_angle = knee_bend;
                    }
                    "right_knee" => {
                        let knee_bend = if (phase + std::f32::consts::PI).sin() > 0.0 {
                            (phase + std::f32::consts::PI).sin() * 0.8
                        } else {
                            0.1
                        };
                        joint.target_angle = knee_bend;
                    }
                    "left_ankle" => {
                        joint.target_angle = -(phase).sin() * 0.3;
                    }
                    "right_ankle" => {
                        joint.target_angle = -(phase + std::f32::consts::PI).sin() * 0.3;
                    }
                    "left_shoulder" => {
                        joint.target_angle = -(phase).sin() * 0.3 + 0.15;
                    }
                    "right_shoulder" => {
                        joint.target_angle = -(phase + std::f32::consts::PI).sin() * 0.3 + 0.15;
                    }
                    "left_elbow" | "right_elbow" => {
                        joint.target_angle = 0.3;
                    }
                    _ => {}
                }
            }
            // Walking is continuous, no ending
        }
    }
}

/// Update robot joint rotations
fn update_robot_joints(time: Res<Time>, mut joints: Query<(&mut Transform, &mut JointRotation)>) {
    let dt = time.delta_seconds();

    for (mut transform, mut joint) in joints.iter_mut() {
        // Smooth interpolation toward target
        let diff = joint.target_angle - joint.current_angle;
        joint.current_angle += diff * joint.speed * dt;

        // Apply rotation
        transform.rotation = Quat::from_axis_angle(joint.axis, joint.current_angle);
    }
}

/// Update UI text
fn update_ui_text(robot_commands: Res<RobotCommands>, mut query: Query<&mut Text, With<UiText>>) {
    for mut text in query.iter_mut() {
        if text.sections.len() >= 5 {
            let status = match robot_commands.current_animation {
                Some(Animation::StandUp) => ("Standing Up...", Color::rgb(0.9, 0.9, 0.2)),
                Some(Animation::Crouch) => ("Crouching...", Color::rgb(0.9, 0.6, 0.2)),
                Some(Animation::Wave) => ("Waving!", Color::rgb(0.2, 0.9, 0.9)),
                Some(Animation::TPose) => ("T-Pose", Color::rgb(0.9, 0.2, 0.9)),
                Some(Animation::LeanForward) => ("Leaning Forward", Color::rgb(0.9, 0.4, 0.2)),
                Some(Animation::SitPose) => ("Sitting", Color::rgb(0.6, 0.8, 0.9)),
                Some(Animation::JumpPose) => ("Jumping!", Color::rgb(0.9, 0.9, 0.2)),
                Some(Animation::Walking) => ("Walking", Color::rgb(0.2, 0.9, 0.5)),
                Some(Animation::Reset) => ("Resetting...", Color::rgb(0.9, 0.5, 0.2)),
                None => ("Ready", Color::rgb(0.2, 0.9, 0.2)),
            };
            text.sections[4].value = status.0.to_string();
            text.sections[4].style.color = status.1;
        }
    }
}

/// Camera orbit controller
fn camera_controller(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut CameraController)>,
) {
    for (mut transform, mut controller) in query.iter_mut() {
        let dt = time.delta_seconds();

        // Rotate camera with Q/E
        if keyboard.pressed(KeyCode::KeyQ) {
            controller.angle_x -= 1.5 * dt;
        }
        if keyboard.pressed(KeyCode::KeyE) {
            controller.angle_x += 1.5 * dt;
        }

        // Zoom with mouse wheel would go here

        // Update camera position
        let x = controller.angle_x.sin() * controller.distance;
        let z = controller.angle_x.cos() * controller.distance;
        let y = 1.5 + controller.angle_y * controller.distance;

        transform.translation = Vec3::new(x, y, z);
        transform.look_at(Vec3::new(0.0, 0.8, 0.0), Vec3::Y);
    }
}

/// Run the simulator
pub fn run_simulator() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Humanoid-RS Simulator".to_string(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(HumanoidSimulatorPlugin)
        .run();
}

/// Resource for SimulatedRobot (kept for API compatibility)
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

    pub fn set_target_positions(&mut self, positions: Vec<f64>) {
        if positions.len() == self.state.joints.iter().count() {
            for (joint, &pos) in self.state.joints.iter_mut().zip(positions.iter()) {
                joint.target_position = pos;
            }
        }
    }

    pub fn get_state_json(&self) -> String {
        serde_json::to_string(&self.state).unwrap_or_default()
    }

    pub fn load_trajectory(&mut self, trajectory: Trajectory) {
        if let Some(ref mut controller) = self.controller {
            controller.set_trajectory(trajectory);
        }
    }
}
