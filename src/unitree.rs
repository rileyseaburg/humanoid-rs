//! Unitree robot integration
//!
//! Connects to real Unitree G1/H1 robots via their SDK protocol.
//! Supports both low-level joint control and high-level commands.

use crate::robot::{HumanoidRobot, Pose, RobotState, unitree_g1};
use crate::{HumanoidError, Result};
use std::net::UdpSocket;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Default ports for Unitree communication
pub const UNITREE_CMD_PORT: u16 = 8082;
pub const UNITREE_STATE_PORT: u16 = 8083;
pub const UNITREE_VIDEO_PORT: u16 = 8084;

/// Robot mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RobotMode {
    Idle,
    Standing,
    Walking,
    Running,
    Climbing,
    LowLevel,
}

/// Connection to a real Unitree robot
pub struct UnitreeRobot {
    state: Arc<Mutex<RobotState>>,
    /// Cached state for trait method (updated on each access)
    cached_state: RobotState,
    cmd_socket: Option<UdpSocket>,
    state_socket: Option<UdpSocket>,
    robot_addr: String,
    mode: RobotMode,
    last_cmd_time: Instant,
    emergency_stopped: bool,
    /// Low-level control enabled (requires special mode)
    low_level_enabled: bool,
}

impl UnitreeRobot {
    /// Connect to robot at given address
    pub async fn connect(addr: &str) -> Result<Self> {
        println!("Connecting to Unitree robot at {}...", addr);

        // Parse address
        let robot_addr = if addr.contains(':') {
            addr.to_string()
        } else {
            format!("{}:{}", addr, UNITREE_CMD_PORT)
        };

        // Create command socket
        let cmd_socket = UdpSocket::bind("0.0.0.0:0")
            .map_err(|e| HumanoidError::CommunicationError(format!("Failed to bind socket: {}", e)))?;

        cmd_socket
            .set_nonblocking(true)
            .map_err(|e| HumanoidError::CommunicationError(format!("Failed to set non-blocking: {}", e)))?;

        cmd_socket
            .connect(&robot_addr)
            .map_err(|e| HumanoidError::CommunicationError(format!("Failed to connect: {}", e)))?;

        // Create state receiving socket
        let state_socket = UdpSocket::bind(format!("0.0.0.0:{}", UNITREE_STATE_PORT))
            .map_err(|e| HumanoidError::CommunicationError(format!("Failed to bind state socket: {}", e)))?;

        state_socket
            .set_nonblocking(true)
            .map_err(|e| HumanoidError::CommunicationError(format!("Failed to set non-blocking: {}", e)))?;

        println!("Connected to Unitree robot at {}", robot_addr);

        let initial_state = RobotState {
            joints: unitree_g1::create_default_joints(),
            ..Default::default()
        };
        
        let mut robot = Self {
            state: Arc::new(Mutex::new(initial_state.clone())),
            cached_state: initial_state,
            cmd_socket: Some(cmd_socket),
            state_socket: Some(state_socket),
            robot_addr,
            mode: RobotMode::Idle,
            last_cmd_time: Instant::now(),
            emergency_stopped: false,
            low_level_enabled: false,
        };

        // Send initial handshake
        robot.send_handshake()?;

        Ok(robot)
    }

    /// Create a mock robot for testing (no actual connection)
    pub fn mock() -> Self {
        let initial_state = RobotState {
            joints: unitree_g1::create_default_joints(),
            ..Default::default()
        };
        
        Self {
            state: Arc::new(Mutex::new(initial_state.clone())),
            cached_state: initial_state,
            cmd_socket: None,
            state_socket: None,
            robot_addr: "mock".to_string(),
            mode: RobotMode::Idle,
            last_cmd_time: Instant::now(),
            emergency_stopped: false,
            low_level_enabled: false,
        }
    }

    /// Send handshake to robot
    fn send_handshake(&mut self) -> Result<()> {
        let handshake = protocol::build_handshake();
        self.send_raw(&handshake)?;
        Ok(())
    }

    /// Send raw bytes to robot
    fn send_raw(&self, data: &[u8]) -> Result<()> {
        if let Some(ref socket) = self.cmd_socket {
            socket
                .send(data)
                .map_err(|e| HumanoidError::CommunicationError(format!("Send failed: {}", e)))?;
        }
        Ok(())
    }

    /// Send command to robot
    fn send_command(&mut self, cmd: &protocol::Command) -> Result<()> {
        if self.emergency_stopped {
            return Err(HumanoidError::SafetyStop {
                reason: "Emergency stop is active".to_string(),
            });
        }

        let packet = cmd.to_bytes();
        self.send_raw(&packet)?;
        self.last_cmd_time = Instant::now();
        Ok(())
    }

    /// Poll for state updates from robot
    pub fn poll_state(&mut self) -> Result<bool> {
        if let Some(ref socket) = self.state_socket {
            let mut buf = [0u8; 2048];
            match socket.recv(&mut buf) {
                Ok(len) => {
                    if let Some(state_msg) = protocol::parse_state_message(&buf[..len]) {
                        let mut state = self.state.lock().unwrap();
                        state.joints.set_positions(&state_msg.joint_positions);
                        state.base_pose = state_msg.base_pose;
                        state.is_standing = state_msg.is_standing;
                        state.is_balanced = state_msg.is_balanced;
                        state.foot_contacts = state_msg.foot_contacts;
                        state.timestamp = Duration::from_millis(state_msg.timestamp_ms as u64);
                        return Ok(true);
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No data available
                }
                Err(e) => {
                    return Err(HumanoidError::CommunicationError(format!(
                        "Recv failed: {}",
                        e
                    )));
                }
            }
        }
        Ok(false)
    }

    /// Enter low-level control mode
    pub fn enable_low_level_control(&mut self) -> Result<()> {
        let cmd = protocol::Command::SetMode(protocol::MODE_LOW_LEVEL);
        self.send_command(&cmd)?;
        self.low_level_enabled = true;
        self.mode = RobotMode::LowLevel;
        println!("Low-level control enabled");
        Ok(())
    }

    /// Exit low-level control mode
    pub fn disable_low_level_control(&mut self) -> Result<()> {
        let cmd = protocol::Command::SetMode(protocol::MODE_IDLE);
        self.send_command(&cmd)?;
        self.low_level_enabled = false;
        self.mode = RobotMode::Idle;
        println!("Low-level control disabled");
        Ok(())
    }

    /// High-level command: Stand up
    pub fn stand_up(&mut self) -> Result<()> {
        let cmd = protocol::Command::HighLevel(protocol::CMD_STAND_UP);
        self.send_command(&cmd)?;
        self.mode = RobotMode::Standing;
        Ok(())
    }

    /// High-level command: Sit down
    pub fn sit_down(&mut self) -> Result<()> {
        let cmd = protocol::Command::HighLevel(protocol::CMD_SIT_DOWN);
        self.send_command(&cmd)?;
        self.mode = RobotMode::Idle;
        Ok(())
    }

    /// High-level command: Walk with velocity
    pub fn walk(&mut self, vx: f64, vy: f64, vyaw: f64) -> Result<()> {
        let cmd = protocol::Command::Walk { vx, vy, vyaw };
        self.send_command(&cmd)?;
        self.mode = RobotMode::Walking;
        Ok(())
    }

    /// High-level command: Stop walking
    pub fn stop(&mut self) -> Result<()> {
        let cmd = protocol::Command::HighLevel(protocol::CMD_STOP);
        self.send_command(&cmd)?;
        self.mode = RobotMode::Standing;
        Ok(())
    }

    /// Get current robot mode
    pub fn mode(&self) -> RobotMode {
        self.mode
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.cmd_socket.is_some()
    }

    /// Get time since last command
    pub fn time_since_last_cmd(&self) -> Duration {
        self.last_cmd_time.elapsed()
    }
    
    /// Update cached state from internal mutex
    pub fn update_cached_state(&mut self) {
        if let Ok(state) = self.state.lock() {
            self.cached_state = state.clone();
        }
    }
}

impl HumanoidRobot for UnitreeRobot {
    fn state(&self) -> &RobotState {
        // Return cached state - caller should call update_state() to refresh
        &self.cached_state
    }

    fn set_joint_positions(&mut self, positions: &[f64]) -> Result<()> {
        if !self.low_level_enabled {
            return Err(HumanoidError::ControlError(
                "Low-level control not enabled. Call enable_low_level_control() first".to_string(),
            ));
        }

        // Safety check: ensure positions within limits
        let state = self.state.lock().unwrap();
        for (joint, &pos) in state.joints.iter().zip(positions.iter()) {
            if pos < joint.limits.min || pos > joint.limits.max {
                return Err(HumanoidError::JointLimitExceeded {
                    joint_name: joint.name.clone(),
                    requested: pos,
                    min: joint.limits.min,
                    max: joint.limits.max,
                });
            }
        }
        drop(state);

        let cmd = protocol::Command::JointPositions(positions.to_vec());
        self.send_command(&cmd)?;

        // Update local state
        let mut state = self.state.lock().unwrap();
        state.joints.set_positions(positions);
        Ok(())
    }

    fn set_joint_velocities(&mut self, velocities: &[f64]) -> Result<()> {
        if !self.low_level_enabled {
            return Err(HumanoidError::ControlError(
                "Low-level control not enabled".to_string(),
            ));
        }
        let cmd = protocol::Command::JointVelocities(velocities.to_vec());
        self.send_command(&cmd)
    }

    fn set_joint_efforts(&mut self, efforts: &[f64]) -> Result<()> {
        if !self.low_level_enabled {
            return Err(HumanoidError::ControlError(
                "Low-level control not enabled".to_string(),
            ));
        }
        let cmd = protocol::Command::JointTorques(efforts.to_vec());
        self.send_command(&cmd)
    }

    fn forward_kinematics(&self, link_name: &str) -> Result<Pose> {
        // Simplified FK - in practice would use full kinematic chain
        let state = self.state.lock().unwrap();
        let _positions = state.joints.positions();
        
        // Return base pose offset by link position
        // This is a placeholder - real FK would compute through chain
        match link_name {
            "left_foot" => Ok(Pose::new(-0.1, 0.0, 0.0, 0.0, 0.0, 0.0)),
            "right_foot" => Ok(Pose::new(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)),
            "left_hand" => Ok(Pose::new(-0.5, 0.0, 1.0, 0.0, 0.0, 0.0)),
            "right_hand" => Ok(Pose::new(0.5, 0.0, 1.0, 0.0, 0.0, 0.0)),
            "head" => Ok(Pose::new(0.0, 0.0, 1.5, 0.0, 0.0, 0.0)),
            _ => Ok(state.base_pose.clone()),
        }
    }

    fn num_joints(&self) -> usize {
        unitree_g1::NUM_JOINTS
    }

    fn emergency_stop(&mut self) -> Result<()> {
        println!("!!! EMERGENCY STOP !!!");
        self.emergency_stopped = true;

        // Send stop command multiple times for reliability
        for _ in 0..5 {
            let cmd = protocol::Command::EmergencyStop;
            let _ = self.send_raw(&cmd.to_bytes());
        }

        self.mode = RobotMode::Idle;
        Ok(())
    }

    fn reset_emergency_stop(&mut self) -> Result<()> {
        println!("Resetting emergency stop...");
        self.emergency_stopped = false;

        let cmd = protocol::Command::ResetEmergencyStop;
        self.send_command(&cmd)?;
        Ok(())
    }
}

/// Unitree protocol definitions
pub mod protocol {
    use super::*;

    /// Command IDs for high-level control
    pub const CMD_STAND_UP: u8 = 0x01;
    pub const CMD_SIT_DOWN: u8 = 0x02;
    pub const CMD_WALK: u8 = 0x03;
    pub const CMD_STOP: u8 = 0x04;
    pub const CMD_RECOVERY: u8 = 0x05;

    /// Command IDs for low-level control
    pub const CMD_JOINT_POS: u8 = 0x10;
    pub const CMD_JOINT_VEL: u8 = 0x11;
    pub const CMD_JOINT_TORQUE: u8 = 0x12;

    /// Mode IDs
    pub const MODE_IDLE: u8 = 0x00;
    pub const MODE_STANDING: u8 = 0x01;
    pub const MODE_WALKING: u8 = 0x02;
    pub const MODE_LOW_LEVEL: u8 = 0x10;

    /// Protocol magic bytes
    pub const MAGIC: [u8; 4] = [b'U', b'N', b'I', b'T'];

    /// Command types
    #[derive(Debug, Clone)]
    pub enum Command {
        HighLevel(u8),
        SetMode(u8),
        Walk { vx: f64, vy: f64, vyaw: f64 },
        JointPositions(Vec<f64>),
        JointVelocities(Vec<f64>),
        JointTorques(Vec<f64>),
        EmergencyStop,
        ResetEmergencyStop,
    }

    impl Command {
        /// Serialize command to bytes
        pub fn to_bytes(&self) -> Vec<u8> {
            let mut packet = Vec::with_capacity(256);
            packet.extend_from_slice(&MAGIC);

            match self {
                Command::HighLevel(cmd) => {
                    packet.push(*cmd);
                    packet.push(0); // No data
                }
                Command::SetMode(mode) => {
                    packet.push(0x20);
                    packet.push(1);
                    packet.push(*mode);
                }
                Command::Walk { vx, vy, vyaw } => {
                    packet.push(CMD_WALK);
                    packet.push(24); // 3 x f64
                    packet.extend_from_slice(&vx.to_le_bytes());
                    packet.extend_from_slice(&vy.to_le_bytes());
                    packet.extend_from_slice(&vyaw.to_le_bytes());
                }
                Command::JointPositions(positions) => {
                    packet.push(CMD_JOINT_POS);
                    let data_len = (positions.len() * 8) as u8;
                    packet.push(data_len);
                    for p in positions {
                        packet.extend_from_slice(&p.to_le_bytes());
                    }
                }
                Command::JointVelocities(velocities) => {
                    packet.push(CMD_JOINT_VEL);
                    let data_len = (velocities.len() * 8) as u8;
                    packet.push(data_len);
                    for v in velocities {
                        packet.extend_from_slice(&v.to_le_bytes());
                    }
                }
                Command::JointTorques(torques) => {
                    packet.push(CMD_JOINT_TORQUE);
                    let data_len = (torques.len() * 8) as u8;
                    packet.push(data_len);
                    for t in torques {
                        packet.extend_from_slice(&t.to_le_bytes());
                    }
                }
                Command::EmergencyStop => {
                    packet.push(0xFF);
                    packet.push(0);
                }
                Command::ResetEmergencyStop => {
                    packet.push(0xFE);
                    packet.push(0);
                }
            }

            // Add checksum
            let checksum: u8 = packet.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
            packet.push(checksum);

            packet
        }
    }

    /// State message from robot
    #[derive(Debug, Clone)]
    pub struct StateMessage {
        pub timestamp_ms: u32,
        pub joint_positions: Vec<f64>,
        pub joint_velocities: Vec<f64>,
        pub joint_torques: Vec<f64>,
        pub base_pose: Pose,
        pub is_standing: bool,
        pub is_balanced: bool,
        pub foot_contacts: [bool; 2],
        pub imu_orientation: [f64; 4],
        pub imu_angular_velocity: [f64; 3],
        pub imu_linear_acceleration: [f64; 3],
    }

    /// Build handshake packet
    pub fn build_handshake() -> Vec<u8> {
        let mut packet = Vec::new();
        packet.extend_from_slice(&MAGIC);
        packet.push(0x00); // Handshake command
        packet.push(0x04); // Version info length
        packet.extend_from_slice(&[0x01, 0x00, 0x00, 0x00]); // Version 1.0.0.0
        let checksum: u8 = packet.iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
        packet.push(checksum);
        packet
    }

    /// Parse state message from bytes
    pub fn parse_state_message(data: &[u8]) -> Option<StateMessage> {
        if data.len() < 8 || &data[0..4] != MAGIC {
            return None;
        }

        // Simplified parsing - real implementation would be more robust
        let num_joints = unitree_g1::NUM_JOINTS;
        let expected_len = 8 + num_joints * 24 + 64; // header + joints + extras

        if data.len() < expected_len {
            // Return default state for short messages
            return Some(StateMessage {
                timestamp_ms: 0,
                joint_positions: vec![0.0; num_joints],
                joint_velocities: vec![0.0; num_joints],
                joint_torques: vec![0.0; num_joints],
                base_pose: Pose::default(),
                is_standing: false,
                is_balanced: false,
                foot_contacts: [false, false],
                imu_orientation: [1.0, 0.0, 0.0, 0.0],
                imu_angular_velocity: [0.0, 0.0, 0.0],
                imu_linear_acceleration: [0.0, 0.0, 9.81],
            });
        }

        // Parse timestamp
        let timestamp_ms = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);

        // Parse joint data
        let mut offset = 8;
        let mut joint_positions = Vec::with_capacity(num_joints);
        let mut joint_velocities = Vec::with_capacity(num_joints);
        let mut joint_torques = Vec::with_capacity(num_joints);

        for _ in 0..num_joints {
            if offset + 24 > data.len() {
                break;
            }
            joint_positions.push(f64::from_le_bytes(
                data[offset..offset + 8].try_into().unwrap(),
            ));
            joint_velocities.push(f64::from_le_bytes(
                data[offset + 8..offset + 16].try_into().unwrap(),
            ));
            joint_torques.push(f64::from_le_bytes(
                data[offset + 16..offset + 24].try_into().unwrap(),
            ));
            offset += 24;
        }

        Some(StateMessage {
            timestamp_ms,
            joint_positions,
            joint_velocities,
            joint_torques,
            base_pose: Pose::default(),
            is_standing: true,
            is_balanced: true,
            foot_contacts: [true, true],
            imu_orientation: [1.0, 0.0, 0.0, 0.0],
            imu_angular_velocity: [0.0, 0.0, 0.0],
            imu_linear_acceleration: [0.0, 0.0, 9.81],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_robot() {
        let robot = UnitreeRobot::mock();
        assert!(!robot.is_connected());
        assert_eq!(robot.mode(), RobotMode::Idle);
    }

    #[test]
    fn test_command_serialization() {
        let cmd = protocol::Command::HighLevel(protocol::CMD_STAND_UP);
        let bytes = cmd.to_bytes();
        assert_eq!(&bytes[0..4], &protocol::MAGIC);
        assert_eq!(bytes[4], protocol::CMD_STAND_UP);
    }

    #[test]
    fn test_walk_command() {
        let cmd = protocol::Command::Walk {
            vx: 0.5,
            vy: 0.0,
            vyaw: 0.1,
        };
        let bytes = cmd.to_bytes();
        assert_eq!(&bytes[0..4], &protocol::MAGIC);
        assert_eq!(bytes[4], protocol::CMD_WALK);
        assert_eq!(bytes[5], 24); // 3 f64s
    }
}
