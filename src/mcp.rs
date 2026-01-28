//! MCP (Model Context Protocol) Integration for LLM Control
//!
//! Provides MCP tools that allow LLMs to control the humanoid robot.
//! Implements JSON-RPC handlers for robot actions like standing, walking, waving, etc.

use crate::robot::{unitree_g1, HumanoidRobot, JointSet, Pose, RobotState};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// MCP Tool definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct McpTool {
    /// Tool name
    pub name: String,
    /// Human-readable description
    pub description: String,
    /// Input schema (JSON Schema)
    pub input_schema: Value,
}

/// MCP Tool result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct McpToolResult {
    /// Whether the call was successful
    pub success: bool,
    /// Result content
    pub content: Vec<McpContent>,
    /// Error message if failed
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Content block in MCP response
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum McpContent {
    #[serde(rename = "text")]
    Text { text: String },
    #[serde(rename = "image")]
    Image { data: String, mime_type: String },
    #[serde(rename = "resource")]
    Resource { uri: String, text: String },
}

impl McpToolResult {
    pub fn success(message: impl Into<String>) -> Self {
        Self {
            success: true,
            content: vec![McpContent::Text {
                text: message.into(),
            }],
            error: None,
        }
    }

    pub fn error(message: impl Into<String>) -> Self {
        let msg = message.into();
        Self {
            success: false,
            content: vec![McpContent::Text { text: msg.clone() }],
            error: Some(msg),
        }
    }
}

/// Robot state for MCP (thread-safe)
pub struct McpRobotState {
    state: RwLock<RobotState>,
    is_moving: RwLock<bool>,
    current_action: RwLock<String>,
}

impl Default for McpRobotState {
    fn default() -> Self {
        Self::new()
    }
}

impl McpRobotState {
    pub fn new() -> Self {
        let mut state = RobotState::default();
        state.joints = unitree_g1::create_default_joints();
        state.base_pose = Pose::new(0.0, 0.9, 0.0, 0.0, 0.0, 0.0);
        state.is_standing = true;

        Self {
            state: RwLock::new(state),
            is_moving: RwLock::new(false),
            current_action: RwLock::new("idle".to_string()),
        }
    }

    pub fn get_state(&self) -> RobotState {
        self.state.read().unwrap().clone()
    }

    pub fn set_joint_positions(&self, positions: &[f64]) {
        self.state.write().unwrap().joints.set_positions(positions);
    }

    pub fn is_moving(&self) -> bool {
        *self.is_moving.read().unwrap()
    }

    pub fn set_moving(&self, moving: bool) {
        *self.is_moving.write().unwrap() = moving;
    }

    pub fn current_action(&self) -> String {
        self.current_action.read().unwrap().clone()
    }

    pub fn set_current_action(&self, action: &str) {
        *self.current_action.write().unwrap() = action.to_string();
    }
}

/// MCP Server for humanoid robot control
pub struct McpServer {
    /// Robot state
    robot: Arc<McpRobotState>,
    /// Available tools
    tools: HashMap<String, McpTool>,
}

impl McpServer {
    /// Create a new MCP server
    pub fn new(robot: Arc<McpRobotState>) -> Self {
        let mut server = Self {
            robot,
            tools: HashMap::new(),
        };

        server.register_tools();
        server
    }

    /// Register all available tools
    fn register_tools(&mut self) {
        // robot_stand
        self.tools.insert(
            "robot_stand".to_string(),
            McpTool {
                name: "robot_stand".to_string(),
                description: "Make the robot stand up in a stable standing position.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {},
                    "required": []
                }),
            },
        );

        // robot_sit
        self.tools.insert(
            "robot_sit".to_string(),
            McpTool {
                name: "robot_sit".to_string(),
                description: "Make the robot sit down on the ground.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {},
                    "required": []
                }),
            },
        );

        // robot_crouch
        self.tools.insert(
            "robot_crouch".to_string(),
            McpTool {
                name: "robot_crouch".to_string(),
                description: "Make the robot crouch (lower its center of mass).".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {},
                    "required": []
                }),
            },
        );

        // robot_walk
        self.tools.insert(
            "robot_walk".to_string(),
            McpTool {
                name: "robot_walk".to_string(),
                description: "Make the robot walk in a specified direction.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["forward", "backward", "left", "right"],
                            "description": "Direction to walk"
                        },
                        "speed": {
                            "type": "number",
                            "description": "Walking speed (0.0 to 1.0)",
                            "minimum": 0.0,
                            "maximum": 1.0
                        },
                        "duration_seconds": {
                            "type": "number",
                            "description": "How long to walk in seconds",
                            "minimum": 0.1,
                            "maximum": 30.0
                        }
                    },
                    "required": ["direction"]
                }),
            },
        );

        // robot_turn
        self.tools.insert(
            "robot_turn".to_string(),
            McpTool {
                name: "robot_turn".to_string(),
                description: "Make the robot turn in place.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["left", "right"],
                            "description": "Direction to turn"
                        },
                        "angle_degrees": {
                            "type": "number",
                            "description": "Angle to turn in degrees",
                            "minimum": 0,
                            "maximum": 360
                        }
                    },
                    "required": ["direction"]
                }),
            },
        );

        // robot_wave
        self.tools.insert(
            "robot_wave".to_string(),
            McpTool {
                name: "robot_wave".to_string(),
                description: "Make the robot wave its hand in greeting.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "hand": {
                            "type": "string",
                            "enum": ["left", "right"],
                            "description": "Which hand to wave"
                        },
                        "repetitions": {
                            "type": "integer",
                            "description": "Number of wave motions",
                            "minimum": 1,
                            "maximum": 5
                        }
                    },
                    "required": []
                }),
            },
        );

        // robot_point
        self.tools.insert(
            "robot_point".to_string(),
            McpTool {
                name: "robot_point".to_string(),
                description: "Make the robot point in a direction.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "hand": {
                            "type": "string",
                            "enum": ["left", "right"],
                            "description": "Which hand to use"
                        },
                        "direction": {
                            "type": "string",
                            "enum": ["forward", "left", "right", "up", "down"],
                            "description": "Direction to point"
                        }
                    },
                    "required": ["direction"]
                }),
            },
        );

        // robot_reach
        self.tools.insert(
            "robot_reach".to_string(),
            McpTool {
                name: "robot_reach".to_string(),
                description: "Make the robot reach to a target position.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "hand": {
                            "type": "string",
                            "enum": ["left", "right"],
                            "description": "Which hand to use"
                        },
                        "x": {
                            "type": "number",
                            "description": "Target X position (forward) in meters"
                        },
                        "y": {
                            "type": "number",
                            "description": "Target Y position (up) in meters"
                        },
                        "z": {
                            "type": "number",
                            "description": "Target Z position (left) in meters"
                        }
                    },
                    "required": ["x", "y", "z"]
                }),
            },
        );

        // robot_stop
        self.tools.insert(
            "robot_stop".to_string(),
            McpTool {
                name: "robot_stop".to_string(),
                description: "Stop the robot's current motion and hold position.".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {},
                    "required": []
                }),
            },
        );

        // robot_get_state
        self.tools.insert("robot_get_state".to_string(), McpTool {
            name: "robot_get_state".to_string(),
            description: "Get the current state of the robot including position, joint angles, and status.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "include_joints": {
                        "type": "boolean",
                        "description": "Whether to include detailed joint information"
                    }
                },
                "required": []
            }),
        });

        // robot_set_pose
        self.tools.insert("robot_set_pose".to_string(), McpTool {
            name: "robot_set_pose".to_string(),
            description: "Set the robot to a named pose.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "pose_name": {
                        "type": "string",
                        "enum": ["stand", "crouch", "t_pose", "arms_up", "arms_forward", "rest"],
                        "description": "Name of the predefined pose"
                    }
                },
                "required": ["pose_name"]
            }),
        });

        // robot_nod
        self.tools.insert(
            "robot_nod".to_string(),
            McpTool {
                name: "robot_nod".to_string(),
                description: "Make the robot nod its head (yes gesture).".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "repetitions": {
                            "type": "integer",
                            "minimum": 1,
                            "maximum": 5
                        }
                    },
                    "required": []
                }),
            },
        );

        // robot_shake_head
        self.tools.insert(
            "robot_shake_head".to_string(),
            McpTool {
                name: "robot_shake_head".to_string(),
                description: "Make the robot shake its head (no gesture).".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "repetitions": {
                            "type": "integer",
                            "minimum": 1,
                            "maximum": 5
                        }
                    },
                    "required": []
                }),
            },
        );

        // robot_bow
        self.tools.insert(
            "robot_bow".to_string(),
            McpTool {
                name: "robot_bow".to_string(),
                description: "Make the robot bow (respectful greeting).".to_string(),
                input_schema: json!({
                    "type": "object",
                    "properties": {
                        "depth": {
                            "type": "string",
                            "enum": ["slight", "normal", "deep"],
                            "description": "How deep to bow"
                        }
                    },
                    "required": []
                }),
            },
        );
    }

    /// List all available tools
    pub fn list_tools(&self) -> Vec<&McpTool> {
        self.tools.values().collect()
    }

    /// Call a tool with given arguments
    pub fn call_tool(&self, name: &str, arguments: Value) -> McpToolResult {
        match name {
            "robot_stand" => self.handle_stand(),
            "robot_sit" => self.handle_sit(),
            "robot_crouch" => self.handle_crouch(),
            "robot_walk" => self.handle_walk(&arguments),
            "robot_turn" => self.handle_turn(&arguments),
            "robot_wave" => self.handle_wave(&arguments),
            "robot_point" => self.handle_point(&arguments),
            "robot_reach" => self.handle_reach(&arguments),
            "robot_stop" => self.handle_stop(),
            "robot_get_state" => self.handle_get_state(&arguments),
            "robot_set_pose" => self.handle_set_pose(&arguments),
            "robot_nod" => self.handle_nod(&arguments),
            "robot_shake_head" => self.handle_shake_head(&arguments),
            "robot_bow" => self.handle_bow(&arguments),
            _ => McpToolResult::error(format!("Unknown tool: {}", name)),
        }
    }

    // Tool handlers

    fn handle_stand(&self) -> McpToolResult {
        // Standing pose - legs straight, arms at sides
        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        // Slight knee bend for stability
        positions[3] = 0.1; // left knee
        positions[9] = 0.1; // right knee

        // Ankle compensation
        positions[4] = -0.05; // left ankle
        positions[10] = -0.05; // right ankle

        self.robot.set_joint_positions(&positions);
        self.robot.set_current_action("standing");
        self.robot.set_moving(false);

        McpToolResult::success("Robot is now standing in a stable position.")
    }

    fn handle_sit(&self) -> McpToolResult {
        // Sitting pose
        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        // Bend knees significantly
        positions[0] = -1.2; // left hip pitch
        positions[3] = 2.0; // left knee
        positions[4] = -0.8; // left ankle

        positions[6] = -1.2; // right hip pitch
        positions[9] = 2.0; // right knee
        positions[10] = -0.8; // right ankle

        self.robot.set_joint_positions(&positions);
        self.robot.set_current_action("sitting");
        self.robot.set_moving(false);

        McpToolResult::success("Robot is now sitting down.")
    }

    fn handle_crouch(&self) -> McpToolResult {
        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        // Crouch pose - knees bent, lowered center of mass
        positions[0] = -0.6; // left hip pitch
        positions[3] = 1.2; // left knee
        positions[4] = -0.6; // left ankle

        positions[6] = -0.6; // right hip pitch
        positions[9] = 1.2; // right knee
        positions[10] = -0.6; // right ankle

        self.robot.set_joint_positions(&positions);
        self.robot.set_current_action("crouching");
        self.robot.set_moving(false);

        McpToolResult::success("Robot is now in a crouched position.")
    }

    fn handle_walk(&self, args: &Value) -> McpToolResult {
        let direction = args
            .get("direction")
            .and_then(|v| v.as_str())
            .unwrap_or("forward");

        let speed = args.get("speed").and_then(|v| v.as_f64()).unwrap_or(0.5);

        let duration = args
            .get("duration_seconds")
            .and_then(|v| v.as_f64())
            .unwrap_or(2.0);

        self.robot.set_moving(true);
        self.robot
            .set_current_action(&format!("walking_{}", direction));

        McpToolResult::success(format!(
            "Robot is now walking {} at {:.0}% speed for {:.1} seconds.",
            direction,
            speed * 100.0,
            duration
        ))
    }

    fn handle_turn(&self, args: &Value) -> McpToolResult {
        let direction = args
            .get("direction")
            .and_then(|v| v.as_str())
            .unwrap_or("left");

        let angle = args
            .get("angle_degrees")
            .and_then(|v| v.as_f64())
            .unwrap_or(90.0);

        self.robot.set_moving(true);
        self.robot
            .set_current_action(&format!("turning_{}", direction));

        McpToolResult::success(format!(
            "Robot is turning {} by {} degrees.",
            direction, angle
        ))
    }

    fn handle_wave(&self, args: &Value) -> McpToolResult {
        let hand = args.get("hand").and_then(|v| v.as_str()).unwrap_or("right");

        let reps = args
            .get("repetitions")
            .and_then(|v| v.as_u64())
            .unwrap_or(3) as usize;

        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        // Standing base
        positions[3] = 0.1;
        positions[9] = 0.1;

        // Raise arm for waving
        let (shoulder_pitch, shoulder_roll, elbow) = if hand == "right" {
            (22, 23, 25) // right arm indices
        } else {
            (15, 16, 18) // left arm indices
        };

        positions[shoulder_pitch] = -1.5; // Raise arm up
        positions[shoulder_roll] = 0.5; // Out to side
        positions[elbow] = 0.5; // Bend elbow

        self.robot.set_joint_positions(&positions);
        self.robot.set_current_action("waving");
        self.robot.set_moving(true);

        McpToolResult::success(format!(
            "Robot is waving its {} hand ({} times).",
            hand, reps
        ))
    }

    fn handle_point(&self, args: &Value) -> McpToolResult {
        let hand = args.get("hand").and_then(|v| v.as_str()).unwrap_or("right");

        let direction = args
            .get("direction")
            .and_then(|v| v.as_str())
            .unwrap_or("forward");

        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        // Standing base
        positions[3] = 0.1;
        positions[9] = 0.1;

        let (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow) = if hand == "right" {
            (22, 23, 24, 25)
        } else {
            (15, 16, 17, 18)
        };

        match direction {
            "forward" => {
                positions[shoulder_pitch] = -1.57; // Arm straight forward
                positions[elbow] = 0.0;
            }
            "up" => {
                positions[shoulder_pitch] = -3.0; // Arm up
                positions[elbow] = 0.0;
            }
            "down" => {
                positions[shoulder_pitch] = 0.5; // Arm down forward
                positions[elbow] = 0.0;
            }
            "left" => {
                positions[shoulder_pitch] = -1.2;
                positions[shoulder_yaw] = if hand == "right" { 1.0 } else { -0.5 };
            }
            "right" => {
                positions[shoulder_pitch] = -1.2;
                positions[shoulder_yaw] = if hand == "right" { -0.5 } else { 1.0 };
            }
            _ => {}
        }

        self.robot.set_joint_positions(&positions);
        self.robot
            .set_current_action(&format!("pointing_{}", direction));
        self.robot.set_moving(false);

        McpToolResult::success(format!(
            "Robot is pointing {} with its {} hand.",
            direction, hand
        ))
    }

    fn handle_reach(&self, args: &Value) -> McpToolResult {
        let hand = args.get("hand").and_then(|v| v.as_str()).unwrap_or("right");

        let x = args.get("x").and_then(|v| v.as_f64()).unwrap_or(0.3);
        let y = args.get("y").and_then(|v| v.as_f64()).unwrap_or(0.8);
        let z = args.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0);

        self.robot.set_current_action("reaching");
        self.robot.set_moving(true);

        McpToolResult::success(format!(
            "Robot is reaching with {} hand to position ({:.2}, {:.2}, {:.2}) meters.",
            hand, x, y, z
        ))
    }

    fn handle_stop(&self) -> McpToolResult {
        self.robot.set_moving(false);
        self.robot.set_current_action("stopped");

        McpToolResult::success("Robot has stopped and is holding its current position.")
    }

    fn handle_get_state(&self, args: &Value) -> McpToolResult {
        let include_joints = args
            .get("include_joints")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);

        let state = self.robot.get_state();
        let current_action = self.robot.current_action();
        let is_moving = self.robot.is_moving();

        let mut response = format!(
            "Robot State:\n\
            - Position: ({:.2}, {:.2}, {:.2}) meters\n\
            - Standing: {}\n\
            - Balanced: {}\n\
            - Currently moving: {}\n\
            - Current action: {}\n\
            - Foot contacts: Left={}, Right={}",
            state.base_pose.position.x,
            state.base_pose.position.y,
            state.base_pose.position.z,
            state.is_standing,
            state.is_balanced,
            is_moving,
            current_action,
            state.foot_contacts[0],
            state.foot_contacts[1],
        );

        if include_joints {
            response.push_str("\n\nJoint Positions:");
            for joint in state.joints.iter() {
                response.push_str(&format!(
                    "\n  - {}: {:.3} rad",
                    joint.name, joint.current_position
                ));
            }
        }

        McpToolResult::success(response)
    }

    fn handle_set_pose(&self, args: &Value) -> McpToolResult {
        let pose_name = args
            .get("pose_name")
            .and_then(|v| v.as_str())
            .unwrap_or("stand");

        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        match pose_name {
            "stand" => {
                positions[3] = 0.1;
                positions[9] = 0.1;
            }
            "crouch" => {
                positions[0] = -0.6;
                positions[3] = 1.2;
                positions[4] = -0.6;
                positions[6] = -0.6;
                positions[9] = 1.2;
                positions[10] = -0.6;
            }
            "t_pose" => {
                // Arms straight out to sides
                positions[16] = -1.57; // left shoulder roll
                positions[23] = 1.57; // right shoulder roll
            }
            "arms_up" => {
                positions[15] = -2.5; // left shoulder pitch
                positions[22] = -2.5; // right shoulder pitch
            }
            "arms_forward" => {
                positions[15] = -1.57;
                positions[22] = -1.57;
            }
            "rest" => {
                // All zeros
            }
            _ => {
                return McpToolResult::error(format!("Unknown pose: {}", pose_name));
            }
        }

        self.robot.set_joint_positions(&positions);
        self.robot
            .set_current_action(&format!("pose_{}", pose_name));
        self.robot.set_moving(false);

        McpToolResult::success(format!("Robot is now in the '{}' pose.", pose_name))
    }

    fn handle_nod(&self, args: &Value) -> McpToolResult {
        let reps = args
            .get("repetitions")
            .and_then(|v| v.as_u64())
            .unwrap_or(2);

        self.robot.set_current_action("nodding");
        self.robot.set_moving(true);

        McpToolResult::success(format!("Robot is nodding its head ({} times).", reps))
    }

    fn handle_shake_head(&self, args: &Value) -> McpToolResult {
        let reps = args
            .get("repetitions")
            .and_then(|v| v.as_u64())
            .unwrap_or(2);

        self.robot.set_current_action("shaking_head");
        self.robot.set_moving(true);

        McpToolResult::success(format!("Robot is shaking its head ({} times).", reps))
    }

    fn handle_bow(&self, args: &Value) -> McpToolResult {
        let depth = args
            .get("depth")
            .and_then(|v| v.as_str())
            .unwrap_or("normal");

        let mut positions = vec![0.0; unitree_g1::NUM_JOINTS];

        // Bow by bending at waist
        let waist_angle = match depth {
            "slight" => 0.3,
            "normal" => 0.6,
            "deep" => 1.0,
            _ => 0.6,
        };

        positions[13] = waist_angle; // waist pitch

        // Slight knee bend for balance
        positions[3] = 0.2;
        positions[9] = 0.2;

        self.robot.set_joint_positions(&positions);
        self.robot.set_current_action("bowing");
        self.robot.set_moving(true);

        McpToolResult::success(format!("Robot is bowing ({} bow).", depth))
    }
}

/// JSON-RPC request structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonRpcRequest {
    pub jsonrpc: String,
    pub id: Option<Value>,
    pub method: String,
    pub params: Option<Value>,
}

/// JSON-RPC response structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonRpcResponse {
    pub jsonrpc: String,
    pub id: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<JsonRpcError>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonRpcError {
    pub code: i32,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<Value>,
}

/// MCP Protocol handler for JSON-RPC
pub struct McpProtocol {
    server: McpServer,
}

impl McpProtocol {
    pub fn new(robot: Arc<McpRobotState>) -> Self {
        Self {
            server: McpServer::new(robot),
        }
    }

    /// Handle a JSON-RPC request
    pub fn handle_request(&self, request: &JsonRpcRequest) -> JsonRpcResponse {
        let result = match request.method.as_str() {
            "initialize" => self.handle_initialize(),
            "tools/list" => self.handle_list_tools(),
            "tools/call" => self.handle_call_tool(&request.params),
            _ => Err(JsonRpcError {
                code: -32601,
                message: format!("Method not found: {}", request.method),
                data: None,
            }),
        };

        match result {
            Ok(value) => JsonRpcResponse {
                jsonrpc: "2.0".to_string(),
                id: request.id.clone(),
                result: Some(value),
                error: None,
            },
            Err(error) => JsonRpcResponse {
                jsonrpc: "2.0".to_string(),
                id: request.id.clone(),
                result: None,
                error: Some(error),
            },
        }
    }

    fn handle_initialize(&self) -> Result<Value, JsonRpcError> {
        Ok(json!({
            "protocolVersion": "2024-11-05",
            "serverInfo": {
                "name": "humanoid-rs",
                "version": env!("CARGO_PKG_VERSION")
            },
            "capabilities": {
                "tools": {}
            }
        }))
    }

    fn handle_list_tools(&self) -> Result<Value, JsonRpcError> {
        let tools: Vec<Value> = self
            .server
            .list_tools()
            .iter()
            .map(|t| {
                json!({
                    "name": t.name,
                    "description": t.description,
                    "inputSchema": t.input_schema
                })
            })
            .collect();

        Ok(json!({ "tools": tools }))
    }

    fn handle_call_tool(&self, params: &Option<Value>) -> Result<Value, JsonRpcError> {
        let params = params.as_ref().ok_or_else(|| JsonRpcError {
            code: -32602,
            message: "Missing params".to_string(),
            data: None,
        })?;

        let name = params
            .get("name")
            .and_then(|v| v.as_str())
            .ok_or_else(|| JsonRpcError {
                code: -32602,
                message: "Missing tool name".to_string(),
                data: None,
            })?;

        let arguments = params.get("arguments").cloned().unwrap_or(json!({}));

        let result = self.server.call_tool(name, arguments);

        Ok(json!({
            "content": result.content,
            "isError": !result.success
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mcp_server_creation() {
        let robot = Arc::new(McpRobotState::new());
        let server = McpServer::new(robot);

        let tools = server.list_tools();
        assert!(!tools.is_empty());
        assert!(tools.iter().any(|t| t.name == "robot_stand"));
    }

    #[test]
    fn test_robot_stand() {
        let robot = Arc::new(McpRobotState::new());
        let server = McpServer::new(robot);

        let result = server.call_tool("robot_stand", json!({}));
        assert!(result.success);
    }

    #[test]
    fn test_robot_walk() {
        let robot = Arc::new(McpRobotState::new());
        let server = McpServer::new(Arc::clone(&robot));

        let result = server.call_tool(
            "robot_walk",
            json!({
                "direction": "forward",
                "speed": 0.7,
                "duration_seconds": 3.0
            }),
        );

        assert!(result.success);
        assert!(robot.is_moving());
    }

    #[test]
    fn test_robot_get_state() {
        let robot = Arc::new(McpRobotState::new());
        let server = McpServer::new(robot);

        let result = server.call_tool(
            "robot_get_state",
            json!({
                "include_joints": false
            }),
        );

        assert!(result.success);
    }

    #[test]
    fn test_json_rpc_protocol() {
        let robot = Arc::new(McpRobotState::new());
        let protocol = McpProtocol::new(robot);

        // Test initialize
        let request = JsonRpcRequest {
            jsonrpc: "2.0".to_string(),
            id: Some(json!(1)),
            method: "initialize".to_string(),
            params: None,
        };

        let response = protocol.handle_request(&request);
        assert!(response.result.is_some());
        assert!(response.error.is_none());

        // Test list tools
        let request = JsonRpcRequest {
            jsonrpc: "2.0".to_string(),
            id: Some(json!(2)),
            method: "tools/list".to_string(),
            params: None,
        };

        let response = protocol.handle_request(&request);
        assert!(response.result.is_some());
    }

    #[test]
    fn test_unknown_tool() {
        let robot = Arc::new(McpRobotState::new());
        let server = McpServer::new(robot);

        let result = server.call_tool("unknown_tool", json!({}));
        assert!(!result.success);
    }
}
