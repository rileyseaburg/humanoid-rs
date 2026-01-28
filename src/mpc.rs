//! Model Predictive Control (MPC) for humanoid locomotion
//!
//! Implements a simplified MPC controller for walking and balance.
//! Uses a linear inverted pendulum model (LIPM) for computational efficiency.

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use std::time::Duration;

/// MPC configuration
#[derive(Debug, Clone)]
pub struct MpcConfig {
    /// Prediction horizon (number of steps)
    pub horizon: usize,

    /// Control timestep
    pub dt: f64,

    /// CoM height (for LIPM)
    pub com_height: f64,

    /// Weight on CoM position tracking
    pub weight_com_pos: f64,

    /// Weight on CoM velocity tracking  
    pub weight_com_vel: f64,

    /// Weight on ZMP tracking
    pub weight_zmp: f64,

    /// Weight on control effort (jerk)
    pub weight_jerk: f64,

    /// Maximum ZMP distance from CoM
    pub zmp_margin: f64,

    /// Gravity constant
    pub gravity: f64,
}

impl Default for MpcConfig {
    fn default() -> Self {
        Self {
            horizon: 20,
            dt: 0.02, // 50 Hz
            com_height: 0.85,
            weight_com_pos: 1.0,
            weight_com_vel: 0.1,
            weight_zmp: 10.0,
            weight_jerk: 0.001,
            zmp_margin: 0.08,
            gravity: 9.81,
        }
    }
}

/// State vector for LIPM: [x, x_dot, y, y_dot]
#[derive(Debug, Clone, Default)]
pub struct LipmState {
    pub x: f64,
    pub x_dot: f64,
    pub y: f64,
    pub y_dot: f64,
}

impl LipmState {
    pub fn new(x: f64, x_dot: f64, y: f64, y_dot: f64) -> Self {
        Self { x, x_dot, y, y_dot }
    }

    pub fn to_vector(&self) -> DVector<f64> {
        DVector::from_vec(vec![self.x, self.x_dot, self.y, self.y_dot])
    }

    pub fn from_vector(v: &DVector<f64>) -> Self {
        Self {
            x: v[0],
            x_dot: v[1],
            y: v[2],
            y_dot: v[3],
        }
    }

    pub fn com_position(&self) -> Vector3<f64> {
        Vector3::new(self.x, 0.0, self.y)
    }

    pub fn com_velocity(&self) -> Vector3<f64> {
        Vector3::new(self.x_dot, 0.0, self.y_dot)
    }
}

/// ZMP reference trajectory
#[derive(Debug, Clone)]
pub struct ZmpReference {
    pub points: Vec<Vector3<f64>>,
    pub times: Vec<f64>,
}

impl ZmpReference {
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            times: Vec::new(),
        }
    }

    /// Create static reference (standing)
    pub fn standing(duration: f64, dt: f64, position: Vector3<f64>) -> Self {
        let n = (duration / dt) as usize;
        Self {
            points: vec![position; n],
            times: (0..n).map(|i| i as f64 * dt).collect(),
        }
    }

    /// Create walking reference
    pub fn walking(
        step_length: f64,
        step_width: f64,
        step_duration: f64,
        num_steps: usize,
        dt: f64,
    ) -> Self {
        let mut points = Vec::new();
        let mut times = Vec::new();

        let points_per_step = (step_duration / dt) as usize;

        for step in 0..num_steps {
            let is_left = step % 2 == 0;
            let y_offset = if is_left {
                step_width / 2.0
            } else {
                -step_width / 2.0
            };
            let x_pos = step as f64 * step_length;

            for i in 0..points_per_step {
                let t = (step * points_per_step + i) as f64 * dt;
                // Interpolate ZMP during step
                let progress = i as f64 / points_per_step as f64;
                let x = x_pos + progress * step_length;

                points.push(Vector3::new(x, 0.0, y_offset));
                times.push(t);
            }
        }

        Self { points, times }
    }

    /// Get ZMP reference at time t
    pub fn at_time(&self, t: f64) -> Vector3<f64> {
        if self.times.is_empty() {
            return Vector3::zeros();
        }

        // Find bracketing indices
        let idx = self
            .times
            .iter()
            .position(|&time| time > t)
            .unwrap_or(self.times.len());

        if idx == 0 {
            return self.points[0];
        }
        if idx >= self.times.len() {
            return *self.points.last().unwrap();
        }

        // Linear interpolation
        let t0 = self.times[idx - 1];
        let t1 = self.times[idx];
        let alpha = (t - t0) / (t1 - t0);

        self.points[idx - 1] * (1.0 - alpha) + self.points[idx] * alpha
    }
}

/// MPC controller using Linear Inverted Pendulum Model
pub struct MpcController {
    config: MpcConfig,

    /// Natural frequency of LIPM
    omega: f64,

    /// State transition matrix (discrete)
    a_matrix: DMatrix<f64>,

    /// Control input matrix (discrete)  
    b_matrix: DMatrix<f64>,

    /// ZMP output matrix
    c_zmp: DMatrix<f64>,

    /// Precomputed prediction matrices
    psi: DMatrix<f64>, // State prediction
    theta: DMatrix<f64>, // Control to output

    /// Current optimal control sequence
    u_optimal: DVector<f64>,

    /// Reference trajectory
    zmp_ref: ZmpReference,

    /// Current time
    current_time: f64,
}

impl MpcController {
    pub fn new(config: MpcConfig) -> Self {
        let omega = (config.gravity / config.com_height).sqrt();

        // LIPM discrete-time matrices (for x direction, y is identical)
        // State: [x, x_dot], Input: x_zmp, Output: x_zmp
        let dt = config.dt;
        let ch = (omega * dt).cosh();
        let sh = (omega * dt).sinh();

        // 2x2 state transition for each axis
        let a_1d = DMatrix::from_row_slice(2, 2, &[ch, sh / omega, omega * sh, ch]);

        let b_1d = DMatrix::from_row_slice(2, 1, &[1.0 - ch, -omega * sh]);

        // ZMP = x - x_ddot / omega^2 = x - omega^2 * (x - zmp) / omega^2 = zmp
        // For LIPM with ZMP input, c = [1, 0] gives position
        let c_1d = DMatrix::from_row_slice(1, 2, &[1.0, 0.0]);

        // Stack for x and y axes
        let a_matrix = stack_diagonal_2d(&a_1d);
        let b_matrix = stack_diagonal_1d(&b_1d);
        let c_zmp = stack_diagonal_1d_output(&c_1d);

        // Compute prediction matrices
        let (psi, theta) =
            compute_prediction_matrices(&a_matrix, &b_matrix, &c_zmp, config.horizon);

        Self {
            config: config.clone(),
            omega,
            a_matrix,
            b_matrix,
            c_zmp,
            psi,
            theta,
            u_optimal: DVector::zeros(config.horizon * 2),
            zmp_ref: ZmpReference::new(),
            current_time: 0.0,
        }
    }

    /// Set ZMP reference trajectory
    pub fn set_reference(&mut self, zmp_ref: ZmpReference) {
        self.zmp_ref = zmp_ref;
    }

    /// Solve MPC optimization problem
    pub fn solve(&mut self, state: &LipmState) -> (Vector3<f64>, Vector3<f64>) {
        let x = state.to_vector();

        // Build reference vector over horizon
        let mut y_ref = DVector::zeros(self.config.horizon * 2);
        for i in 0..self.config.horizon {
            let t = self.current_time + i as f64 * self.config.dt;
            let ref_point = self.zmp_ref.at_time(t);
            y_ref[i * 2] = ref_point.x;
            y_ref[i * 2 + 1] = ref_point.z;
        }

        // Predicted output without control: Psi * x
        let y_free = &self.psi * &x;

        // Error: y_ref - y_free
        let e = &y_ref - &y_free;

        // QP cost: min ||Theta * u - e||^2 + weight_jerk * ||u||^2
        // Solution: u = (Theta'*Theta + R)^{-1} * Theta' * e
        let theta_t = self.theta.transpose();
        let r = DMatrix::identity(self.config.horizon * 2, self.config.horizon * 2)
            * self.config.weight_jerk;

        let h = &theta_t * &self.theta + &r;
        let g = &theta_t * &e;

        // Solve linear system (simplified - real impl would use proper QP solver)
        if let Some(h_inv) = h.clone().try_inverse() {
            self.u_optimal = h_inv * g;
        }

        // Extract first control input
        let zmp_cmd = Vector3::new(self.u_optimal[0], 0.0, self.u_optimal[1]);

        // Also return predicted CoM velocity for feedforward
        let com_vel_cmd = Vector3::new(
            self.omega * (state.x - zmp_cmd.x).sinh() * self.config.dt,
            0.0,
            self.omega * (state.y - zmp_cmd.z).sinh() * self.config.dt,
        );

        self.current_time += self.config.dt;

        (zmp_cmd, com_vel_cmd)
    }

    /// Step the LIPM model forward
    pub fn predict_state(&self, state: &LipmState, zmp: Vector3<f64>) -> LipmState {
        let x = state.to_vector();
        let u = DVector::from_vec(vec![zmp.x, zmp.z]);

        let x_next = &self.a_matrix * &x + &self.b_matrix * &u;
        LipmState::from_vector(&x_next)
    }

    /// Get capture point (DCM)
    pub fn capture_point(&self, state: &LipmState) -> Vector3<f64> {
        // DCM = CoM + CoM_vel / omega
        Vector3::new(
            state.x + state.x_dot / self.omega,
            0.0,
            state.y + state.y_dot / self.omega,
        )
    }

    /// Reset controller
    pub fn reset(&mut self) {
        self.current_time = 0.0;
        self.u_optimal = DVector::zeros(self.config.horizon * 2);
    }
}

/// Stack two 2x2 matrices diagonally
fn stack_diagonal_2d(m: &DMatrix<f64>) -> DMatrix<f64> {
    let mut result = DMatrix::zeros(4, 4);
    for i in 0..2 {
        for j in 0..2 {
            result[(i, j)] = m[(i, j)];
            result[(i + 2, j + 2)] = m[(i, j)];
        }
    }
    result
}

/// Stack two 2x1 matrices diagonally
fn stack_diagonal_1d(m: &DMatrix<f64>) -> DMatrix<f64> {
    let mut result = DMatrix::zeros(4, 2);
    result[(0, 0)] = m[(0, 0)];
    result[(1, 0)] = m[(1, 0)];
    result[(2, 1)] = m[(0, 0)];
    result[(3, 1)] = m[(1, 0)];
    result
}

/// Stack output matrices
fn stack_diagonal_1d_output(m: &DMatrix<f64>) -> DMatrix<f64> {
    let mut result = DMatrix::zeros(2, 4);
    result[(0, 0)] = m[(0, 0)];
    result[(0, 1)] = m[(0, 1)];
    result[(1, 2)] = m[(0, 0)];
    result[(1, 3)] = m[(0, 1)];
    result
}

/// Compute prediction matrices for MPC
fn compute_prediction_matrices(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    c: &DMatrix<f64>,
    horizon: usize,
) -> (DMatrix<f64>, DMatrix<f64>) {
    let n_state = a.nrows();
    let n_input = b.ncols();
    let n_output = c.nrows();

    // Psi: state to output prediction
    let mut psi = DMatrix::zeros(horizon * n_output, n_state);

    // Theta: input to output prediction
    let mut theta = DMatrix::zeros(horizon * n_output, horizon * n_input);

    let mut a_power = DMatrix::identity(n_state, n_state);

    for i in 0..horizon {
        a_power = &a_power * a;
        let ca = c * &a_power;

        // Fill Psi
        for r in 0..n_output {
            for c_idx in 0..n_state {
                psi[(i * n_output + r, c_idx)] = ca[(r, c_idx)];
            }
        }

        // Fill Theta
        let mut a_pow_j = DMatrix::identity(n_state, n_state);
        for j in 0..=i {
            let cab = c * &a_pow_j * b;
            for r in 0..n_output {
                for c_idx in 0..n_input {
                    theta[(i * n_output + r, j * n_input + c_idx)] = cab[(r, c_idx)];
                }
            }
            a_pow_j = &a_pow_j * a;
        }
    }

    (psi, theta)
}

/// Footstep planner for MPC reference generation
pub struct FootstepPlanner {
    pub step_length: f64,
    pub step_width: f64,
    pub step_duration: f64,
    pub double_support_ratio: f64,
}

impl Default for FootstepPlanner {
    fn default() -> Self {
        Self {
            step_length: 0.15,
            step_width: 0.20,
            step_duration: 0.4,
            double_support_ratio: 0.2,
        }
    }
}

impl FootstepPlanner {
    /// Plan footsteps for walking in a direction
    pub fn plan_walking(&self, velocity: Vector3<f64>, duration: f64) -> Vec<Footstep> {
        let speed = Vector3::new(velocity.x, 0.0, velocity.z).norm();
        if speed < 0.01 {
            return vec![Footstep::default()]; // Standing
        }

        let step_time = self.step_duration;
        let num_steps = (duration / step_time) as usize;

        let mut footsteps = Vec::with_capacity(num_steps);
        let direction = velocity / speed;

        for i in 0..num_steps {
            let is_left = i % 2 == 0;
            let forward = direction * self.step_length * i as f64;
            let lateral = if is_left {
                Vector3::new(-self.step_width / 2.0, 0.0, 0.0)
            } else {
                Vector3::new(self.step_width / 2.0, 0.0, 0.0)
            };

            footsteps.push(Footstep {
                position: forward + lateral,
                is_left,
                start_time: i as f64 * step_time,
                end_time: (i + 1) as f64 * step_time,
            });
        }

        footsteps
    }

    /// Generate ZMP reference from footsteps
    pub fn footsteps_to_zmp(&self, footsteps: &[Footstep], dt: f64) -> ZmpReference {
        let mut points = Vec::new();
        let mut times = Vec::new();

        for step in footsteps {
            let n = ((step.end_time - step.start_time) / dt) as usize;
            for i in 0..n {
                let t = step.start_time + i as f64 * dt;
                points.push(step.position);
                times.push(t);
            }
        }

        ZmpReference { points, times }
    }
}

/// Single footstep
#[derive(Debug, Clone, Default)]
pub struct Footstep {
    pub position: Vector3<f64>,
    pub is_left: bool,
    pub start_time: f64,
    pub end_time: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lipm_state() {
        let state = LipmState::new(0.1, 0.5, 0.0, 0.1);
        let v = state.to_vector();
        let state2 = LipmState::from_vector(&v);
        assert!((state.x - state2.x).abs() < 1e-10);
    }

    #[test]
    fn test_zmp_reference() {
        let zmp_ref = ZmpReference::standing(1.0, 0.02, Vector3::zeros());
        assert!(!zmp_ref.points.is_empty());

        let point = zmp_ref.at_time(0.5);
        assert!((point.x).abs() < 1e-10);
    }

    #[test]
    fn test_mpc_controller() {
        let config = MpcConfig::default();
        let mut mpc = MpcController::new(config);

        mpc.set_reference(ZmpReference::standing(2.0, 0.02, Vector3::zeros()));

        let state = LipmState::new(0.01, 0.0, 0.0, 0.0);
        let (zmp_cmd, _vel_cmd) = mpc.solve(&state);

        // ZMP should try to move toward reference
        assert!(zmp_cmd.x.is_finite());
    }

    #[test]
    fn test_capture_point() {
        let config = MpcConfig::default();
        let mpc = MpcController::new(config);

        let state = LipmState::new(0.0, 0.5, 0.0, 0.0);
        let dcm = mpc.capture_point(&state);

        // DCM should be ahead of CoM when moving forward
        assert!(dcm.x > state.x);
    }

    #[test]
    fn test_footstep_planner() {
        let planner = FootstepPlanner::default();
        let footsteps = planner.plan_walking(Vector3::new(0.5, 0.0, 0.0), 2.0);

        assert!(!footsteps.is_empty());

        // Alternating feet
        for (i, step) in footsteps.iter().enumerate() {
            assert_eq!(step.is_left, i % 2 == 0);
        }
    }
}
