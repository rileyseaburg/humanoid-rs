//! Humanoid Robot Physics Simulator
//!
//! Runs a physics-accurate simulation of the Unitree G1 humanoid robot
//! using Bevy + Rapier for articulated rigid body dynamics.

use humanoid_rs::physics_sim::run_physics_simulator;

fn main() {
    println!("Starting Humanoid Robot Physics Simulator...");
    println!("Simulating Unitree G1 with articulated body dynamics");
    println!();

    run_physics_simulator();
}
