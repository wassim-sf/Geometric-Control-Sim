//! Geometric tracking control of a quadrotor on SE(3).
//!
//! This is a Rust port of the MATLAB/Simulink implementation in `../../matlab`,
//! which follows Lee, Leok & McClamroch, *Control of Complex Maneuvers for a
//! Quadrotor UAV using Geometric Methods on SE(3)* (arXiv:1003.2005).
//!
//! The crate is GUI-free: it exposes the dynamics, three attitude-control
//! formulations (geometric / quaternion / Euler), trajectory + scenario
//! generation, and a [`Sim`] runner that integrates several controllers from a
//! shared initial condition so their behaviour can be compared side-by-side.

pub mod controllers;
pub mod dirty_derivative;
pub mod dynamics;
pub mod math;
pub mod params;
pub mod plant;
pub mod realism;
pub mod safety;
pub mod scenario;
pub mod sim;
pub mod state;
pub mod trajectory;

pub use controllers::{AttitudeMode, ControlOutput, GeometricController};
pub use dirty_derivative::DirtyDerivative;
pub use params::QuadParams;
pub use plant::Plant;
pub use realism::Realism;
pub use safety::Safety;
pub use scenario::Scenario;
pub use sim::{ControllerInstance, Sample, Sim};
pub use state::QuadState;
pub use trajectory::{ControlMode, FlightSegment, Target, Trajectory};
