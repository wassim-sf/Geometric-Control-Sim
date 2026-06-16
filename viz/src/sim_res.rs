//! The simulation as a Bevy resource, plus the fixed-step stepping system.

use bevy::prelude::*;
use nalgebra::Vector3;
use se3quad_core::{
    scenario, trajectory::yaw_to_b1d, AttitudeMode, QuadParams, Realism, Scenario, Sim, Target,
    Trajectory,
};

/// Editable parameters for the user-definable [`Trajectory::Custom`] Lissajous.
#[derive(Clone, Copy)]
pub struct CustomTraj {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub wx: f32,
    pub wy: f32,
    pub wz: f32,
    pub py: f32,
    pub pz: f32,
    pub height_up: f32,
    pub yaw: f32,
}

impl Default for CustomTraj {
    fn default() -> Self {
        CustomTraj {
            ax: 2.0,
            ay: 1.5,
            az: 0.6,
            wx: 0.6,
            wy: 1.2,
            wz: 0.9,
            py: 0.0,
            pz: std::f32::consts::FRAC_PI_2,
            height_up: 3.0,
            yaw: 0.0,
        }
    }
}

impl CustomTraj {
    pub fn to_trajectory(self) -> Trajectory {
        Trajectory::Custom {
            ax: self.ax as f64,
            ay: self.ay as f64,
            az: self.az as f64,
            wx: self.wx as f64,
            wy: self.wy as f64,
            wz: self.wz as f64,
            py: self.py as f64,
            pz: self.pz as f64,
            height: -(self.height_up as f64),
            yaw: self.yaw as f64,
        }
    }
}

/// Holds the running [`Sim`] (always simulating all three controllers) plus all
/// UI-facing view state. Controller *visibility* is a view filter — every
/// controller is always integrated so toggling the comparison never resets.
#[derive(Resource)]
pub struct SimState {
    pub sim: Sim,
    pub scenarios: Vec<Scenario>,
    pub selected: usize,
    pub playing: bool,
    pub speed: f32,
    pub accumulator: f32,
    /// Visibility per controller, indexed like [`AttitudeMode::ALL`].
    pub show: [bool; 3],
    pub trails: bool,
    /// Render the controllers spread apart (so all three are visible at once).
    pub spread: bool,
    /// Lateral gap between spread controllers [m].
    pub gap: f32,
    /// Use realistic standalone baselines (the naive Euler controller then fails
    /// at gimbal lock) instead of the "fair isolation" where only `eR` differs.
    pub naive: bool,
    /// Whether the real-world plant effects are active.
    pub realism_on: bool,
    /// Editable real-world effect parameters (applied on toggle / "Apply").
    pub realism: Realism,
    /// Editable parameters for the custom-trajectory scenario.
    pub custom: CustomTraj,
    // Live-target sliders (north / east / up / yaw).
    pub live_n: f32,
    pub live_e: f32,
    pub live_up: f32,
    pub live_yaw: f32,
}

impl Default for SimState {
    fn default() -> Self {
        let scenarios = scenario::all();
        // Optionally launch straight into a scenario (e.g. SE3QUAD_SCENARIO=flip).
        let selected = std::env::var("SE3QUAD_SCENARIO")
            .ok()
            .and_then(|n| scenario::resolve(&n))
            .and_then(|sc| scenarios.iter().position(|s| s.name == sc.name))
            .unwrap_or(1); // default: Upside-Down Recovery
        let sim = Sim::new(
            scenarios[selected].clone(),
            &AttitudeMode::ALL,
            QuadParams::default(),
        );
        SimState {
            sim,
            scenarios,
            selected,
            playing: true,
            speed: 1.0,
            accumulator: 0.0,
            show: [true, true, true],
            trails: true,
            spread: true,
            gap: 2.6,
            naive: std::env::var("SE3QUAD_NAIVE").is_ok(),
            realism_on: false,
            realism: Realism::realistic(),
            custom: CustomTraj::default(),
            live_n: 0.0,
            live_e: 0.0,
            live_up: 0.0,
            live_yaw: 0.0,
        }
    }
}

impl SimState {
    pub fn select_scenario(&mut self, idx: usize) {
        self.selected = idx;
        self.sim.set_scenario(self.scenarios[idx].clone());
        self.accumulator = 0.0;
        let p = self.sim.scenario.initial.p;
        self.live_n = p.x as f32;
        self.live_e = p.y as f32;
        self.live_up = -p.z as f32;
        self.live_yaw = 0.0;
    }

    pub fn reset(&mut self) {
        self.sim.reset();
        self.accumulator = 0.0;
    }

    pub fn is_live(&self) -> bool {
        matches!(self.sim.trajectory, Trajectory::Live)
    }

    pub fn is_custom(&self) -> bool {
        matches!(self.sim.trajectory, Trajectory::Custom { .. })
    }

    /// Apply the current realism preset/parameters to the plant (rebuilds the
    /// airframes and restarts the run).
    pub fn apply_realism(&mut self) {
        let r = if self.realism_on {
            let mut r = self.realism.clone();
            r.enabled = true;
            r
        } else {
            Realism::ideal()
        };
        self.sim.set_realism(r);
        self.accumulator = 0.0;
    }
}

/// Advance the simulation in fixed `params.ts` steps, scaled by `speed`.
pub fn step_sim(time: Res<Time>, mut s: ResMut<SimState>) {
    s.sim.naive_baselines = s.naive;
    if s.is_live() {
        let xd = Vector3::new(s.live_n as f64, s.live_e as f64, -(s.live_up as f64));
        let yaw = s.live_yaw as f64;
        s.sim.live_target = Target::hold(xd, yaw_to_b1d(yaw));
    }
    // Live-edit the custom trajectory: dragging the sliders reshapes the path.
    if s.is_custom() {
        s.sim.trajectory = s.custom.to_trajectory();
    }
    if !s.playing {
        return;
    }
    let ts = s.sim.params.ts as f32;
    s.accumulator += time.delta_secs() * s.speed;
    let mut steps = 0;
    while s.accumulator >= ts && steps < 400 {
        s.sim.step();
        s.accumulator -= ts;
        steps += 1;
    }
}
