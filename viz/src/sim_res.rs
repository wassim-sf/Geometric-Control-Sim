//! The simulation as a Bevy resource, plus the fixed-step stepping system.

use bevy::prelude::*;
use nalgebra::Vector3;
use se3quad_core::{
    scenario, trajectory::yaw_to_b1d, AttitudeMode, QuadParams, Realism, Safety, Scenario, Sim,
    Target, Trajectory,
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
    /// Draw the expected reference path as a dotted line ahead of the vehicle.
    pub show_path: bool,
    /// Environment severity: 0 = off (ideal), 1 = calm, 2 = moderate, 3 = harsh.
    pub env_choice: usize,
    /// Per-aspect dropdown selections (refine the active environment).
    pub noise_choice: usize,
    pub wind_choice: usize,
    pub battery_choice: usize,
    /// Render the controllers spread apart (so all three are visible at once).
    pub spread: bool,
    /// Lateral gap between spread controllers [m].
    pub gap: f32,
    /// Use realistic standalone baselines (the naive Euler controller then fails
    /// at gimbal lock) instead of the "fair isolation" where only `eR` differs.
    pub naive: bool,
    /// Run the Euler/quaternion baselines as standalone single-domain
    /// controllers (no sharing of the geometric loop). Takes precedence over
    /// `naive` for those two.
    pub standalone: bool,
    /// Whether the real-world plant effects are active.
    pub realism_on: bool,
    /// Use physically-strict positive-only motors (`rotor_min = 0`).
    pub real_motors: bool,
    /// Whether the real-drone safety supervisor is active.
    pub safety_on: bool,
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
            show_path: true,
            env_choice: 0,
            noise_choice: 2,
            wind_choice: 2,
            battery_choice: 0,
            spread: true,
            gap: 2.6,
            naive: std::env::var("SE3QUAD_NAIVE").is_ok(),
            standalone: false,
            realism_on: false,
            real_motors: false,
            safety_on: false,
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

    /// Select an environment-severity preset (0 off / 1 calm / 2 moderate /
    /// 3 harsh) and apply it.
    pub fn apply_env(&mut self, choice: usize) {
        self.env_choice = choice;
        self.realism_on = choice != 0;
        if let Some(preset) = match choice {
            1 => Some(Realism::low()),
            2 => Some(Realism::medium()),
            3 => Some(Realism::harsh()),
            _ => None,
        } {
            self.realism = preset;
        }
        self.apply_realism();
    }

    /// IMU-noise preset (0 none / 1 low / 2 medium / 3 high) — refines the
    /// active environment's gyro/attitude noise + sensor latency.
    pub fn set_noise(&mut self, choice: usize) {
        self.noise_choice = choice;
        let r = &mut self.realism;
        let (gyro, att, bias, delay) = match choice {
            0 => (0.0, 0.0, 0.0, 0),
            1 => (0.003, 0.001, 0.002, 1),
            2 => (0.012, 0.005, 0.006, 2),
            _ => (0.03, 0.012, 0.015, 3),
        };
        r.gyro_noise = gyro;
        r.att_noise = att;
        r.gyro_bias = Vector3::new(bias, -bias * 0.7, bias * 0.5);
        r.sensor_delay = delay;
        self.realism_on = true;
        self.apply_realism();
    }

    /// Wind preset (0 calm / 1 breeze / 2 gusty / 3 storm).
    pub fn set_wind(&mut self, choice: usize) {
        self.wind_choice = choice;
        let r = &mut self.realism;
        let (wx, wy, amp, freq) = match choice {
            0 => (0.0, 0.0, 0.0, 0.0),
            1 => (0.4, 0.0, 0.3, 1.0),
            2 => (0.9, -0.4, 1.0, 1.8),
            _ => (1.6, -0.8, 2.0, 2.5),
        };
        r.wind = Vector3::new(wx, wy, 0.0);
        r.gust_amp = amp;
        r.gust_freq = freq;
        self.realism_on = true;
        self.apply_realism();
    }

    /// Battery-status preset (0 full / 1 half / 2 low / 3 draining fast).
    pub fn set_battery(&mut self, choice: usize) {
        self.battery_choice = choice;
        let r = &mut self.realism;
        let (on, init, sag, drain) = match choice {
            0 => (false, 1.0, 0.0, 0.0),
            1 => (true, 0.5, 0.12, 2.0e-4),
            2 => (true, 0.25, 0.18, 3.0e-4),
            _ => (true, 1.0, 0.22, 1.2e-3),
        };
        r.battery = on;
        r.battery_init = init;
        r.battery_sag = sag;
        r.battery_drain = drain;
        self.realism_on = true;
        self.apply_realism();
    }

    /// Apply the current realism preset/parameters to the plant (rebuilds the
    /// airframes and restarts the run).
    pub fn apply_realism(&mut self) {
        let r = if self.realism_on {
            let mut r = self.realism.clone();
            r.enabled = true;
            // Physically-strict motors can only push.
            r.rotor_min = if self.real_motors { 0.0 } else { -12.0 };
            r
        } else {
            Realism::ideal()
        };
        self.sim.set_realism(r);
        self.accumulator = 0.0;
    }

    /// Apply the safety supervisor (or disable it).
    pub fn apply_safety(&mut self) {
        self.sim.set_safety(if self.safety_on {
            Safety::standard()
        } else {
            Safety::off()
        });
        self.accumulator = 0.0;
    }
}

/// Advance the simulation in fixed `params.ts` steps, scaled by `speed`.
pub fn step_sim(time: Res<Time>, mut s: ResMut<SimState>) {
    s.sim.naive_baselines = s.naive;
    s.sim.standalone_baselines = s.standalone;
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
