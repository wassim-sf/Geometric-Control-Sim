//! Optional real-world effects that turn the idealized SE(3) plant into
//! something closer to a flying airframe: actuator saturation + motor lag,
//! aerodynamic drag, wind/gusts, plant↔controller model mismatch, a body-frame
//! CoM offset, IMU noise/bias/latency, and physics substepping.
//!
//! [`Realism::ideal`] is a pass-through: with it the plant reproduces the
//! original idealized dynamics bit-for-bit (so the existing tests still hold).
//! [`Realism::realistic`] turns on a sensible, still-flyable set of effects.

use nalgebra::{Vector3, Vector4};

/// A tiny deterministic RNG (xorshift64*) with a Gaussian sampler, so sensor
/// noise is reproducible and we avoid an external `rand` dependency. Seeding the
/// three controllers identically keeps the comparison fair (same disturbances).
#[derive(Clone, Debug)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng {
            state: seed.max(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard-normal sample via Box–Muller.
    pub fn gaussian(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    /// Three independent standard-normal samples.
    pub fn gaussian3(&mut self) -> Vector3<f64> {
        Vector3::new(self.gaussian(), self.gaussian(), self.gaussian())
    }
}

/// Real-world effects applied by the [`crate::plant::Plant`]. All magnitudes are
/// SI; the controller is *not* told about any of these (that is the point —
/// it must be robust to them).
#[derive(Clone, Debug)]
pub struct Realism {
    /// Master switch. When `false`, every effect below is bypassed regardless of
    /// its value, and the plant is the original idealized SE(3) dynamics.
    pub enabled: bool,

    // --- actuator ---
    /// Saturate per-rotor thrust to `[rotor_min, rotor_max]` (a real rotor can
    /// only push, never pull, and has a finite ceiling).
    pub actuator: bool,
    pub rotor_min: f64,
    pub rotor_max: f64,
    /// First-order motor/ESC lag time constant [s] (0 ⇒ instantaneous).
    pub motor_tau: f64,

    // --- aerodynamics / disturbances ---
    /// Linear translational drag coefficient [N/(m/s)].
    pub drag_lin: f64,
    /// Linear rotational drag coefficient [N·m/(rad/s)].
    pub drag_rot: f64,
    /// Steady wind force on the airframe [N], inertial frame.
    pub wind: Vector3<f64>,
    /// Sinusoidal gust amplitude [N] and angular frequency [rad/s], along north.
    pub gust_amp: f64,
    pub gust_freq: f64,

    // --- model mismatch (plant vs. the controller's nominal params) ---
    /// Plant mass = nominal · `mass_scale`.
    pub mass_scale: f64,
    /// Plant inertia = nominal · `inertia_scale`.
    pub inertia_scale: f64,
    /// Center-of-mass offset in the body frame [m]; the thrust then exerts a
    /// parasitic torque `com_offset × F_thrust`.
    pub com_offset: Vector3<f64>,

    // --- sensors (IMU) ---
    /// Constant gyro bias [rad/s].
    pub gyro_bias: Vector3<f64>,
    /// Gyro white-noise standard deviation [rad/s].
    pub gyro_noise: f64,
    /// Attitude-measurement white-noise standard deviation [rad].
    pub att_noise: f64,
    /// Sensor/processing latency, in controller steps.
    pub sensor_delay: usize,

    // --- timing ---
    /// Physics substeps per control step (control runs ZOH; physics finer).
    pub substeps: usize,
    /// RNG seed (shared across controllers for a fair comparison).
    pub seed: u64,
}

impl Realism {
    /// Pass-through: reproduces the idealized plant exactly.
    pub fn ideal() -> Self {
        Realism {
            enabled: false,
            actuator: false,
            rotor_min: 0.0,
            rotor_max: f64::INFINITY,
            motor_tau: 0.0,
            drag_lin: 0.0,
            drag_rot: 0.0,
            wind: Vector3::zeros(),
            gust_amp: 0.0,
            gust_freq: 0.0,
            mass_scale: 1.0,
            inertia_scale: 1.0,
            com_offset: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            gyro_noise: 0.0,
            att_noise: 0.0,
            sensor_delay: 0,
            substeps: 1,
            seed: 1,
        }
    }

    /// A moderate, still-flyable real-world preset (tuned for the default 4.34 kg
    /// airframe, whose per-rotor hover thrust is ≈ 10.6 N). Gentle enough that
    /// the tracking scenarios stay stable while showing realistic degradation;
    /// extreme upsets (e.g. fully inverted) may legitimately fail, since
    /// unidirectional rotors cannot reverse-thrust out of inversion.
    pub fn realistic() -> Self {
        Realism {
            enabled: true,
            actuator: true,
            // A modest reverse-thrust allowance (reversible ESC / 3-D props).
            // A hard `rotor_min = 0` floor is physically real but makes it
            // impossible to flip out of full inversion — which would make *every*
            // controller fail the recovery demos together. Allowing limited
            // reverse thrust keeps the recoveries working while the rest of the
            // real-world effects (lag, drag, wind, noise, mismatch) still bite.
            rotor_min: -12.0,
            rotor_max: 30.0,
            motor_tau: 0.02,
            drag_lin: 0.2,
            drag_rot: 0.02,
            wind: Vector3::new(0.3, 0.0, 0.0),
            gust_amp: 0.4,
            gust_freq: 1.2,
            mass_scale: 1.02,
            inertia_scale: 1.03,
            com_offset: Vector3::new(0.006, -0.004, 0.0),
            gyro_bias: Vector3::new(0.006, -0.004, 0.003),
            gyro_noise: 0.004,
            att_noise: 0.0015,
            sensor_delay: 1,
            substeps: 4,
            seed: 1,
        }
    }

    /// Saturate a per-rotor force command to `[rotor_min, rotor_max]`.
    pub fn clamp_rotors(&self, cmd: Vector4<f64>) -> Vector4<f64> {
        cmd.map(|f| f.clamp(self.rotor_min, self.rotor_max))
    }

    /// Wind force at time `t` (steady + along-north sinusoidal gust).
    pub fn wind_force(&self, t: f64) -> Vector3<f64> {
        self.wind + Vector3::new(self.gust_amp * (self.gust_freq * t).sin(), 0.0, 0.0)
    }
}

impl Default for Realism {
    fn default() -> Self {
        Realism::ideal()
    }
}
