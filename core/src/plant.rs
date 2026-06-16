//! The physical airframe the controller actually flies: it takes the four
//! commanded rotor forces, passes them through actuator limits + lag, integrates
//! the rigid-body dynamics (with drag, wind, a CoM-offset torque, and optional
//! model mismatch) on a fine substep, and returns a *measured* state corrupted
//! by IMU noise, bias, and latency.
//!
//! With [`Realism::ideal`] the plant collapses to the original RK4 dynamics and
//! a perfect sensor, so it is a drop-in for the previous idealized loop.

use crate::dynamics::deriv_ext;
use crate::math::{project_to_so3, so3_exp};
use crate::params::QuadParams;
use crate::realism::{Realism, Rng};
use crate::state::QuadState;
use nalgebra::{Vector3, Vector4};
use std::collections::VecDeque;

/// One airframe instance: actuator state, the RNG/latency buffer for its sensor,
/// and the *effective* physical parameters (which may differ from the
/// controller's nominal params when model mismatch is enabled).
#[derive(Clone, Debug)]
pub struct Plant {
    /// Nominal parameters (rotor mixing geometry, gravity, sample time).
    nominal: QuadParams,
    /// Effective parameters actually governing the motion (mass/inertia possibly
    /// scaled away from `nominal`).
    eff: QuadParams,
    realism: Realism,
    /// Actuator state — the *actual* per-rotor forces, lagging the command.
    rotor: Vector4<f64>,
    rng: Rng,
    /// Ring buffer of true states for sensor latency.
    delay_buf: VecDeque<QuadState>,
}

impl Plant {
    pub fn new(nominal: &QuadParams, realism: &Realism) -> Self {
        let mut eff = nominal.clone();
        if realism.enabled {
            eff.mass = nominal.mass * realism.mass_scale;
            eff.j = nominal.j * realism.inertia_scale;
            eff.j_inv = eff.j.try_inverse().expect("scaled inertia invertible");
        }
        Plant {
            nominal: nominal.clone(),
            eff,
            realism: realism.clone(),
            rotor: Vector4::zeros(),
            rng: Rng::new(realism.seed),
            delay_buf: VecDeque::new(),
        }
    }

    /// Reconstruct the *achieved* `(thrust, moment)` from the current rotor
    /// forces via the forward wrench map.
    fn achieved_wrench(&self) -> (f64, Vector3<f64>) {
        let w = self.nominal.wrench * self.rotor;
        (w[0], Vector3::new(w[1], w[2], w[3]))
    }

    /// External force (inertial): translational drag + wind/gusts.
    fn ext_force(&self, s: &QuadState, t: f64) -> Vector3<f64> {
        if !self.realism.enabled {
            return Vector3::zeros();
        }
        -self.realism.drag_lin * s.v + self.realism.wind_force(t)
    }

    /// External torque (body): rotational drag + the parasitic CoM-offset moment.
    fn ext_torque(&self, s: &QuadState, f: f64) -> Vector3<f64> {
        if !self.realism.enabled {
            return Vector3::zeros();
        }
        let e3 = Vector3::new(0.0, 0.0, 1.0);
        // Thrust force in the body frame is `-f·e3`; an offset CoM turns it into
        // a torque about the true center of mass.
        let thrust_body = -f * e3;
        -self.realism.drag_rot * s.omega + self.realism.com_offset.cross(&thrust_body)
    }

    /// One RK4 step of length `h`, holding `(f, moment)` constant but
    /// recomputing the external wrench per stage (drag depends on the stage
    /// state, wind on time).
    fn rk4_sub(&self, s: &QuadState, f: f64, m: &Vector3<f64>, h: f64, t: f64) -> QuadState {
        let d = |st: &QuadState| {
            deriv_ext(st, f, m, &self.ext_force(st, t), &self.ext_torque(st, f), &self.eff)
        };
        let k1 = d(s);
        let k2 = d(&s.add_scaled(&k1, h * 0.5));
        let k3 = d(&s.add_scaled(&k2, h * 0.5));
        let k4 = d(&s.add_scaled(&k3, h));
        let sixth = h / 6.0;
        QuadState {
            p: s.p + sixth * (k1.p + 2.0 * k2.p + 2.0 * k3.p + k4.p),
            v: s.v + sixth * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v),
            r: s.r + sixth * (k1.r + 2.0 * k2.r + 2.0 * k3.r + k4.r),
            omega: s.omega + sixth * (k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega),
        }
    }

    /// Advance the airframe by one control period `dt`, starting at time `t`,
    /// given the controller's commanded per-rotor forces. Returns the new true
    /// state.
    pub fn step(&mut self, state: &QuadState, rotor_cmd: Vector4<f64>, dt: f64, t: f64) -> QuadState {
        let substeps = if self.realism.enabled {
            self.realism.substeps.max(1)
        } else {
            1
        };
        let h = dt / substeps as f64;

        // Actuator: saturate the command, then lag the actual rotor forces.
        let target = if self.realism.enabled && self.realism.actuator {
            self.realism.clamp_rotors(rotor_cmd)
        } else {
            rotor_cmd
        };

        let mut s = state.clone();
        for i in 0..substeps {
            if self.realism.enabled && self.realism.actuator {
                let alpha = if self.realism.motor_tau > 0.0 {
                    1.0 - (-h / self.realism.motor_tau).exp()
                } else {
                    1.0
                };
                self.rotor += alpha * (target - self.rotor);
            } else {
                self.rotor = target;
            }
            let (f, m) = self.achieved_wrench();
            s = self.rk4_sub(&s, f, &m, h, t + i as f64 * h);
        }
        s.r = project_to_so3(&s.r);
        s
    }

    /// Produce the *measured* state the controller sees: a latency-delayed copy
    /// of the true state with gyro bias/noise and small attitude noise added.
    pub fn measure(&mut self, true_state: &QuadState) -> QuadState {
        if !self.realism.enabled {
            return true_state.clone();
        }
        // Latency: push the latest truth, read the one `sensor_delay` steps back.
        self.delay_buf.push_back(true_state.clone());
        while self.delay_buf.len() > self.realism.sensor_delay + 1 {
            self.delay_buf.pop_front();
        }
        let mut m = self.delay_buf.front().cloned().unwrap_or_else(|| true_state.clone());

        // Gyro: constant bias + white noise.
        m.omega += self.realism.gyro_bias + self.realism.gyro_noise * self.rng.gaussian3();
        // Attitude: perturb by a small random rotation, then re-project.
        if self.realism.att_noise > 0.0 {
            let dtheta = self.realism.att_noise * self.rng.gaussian3();
            m.r = project_to_so3(&(m.r * so3_exp(&dtheta)));
        }
        m
    }
}
