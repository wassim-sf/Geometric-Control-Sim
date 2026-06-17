//! Real-drone safety logic (trap #11). A real autopilot never feeds the raw
//! controller output straight to the motors — it goes through a supervisor that
//! enforces the limits you'd use on a test stand and in early flights:
//!
//! - **arming**: disarmed ⇒ motors idle, no thrust;
//! - **thrust clamp**: total thrust held in `[f_min, f_max]`;
//! - **tilt limit**: the *commanded* attitude is capped to `max_tilt` from
//!   upright, so an aggressive outer-loop command can't flip the airframe during
//!   cautious testing;
//! - **altitude floor / ground contact**: the airframe can't fall through the
//!   pad (and a floor keeps early tests off the ground);
//! - **estimator health**: a non-finite estimate trips a failsafe.
//!
//! This is intentionally opt-in: [`Safety::off`] is a transparent pass-through.

use nalgebra::Vector3;

/// Supervisor limits applied around the controller. SI units; angles in radians.
#[derive(Clone, Debug)]
pub struct Safety {
    pub enabled: bool,
    /// Motors only produce thrust when armed.
    pub armed: bool,
    /// Total-thrust bounds [N].
    pub f_min: f64,
    pub f_max: f64,
    /// Maximum commanded tilt from upright [rad].
    pub max_tilt: f64,
    /// Minimum altitude above the pad [m] (ground contact / floor).
    pub altitude_floor: f64,
}

impl Safety {
    /// Transparent pass-through (no limits).
    pub fn off() -> Self {
        Safety {
            enabled: false,
            armed: true,
            f_min: 0.0,
            f_max: f64::INFINITY,
            max_tilt: std::f64::consts::PI,
            altitude_floor: f64::NEG_INFINITY,
        }
    }

    /// A cautious test-stand / early-flight preset (tuned for the 4.34 kg
    /// airframe: hover ≈ 42.6 N): positive thrust up to ~2.5× hover, a 50° tilt
    /// cap, and a ground floor at the pad.
    pub fn standard() -> Self {
        Safety {
            enabled: true,
            armed: true,
            f_min: 0.0,
            f_max: 110.0,
            max_tilt: 50_f64.to_radians(),
            altitude_floor: 0.0,
        }
    }

    /// Clamp the commanded desired body-z axis `b3c` so the tilt from upright
    /// (the `+e3` / NED-down direction, which `b3c = e3` at hover) never exceeds
    /// `max_tilt`. Returns `b3c` unchanged when within the limit or disabled.
    pub fn limit_tilt(&self, b3c: Vector3<f64>) -> Vector3<f64> {
        if !self.enabled {
            return b3c;
        }
        let e3 = Vector3::new(0.0, 0.0, 1.0);
        let cos_tilt = b3c.dot(&e3).clamp(-1.0, 1.0);
        let tilt = cos_tilt.acos();
        if tilt <= self.max_tilt {
            return b3c;
        }
        // Re-aim `b3c` to sit exactly `max_tilt` from `e3`, in the same plane.
        let tangent = b3c - cos_tilt * e3;
        let tn = tangent.norm();
        if tn < 1e-9 {
            return b3c; // exactly anti-parallel: nothing sensible to clamp toward
        }
        let t_hat = tangent / tn;
        (e3 * self.max_tilt.cos() + t_hat * self.max_tilt.sin()).normalize()
    }

    /// Clamp total thrust to `[f_min, f_max]`, and force it to zero when disarmed.
    pub fn clamp_thrust(&self, f: f64) -> f64 {
        if !self.enabled {
            return f;
        }
        if !self.armed {
            return 0.0;
        }
        f.clamp(self.f_min, self.f_max)
    }

    /// Whether thrust is currently inhibited (disarmed while the layer is on).
    pub fn inhibited(&self) -> bool {
        self.enabled && !self.armed
    }
}

impl Default for Safety {
    fn default() -> Self {
        Safety::off()
    }
}
