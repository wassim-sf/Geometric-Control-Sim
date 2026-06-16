//! The geometric tracking controller — a port of `matlab/controller.m`.
//!
//! The position outer loop, the desired-attitude construction, all the
//! Appendix-F feedforward derivatives, and the moment law are **identical** for
//! every controller. The *only* thing that varies is how the attitude error
//! `eR` is parameterised ([`AttitudeMode`]). Keeping everything else fixed makes
//! the three formulations directly comparable: any difference in behaviour is
//! attributable to the attitude-error representation alone.

pub mod euler;
pub mod geometric;
pub mod quaternion;

use crate::dirty_derivative::DirtyDerivative;
use crate::math::{body_to_euler_rates, hat, r_to_zyx, vee};
use crate::params::QuadParams;
use crate::state::QuadState;
use crate::trajectory::{ControlMode, Target};
use nalgebra::{Matrix3, Vector3, Vector4};

/// Which attitude-error formulation the inner loop uses.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum AttitudeMode {
    /// Coordinate-free SO(3) error (Lee et al.).
    Geometric,
    /// Sign-corrected quaternion-vector error.
    Quaternion,
    /// Naive ZYX Euler-angle error (gimbal-locks).
    Euler,
}

impl AttitudeMode {
    pub const ALL: [AttitudeMode; 3] = [
        AttitudeMode::Geometric,
        AttitudeMode::Quaternion,
        AttitudeMode::Euler,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            AttitudeMode::Geometric => "Geometric SE(3)",
            AttitudeMode::Quaternion => "Quaternion",
            AttitudeMode::Euler => "Euler",
        }
    }

    /// Parse a short token (`geometric`/`quaternion`/`euler`).
    pub fn from_token(s: &str) -> Option<AttitudeMode> {
        match s.to_ascii_lowercase().as_str() {
            "geometric" | "geo" | "se3" => Some(AttitudeMode::Geometric),
            "quaternion" | "quat" | "q" => Some(AttitudeMode::Quaternion),
            "euler" | "e" => Some(AttitudeMode::Euler),
            _ => None,
        }
    }

    /// Attitude error `eR` for the current `R` and desired `Rc`.
    pub fn attitude_error(&self, r: &Matrix3<f64>, rc: &Matrix3<f64>) -> Vector3<f64> {
        match self {
            AttitudeMode::Geometric => geometric::attitude_error(r, rc),
            AttitudeMode::Quaternion => quaternion::attitude_error(r, rc),
            AttitudeMode::Euler => euler::attitude_error(r, rc),
        }
    }
}

/// Output of one controller evaluation.
#[derive(Clone, Debug)]
pub struct ControlOutput {
    /// Total thrust magnitude `f` [N].
    pub f: f64,
    /// Body moment `M` [N·m].
    pub moment: Vector3<f64>,
    /// Commanded attitude `Rc`.
    pub rc: Matrix3<f64>,
    /// Commanded body angular velocity `Ωc`.
    pub omega_c: Vector3<f64>,
    /// Attitude error `eR` used by the moment law.
    pub er: Vector3<f64>,
    /// SO(3) configuration error `Ψ = ½·tr(I − Rcᵀ·R) ∈ [0, 2]`.
    pub psi: f64,
    /// Individual rotor forces `[f1, f2, f3, f4]`.
    pub rotor_forces: Vector4<f64>,
}

/// A stateful geometric tracking controller.
///
/// The desired-trajectory derivatives come analytically from the [`Target`]; the
/// only quantity differentiated numerically is the *measured* velocity (a
/// feedback signal with no closed form), via two persistent dirty-derivative
/// filters that mirror `controller.m`'s `dv1dt`/`dv2dt`.
#[derive(Clone, Debug)]
pub struct GeometricController {
    mode: AttitudeMode,
    dv1: DirtyDerivative,
    dv2: DirtyDerivative,
    /// Stateful sign-naive quaternion error (only used in the `naive` quaternion
    /// case, to demonstrate double-cover unwinding).
    naive_quat: quaternion::NaiveQuatTracker,
}

impl GeometricController {
    pub fn new(mode: AttitudeMode, p: &QuadParams) -> Self {
        GeometricController {
            mode,
            dv1: DirtyDerivative::new(1, p.tau, p.ts),
            dv2: DirtyDerivative::new(2, p.tau * 10.0, p.ts),
            naive_quat: quaternion::NaiveQuatTracker::new(),
        }
    }

    pub fn mode(&self) -> AttitudeMode {
        self.mode
    }

    /// Evaluate the control law for the current `state` and `target`.
    ///
    /// `naive` selects how the *baseline* (non-geometric) controllers form their
    /// moment. With `naive = false` ("fair isolation") all controllers share the
    /// geometric outer loop, gyroscopic compensation, and SO(3) feedforward —
    /// only the attitude error `eR` differs. With `naive = true` the **Euler**
    /// controller becomes a true standalone Euler-frame PD that feeds back the
    /// singular Euler rates, so it destabilises at gimbal lock. The geometric and
    /// quaternion controllers are unchanged (both are global / singularity-free).
    pub fn control(
        &mut self,
        state: &QuadState,
        target: &Target,
        p: &QuadParams,
        naive: bool,
        cmode: ControlMode,
    ) -> ControlOutput {
        let e3 = Vector3::new(0.0, 0.0, 1.0);
        let xd = target.xd;
        let b1d = target.b1d;
        let (x, v, r, omega) = (state.p, state.v, state.r, state.omega);

        // Desired-trajectory derivatives come analytically from the target.
        let (xd_1, xd_2, xd_3, xd_4) =
            (target.xd_dot, target.xd_ddot, target.xd_3dot, target.xd_4dot);
        let (b1d_1, b1d_2) = (target.b1d_dot, target.b1d_ddot);
        // The measured velocity is differentiated numerically (feedback signal).
        let v_1 = self.dv1.calculate(v);
        let v_2 = self.dv2.calculate(v_1);

        // In velocity mode there is no position feedback, so the proportional
        // position gain (and all of its time-derivatives in the feedforward) is
        // dropped consistently. Position and attitude modes keep it.
        let kx = if cmode == ControlMode::Velocity { 0.0 } else { p.kx };

        // Position / velocity / accel / jerk errors (eq. 17-18).
        let ex = x - xd;
        let ev = v - xd_1;
        let ea = v_1 - xd_2;
        let ej = v_2 - xd_3;

        // Thrust magnitude (eq. 19).
        let a = -kx * ex - p.kv * ev - p.mass * p.gravity * e3 + p.mass * xd_2;
        let na = a.norm().max(1e-9);
        let f_pos = (-a).dot(&(r * e3));

        // Desired body axes (eq. 23, 38).
        let b3c = -a / na;
        let c = b3c.cross(&b1d);
        let nc = c.norm().max(1e-9);
        let b2c = c / nc;
        let b1c = -(b3c.cross(&c)) / nc;
        let rc = Matrix3::from_columns(&[b1c, b2c, b3c]);

        // First time-derivatives of the body axes (arXiv:1003.2005, Appendix F).
        let a_1 = -kx * ev - p.kv * ea + p.mass * xd_3;
        let b3c_1 = -a_1 / na + (a.dot(&a_1) / na.powi(3)) * a;
        let c_1 = b3c_1.cross(&b1d) + b3c.cross(&b1d_1);
        // NOTE: the original MATLAB used `C/norm(C)` for the first term here; the
        // correct quotient-rule derivative of C/||C|| is `C_1dot/norm(C)`.
        let b2c_1 = c_1 / nc - (c.dot(&c_1) / nc.powi(3)) * c;
        let b1c_1 = b2c_1.cross(&b3c) + b2c.cross(&b3c_1);

        // Second time-derivatives.
        let a_2 = -kx * ea - p.kv * ej + p.mass * xd_4;
        let b3c_2 = -a_2 / na
            + (2.0 / na.powi(3)) * a.dot(&a_1) * a_1
            + ((a_1.norm().powi(2) + a.dot(&a_2)) / na.powi(3)) * a
            - (3.0 / na.powi(5)) * a.dot(&a_1).powi(2) * a;
        let c_2 = b3c_2.cross(&b1d) + b3c.cross(&b1d_2) + 2.0 * b3c_1.cross(&b1d_1);
        // NOTE: the original MATLAB used `norm(C_2dot)^2`; the correct term
        // (matching the b3c expansion above) is `norm(C_1dot)^2`.
        let b2c_2 = c_2 / nc
            - (2.0 / nc.powi(3)) * c.dot(&c_1) * c_1
            - ((c_1.norm().powi(2) + c.dot(&c_2)) / nc.powi(3)) * c
            + (3.0 / nc.powi(5)) * c.dot(&c_1).powi(2) * c;
        let b1c_2 = b2c_2.cross(&b3c) + b2c.cross(&b3c_2) + 2.0 * b2c_1.cross(&b3c_1);

        let rc_1 = Matrix3::from_columns(&[b1c_1, b2c_1, b3c_1]);
        let rc_2 = Matrix3::from_columns(&[b1c_2, b2c_2, b3c_2]);

        // In Position/Velocity modes the attitude command (and its rates) come
        // from the thrust-direction construction above. In Attitude mode the
        // controller is handed `Rd`, `Ωd`, `Ω̇d` directly, and thrust simply
        // holds hover — so the airframe is free to slew to any orientation.
        let (rc, omega_c, omega_c_1, f) = if cmode == ControlMode::Attitude {
            (target.rd, target.omega_d, target.omega_d_dot, p.mass * p.gravity)
        } else {
            let omega_c = vee(&(rc.transpose() * rc_1));
            let omega_c_1 = vee(&(rc.transpose() * rc_2 - hat(&omega_c) * hat(&omega_c)));
            (rc, omega_c, omega_c_1, f_pos)
        };

        // Attitude error (mode-dependent) and angular-velocity error (eq. 21).
        // With `naive`, the quaternion controller drops its sign correction and
        // so exhibits the double-cover "unwinding".
        let er = if naive && self.mode == AttitudeMode::Quaternion {
            self.naive_quat.error(&r, &rc)
        } else {
            self.mode.attitude_error(&r, &rc)
        };
        let e_omega = omega - r.transpose() * rc * omega_c;

        let moment = if naive && self.mode == AttitudeMode::Euler {
            // Standalone naive Euler-frame PD: proportional on the (aliasing)
            // Euler error, derivative on the *Euler rates* — whose `1/cos(θ)`
            // term blows up at θ = ±90°, so this controller tumbles at gimbal
            // lock. No gyroscopic compensation, no SO(3) feedforward.
            let (roll, pitch, _) = r_to_zyx(&r);
            let eta_dot = body_to_euler_rates(roll, pitch, &omega);
            -p.kr * er - p.komega * eta_dot
        } else {
            // Geometric moment law: feedback + gyroscopic + SO(3) feedforward.
            -p.kr * er - p.komega * e_omega + omega.cross(&(p.j * omega))
                - p.j * (hat(&omega) * r.transpose() * rc * omega_c
                    - r.transpose() * rc * omega_c_1)
        };

        let psi = 0.5 * (Matrix3::identity() - rc.transpose() * r).trace();
        let rotor_forces = p.rotor_forces(f, &moment);

        ControlOutput {
            f,
            moment,
            rc,
            omega_c,
            er,
            psi,
            rotor_forces,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::so3_exp;
    use std::f64::consts::PI;

    /// Past the Euler singularity (100° pitch) the geometric and quaternion
    /// errors point correctly along the pitch axis, while the naive Euler error
    /// is huge and aliased into roll/yaw — a direction the true error has no
    /// component in. This is the representation flaw the comparison highlights.
    #[test]
    fn euler_error_misrepresents_past_gimbal_lock() {
        let r = so3_exp(&Vector3::new(0.0, 100_f64.to_radians(), 0.0));
        let rc = Matrix3::identity();

        let geo = AttitudeMode::Geometric.attitude_error(&r, &rc);
        let quat = AttitudeMode::Quaternion.attitude_error(&r, &rc);
        let euler = AttitudeMode::Euler.attitude_error(&r, &rc);

        // Geometric & quaternion: aligned with the pitch (+y) axis, modest size.
        for e in [geo, quat] {
            assert!(e.x.abs() < 1e-6 && e.z.abs() < 1e-6, "off-axis: {e:?}");
            assert!(e.y > 0.9, "pitch component too small: {e:?}");
        }
        // Euler: badly aliased — large roll & yaw that don't physically exist.
        assert!(euler.norm() > 4.0, "Euler error should blow up: {euler:?}");
        assert!(
            euler.x.abs() > 3.0 && euler.z.abs() > 3.0,
            "Euler error should be aliased into roll/yaw: {euler:?}"
        );
    }

    /// At *exactly* 180° the geometric error vanishes (`sin(pi) = 0`, an unstable
    /// equilibrium), whereas the sign-corrected quaternion error stays at full
    /// magnitude and commands recovery.
    #[test]
    fn geometric_vanishes_at_180_but_quaternion_does_not() {
        let r = so3_exp(&Vector3::new(PI, 0.0, 0.0));
        let rc = Matrix3::identity();
        let geo = AttitudeMode::Geometric.attitude_error(&r, &rc);
        let quat = AttitudeMode::Quaternion.attitude_error(&r, &rc);
        assert!(geo.norm() < 1e-9, "geometric error should vanish: {geo:?}");
        assert!(quat.norm() > 1.9, "quaternion error should be ~2: {quat:?}");
    }

    /// Near the identity all three formulations agree (they share the same
    /// small-angle linearisation), so they behave identically for gentle flight.
    #[test]
    fn all_modes_agree_for_small_errors() {
        let r = so3_exp(&Vector3::new(0.01, -0.02, 0.015));
        let rc = Matrix3::identity();
        let geo = AttitudeMode::Geometric.attitude_error(&r, &rc);
        let quat = AttitudeMode::Quaternion.attitude_error(&r, &rc);
        let euler = AttitudeMode::Euler.attitude_error(&r, &rc);
        assert!((geo - quat).norm() < 1e-3);
        assert!((geo - euler).norm() < 1e-3);
    }
}
