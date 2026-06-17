//! Standalone, single-domain baseline controllers — used when you want a
//! comparison with **no mixing of mathematical domains** (as opposed to the
//! default "fair isolation", where Euler/quaternion borrow the geometric loop
//! and only `eR` differs).
//!
//! Both still share the *position* outer loop (Newton's law in the inertial
//! frame is representation-agnostic), but each then builds its desired attitude
//! and closes the inner loop **entirely in its own domain**:
//!
//! - [`GeometricController::control_euler_pid`] derives desired ZYX Euler angles
//!   from the thrust direction, then does a pure Euler-angle PD with feedback on
//!   the Euler *rates* (`η̇`, whose `1/cosθ` Jacobian is singular at θ = ±90°).
//!   It is the genuine textbook Euler controller — and it gimbal-locks.
//! - [`GeometricController::control_quaternion_pd`] builds a desired quaternion
//!   and does a sign-corrected quaternion PD with gyroscopic compensation — a
//!   genuine quaternion attitude controller, no SO(3) feedforward.

use super::{ControlOutput, GeometricController};
use crate::math::{body_to_euler_rates, r_to_zyx, wrap_angle};
use crate::params::QuadParams;
use crate::safety::Safety;
use crate::state::QuadState;
use crate::trajectory::{ControlMode, Target};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

impl GeometricController {
    /// Shared position outer loop → desired force `A` (eq. 19). Advances the
    /// measured-velocity dirty-derivatives so the filter stays consistent with
    /// the geometric path (exactly one control path runs per instance per step).
    fn outer_force(
        &mut self,
        state: &QuadState,
        target: &Target,
        p: &QuadParams,
        cmode: ControlMode,
    ) -> Vector3<f64> {
        let e3 = Vector3::new(0.0, 0.0, 1.0);
        let v_1 = self.dv1.calculate(state.v);
        let _ = self.dv2.calculate(v_1);
        let kx = if cmode == ControlMode::Velocity {
            0.0
        } else {
            p.kx
        };
        let ex = state.p - target.xd;
        let ev = state.v - target.xd_dot;
        // Integral term with anti-windup clamping (Position mode only).
        let ki_term = if cmode == ControlMode::Position && p.ki > 0.0 {
            p.ki * self.integrate(&ex, p)
        } else {
            Vector3::zeros()
        };
        -kx * ex - p.kv * ev - ki_term - p.mass * p.gravity * e3 + p.mass * target.xd_ddot
    }

    /// Standalone Euler-angle PID attitude controller (gimbal-locks at θ = ±90°).
    pub fn control_euler_pid(
        &mut self,
        state: &QuadState,
        target: &Target,
        p: &QuadParams,
        cmode: ControlMode,
        safety: &Safety,
    ) -> ControlOutput {
        let a = self.outer_force(state, target, p, cmode);
        let r = state.r;
        let omega = state.omega;

        // Desired ZYX Euler angles + thrust.
        let (eta_d, f) = if cmode == ControlMode::Attitude {
            let (rd, pd, yd) = r_to_zyx(&target.rd);
            (Vector3::new(rd, pd, yd), p.hover_thrust())
        } else {
            let na = a.norm().max(1e-9);
            let b3d = safety.limit_tilt(-a / na);
            let yaw_d = target.b1d.y.atan2(target.b1d.x);
            // Strip yaw, then read roll/pitch off the thrust direction.
            let u = Rotation3::from_axis_angle(&Vector3::z_axis(), -yaw_d) * b3d;
            let roll_d = -(u.y.clamp(-1.0, 1.0)).asin();
            let pitch_d = u.x.atan2(u.z);
            // Thrust projects onto the *actual* body-z (a quad can only push
            // along −b3); this can go negative when inverted (ideal plant only).
            let e3 = Vector3::new(0.0, 0.0, 1.0);
            (Vector3::new(roll_d, pitch_d, yaw_d), (-a).dot(&(r * e3)))
        };

        // Current Euler angles + Euler rates (the singular Jacobian).
        let (roll, pitch, yaw) = r_to_zyx(&r);
        let eta = Vector3::new(roll, pitch, yaw);
        let eta_err = Vector3::new(
            wrap_angle(eta.x - eta_d.x),
            wrap_angle(eta.y - eta_d.y),
            wrap_angle(eta.z - eta_d.z),
        );
        let eta_dot = body_to_euler_rates(roll, pitch, &omega);

        // Pure Euler-frame PD.
        let moment = -p.kr * eta_err - p.komega * eta_dot;

        let rd = Rotation3::from_euler_angles(eta_d.x, eta_d.y, eta_d.z).into_inner();
        self.finish(rd, eta_err, f, moment, &r, p, safety)
    }

    /// Standalone quaternion attitude controller (sign-corrected, with gyro
    /// compensation; no SO(3) feedforward).
    pub fn control_quaternion_pd(
        &mut self,
        state: &QuadState,
        target: &Target,
        p: &QuadParams,
        cmode: ControlMode,
        safety: &Safety,
    ) -> ControlOutput {
        let a = self.outer_force(state, target, p, cmode);
        let r = state.r;
        let omega = state.omega;

        let (rd, f) = if cmode == ControlMode::Attitude {
            (target.rd, p.hover_thrust())
        } else {
            let na = a.norm().max(1e-9);
            let b3d = safety.limit_tilt(-a / na);
            let c = b3d.cross(&target.b1d);
            let nc = c.norm().max(1e-9);
            let b2d = c / nc;
            let b1d = -(b3d.cross(&c)) / nc;
            let e3 = Vector3::new(0.0, 0.0, 1.0);
            (Matrix3::from_columns(&[b1d, b2d, b3d]), (-a).dot(&(r * e3)))
        };

        // Sign-corrected quaternion error, entirely in the quaternion domain.
        let qd = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rd));
        let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r));
        let qe = qd.inverse() * q;
        let sign = if qe.scalar() < 0.0 { -1.0 } else { 1.0 };
        let er = 2.0 * sign * qe.imag();

        // Quaternion PD + gyroscopic compensation (Ωd = 0).
        let moment = -p.kr * er - p.komega * omega + omega.cross(&(p.j * omega));
        self.finish(rd, er, f, moment, &r, p, safety)
    }

    /// Common tail: apply the safety supervisor and pack the [`ControlOutput`].
    #[allow(clippy::too_many_arguments)]
    fn finish(
        &self,
        rd: Matrix3<f64>,
        er: Vector3<f64>,
        f: f64,
        moment: Vector3<f64>,
        r: &Matrix3<f64>,
        p: &QuadParams,
        safety: &Safety,
    ) -> ControlOutput {
        let psi = 0.5 * (Matrix3::identity() - rd.transpose() * r).trace();
        let f = safety.clamp_thrust(f);
        let moment = if safety.inhibited() {
            Vector3::zeros()
        } else {
            moment
        };
        let rotor_forces = p.rotor_forces(f, &moment);
        ControlOutput {
            f,
            moment,
            rc: rd,
            omega_c: Vector3::zeros(),
            er,
            psi,
            rotor_forces,
        }
    }
}
