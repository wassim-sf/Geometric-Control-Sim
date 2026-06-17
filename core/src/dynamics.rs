//! Rigid-body equations of motion and a fixed-step RK4 integrator.
//! Ported from `mdlDerivatives` in `matlab/quad_dynamics.m`.

use crate::math::{hat, project_to_so3};
use crate::params::QuadParams;
use crate::state::{QuadState, StateDot};
use nalgebra::Vector3;

/// Continuous-time dynamics given total thrust `f` and body moment `moment`.
///
/// ```text
/// ṗ = v
/// v̇ = g·e3 − (1/m)·f·R·e3
/// Ṙ = R·hat(Ω)
/// Ω̇ = J⁻¹(M − Ω×J·Ω)
/// ```
pub fn deriv(s: &QuadState, f: f64, moment: &Vector3<f64>, p: &QuadParams) -> StateDot {
    deriv_ext(s, f, moment, &Vector3::zeros(), &Vector3::zeros(), p)
}

/// Dynamics with an additional external **force** (inertial frame, e.g. drag and
/// wind) and external **torque** (body frame, e.g. rotational drag, a CoM-offset
/// moment). The idealized [`deriv`] is the zero-external-wrench case.
pub fn deriv_ext(
    s: &QuadState,
    f: f64,
    moment: &Vector3<f64>,
    ext_force: &Vector3<f64>,
    ext_torque: &Vector3<f64>,
    p: &QuadParams,
) -> StateDot {
    let e3 = Vector3::new(0.0, 0.0, 1.0);
    StateDot {
        p: s.v,
        v: p.gravity * e3 - (f / p.mass) * (s.r * e3) + ext_force / p.mass,
        r: s.r * hat(&s.omega),
        omega: p.j_inv * (moment + ext_torque - s.omega.cross(&(p.j * s.omega))),
    }
}

/// One classical RK4 step of length `dt` holding `(f, moment)` constant, then
/// re-orthonormalising `R` back onto SO(3).
pub fn rk4_step(
    s: &QuadState,
    f: f64,
    moment: &Vector3<f64>,
    p: &QuadParams,
    dt: f64,
) -> QuadState {
    let k1 = deriv(s, f, moment, p);
    let k2 = deriv(&s.add_scaled(&k1, dt * 0.5), f, moment, p);
    let k3 = deriv(&s.add_scaled(&k2, dt * 0.5), f, moment, p);
    let k4 = deriv(&s.add_scaled(&k3, dt), f, moment, p);

    let sixth = dt / 6.0;
    let mut next = QuadState {
        p: s.p + sixth * (k1.p + 2.0 * k2.p + 2.0 * k3.p + k4.p),
        v: s.v + sixth * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v),
        r: s.r + sixth * (k1.r + 2.0 * k2.r + 2.0 * k3.r + k4.r),
        omega: s.omega + sixth * (k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega),
    };
    next.r = project_to_so3(&next.r);
    next
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Matrix3;

    #[test]
    fn hover_equilibrium_is_stationary() {
        let p = QuadParams::default();
        let s = QuadState::at(Vector3::zeros(), Matrix3::identity());
        // Upright, thrust = m*g opposing gravity, zero moment.
        let d = deriv(&s, p.hover_thrust(), &Vector3::zeros(), &p);
        assert!(d.v.norm() < 1e-9, "hover should have zero acceleration");
        assert!(d.omega.norm() < 1e-9);
    }

    #[test]
    fn free_state_stays_on_so3() {
        let p = QuadParams::default();
        let mut s = QuadState::at(Vector3::zeros(), Matrix3::identity());
        s.omega = Vector3::new(1.0, -2.0, 0.5);
        for _ in 0..1000 {
            s = rk4_step(&s, p.hover_thrust(), &Vector3::zeros(), &p, 0.01);
        }
        assert!((s.r.transpose() * s.r - Matrix3::identity()).norm() < 1e-6);
        assert!((s.r.determinant() - 1.0).abs() < 1e-6);
    }
}
