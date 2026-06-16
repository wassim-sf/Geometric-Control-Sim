//! Quadrotor rigid-body state on SE(3) plus its time derivative.

use nalgebra::{Matrix3, Vector3};

/// Full state of the quadrotor.
///
/// Convention follows the paper / `matlab/quad_dynamics.m` (NED-like, gravity
/// along `+e3`):
/// * `p` — position in the inertial frame,
/// * `v` — velocity in the inertial frame,
/// * `r` — body→inertial rotation `R ∈ SO(3)`,
/// * `omega` — angular velocity in the body frame.
#[derive(Clone, Debug)]
pub struct QuadState {
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub r: Matrix3<f64>,
    pub omega: Vector3<f64>,
}

impl QuadState {
    /// State at rest at position `p` with orientation `r`.
    pub fn at(p: Vector3<f64>, r: Matrix3<f64>) -> Self {
        QuadState {
            p,
            v: Vector3::zeros(),
            r,
            omega: Vector3::zeros(),
        }
    }

    /// Whether every component is finite (used for divergence detection).
    pub fn is_finite(&self) -> bool {
        self.p.iter().all(|x| x.is_finite())
            && self.v.iter().all(|x| x.is_finite())
            && self.r.iter().all(|x| x.is_finite())
            && self.omega.iter().all(|x| x.is_finite())
    }

    /// `self + h * k`, component-wise. The resulting `r` is generally not on
    /// SO(3) (it is an RK4 intermediate); the integrator re-projects at the end.
    pub fn add_scaled(&self, k: &StateDot, h: f64) -> QuadState {
        QuadState {
            p: self.p + k.p * h,
            v: self.v + k.v * h,
            r: self.r + k.r * h,
            omega: self.omega + k.omega * h,
        }
    }
}

/// Time derivative of [`QuadState`].
#[derive(Clone, Debug)]
pub struct StateDot {
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub r: Matrix3<f64>,
    pub omega: Vector3<f64>,
}
