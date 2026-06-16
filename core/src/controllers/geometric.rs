//! Geometric attitude error on SO(3): `eR = ½·vee(Rcᵀ·R − Rᵀ·Rc)`.
//! This is the formulation from Lee et al. and `matlab/controller.m` (eq. 21).

use crate::math::vee;
use nalgebra::{Matrix3, Vector3};

/// Coordinate-free attitude error. Global on SO(3); vanishes at `R == Rc` (and,
/// as an unstable equilibrium, at a 180° error where `sin(angle) -> 0`).
pub fn attitude_error(r: &Matrix3<f64>, rc: &Matrix3<f64>) -> Vector3<f64> {
    0.5 * vee(&(rc.transpose() * r - r.transpose() * rc))
}
