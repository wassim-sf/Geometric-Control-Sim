//! Quaternion attitude error: `eR = 2·sign(qe_w)·qe_vec`, where
//! `qe = qc⁻¹ ⊗ q` is the error quaternion (current relative to desired).
//!
//! The `sign(qe_w)` correction always chooses the short way around, avoiding the
//! "unwinding" that the double cover of SO(3) would otherwise cause. The factor
//! `2` matches the small-angle scaling of the geometric error (`eR ≈ angle·axis`).
//! Unlike the geometric error, this one does *not* vanish at a 180° error
//! (`|eR| -> 2`), so it commands an aggressive escape from inversion.

use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

/// Sign-corrected quaternion attitude error — the *correct* law. It is computed
/// fresh from the rotation matrices each step, so it is immune to which
/// quaternion sheet the estimate lives on: `sign(qe_w)` always picks the short
/// way. This is what guards against the double-cover "unwinding".
pub fn attitude_error(r: &Matrix3<f64>, rc: &Matrix3<f64>) -> Vector3<f64> {
    let qe = error_quat(r, rc);
    let sign = if qe.scalar() < 0.0 { -1.0 } else { 1.0 };
    2.0 * sign * qe.imag()
}

/// The error quaternion `qe = qc⁻¹ ⊗ q` (current attitude relative to desired).
pub fn error_quat(r: &Matrix3<f64>, rc: &Matrix3<f64>) -> UnitQuaternion<f64> {
    let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*r));
    let qc = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*rc));
    qc.inverse() * q
}

/// A stateful, sign-*naive* quaternion error that exhibits the double-cover
/// **unwinding**.
///
/// Instantaneous matrix→quaternion feedback can never unwind: the rotation
/// matrix erases which of the two antipodal quaternions (`q` vs `−q`) you are
/// on, and `nalgebra` hands back a continuous representative. Unwinding is a
/// property of a quaternion *estimator/controller* that carries a quaternion
/// state and can latch onto the wrong sheet.
///
/// [`NaiveQuatTracker`] reproduces exactly that failure: it carries the error
/// quaternion across steps, keeping it sign-*continuous* (so it never jumps
/// sheets) and **never** applying the shortest-path `sign(qe_w)` correction.
/// Seeded on the antipodal sheet (a quaternion estimate that converged to `−q`),
/// it commands the long way around — an almost-full rotation where the corrected
/// law takes the short path. This is the textbook reason the sign correction
/// exists.
#[derive(Clone, Debug, Default)]
pub struct NaiveQuatTracker {
    qe: Option<UnitQuaternion<f64>>,
}

impl NaiveQuatTracker {
    pub fn new() -> Self {
        NaiveQuatTracker { qe: None }
    }

    /// Update with the current matrices and return `eR = 2·qe_vec` (no
    /// shortest-path correction).
    pub fn error(&mut self, r: &Matrix3<f64>, rc: &Matrix3<f64>) -> Vector3<f64> {
        let fresh = error_quat(r, rc);
        let qe = match self.qe {
            // First call: seed on the *antipodal* sheet (the estimate latched
            // onto −q). With no sign correction this forces the unwinding.
            None => negate(&fresh),
            // Subsequent calls: keep the quaternion continuous (flip `fresh` if
            // it jumped to the opposite sheet), so the unwinding plays out
            // smoothly instead of snapping back.
            Some(prev) => {
                if fresh.coords.dot(&prev.coords) < 0.0 {
                    negate(&fresh)
                } else {
                    fresh
                }
            }
        };
        self.qe = Some(qe);
        2.0 * qe.imag()
    }
}

/// The antipodal unit quaternion (same rotation, opposite sheet).
fn negate(q: &UnitQuaternion<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::new_unchecked(-q.into_inner())
}
