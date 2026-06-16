//! Lie-group helpers for SO(3): `hat`/`vee`, the exponential map, SO(3)
//! projection, and a ZYX Euler extraction. Direct ports of `matlab/hat.m`,
//! `matlab/vee.m`, and `expm(hat(.))`.

use nalgebra::{Matrix3, Vector3};

/// `hat`: maps a vector in R^3 to its skew-symmetric matrix in so(3).
///
/// Matches `matlab/hat.m`: `hat(v) * w == v.cross(w)`.
pub fn hat(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y, //
        v.z, 0.0, -v.x, //
        -v.y, v.x, 0.0,
    )
}

/// `vee`: inverse of [`hat`], extracting the vector from a skew-symmetric matrix.
///
/// Matches `matlab/vee.m`: `[m(3,2); m(1,3); m(2,1)]` (1-indexed).
pub fn vee(m: &Matrix3<f64>) -> Vector3<f64> {
    Vector3::new(m[(2, 1)], m[(0, 2)], m[(1, 0)])
}

/// SO(3) exponential map via Rodrigues' formula: `exp(hat(w))`.
///
/// Equivalent to MATLAB `expm(hat(w))` for `w` a rotation vector (axis * angle).
pub fn so3_exp(w: &Vector3<f64>) -> Matrix3<f64> {
    let theta = w.norm();
    if theta < 1e-12 {
        return Matrix3::identity() + hat(w);
    }
    let k = w / theta;
    let kx = hat(&k);
    Matrix3::identity() + theta.sin() * kx + (1.0 - theta.cos()) * (kx * kx)
}

/// Project a matrix back onto SO(3) (nearest rotation, in Frobenius norm) via
/// SVD. Used after each integration step to cancel numerical drift of `R`.
pub fn project_to_so3(m: &Matrix3<f64>) -> Matrix3<f64> {
    let svd = m.svd(true, true);
    let u = svd.u.expect("SVD U");
    let v_t = svd.v_t.expect("SVD V^T");
    // Guard against reflections: ensure det = +1.
    let d = (u * v_t).determinant().signum();
    let s = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d));
    u * s * v_t
}

/// Extract ZYX (yaw-pitch-roll) Euler angles from a rotation matrix
/// `R = Rz(yaw) * Ry(pitch) * Rx(roll)`. Returns `(roll, pitch, yaw)` in radians.
///
/// This is deliberately the naive aerospace extraction so the Euler-angle
/// controller exhibits gimbal lock as `pitch -> ±pi/2`.
pub fn r_to_zyx(r: &Matrix3<f64>) -> (f64, f64, f64) {
    let roll = r[(2, 1)].atan2(r[(2, 2)]);
    let pitch = (-r[(2, 0)]).clamp(-1.0, 1.0).asin();
    let yaw = r[(1, 0)].atan2(r[(0, 0)]);
    (roll, pitch, yaw)
}

/// Map body angular velocity `Ω = (p, q, r)` to ZYX Euler-angle rates
/// `(φ̇, θ̇, ψ̇)` for `roll = φ`, `pitch = θ`.
///
/// This kinematic Jacobian contains `tan(θ)` and `1/cos(θ)` terms that **blow up
/// at θ = ±90°** — the gimbal-lock singularity. A naive Euler-angle controller
/// that feeds back Euler rates inherits this blow-up.
pub fn body_to_euler_rates(roll: f64, pitch: f64, omega: &Vector3<f64>) -> Vector3<f64> {
    let (sphi, cphi) = roll.sin_cos();
    let (sth, cth) = pitch.sin_cos();
    let tth = sth / cth;
    let (p, q, r) = (omega.x, omega.y, omega.z);
    Vector3::new(
        p + (q * sphi + r * cphi) * tth,
        q * cphi - r * sphi,
        (q * sphi + r * cphi) / cth,
    )
}

/// Wrap an angle to (-pi, pi].
pub fn wrap_angle(a: f64) -> f64 {
    use std::f64::consts::PI;
    let mut x = a;
    while x > PI {
        x -= 2.0 * PI;
    }
    while x < -PI {
        x += 2.0 * PI;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn hat_vee_roundtrip() {
        let v = Vector3::new(0.3, -1.2, 0.7);
        assert!((vee(&hat(&v)) - v).norm() < 1e-12);
    }

    #[test]
    fn hat_is_cross_product() {
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(-0.5, 0.4, 2.1);
        assert!((hat(&a) * b - a.cross(&b)).norm() < 1e-12);
    }

    #[test]
    fn so3_exp_rotates_about_z() {
        let r = so3_exp(&Vector3::new(0.0, 0.0, PI / 2.0));
        // x-axis should map to y-axis.
        let x = Vector3::new(1.0, 0.0, 0.0);
        assert!((r * x - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-9);
        // valid rotation
        assert!((r.determinant() - 1.0).abs() < 1e-9);
        assert!((r.transpose() * r - Matrix3::identity()).norm() < 1e-9);
    }

    #[test]
    fn zyx_roundtrip_away_from_singularity() {
        let (roll, pitch, yaw) = (0.3, -0.6, 1.1);
        let r = so3_exp(&Vector3::new(0.0, 0.0, yaw))
            * so3_exp(&Vector3::new(0.0, pitch, 0.0))
            * so3_exp(&Vector3::new(roll, 0.0, 0.0));
        let (r2, p2, y2) = r_to_zyx(&r);
        assert!((r2 - roll).abs() < 1e-9);
        assert!((p2 - pitch).abs() < 1e-9);
        assert!((y2 - yaw).abs() < 1e-9);
    }

    #[test]
    fn project_fixes_drift() {
        let mut r = so3_exp(&Vector3::new(0.1, 0.2, 0.3));
        r[(0, 0)] += 1e-3; // perturb
        let p = project_to_so3(&r);
        assert!((p.transpose() * p - Matrix3::identity()).norm() < 1e-9);
        assert!((p.determinant() - 1.0).abs() < 1e-9);
    }
}
