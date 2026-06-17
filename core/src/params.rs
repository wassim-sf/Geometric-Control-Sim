//! Physical parameters, control gains, and the rotor mixing matrix.
//! Values are taken verbatim from `matlab/param.m`.

use nalgebra::{Matrix3, Matrix4, Vector3, Vector4};

/// Airframe physical parameters and controller gains.
#[derive(Clone, Debug)]
pub struct QuadParams {
    /// Gravitational acceleration [m/s^2].
    pub gravity: f64,
    /// Mass [kg].
    pub mass: f64,
    /// Inertia matrix `diag(Jxx, Jyy, Jzz)` [kg·m^2].
    pub j: Matrix3<f64>,
    /// Inverse inertia matrix (precomputed).
    pub j_inv: Matrix3<f64>,
    /// Distance from CoM to each rotor in the b1-b2 plane [m].
    pub d: f64,
    /// Rotor moment-to-thrust coupling coefficient [m].
    pub c_tauf: f64,
    /// Number of rotors.
    pub n_rotors: usize,
    /// Controller sample time [s].
    pub ts: f64,
    /// Dirty-derivative filter time constant [s].
    pub tau: f64,

    // --- control gains (Lee 2011) ---
    pub kx: f64,
    pub kv: f64,
    pub kr: f64,
    pub komega: f64,
    /// Position integral gain (0 = pure PD, as in the paper). A small `ki`
    /// rejects steady disturbances (wind, battery sag, mass/CoM mismatch).
    pub ki: f64,
    /// Anti-windup bound on each component of the position-error integral [m·s].
    pub i_max: f64,

    /// Mixing matrix: `[f1,f2,f3,f4]^T = mix * [f, Mx, My, Mz]^T`.
    pub mix: Matrix4<f64>,
    /// Forward wrench map: `[f, Mx, My, Mz]^T = wrench * [f1,f2,f3,f4]^T`.
    pub wrench: Matrix4<f64>,
}

impl Default for QuadParams {
    fn default() -> Self {
        let mass = 4.34;
        let d = 0.315;
        let c_tauf = 8.004e-3;

        // [f; Mx; My; Mz] = wrench * [f1; f2; f3; f4]   (matlab/param.m)
        let wrench = Matrix4::new(
            1.0, 1.0, 1.0, 1.0, //
            0.0, -d, 0.0, d, //
            d, 0.0, -d, 0.0, //
            -c_tauf, c_tauf, -c_tauf, c_tauf,
        );
        let mix = wrench.try_inverse().expect("mixing matrix is invertible");

        let j = Matrix3::from_diagonal(&Vector3::new(0.0820, 0.0845, 0.1377));

        QuadParams {
            gravity: 9.81,
            mass,
            j,
            j_inv: j.try_inverse().expect("inertia invertible"),
            d,
            c_tauf,
            n_rotors: 4,
            ts: 0.01,
            tau: 0.05,
            kx: 4.0 * mass,
            kv: 5.6 * mass,
            kr: 8.81,
            komega: 2.54,
            ki: 0.0,
            i_max: 3.0,
            mix,
            wrench,
        }
    }
}

impl QuadParams {
    /// Map a thrust + body moment to the four individual rotor forces.
    pub fn rotor_forces(&self, f: f64, moment: &Vector3<f64>) -> Vector4<f64> {
        self.mix * Vector4::new(f, moment.x, moment.y, moment.z)
    }

    /// Hover thrust `m*g`.
    pub fn hover_thrust(&self) -> f64 {
        self.mass * self.gravity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mixing_roundtrip() {
        let p = QuadParams::default();
        let f = 42.0;
        let m = Vector3::new(0.5, -0.3, 0.1);
        let rotors = p.rotor_forces(f, &m);
        let back = p.wrench * rotors;
        assert!((back - Vector4::new(f, m.x, m.y, m.z)).norm() < 1e-9);
    }

    #[test]
    fn gains_match_matlab() {
        let p = QuadParams::default();
        assert!((p.kx - 4.0 * 4.34).abs() < 1e-12);
        assert!((p.kv - 5.6 * 4.34).abs() < 1e-12);
        assert!((p.kr - 8.81).abs() < 1e-12);
        assert!((p.komega - 2.54).abs() < 1e-12);
    }
}
