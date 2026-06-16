//! Conversion between the simulation's NED-like frame and Bevy's y-up frame.
//!
//! The core integrates in the paper's frame `(north, east, down)` with gravity
//! along `+down`. Bevy renders in a right-handed, y-up frame. The change of
//! basis `C` below is a *proper* rotation (det = +1) mapping
//! `(n, e, d) -> (east, -down, -north)`, i.e. up becomes `+y` and north becomes
//! `-z` ("into the screen"). Applying `C` keeps the quad's chirality correct.

use bevy::prelude::{Quat, Vec3};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

/// `v_bevy = C · v_ned`.
#[rustfmt::skip]
fn c_matrix() -> Matrix3<f64> {
    Matrix3::new(
        0.0, 1.0,  0.0,
        0.0, 0.0, -1.0,
       -1.0, 0.0,  0.0,
    )
}

/// Convert an NED position to a Bevy world position.
pub fn pos_to_bevy(p: &Vector3<f64>) -> Vec3 {
    Vec3::new(p.y as f32, -p.z as f32, -p.x as f32)
}

/// Convert an NED direction (free vector) to a Bevy direction.
pub fn dir_to_bevy(d: &Vector3<f64>) -> Vec3 {
    Vec3::new(d.y as f32, -d.z as f32, -d.x as f32)
}

/// Convert a body→NED rotation matrix to the Bevy world orientation of a
/// body-authored mesh (`M = C · R`).
pub fn rot_to_bevy(r: &Matrix3<f64>) -> Quat {
    let m = c_matrix() * r;
    let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(m));
    Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn up_maps_to_plus_y() {
        // NED "up" is -down = (0,0,-1); should render at +y.
        let up = dir_to_bevy(&Vector3::new(0.0, 0.0, -1.0));
        assert!((up - Vec3::Y).length() < 1e-6);
    }

    #[test]
    fn change_of_basis_is_proper_rotation() {
        assert!((c_matrix().determinant() - 1.0).abs() < 1e-9);
    }
}
