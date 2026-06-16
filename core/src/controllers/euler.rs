//! Naive Euler-angle attitude error: `eR = wrap(zyx(R) − zyx(Rc))`, treated
//! directly as a body-frame error `[roll, pitch, yaw]`.
//!
//! This is the textbook small-angle approach. It works near hover but is
//! singular at `pitch = ±90°` (gimbal lock): the ZYX extraction loses a degree
//! of freedom there, so roll/yaw errors become ill-defined and the controller
//! tumbles. That failure is exactly what the side-by-side comparison showcases.

use crate::math::{r_to_zyx, wrap_angle};
use nalgebra::{Matrix3, Vector3};

pub fn attitude_error(r: &Matrix3<f64>, rc: &Matrix3<f64>) -> Vector3<f64> {
    let (roll, pitch, yaw) = r_to_zyx(r);
    let (roll_c, pitch_c, yaw_c) = r_to_zyx(rc);
    Vector3::new(
        wrap_angle(roll - roll_c),
        wrap_angle(pitch - pitch_c),
        wrap_angle(yaw - yaw_c),
    )
}
