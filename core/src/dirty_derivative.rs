//! Band-limited ("dirty") numerical derivative — a direct port of
//! `matlab/DirtyDerivative.m`.
//!
//! Implements the causal filtered differentiator with transfer function
//! `P(s) = s / (tau*s + 1)`, discretised with the Tustin (bilinear) transform:
//!
//! ```text
//! dot[k] = a1 * dot[k-1] + a2 * (x[k] - x[k-1])
//! a1 = (2*tau - Ts) / (2*tau + Ts)
//! a2 = 2 / (2*tau + Ts)
//! ```
//!
//! The filter is warmed up for `order` samples before it starts producing
//! output, mirroring the MATLAB `it > order` guard.

use nalgebra::Vector3;

#[derive(Clone, Debug)]
pub struct DirtyDerivative {
    a1: f64,
    a2: f64,
    order: u32,
    dot: Vector3<f64>,
    x_d1: Vector3<f64>,
    it: u32,
}

impl DirtyDerivative {
    /// `order` = which derivative this filter feeds (number of warm-up samples),
    /// `tau` = filter time constant, `ts` = sample time.
    pub fn new(order: u32, tau: f64, ts: f64) -> Self {
        DirtyDerivative {
            a1: (2.0 * tau - ts) / (2.0 * tau + ts),
            a2: 2.0 / (2.0 * tau + ts),
            order,
            dot: Vector3::zeros(),
            x_d1: Vector3::zeros(),
            it: 1,
        }
    }

    /// Feed the next sample, returning the current filtered derivative.
    pub fn calculate(&mut self, x: Vector3<f64>) -> Vector3<f64> {
        if self.it > self.order {
            self.dot = self.a1 * self.dot + self.a2 * (x - self.x_d1);
        }
        self.it += 1;
        self.x_d1 = x;
        self.dot
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn tracks_sine_derivative() {
        // d/dt sin(2*pi*f*t) = 2*pi*f*cos(2*pi*f*t)
        let ts = 0.001;
        let tau = 0.01;
        let freq = 1.0;
        let mut d = DirtyDerivative::new(1, tau, ts);
        let mut max_err: f64 = 0.0;
        for k in 0..3000 {
            let t = k as f64 * ts;
            let x = (2.0 * PI * freq * t).sin();
            let est = d.calculate(Vector3::new(x, 0.0, 0.0)).x;
            let truth = 2.0 * PI * freq * (2.0 * PI * freq * t).cos();
            if t > 0.5 {
                // after the filter transient settles
                max_err = max_err.max((est - truth).abs());
            }
        }
        // The residual is dominated by the filter's phase lag (atan(tau*omega));
        // the true derivative amplitude here is 2*pi ≈ 6.28, so this is ~6%.
        assert!(max_err < 0.5, "max derivative error too large: {max_err}");
    }
}
