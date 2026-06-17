//! Desired trajectories. Each produces a [`Target`] = desired position `xd` and
//! heading direction `b1d`, *together with their time derivatives*.
//!
//! The original `matlab/controller.m` recovered these derivatives by numerically
//! differentiating `xd`/`b1d` with dirty-derivative filters. For the smooth,
//! analytically-known trajectories here we instead supply the derivatives in
//! closed form, which makes the feedforward exact and the tracking crisp (the
//! filters are still used for the *measured* velocity, which has no analytic
//! form). Constant/stepped targets simply leave the derivatives at zero.

use nalgebra::{Matrix3, Vector3};

/// Which quantity the controller is asked to track. The paper distinguishes an
/// attitude-controlled mode (track `Rd`, `Ωd`) from a position-controlled mode
/// (track `xd`, `b1d`); we add a velocity-controlled mode in between.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default)]
pub enum ControlMode {
    /// Track a desired position `xd` (and heading `b1d`). Full outer loop.
    #[default]
    Position,
    /// Track a desired velocity `ẋd` (and heading); no position feedback.
    Velocity,
    /// Track a desired attitude `Rd`, `Ωd` directly; thrust holds hover.
    Attitude,
}

impl ControlMode {
    pub fn label(&self) -> &'static str {
        match self {
            ControlMode::Position => "Position",
            ControlMode::Velocity => "Velocity",
            ControlMode::Attitude => "Attitude",
        }
    }
}

/// One leg of a [mode schedule](crate::sim::Sim): a flight mode + trajectory held
/// for `duration` seconds. Chaining several realises the paper's recommended
/// bring-up order — attitude → velocity → position → aggressive — in one run.
#[derive(Clone, Debug)]
pub struct FlightSegment {
    pub label: String,
    pub duration: f64,
    pub mode: ControlMode,
    pub trajectory: Trajectory,
}

impl FlightSegment {
    pub fn new(label: &str, duration: f64, mode: ControlMode, trajectory: Trajectory) -> Self {
        FlightSegment {
            label: label.to_string(),
            duration,
            mode,
            trajectory,
        }
    }
}

/// Desired flat outputs (and derivatives) fed to the controller. The position
/// fields drive [`ControlMode::Position`]/[`ControlMode::Velocity`]; the
/// attitude fields (`rd`, `omega_d`, `omega_d_dot`) drive
/// [`ControlMode::Attitude`].
#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub xd: Vector3<f64>,
    pub xd_dot: Vector3<f64>,
    pub xd_ddot: Vector3<f64>,
    pub xd_3dot: Vector3<f64>,
    pub xd_4dot: Vector3<f64>,
    pub b1d: Vector3<f64>,
    pub b1d_dot: Vector3<f64>,
    pub b1d_ddot: Vector3<f64>,
    /// Directly commanded attitude (used in [`ControlMode::Attitude`]).
    pub rd: Matrix3<f64>,
    pub omega_d: Vector3<f64>,
    pub omega_d_dot: Vector3<f64>,
}

impl Target {
    /// A held setpoint: position `xd`, heading `b1d`, all derivatives zero.
    pub fn hold(xd: Vector3<f64>, b1d: Vector3<f64>) -> Self {
        let z = Vector3::zeros();
        Target {
            xd,
            xd_dot: z,
            xd_ddot: z,
            xd_3dot: z,
            xd_4dot: z,
            b1d,
            b1d_dot: z,
            b1d_ddot: z,
            rd: Matrix3::identity(),
            omega_d: z,
            omega_d_dot: z,
        }
    }

    /// A held *attitude* setpoint `rd` (for [`ControlMode::Attitude`]).
    pub fn hold_attitude(rd: Matrix3<f64>) -> Self {
        Target {
            rd,
            ..Target::hold(Vector3::zeros(), Vector3::x())
        }
    }
}

impl Default for Target {
    fn default() -> Self {
        Target::hold(Vector3::zeros(), Vector3::x())
    }
}

/// Heading direction from a yaw angle (radians).
pub fn yaw_to_b1d(yaw: f64) -> Vector3<f64> {
    Vector3::new(yaw.cos(), yaw.sin(), 0.0)
}

/// A parametric desired trajectory. Heights use the NED convention (up is `-z`).
#[derive(Clone, Copy, Debug)]
pub enum Trajectory {
    /// Hold a fixed position and heading.
    Hover { pos: Vector3<f64>, yaw: f64 },
    /// Jump from `from` to `to` at `t_step` seconds.
    Step {
        from: Vector3<f64>,
        to: Vector3<f64>,
        t_step: f64,
        yaw: f64,
    },
    /// Horizontal circle of radius `radius` at angular rate `omega`, heading tangent.
    Circle {
        radius: f64,
        omega: f64,
        height: f64,
    },
    /// Horizontal figure-eight (Lissajous 1:2).
    Figure8 { scale: f64, omega: f64, height: f64 },
    /// Climbing helix.
    Helix {
        radius: f64,
        omega: f64,
        climb_rate: f64,
    },
    /// A user-definable 3-D Lissajous curve, fully parameterised (with analytic
    /// derivatives). `x = ax·sin(wx·t)`, `y = ay·sin(wy·t + py)`,
    /// `z = height + az·sin(wz·t + pz)`; heading fixed at `yaw`.
    Custom {
        ax: f64,
        ay: f64,
        az: f64,
        wx: f64,
        wy: f64,
        wz: f64,
        py: f64,
        pz: f64,
        height: f64,
        yaw: f64,
    },
    /// Hold a fixed *attitude* `exp(hat(axis_angle))` (for [`ControlMode::Attitude`]).
    HoldAttitude { axis_angle: Vector3<f64> },
    /// Spin in place about the vertical at `rate` rad/s — a *moving* attitude
    /// reference `Rd(t) = Rz(rate·t)` with `Ωd = (0, 0, rate)`. Stays upright, so
    /// it neither falls nor drifts (for [`ControlMode::Attitude`]).
    AttitudeSpin { rate: f64 },
    /// Externally driven target (set live from the UI).
    Live,
}

impl Trajectory {
    /// Sample the trajectory at time `t`. `live` is used only by [`Trajectory::Live`].
    pub fn sample(&self, t: f64, live: Target) -> Target {
        match *self {
            Trajectory::Hover { pos, yaw } => Target::hold(pos, yaw_to_b1d(yaw)),
            Trajectory::Step {
                from,
                to,
                t_step,
                yaw,
            } => Target::hold(if t < t_step { from } else { to }, yaw_to_b1d(yaw)),
            Trajectory::Circle {
                radius,
                omega,
                height,
            } => circle(radius, omega, height, 0.0, omega, t),
            Trajectory::Figure8 {
                scale,
                omega,
                height,
            } => figure8(scale, omega, height, t),
            Trajectory::Helix {
                radius,
                omega,
                climb_rate,
            } => circle(radius, omega, 0.0, -climb_rate, omega, t),
            Trajectory::Custom {
                ax,
                ay,
                az,
                wx,
                wy,
                wz,
                py,
                pz,
                height,
                yaw,
            } => custom(ax, ay, az, wx, wy, wz, py, pz, height, yaw, t),
            Trajectory::HoldAttitude { axis_angle } => {
                Target::hold_attitude(crate::math::so3_exp(&axis_angle))
            }
            Trajectory::AttitudeSpin { rate } => {
                let rd = crate::math::so3_exp(&Vector3::new(0.0, 0.0, rate * t));
                Target {
                    omega_d: Vector3::new(0.0, 0.0, rate),
                    ..Target::hold_attitude(rd)
                }
            }
            Trajectory::Live => live,
        }
    }
}

/// A general 3-D Lissajous with analytic derivatives (see [`Trajectory::Custom`]).
#[allow(clippy::too_many_arguments)]
fn custom(
    ax: f64,
    ay: f64,
    az: f64,
    wx: f64,
    wy: f64,
    wz: f64,
    py: f64,
    pz: f64,
    height: f64,
    yaw: f64,
    t: f64,
) -> Target {
    // Per-axis sinusoid s(t) = a·sin(w·t + p) and its first four derivatives.
    let axis = |a: f64, w: f64, p: f64, base: f64| {
        let (s, c) = (w * t + p).sin_cos();
        let w2 = w * w;
        [
            base + a * s,
            a * w * c,
            -a * w2 * s,
            -a * w2 * w * c,
            a * w2 * w2 * s,
        ]
    };
    let x = axis(ax, wx, 0.0, 0.0);
    let y = axis(ay, wy, py, 0.0);
    let z = axis(az, wz, pz, height);
    Target {
        xd: Vector3::new(x[0], y[0], z[0]),
        xd_dot: Vector3::new(x[1], y[1], z[1]),
        xd_ddot: Vector3::new(x[2], y[2], z[2]),
        xd_3dot: Vector3::new(x[3], y[3], z[3]),
        xd_4dot: Vector3::new(x[4], y[4], z[4]),
        b1d: yaw_to_b1d(yaw),
        b1d_dot: Vector3::zeros(),
        b1d_ddot: Vector3::zeros(),
        rd: Matrix3::identity(),
        omega_d: Vector3::zeros(),
        omega_d_dot: Vector3::zeros(),
    }
}

/// Circle / helix with analytic derivatives. `z0` + `vz`·t gives the (linear)
/// vertical profile (`vz = 0` => flat circle), heading tangent to the path.
fn circle(radius: f64, omega: f64, z0: f64, vz: f64, w: f64, t: f64) -> Target {
    let (s, c) = (omega * t).sin_cos();
    let (r, w2, w3, w4) = (radius, w * w, w * w * w, w * w * w * w);
    Target {
        xd: Vector3::new(r * c, r * s, z0 + vz * t),
        xd_dot: Vector3::new(-r * w * s, r * w * c, vz),
        xd_ddot: Vector3::new(-r * w2 * c, -r * w2 * s, 0.0),
        xd_3dot: Vector3::new(r * w3 * s, -r * w3 * c, 0.0),
        xd_4dot: Vector3::new(r * w4 * c, r * w4 * s, 0.0),
        b1d: Vector3::new(-s, c, 0.0),
        b1d_dot: Vector3::new(-w * c, -w * s, 0.0),
        b1d_ddot: Vector3::new(w2 * s, -w2 * c, 0.0),
        rd: Matrix3::identity(),
        omega_d: Vector3::zeros(),
        omega_d_dot: Vector3::zeros(),
    }
}

/// Figure-eight (x = A·sin(ωt), y = (A/2)·sin(2ωt)) with analytic derivatives.
fn figure8(scale: f64, omega: f64, height: f64, t: f64) -> Target {
    let a = scale;
    let (w, w2, w3, w4) = (omega, omega * omega, omega.powi(3), omega.powi(4));
    let (s1, c1) = (w * t).sin_cos();
    let (s2, c2) = (2.0 * w * t).sin_cos();
    Target {
        xd: Vector3::new(a * s1, 0.5 * a * s2, height),
        xd_dot: Vector3::new(a * w * c1, a * w * c2, 0.0),
        xd_ddot: Vector3::new(-a * w2 * s1, -2.0 * a * w2 * s2, 0.0),
        xd_3dot: Vector3::new(-a * w3 * c1, -4.0 * a * w3 * c2, 0.0),
        xd_4dot: Vector3::new(a * w4 * s1, 8.0 * a * w4 * s2, 0.0),
        b1d: Vector3::x(),
        b1d_dot: Vector3::zeros(),
        b1d_ddot: Vector3::zeros(),
        rd: Matrix3::identity(),
        omega_d: Vector3::zeros(),
        omega_d_dot: Vector3::zeros(),
    }
}
