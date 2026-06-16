//! Pre-generated demonstration cases: an initial condition + a desired
//! trajectory + a flight mode + a short description shown in the UI.

use crate::math::so3_exp;
use crate::state::QuadState;
use crate::trajectory::{ControlMode, Trajectory};
use nalgebra::{Matrix3, Vector3};

/// A named demonstration case.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: String,
    pub blurb: String,
    pub initial: QuadState,
    pub trajectory: Trajectory,
    /// Which quantity the controllers track for this case.
    pub control_mode: ControlMode,
    /// Suggested run length [s] (`INFINITY` for the open-ended live case).
    pub duration: f64,
}

fn deg(x: f64) -> f64 {
    x.to_radians()
}

/// Default cruising altitude for the attitude-recovery demos, in NED `z`
/// (negative = above ground), so the airframe floats well above the render
/// plane instead of sitting on it.
const ALT: f64 = -3.0;

/// State at rest at `p` with orientation `r`.
fn at(p: Vector3<f64>, r: Matrix3<f64>) -> QuadState {
    QuadState::at(p, r)
}

/// State moving with velocity `v` (used so trajectory-tracking cases start *on*
/// the trajectory rather than with a large initial velocity error).
fn moving(p: Vector3<f64>, v: Vector3<f64>, r: Matrix3<f64>) -> QuadState {
    QuadState {
        p,
        v,
        r,
        omega: Vector3::zeros(),
    }
}

/// All built-in scenarios, in display order.
pub fn all() -> Vec<Scenario> {
    let up = Vector3::new(0.0, 0.0, ALT);
    vec![
        Scenario {
            name: "Hover".into(),
            blurb: "Upright hover 3 m above the pad — the trivial baseline.\n\
                    Expect: all three sit identical and rock-steady. Turn on the real-world \
                    plant and you'll see a small jitter and a few-cm offset from IMU noise, \
                    wind and mass mismatch."
                .into(),
            initial: at(up, Matrix3::identity()),
            trajectory: Trajectory::Hover { pos: up, yaw: 0.0 },
            control_mode: ControlMode::Position,
            duration: 6.0,
        },
        Scenario {
            name: "Upside-Down Recovery".into(),
            blurb: "Starts inverted (178.2° roll) and rights itself — the canonical SE(3) demo.\n\
                    Expect: all three flip upright and recover in ~2 s. Geometric recovers a touch \
                    slower near inversion (its error shrinks as it nears 180°) — that's correct, \
                    not a bug. Real-world on: still recovers, with a little extra drift."
                .into(),
            initial: at(up, so3_exp(&Vector3::new(deg(178.2), 0.0, 0.0))),
            trajectory: Trajectory::Hover { pos: up, yaw: 0.0 },
            control_mode: ControlMode::Position,
            duration: 8.0,
        },
        Scenario {
            name: "Exact 180° Flip".into(),
            blurb: "Starts at *exactly* 180° roll, an unstable equilibrium.\n\
                    Expect: quaternion (and Euler) flip decisively and recover; GEOMETRIC STALLS \
                    upside-down, because its error sin(angle)·axis vanishes at 180°. This is the \
                    single case the geometric law loses — its one blind spot."
                .into(),
            initial: at(up, so3_exp(&Vector3::new(std::f64::consts::PI, 0.0, 0.0))),
            trajectory: Trajectory::Hover { pos: up, yaw: 0.0 },
            control_mode: ControlMode::Position,
            duration: 8.0,
        },
        Scenario {
            name: "Gimbal-Lock Flip".into(),
            blurb: "Starts at 100° pitch — just past the Euler ±90° singularity.\n\
                    Expect (naive baselines ON): Euler tumbles wildly — the pitch plot spikes — \
                    while geometric/quaternion right themselves smoothly. In fair mode (naive \
                    OFF) all three recover, because they share the geometric law."
                .into(),
            initial: at(up, so3_exp(&Vector3::new(0.0, deg(100.0), 0.0))),
            trajectory: Trajectory::Hover { pos: up, yaw: 0.0 },
            control_mode: ControlMode::Position,
            duration: 8.0,
        },
        Scenario {
            name: "Pitch Inversion".into(),
            blurb: "Starts inverted about the pitch axis (175°); recovering forces the attitude \
                    through the Euler θ=90° singularity.\n\
                    Expect: geometric & quaternion sail through and recover. Euler is fragile at \
                    θ=90° — with naive baselines or the real-world plant on, it diverges while \
                    the other two succeed."
                .into(),
            initial: at(up, so3_exp(&Vector3::new(0.0, deg(175.0), 0.0))),
            trajectory: Trajectory::Hover { pos: up, yaw: 0.0 },
            control_mode: ControlMode::Position,
            duration: 8.0,
        },
        Scenario {
            name: "Quaternion Unwinding".into(),
            blurb: "Heading starts at +170° yaw; the command is −170° yaw — only 20° apart the \
                    short way.\n\
                    Expect (naive baselines ON): the naive quaternion UNWINDS the long way (≈340°, \
                    its Ψ plot swings all the way up to 2) because of SO(3)'s double cover, while \
                    geometric and the corrected quaternion snap the short 20°. The quad stays \
                    upright and in place, so you literally watch the heading spin almost full \
                    circle. (Naive OFF: all three take the short 20°.)"
                .into(),
            initial: at(up, so3_exp(&Vector3::new(0.0, 0.0, deg(170.0)))),
            trajectory: Trajectory::Hover {
                pos: up,
                yaw: deg(-170.0),
            },
            control_mode: ControlMode::Position,
            duration: 6.0,
        },
        Scenario {
            name: "Attitude Spin".into(),
            blurb: "ATTITUDE mode. Tracks a moving attitude reference — a steady spin about the \
                    vertical (Ωd ≠ 0).\n\
                    Expect: all three spin in place, tracking the reference almost perfectly \
                    (Ψ ≈ 0). There is NO position control in this mode, so with the real-world \
                    plant on the quad slowly drifts/sinks away even while the spin stays locked — \
                    that drift is the defining feature of attitude-only control, not a failure."
                .into(),
            initial: at(up, Matrix3::identity()),
            trajectory: Trajectory::AttitudeSpin {
                rate: std::f64::consts::PI, // half a turn per second
            },
            control_mode: ControlMode::Attitude,
            duration: 8.0,
        },
        Scenario {
            name: "Position Step".into(),
            blurb: "Steps from a 3 m hover to (2, 2, -4) m at t = 1 s.\n\
                    Expect: a crisp, identical move for all three — tilt to accelerate, then \
                    settle. Real-world on: slightly slower settle and a small steady-state offset."
                .into(),
            initial: at(up, Matrix3::identity()),
            trajectory: Trajectory::Step {
                from: up,
                to: Vector3::new(2.0, 2.0, -4.0),
                t_step: 1.0,
                yaw: 0.0,
            },
            control_mode: ControlMode::Position,
            duration: 8.0,
        },
        Scenario {
            name: "Velocity Cruise".into(),
            blurb: "VELOCITY mode on the circle's velocity profile, started 1.5 m off the path.\n\
                    Expect: the quad matches the commanded speed but NEVER closes the 1.5 m gap — \
                    it traces a circle parallel to the (white) target marker. That persistent \
                    offset is the signature of velocity control: it commands how fast to go, not \
                    where to be."
                .into(),
            initial: moving(
                Vector3::new(2.0, 1.5, ALT),
                Vector3::new(0.0, 2.0, 0.0),
                Matrix3::identity(),
            ),
            trajectory: Trajectory::Circle {
                radius: 2.0,
                omega: 1.0,
                height: ALT,
            },
            control_mode: ControlMode::Velocity,
            duration: 12.0,
        },
        Scenario {
            name: "Circle".into(),
            blurb: "Tracks a 2 m circle at 3 m altitude with tangent heading.\n\
                    Expect: tight, identical tracking for all three, nose pointing along the path. \
                    Real-world on: a steady ~10 cm lag behind the target from drag and motor lag."
                .into(),
            initial: moving(
                Vector3::new(2.0, 0.0, ALT),
                Vector3::new(0.0, 2.0 * 1.0, 0.0),
                Matrix3::identity(),
            ),
            trajectory: Trajectory::Circle {
                radius: 2.0,
                omega: 1.0,
                height: ALT,
            },
            control_mode: ControlMode::Position,
            duration: 12.0,
        },
        Scenario {
            name: "Aggressive Orbit".into(),
            blurb: "A fast 1.5 m orbit demanding sustained tilt and a rapidly slewing heading.\n\
                    Expect: geometric & quaternion track it tightly, even with the real-world \
                    plant on. Euler is the weak one here — under the real-world plant it loses \
                    the slew and flies off, while the other two stay locked on."
                .into(),
            initial: moving(
                Vector3::new(1.5, 0.0, ALT),
                Vector3::new(0.0, 1.5 * 1.8, 0.0),
                Matrix3::identity(),
            ),
            trajectory: Trajectory::Circle {
                radius: 1.5,
                omega: 1.8,
                height: ALT,
            },
            control_mode: ControlMode::Position,
            duration: 10.0,
        },
        Scenario {
            name: "Figure-Eight".into(),
            blurb: "Tracks a horizontal figure-eight at 3 m altitude.\n\
                    Expect: smooth tracking through both lobes and the crossing. Geometric & \
                    quaternion stay glued to it; with the real-world plant on, Euler struggles \
                    on the direction reversals while the other two hold."
                .into(),
            initial: at(Vector3::new(0.0, 0.0, ALT), Matrix3::identity()),
            trajectory: Trajectory::Figure8 {
                scale: 2.0,
                omega: 0.8,
                height: ALT,
            },
            control_mode: ControlMode::Position,
            duration: 16.0,
        },
        Scenario {
            name: "Custom Trajectory".into(),
            blurb: "A user-definable 3-D Lissajous — edit amplitudes/frequencies in the panel to \
                    design your own path.\n\
                    Expect: all three follow whatever curve you dial in; the sliders reshape the \
                    path live. Keep frequencies/amplitudes modest and they track tightly — crank \
                    them up to make it aggressive and watch tracking degrade."
                .into(),
            initial: at(Vector3::new(0.0, 0.0, ALT), Matrix3::identity()),
            trajectory: Trajectory::Custom {
                ax: 2.0,
                ay: 1.5,
                az: 0.5,
                wx: 0.5,
                wy: 1.0,
                wz: 0.5,
                py: 0.0,
                pz: std::f64::consts::FRAC_PI_2,
                height: ALT,
                yaw: 0.0,
            },
            control_mode: ControlMode::Position,
            duration: 20.0,
        },
        Scenario {
            name: "Live (interactive)".into(),
            blurb: "Drag the target position and heading sliders; the controllers track it live.\n\
                    Expect: smooth chasing of your setpoint. Push the target to extreme angles \
                    (with naive baselines on) to stress the Euler controller and see it lose the \
                    plot while geometric/quaternion stay composed."
                .into(),
            initial: at(up, Matrix3::identity()),
            trajectory: Trajectory::Live,
            control_mode: ControlMode::Position,
            duration: f64::INFINITY,
        },
    ]
}

/// Look up a scenario by exact name (case-insensitive) or a short token.
pub fn resolve(name: &str) -> Option<Scenario> {
    let cases = all();
    if let Some(s) = cases.iter().find(|s| s.name.eq_ignore_ascii_case(name)) {
        return Some(s.clone());
    }
    let canonical = match name.to_ascii_lowercase().as_str() {
        "hover" => "Hover",
        "upside_down" | "upsidedown" | "recovery" | "inverted" => "Upside-Down Recovery",
        "flip" | "exact180" | "180" => "Exact 180° Flip",
        "gimbal" | "near_gimbal" | "gimbal_lock" => "Gimbal-Lock Flip",
        "pitch" | "pitch_inversion" => "Pitch Inversion",
        "unwind" | "unwinding" | "double_cover" | "quaternion_unwinding" => "Quaternion Unwinding",
        "attitude" | "spin" | "attitude_spin" => "Attitude Spin",
        "step" | "position_step" => "Position Step",
        "velocity" | "cruise" | "velocity_cruise" => "Velocity Cruise",
        "circle" => "Circle",
        "orbit" | "aggressive" | "aggressive_orbit" => "Aggressive Orbit",
        "fig8" | "figure8" | "figure_eight" => "Figure-Eight",
        "custom" => "Custom Trajectory",
        "live" => "Live (interactive)",
        _ => return None,
    };
    cases.into_iter().find(|s| s.name == canonical)
}

impl Default for Scenario {
    fn default() -> Self {
        resolve("upside_down").unwrap()
    }
}
