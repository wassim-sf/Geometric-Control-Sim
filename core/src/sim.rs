//! Simulation runner. Holds one or more controllers, each integrating its own
//! copy of the state from the scenario's shared initial condition, so the
//! formulations can be compared as they converge (or diverge) under the same
//! commanded trajectory.

use crate::controllers::{AttitudeMode, GeometricController};
use crate::math::r_to_zyx;
use crate::params::QuadParams;
use crate::plant::Plant;
use crate::realism::Realism;
use crate::safety::Safety;
use crate::scenario::{self, Scenario};
use crate::state::QuadState;
use crate::trajectory::{ControlMode, FlightSegment, Target, Trajectory};
use nalgebra::{Matrix3, Vector3};

/// One logged time sample for a single controller.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t: f64,
    pub pos: Vector3<f64>,
    /// `||x - xd||`.
    pub pos_err: f64,
    /// SO(3) configuration error `Ψ`.
    pub psi: f64,
    /// Total thrust [N].
    pub f: f64,
    /// Per-rotor forces.
    pub rotor: [f64; 4],
    /// ZYX pitch of the actual attitude [rad] — spikes at gimbal lock.
    pub pitch: f64,
}

/// A single controller plus the airframe it drives and its logged history.
#[derive(Clone, Debug)]
pub struct ControllerInstance {
    pub mode: AttitudeMode,
    pub state: QuadState,
    pub controller: GeometricController,
    /// The physical airframe (actuator state, sensors, real-world effects).
    pub plant: Plant,
    pub history: Vec<Sample>,
    /// Set once the state goes non-finite or flies away; integration freezes.
    pub diverged: bool,
}

/// The simulation: shared clock + trajectory, one instance per active controller.
#[derive(Clone)]
pub struct Sim {
    pub params: QuadParams,
    pub scenario: Scenario,
    pub trajectory: Trajectory,
    /// Target used when the trajectory is [`Trajectory::Live`].
    pub live_target: Target,
    pub t: f64,
    pub instances: Vec<ControllerInstance>,
    /// History of the objective (target) position — the path of the white
    /// marker, logged each step so it can be drawn as a trail.
    pub target_history: Vec<Vector3<f64>>,
    /// Cap on retained history samples per instance.
    pub max_history: usize,
    /// When true, baseline (non-geometric) controllers use their realistic
    /// standalone form: the naive Euler controller fails at gimbal lock, and the
    /// quaternion controller drops its sign correction (double-cover unwinding).
    pub naive_baselines: bool,
    /// When true, the Euler and quaternion baselines run as **standalone,
    /// single-domain** controllers (Euler PID with its own Euler-angle loop; a
    /// quaternion PD), with no sharing of the geometric loop. Takes precedence
    /// over `naive_baselines` for those two controllers.
    pub standalone_baselines: bool,
    /// Which quantity the controllers track (from the scenario). When a
    /// [`Sim::schedule`] is active this is overridden per-segment.
    pub control_mode: ControlMode,
    /// Real-world plant effects shared by every airframe.
    pub realism: Realism,
    /// Real-drone safety supervisor (arming, thrust/tilt limits, ground floor).
    pub safety: Safety,
    /// Optional timed flight-mode schedule (empty = single mode/trajectory).
    pub schedule: Vec<FlightSegment>,
}

impl Sim {
    pub fn new(scenario: Scenario, modes: &[AttitudeMode], params: QuadParams) -> Self {
        let trajectory = scenario.trajectory;
        let control_mode = scenario.control_mode;
        let live_target = Target::hold(scenario.initial.p, Vector3::x());
        let realism = Realism::ideal();
        let schedule = scenario::schedule_for(&scenario.name);
        let instances = build_instances(&scenario, modes, &params, &realism);
        Sim {
            params,
            scenario,
            trajectory,
            live_target,
            t: 0.0,
            instances,
            target_history: Vec::new(),
            max_history: 6000,
            naive_baselines: false,
            standalone_baselines: false,
            control_mode,
            realism,
            safety: Safety::off(),
            schedule,
        }
    }

    /// The (mode, trajectory) in force at time `t` — the active schedule segment,
    /// or the scenario's single mode/trajectory when there is no schedule.
    fn effective(&self, t: f64) -> (ControlMode, Trajectory) {
        if let Some(seg) = self.active_segment_at(t) {
            (seg.mode, seg.trajectory)
        } else {
            (self.control_mode, self.trajectory)
        }
    }

    /// The active schedule segment at time `t`, if a schedule is running.
    fn active_segment_at(&self, t: f64) -> Option<&FlightSegment> {
        if self.schedule.is_empty() {
            return None;
        }
        let mut acc = 0.0;
        for seg in &self.schedule {
            acc += seg.duration;
            if t < acc {
                return Some(seg);
            }
        }
        self.schedule.last()
    }

    /// Label of the currently-active schedule segment (for the UI), if any.
    pub fn active_segment_label(&self) -> Option<&str> {
        self.active_segment_at(self.t).map(|s| s.label.as_str())
    }

    /// The flight mode currently in force (schedule-aware).
    pub fn active_mode(&self) -> ControlMode {
        self.effective(self.t).0
    }

    /// Current commanded target.
    pub fn current_target(&self) -> Target {
        let (_, traj) = self.effective(self.t);
        traj.sample(self.t, self.live_target)
    }

    /// The commanded target at an arbitrary absolute time `t` (schedule-aware) —
    /// used to draw the expected reference path ahead of the vehicle.
    pub fn target_at(&self, t: f64) -> Target {
        let (_, traj) = self.effective(t);
        traj.sample(t, self.live_target)
    }

    /// Advance every active controller by one controller period (`params.ts`).
    pub fn step(&mut self) {
        let dt = self.params.ts;
        let t = self.t;
        let (cmode, traj) = self.effective(t);
        let target = traj.sample(t, self.live_target);
        // Log the objective position (the white marker's path).
        self.target_history.push(target.xd);
        if self.target_history.len() > self.max_history {
            let excess = self.target_history.len() - self.max_history;
            self.target_history.drain(0..excess);
        }
        let naive = self.naive_baselines;
        let floor_down = if self.safety.enabled {
            -self.safety.altitude_floor
        } else {
            f64::INFINITY
        };
        for inst in self.instances.iter_mut() {
            if inst.diverged {
                continue;
            }
            // The controller flies on the *measured* (delayed/noisy) state.
            let meas = inst.plant.measure(&inst.state);
            // Estimator-health failsafe: a non-finite estimate trips divergence
            // rather than feeding garbage into the control law.
            if !meas.is_finite() {
                inst.diverged = true;
                continue;
            }
            let out = match (self.standalone_baselines, inst.mode) {
                (true, AttitudeMode::Euler) => {
                    inst.controller
                        .control_euler_pid(&meas, &target, &self.params, cmode, &self.safety)
                }
                (true, AttitudeMode::Quaternion) => {
                    inst.controller
                        .control_quaternion_pd(&meas, &target, &self.params, cmode, &self.safety)
                }
                _ => inst
                    .controller
                    .control(&meas, &target, &self.params, naive, cmode, &self.safety),
            };

            // Log at the *true* pre-integration state so pos/Ψ line up at `t`.
            let (_, pitch, _) = r_to_zyx(&inst.state.r);
            let psi_true = 0.5 * (Matrix3::identity() - out.rc.transpose() * inst.state.r).trace();
            push_sample(
                inst,
                Sample {
                    t,
                    pos: inst.state.p,
                    pos_err: (inst.state.p - target.xd).norm(),
                    psi: psi_true,
                    f: out.f,
                    rotor: [
                        out.rotor_forces[0],
                        out.rotor_forces[1],
                        out.rotor_forces[2],
                        out.rotor_forces[3],
                    ],
                    pitch,
                },
                self.max_history,
            );

            // The airframe responds to the commanded rotor forces (through
            // actuator limits/lag and any real-world effects).
            inst.state = inst.plant.step(&inst.state, out.rotor_forces, dt, t);

            // Ground contact / altitude floor: the airframe can't sink below the
            // pad (down `z` is bounded; downward velocity is arrested on contact).
            if inst.state.p.z > floor_down {
                inst.state.p.z = floor_down;
                if inst.state.v.z > 0.0 {
                    inst.state.v.z = 0.0;
                }
            }
            if !inst.state.is_finite() || inst.state.p.norm() > 1.0e3 {
                inst.diverged = true;
            }
        }
        self.t += dt;
    }

    /// Step until the clock reaches `secs`.
    pub fn run_for(&mut self, secs: f64) {
        while self.t < secs {
            self.step();
        }
    }

    /// Reset the clock, states, controllers, airframes, and history (same
    /// scenario/modes/realism).
    pub fn reset(&mut self) {
        self.t = 0.0;
        self.target_history.clear();
        for inst in &mut self.instances {
            inst.state = self.scenario.initial.clone();
            inst.controller = GeometricController::new(inst.mode, &self.params);
            inst.plant = Plant::new(&self.params, &self.realism);
            inst.history.clear();
            inst.diverged = false;
        }
    }

    /// Swap in a new scenario and reset.
    pub fn set_scenario(&mut self, scenario: Scenario) {
        let modes: Vec<AttitudeMode> = self.instances.iter().map(|i| i.mode).collect();
        self.trajectory = scenario.trajectory;
        self.control_mode = scenario.control_mode;
        self.schedule = scenario::schedule_for(&scenario.name);
        self.live_target = Target::hold(scenario.initial.p, Vector3::x());
        self.instances = build_instances(&scenario, &modes, &self.params, &self.realism);
        self.scenario = scenario;
        self.t = 0.0;
        self.target_history.clear();
    }

    /// Replace the safety supervisor (does not rebuild the airframes).
    pub fn set_safety(&mut self, safety: Safety) {
        self.safety = safety;
    }

    /// Change the set of active controllers and reset.
    pub fn set_modes(&mut self, modes: &[AttitudeMode]) {
        self.instances = build_instances(&self.scenario, modes, &self.params, &self.realism);
        self.t = 0.0;
        self.target_history.clear();
    }

    /// Replace the real-world effects and rebuild the airframes (clears history).
    pub fn set_realism(&mut self, realism: Realism) {
        self.realism = realism;
        let modes: Vec<AttitudeMode> = self.instances.iter().map(|i| i.mode).collect();
        self.instances = build_instances(&self.scenario, &modes, &self.params, &self.realism);
        self.t = 0.0;
        self.target_history.clear();
    }

    /// Find the instance for a given mode, if active.
    pub fn instance(&self, mode: AttitudeMode) -> Option<&ControllerInstance> {
        self.instances.iter().find(|i| i.mode == mode)
    }
}

fn build_instances(
    scenario: &Scenario,
    modes: &[AttitudeMode],
    params: &QuadParams,
    realism: &Realism,
) -> Vec<ControllerInstance> {
    modes
        .iter()
        .map(|&mode| ControllerInstance {
            mode,
            state: scenario.initial.clone(),
            controller: GeometricController::new(mode, params),
            plant: Plant::new(params, realism),
            history: Vec::new(),
            diverged: false,
        })
        .collect()
}

fn push_sample(inst: &mut ControllerInstance, sample: Sample, max_history: usize) {
    inst.history.push(sample);
    if inst.history.len() > max_history {
        let excess = inst.history.len() - max_history;
        inst.history.drain(0..excess);
    }
}
