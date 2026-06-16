//! Simulation runner. Holds one or more controllers, each integrating its own
//! copy of the state from the scenario's shared initial condition, so the
//! formulations can be compared as they converge (or diverge) under the same
//! commanded trajectory.

use crate::controllers::{AttitudeMode, GeometricController};
use crate::math::r_to_zyx;
use crate::params::QuadParams;
use crate::plant::Plant;
use crate::realism::Realism;
use crate::scenario::Scenario;
use crate::state::QuadState;
use crate::trajectory::{ControlMode, Target, Trajectory};
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
    /// Cap on retained history samples per instance.
    pub max_history: usize,
    /// When true, baseline (non-geometric) controllers use their realistic
    /// standalone form: the naive Euler controller fails at gimbal lock, and the
    /// quaternion controller drops its sign correction (double-cover unwinding).
    pub naive_baselines: bool,
    /// Which quantity the controllers track (from the scenario).
    pub control_mode: ControlMode,
    /// Real-world plant effects shared by every airframe.
    pub realism: Realism,
}

impl Sim {
    pub fn new(scenario: Scenario, modes: &[AttitudeMode], params: QuadParams) -> Self {
        let trajectory = scenario.trajectory;
        let control_mode = scenario.control_mode;
        let live_target = Target::hold(scenario.initial.p, Vector3::x());
        let realism = Realism::ideal();
        let instances = build_instances(&scenario, modes, &params, &realism);
        Sim {
            params,
            scenario,
            trajectory,
            live_target,
            t: 0.0,
            instances,
            max_history: 6000,
            naive_baselines: false,
            control_mode,
            realism,
        }
    }

    /// Current commanded target.
    pub fn current_target(&self) -> Target {
        self.trajectory.sample(self.t, self.live_target)
    }

    /// Advance every active controller by one controller period (`params.ts`).
    pub fn step(&mut self) {
        let dt = self.params.ts;
        let target = self.trajectory.sample(self.t, self.live_target);
        let t = self.t;
        let naive = self.naive_baselines;
        let cmode = self.control_mode;
        for inst in self.instances.iter_mut() {
            if inst.diverged {
                continue;
            }
            // The controller flies on the *measured* (delayed/noisy) state.
            let meas = inst.plant.measure(&inst.state);
            let out = inst.controller.control(&meas, &target, &self.params, naive, cmode);

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
        self.live_target = Target::hold(scenario.initial.p, Vector3::x());
        self.instances = build_instances(&scenario, &modes, &self.params, &self.realism);
        self.scenario = scenario;
        self.t = 0.0;
    }

    /// Change the set of active controllers and reset.
    pub fn set_modes(&mut self, modes: &[AttitudeMode]) {
        self.instances = build_instances(&self.scenario, modes, &self.params, &self.realism);
        self.t = 0.0;
    }

    /// Replace the real-world effects and rebuild the airframes (clears history).
    pub fn set_realism(&mut self, realism: Realism) {
        self.realism = realism;
        let modes: Vec<AttitudeMode> = self.instances.iter().map(|i| i.mode).collect();
        self.instances = build_instances(&self.scenario, &modes, &self.params, &self.realism);
        self.t = 0.0;
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
