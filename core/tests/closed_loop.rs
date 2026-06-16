//! End-to-end behavioural tests of the closed loop. These reproduce the
//! demonstration the MATLAB project is built around (upside-down recovery) and
//! verify the central claim of the comparison: the geometric/quaternion
//! controllers handle large attitudes that break the Euler-angle controller.

use se3quad_core::{scenario, AttitudeMode, QuadParams, Realism, Sim};

fn peak_psi(sim: &Sim, mode: AttitudeMode) -> f64 {
    sim.instance(mode)
        .map(|i| i.history.iter().map(|s| s.psi).fold(0.0_f64, f64::max))
        .expect("history present")
}

fn final_psi(sim: &Sim, mode: AttitudeMode) -> f64 {
    sim.instance(mode)
        .and_then(|i| i.history.last())
        .map(|s| s.psi)
        .expect("history present")
}

fn final_pos_err(sim: &Sim, mode: AttitudeMode) -> f64 {
    sim.instance(mode)
        .and_then(|i| i.history.last())
        .map(|s| s.pos_err)
        .expect("history present")
}

#[test]
fn geometric_recovers_from_upside_down() {
    let sc = scenario::resolve("upside_down").unwrap();
    let mut sim = Sim::new(sc, &[AttitudeMode::Geometric], QuadParams::default());
    sim.run_for(8.0);

    let psi = final_psi(&sim, AttitudeMode::Geometric);
    let perr = final_pos_err(&sim, AttitudeMode::Geometric);
    assert!(psi < 1e-2, "attitude did not converge: final Ψ = {psi}");
    assert!(perr < 1e-1, "position did not converge: |e_x| = {perr}");
}

#[test]
fn quaternion_also_recovers_from_upside_down() {
    let sc = scenario::resolve("upside_down").unwrap();
    let mut sim = Sim::new(sc, &[AttitudeMode::Quaternion], QuadParams::default());
    sim.run_for(8.0);
    assert!(final_psi(&sim, AttitudeMode::Quaternion) < 1e-2);
    assert!(final_pos_err(&sim, AttitudeMode::Quaternion) < 1e-1);
}

#[test]
fn geometric_stalls_at_exact_180_but_quaternion_recovers() {
    // At exactly 180° the geometric attitude error vanishes (unstable
    // equilibrium), so the geometric controller stalls inverted; the
    // sign-corrected quaternion error still drives a decisive recovery.
    let sc = scenario::resolve("flip").unwrap();
    let mut sim = Sim::new(
        sc,
        &[AttitudeMode::Geometric, AttitudeMode::Quaternion],
        QuadParams::default(),
    );
    sim.run_for(8.0);

    let geo = final_psi(&sim, AttitudeMode::Geometric);
    let quat = final_psi(&sim, AttitudeMode::Quaternion);
    assert!(
        quat < 1e-2,
        "quaternion should recover from exact 180°, Ψ = {quat}"
    );
    assert!(
        geo > 1.0,
        "geometric should stall at the 180° unstable equilibrium, Ψ = {geo}"
    );
}

#[test]
fn naive_euler_fails_at_gimbal_lock_while_geometric_survives() {
    // With realistic standalone baselines, the naive Euler controller feeds back
    // the singular Euler rates and tumbles at gimbal lock; geometric and
    // quaternion (both singularity-free) recover. This is the headline benefit.
    let sc = scenario::resolve("gimbal").unwrap();
    let mut sim = Sim::new(sc, &AttitudeMode::ALL, QuadParams::default());
    sim.naive_baselines = true;
    sim.run_for(8.0);

    assert!(final_psi(&sim, AttitudeMode::Geometric) < 1e-2);
    assert!(final_psi(&sim, AttitudeMode::Quaternion) < 1e-2);

    let euler = sim.instance(AttitudeMode::Euler).unwrap();
    let euler_psi = euler.history.last().unwrap().psi;
    assert!(
        euler.diverged || euler_psi > 0.5,
        "naive Euler should fail at gimbal lock (Ψ = {euler_psi}, diverged = {})",
        euler.diverged
    );
}

#[test]
fn hover_stays_put() {
    let sc = scenario::resolve("hover").unwrap();
    let mut sim = Sim::new(sc, &AttitudeMode::ALL, QuadParams::default());
    sim.run_for(4.0);
    for mode in AttitudeMode::ALL {
        assert!(final_psi(&sim, mode) < 1e-3, "{mode:?} drifted in hover");
        assert!(final_pos_err(&sim, mode) < 1e-2, "{mode:?} drifted in hover");
    }
}

#[test]
fn naive_quaternion_unwinds_while_geometric_takes_short_path() {
    // A 20°-the-short-way heading error. The stateful sign-naive quaternion
    // (seeded on the antipodal sheet) unwinds ≈340° — its configuration error Ψ
    // swings all the way up through 180° (Ψ→2) before settling — whereas the
    // geometric law snaps the short way (tiny peak Ψ). This is the double cover.
    let sc = scenario::resolve("unwind").unwrap();
    let mut sim = Sim::new(
        sc,
        &[AttitudeMode::Geometric, AttitudeMode::Quaternion],
        QuadParams::default(),
    );
    sim.naive_baselines = true;
    sim.run_for(6.0);

    assert!(
        peak_psi(&sim, AttitudeMode::Geometric) < 0.2,
        "geometric should take the short path"
    );
    assert!(
        peak_psi(&sim, AttitudeMode::Quaternion) > 1.5,
        "naive quaternion should unwind through inversion (Ψ→2)"
    );
    // Both still converge.
    assert!(final_psi(&sim, AttitudeMode::Geometric) < 1e-2);
    assert!(final_psi(&sim, AttitudeMode::Quaternion) < 1e-2);
}

#[test]
fn attitude_mode_tracks_a_moving_reference() {
    // Attitude mode following a steady spin reference (Ωd ≠ 0): the error stays
    // small throughout and the airframe neither falls nor drifts far.
    let sc = scenario::resolve("attitude").unwrap();
    let mut sim = Sim::new(sc, &[AttitudeMode::Geometric], QuadParams::default());
    sim.run_for(8.0);
    assert!(final_psi(&sim, AttitudeMode::Geometric) < 1e-2);
    assert!(!sim.instance(AttitudeMode::Geometric).unwrap().diverged);
}

#[test]
fn realistic_plant_still_hovers_and_tracks() {
    // With the full real-world preset (actuator limits, lag, drag, wind, model
    // mismatch, IMU noise/latency, substepping) the controller still holds a
    // hover to within a few cm and tracks the circle without diverging.
    for (name, tol) in [("hover", 0.4), ("circle", 0.5)] {
        let sc = scenario::resolve(name).unwrap();
        let mut sim = Sim::new(sc, &[AttitudeMode::Geometric], QuadParams::default());
        sim.set_realism(Realism::realistic());
        sim.run_for(8.0);
        let inst = sim.instance(AttitudeMode::Geometric).unwrap();
        assert!(!inst.diverged, "{name}: realistic plant diverged");
        assert!(
            final_pos_err(&sim, AttitudeMode::Geometric) < tol,
            "{name}: realistic tracking error too large"
        );
    }
}

#[test]
fn idealized_plant_is_unchanged_by_realism_machinery() {
    // The default (ideal) realism must reproduce the original dynamics, so the
    // upside-down recovery converges exactly as before.
    let sc = scenario::resolve("upside_down").unwrap();
    let mut sim = Sim::new(sc, &[AttitudeMode::Geometric], QuadParams::default());
    sim.run_for(8.0);
    assert!(final_psi(&sim, AttitudeMode::Geometric) < 1e-2);
    assert!(final_pos_err(&sim, AttitudeMode::Geometric) < 1e-1);
}
