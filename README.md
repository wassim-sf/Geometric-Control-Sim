Geometric Control of a Quadrotor on SE(3)
=========================================

Geometric tracking control of a quadrotor UAV on **SE(3)**, after Lee, Leok &
McClamroch [1],[2]. The attitude is represented directly by a rotation matrix
`R ∈ SO(3)` — never Euler angles — so the controller is **free of singularities
and works globally**, even when the vehicle starts completely upside-down.

This repository now contains two implementations:

* **`rust/`** — a modern, self-contained **Rust** port with an interactive 3-D
  **visualizer** (no MATLAB required). It generates demonstration cases, lets you
  pick a desired position + attitude, and runs the geometric controller against
  an **Euler-angle** and a **quaternion** controller *side-by-side* so you can
  see exactly where the geometric formulation wins.
* **`matlab/`** — the original MATLAB/Simulink reference implementation.

<p align="center">
    <img src="rust/docs/flip_comparison.png" width="80%" />
</p>

> *Exact 180° flip.* All three controllers start inverted. The **geometric**
> controller (green, left) stalls at the 180° *unstable equilibrium* where its
> error `sin(π)=0` vanishes; the sign-corrected **quaternion** controller (blue,
> centre) and the **Euler** controller (orange, right) flip upright. Each blind
> spot of each method is something you can watch happen.

<p align="center">
    <img src="rust/docs/circle_tracking.png" width="80%" />
</p>

> *Circle tracking.* With exact analytic feed-forward the geometric controller
> tracks aggressive trajectories tightly (here a banked circle), trails shown.


## The Rust implementation ##

### Build & run ###

Requires a recent Rust toolchain. From the `rust/` directory:

```bash
cd rust
cargo run -p se3quad-viz --release      # launch the interactive visualizer
```

The first build compiles Bevy and takes a few minutes; subsequent builds are
fast. You can jump straight into a scenario with an environment variable:

```bash
SE3QUAD_SCENARIO=flip   cargo run -p se3quad-viz --release   # the 180° flip
SE3QUAD_SCENARIO=gimbal cargo run -p se3quad-viz --release   # past gimbal lock
```

### Using the visualizer ###

* **Scenario** dropdown — choose a generated case (see below). A one-line
  description of each is shown beneath it.
* **Play / Pause / Reset / Step** and a **speed** slider control the clock.
* **Controllers** — tick/untick *Geometric SE(3)*, *Quaternion*, and *Euler* to
  show/hide each. They are always simulated, so toggling never restarts the run;
  each shows its live attitude error `Ψ` and position error. With **spread
  apart** on, the three are rendered side-by-side (rendering only — the physics
  runs at the true position) so you can compare them at a glance.
* **Realistic naive baselines** — see *Comparison modes* below.
* **Live (interactive)** scenario — drag **north / east / up / heading** sliders
  to command a desired position and attitude and watch the controller track it.
* **Plots** (bottom) — attitude error `Ψ`, position error `|eₓ|`, and pitch
  (which spikes / aliases for the Euler controller near ±90°).
* **Camera** — drag the right mouse button to orbit, scroll to zoom.

### Generated scenarios ###

| Scenario | What it shows |
|---|---|
| Hover | Trivial baseline. |
| **Upside-Down Recovery** | Starts at 178.2° roll and rights itself — the canonical SE(3) demo (all three recover). |
| **Exact 180° Flip** | Geometric error vanishes at the 180° unstable equilibrium and stalls; quaternion still recovers. |
| **Gimbal-Lock Flip** | Starts at 100° pitch, past the Euler ±90° singularity. With *realistic naive baselines* on, the Euler controller tumbles here. |
| Pitch Inversion | Inverted about the pitch axis (175°); recovery crosses the Euler θ=90° singularity. |
| Position Step | Step to (2, 2, −4) m. |
| Circle / Figure-Eight / Aggressive Orbit | Trajectory tracking with analytic feed-forward. |
| Live (interactive) | Drive the desired position + heading yourself. |

### Why three controllers? ###

The three differ only in how the attitude error `eR` is formed:

* **Geometric:** `eR = ½·vee(Rcᵀ·R − Rᵀ·Rc) = sin(α)·n` — coordinate-free, global.
* **Quaternion:** `eR = 2·sign(qₑ_w)·qₑ_vec` — global; the sign correction prevents "unwinding".
* **Euler:** `eR = wrap(zyx(R) − zyx(Rc))` — the textbook approach; correct near
  hover but **aliases / gimbal-locks** past ±90° pitch.

Near hover all three are identical (same small-angle linearisation); they only
diverge at large attitudes.

### Comparison modes ###

A common misconception is that the geometric controller is simply "best". It
isn't the *fastest* — its benefit is **global, singularity-free** stability. The
**Realistic naive baselines** toggle makes this concrete:

* **Off — fair isolation (default).** All three share the geometric outer loop,
  gyroscopic compensation, and SO(3) feed-forward; *only* `eR` differs. This
  cleanly isolates the error function. A consequence worth understanding: the
  geometric error `sin(α)` *shrinks* toward 180°, so from a near-inverted start
  the geometric controller pushes *gently* and recovers a bit **slower** than the
  quaternion/Euler errors (which grow toward 180°). That is expected behaviour of
  the Lee error function — not a bug — and it has a genuine blind spot at *exactly*
  180° (see the flip image above).
* **On — realistic standalone baselines.** The **Euler** controller becomes a true
  Euler-frame PD that feeds back the singular Euler rates (`1/cos θ`). It now
  **tumbles and diverges at gimbal lock**, while the geometric and quaternion
  controllers — both singularity-free — recover. This is the real, practical
  benefit of avoiding an Euler-angle representation.

<p align="center">
    <img src="rust/docs/naive_gimbal_failure.png" width="80%" />
</p>

> *Gimbal-Lock Flip, realistic naive baselines.* The Euler controller (orange)
> diverges — watch its pitch trace blow up in the right-hand plot — while
> geometric (green) and quaternion (blue) recover smoothly.

See `rust/core/src/controllers/` and the unit tests in that crate (`cargo test`),
which prove these properties numerically (e.g. the geometric error vanishing at
180°, and the naive Euler controller failing at gimbal lock).

### Headless runner ###

Run a case without the GUI and dump a CSV of the time histories:

```bash
cargo run -p se3quad-core --example run_case -- \
    --scenario upside_down --controllers geometric,quaternion,euler \
    --secs 8 --out /tmp/out.csv

# add --naive to use the realistic standalone baselines (Euler fails at gimbal lock):
cargo run -p se3quad-core --example run_case -- \
    --scenario gimbal --controllers geometric,quaternion,euler --naive
```

### Layout ###

```
rust/
  core/   se3quad-core  — dynamics, the three controllers, sim runner, scenarios (GUI-free, tested)
  viz/    se3quad-viz   — the Bevy visualizer
matlab/   original MATLAB/Simulink reference
```

The simulation is integrated with fixed-step RK4 in the paper's NED-like frame
(re-orthonormalising `R` each step); the visualizer converts to Bevy's y-up
frame for rendering only. The control law, parameters, and gains are ported
verbatim from `matlab/controller.m` / `matlab/param.m`; the desired-trajectory
feed-forward is supplied analytically (the MATLAB recovered it with the
dirty-derivative filter, which is retained for the measured velocity).


## Bibliography ##

[1] T. Lee, M. Leok, and N. H. Mcclamroch, *Geometric Tracking Control of a Quadrotor UAV on SE(3)*, in Conference on Decision and Control, 2010, pp. 5420–5425.

[2] T. Lee, M. Leok, and N. H. McClamroch, *Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)*, [arXiv:1003.2005v4](https://arxiv.org/abs/1003.2005), 2010.
