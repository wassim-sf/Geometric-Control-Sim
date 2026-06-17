//! Headless scenario runner — integrates one or more controllers through a
//! scenario and writes a CSV of the time histories (or a summary to stdout).
//!
//! ```text
//! cargo run -p se3quad-core --example run_case -- \
//!     --scenario upside_down --controllers geometric,quaternion,euler \
//!     --secs 8 --out /tmp/out.csv
//! ```

use se3quad_core::{scenario, AttitudeMode, QuadParams, Realism, Safety, Sim};
use std::fmt::Write as _;
use std::fs;

struct Args {
    scenario: String,
    modes: Vec<AttitudeMode>,
    secs: f64,
    out: Option<String>,
    naive: bool,
    standalone: bool,
    realistic: bool,
    real_motors: bool,
    safety: bool,
    env: Option<String>,
    ki: Option<f64>,
}

fn parse_args() -> Args {
    let mut args = Args {
        scenario: "upside_down".to_string(),
        modes: vec![AttitudeMode::Geometric],
        secs: 8.0,
        out: None,
        naive: false,
        standalone: false,
        realistic: false,
        real_motors: false,
        safety: false,
        env: None,
        ki: None,
    };
    let raw: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < raw.len() {
        match raw[i].as_str() {
            "--scenario" | "-s" => {
                args.scenario = raw[i + 1].clone();
                i += 2;
            }
            "--controllers" | "-c" => {
                args.modes = raw[i + 1]
                    .split(',')
                    .filter_map(AttitudeMode::from_token)
                    .collect();
                i += 2;
            }
            "--secs" | "-t" => {
                args.secs = raw[i + 1].parse().expect("--secs must be a number");
                i += 2;
            }
            "--out" | "-o" => {
                args.out = Some(raw[i + 1].clone());
                i += 2;
            }
            "--naive" => {
                args.naive = true;
                i += 1;
            }
            "--standalone" => {
                args.standalone = true;
                i += 1;
            }
            "--realistic" => {
                args.realistic = true;
                i += 1;
            }
            "--real-motors" => {
                args.real_motors = true;
                i += 1;
            }
            "--safety" => {
                args.safety = true;
                i += 1;
            }
            "--env" => {
                args.env = Some(raw[i + 1].clone());
                i += 2;
            }
            "--ki" => {
                args.ki = Some(raw[i + 1].parse().expect("--ki must be a number"));
                i += 2;
            }
            other => {
                eprintln!("warning: ignoring unknown argument `{other}`");
                i += 1;
            }
        }
    }
    if args.modes.is_empty() {
        args.modes = vec![AttitudeMode::Geometric];
    }
    args
}

fn main() {
    let args = parse_args();
    let scenario = scenario::resolve(&args.scenario)
        .unwrap_or_else(|| panic!("unknown scenario `{}`", args.scenario));
    let name = scenario.name.clone();

    let mut params = QuadParams::default();
    if let Some(ki) = args.ki {
        params.ki = ki;
    }
    let mut sim = Sim::new(scenario, &args.modes, params);
    sim.naive_baselines = args.naive;
    sim.standalone_baselines = args.standalone;
    if let Some(env) = args.env.as_deref() {
        let r = match env {
            "low" | "calm" => Realism::low(),
            "medium" | "moderate" => Realism::medium(),
            "harsh" | "high" => Realism::harsh(),
            _ => panic!("--env must be low|medium|harsh"),
        };
        sim.set_realism(r);
    } else if args.real_motors {
        sim.set_realism(Realism::real_motors());
    } else if args.realistic {
        sim.set_realism(Realism::realistic());
    }
    if args.safety {
        sim.set_safety(Safety::standard());
    }
    sim.run_for(args.secs);

    // Console summary.
    println!(
        "scenario: {name}   ({:.1}s @ {} Hz)",
        args.secs,
        (1.0 / sim.params.ts) as i64
    );
    for inst in &sim.instances {
        let last = inst.history.last();
        let (psi, perr) = last
            .map(|s| (s.psi, s.pos_err))
            .unwrap_or((f64::NAN, f64::NAN));
        let status = if inst.diverged { "DIVERGED" } else { "ok" };
        println!(
            "  {:<16} final Ψ = {:>8.4}   |e_x| = {:>8.4} m   [{status}]",
            inst.mode.label(),
            psi,
            perr
        );
    }

    if let Some(path) = args.out {
        let n = sim
            .instances
            .iter()
            .map(|i| i.history.len())
            .min()
            .unwrap_or(0);
        let mut csv = String::new();
        csv.push('t');
        for inst in &sim.instances {
            let m = inst.mode.label().replace(' ', "_");
            let _ = write!(csv, ",psi_{m},poserr_{m},f_{m},pitch_{m}");
        }
        csv.push('\n');
        for k in 0..n {
            let _ = write!(csv, "{:.4}", sim.instances[0].history[k].t);
            for inst in &sim.instances {
                let s = inst.history[k];
                let _ = write!(
                    csv,
                    ",{:.6},{:.6},{:.6},{:.6}",
                    s.psi, s.pos_err, s.f, s.pitch
                );
            }
            csv.push('\n');
        }
        fs::write(&path, csv).expect("write CSV");
        println!("wrote {n} rows to {path}");
    }
}
