//! Gizmo overlays: a ground grid, the world axes, and per-controller flight
//! trails drawn from the logged history.

use crate::conv;
use crate::quad::mode_color;
use crate::sim_res::SimState;
use bevy::color::palettes::css;
use bevy::prelude::*;
use se3quad_core::AttitudeMode;

pub fn draw_overlays(mut gizmos: Gizmos, s: Res<SimState>) {
    // Ground grid in the Bevy x-z plane (y = 0).
    let half = 10i32;
    let step = 1.0;
    let grid = Color::srgba(0.35, 0.38, 0.42, 0.5);
    for i in -half..=half {
        let f = i as f32 * step;
        let e = half as f32 * step;
        gizmos.line(Vec3::new(f, 0.0, -e), Vec3::new(f, 0.0, e), grid);
        gizmos.line(Vec3::new(-e, 0.0, f), Vec3::new(e, 0.0, f), grid);
    }

    // World axes at the origin: North (red), East (green), Up (blue).
    let o = Vec3::ZERO;
    gizmos.line(o, conv::dir_to_bevy(&nalgebra::Vector3::new(1.5, 0.0, 0.0)), css::RED);
    gizmos.line(o, conv::dir_to_bevy(&nalgebra::Vector3::new(0.0, 1.5, 0.0)), css::LIME);
    gizmos.line(o, conv::dir_to_bevy(&nalgebra::Vector3::new(0.0, 0.0, -1.5)), css::DEEP_SKY_BLUE);

    // Flight trails.
    if s.trails {
        for (index, mode) in AttitudeMode::ALL.into_iter().enumerate() {
            if !s.show[index] {
                continue;
            }
            let Some(inst) = s.sim.instance(mode) else {
                continue;
            };
            let color = mode_color(index).with_alpha(0.8);
            let off = crate::quad::render_offset(index, s.spread, s.gap);
            let mut prev: Option<Vec3> = None;
            // Subsample the history so the polyline stays light.
            for sample in inst.history.iter().step_by(3) {
                if !sample.pos.iter().all(|v| v.is_finite()) {
                    break;
                }
                let p = conv::pos_to_bevy(&sample.pos) + off;
                if let Some(pp) = prev {
                    gizmos.line(pp, p, color);
                }
                prev = Some(p);
            }
        }
    }
}
