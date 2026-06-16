//! egui control panel + hand-rolled time-series plots.

use crate::sim_res::SimState;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use bevy_egui::egui::Color32;
use se3quad_core::AttitudeMode;

/// Set true when egui wants the pointer, so the orbit camera ignores that input.
#[derive(Resource, Default)]
pub struct EguiWantsPointer(pub bool);

fn mode_color32(index: usize) -> Color32 {
    match index {
        0 => Color32::from_rgb(64, 217, 102),
        1 => Color32::from_rgb(77, 153, 250),
        _ => Color32::from_rgb(247, 115, 64),
    }
}

pub fn ui_system(
    mut contexts: EguiContexts,
    mut s: ResMut<SimState>,
    mut wants: ResMut<EguiWantsPointer>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::SidePanel::left("controls")
        .default_width(280.0)
        .show(ctx, |ui| {
            ui.add_space(4.0);
            ui.heading("Geometric Control on SE(3)");
            ui.label(
                egui::RichText::new("Quadrotor attitude + position tracking")
                    .small()
                    .weak(),
            );
            ui.separator();

            // Scenario picker.
            let current = s.scenarios[s.selected].name.clone();
            egui::ComboBox::from_label("Scenario")
                .selected_text(current)
                .show_ui(ui, |ui| {
                    for i in 0..s.scenarios.len() {
                        let name = s.scenarios[i].name.clone();
                        if ui.selectable_label(s.selected == i, name).clicked() {
                            s.select_scenario(i);
                        }
                    }
                });
            ui.label(egui::RichText::new(s.scenarios[s.selected].blurb.clone()).small());
            ui.label(
                egui::RichText::new(format!("Flight mode: {}", s.sim.control_mode.label()))
                    .small()
                    .strong(),
            );
            ui.separator();

            // Transport.
            ui.horizontal(|ui| {
                let label = if s.playing { "⏸ Pause" } else { "▶ Play" };
                if ui.button(label).clicked() {
                    s.playing = !s.playing;
                }
                if ui.button("⟲ Reset").clicked() {
                    s.reset();
                }
                if ui.button("Step").clicked() {
                    s.sim.step();
                }
            });
            ui.add(egui::Slider::new(&mut s.speed, 0.05..=2.0).text("speed ×"));
            ui.add_space(4.0);
            ui.label(format!("t = {:.2} s", s.sim.t));
            ui.separator();

            // Controller comparison toggles + live readout.
            ui.label(egui::RichText::new("Controllers (compare side-by-side)").strong());
            for (index, mode) in AttitudeMode::ALL.into_iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut s.show[index], "");
                    let (mark, color) = ("⬤", mode_color32(index));
                    ui.colored_label(color, mark);
                    ui.label(mode.label());
                    if let Some(inst) = s.sim.instance(mode) {
                        if let Some(last) = inst.history.last() {
                            let txt = if inst.diverged {
                                "  diverged".to_string()
                            } else {
                                format!("  Ψ={:.3}  |e|={:.2}m", last.psi, last.pos_err)
                            };
                            ui.label(egui::RichText::new(txt).small().weak());
                        }
                    }
                });
            }
            ui.add_space(2.0);
            ui.checkbox(&mut s.naive, "realistic naive baselines");
            let hint = if s.naive {
                "Naive baselines on: Euler is a true Euler-frame PD (tumbles at gimbal lock — \
                 try “Gimbal-Lock Flip”); the quaternion drops its sign fix and UNWINDS (try \
                 “Quaternion Unwinding”)."
            } else {
                "Fair isolation: all three share the geometric law; only the \
                 attitude error differs."
            };
            ui.label(egui::RichText::new(hint).small().weak());
            ui.add_space(4.0);

            // Real-world plant effects.
            real_world_section(ui, &mut s);
            ui.add_space(4.0);

            // Custom-trajectory editor (only meaningful for the Custom scenario).
            if s.is_custom() {
                custom_traj_section(ui, &mut s);
                ui.add_space(4.0);
            }

            ui.horizontal(|ui| {
                ui.checkbox(&mut s.trails, "trails");
                ui.checkbox(&mut s.spread, "spread apart");
            });
            if s.spread {
                ui.add(egui::Slider::new(&mut s.gap, 0.0..=5.0).text("spread gap (m)"));
            }
            ui.separator();

            // Live target sliders.
            if s.is_live() {
                ui.label(egui::RichText::new("Target setpoint").strong());
                ui.add(egui::Slider::new(&mut s.live_n, -5.0..=5.0).text("north (m)"));
                ui.add(egui::Slider::new(&mut s.live_e, -5.0..=5.0).text("east (m)"));
                ui.add(egui::Slider::new(&mut s.live_up, 0.0..=6.0).text("up (m)"));
                ui.add(
                    egui::Slider::new(&mut s.live_yaw, -std::f32::consts::PI..=std::f32::consts::PI)
                        .text("heading (rad)"),
                );
            } else {
                ui.label(
                    egui::RichText::new("Tip: choose “Live (interactive)” to drive the target.")
                        .small()
                        .weak(),
                );
            }

            ui.add_space(8.0);
            ui.label(
                egui::RichText::new("Drag right mouse to orbit · wheel to zoom")
                    .small()
                    .weak(),
            );
        });

    // Bottom panel: plots.
    egui::TopBottomPanel::bottom("plots")
        .default_height(200.0)
        .resizable(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                let w = (ui.available_width() - 24.0) / 3.0;
                plot_panel(ui, &s, w, "Attitude error  Ψ", 2.0, |sm| sm.psi);
                plot_panel(ui, &s, w, "Position error  |eₓ| (m)", 1.0, |sm| sm.pos_err);
                plot_panel(ui, &s, w, "Pitch (deg)  — Euler aliasing", 180.0, |sm| {
                    sm.pitch.to_degrees()
                });
            });
        });

    wants.0 = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
}

/// Real-world plant effects: a master toggle plus editable parameters that take
/// effect (rebuilding the airframes) when "Apply & restart" is pressed.
fn real_world_section(ui: &mut egui::Ui, s: &mut SimState) {
    let mut on = s.realism_on;
    if ui
        .checkbox(&mut on, "real-world plant (actuator + drag + IMU + wind)")
        .changed()
    {
        s.realism_on = on;
        s.apply_realism();
    }
    if !s.realism_on {
        ui.label(
            egui::RichText::new("Off: ideal SE(3) plant with perfect sensors.")
                .small()
                .weak(),
        );
        return;
    }
    egui::CollapsingHeader::new("real-world parameters")
        .default_open(false)
        .show(ui, |ui| {
            let r = &mut s.realism;
            ui.label(egui::RichText::new("Actuator").small().strong());
            ui.add(egui::Slider::new(&mut r.rotor_max, 12.0..=40.0).text("rotor max (N)"));
            ui.add(egui::Slider::new(&mut r.motor_tau, 0.0..=0.1).text("motor lag τ (s)"));
            ui.label(egui::RichText::new("Aerodynamics / wind").small().strong());
            ui.add(egui::Slider::new(&mut r.drag_lin, 0.0..=1.0).text("lin drag"));
            ui.add(egui::Slider::new(&mut r.drag_rot, 0.0..=0.2).text("rot drag"));
            ui.add(egui::Slider::new(&mut r.wind.x, -3.0..=3.0).text("wind N (N)"));
            ui.add(egui::Slider::new(&mut r.gust_amp, 0.0..=3.0).text("gust amp (N)"));
            ui.label(egui::RichText::new("Model mismatch").small().strong());
            ui.add(egui::Slider::new(&mut r.mass_scale, 0.8..=1.2).text("mass ×"));
            ui.add(egui::Slider::new(&mut r.inertia_scale, 0.8..=1.2).text("inertia ×"));
            ui.label(egui::RichText::new("Sensors (IMU)").small().strong());
            ui.add(egui::Slider::new(&mut r.gyro_noise, 0.0..=0.05).text("gyro noise"));
            ui.add(egui::Slider::new(&mut r.att_noise, 0.0..=0.02).text("attitude noise"));
            ui.add(
                egui::Slider::new(&mut r.sensor_delay, 0..=5).text("sensor delay (steps)"),
            );
            ui.add(egui::Slider::new(&mut r.substeps, 1..=10).text("physics substeps"));
            if ui.button("Apply & restart").clicked() {
                s.apply_realism();
            }
        });
}

/// Editor for the user-definable custom Lissajous trajectory.
fn custom_traj_section(ui: &mut egui::Ui, s: &mut SimState) {
    egui::CollapsingHeader::new("custom trajectory")
        .default_open(true)
        .show(ui, |ui| {
            let c = &mut s.custom;
            ui.label(egui::RichText::new("x = ax·sin(wx·t)").small().weak());
            ui.add(egui::Slider::new(&mut c.ax, 0.0..=5.0).text("ax (m)"));
            ui.add(egui::Slider::new(&mut c.wx, 0.0..=3.0).text("wx (rad/s)"));
            ui.label(egui::RichText::new("y = ay·sin(wy·t + py)").small().weak());
            ui.add(egui::Slider::new(&mut c.ay, 0.0..=5.0).text("ay (m)"));
            ui.add(egui::Slider::new(&mut c.wy, 0.0..=3.0).text("wy (rad/s)"));
            ui.add(egui::Slider::new(&mut c.py, -3.14..=3.14).text("py (rad)"));
            ui.label(egui::RichText::new("z = up + az·sin(wz·t + pz)").small().weak());
            ui.add(egui::Slider::new(&mut c.az, 0.0..=3.0).text("az (m)"));
            ui.add(egui::Slider::new(&mut c.wz, 0.0..=3.0).text("wz (rad/s)"));
            ui.add(egui::Slider::new(&mut c.height_up, 0.0..=6.0).text("up (m)"));
            ui.label(egui::RichText::new("Edits reshape the path live.").small().weak());
        });
}

/// Collect the visible controllers' series for a given scalar, then draw.
fn plot_panel(
    ui: &mut egui::Ui,
    s: &SimState,
    width: f32,
    title: &str,
    y_hint: f64,
    sel: impl Fn(&se3quad_core::Sample) -> f64,
) {
    let mut series: Vec<(Color32, Vec<[f64; 2]>)> = Vec::new();
    let mut y_max = y_hint;
    for (index, mode) in AttitudeMode::ALL.into_iter().enumerate() {
        if !s.show[index] {
            continue;
        }
        if let Some(inst) = s.sim.instance(mode) {
            let pts: Vec<[f64; 2]> = inst
                .history
                .iter()
                .step_by(2)
                .map(|sm| {
                    let v = sel(sm);
                    y_max = y_max.max(v.abs());
                    [sm.t, v]
                })
                .collect();
            series.push((mode_color32(index), pts));
        }
    }
    draw_plot(ui, width, 150.0, title, -0.05 * y_max, y_max * 1.05, &series);
}

fn draw_plot(
    ui: &mut egui::Ui,
    width: f32,
    height: f32,
    title: &str,
    y_min: f64,
    y_max: f64,
    series: &[(Color32, Vec<[f64; 2]>)],
) {
    ui.vertical(|ui| {
        ui.label(egui::RichText::new(title).small().strong());
        let (resp, painter) =
            ui.allocate_painter(egui::vec2(width, height), egui::Sense::hover());
        let rect = resp.rect;
        painter.rect_filled(rect, 2.0, Color32::from_gray(18));

        let (mut x_min, mut x_max) = (f64::MAX, f64::MIN);
        for (_, pts) in series {
            for p in pts {
                x_min = x_min.min(p[0]);
                x_max = x_max.max(p[0]);
            }
        }
        if !(x_max > x_min) {
            x_max = x_min + 1.0;
        }
        let span_y = (y_max - y_min).max(1e-6);
        let map = |x: f64, y: f64| -> egui::Pos2 {
            let fx = ((x - x_min) / (x_max - x_min)) as f32;
            let fy = ((y - y_min) / span_y).clamp(0.0, 1.0) as f32;
            egui::pos2(rect.left() + fx * rect.width(), rect.bottom() - fy * rect.height())
        };

        // Zero line.
        if y_min < 0.0 && y_max > 0.0 {
            let y0 = map(x_min, 0.0).y;
            painter.line_segment(
                [egui::pos2(rect.left(), y0), egui::pos2(rect.right(), y0)],
                egui::Stroke::new(1.0, Color32::from_gray(60)),
            );
        }
        for (color, pts) in series {
            let mut last: Option<egui::Pos2> = None;
            for p in pts {
                if !p[1].is_finite() {
                    last = None;
                    continue;
                }
                let sp = map(p[0], p[1]);
                if let Some(lp) = last {
                    painter.line_segment([lp, sp], egui::Stroke::new(1.5, *color));
                }
                last = Some(sp);
            }
        }
    });
}
