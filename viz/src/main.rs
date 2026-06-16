//! Interactive Bevy visualizer for the SE(3) geometric quadrotor controller.
//!
//! Renders the geometric / quaternion / Euler controllers side-by-side as they
//! track a chosen scenario (or a live target), with time-series plots of the
//! attitude error Ψ, position error, and pitch.

mod camera;
mod conv;
mod overlay;
mod quad;
mod sim_res;
mod ui;

use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use sim_res::SimState;
use ui::EguiWantsPointer;

fn main() -> AppExit {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "se3quad — Geometric Quadrotor Control on SE(3)".into(),
            ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin::default())
    .init_resource::<SimState>()
    .init_resource::<EguiWantsPointer>()
    .insert_resource(ClearColor(Color::srgb(0.05, 0.06, 0.08)))
    .add_systems(
        Startup,
        (setup_scene, quad::spawn_quads, camera::spawn_camera),
    )
    .add_systems(
        Update,
        (
            sim_res::step_sim,
            quad::update_quads,
            quad::update_target,
            overlay::draw_overlays,
            camera::orbit_camera,
        ),
    )
    .add_systems(EguiPrimaryContextPass, ui::ui_system);

    // Optional: SE3QUAD_SHOT=<path> grabs a screenshot after a moment and exits.
    // Used to validate the render headlessly; no effect on normal runs.
    if let Ok(path) = std::env::var("SE3QUAD_SHOT") {
        let at = std::env::var("SE3QUAD_SHOT_FRAME")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(150);
        app.insert_resource(ShotRequest { path, frame: 0, at })
            .add_systems(Update, screenshot_then_exit);
    }

    app.run()
}

#[derive(Resource)]
struct ShotRequest {
    path: String,
    frame: u32,
    at: u32,
}

fn screenshot_then_exit(
    mut commands: Commands,
    mut shot: ResMut<ShotRequest>,
    mut exit: MessageWriter<AppExit>,
) {
    shot.frame += 1;
    if shot.frame == shot.at {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(shot.path.clone()));
    }
    if shot.frame == shot.at + 15 {
        exit.write(AppExit::Success);
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 9000.0,
            ..default()
        },
        Transform::from_xyz(6.0, 12.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(40.0, 40.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.10, 0.11, 0.14),
            perceptual_roughness: 0.95,
            ..default()
        })),
        Transform::from_xyz(0.0, -0.01, 0.0),
    ));
}
