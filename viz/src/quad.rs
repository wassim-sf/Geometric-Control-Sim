//! Quadrotor meshes: one color-coded model per controller, plus the target
//! marker. Each model is a parent entity (placed from the sim each frame) with
//! child primitives authored in the body frame (b1 = +x forward, b3 = +z down).

use crate::{conv, sim_res::SimState};
use bevy::prelude::*;
use se3quad_core::AttitudeMode;

#[derive(Component)]
pub struct QuadView {
    pub mode: AttitudeMode,
    pub index: usize,
}

#[derive(Component)]
pub struct TargetMarker {
    pub index: usize,
}

/// One color per controller (geometric / quaternion / euler).
pub fn mode_color(index: usize) -> Color {
    match index {
        0 => Color::srgb(0.25, 0.85, 0.40), // geometric — green
        1 => Color::srgb(0.30, 0.60, 0.98), // quaternion — blue
        _ => Color::srgb(0.97, 0.45, 0.25), // euler — orange/red
    }
}

/// Lateral render offset (east axis) used to spread the controllers apart so
/// all three are visible even when they occupy the same physical position. This
/// affects *rendering only* — the physics runs at the true position.
pub fn render_offset(index: usize, spread: bool, gap: f32) -> Vec3 {
    if spread {
        Vec3::new((index as f32 - 1.0) * gap, 0.0, 0.0)
    } else {
        Vec3::ZERO
    }
}

pub fn spawn_quads(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (index, mode) in AttitudeMode::ALL.into_iter().enumerate() {
        let base = materials.add(StandardMaterial {
            base_color: mode_color(index),
            perceptual_roughness: 0.6,
            ..default()
        });
        // The front arm/rotor uses a bright accent so heading (b1) is visible.
        let accent = materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.93, 0.2),
            emissive: LinearRgba::rgb(0.35, 0.32, 0.0),
            ..default()
        });
        spawn_one(
            &mut commands,
            &mut meshes,
            base,
            accent,
            QuadView { mode, index },
        );
    }

    // One translucent target marker per controller (so a spread-apart view
    // shows each controller chasing its own copy of the setpoint).
    let tmat = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 1.0, 1.0, 0.8),
        emissive: LinearRgba::rgb(0.4, 0.4, 0.45),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    let tmesh = meshes.add(Sphere::new(0.12));
    for index in 0..3 {
        commands.spawn((
            Mesh3d(tmesh.clone()),
            MeshMaterial3d(tmat.clone()),
            Transform::default(),
            TargetMarker { index },
        ));
    }
}

fn spawn_one(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    base: Handle<StandardMaterial>,
    accent: Handle<StandardMaterial>,
    view: QuadView,
) {
    let body = meshes.add(Cuboid::new(0.22, 0.22, 0.06));
    let arm_x = meshes.add(Cuboid::new(0.40, 0.035, 0.035));
    let arm_y = meshes.add(Cuboid::new(0.035, 0.40, 0.035));
    let rotor = meshes.add(Cylinder::new(0.12, 0.02));
    // Bevy's cylinder axis is +y; rotate so the disk lies in the body x-y plane.
    let disk = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let zt = -0.045; // rotors sit toward body-up (-z)

    commands
        .spawn((Transform::default(), Visibility::Visible, view))
        .with_children(|p| {
            p.spawn((
                Mesh3d(body),
                MeshMaterial3d(base.clone()),
                Transform::default(),
            ));
            // Arms (front +x is the accent color).
            p.spawn((
                Mesh3d(arm_x.clone()),
                MeshMaterial3d(accent.clone()),
                Transform::from_xyz(0.20, 0.0, 0.0),
            ));
            p.spawn((
                Mesh3d(arm_x),
                MeshMaterial3d(base.clone()),
                Transform::from_xyz(-0.20, 0.0, 0.0),
            ));
            p.spawn((
                Mesh3d(arm_y.clone()),
                MeshMaterial3d(base.clone()),
                Transform::from_xyz(0.0, 0.20, 0.0),
            ));
            p.spawn((
                Mesh3d(arm_y),
                MeshMaterial3d(base.clone()),
                Transform::from_xyz(0.0, -0.20, 0.0),
            ));
            // Rotors.
            for (pos, mat) in [
                (Vec3::new(0.38, 0.0, zt), accent.clone()),
                (Vec3::new(-0.38, 0.0, zt), base.clone()),
                (Vec3::new(0.0, 0.38, zt), base.clone()),
                (Vec3::new(0.0, -0.38, zt), base.clone()),
            ] {
                p.spawn((
                    Mesh3d(rotor.clone()),
                    MeshMaterial3d(mat),
                    Transform {
                        translation: pos,
                        rotation: disk,
                        ..default()
                    },
                ));
            }
        });
}

/// Place each quad from its controller's current state, hiding it if the
/// controller is toggled off or has diverged.
pub fn update_quads(s: Res<SimState>, mut q: Query<(&QuadView, &mut Transform, &mut Visibility)>) {
    for (view, mut tf, mut vis) in &mut q {
        // Hidden only if the controller is toggled off.
        if !s.show[view.index] {
            *vis = Visibility::Hidden;
            continue;
        }
        *vis = Visibility::Visible;
        tf.scale = Vec3::splat(2.2);
        // Update the pose only while the state is finite. A diverged controller
        // (e.g. naive Euler at gimbal lock) freezes at its last tumbled pose so
        // the failure stays visible rather than vanishing.
        if let Some(inst) = s.sim.instance(view.mode) {
            if inst.state.is_finite() {
                tf.translation =
                    conv::pos_to_bevy(&inst.state.p) + render_offset(view.index, s.spread, s.gap);
                tf.rotation = conv::rot_to_bevy(&inst.state.r);
            }
        }
    }
}

pub fn update_target(
    s: Res<SimState>,
    mut q: Query<(&TargetMarker, &mut Transform, &mut Visibility)>,
) {
    let target = s.sim.current_target();
    for (marker, mut tf, mut vis) in &mut q {
        if !s.show[marker.index] {
            *vis = Visibility::Hidden;
            continue;
        }
        *vis = Visibility::Visible;
        tf.translation =
            conv::pos_to_bevy(&target.xd) + render_offset(marker.index, s.spread, s.gap);
    }
}
