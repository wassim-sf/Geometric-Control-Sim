//! A small mouse-driven orbit camera (right/middle-drag to orbit, wheel to
//! zoom). Hand-rolled to avoid a version-matched camera dependency.

use crate::ui::EguiWantsPointer;
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;

#[derive(Component)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub radius: f32,
    pub yaw: f32,
    pub pitch: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        OrbitCamera {
            focus: Vec3::new(0.0, 0.8, 0.0),
            radius: 6.0,
            yaw: 0.7,
            pitch: 0.35,
        }
    }
}

fn transform_of(c: &OrbitCamera) -> Transform {
    let rot = Quat::from_euler(EulerRot::YXZ, c.yaw, -c.pitch, 0.0);
    let offset = rot * Vec3::new(0.0, 0.0, c.radius);
    Transform::from_translation(c.focus + offset).looking_at(c.focus, Vec3::Y)
}

pub fn spawn_camera(mut commands: Commands) {
    let cam = OrbitCamera::default();
    commands.spawn((
        Camera3d::default(),
        transform_of(&cam),
        cam,
        AmbientLight {
            brightness: 220.0,
            ..default()
        },
    ));
}

pub fn orbit_camera(
    mut motion: MessageReader<MouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    buttons: Res<ButtonInput<MouseButton>>,
    wants: Res<EguiWantsPointer>,
    mut q: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let Ok((mut cam, mut tf)) = q.single_mut() else {
        return;
    };

    let mut drag = Vec2::ZERO;
    for ev in motion.read() {
        drag += ev.delta;
    }
    let mut scroll = 0.0;
    for ev in wheel.read() {
        scroll += ev.y;
    }

    if !wants.0 {
        if buttons.pressed(MouseButton::Right) || buttons.pressed(MouseButton::Middle) {
            cam.yaw -= drag.x * 0.008;
            cam.pitch = (cam.pitch + drag.y * 0.008).clamp(-1.45, 1.45);
        }
        if scroll != 0.0 {
            cam.radius = (cam.radius * (1.0 - scroll * 0.12)).clamp(2.0, 60.0);
        }
    }

    *tf = transform_of(&cam);
}
