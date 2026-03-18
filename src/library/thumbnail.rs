// CPU raymarcher for 128x128 library component thumbnails.
// Returns raw RGBA bytes (128 * 128 * 4).

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;

pub const THUMB_W: usize = 128;
pub const THUMB_H: usize = 128;

pub fn render_thumbnail(sdf: Arc<dyn Sdf>) -> Vec<u8> {
    // Isometric 45 degree viewpoint: looking from (1, 1, -1) direction.
    let view_dir = Vec3::new(1.0, 1.0, -1.0).normalize();
    let world_up = Vec3::Z;
    let right    = world_up.cross(view_dir).normalize();
    let cam_up   = view_dir.cross(right).normalize();

    let scale       = estimate_radius(sdf.as_ref()).max(0.5);
    let cam_origin  = -view_dir * scale * 3.0;
    let light_dir   = Vec3::new(0.6, 0.8, 1.0).normalize();
    let bg_top      = Vec3::new(0.12, 0.15, 0.22);
    let bg_bot      = Vec3::new(0.06, 0.07, 0.10);

    let mut pixels = vec![0u8; THUMB_W * THUMB_H * 4];

    for py in 0..THUMB_H {
        for px in 0..THUMB_W {
            // Pixel in [-1, 1]
            let u = (px as f32 / THUMB_W as f32 - 0.5) * 2.0 * scale;
            let v = ((THUMB_H - 1 - py) as f32 / THUMB_H as f32 - 0.5) * 2.0 * scale;

            let ray_origin = cam_origin + right * u + cam_up * v;
            let ray_dir    = view_dir;

            let color = trace(sdf.as_ref(), ray_origin, ray_dir, light_dir, bg_top, bg_bot);

            let idx = (py * THUMB_W + px) * 4;
            pixels[idx    ] = (color.x.clamp(0.0, 1.0) * 255.0) as u8;
            pixels[idx + 1] = (color.y.clamp(0.0, 1.0) * 255.0) as u8;
            pixels[idx + 2] = (color.z.clamp(0.0, 1.0) * 255.0) as u8;
            pixels[idx + 3] = 255;
        }
    }
    pixels
}

fn estimate_radius(sdf: &dyn Sdf) -> f32 {
    let probes = [Vec3::X, Vec3::Y, Vec3::Z, Vec3::NEG_X, Vec3::NEG_Y, Vec3::NEG_Z,
                  Vec3::new(1.0, 1.0, 0.0).normalize(), Vec3::new(0.0, 1.0, 1.0).normalize()];
    let mut r = 1.0_f32;
    for &scale in &[1.0_f32, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0] {
        for &p in &probes {
            if sdf.distance(p * scale) < 0.0 {
                r = r.max(scale * 1.5);
            }
        }
    }
    r.min(200.0)
}

fn trace(sdf: &dyn Sdf, origin: Vec3, dir: Vec3, light: Vec3, bg_top: Vec3, bg_bot: Vec3) -> Vec3 {
    const MAX_STEPS: usize = 80;
    const MAX_T:     f32   = 500.0;
    const SURF_EPS:  f32   = 0.001;

    let mut t = 0.0_f32;
    for _ in 0..MAX_STEPS {
        let p = origin + dir * t;
        let d = sdf.distance(p);
        if d < SURF_EPS {
            let n = normal(sdf, p);
            let diffuse  = n.dot(light).max(0.0);
            let ambient  = 0.15_f32;
            let specular = n.dot((light - dir).normalize()).max(0.0).powi(32) * 0.3;
            let base = Vec3::new(0.55, 0.72, 0.92);
            return base * (ambient + diffuse) + Vec3::splat(specular);
        }
        t += d.abs().max(SURF_EPS * 0.5);
        if t > MAX_T { break; }
    }
    // Background gradient top to bottom
    let blend = ((dir.z + 1.0) * 0.5).clamp(0.0, 1.0);
    bg_top * blend + bg_bot * (1.0 - blend)
}

fn normal(sdf: &dyn Sdf, p: Vec3) -> Vec3 {
    let e = 0.002;
    Vec3::new(
        sdf.distance(p + Vec3::new(e, 0.0, 0.0)) - sdf.distance(p - Vec3::new(e, 0.0, 0.0)),
        sdf.distance(p + Vec3::new(0.0, e, 0.0)) - sdf.distance(p - Vec3::new(0.0, e, 0.0)),
        sdf.distance(p + Vec3::new(0.0, 0.0, e)) - sdf.distance(p - Vec3::new(0.0, 0.0, e)),
    ).normalize_or_zero()
}
