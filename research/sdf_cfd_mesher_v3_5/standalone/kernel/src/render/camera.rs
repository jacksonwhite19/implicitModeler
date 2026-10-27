// Camera and controls

use glam::{Mat4, Vec3};

/// Opt-in, bounded input diagnostics; no per-frame output unless explicitly enabled.
pub fn trace_drag(
    before: &Camera,
    after: &Camera,
    delta: [f32; 2],
    viewport: [f32; 2],
    ppp: f32,
    pan: bool,
) {
    use std::sync::{
        OnceLock,
        atomic::{AtomicUsize, Ordering},
    };
    static ENABLED: OnceLock<bool> = OnceLock::new();
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    if !*ENABLED.get_or_init(|| std::env::var_os("IMPLICIT_CAMERA_DIAGNOSTICS").is_some())
        || COUNT.fetch_add(1, Ordering::Relaxed) >= 256
    {
        return;
    }
    let angles = |c: &Camera| {
        let d = (c.eye - c.target).normalize();
        [d.y.atan2(d.x), d.z.clamp(-1., 1.).acos()]
    };
    let a = angles(before);
    let b = angles(after);
    eprintln!(
        "CAMERA_DRAG {}",
        serde_json::json!({"pan":pan,"delta_points":delta,"viewport_points":viewport,"pixels_per_point":ppp,"distance":before.distance_to_target(),"orbit_radians_per_point":Camera::ORBIT_RADIANS_PER_POINT,"pan_world_per_point":2.0*(before.eye-before.target).length()*(before.fov*0.5).tan()/viewport[1],"angle_change_radians":[b[0]-a[0],b[1]-a[1]],"translation":(after.target-before.target).to_array()})
    );
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StandardView {
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom,
    Isometric,
}

#[derive(Clone)]
pub struct Camera {
    pub eye: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    pub fov: f32,
    pub aspect: f32,
    pub near: f32,
    pub far: f32,
}

impl Camera {
    pub const ORBIT_RADIANS_PER_POINT: f32 = 0.01;
    pub fn new(aspect: f32) -> Self {
        Self {
            eye: Vec3::new(0.0, -50.0, 30.0),
            target: Vec3::ZERO,
            up: Vec3::Z,
            fov: 30.0_f32.to_radians(),
            aspect,
            near: 0.1,
            far: 1000.0,
        }
    }

    pub fn view_matrix(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye, self.target, self.up)
    }

    pub fn projection_matrix(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov, self.aspect, self.near, self.far)
    }

    pub fn view_projection(&self) -> Mat4 {
        self.projection_matrix() * self.view_matrix()
    }

    pub fn distance_to_target(&self) -> f32 {
        (self.eye - self.target).length().max(1.0)
    }

    pub fn snap_to_view(&mut self, view: StandardView) {
        let distance = self.distance_to_target();
        match view {
            StandardView::Front => self.set_forward_view(Vec3::X, Vec3::Z, distance),
            StandardView::Back => self.set_forward_view(-Vec3::X, Vec3::Z, distance),
            StandardView::Left => self.set_forward_view(Vec3::Y, Vec3::Z, distance),
            StandardView::Right => self.set_forward_view(-Vec3::Y, Vec3::Z, distance),
            StandardView::Top => self.set_forward_view(-Vec3::Z, Vec3::Y, distance),
            StandardView::Bottom => self.set_forward_view(Vec3::Z, -Vec3::Y, distance),
            StandardView::Isometric => {
                self.set_forward_view(Vec3::new(1.0, 1.0, -0.8), Vec3::Z, distance)
            }
        }
    }

    fn set_forward_view(&mut self, forward: Vec3, up: Vec3, distance: f32) {
        let forward = forward.normalize_or_zero();
        let up = up.normalize_or_zero();
        if forward == Vec3::ZERO || up == Vec3::ZERO {
            return;
        }
        self.eye = self.target - forward * distance;
        self.up = up;
    }

    // Orbit around target
    pub fn orbit(&mut self, delta_x: f32, delta_y: f32) {
        let radius = (self.eye - self.target).length();
        let direction = (self.eye - self.target).normalize();

        // Convert to spherical coordinates
        let mut theta = direction.y.atan2(direction.x); // Azimuth
        let mut phi = direction.z.clamp(-1.0, 1.0).acos(); // Polar angle (direction is normalized)

        // Apply deltas (with sensitivity scaling)
        theta -= delta_x * Self::ORBIT_RADIANS_PER_POINT;
        phi = (phi - delta_y * Self::ORBIT_RADIANS_PER_POINT)
            .clamp(0.01, std::f32::consts::PI - 0.01);

        // Convert back to cartesian
        let new_direction = Vec3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos());

        self.eye = self.target + new_direction * radius;
    }

    /// Scene drag in logical screen points, positive Y up. Delta and height use
    /// the SAME units; egui already handles DPI. No frame-time multiplier.
    pub fn pan(&mut self, delta_x: f32, delta_y: f32, viewport_height: f32) {
        if !viewport_height.is_finite() || viewport_height <= 0.0 {
            return;
        }
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward).normalize();

        let world_per_point =
            2.0 * (self.eye - self.target).length() * (self.fov * 0.5).tan() / viewport_height;
        // Move the camera opposite to the requested scene displacement.
        let offset = -(right * delta_x + up * delta_y) * world_per_point;

        self.eye += offset;
        self.target += offset;
    }

    // Zoom (move eye closer/further from target)
    pub fn zoom(&mut self, delta: f32) {
        let direction = (self.eye - self.target).normalize();
        let distance = (self.eye - self.target).length();
        let new_distance = (distance - delta * 2.0).max(1.0);

        self.eye = self.target + direction * new_distance;
    }

    // Frame a bounding box
    pub fn frame_bounds(&mut self, bounds_min: Vec3, bounds_max: Vec3) {
        self.target = (bounds_min + bounds_max) * 0.5;
        let bounds_size = (bounds_max - bounds_min).length();
        let distance = bounds_size * 1.5;

        let direction = Vec3::new(0.0, -1.0, 0.6).normalize();
        self.eye = self.target + direction * distance;
    }

    // Reset to default view
    pub fn reset(&mut self) {
        self.eye = Vec3::new(0.0, -50.0, 30.0);
        self.target = Vec3::ZERO;
        self.up = Vec3::Z;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scene_pan_tracks_screen_at_target_depth() {
        for view in [
            StandardView::Front,
            StandardView::Right,
            StandardView::Top,
            StandardView::Isometric,
        ] {
            for distance in [2.0, 20.0, 200.0, 2000.0] {
                for height in [300.0, 600.0, 1200.0] {
                    let mut c = Camera::new(4.0 / 3.0);
                    c.eye = c.target + Vec3::Y * distance;
                    c.snap_to_view(view);
                    let world = c.target;
                    c.pan(70.0, 40.0, height);
                    let screen = c.view_projection().project_point3(world);
                    // Existing camera test rule: 1e-4 absolute, here in dimensionless NDC.
                    assert!((screen.x - 140.0 / (height * c.aspect)).abs() < 1e-4);
                    assert!((screen.y - 80.0 / height).abs() < 1e-4);
                }
            }
        }
    }

    #[test]
    fn pan_all_directions_and_degenerate_height() {
        for (dx, dy) in [(70.0,0.0),(-70.0,0.0),(0.0,40.0),(0.0,-40.0)] {
            let mut c=Camera::new(4.0/3.0);
            let point=c.target;
            c.pan(dx,dy,600.0);
            let p=c.view_projection().project_point3(point);
            assert!((p.x-dx/400.0).abs()<1e-4);
            assert!((p.y-dy/300.0).abs()<1e-4);
        }
        let mut c=Camera::new(1.0);let original=c.clone();
        c.pan(10.0,10.0,0.0);
        assert_eq!(c.eye,original.eye);assert_eq!(c.target,original.target);
    }

    #[test]
    fn pan_physical_pixel_mapping_is_dpi_invariant() {
        let mut reference = Camera::new(4.0 / 3.0);
        reference.pan(120.0, 60.0, 960.0);
        for ppp in [1.0, 1.25, 1.5, 2.0] {
            let mut c = Camera::new(4.0 / 3.0);
            c.pan(120.0 / ppp, 60.0 / ppp, 960.0 / ppp);
            assert!((c.eye - reference.eye).length() < 1e-4);
            assert!((c.target - reference.target).length() < 1e-4);
        }
    }

    #[test]
    fn orbit_keeps_existing_one_point_conversion() {
        let mut c = Camera::new(1.0);
        let d = (c.eye - c.target).normalize();
        let yaw = d.y.atan2(d.x);
        let pitch = d.z.acos();
        c.orbit(20.0, 10.0);
        let after = (c.eye - c.target).normalize();
        assert!((after.y.atan2(after.x) - (yaw - 0.2)).abs() < 1e-4);
        assert!((after.z.acos() - (pitch - 0.1)).abs() < 1e-4);
    }

    #[test]
    fn test_camera_matrices() {
        let camera = Camera::new(16.0 / 9.0);

        // Just verify matrices can be computed without panicking
        let view = camera.view_matrix();
        let proj = camera.projection_matrix();
        let view_proj = camera.view_projection();

        assert!(view.is_finite());
        assert!(proj.is_finite());
        assert!(view_proj.is_finite());
    }

    #[test]
    fn test_standard_views_preserve_distance() {
        let mut camera = Camera::new(16.0 / 9.0);
        let distance = camera.distance_to_target();

        camera.snap_to_view(StandardView::Top);
        assert!((camera.distance_to_target() - distance).abs() < 1e-4);
        assert!((camera.eye.z - distance).abs() < 1e-4);

        camera.snap_to_view(StandardView::Front);
        assert!((camera.distance_to_target() - distance).abs() < 1e-4);
        assert!((camera.eye.x + distance).abs() < 1e-4);
    }
}
