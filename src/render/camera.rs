// Camera and controls

use glam::{Mat4, Vec3};

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

    // Orbit around target
    pub fn orbit(&mut self, delta_x: f32, delta_y: f32) {
        let radius = (self.eye - self.target).length();
        let direction = (self.eye - self.target).normalize();

        // Convert to spherical coordinates
        let mut theta = direction.y.atan2(direction.x); // Azimuth
        let mut phi = direction.z.clamp(-1.0, 1.0).acos(); // Polar angle (direction is normalized)

        // Apply deltas (with sensitivity scaling)
        theta -= delta_x * 0.01;
        phi = (phi - delta_y * 0.01).clamp(0.01, std::f32::consts::PI - 0.01);

        // Convert back to cartesian
        let new_direction = Vec3::new(
            phi.sin() * theta.cos(),
            phi.sin() * theta.sin(),
            phi.cos(),
        );

        self.eye = self.target + new_direction * radius;
    }

    // Pan camera (move target and eye)
    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward).normalize();

        let pan_speed = 0.1;
        let offset = right * delta_x * pan_speed + up * delta_y * pan_speed;

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
}
