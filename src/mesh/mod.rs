// Mesh data structures

use bytemuck::{Pod, Zeroable};
use glam::Vec3;

/// Vertex with position and normal
#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

/// Triangle mesh
#[derive(Clone, Debug)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

pub mod marching_cubes;
pub mod adaptive_mc;
pub use adaptive_mc::MeshQuality;

/// Compute signed mesh volume using the divergence theorem (signed tetrahedral volumes).
/// Units match the scene units cubed (mm³ if scene is in mm).
pub fn compute_volume(mesh: &Mesh) -> f32 {
    let mut volume = 0.0f32;
    for tri in mesh.indices.chunks(3) {
        let v0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let v1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let v2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        volume += v0.dot(v1.cross(v2));
    }
    (volume / 6.0).abs()
}
