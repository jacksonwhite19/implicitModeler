// Mesh export functionality

use crate::mesh::Mesh;
use std::fs::File;
use std::io::{self, Write};

/// Export mesh to binary STL format
pub fn export_stl(mesh: &Mesh, path: &str) -> io::Result<()> {
    let mut file = File::create(path)?;

    // STL header (80 bytes)
    let header = [0u8; 80];
    file.write_all(&header)?;

    // Number of triangles (4 bytes, little-endian)
    let num_triangles = (mesh.indices.len() / 3) as u32;
    file.write_all(&num_triangles.to_le_bytes())?;

    // Write each triangle
    for i in (0..mesh.indices.len()).step_by(3) {
        let idx0 = mesh.indices[i] as usize;
        let idx1 = mesh.indices[i + 1] as usize;
        let idx2 = mesh.indices[i + 2] as usize;

        let v0 = &mesh.vertices[idx0];
        let v1 = &mesh.vertices[idx1];
        let v2 = &mesh.vertices[idx2];

        // Compute face normal from vertices
        let p0 = glam::Vec3::from_array(v0.position);
        let p1 = glam::Vec3::from_array(v1.position);
        let p2 = glam::Vec3::from_array(v2.position);

        let edge1 = p1 - p0;
        let edge2 = p2 - p0;
        let normal = edge1.cross(edge2).normalize_or_zero();

        // Write normal (12 bytes)
        file.write_all(&normal.x.to_le_bytes())?;
        file.write_all(&normal.y.to_le_bytes())?;
        file.write_all(&normal.z.to_le_bytes())?;

        // Write vertices (36 bytes total)
        for &vertex in &[v0, v1, v2] {
            file.write_all(&vertex.position[0].to_le_bytes())?;
            file.write_all(&vertex.position[1].to_le_bytes())?;
            file.write_all(&vertex.position[2].to_le_bytes())?;
        }

        // Attribute byte count (2 bytes, usually 0)
        file.write_all(&[0u8, 0u8])?;
    }

    Ok(())
}

/// Export mesh to OBJ format
pub fn export_obj(mesh: &Mesh, path: &str) -> io::Result<()> {
    let mut file = File::create(path)?;

    // Write header comment
    writeln!(file, "# Exported from Implicit CAD")?;
    writeln!(file, "# Vertices: {}", mesh.vertices.len())?;
    writeln!(file, "# Triangles: {}", mesh.indices.len() / 3)?;
    writeln!(file)?;

    // Write vertices
    for vertex in &mesh.vertices {
        writeln!(
            file,
            "v {} {} {}",
            vertex.position[0], vertex.position[1], vertex.position[2]
        )?;
    }

    writeln!(file)?;

    // Write vertex normals
    for vertex in &mesh.vertices {
        writeln!(
            file,
            "vn {} {} {}",
            vertex.normal[0], vertex.normal[1], vertex.normal[2]
        )?;
    }

    writeln!(file)?;

    // Write faces (OBJ uses 1-based indexing)
    for i in (0..mesh.indices.len()).step_by(3) {
        let idx0 = mesh.indices[i] + 1;
        let idx1 = mesh.indices[i + 1] + 1;
        let idx2 = mesh.indices[i + 2] + 1;

        // Format: f v1//vn1 v2//vn2 v3//vn3 (vertex//normal)
        writeln!(
            file,
            "f {}//{} {}//{} {}//{}",
            idx0, idx0, idx1, idx1, idx2, idx2
        )?;
    }

    Ok(())
}
