// Mesh export functionality

use crate::mesh::{Mesh, MeshQuality, adaptive_mc};
use crate::sdf::Sdf;
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;
use glam::Vec3;

pub fn build_export_mesh(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    quality: MeshQuality,
    smooth_normals: bool,
) -> Mesh {
    let target_cell = quality.target_cell_size_mm().max(0.02);
    adaptive_mc::extract_mesh_adaptive(sdf, bounds_min, bounds_max, target_cell, smooth_normals)
}

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

pub fn export_by_format(mesh: &Mesh, path: &Path, format: &str) -> Result<(), String> {
    match format.to_ascii_lowercase().as_str() {
        "stl" => export_stl(mesh, path.to_str().ok_or("invalid output path")?).map_err(|e| e.to_string()),
        "obj" => export_obj(mesh, path.to_str().ok_or("invalid output path")?).map_err(|e| e.to_string()),
        other => Err(format!("Unknown format: {}. Use 'stl' or 'obj'.", other)),
    }
}

// ── Manufacturing package export ──────────────────────────────────────────────

/// A bundled manufacturing output: STL files, BOM, and assembly notes.
#[allow(dead_code)] // Part of manufacturing export API
pub struct ManufacturingPackage {
    pub stl_files: Vec<String>,
    pub bom_csv: String,
    pub assembly_notes: String,
}

/// Export a complete manufacturing package to `output_dir`.
#[allow(dead_code)] // Part of manufacturing export API — not yet called from UI
///
/// Creates:
/// - `main_body.stl` — binary STL mesh
/// - `bom.csv` — bill of materials (CSV)
/// - `bom.md` — bill of materials (Markdown)
/// - `assembly_notes.md` — print settings and assembly guidance
pub fn export_manufacturing_package(
    mesh: &Mesh,
    project_name: &str,
    output_dir: &Path,
) -> Result<ManufacturingPackage, String> {
    std::fs::create_dir_all(output_dir).map_err(|e| e.to_string())?;

    // Export main body STL
    let stl_path = output_dir.join("main_body.stl");
    export_stl(mesh, stl_path.to_str().ok_or("invalid path")?)
        .map_err(|e| e.to_string())?;

    let triangle_count = mesh.indices.len() / 3;

    // BOM CSV
    let bom_csv = format!(
        "Part,Type,Quantity,Notes\nmain_body,Printed Part,1,{} triangles\n",
        triangle_count
    );
    let bom_path = output_dir.join("bom.csv");
    std::fs::write(&bom_path, &bom_csv).map_err(|e| e.to_string())?;

    // BOM Markdown
    let bom_md = format!(
        "# Bill of Materials\n\n| Part | Type | Qty | Notes |\n|------|------|-----|-------|\n| main_body | Printed Part | 1 | {} triangles |\n",
        triangle_count
    );
    let bom_md_path = output_dir.join("bom.md");
    std::fs::write(&bom_md_path, &bom_md).map_err(|e| e.to_string())?;

    // Assembly notes — use chrono for date
    let date_str = chrono::Local::now().format("%Y-%m-%d").to_string();
    let assembly_notes = format!(
        "# Assembly Notes\n\n**Project:** {}\n**Date:** {}\n**Triangles:** {}\n\n## Print Settings\n\n- Layer height: 0.2mm\n- Infill: 20%\n- Material: PLA\n",
        project_name, date_str, triangle_count
    );
    let notes_path = output_dir.join("assembly_notes.md");
    std::fs::write(&notes_path, &assembly_notes).map_err(|e| e.to_string())?;

    Ok(ManufacturingPackage {
        stl_files: vec!["main_body.stl".to_string()],
        bom_csv,
        assembly_notes,
    })
}
