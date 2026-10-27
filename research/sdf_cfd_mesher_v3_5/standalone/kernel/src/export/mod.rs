// Mesh export functionality
pub mod adaptive_octree;
pub mod aero;
pub mod dual_contouring;

use crate::export::adaptive_octree::{
    AdaptiveOctreeSettings, build_adaptive_octree, summarize_octree,
};
use crate::export::dual_contouring::{
    boundary_edge_count, extract_dual_contour_mesh, extract_dual_contour_mesh_from_octree,
};
use crate::gpu::{extract_mesh_from_vertex_grid_gpu, extract_mesh_gpu, lower_sdf_ir};
use crate::mesh::{
    Mesh, MeshQuality, Vertex, adaptive_mc,
    marching_cubes::{self, CUBE_CORNERS, EDGE_TABLE, EDGE_VERTICES, TRI_TABLE},
};
use crate::pipeline::compute_sdf_grid;
use crate::sdf::Sdf;
use glam::Vec3;
use std::collections::HashMap;
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;

pub fn build_export_mesh(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    quality: MeshQuality,
    smooth_normals: bool,
) -> Mesh {
    build_export_mesh_with_target_cell(
        sdf,
        bounds_min,
        bounds_max,
        quality.target_cell_size_mm(),
        quality,
        smooth_normals,
    )
}

pub fn build_export_mesh_with_target_cell(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
    quality_hint: MeshQuality,
    smooth_normals: bool,
) -> Mesh {
    if let Some(ir) = lower_sdf_ir(sdf) {
        if let Ok(mesh) =
            extract_mesh_gpu(&ir, bounds_min, bounds_max, quality_hint, smooth_normals)
        {
            if accelerated_mesh_is_usable(&mesh) {
                return mesh;
            }
        }
    }
    let target_cell = target_cell_mm.max(0.02);
    let cell_resolution =
        (((bounds_max - bounds_min).max_element() / target_cell).ceil() as u32).clamp(8, 256);
    let vertex_resolution = cell_resolution + 1;
    let vertex_grid = compute_sdf_grid(sdf, bounds_min, bounds_max, vertex_resolution);
    if let Ok(mesh) = extract_mesh_from_vertex_grid_gpu(
        bounds_min,
        bounds_max,
        cell_resolution,
        &vertex_grid.data,
        smooth_normals,
    ) {
        if accelerated_mesh_is_usable(&mesh) {
            return mesh;
        }
    }
    let mesh = extract_mesh_from_vertex_grid_cpu(
        bounds_min,
        bounds_max,
        cell_resolution,
        &vertex_grid.data,
        smooth_normals,
    );
    if accelerated_mesh_is_usable(&mesh) {
        return mesh;
    }
    adaptive_mc::extract_mesh_adaptive(sdf, bounds_min, bounds_max, target_cell, smooth_normals)
}

fn accelerated_mesh_is_usable(mesh: &Mesh) -> bool {
    if mesh.vertices.is_empty() || mesh.indices.len() < 3 {
        return false;
    }

    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for vertex in &mesh.vertices {
        let p = Vec3::from_array(vertex.position);
        if !p.is_finite() {
            return false;
        }
        min = min.min(p);
        max = max.max(p);
    }

    if (max - min).max_element() <= 1e-5 {
        return false;
    }

    for tri in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [tri[0] as usize, tri[1] as usize, tri[2] as usize];
        if a >= mesh.vertices.len() || b >= mesh.vertices.len() || c >= mesh.vertices.len() {
            return false;
        }
        let p0 = Vec3::from_array(mesh.vertices[a].position);
        let p1 = Vec3::from_array(mesh.vertices[b].position);
        let p2 = Vec3::from_array(mesh.vertices[c].position);
        if (p1 - p0).cross(p2 - p0).length_squared() > 1e-12 {
            return true;
        }
    }

    false
}

fn extract_mesh_from_vertex_grid_cpu(
    bounds_min: Vec3,
    bounds_max: Vec3,
    cell_resolution: u32,
    vertex_scalars: &[f32],
    smooth_normals: bool,
) -> Mesh {
    let vertex_resolution = cell_resolution + 1;
    let expected = (vertex_resolution * vertex_resolution * vertex_resolution) as usize;
    if vertex_scalars.len() != expected {
        return Mesh {
            vertices: Vec::new(),
            indices: Vec::new(),
        };
    }

    let step = (bounds_max - bounds_min) / cell_resolution as f32;
    let min_area = (step.x.min(step.y).min(step.z) * 0.001).powi(2);
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let grid_index = |x: u32, y: u32, z: u32| -> usize {
        (x + y * vertex_resolution + z * vertex_resolution * vertex_resolution) as usize
    };

    for z in 0..cell_resolution {
        for y in 0..cell_resolution {
            for x in 0..cell_resolution {
                let mut cube_index = 0usize;
                let mut corner_values = [0.0f32; 8];
                let mut corner_positions = [Vec3::ZERO; 8];

                for (corner_idx, corner) in CUBE_CORNERS.iter().enumerate() {
                    let gx = x + corner[0];
                    let gy = y + corner[1];
                    let gz = z + corner[2];
                    let value = vertex_scalars[grid_index(gx, gy, gz)];
                    corner_values[corner_idx] = value;
                    corner_positions[corner_idx] = bounds_min
                        + Vec3::new(gx as f32 * step.x, gy as f32 * step.y, gz as f32 * step.z);
                    if value < 0.0 {
                        cube_index |= 1 << corner_idx;
                    }
                }

                let edge_flags = EDGE_TABLE[cube_index];
                if cube_index == 0 || cube_index == 255 || edge_flags == 0 {
                    continue;
                }

                let mut edge_positions = [Vec3::ZERO; 12];
                for edge_idx in 0..12 {
                    if (edge_flags & (1 << edge_idx)) == 0 {
                        continue;
                    }
                    let [v0, v1] = EDGE_VERTICES[edge_idx];
                    let d0 = corner_values[v0];
                    let d1 = corner_values[v1];
                    let t = if (d1 - d0).abs() > 1e-6 {
                        (-d0 / (d1 - d0)).clamp(0.0, 1.0)
                    } else {
                        0.5
                    };
                    edge_positions[edge_idx] =
                        corner_positions[v0] + (corner_positions[v1] - corner_positions[v0]) * t;
                }

                let tri_row = &TRI_TABLE[cube_index];
                let mut i = 0;
                while i < 16 && tri_row[i] >= 0 {
                    let p0 = edge_positions[tri_row[i] as usize];
                    let p1 = edge_positions[tri_row[i + 1] as usize];
                    let p2 = edge_positions[tri_row[i + 2] as usize];
                    let normal = (p1 - p0).cross(p2 - p0);
                    let area = normal.length() * 0.5;
                    if area > min_area {
                        let normal = normal.normalize_or_zero();
                        let base = vertices.len() as u32;
                        vertices.push(Vertex {
                            position: p0.to_array(),
                            normal: normal.to_array(),
                        });
                        vertices.push(Vertex {
                            position: p1.to_array(),
                            normal: normal.to_array(),
                        });
                        vertices.push(Vertex {
                            position: p2.to_array(),
                            normal: normal.to_array(),
                        });
                        indices.extend_from_slice(&[base, base + 1, base + 2]);
                    }
                    i += 3;
                }
            }
        }
    }

    let mesh = Mesh { vertices, indices };
    if smooth_normals {
        weld_mesh_vertices_local(&mesh, (step.x.min(step.y).min(step.z) * 0.05).max(0.02))
    } else {
        mesh
    }
}

pub fn build_export_mesh_uniform_reference(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
    smooth_normals: bool,
) -> Mesh {
    let target_cell = target_cell_mm.max(0.02);
    let cell_resolution =
        (((bounds_max - bounds_min).max_element() / target_cell).ceil() as u32).clamp(8, 512);
    let mesh =
        marching_cubes::extract_mesh(sdf, bounds_min, bounds_max, cell_resolution, smooth_normals);
    weld_mesh_vertices_local(&mesh, (target_cell * 0.05).max(0.02))
}

fn weld_mesh_vertices_local(mesh: &Mesh, tolerance_mm: f32) -> Mesh {
    if mesh.vertices.is_empty() {
        return mesh.clone();
    }

    let inv_tol = if tolerance_mm > 1e-8 {
        1.0 / tolerance_mm
    } else {
        1e6
    };
    let mut buckets: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
    let mut new_vertices: Vec<crate::mesh::Vertex> = Vec::new();
    let mut remap = vec![0u32; mesh.vertices.len()];

    for (old_idx, vertex) in mesh.vertices.iter().enumerate() {
        let p = Vec3::from_array(vertex.position);
        let key = (
            (p.x * inv_tol).round() as i32,
            (p.y * inv_tol).round() as i32,
            (p.z * inv_tol).round() as i32,
        );
        let mut mapped = None;
        if let Some(candidates) = buckets.get(&key) {
            for &candidate_idx in candidates {
                let c = Vec3::from_array(new_vertices[candidate_idx].position);
                if p.distance(c) <= tolerance_mm {
                    mapped = Some(candidate_idx as u32);
                    break;
                }
            }
        }
        let new_idx = if let Some(idx) = mapped {
            idx
        } else {
            let idx = new_vertices.len() as u32;
            new_vertices.push(*vertex);
            buckets.entry(key).or_default().push(idx as usize);
            idx
        };
        remap[old_idx] = new_idx;
    }

    let mut new_indices = mesh.indices.clone();
    for idx in &mut new_indices {
        *idx = remap[*idx as usize];
    }

    Mesh {
        vertices: new_vertices,
        indices: new_indices,
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AdaptiveExportSettings {
    pub quality: MeshQuality,
    pub smooth_normals: bool,
    pub octree: AdaptiveOctreeSettings,
    pub prefer_dual_contouring: bool,
}

impl Default for AdaptiveExportSettings {
    fn default() -> Self {
        Self {
            quality: MeshQuality::Normal,
            smooth_normals: false,
            octree: AdaptiveOctreeSettings::default(),
            prefer_dual_contouring: true,
        }
    }
}

#[derive(Clone, Debug)]
pub struct AdaptiveExportResult {
    pub mesh: Mesh,
    pub backend: &'static str,
    pub reason: String,
}

pub fn build_export_mesh_adaptive(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    settings: AdaptiveExportSettings,
) -> AdaptiveExportResult {
    let octree = build_adaptive_octree(sdf, bounds_min, bounds_max, &settings.octree);
    let summary = summarize_octree(&octree);
    if summary.surface_leaf_count == 0 {
        return AdaptiveExportResult {
            mesh: build_export_mesh(
                sdf,
                bounds_min,
                bounds_max,
                settings.quality,
                settings.smooth_normals,
            ),
            backend: "fallback_uniform",
            reason: "adaptive octree found no surface leaves".to_string(),
        };
    }

    let pad = Vec3::splat(
        settings
            .octree
            .surface_band_mm
            .max(settings.octree.min_cell_size_mm),
    );
    let tight_min = summary.surface_bounds_min - pad;
    let tight_max = summary.surface_bounds_max + pad;
    let target_cell = settings
        .quality
        .target_cell_size_mm()
        .max(settings.octree.min_cell_size_mm);
    if settings.prefer_dual_contouring {
        let mesh = extract_dual_contour_mesh_from_octree(sdf, &octree, target_cell);
        if !mesh.vertices.is_empty() && !mesh.indices.is_empty() && boundary_edge_count(&mesh) == 0
        {
            return AdaptiveExportResult {
                mesh,
                backend: "dual_contouring_octree",
                reason: format!(
                    "octree dual succeeded with {} surface leaves",
                    summary.surface_leaf_count
                ),
            };
        }
        let mesh = extract_dual_contour_mesh(sdf, tight_min, tight_max, target_cell);
        if !mesh.vertices.is_empty() && !mesh.indices.is_empty() && boundary_edge_count(&mesh) == 0
        {
            return AdaptiveExportResult {
                mesh,
                backend: "dual_contouring_uniform",
                reason: "uniform dual succeeded after octree dual fallback".to_string(),
            };
        }
    }

    let mesh = adaptive_mc::extract_mesh_adaptive(
        sdf,
        tight_min,
        tight_max,
        target_cell,
        settings.smooth_normals,
    );
    if !mesh.vertices.is_empty() && !mesh.indices.is_empty() {
        return AdaptiveExportResult {
            mesh,
            backend: "adaptive_mc_octree_domain",
            reason: format!(
                "dual contouring unavailable or non-watertight; used adaptive MC on restricted domain with {} surface leaves",
                summary.surface_leaf_count
            ),
        };
    }

    AdaptiveExportResult {
        mesh: build_export_mesh(
            sdf,
            bounds_min,
            bounds_max,
            settings.quality,
            settings.smooth_normals,
        ),
        backend: "fallback_uniform",
        reason: "adaptive restricted export failed; fell back to uniform export".to_string(),
    }
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
        "stl" => {
            export_stl(mesh, path.to_str().ok_or("invalid output path")?).map_err(|e| e.to_string())
        }
        "obj" => {
            export_obj(mesh, path.to_str().ok_or("invalid output path")?).map_err(|e| e.to_string())
        }
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
    export_stl(mesh, stl_path.to_str().ok_or("invalid path")?).map_err(|e| e.to_string())?;

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

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use glam::Vec3;

    use super::{AdaptiveExportSettings, build_export_mesh, build_export_mesh_adaptive};
    use crate::mesh::{Mesh, MeshQuality, Vertex};
    use crate::sdf::Sdf;
    use crate::sdf::field::lattice::GyroidLattice;
    use crate::sdf::primitives::Sphere;

    #[test]
    fn accelerated_mesh_gate_rejects_zero_extent_degenerate_output() {
        let degenerate = Mesh {
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 0.0],
                };
                3
            ],
            indices: vec![0, 1, 2],
        };
        assert!(!super::accelerated_mesh_is_usable(&degenerate));

        let valid = Mesh {
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                Vertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                Vertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2],
        };
        assert!(super::accelerated_mesh_is_usable(&valid));
    }

    #[test]
    fn cpu_vertex_grid_fallback_extracts_non_degenerate_mesh() {
        let bounds_min = Vec3::splat(-2.0);
        let bounds_max = Vec3::splat(2.0);
        let cell_resolution = 12u32;
        let vertex_resolution = cell_resolution + 1;
        let step = (bounds_max - bounds_min) / cell_resolution as f32;
        let mut scalars = Vec::new();

        for z in 0..vertex_resolution {
            for y in 0..vertex_resolution {
                for x in 0..vertex_resolution {
                    let p = bounds_min
                        + Vec3::new(x as f32 * step.x, y as f32 * step.y, z as f32 * step.z);
                    scalars.push(p.length() - 1.0);
                }
            }
        }

        let mesh = super::extract_mesh_from_vertex_grid_cpu(
            bounds_min,
            bounds_max,
            cell_resolution,
            &scalars,
            false,
        );
        assert!(super::accelerated_mesh_is_usable(&mesh));
        assert_eq!(mesh.indices.len() % 3, 0);
    }

    #[test]
    fn export_mesh_uses_gpu_fallback_for_unsupported_tree() {
        let shape: Arc<dyn Sdf> = Arc::new(GyroidLattice::new(8.0, 1.0));
        let mesh = build_export_mesh(
            shape.as_ref(),
            Vec3::new(-12.0, -12.0, -12.0),
            Vec3::new(12.0, 12.0, 12.0),
            MeshQuality::Draft,
            false,
        );
        assert!(
            !mesh.vertices.is_empty(),
            "Unsupported trees should still produce export meshes"
        );
        assert_eq!(
            mesh.indices.len() % 3,
            0,
            "Export mesh output should remain triangle indexed"
        );
    }

    #[test]
    fn adaptive_export_mesh_produces_mesh_for_supported_surface() {
        let shape = Sphere::new(10.0);
        let result = build_export_mesh_adaptive(
            &shape,
            Vec3::splat(-14.0),
            Vec3::splat(14.0),
            AdaptiveExportSettings {
                quality: MeshQuality::Draft,
                smooth_normals: false,
                ..Default::default()
            },
        );
        assert!(!result.mesh.vertices.is_empty());
        assert_eq!(result.mesh.indices.len() % 3, 0);
        assert!(matches!(
            result.backend,
            "dual_contouring_octree"
                | "dual_contouring_uniform"
                | "adaptive_mc_octree_domain"
                | "fallback_uniform"
        ));
    }

    #[test]
    fn adaptive_export_mesh_falls_back_when_dual_has_boundary_edges() {
        let shape = Sphere::new(10.0);
        let result = build_export_mesh_adaptive(
            &shape,
            Vec3::splat(-14.0),
            Vec3::splat(14.0),
            AdaptiveExportSettings {
                quality: MeshQuality::Draft,
                smooth_normals: false,
                prefer_dual_contouring: true,
                ..Default::default()
            },
        );
        assert!(!result.mesh.vertices.is_empty());
        assert!(
            matches!(
                result.backend,
                "adaptive_mc_octree_domain" | "dual_contouring_octree" | "dual_contouring_uniform"
            ),
            "adaptive export should choose a valid backend"
        );
    }
}
