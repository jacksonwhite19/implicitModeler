use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

use glam::Vec3;
use serde::Serialize;

use crate::analysis::compute_model_properties;
use crate::analysis::print_analysis::{PrintAnalysisSettings, compute_print_analysis};
use crate::analysis::thickness::compute_thickness;
use crate::mesh::Mesh;
use crate::mesh::{MeshQualityReport, analyze_mesh_quality};
use crate::pipeline::compute_sdf_grid_cached_arc;
use crate::sdf::Sdf;

#[derive(Clone, Debug, Serialize)]
pub struct ValidationSettings {
    pub grid_resolution: u32,
    pub min_wall_thickness_mm: f32,
    pub max_disconnected_bodies: usize,
    pub min_volume_mm3: f32,
    pub max_degenerate_triangle_count: usize,
    pub max_degenerate_triangle_ratio: f32,
    pub max_surface_area_to_volume_ratio: f32,
    pub max_support_volume_ratio: f32,
}

impl Default for ValidationSettings {
    fn default() -> Self {
        Self {
            grid_resolution: 32,
            min_wall_thickness_mm: 1.0,
            max_disconnected_bodies: 1,
            min_volume_mm3: 1.0,
            max_degenerate_triangle_count: 1024,
            max_degenerate_triangle_ratio: 0.10,
            max_surface_area_to_volume_ratio: 8.0,
            max_support_volume_ratio: 1.25,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct ValidationMetrics {
    pub vertex_count: usize,
    pub triangle_count: usize,
    pub mesh_quality: MeshQualityReport,
    pub disconnected_body_count: usize,
    pub support_volume_estimate_mm3: f32,
    pub support_volume_ratio: f32,
    pub overhang_area_mm2: f32,
    pub critical_overhang_area_mm2: f32,
    pub surface_area_to_volume_ratio: f32,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct GeometryValidationResult {
    pub valid: bool,
    pub hard_failures: Vec<String>,
    pub warnings: Vec<String>,
    pub penalties: HashMap<String, f32>,
    pub metrics: ValidationMetrics,
}

fn mesh_bounds(mesh: &Mesh) -> Option<(Vec3, Vec3)> {
    if mesh.vertices.is_empty() {
        return None;
    }
    let mut min = Vec3::splat(f32::MAX);
    let mut max = Vec3::splat(f32::MIN);
    for v in &mesh.vertices {
        let p = Vec3::from(v.position);
        min = min.min(p);
        max = max.max(p);
    }
    Some((min, max))
}

fn count_grid_components(grid: &crate::render::SdfGrid) -> usize {
    let res = grid.resolution as usize;
    let total = res * res * res;
    if total == 0 {
        return 0;
    }

    let mut visited = vec![false; total];
    let mut components = 0usize;
    let neighbors = [
        (1isize, 0isize, 0isize),
        (-1, 0, 0),
        (0, 1, 0),
        (0, -1, 0),
        (0, 0, 1),
        (0, 0, -1),
    ];

    for start in 0..total {
        if visited[start] || grid.data[start] >= 0.0 {
            continue;
        }
        components += 1;
        let mut queue = VecDeque::from([start]);
        visited[start] = true;

        while let Some(idx) = queue.pop_front() {
            let x = idx % res;
            let y = (idx / res) % res;
            let z = idx / (res * res);

            for (dx, dy, dz) in neighbors {
                let nx = x as isize + dx;
                let ny = y as isize + dy;
                let nz = z as isize + dz;
                if nx < 0
                    || ny < 0
                    || nz < 0
                    || nx >= res as isize
                    || ny >= res as isize
                    || nz >= res as isize
                {
                    continue;
                }
                let next = nx as usize + ny as usize * res + nz as usize * res * res;
                if !visited[next] && grid.data[next] < 0.0 {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
    }

    components
}

pub fn validate_geometry(
    sdf: &Arc<dyn Sdf>,
    mesh: &Mesh,
    settings: &ValidationSettings,
) -> GeometryValidationResult {
    let mut result = GeometryValidationResult {
        valid: true,
        ..Default::default()
    };

    result.metrics.vertex_count = mesh.vertices.len();
    result.metrics.triangle_count = mesh.indices.len() / 3;
    result.metrics.mesh_quality = analyze_mesh_quality(mesh);

    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        result.valid = false;
        result.hard_failures.push("empty_mesh".to_string());
        return result;
    }

    let Some((bb_min, bb_max)) = mesh_bounds(mesh) else {
        result.valid = false;
        result.hard_failures.push("invalid_bounds".to_string());
        return result;
    };

    if !bb_min.is_finite() || !bb_max.is_finite() {
        result.valid = false;
        result.hard_failures.push("non_finite_bounds".to_string());
        return result;
    }

    let degenerate_count = result.metrics.mesh_quality.degenerate_triangle_count;
    if degenerate_count > 0 {
        let degenerate_ratio =
            degenerate_count as f32 / result.metrics.triangle_count.max(1) as f32;
        let exceeds_count = degenerate_count > settings.max_degenerate_triangle_count;
        let exceeds_ratio = degenerate_ratio > settings.max_degenerate_triangle_ratio.max(0.0);
        let message = format!(
            "degenerate_triangles:{}:{:.4}",
            degenerate_count, degenerate_ratio
        );
        if exceeds_count || exceeds_ratio {
            result.valid = false;
            result.hard_failures.push(message);
        } else {
            result.warnings.push(message);
        }
    }
    if result.metrics.mesh_quality.non_manifold_edge_count > 0 {
        result.valid = false;
        result.hard_failures.push(format!(
            "non_manifold_edges:{}",
            result.metrics.mesh_quality.non_manifold_edge_count
        ));
    }
    if result.metrics.mesh_quality.boundary_edge_count > 0 {
        result.warnings.push(format!(
            "open_boundary_edges:{}",
            result.metrics.mesh_quality.boundary_edge_count
        ));
    }
    if result.metrics.mesh_quality.connected_component_count > 1 {
        result.warnings.push(format!(
            "mesh_components:{}",
            result.metrics.mesh_quality.connected_component_count
        ));
    }

    let span = (bb_max - bb_min).max(Vec3::splat(1.0));
    let pad = span * 0.05 + Vec3::splat(1.0);
    let grid = compute_sdf_grid_cached_arc(
        sdf,
        bb_min - pad,
        bb_max + pad,
        settings.grid_resolution.clamp(16, 64),
    );
    let (volume, surface_area, _grid_cg) = compute_model_properties(&grid);
    if volume < settings.min_volume_mm3 {
        result.valid = false;
        result.hard_failures.push("volume_too_small".to_string());
    }
    let sa_to_vol = if volume > 1e-6 {
        surface_area / volume
    } else {
        0.0
    };
    result.metrics.surface_area_to_volume_ratio = sa_to_vol;
    if sa_to_vol > settings.max_surface_area_to_volume_ratio {
        result
            .warnings
            .push("surface_area_to_volume_high".to_string());
        result.penalties.insert(
            "surface_area_to_volume".to_string(),
            ((sa_to_vol / settings.max_surface_area_to_volume_ratio) - 1.0).max(0.0),
        );
    }
    let thickness = compute_thickness(&grid, settings.min_wall_thickness_mm.max(2.0) * 4.0);
    let body_count = count_grid_components(&grid);
    result.metrics.disconnected_body_count = body_count;
    if body_count > settings.max_disconnected_bodies {
        result.valid = false;
        result
            .hard_failures
            .push(format!("too_many_disconnected_bodies:{body_count}"));
    } else if body_count > 1 {
        result
            .warnings
            .push(format!("multiple_bodies:{body_count}"));
    }
    if thickness.min_thickness > 0.0 && thickness.min_thickness < settings.min_wall_thickness_mm {
        result.valid = false;
        result
            .hard_failures
            .push(format!("wall_too_thin:{:.3}mm", thickness.min_thickness));
    } else if thickness.min_thickness > 0.0
        && thickness.min_thickness < settings.min_wall_thickness_mm * 1.25
    {
        result
            .warnings
            .push(format!("wall_near_limit:{:.3}mm", thickness.min_thickness));
    }

    let print_analysis = compute_print_analysis(
        &grid,
        &PrintAnalysisSettings {
            min_wall_thickness: settings.min_wall_thickness_mm,
            ..Default::default()
        },
        Some(&thickness),
        None,
    );
    result.metrics.overhang_area_mm2 = print_analysis.overhang.overhang_area_mm2;
    result.metrics.critical_overhang_area_mm2 = print_analysis.overhang.critical_overhang_area_mm2;
    result.metrics.support_volume_estimate_mm3 =
        print_analysis.overhang.support_volume_estimate_mm3;
    result.metrics.support_volume_ratio = if volume > 1e-6 {
        print_analysis.overhang.support_volume_estimate_mm3 / volume
    } else {
        0.0
    };

    if result.metrics.critical_overhang_area_mm2 > 0.0 {
        result
            .warnings
            .push("critical_overhang_present".to_string());
    }

    if result.metrics.support_volume_ratio > settings.max_support_volume_ratio {
        result.warnings.push("support_volume_high".to_string());
        result.penalties.insert(
            "support_bloat".to_string(),
            ((result.metrics.support_volume_ratio / settings.max_support_volume_ratio) - 1.0)
                .max(0.0),
        );
    }

    result
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use glam::Vec3;

    use super::*;
    use crate::mesh::MeshQuality;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::transforms::Translate;
    use crate::{export::build_export_mesh, sdf::booleans::Union};

    #[test]
    fn test_validate_geometry_accepts_single_sphere() {
        let sdf: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let mesh = build_export_mesh(
            sdf.as_ref(),
            Vec3::splat(-15.0),
            Vec3::splat(15.0),
            MeshQuality::Normal,
            false,
        );
        let result = validate_geometry(&sdf, &mesh, &ValidationSettings::default());
        assert!(
            result.valid,
            "sphere should validate: {:?}",
            result.hard_failures
        );
        assert_eq!(result.metrics.disconnected_body_count, 1);
    }

    #[test]
    fn test_validate_geometry_rejects_disconnected_bodies() {
        let left: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(5.0)),
            Vec3::new(-20.0, 0.0, 0.0),
        ));
        let right: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(5.0)),
            Vec3::new(20.0, 0.0, 0.0),
        ));
        let sdf: Arc<dyn Sdf> = Arc::new(Union::new(left, right));
        let mesh = build_export_mesh(
            sdf.as_ref(),
            Vec3::new(-30.0, -10.0, -10.0),
            Vec3::new(30.0, 10.0, 10.0),
            MeshQuality::Normal,
            false,
        );
        let result = validate_geometry(&sdf, &mesh, &ValidationSettings::default());
        assert!(!result.valid, "two disconnected spheres should fail");
        assert!(
            result
                .hard_failures
                .iter()
                .any(|f| f.starts_with("too_many_disconnected_bodies"))
        );
    }
}
