use glam::Vec3;

use crate::sdf::Sdf;

#[derive(Clone, Copy, Debug)]
pub struct AdaptiveOctreeSettings {
    pub max_depth: u32,
    pub min_cell_size_mm: f32,
    pub surface_band_mm: f32,
    pub error_tolerance_mm: f32,
    pub curvature_refine_threshold: f32,
    pub feature_scale_factor: f32,
    pub max_leaf_cells: usize,
}

impl Default for AdaptiveOctreeSettings {
    fn default() -> Self {
        Self {
            max_depth: 6,
            min_cell_size_mm: 1.0,
            surface_band_mm: 2.0,
            error_tolerance_mm: 0.5,
            curvature_refine_threshold: 0.15,
            feature_scale_factor: 2.5,
            max_leaf_cells: 32_768,
        }
    }
}

#[derive(Clone, Debug)]
pub struct OctreeNode {
    pub min: Vec3,
    pub max: Vec3,
    pub depth: u32,
    pub corner_min_distance: f32,
    pub corner_max_distance: f32,
    pub intersects_surface: bool,
    pub curvature_estimate: f32,
    pub error_estimate: f32,
    pub feature_size_estimate: f32,
    pub refinement_flags: u32,
    pub children: Option<Vec<OctreeNode>>,
}

impl OctreeNode {
    pub fn is_leaf(&self) -> bool {
        self.children.is_none()
    }

    pub fn cell_size(&self) -> Vec3 {
        self.max - self.min
    }
}

#[derive(Clone, Debug)]
pub struct AdaptiveOctree {
    pub root: OctreeNode,
}

impl AdaptiveOctree {
    pub fn leaf_count(&self) -> usize {
        fn count(node: &OctreeNode) -> usize {
            match &node.children {
                Some(children) => children.iter().map(count).sum(),
                None => 1,
            }
        }
        count(&self.root)
    }

    pub fn surface_leaf_count(&self) -> usize {
        fn count(node: &OctreeNode) -> usize {
            match &node.children {
                Some(children) => children.iter().map(count).sum(),
                None => usize::from(node.intersects_surface),
            }
        }
        count(&self.root)
    }
}

#[derive(Clone, Copy, Debug)]
struct CellSampleSummary {
    min_distance: f32,
    max_distance: f32,
    intersects_surface: bool,
    curvature_estimate: f32,
    error_estimate: f32,
    feature_size_estimate: f32,
    has_multi_crossing: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AdaptiveDomainSummary {
    pub leaf_count: usize,
    pub surface_leaf_count: usize,
    pub max_depth_reached: u32,
    pub surface_bounds_min: Vec3,
    pub surface_bounds_max: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct SurfaceLeafCell {
    pub min: Vec3,
    pub max: Vec3,
    pub depth: u32,
}

pub const REFINE_ZERO_SET: u32 = 1 << 0;
pub const REFINE_CURVATURE: u32 = 1 << 1;
pub const REFINE_FEATURE_SIZE: u32 = 1 << 2;
pub const REFINE_ERROR_TOLERANCE: u32 = 1 << 3;
pub const REFINE_SURFACE_BAND: u32 = 1 << 4;
pub const REFINE_MULTI_CROSSING: u32 = 1 << 5;
pub const REFINE_MAX_DEPTH: u32 = 1 << 6;
pub const REFINE_MIN_CELL: u32 = 1 << 7;
pub const REFINE_BUDGET_EXHAUSTED: u32 = 1 << 8;

fn estimate_gradient(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    ) / (2.0 * eps)
}

fn classify_sign(d: f32, deadband: f32) -> i8 {
    if d > deadband {
        1
    } else if d < -deadband {
        -1
    } else {
        0
    }
}

fn count_sign_alternations(samples: &[f32], deadband: f32) -> u32 {
    let mut alternations = 0u32;
    let mut prev = 0i8;
    for &sample in samples {
        let sign = classify_sign(sample, deadband);
        if sign == 0 {
            continue;
        }
        if prev != 0 && sign != prev {
            alternations += 1;
        }
        prev = sign;
    }
    alternations
}

fn sample_cell_summary(sdf: &dyn Sdf, min: Vec3, max: Vec3) -> CellSampleSummary {
    let center = (min + max) * 0.5;
    let eps = ((max - min).max_element() * 0.2).max(0.05);
    let cell_extent = (max - min).max_element();
    let sign_deadband = (cell_extent * 0.005_f32).max(1e-4_f32);
    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, max.y, max.z),
    ];
    let face_centers = [
        Vec3::new(min.x, center.y, center.z),
        Vec3::new(max.x, center.y, center.z),
        Vec3::new(center.x, min.y, center.z),
        Vec3::new(center.x, max.y, center.z),
        Vec3::new(center.x, center.y, min.z),
        Vec3::new(center.x, center.y, max.z),
        center,
    ];

    let mut min_d = f32::MAX;
    let mut max_d = f32::MIN;
    let mut probe_values = Vec::with_capacity(corners.len() + face_centers.len());
    for corner in corners {
        let d = sdf.distance(corner);
        min_d = min_d.min(d);
        max_d = max_d.max(d);
        probe_values.push(d);
    }
    for probe in face_centers {
        let d = sdf.distance(probe);
        min_d = min_d.min(d);
        max_d = max_d.max(d);
        probe_values.push(d);
    }
    let intersects_surface = min_d <= 0.0 && max_d >= 0.0;
    let center_grad = estimate_gradient(sdf, center, eps);
    let mut curvature = 0.0f32;
    for probe in face_centers.into_iter().take(6) {
        let grad = estimate_gradient(sdf, probe, eps);
        curvature = curvature.max((grad - center_grad).length());
    }
    let avg = probe_values.iter().copied().sum::<f32>() / probe_values.len() as f32;
    let error_estimate = probe_values
        .iter()
        .map(|d| (d - avg).abs())
        .fold(0.0f32, f32::max);
    let feature_size_estimate = min_d.abs().min(max_d.abs());
    let sample_axis_line = |start: Vec3, step: Vec3| -> [f32; 5] {
        std::array::from_fn(|i| sdf.distance(start + step * i as f32))
    };
    let x_line = sample_axis_line(
        Vec3::new(min.x, center.y, center.z),
        Vec3::new((max.x - min.x) * 0.25, 0.0, 0.0),
    );
    let y_line = sample_axis_line(
        Vec3::new(center.x, min.y, center.z),
        Vec3::new(0.0, (max.y - min.y) * 0.25, 0.0),
    );
    let z_line = sample_axis_line(
        Vec3::new(center.x, center.y, min.z),
        Vec3::new(0.0, 0.0, (max.z - min.z) * 0.25),
    );
    let max_sign_alternations = count_sign_alternations(&x_line, sign_deadband)
        .max(count_sign_alternations(&y_line, sign_deadband))
        .max(count_sign_alternations(&z_line, sign_deadband));
    CellSampleSummary {
        min_distance: min_d,
        max_distance: max_d,
        intersects_surface,
        curvature_estimate: curvature,
        error_estimate,
        feature_size_estimate,
        has_multi_crossing: max_sign_alternations > 1,
    }
}

fn node_should_split(
    summary: &CellSampleSummary,
    max_dim: f32,
    depth: u32,
    settings: &AdaptiveOctreeSettings,
    remaining_budget: usize,
) -> (bool, u32) {
    let near_surface =
        summary.min_distance.abs().min(summary.max_distance.abs()) <= settings.surface_band_mm;
    let mut flags = 0u32;
    if summary.intersects_surface {
        flags |= REFINE_ZERO_SET;
    }
    if near_surface {
        flags |= REFINE_SURFACE_BAND;
    }
    if near_surface && summary.curvature_estimate >= settings.curvature_refine_threshold {
        flags |= REFINE_CURVATURE;
    }
    if near_surface && summary.feature_size_estimate <= max_dim * settings.feature_scale_factor {
        flags |= REFINE_FEATURE_SIZE;
    }
    if near_surface && summary.error_estimate >= settings.error_tolerance_mm {
        flags |= REFINE_ERROR_TOLERANCE;
    }
    if summary.has_multi_crossing {
        flags |= REFINE_MULTI_CROSSING;
    }
    if depth >= settings.max_depth {
        flags |= REFINE_MAX_DEPTH;
    }
    if max_dim <= settings.min_cell_size_mm {
        flags |= REFINE_MIN_CELL;
    }
    if remaining_budget <= 1 {
        flags |= REFINE_BUDGET_EXHAUSTED;
    }
    let wants_split = flags
        & (REFINE_ZERO_SET
            | REFINE_SURFACE_BAND
            | REFINE_CURVATURE
            | REFINE_FEATURE_SIZE
            | REFINE_ERROR_TOLERANCE
            | REFINE_MULTI_CROSSING)
        != 0;
    let allowed = flags & (REFINE_MAX_DEPTH | REFINE_MIN_CELL | REFINE_BUDGET_EXHAUSTED) == 0;
    (wants_split && allowed, flags)
}

fn build_node(
    sdf: &dyn Sdf,
    min: Vec3,
    max: Vec3,
    depth: u32,
    settings: &AdaptiveOctreeSettings,
    remaining_budget: &mut usize,
) -> OctreeNode {
    let summary = sample_cell_summary(sdf, min, max);
    let size = max - min;
    let max_dim = size.max_element();
    let (should_split, refinement_flags) =
        node_should_split(&summary, max_dim, depth, settings, *remaining_budget);

    let children = if should_split {
        let center = (min + max) * 0.5;
        let mut nodes = Vec::with_capacity(8);
        *remaining_budget = remaining_budget.saturating_sub(1);
        for &dx in &[min.x, center.x] {
            for &dy in &[min.y, center.y] {
                for &dz in &[min.z, center.z] {
                    let child_min = Vec3::new(dx, dy, dz);
                    let child_max = Vec3::new(
                        if dx == min.x { center.x } else { max.x },
                        if dy == min.y { center.y } else { max.y },
                        if dz == min.z { center.z } else { max.z },
                    );
                    nodes.push(build_node(
                        sdf,
                        child_min,
                        child_max,
                        depth + 1,
                        settings,
                        remaining_budget,
                    ));
                }
            }
        }
        Some(nodes)
    } else {
        None
    };

    OctreeNode {
        min,
        max,
        depth,
        corner_min_distance: summary.min_distance,
        corner_max_distance: summary.max_distance,
        intersects_surface: summary.intersects_surface,
        curvature_estimate: summary.curvature_estimate,
        error_estimate: summary.error_estimate,
        feature_size_estimate: summary.feature_size_estimate,
        refinement_flags,
        children,
    }
}

pub fn build_adaptive_octree(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    settings: &AdaptiveOctreeSettings,
) -> AdaptiveOctree {
    let mut remaining_budget = settings.max_leaf_cells.max(1);
    AdaptiveOctree {
        root: build_node(
            sdf,
            bounds_min,
            bounds_max,
            0,
            settings,
            &mut remaining_budget,
        ),
    }
}

pub fn summarize_octree(octree: &AdaptiveOctree) -> AdaptiveDomainSummary {
    fn walk(node: &OctreeNode, out: &mut AdaptiveDomainSummary) {
        out.max_depth_reached = out.max_depth_reached.max(node.depth);
        match &node.children {
            Some(children) => {
                for child in children {
                    walk(child, out);
                }
            }
            None => {
                out.leaf_count += 1;
                if node.intersects_surface {
                    out.surface_leaf_count += 1;
                    if out.surface_leaf_count == 1 {
                        out.surface_bounds_min = node.min;
                        out.surface_bounds_max = node.max;
                    } else {
                        out.surface_bounds_min = out.surface_bounds_min.min(node.min);
                        out.surface_bounds_max = out.surface_bounds_max.max(node.max);
                    }
                }
            }
        }
    }

    let mut summary = AdaptiveDomainSummary::default();
    walk(&octree.root, &mut summary);
    summary
}

pub fn collect_surface_leaves(octree: &AdaptiveOctree) -> Vec<SurfaceLeafCell> {
    fn walk(node: &OctreeNode, out: &mut Vec<SurfaceLeafCell>) {
        match &node.children {
            Some(children) => {
                for child in children {
                    walk(child, out);
                }
            }
            None => {
                if node.intersects_surface {
                    out.push(SurfaceLeafCell {
                        min: node.min,
                        max: node.max,
                        depth: node.depth,
                    });
                }
            }
        }
    }

    let mut leaves = Vec::new();
    walk(&octree.root, &mut leaves);
    leaves
}

#[cfg(test)]
mod tests {
    use glam::Vec3;

    use super::*;
    use crate::sdf::primitives::{SdfBox, Sphere};

    #[test]
    fn adaptive_octree_refines_near_surface() {
        let sphere = Sphere::new(10.0);
        let octree = build_adaptive_octree(
            &sphere,
            Vec3::splat(-16.0),
            Vec3::splat(16.0),
            &AdaptiveOctreeSettings {
                max_depth: 4,
                min_cell_size_mm: 2.0,
                surface_band_mm: 1.5,
                error_tolerance_mm: 0.5,
                curvature_refine_threshold: 0.15,
                feature_scale_factor: 2.5,
                max_leaf_cells: 32_768,
            },
        );
        assert!(
            octree.leaf_count() > 1,
            "surface-intersecting cells should refine"
        );
        assert!(
            octree.surface_leaf_count() > 0,
            "sphere should produce surface leaves"
        );
    }

    #[test]
    fn adaptive_octree_stays_coarse_for_fully_inside_box() {
        let solid = SdfBox::new(Vec3::splat(200.0));
        let octree = build_adaptive_octree(
            &solid,
            Vec3::splat(-10.0),
            Vec3::splat(10.0),
            &AdaptiveOctreeSettings {
                max_depth: 4,
                min_cell_size_mm: 1.0,
                surface_band_mm: 1.0,
                error_tolerance_mm: 0.5,
                curvature_refine_threshold: 0.15,
                feature_scale_factor: 2.5,
                max_leaf_cells: 32_768,
            },
        );
        assert_eq!(
            octree.leaf_count(),
            1,
            "cell far from the zero set should not refine"
        );
        assert!(!octree.root.intersects_surface);
    }

    #[test]
    fn adaptive_octree_summary_tracks_surface_bounds() {
        let sphere = Sphere::new(8.0);
        let octree = build_adaptive_octree(
            &sphere,
            Vec3::splat(-12.0),
            Vec3::splat(12.0),
            &AdaptiveOctreeSettings::default(),
        );
        let summary = summarize_octree(&octree);
        assert!(summary.surface_leaf_count > 0);
        assert!(summary.surface_bounds_min.x < 0.0);
        assert!(summary.surface_bounds_max.x > 0.0);
    }

    #[test]
    fn adaptive_octree_stops_when_leaf_budget_exhausted() {
        let sphere = Sphere::new(10.0);
        let octree = build_adaptive_octree(
            &sphere,
            Vec3::splat(-16.0),
            Vec3::splat(16.0),
            &AdaptiveOctreeSettings {
                max_depth: 8,
                min_cell_size_mm: 0.5,
                surface_band_mm: 2.0,
                error_tolerance_mm: 0.1,
                curvature_refine_threshold: 0.05,
                feature_scale_factor: 3.0,
                max_leaf_cells: 32,
            },
        );
        assert!(
            octree.leaf_count() <= 32 * 8,
            "budget should cap runaway refinement"
        );
    }

    #[test]
    fn adaptive_octree_collects_surface_leaves() {
        let sphere = Sphere::new(8.0);
        let octree = build_adaptive_octree(
            &sphere,
            Vec3::splat(-12.0),
            Vec3::splat(12.0),
            &AdaptiveOctreeSettings::default(),
        );
        let leaves = collect_surface_leaves(&octree);
        assert!(!leaves.is_empty());
    }
}
