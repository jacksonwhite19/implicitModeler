// Print analysis: overhang detection, orientation advisor, and feature detection
// for FDM-style 3D printing.
#![allow(dead_code)] // Result struct fields not all displayed in the current UI

use glam::Vec3;
use rayon::prelude::*;
use crate::render::SdfGrid;
use crate::analysis::thickness::ThicknessResult;

// ---------------------------------------------------------------------------
// Serde helpers for glam::Vec3 (stored as [f32; 3])
// ---------------------------------------------------------------------------

mod vec3_serde {
    use glam::Vec3;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(v: &Vec3, s: S) -> Result<S::Ok, S::Error> {
        [v.x, v.y, v.z].serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Vec3, D::Error> {
        let arr: [f32; 3] = Deserialize::deserialize(d)?;
        Ok(Vec3::new(arr[0], arr[1], arr[2]))
    }
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PrinterPreset {
    GenericFDM,
    BambuX1C,
    PrusaMK4,
    Voron24_300,
    Custom,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PrintAnalysisSettings {
    #[serde(with = "vec3_serde")]
    pub build_direction: Vec3,
    pub overhang_threshold_deg: f32,
    pub min_wall_thickness: f32,
    pub min_feature_size: f32,
    pub max_aspect_ratio: f32,
    #[serde(with = "vec3_serde")]
    pub build_volume: Vec3,
    pub preset: PrinterPreset,
}

impl Default for PrintAnalysisSettings {
    fn default() -> Self {
        Self {
            build_direction: Vec3::Z,
            overhang_threshold_deg: 45.0,
            min_wall_thickness: 1.2,
            min_feature_size: 0.4,
            max_aspect_ratio: 8.0,
            build_volume: Vec3::new(220.0, 220.0, 250.0),
            preset: PrinterPreset::GenericFDM,
        }
    }
}

impl PrintAnalysisSettings {
    pub fn apply_preset(&mut self, preset: PrinterPreset) {
        match preset {
            PrinterPreset::GenericFDM => {
                self.build_volume = Vec3::new(220.0, 220.0, 250.0);
                self.min_wall_thickness = 1.2;
                self.min_feature_size = 0.4;
            }
            PrinterPreset::BambuX1C => {
                self.build_volume = Vec3::new(256.0, 256.0, 256.0);
                self.min_wall_thickness = 0.8;
                self.min_feature_size = 0.2;
            }
            PrinterPreset::PrusaMK4 => {
                self.build_volume = Vec3::new(250.0, 210.0, 220.0);
                self.min_wall_thickness = 1.2;
                self.min_feature_size = 0.4;
            }
            PrinterPreset::Voron24_300 => {
                self.build_volume = Vec3::new(300.0, 300.0, 300.0);
                self.min_wall_thickness = 1.2;
                self.min_feature_size = 0.4;
            }
            PrinterPreset::Custom => {}
        }
        self.preset = preset;
    }
}

// ---------------------------------------------------------------------------
// Surface extraction
// ---------------------------------------------------------------------------

pub struct SurfacePoint {
    pub position: Vec3,
    pub normal: Vec3,
    pub sdf_value: f32,
    pub grid_idx: usize,
}

fn load_raw(data: &[f32], res: usize, ix: i32, iy: i32, iz: i32) -> f32 {
    let ci = ix.clamp(0, res as i32 - 1) as usize;
    let cj = iy.clamp(0, res as i32 - 1) as usize;
    let ck = iz.clamp(0, res as i32 - 1) as usize;
    data[ci + cj * res + ck * res * res]
}

pub fn extract_surface_points(grid: &SdfGrid) -> Vec<SurfacePoint> {
    let res = grid.resolution as usize;
    let span = (grid.bounds_max - grid.bounds_min).max_element().max(1e-6);
    let voxel_size = span / grid.resolution as f32;

    (0..res * res * res)
        .into_par_iter()
        .filter_map(|idx| {
            let sdf_val = grid.data[idx];
            if sdf_val.abs() >= voxel_size * 1.5 {
                return None;
            }

            let ix = (idx % res) as i32;
            let iy = ((idx / res) % res) as i32;
            let iz = (idx / (res * res)) as i32;

            let nx = load_raw(&grid.data, res, ix + 1, iy, iz)
                - load_raw(&grid.data, res, ix - 1, iy, iz);
            let ny = load_raw(&grid.data, res, ix, iy + 1, iz)
                - load_raw(&grid.data, res, ix, iy - 1, iz);
            let nz = load_raw(&grid.data, res, ix, iy, iz + 1)
                - load_raw(&grid.data, res, ix, iy, iz - 1);
            let n = Vec3::new(nx, ny, nz);
            let nl = n.length();
            if nl < 1e-6 {
                return None;
            }
            let normal = n / nl;

            let t = Vec3::new(
                (ix as f32 + 0.5) / res as f32,
                (iy as f32 + 0.5) / res as f32,
                (iz as f32 + 0.5) / res as f32,
            );
            let position = grid.bounds_min + t * (grid.bounds_max - grid.bounds_min);

            Some(SurfacePoint {
                position,
                normal,
                sdf_value: sdf_val,
                grid_idx: idx,
            })
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Overhang analysis
// ---------------------------------------------------------------------------

#[derive(Copy, Clone)]
pub enum OverhangClass {
    NotSurface = 0,
    Good = 1,
    Marginal = 2,
    Overhang = 3,
    Critical = 4,
}

pub struct OverhangResult {
    pub overhang_grid: Vec<f32>,
    pub overhang_area_mm2: f32,
    pub critical_overhang_area_mm2: f32,
    pub support_volume_estimate_mm3: f32,
    pub resolution: u32,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
}

pub fn compute_overhang_analysis(
    surface_points: &[SurfacePoint],
    settings: &PrintAnalysisSettings,
    grid: &SdfGrid,
) -> OverhangResult {
    let res = grid.resolution as usize;
    let total_voxels = res * res * res;
    let span = (grid.bounds_max - grid.bounds_min).max_element().max(1e-6);
    let voxel_size = span / grid.resolution as f32;
    let face_area = voxel_size * voxel_size;

    let build_dir = settings.build_direction.normalize();
    let threshold = settings.overhang_threshold_deg;

    let mut overhang_grid = vec![0.0_f32; total_voxels];

    let mut overhang_count = 0u32;
    let mut critical_count = 0u32;

    for sp in surface_points {
        // Compute the angle between the outward surface normal and the downward direction (-build_dir).
        // A small angle means the surface faces downward (an overhang that needs support).
        // A large angle (near 180°) means the surface faces upward (safe to print).
        // angle = 0°  → normal points straight down  → severe overhang (Critical)
        // angle = 90° → vertical wall               → Marginal/borderline
        // angle = 180°→ normal points straight up    → top face (Good, no support needed)
        let dot = sp.normal.dot(-build_dir).clamp(-1.0, 1.0);
        let angle_deg = dot.acos().to_degrees();

        // Remap: overhang_angle = 180 - angle_deg, so 0 = facing up (Good), 180 = facing down (Critical)
        // Equivalently: a surface needs support when dot(normal, -build_dir) > cos(threshold),
        // i.e., when angle_deg < threshold.
        let cls = if angle_deg >= 180.0 - threshold {
            // Normal has strong upward component → well-supported top face
            OverhangClass::Good
        } else if angle_deg >= 180.0 - (threshold + 10.0) {
            OverhangClass::Marginal
        } else if angle_deg >= 90.0 {
            // Vertical to slightly upward-leaning: borderline but usually printable
            OverhangClass::Marginal
        } else if angle_deg >= threshold {
            // Downward-leaning: overhang that may need support
            OverhangClass::Overhang
        } else {
            // Nearly straight down: critical overhang
            OverhangClass::Critical
        };

        overhang_grid[sp.grid_idx] = cls as u8 as f32;

        match cls {
            OverhangClass::Overhang => overhang_count += 1,
            OverhangClass::Critical => {
                overhang_count += 1;
                critical_count += 1;
            }
            _ => {}
        }
    }

    let overhang_area = overhang_count as f32 * face_area;
    let critical_overhang_area = critical_count as f32 * face_area;

    // Support volume estimate: overhang area * average height/2
    let build_extent = (grid.bounds_max - grid.bounds_min).dot(build_dir.abs());
    let avg_dist = build_extent / 2.0;
    let support_volume = overhang_area * avg_dist;

    OverhangResult {
        overhang_grid,
        overhang_area_mm2: overhang_area,
        critical_overhang_area_mm2: critical_overhang_area,
        support_volume_estimate_mm3: support_volume,
        resolution: grid.resolution,
        bounds_min: grid.bounds_min,
        bounds_max: grid.bounds_max,
    }
}

// ---------------------------------------------------------------------------
// Orientation advisor
// ---------------------------------------------------------------------------

pub struct OrientationCandidate {
    pub build_direction: Vec3,
    pub rotation_axis: Vec3,
    pub rotation_angle_deg: f32,
    pub overhang_area_mm2: f32,
    pub support_volume_mm3: f32,
    pub fits_build_volume: bool,
    pub layer_line_alignment_score: f32,
    pub score: f32,
}

pub struct OrientationResult {
    pub candidates: Vec<OrientationCandidate>,
    pub recommended: usize,
}

fn candidate_directions() -> Vec<Vec3> {
    let mut dirs = Vec::with_capacity(26);
    // 6 axis-aligned
    dirs.push(Vec3::X);
    dirs.push(-Vec3::X);
    dirs.push(Vec3::Y);
    dirs.push(-Vec3::Y);
    dirs.push(Vec3::Z);
    dirs.push(-Vec3::Z);
    // 12 edge midpoints
    for &(a, b, c) in &[
        (1.0_f32, 1.0, 0.0), (1.0, -1.0, 0.0), (-1.0, 1.0, 0.0), (-1.0, -1.0, 0.0),
        (1.0, 0.0, 1.0), (1.0, 0.0, -1.0), (-1.0, 0.0, 1.0), (-1.0, 0.0, -1.0),
        (0.0, 1.0, 1.0), (0.0, 1.0, -1.0), (0.0, -1.0, 1.0), (0.0, -1.0, -1.0),
    ] {
        dirs.push(Vec3::new(a, b, c).normalize());
    }
    // 8 vertex directions
    for &(a, b, c) in &[
        (1.0_f32, 1.0, 1.0), (1.0, 1.0, -1.0), (1.0, -1.0, 1.0), (1.0, -1.0, -1.0),
        (-1.0, 1.0, 1.0), (-1.0, 1.0, -1.0), (-1.0, -1.0, 1.0), (-1.0, -1.0, -1.0),
    ] {
        dirs.push(Vec3::new(a, b, c).normalize());
    }
    dirs
}

/// Compute rotation (axis, angle) from `from` to `to` (both normalized).
fn rotation_from_to(from: Vec3, to: Vec3) -> (Vec3, f32) {
    let dot = from.dot(to).clamp(-1.0, 1.0);
    let angle = dot.acos();
    if angle < 1e-6 {
        return (Vec3::Z, 0.0);
    }
    if (angle - std::f32::consts::PI).abs() < 1e-6 {
        // 180-degree rotation: pick a perpendicular axis
        let axis = if from.x.abs() < 0.9 {
            Vec3::X.cross(from).normalize()
        } else {
            Vec3::Y.cross(from).normalize()
        };
        return (axis, 180.0);
    }
    let axis = from.cross(to).normalize();
    (axis, angle.to_degrees())
}

/// Transform a single point by a rotation (axis-angle) applied to a center.
fn rotate_point(p: Vec3, axis: Vec3, angle_rad: f32) -> Vec3 {
    let cos_a = angle_rad.cos();
    let sin_a = angle_rad.sin();
    // Rodrigues' rotation formula
    p * cos_a + axis.cross(p) * sin_a + axis * axis.dot(p) * (1.0 - cos_a)
}

/// Check if the rotated AABB of the model fits within the build volume.
fn fits_in_build_volume(
    bounds_min: Vec3,
    bounds_max: Vec3,
    axis: Vec3,
    angle_rad: f32,
    build_volume: Vec3,
) -> bool {
    let corners = [
        Vec3::new(bounds_min.x, bounds_min.y, bounds_min.z),
        Vec3::new(bounds_max.x, bounds_min.y, bounds_min.z),
        Vec3::new(bounds_min.x, bounds_max.y, bounds_min.z),
        Vec3::new(bounds_max.x, bounds_max.y, bounds_min.z),
        Vec3::new(bounds_min.x, bounds_min.y, bounds_max.z),
        Vec3::new(bounds_max.x, bounds_min.y, bounds_max.z),
        Vec3::new(bounds_min.x, bounds_max.y, bounds_max.z),
        Vec3::new(bounds_max.x, bounds_max.y, bounds_max.z),
    ];
    let mut rmin = Vec3::splat(f32::MAX);
    let mut rmax = Vec3::splat(f32::MIN);
    for &c in &corners {
        let rc = rotate_point(c, axis, angle_rad);
        rmin = rmin.min(rc);
        rmax = rmax.max(rc);
    }
    let extents = rmax - rmin;
    extents.x <= build_volume.x && extents.y <= build_volume.y && extents.z <= build_volume.z
}

pub fn compute_orientation_advisor(
    surface_points: &[SurfacePoint],
    grid: &SdfGrid,
    settings: &PrintAnalysisSettings,
    fea_stress: Option<&[f32]>,
) -> OrientationResult {
    let span = (grid.bounds_max - grid.bounds_min).max_element().max(1e-6);
    let voxel_size = span / grid.resolution as f32;

    let total_surface_area = surface_points.len() as f32 * voxel_size * voxel_size;
    let part_volume = grid.data.iter().filter(|&&v| v < 0.0).count() as f32 * voxel_size.powi(3);

    let current_dir = settings.build_direction.normalize();
    let candidates_dirs = candidate_directions();

    let mut candidates: Vec<OrientationCandidate> = candidates_dirs
        .par_iter()
        .map(|&candidate_dir| {
            let mut candidate_settings = settings.clone();
            candidate_settings.build_direction = candidate_dir;

            let overhang = compute_overhang_analysis(surface_points, &candidate_settings, grid);
            let overhang_area = overhang.overhang_area_mm2;
            let support_volume = overhang.support_volume_estimate_mm3;

            let (rot_axis, rot_angle_deg) = rotation_from_to(current_dir, candidate_dir);
            let rot_angle_rad = rot_angle_deg.to_radians();

            let fits = fits_in_build_volume(
                grid.bounds_min,
                grid.bounds_max,
                rot_axis,
                rot_angle_rad,
                settings.build_volume,
            );

            let layer_line_score = if fea_stress.is_some() {
                // Placeholder: proper stress-alignment computation would go here
                0.0_f32
            } else {
                0.0_f32
            };

            let surface_ratio = if total_surface_area > 1e-6 {
                overhang_area / total_surface_area
            } else {
                0.0
            };
            let volume_ratio = if part_volume > 1e-6 {
                support_volume / part_volume
            } else {
                0.0
            };

            let mut score = 1.0
                - surface_ratio * 0.5
                - volume_ratio * 0.3
                + layer_line_score * 0.2;
            if !fits {
                score *= 0.1;
            }

            OrientationCandidate {
                build_direction: candidate_dir,
                rotation_axis: rot_axis,
                rotation_angle_deg: rot_angle_deg,
                overhang_area_mm2: overhang_area,
                support_volume_mm3: support_volume,
                fits_build_volume: fits,
                layer_line_alignment_score: layer_line_score,
                score,
            }
        })
        .collect();

    candidates.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
    candidates.truncate(6);

    OrientationResult {
        candidates,
        recommended: 0,
    }
}

// ---------------------------------------------------------------------------
// Feature detection
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub enum IssueType {
    ThinWall,
    TinyFeature,
    HighAspectRatio,
    BridgeSpan,
    SharpInternalCorner,
}

#[derive(Clone, Debug)]
pub enum IssueSeverity {
    Warning,
    Error,
}

pub struct PrintabilityIssue {
    pub issue_type: IssueType,
    pub location: Vec3,
    pub severity: IssueSeverity,
    pub description: String,
}

#[derive(Clone, Copy)]
pub enum IssueClass {
    None = 0,
    Warning = 1,
    Error = 2,
}

pub struct FeatureDetectionResult {
    pub issues: Vec<PrintabilityIssue>,
    pub issue_grid: Vec<f32>,
}

pub fn compute_feature_detection(
    surface_points: &[SurfacePoint],
    thickness_result: Option<&ThicknessResult>,
    settings: &PrintAnalysisSettings,
    grid: &SdfGrid,
) -> FeatureDetectionResult {
    let res = grid.resolution as usize;
    let total_voxels = res * res * res;
    let span = (grid.bounds_max - grid.bounds_min).max_element().max(1e-6);
    let voxel_size = span / grid.resolution as f32;

    let mut issues: Vec<PrintabilityIssue> = Vec::new();
    let mut issue_grid = vec![0.0_f32; total_voxels];

    // --- ThinWall detection from thickness_result ---
    if let Some(tr) = thickness_result {
        let tr_res = tr.resolution as usize;
        // Iterate over all voxels in the thickness result
        for idx in 0..tr_res * tr_res * tr_res {
            let t = tr.analysis_grid[idx];
            if t <= 0.0 {
                continue;
            }

            let ix = (idx % tr_res) as f32;
            let iy = ((idx / tr_res) % tr_res) as f32;
            let iz = (idx / (tr_res * tr_res)) as f32;
            let frac = Vec3::new(
                (ix + 0.5) / tr_res as f32,
                (iy + 0.5) / tr_res as f32,
                (iz + 0.5) / tr_res as f32,
            );
            let loc = tr.bounds_min + frac * (tr.bounds_max - tr.bounds_min);

            let (severity, severity_float) = if t < settings.min_wall_thickness {
                (IssueSeverity::Error, 1.0_f32)
            } else if t < settings.min_wall_thickness * 1.5 {
                (IssueSeverity::Warning, 0.5_f32)
            } else {
                continue;
            };

            // Map the thickness voxel into the analysis grid index (same resolution assumed)
            if idx < total_voxels {
                if issue_grid[idx] < severity_float {
                    issue_grid[idx] = severity_float;
                }
            }

            issues.push(PrintabilityIssue {
                issue_type: IssueType::ThinWall,
                location: loc,
                severity,
                description: format!(
                    "Wall thickness {:.2}mm below minimum {:.2}mm",
                    t, settings.min_wall_thickness
                ),
            });
        }
    }

    // --- TinyFeature detection from surface points ---
    for sp in surface_points {
        if sp.sdf_value.abs() < settings.min_feature_size {
            let g = &mut issue_grid[sp.grid_idx];
            if *g < 1.0 {
                *g = 1.0;
            }
            issues.push(PrintabilityIssue {
                issue_type: IssueType::TinyFeature,
                location: sp.position,
                severity: IssueSeverity::Error,
                description: format!(
                    "Feature size {:.2}mm below minimum {:.2}mm",
                    sp.sdf_value.abs(),
                    settings.min_feature_size
                ),
            });
        }
    }

    // --- HighAspectRatio detection ---
    // Identify thin surface points from thickness_result, then march along build_dir
    if let Some(tr) = thickness_result {
        let build_dir = settings.build_direction.normalize();
        let tr_res = tr.resolution as usize;

        // Collect voxel indices flagged as thin
        let thin_voxels: std::collections::HashSet<usize> = (0..tr_res * tr_res * tr_res)
            .filter(|&idx| {
                let t = tr.analysis_grid[idx];
                t > 0.0 && t < settings.min_wall_thickness
            })
            .collect();

        let max_march = (settings.max_aspect_ratio * 10.0) as usize;
        let aspect_threshold = settings.max_aspect_ratio * settings.min_wall_thickness;

        for sp in surface_points {
            // Only check surface points that are thin
            let sp_idx_in_tr = sp.grid_idx.min(tr_res * tr_res * tr_res - 1);
            if !thin_voxels.contains(&sp_idx_in_tr) {
                continue;
            }

            // March along build direction, count consecutive thin voxels
            let mut count = 0usize;
            let step = voxel_size;
            for k in 1..=max_march {
                let p = sp.position + build_dir * (k as f32 * step);
                // Map p back to a grid index in the thickness result
                let uvw = (p - tr.bounds_min) / (tr.bounds_max - tr.bounds_min);
                if uvw.x < 0.0 || uvw.y < 0.0 || uvw.z < 0.0
                    || uvw.x >= 1.0 || uvw.y >= 1.0 || uvw.z >= 1.0
                {
                    break;
                }
                let ci = (uvw.x * tr_res as f32) as usize;
                let cj = (uvw.y * tr_res as f32) as usize;
                let ck = (uvw.z * tr_res as f32) as usize;
                let tidx = ci + cj * tr_res + ck * tr_res * tr_res;
                if thin_voxels.contains(&tidx) {
                    count += 1;
                } else {
                    break;
                }
            }

            let column_height = count as f32 * voxel_size;
            if column_height > aspect_threshold {
                let g = &mut issue_grid[sp.grid_idx];
                if *g < 0.5 {
                    *g = 0.5;
                }
                issues.push(PrintabilityIssue {
                    issue_type: IssueType::HighAspectRatio,
                    location: sp.position,
                    severity: IssueSeverity::Warning,
                    description: format!(
                        "High aspect ratio column: height {:.1}mm, threshold {:.1}mm",
                        column_height, aspect_threshold
                    ),
                });
            }
        }
    }

    // --- BridgeSpan estimation ---
    // Look for surface points whose normal is close to anti-parallel to build_dir (bottom faces)
    // and estimate the connected overhang patch size.
    {
        let build_dir = settings.build_direction.normalize();
        let mut overhang_pts: Vec<&SurfacePoint> = surface_points
            .iter()
            .filter(|sp| {
                let dot = sp.normal.dot(-build_dir);
                dot > 0.7 // normal pointing downward (against build direction)
            })
            .collect();

        // Simple connected-component estimation by spatial proximity
        if !overhang_pts.is_empty() {
            // Sort by X then Y for a crude sweep; estimate max span via bounding box of clusters
            overhang_pts.sort_by(|a, b| {
                a.position.x.partial_cmp(&b.position.x).unwrap_or(std::cmp::Ordering::Equal)
            });

            // Compute overall bounding box of overhang points projected perpendicular to build_dir
            let mut min_p = Vec3::splat(f32::MAX);
            let mut max_p = Vec3::splat(f32::MIN);
            for sp in &overhang_pts {
                min_p = min_p.min(sp.position);
                max_p = max_p.max(sp.position);
            }
            // Project extent onto plane perpendicular to build_dir
            let extent = max_p - min_p;
            // Remove component along build_dir
            let along = build_dir * extent.dot(build_dir);
            let perp_extent = extent - along;
            let span_mm = perp_extent.length();

            if span_mm > 80.0 {
                issues.push(PrintabilityIssue {
                    issue_type: IssueType::BridgeSpan,
                    location: (min_p + max_p) * 0.5,
                    severity: IssueSeverity::Error,
                    description: format!(
                        "Bridge span {:.1}mm exceeds maximum 80mm without support",
                        span_mm
                    ),
                });
            } else if span_mm > 50.0 {
                issues.push(PrintabilityIssue {
                    issue_type: IssueType::BridgeSpan,
                    location: (min_p + max_p) * 0.5,
                    severity: IssueSeverity::Warning,
                    description: format!(
                        "Bridge span {:.1}mm may require support (>50mm)",
                        span_mm
                    ),
                });
            }
        }
    }

    // SharpInternalCorner: skipped (too expensive at voxel resolution)

    FeatureDetectionResult { issues, issue_grid }
}

// ---------------------------------------------------------------------------
// Top-level entry point
// ---------------------------------------------------------------------------

pub struct PrintAnalysisResult {
    pub surface_points: Vec<SurfacePoint>,
    pub overhang: OverhangResult,
    pub orientation: OrientationResult,
    pub features: FeatureDetectionResult,
}

pub fn compute_print_analysis(
    grid: &SdfGrid,
    settings: &PrintAnalysisSettings,
    thickness_result: Option<&ThicknessResult>,
    fea_stress: Option<&[f32]>,
) -> PrintAnalysisResult {
    // Step 1: extract surface points
    let surface_points = extract_surface_points(grid);

    // Step 2: run overhang, orientation, and feature detection in parallel
    // rayon::join only supports 2 closures; use nested join for 3 tasks.
    let (overhang, (orientation, features)) = rayon::join(
        || compute_overhang_analysis(&surface_points, settings, grid),
        || {
            rayon::join(
                || compute_orientation_advisor(&surface_points, grid, settings, fea_stress),
                || compute_feature_detection(&surface_points, thickness_result, settings, grid),
            )
        },
    );

    PrintAnalysisResult {
        surface_points,
        overhang,
        orientation,
        features,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a fake SdfGrid for a sphere: SDF = |p| - radius
    fn make_sphere_grid(resolution: u32, radius: f32) -> SdfGrid {
        let half = radius * 1.5;
        let bounds_min = Vec3::splat(-half);
        let bounds_max = Vec3::splat(half);
        let res = resolution as usize;
        let mut data = vec![0.0_f32; res * res * res];
        for iz in 0..res {
            for iy in 0..res {
                for ix in 0..res {
                    let fx = (ix as f32 + 0.5) / res as f32;
                    let fy = (iy as f32 + 0.5) / res as f32;
                    let fz = (iz as f32 + 0.5) / res as f32;
                    let p = bounds_min + Vec3::new(fx, fy, fz) * (bounds_max - bounds_min);
                    let sdf = p.length() - radius;
                    data[ix + iy * res + iz * res * res] = sdf;
                }
            }
        }
        SdfGrid { data, resolution, bounds_min, bounds_max }
    }

    /// Build a fake SdfGrid for a flat plate: SDF = |z| - half_thickness
    fn make_flat_plate_grid(resolution: u32, half_thick: f32, half_size: f32) -> SdfGrid {
        let bounds_min = Vec3::new(-half_size, -half_size, -half_thick * 3.0);
        let bounds_max = Vec3::new(half_size, half_size, half_thick * 3.0);
        let res = resolution as usize;
        let mut data = vec![0.0_f32; res * res * res];
        for iz in 0..res {
            for iy in 0..res {
                for ix in 0..res {
                    let fx = (ix as f32 + 0.5) / res as f32;
                    let fy = (iy as f32 + 0.5) / res as f32;
                    let fz = (iz as f32 + 0.5) / res as f32;
                    let p = bounds_min + Vec3::new(fx, fy, fz) * (bounds_max - bounds_min);
                    let sdf = p.z.abs() - half_thick;
                    data[ix + iy * res + iz * res * res] = sdf;
                }
            }
        }
        SdfGrid { data, resolution, bounds_min, bounds_max }
    }

    /// Build a fake SdfGrid for a vertical cylinder: SDF = sqrt(x^2+y^2) - radius (capped along z)
    fn make_cylinder_grid(resolution: u32, radius: f32, half_height: f32) -> SdfGrid {
        let margin = radius * 1.5;
        let bounds_min = Vec3::new(-margin, -margin, -half_height * 1.2);
        let bounds_max = Vec3::new(margin, margin, half_height * 1.2);
        let res = resolution as usize;
        let mut data = vec![0.0_f32; res * res * res];
        for iz in 0..res {
            for iy in 0..res {
                for ix in 0..res {
                    let fx = (ix as f32 + 0.5) / res as f32;
                    let fy = (iy as f32 + 0.5) / res as f32;
                    let fz = (iz as f32 + 0.5) / res as f32;
                    let p = bounds_min + Vec3::new(fx, fy, fz) * (bounds_max - bounds_min);
                    // SDF for finite cylinder
                    let d_radial = (p.x * p.x + p.y * p.y).sqrt() - radius;
                    let d_axial = p.z.abs() - half_height;
                    let sdf = if d_radial > 0.0 && d_axial > 0.0 {
                        (d_radial * d_radial + d_axial * d_axial).sqrt()
                    } else {
                        d_radial.max(d_axial)
                    };
                    data[ix + iy * res + iz * res * res] = sdf;
                }
            }
        }
        SdfGrid { data, resolution, bounds_min, bounds_max }
    }

    #[test]
    fn test_sphere_has_overhang() {
        // A sphere has bottom-hemisphere overhangs when built in Z direction.
        let grid = make_sphere_grid(32, 10.0);
        let settings = PrintAnalysisSettings {
            build_direction: Vec3::Z,
            overhang_threshold_deg: 45.0,
            ..Default::default()
        };
        let surface_pts = extract_surface_points(&grid);
        assert!(!surface_pts.is_empty(), "sphere should have surface points");

        let overhang = compute_overhang_analysis(&surface_pts, &settings, &grid);
        assert!(
            overhang.overhang_area_mm2 > 0.0,
            "sphere bottom hemisphere should register overhang area, got {}",
            overhang.overhang_area_mm2
        );
    }

    #[test]
    fn test_thin_wall_flagged() {
        use std::sync::Arc;

        // Create a minimal ThicknessResult with one thin voxel (thickness 0.5 < min_wall 1.2).
        let res = 4u32;
        let total = (res as usize).pow(3);
        let mut thickness_data = vec![0.0_f32; total];
        // Set voxel at index 0 to a thin wall value
        thickness_data[0] = 0.5;

        let bounds_min = Vec3::splat(-5.0);
        let bounds_max = Vec3::splat(5.0);

        let thickness_result = ThicknessResult {
            min_thickness: 0.5,
            min_location: Vec3::ZERO,
            analysis_grid: Arc::new(thickness_data),
            bounds_min,
            bounds_max,
            resolution: res,
        };

        // Build a matching SdfGrid (a small sphere so surface_points is non-empty)
        let grid = make_sphere_grid(res, 3.0);
        let settings = PrintAnalysisSettings {
            min_wall_thickness: 1.2,
            ..Default::default()
        };
        let surface_pts = extract_surface_points(&grid);

        let features = compute_feature_detection(&surface_pts, Some(&thickness_result), &settings, &grid);

        let thin_wall_errors: Vec<_> = features
            .issues
            .iter()
            .filter(|i| matches!(i.issue_type, IssueType::ThinWall))
            .filter(|i| matches!(i.severity, IssueSeverity::Error))
            .collect();

        assert!(
            !thin_wall_errors.is_empty(),
            "Expected at least one ThinWall Error, got {} issues total",
            features.issues.len()
        );
    }

    #[test]
    fn test_orientation_advisor_tall_cylinder() {
        // A tall thin cylinder (radius=2, height=30) should prefer Z-up orientation.
        let grid = make_cylinder_grid(24, 2.0, 15.0);
        let settings = PrintAnalysisSettings {
            build_volume: Vec3::new(220.0, 220.0, 250.0),
            build_direction: Vec3::X, // start with non-ideal orientation
            overhang_threshold_deg: 45.0,
            ..Default::default()
        };
        let surface_pts = extract_surface_points(&grid);
        assert!(!surface_pts.is_empty(), "cylinder should have surface points");

        let result = compute_orientation_advisor(&surface_pts, &grid, &settings, None);
        assert!(
            !result.candidates.is_empty(),
            "orientation advisor should produce candidates"
        );

        // Best candidate should be close to +Z or -Z
        let best = &result.candidates[0];
        let z_alignment = best.build_direction.dot(Vec3::Z).abs();
        assert!(
            z_alignment > 0.5,
            "Best orientation for tall cylinder should be close to Z axis, got {:?} with Z alignment {:.3}",
            best.build_direction,
            z_alignment
        );
    }
}
