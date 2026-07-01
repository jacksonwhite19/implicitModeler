use clap::Parser;
use glam::Vec3;
use implicit_cad::scripting;
use implicit_cad::sdf::conditioning::{
    condition_sdf_for_backend_with_sample_budget, conditioned_kernel_ref,
};
use indexmap::IndexMap;
use serde::Serialize;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::sync::Arc;

#[derive(Parser, Debug)]
#[command(name = "conditioning-section-compare")]
#[command(about = "Sample canonical and conditioned SDFs on one section plane")]
struct Args {
    #[arg(long)]
    script: PathBuf,
    #[arg(long, default_value = "yz")]
    plane: String,
    #[arg(long, default_value_t = 0.0)]
    coord: f32,
    #[arg(long)]
    amin: f32,
    #[arg(long)]
    amax: f32,
    #[arg(long)]
    bmin: f32,
    #[arg(long)]
    bmax: f32,
    #[arg(long, default_value_t = 401)]
    na: usize,
    #[arg(long, default_value_t = 241)]
    nb: usize,
    #[arg(long, default_value_t = 220_000)]
    sample_budget: usize,
    #[arg(long)]
    out_dir: PathBuf,
    #[arg(long, default_value = "section")]
    label: String,
}

#[derive(Clone, Debug, Default, Serialize)]
struct DeltaStats {
    sample_count: usize,
    finite_count: usize,
    near_interface_count: usize,
    cache_missing_count: usize,
    sign_mismatch_count: usize,
    sign_mismatch_near_interface_count: usize,
    abs_delta_p50: f32,
    abs_delta_p95: f32,
    abs_delta_max: f32,
    near_abs_delta_p50: f32,
    near_abs_delta_p95: f32,
    near_abs_delta_max: f32,
}

#[derive(Clone, Debug, Default, Serialize)]
struct PublishedGuardStats {
    sample_count: usize,
    finite_count: usize,
    near_interface_count: usize,
    published_canonical_equal_count: usize,
    published_canonical_equal_fraction: f32,
    inferred_guarded_sample_count: usize,
    inferred_guarded_fraction: f32,
    inferred_guarded_near_interface_count: usize,
    inferred_guarded_near_interface_fraction: f32,
    raw_cache_missing_published_canonical_count: usize,
}

#[derive(Clone, Debug, Default, Serialize)]
struct ContourDriftStats {
    canonical_point_count: usize,
    comparison_point_count: usize,
    canonical_to_comparison_p50: f32,
    canonical_to_comparison_p95: f32,
    canonical_to_comparison_max: f32,
    comparison_to_canonical_p50: f32,
    comparison_to_canonical_p95: f32,
    comparison_to_canonical_max: f32,
}

#[derive(Clone, Debug, Default, Serialize)]
struct DistanceBandResidualStats {
    band: f32,
    sample_count: usize,
    finite_count: usize,
    sign_mismatch_count: usize,
    signed_residual_p50: f32,
    signed_residual_p95: f32,
    signed_residual_max: f32,
    abs_distance_residual_p50: f32,
    abs_distance_residual_p95: f32,
    abs_distance_residual_max: f32,
}

#[derive(Clone, Debug, Serialize)]
struct WorstBandResidual {
    rank: usize,
    band: f32,
    feature_hint: String,
    a: f32,
    b: f32,
    x: f32,
    y: f32,
    z: f32,
    canonical_distance: f32,
    comparison_distance: f32,
    reference_surface_distance: f32,
    signed_residual: f32,
    abs_distance_residual: f32,
    sign_mismatch: bool,
}

#[derive(Clone, Debug, Serialize)]
struct FeatureBandResidualStats {
    band: f32,
    feature_hint: String,
    sample_count: usize,
    finite_count: usize,
    sign_mismatch_count: usize,
    abs_distance_residual_p50: f32,
    abs_distance_residual_p95: f32,
    abs_distance_residual_max: f32,
}

#[derive(Clone, Debug, Serialize)]
struct SectionCompareMetrics {
    label: String,
    plane: String,
    coord: f32,
    a_name: String,
    b_name: String,
    amin: f32,
    amax: f32,
    bmin: f32,
    bmax: f32,
    na: usize,
    nb: usize,
    cache_grid_spacing: Option<f32>,
    cache_interface_band: Option<f32>,
    cache_state: Option<String>,
    surface_refinement_block_count: Option<usize>,
    surface_refinement_cell_count: Option<usize>,
    surface_refinement_sample_bytes: Option<usize>,
    surface_refinement_distance_cell_count: Option<usize>,
    surface_refinement_distance_sample_bytes: Option<usize>,
    surface_refinement_point_count: Option<usize>,
    surface_refinement_point_bytes: Option<usize>,
    surface_refinement_bounds: Vec<[f32; 6]>,
    runtime_vs_canonical: DeltaStats,
    raw_cache_vs_canonical: DeltaStats,
    surface_refinement_vs_canonical: DeltaStats,
    reinitialized_band_vs_canonical: DeltaStats,
    published_guard: PublishedGuardStats,
    published_zero_contour_drift: ContourDriftStats,
    raw_storage_zero_contour_drift: ContourDriftStats,
    surface_refinement_zero_contour_drift: ContourDriftStats,
    raw_distance_band_residuals: Vec<DistanceBandResidualStats>,
    surface_refinement_distance_band_residuals: Vec<DistanceBandResidualStats>,
    reinitialized_band_distance_band_residuals: Vec<DistanceBandResidualStats>,
    reinitialized_band_direct_sdf_residuals: Vec<DistanceBandResidualStats>,
    reinitialized_band_worst_10mm_residuals: Vec<WorstBandResidual>,
    reinitialized_band_feature_10mm_residuals: Vec<FeatureBandResidualStats>,
    reinitialized_band_direct_worst_10mm_residuals: Vec<WorstBandResidual>,
    reinitialized_band_direct_feature_10mm_residuals: Vec<FeatureBandResidualStats>,
    canonical_csv: String,
    runtime_csv: String,
    raw_cache_csv: String,
    surface_refinement_csv: String,
    reinitialized_band_csv: String,
    runtime_delta_csv: String,
    raw_cache_delta_csv: String,
}

fn percentile(sorted: &[f32], percentile: f32) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let index = ((sorted.len() - 1) as f32 * percentile).round() as usize;
    sorted[index.min(sorted.len() - 1)]
}

fn sign_mismatch(a: f32, b: f32) -> bool {
    (a < 0.0 && b > 0.0) || (a > 0.0 && b < 0.0)
}

fn crosses_zero(a: f32, b: f32) -> bool {
    if !a.is_finite() || !b.is_finite() {
        return false;
    }
    a == 0.0 || b == 0.0 || sign_mismatch(a, b)
}

fn point_for_sample(plane: &str, coord: f32, a: f32, b: f32) -> Vec3 {
    match plane {
        "xz" => Vec3::new(a, coord, b),
        "yz" => Vec3::new(coord, a, b),
        "xy" => Vec3::new(a, b, coord),
        _ => unreachable!(),
    }
}

fn headers_for_plane(plane: &str) -> (&'static str, &'static str) {
    match plane {
        "xz" => ("x", "z"),
        "yz" => ("y", "z"),
        "xy" => ("x", "y"),
        _ => unreachable!(),
    }
}

fn csv_path(out_dir: &PathBuf, label: &str, suffix: &str) -> PathBuf {
    out_dir.join(format!("{label}_{suffix}.csv"))
}

fn bounds_array(bounds: &implicit_cad::sdf::conditioning::SdfBounds) -> [f32; 6] {
    [
        bounds.min.x,
        bounds.min.y,
        bounds.min.z,
        bounds.max.x,
        bounds.max.y,
        bounds.max.z,
    ]
}

fn write_grid_csv(
    path: &PathBuf,
    a_name: &str,
    b_name: &str,
    rows: &[(f32, f32, f32)],
) -> Result<(), String> {
    let mut csv = String::with_capacity(rows.len() * 28);
    csv.push_str(&format!("{a_name},{b_name},d\n"));
    for (a, b, d) in rows {
        csv.push_str(&format!("{a},{b},{d}\n"));
    }
    fs::write(path, csv).map_err(|error| format!("failed to write {}: {error}", path.display()))
}

fn stats_for_pairs(canonical: &[f32], comparison: &[Option<f32>], near_band: f32) -> DeltaStats {
    let mut all = Vec::new();
    let mut near = Vec::new();
    let mut stats = DeltaStats {
        sample_count: canonical.len(),
        ..DeltaStats::default()
    };

    for (canonical_d, comparison_d) in canonical.iter().copied().zip(comparison.iter().copied()) {
        let Some(comparison_d) = comparison_d else {
            stats.cache_missing_count += 1;
            continue;
        };
        if !canonical_d.is_finite() || !comparison_d.is_finite() {
            continue;
        }
        stats.finite_count += 1;
        if sign_mismatch(canonical_d, comparison_d) {
            stats.sign_mismatch_count += 1;
        }
        let delta = (comparison_d - canonical_d).abs();
        all.push(delta);
        if canonical_d.abs() <= near_band || comparison_d.abs() <= near_band {
            stats.near_interface_count += 1;
            near.push(delta);
            if sign_mismatch(canonical_d, comparison_d) {
                stats.sign_mismatch_near_interface_count += 1;
            }
        }
    }

    all.sort_by(|a, b| a.total_cmp(b));
    near.sort_by(|a, b| a.total_cmp(b));
    stats.abs_delta_p50 = percentile(&all, 0.50);
    stats.abs_delta_p95 = percentile(&all, 0.95);
    stats.abs_delta_max = all.last().copied().unwrap_or(0.0);
    stats.near_abs_delta_p50 = percentile(&near, 0.50);
    stats.near_abs_delta_p95 = percentile(&near, 0.95);
    stats.near_abs_delta_max = near.last().copied().unwrap_or(0.0);
    stats
}

fn guard_stats(
    canonical: &[f32],
    published: &[Option<f32>],
    raw_storage: &[Option<f32>],
    near_band: f32,
) -> PublishedGuardStats {
    let epsilon = 1.0e-5_f32;
    let mut stats = PublishedGuardStats {
        sample_count: canonical.len(),
        ..PublishedGuardStats::default()
    };

    for ((canonical_d, published_d), raw_d) in canonical
        .iter()
        .copied()
        .zip(published.iter().copied())
        .zip(raw_storage.iter().copied())
    {
        let Some(published_d) = published_d else {
            continue;
        };
        if !canonical_d.is_finite() || !published_d.is_finite() {
            continue;
        }
        stats.finite_count += 1;
        let near_interface = canonical_d.abs() <= near_band || published_d.abs() <= near_band;
        if near_interface {
            stats.near_interface_count += 1;
        }

        let published_matches_canonical = (published_d - canonical_d).abs() <= epsilon;
        if published_matches_canonical {
            stats.published_canonical_equal_count += 1;
        }

        match raw_d {
            Some(raw_d)
                if raw_d.is_finite()
                    && published_matches_canonical
                    && (raw_d - canonical_d).abs() > epsilon =>
            {
                stats.inferred_guarded_sample_count += 1;
                if near_interface {
                    stats.inferred_guarded_near_interface_count += 1;
                }
            }
            None if published_matches_canonical => {
                stats.raw_cache_missing_published_canonical_count += 1;
            }
            _ => {}
        }
    }

    let finite_count = stats.finite_count.max(1) as f32;
    stats.published_canonical_equal_fraction =
        stats.published_canonical_equal_count as f32 / finite_count;
    stats.inferred_guarded_fraction = stats.inferred_guarded_sample_count as f32 / finite_count;
    let near_count = stats.near_interface_count.max(1) as f32;
    stats.inferred_guarded_near_interface_fraction =
        stats.inferred_guarded_near_interface_count as f32 / near_count;
    stats
}

fn contour_drift_stats(
    canonical: &[f32],
    comparison: &[Option<f32>],
    na: usize,
    nb: usize,
    amin: f32,
    bmin: f32,
    da: f32,
    db: f32,
) -> ContourDriftStats {
    let canonical_points = zero_contour_points(canonical, na, nb, amin, bmin, da, db);
    let comparison_values: Vec<f32> = comparison
        .iter()
        .copied()
        .map(|value| value.unwrap_or(f32::NAN))
        .collect();
    let comparison_points = zero_contour_points(&comparison_values, na, nb, amin, bmin, da, db);

    let mut canonical_to_comparison = nearest_distances(&canonical_points, &comparison_points);
    let mut comparison_to_canonical = nearest_distances(&comparison_points, &canonical_points);
    canonical_to_comparison.sort_by(|a, b| a.total_cmp(b));
    comparison_to_canonical.sort_by(|a, b| a.total_cmp(b));

    ContourDriftStats {
        canonical_point_count: canonical_points.len(),
        comparison_point_count: comparison_points.len(),
        canonical_to_comparison_p50: percentile(&canonical_to_comparison, 0.50),
        canonical_to_comparison_p95: percentile(&canonical_to_comparison, 0.95),
        canonical_to_comparison_max: canonical_to_comparison.last().copied().unwrap_or(0.0),
        comparison_to_canonical_p50: percentile(&comparison_to_canonical, 0.50),
        comparison_to_canonical_p95: percentile(&comparison_to_canonical, 0.95),
        comparison_to_canonical_max: comparison_to_canonical.last().copied().unwrap_or(0.0),
    }
}

fn zero_contour_points(
    values: &[f32],
    na: usize,
    nb: usize,
    amin: f32,
    bmin: f32,
    da: f32,
    db: f32,
) -> Vec<[f32; 2]> {
    let mut points = Vec::new();
    let value_at = |ia: usize, ib: usize| values[ib * na + ia];
    for ib in 0..nb {
        let b = bmin + ib as f32 * db;
        for ia in 0..na.saturating_sub(1) {
            let a0 = amin + ia as f32 * da;
            let v0 = value_at(ia, ib);
            let v1 = value_at(ia + 1, ib);
            if let Some(t) = zero_crossing_t(v0, v1) {
                points.push([a0 + t * da, b]);
            }
        }
    }
    for ib in 0..nb.saturating_sub(1) {
        let b0 = bmin + ib as f32 * db;
        for ia in 0..na {
            let a = amin + ia as f32 * da;
            let v0 = value_at(ia, ib);
            let v1 = value_at(ia, ib + 1);
            if let Some(t) = zero_crossing_t(v0, v1) {
                points.push([a, b0 + t * db]);
            }
        }
    }
    points
}

fn zero_crossing_t(a: f32, b: f32) -> Option<f32> {
    if !crosses_zero(a, b) {
        return None;
    }
    let denom = a.abs() + b.abs();
    if denom <= f32::EPSILON {
        Some(0.5)
    } else {
        Some((a.abs() / denom).clamp(0.0, 1.0))
    }
}

fn nearest_distances(from: &[[f32; 2]], to: &[[f32; 2]]) -> Vec<f32> {
    if from.is_empty() || to.is_empty() {
        return Vec::new();
    }
    from.iter()
        .map(|from_point| {
            to.iter()
                .map(|to_point| {
                    let da = from_point[0] - to_point[0];
                    let db = from_point[1] - to_point[1];
                    (da * da + db * db).sqrt()
                })
                .fold(f32::INFINITY, |best, distance| best.min(distance))
        })
        .filter(|distance| distance.is_finite())
        .collect()
}

fn nearest_distance(point: [f32; 2], to: &[[f32; 2]]) -> Option<f32> {
    if to.is_empty() {
        return None;
    }
    to.iter()
        .map(|to_point| {
            let da = point[0] - to_point[0];
            let db = point[1] - to_point[1];
            (da * da + db * db).sqrt()
        })
        .filter(|distance| distance.is_finite())
        .min_by(|a, b| a.total_cmp(b))
}

fn distance_band_residual_stats(
    canonical: &[f32],
    comparison: &[Option<f32>],
    na: usize,
    nb: usize,
    amin: f32,
    bmin: f32,
    da: f32,
    db: f32,
) -> Vec<DistanceBandResidualStats> {
    let canonical_points = zero_contour_points(canonical, na, nb, amin, bmin, da, db);
    [1.0_f32, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|band| {
            let mut signed_residuals = Vec::new();
            let mut abs_distance_residuals = Vec::new();
            let mut stats = DistanceBandResidualStats {
                band,
                ..DistanceBandResidualStats::default()
            };
            for ib in 0..nb {
                let b = bmin + ib as f32 * db;
                for ia in 0..na {
                    let index = ib * na + ia;
                    let a = amin + ia as f32 * da;
                    let Some(reference_distance) = nearest_distance([a, b], &canonical_points)
                    else {
                        continue;
                    };
                    if reference_distance > band {
                        continue;
                    }
                    stats.sample_count += 1;
                    let canonical_d = canonical[index];
                    let Some(comparison_d) = comparison[index] else {
                        continue;
                    };
                    if !canonical_d.is_finite() || !comparison_d.is_finite() {
                        continue;
                    }
                    stats.finite_count += 1;
                    if sign_mismatch(canonical_d, comparison_d) {
                        stats.sign_mismatch_count += 1;
                    }
                    let expected_sign = if canonical_d < 0.0 { -1.0 } else { 1.0 };
                    let expected_signed_distance = expected_sign * reference_distance;
                    signed_residuals.push((comparison_d - expected_signed_distance).abs());
                    abs_distance_residuals.push((comparison_d.abs() - reference_distance).abs());
                }
            }

            signed_residuals.sort_by(|a, b| a.total_cmp(b));
            abs_distance_residuals.sort_by(|a, b| a.total_cmp(b));
            stats.signed_residual_p50 = percentile(&signed_residuals, 0.50);
            stats.signed_residual_p95 = percentile(&signed_residuals, 0.95);
            stats.signed_residual_max = signed_residuals.last().copied().unwrap_or(0.0);
            stats.abs_distance_residual_p50 = percentile(&abs_distance_residuals, 0.50);
            stats.abs_distance_residual_p95 = percentile(&abs_distance_residuals, 0.95);
            stats.abs_distance_residual_max = abs_distance_residuals.last().copied().unwrap_or(0.0);
            stats
        })
        .collect()
}

fn direct_sdf_band_residual_stats(
    canonical: &[f32],
    comparison: &[Option<f32>],
) -> Vec<DistanceBandResidualStats> {
    [1.0_f32, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|band| {
            let mut signed_residuals = Vec::new();
            let mut abs_distance_residuals = Vec::new();
            let mut stats = DistanceBandResidualStats {
                band,
                ..DistanceBandResidualStats::default()
            };
            for (&canonical_d, comparison_d) in canonical.iter().zip(comparison) {
                if !canonical_d.is_finite() || canonical_d.abs() > band {
                    continue;
                }
                stats.sample_count += 1;
                let Some(comparison_d) = comparison_d else {
                    continue;
                };
                if !comparison_d.is_finite() {
                    continue;
                }
                stats.finite_count += 1;
                if sign_mismatch(canonical_d, *comparison_d) {
                    stats.sign_mismatch_count += 1;
                }
                signed_residuals.push((*comparison_d - canonical_d).abs());
                abs_distance_residuals.push((comparison_d.abs() - canonical_d.abs()).abs());
            }

            signed_residuals.sort_by(|a, b| a.total_cmp(b));
            abs_distance_residuals.sort_by(|a, b| a.total_cmp(b));
            stats.signed_residual_p50 = percentile(&signed_residuals, 0.50);
            stats.signed_residual_p95 = percentile(&signed_residuals, 0.95);
            stats.signed_residual_max = signed_residuals.last().copied().unwrap_or(0.0);
            stats.abs_distance_residual_p50 = percentile(&abs_distance_residuals, 0.50);
            stats.abs_distance_residual_p95 = percentile(&abs_distance_residuals, 0.95);
            stats.abs_distance_residual_max = abs_distance_residuals.last().copied().unwrap_or(0.0);
            stats
        })
        .collect()
}

fn worst_distance_band_residuals(
    canonical: &[f32],
    comparison: &[Option<f32>],
    plane: &str,
    coord: f32,
    na: usize,
    nb: usize,
    amin: f32,
    bmin: f32,
    da: f32,
    db: f32,
    band: f32,
    limit: usize,
) -> (Vec<WorstBandResidual>, Vec<FeatureBandResidualStats>) {
    let canonical_points = zero_contour_points(canonical, na, nb, amin, bmin, da, db);
    let mut worst = Vec::new();
    let mut by_feature: HashMap<String, FeatureAccumulator> = HashMap::new();

    for ib in 0..nb {
        let b = bmin + ib as f32 * db;
        for ia in 0..na {
            let index = ib * na + ia;
            let a = amin + ia as f32 * da;
            let Some(reference_distance) = nearest_distance([a, b], &canonical_points) else {
                continue;
            };
            if reference_distance > band {
                continue;
            }
            let point = point_for_sample(plane, coord, a, b);
            let feature_hint = spatial_feature_hint(point).to_string();
            let accumulator = by_feature.entry(feature_hint.clone()).or_default();
            accumulator.sample_count += 1;

            let canonical_d = canonical[index];
            let Some(comparison_d) = comparison[index] else {
                continue;
            };
            if !canonical_d.is_finite() || !comparison_d.is_finite() {
                continue;
            }

            accumulator.finite_count += 1;
            let mismatch = sign_mismatch(canonical_d, comparison_d);
            if mismatch {
                accumulator.sign_mismatch_count += 1;
            }
            let expected_sign = if canonical_d < 0.0 { -1.0 } else { 1.0 };
            let expected_signed_distance = expected_sign * reference_distance;
            let signed_residual = (comparison_d - expected_signed_distance).abs();
            let abs_distance_residual = (comparison_d.abs() - reference_distance).abs();
            accumulator
                .abs_distance_residuals
                .push(abs_distance_residual);

            worst.push(WorstBandResidual {
                rank: 0,
                band,
                feature_hint,
                a,
                b,
                x: point.x,
                y: point.y,
                z: point.z,
                canonical_distance: canonical_d,
                comparison_distance: comparison_d,
                reference_surface_distance: reference_distance,
                signed_residual,
                abs_distance_residual,
                sign_mismatch: mismatch,
            });
        }
    }

    worst.sort_by(|a, b| b.abs_distance_residual.total_cmp(&a.abs_distance_residual));
    worst.truncate(limit);
    for (rank, residual) in worst.iter_mut().enumerate() {
        residual.rank = rank + 1;
    }

    let mut feature_stats: Vec<_> = by_feature
        .into_iter()
        .map(|(feature_hint, mut accumulator)| {
            accumulator
                .abs_distance_residuals
                .sort_by(|a, b| a.total_cmp(b));
            FeatureBandResidualStats {
                band,
                feature_hint,
                sample_count: accumulator.sample_count,
                finite_count: accumulator.finite_count,
                sign_mismatch_count: accumulator.sign_mismatch_count,
                abs_distance_residual_p50: percentile(&accumulator.abs_distance_residuals, 0.50),
                abs_distance_residual_p95: percentile(&accumulator.abs_distance_residuals, 0.95),
                abs_distance_residual_max: accumulator
                    .abs_distance_residuals
                    .last()
                    .copied()
                    .unwrap_or(0.0),
            }
        })
        .collect();
    feature_stats.sort_by(|a, b| {
        b.abs_distance_residual_p95
            .total_cmp(&a.abs_distance_residual_p95)
    });

    (worst, feature_stats)
}

fn worst_direct_sdf_band_residuals(
    canonical: &[f32],
    comparison: &[Option<f32>],
    plane: &str,
    coord: f32,
    na: usize,
    nb: usize,
    amin: f32,
    bmin: f32,
    da: f32,
    db: f32,
    band: f32,
    limit: usize,
) -> (Vec<WorstBandResidual>, Vec<FeatureBandResidualStats>) {
    let mut worst = Vec::new();
    let mut by_feature: HashMap<String, FeatureAccumulator> = HashMap::new();

    for ib in 0..nb {
        let b = bmin + ib as f32 * db;
        for ia in 0..na {
            let index = ib * na + ia;
            let a = amin + ia as f32 * da;
            let canonical_d = canonical[index];
            if !canonical_d.is_finite() || canonical_d.abs() > band {
                continue;
            }
            let point = point_for_sample(plane, coord, a, b);
            let feature_hint = spatial_feature_hint(point).to_string();
            let accumulator = by_feature.entry(feature_hint.clone()).or_default();
            accumulator.sample_count += 1;

            let Some(comparison_d) = comparison[index] else {
                continue;
            };
            if !comparison_d.is_finite() {
                continue;
            }

            accumulator.finite_count += 1;
            let mismatch = sign_mismatch(canonical_d, comparison_d);
            if mismatch {
                accumulator.sign_mismatch_count += 1;
            }
            let signed_residual = (comparison_d - canonical_d).abs();
            let abs_distance_residual = (comparison_d.abs() - canonical_d.abs()).abs();
            accumulator
                .abs_distance_residuals
                .push(abs_distance_residual);

            worst.push(WorstBandResidual {
                rank: 0,
                band,
                feature_hint,
                a,
                b,
                x: point.x,
                y: point.y,
                z: point.z,
                canonical_distance: canonical_d,
                comparison_distance: comparison_d,
                reference_surface_distance: canonical_d.abs(),
                signed_residual,
                abs_distance_residual,
                sign_mismatch: mismatch,
            });
        }
    }

    worst.sort_by(|a, b| b.abs_distance_residual.total_cmp(&a.abs_distance_residual));
    worst.truncate(limit);
    for (rank, residual) in worst.iter_mut().enumerate() {
        residual.rank = rank + 1;
    }

    let mut feature_stats: Vec<_> = by_feature
        .into_iter()
        .map(|(feature_hint, mut accumulator)| {
            accumulator
                .abs_distance_residuals
                .sort_by(|a, b| a.total_cmp(b));
            FeatureBandResidualStats {
                band,
                feature_hint,
                sample_count: accumulator.sample_count,
                finite_count: accumulator.finite_count,
                sign_mismatch_count: accumulator.sign_mismatch_count,
                abs_distance_residual_p50: percentile(&accumulator.abs_distance_residuals, 0.50),
                abs_distance_residual_p95: percentile(&accumulator.abs_distance_residuals, 0.95),
                abs_distance_residual_max: accumulator
                    .abs_distance_residuals
                    .last()
                    .copied()
                    .unwrap_or(0.0),
            }
        })
        .collect();
    feature_stats.sort_by(|a, b| {
        b.abs_distance_residual_p95
            .total_cmp(&a.abs_distance_residual_p95)
    });

    (worst, feature_stats)
}

#[derive(Clone, Debug, Default)]
struct FeatureAccumulator {
    sample_count: usize,
    finite_count: usize,
    sign_mismatch_count: usize,
    abs_distance_residuals: Vec<f32>,
}

fn spatial_feature_hint(point: Vec3) -> &'static str {
    let x = point.x;
    let y = point.y.abs();
    let z = point.z;

    if x < 90.0 {
        "nose"
    } else if x > 560.0 && z > 0.0 {
        "tail"
    } else if (170.0..=440.0).contains(&x) && (-55.0..=80.0).contains(&z) {
        if y <= 35.0 { "wing_root" } else { "wing" }
    } else if x > 430.0 && z > 20.0 {
        "tail_transition"
    } else if z < -45.0 {
        "lower_fuselage"
    } else {
        "fuselage"
    }
}

fn main() -> Result<(), String> {
    let args = Args::parse();
    let plane = args.plane.to_ascii_lowercase();
    if !matches!(plane.as_str(), "xz" | "yz" | "xy") {
        return Err("plane must be one of: xz, yz, xy".to_string());
    }
    if args.na < 2 || args.nb < 2 {
        return Err("na and nb must be at least 2".to_string());
    }
    fs::create_dir_all(&args.out_dir)
        .map_err(|error| format!("failed to create {}: {error}", args.out_dir.display()))?;

    let source = fs::read_to_string(&args.script)
        .map_err(|error| format!("failed to read {}: {error}", args.script.display()))?;
    let project_dir = args.script.parent();
    let result = scripting::evaluate_script_full(
        &source,
        None,
        None,
        None,
        None,
        &IndexMap::new(),
        project_dir,
        None,
        &[],
    )?;
    let canonical = Arc::clone(&result.canonical_sdf);
    let conditioned =
        condition_sdf_for_backend_with_sample_budget(Arc::clone(&canonical), args.sample_budget);
    let kernel = conditioned_kernel_ref(&conditioned);
    let storage = kernel.map(|kernel| kernel.storage_stats());
    let cache = conditioned.metadata().conditioned_cache;
    let near_band = cache
        .as_ref()
        .map(|cache| cache.interface_band.max(cache.grid_spacing))
        .unwrap_or(1.0);

    let da = (args.amax - args.amin) / (args.na as f32 - 1.0);
    let db = (args.bmax - args.bmin) / (args.nb as f32 - 1.0);
    let (a_name, b_name) = headers_for_plane(&plane);

    let mut canonical_values = Vec::with_capacity(args.na * args.nb);
    let mut runtime_values = Vec::with_capacity(args.na * args.nb);
    let mut raw_cache_values = Vec::with_capacity(args.na * args.nb);
    let mut surface_refinement_values = Vec::with_capacity(args.na * args.nb);
    let mut reinitialized_band_values = Vec::with_capacity(args.na * args.nb);
    let mut canonical_rows = Vec::with_capacity(args.na * args.nb);
    let mut runtime_rows = Vec::with_capacity(args.na * args.nb);
    let mut raw_cache_rows = Vec::with_capacity(args.na * args.nb);
    let mut surface_refinement_rows = Vec::with_capacity(args.na * args.nb);
    let mut reinitialized_band_rows = Vec::with_capacity(args.na * args.nb);
    let mut runtime_delta_rows = Vec::with_capacity(args.na * args.nb);
    let mut raw_cache_delta_rows = Vec::with_capacity(args.na * args.nb);

    for ib in 0..args.nb {
        let b = args.bmin + ib as f32 * db;
        for ia in 0..args.na {
            let a = args.amin + ia as f32 * da;
            let point = point_for_sample(&plane, args.coord, a, b);
            let canonical_d = canonical.distance(point);
            let runtime_d = conditioned.distance(point);
            let raw_cache_d = kernel.and_then(|kernel| kernel.raw_conditioned_distance(point));
            let surface_refinement_d =
                kernel.and_then(|kernel| kernel.raw_surface_refined_distance(point));
            let reinitialized_band_d =
                kernel.and_then(|kernel| kernel.raw_surface_refined_reinitialized_distance(point));

            canonical_values.push(canonical_d);
            runtime_values.push(Some(runtime_d));
            raw_cache_values.push(raw_cache_d);
            surface_refinement_values.push(surface_refinement_d);
            reinitialized_band_values.push(reinitialized_band_d);
            canonical_rows.push((a, b, canonical_d));
            runtime_rows.push((a, b, runtime_d));
            raw_cache_rows.push((a, b, raw_cache_d.unwrap_or(f32::NAN)));
            surface_refinement_rows.push((a, b, surface_refinement_d.unwrap_or(f32::NAN)));
            reinitialized_band_rows.push((a, b, reinitialized_band_d.unwrap_or(f32::NAN)));
            runtime_delta_rows.push((a, b, runtime_d - canonical_d));
            raw_cache_delta_rows.push((a, b, raw_cache_d.unwrap_or(f32::NAN) - canonical_d));
        }
    }

    let canonical_csv = csv_path(&args.out_dir, &args.label, "canonical");
    let runtime_csv = csv_path(&args.out_dir, &args.label, "runtime_conditioned");
    let raw_cache_csv = csv_path(&args.out_dir, &args.label, "raw_cache");
    let surface_refinement_csv = csv_path(&args.out_dir, &args.label, "surface_refinement");
    let reinitialized_band_csv = csv_path(&args.out_dir, &args.label, "reinitialized_band");
    let runtime_delta_csv = csv_path(&args.out_dir, &args.label, "runtime_delta");
    let raw_cache_delta_csv = csv_path(&args.out_dir, &args.label, "raw_cache_delta");
    write_grid_csv(&canonical_csv, a_name, b_name, &canonical_rows)?;
    write_grid_csv(&runtime_csv, a_name, b_name, &runtime_rows)?;
    write_grid_csv(&raw_cache_csv, a_name, b_name, &raw_cache_rows)?;
    write_grid_csv(
        &surface_refinement_csv,
        a_name,
        b_name,
        &surface_refinement_rows,
    )?;
    write_grid_csv(
        &reinitialized_band_csv,
        a_name,
        b_name,
        &reinitialized_band_rows,
    )?;
    write_grid_csv(&runtime_delta_csv, a_name, b_name, &runtime_delta_rows)?;
    write_grid_csv(&raw_cache_delta_csv, a_name, b_name, &raw_cache_delta_rows)?;

    let (reinitialized_band_worst_10mm_residuals, reinitialized_band_feature_10mm_residuals) =
        worst_distance_band_residuals(
            &canonical_values,
            &reinitialized_band_values,
            &plane,
            args.coord,
            args.na,
            args.nb,
            args.amin,
            args.bmin,
            da,
            db,
            10.0,
            20,
        );
    let (
        reinitialized_band_direct_worst_10mm_residuals,
        reinitialized_band_direct_feature_10mm_residuals,
    ) = worst_direct_sdf_band_residuals(
        &canonical_values,
        &reinitialized_band_values,
        &plane,
        args.coord,
        args.na,
        args.nb,
        args.amin,
        args.bmin,
        da,
        db,
        10.0,
        20,
    );

    let metrics = SectionCompareMetrics {
        label: args.label.clone(),
        plane: plane.clone(),
        coord: args.coord,
        a_name: a_name.to_string(),
        b_name: b_name.to_string(),
        amin: args.amin,
        amax: args.amax,
        bmin: args.bmin,
        bmax: args.bmax,
        na: args.na,
        nb: args.nb,
        cache_grid_spacing: cache.as_ref().map(|cache| cache.grid_spacing),
        cache_interface_band: cache.as_ref().map(|cache| cache.interface_band),
        cache_state: cache.as_ref().map(|cache| format!("{:?}", cache.state)),
        surface_refinement_block_count: storage
            .as_ref()
            .map(|storage| storage.surface_refinement_block_count),
        surface_refinement_cell_count: storage
            .as_ref()
            .map(|storage| storage.surface_refinement_cell_count),
        surface_refinement_sample_bytes: storage
            .as_ref()
            .map(|storage| storage.surface_refinement_sample_bytes),
        surface_refinement_distance_cell_count: storage
            .as_ref()
            .map(|storage| storage.surface_refinement_distance_cell_count),
        surface_refinement_distance_sample_bytes: storage
            .as_ref()
            .map(|storage| storage.surface_refinement_distance_sample_bytes),
        surface_refinement_point_count: storage
            .as_ref()
            .map(|storage| storage.surface_refinement_point_count),
        surface_refinement_point_bytes: storage
            .as_ref()
            .map(|storage| storage.surface_refinement_point_bytes),
        surface_refinement_bounds: kernel
            .map(|kernel| {
                kernel
                    .surface_refinement_bounds()
                    .iter()
                    .map(bounds_array)
                    .collect()
            })
            .unwrap_or_default(),
        runtime_vs_canonical: stats_for_pairs(&canonical_values, &runtime_values, near_band),
        raw_cache_vs_canonical: stats_for_pairs(&canonical_values, &raw_cache_values, near_band),
        surface_refinement_vs_canonical: stats_for_pairs(
            &canonical_values,
            &surface_refinement_values,
            near_band,
        ),
        reinitialized_band_vs_canonical: stats_for_pairs(
            &canonical_values,
            &reinitialized_band_values,
            near_band,
        ),
        published_guard: guard_stats(
            &canonical_values,
            &runtime_values,
            &raw_cache_values,
            near_band,
        ),
        published_zero_contour_drift: contour_drift_stats(
            &canonical_values,
            &runtime_values,
            args.na,
            args.nb,
            args.amin,
            args.bmin,
            da,
            db,
        ),
        raw_storage_zero_contour_drift: contour_drift_stats(
            &canonical_values,
            &raw_cache_values,
            args.na,
            args.nb,
            args.amin,
            args.bmin,
            da,
            db,
        ),
        surface_refinement_zero_contour_drift: contour_drift_stats(
            &canonical_values,
            &surface_refinement_values,
            args.na,
            args.nb,
            args.amin,
            args.bmin,
            da,
            db,
        ),
        raw_distance_band_residuals: distance_band_residual_stats(
            &canonical_values,
            &raw_cache_values,
            args.na,
            args.nb,
            args.amin,
            args.bmin,
            da,
            db,
        ),
        surface_refinement_distance_band_residuals: distance_band_residual_stats(
            &canonical_values,
            &surface_refinement_values,
            args.na,
            args.nb,
            args.amin,
            args.bmin,
            da,
            db,
        ),
        reinitialized_band_distance_band_residuals: distance_band_residual_stats(
            &canonical_values,
            &reinitialized_band_values,
            args.na,
            args.nb,
            args.amin,
            args.bmin,
            da,
            db,
        ),
        reinitialized_band_direct_sdf_residuals: direct_sdf_band_residual_stats(
            &canonical_values,
            &reinitialized_band_values,
        ),
        reinitialized_band_worst_10mm_residuals,
        reinitialized_band_feature_10mm_residuals,
        reinitialized_band_direct_worst_10mm_residuals,
        reinitialized_band_direct_feature_10mm_residuals,
        canonical_csv: canonical_csv.display().to_string(),
        runtime_csv: runtime_csv.display().to_string(),
        raw_cache_csv: raw_cache_csv.display().to_string(),
        surface_refinement_csv: surface_refinement_csv.display().to_string(),
        reinitialized_band_csv: reinitialized_band_csv.display().to_string(),
        runtime_delta_csv: runtime_delta_csv.display().to_string(),
        raw_cache_delta_csv: raw_cache_delta_csv.display().to_string(),
    };
    let metrics_path = args.out_dir.join(format!("{}_metrics.json", args.label));
    let metrics_json = serde_json::to_string_pretty(&metrics)
        .map_err(|error| format!("failed to serialize metrics: {error}"))?;
    fs::write(&metrics_path, metrics_json)
        .map_err(|error| format!("failed to write {}: {error}", metrics_path.display()))?;

    println!(
        "saved label={} plane={} coord={} metrics={} surface_refinement_blocks={} surface_refinement_cells={} surface_refinement_distance_cells={} surface_refinement_points={} surface_refinement_samples={} reinitialized_band_samples={} surface_refinement_zero_p95={} reinitialized_band5_signed_p95={} published_p95={} published_near_p95={} published_sign_mismatch_near={} published_guard_fraction={} raw_storage_p95={} raw_storage_near_p95={} raw_storage_sign_mismatch_near={} raw_zero_drift_p95={}",
        args.label,
        plane,
        args.coord,
        metrics_path.display(),
        metrics.surface_refinement_block_count.unwrap_or(0),
        metrics.surface_refinement_cell_count.unwrap_or(0),
        metrics.surface_refinement_distance_cell_count.unwrap_or(0),
        metrics.surface_refinement_point_count.unwrap_or(0),
        metrics.surface_refinement_vs_canonical.finite_count,
        metrics.reinitialized_band_vs_canonical.finite_count,
        metrics
            .surface_refinement_zero_contour_drift
            .canonical_to_comparison_p95,
        metrics
            .reinitialized_band_distance_band_residuals
            .iter()
            .find(|stats| (stats.band - 5.0).abs() <= f32::EPSILON)
            .map(|stats| stats.signed_residual_p95)
            .unwrap_or(0.0),
        metrics.runtime_vs_canonical.abs_delta_p95,
        metrics.runtime_vs_canonical.near_abs_delta_p95,
        metrics
            .runtime_vs_canonical
            .sign_mismatch_near_interface_count,
        metrics.published_guard.inferred_guarded_fraction,
        metrics.raw_cache_vs_canonical.abs_delta_p95,
        metrics.raw_cache_vs_canonical.near_abs_delta_p95,
        metrics
            .raw_cache_vs_canonical
            .sign_mismatch_near_interface_count,
        metrics
            .raw_storage_zero_contour_drift
            .canonical_to_comparison_p95,
    );
    Ok(())
}
