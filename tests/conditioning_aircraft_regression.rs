use glam::Vec3;
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::conditioning::{
    ConditionedCacheState, ConditioningUpdateMode, GeometryEdit, SdfBounds,
    audit_conditionable_geometry, condition_sdf_after_backend_edit_with_sample_budget,
    condition_sdf_for_backend_with_sample_budget, conditioned_kernel_ref,
    diagnose_conditioned_equivalence, summarize_conditioning_metadata,
};
use indexmap::IndexMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Instant;

const AIRCRAFT_SCRIPT: &str = "aircraft_optimizer_platform/software/sdf_generation_manual/curated_rhai/direct_sparse_oml_aircraft_no_inlet.rhai";
const REGRESSION_SAMPLE_BUDGET: usize = 220_000;
const MAX_LOCAL_EDIT_MS: u128 = 90_000;
const MAX_FULL_EDIT_MS: u128 = 300_000;
const MAX_LOCAL_AFFECTED_BLOCK_FRACTION: f32 = 0.20;
const MAX_LOCAL_CONDITIONED_SAMPLES: usize = 50_000;
const MAX_LOCAL_ADAPTIVE_SAMPLES: usize = 25_000;
const MAX_AIRCRAFT_CACHE_BYTES: usize = 384 * 1024 * 1024;
const MAX_RAW_ZERO_CONTOUR_P95_MM: f32 = 1.0;
const TRUSTED_REINITIALIZED_BAND_MM: f32 = 5.0;
const BROAD_REINITIALIZED_BAND_MM: f32 = 10.0;
const MIN_REINITIALIZED_BAND_COVERAGE: f32 = 0.99;
const MIN_BROAD_REINITIALIZED_BAND_COVERAGE: f32 = 0.995;
const MAX_REINITIALIZED_BAND_P95_MM: f32 = 0.5;
const MAX_BROAD_REINITIALIZED_BAND_P95_MM: f32 = 1.0;
const MAX_BAND_GRADIENT_ABS_ERROR_P95: f32 = 0.75;
const MAX_BAND_PROJECTION_RESIDUAL_P95_MM: f32 = 1.0;
const MIN_BAND_NORMAL_ALIGNMENT_P05: f32 = 0.5;
const MAX_LOCAL_VS_FULL_DISTANCE_P95_SPACING_MULTIPLIER: f32 = 2.0;
const MAX_LOCAL_VS_FULL_GRADIENT_P95_DELTA: f32 = 0.5;

static AIRCRAFT_CONDITIONING_TEST_LOCK: Mutex<()> = Mutex::new(());

fn aircraft_conditioning_test_guard() -> MutexGuard<'static, ()> {
    AIRCRAFT_CONDITIONING_TEST_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

#[derive(Clone, Copy)]
struct Replacement {
    old: &'static str,
    new: &'static str,
}

struct AircraftEditCase {
    name: &'static str,
    replacements: &'static [Replacement],
    expected_mode: Option<ConditioningUpdateMode>,
    max_elapsed_ms: u128,
    require_partial_blocks: bool,
    max_affected_block_fraction: Option<f32>,
    max_conditioned_sample_count: Option<usize>,
    max_adaptive_sample_count: Option<usize>,
    max_estimated_cache_bytes: Option<usize>,
}

struct ConditionedEditResult {
    elapsed_ms: u128,
    update_mode: ConditioningUpdateMode,
    cache_state: ConditionedCacheState,
    invalidated_block_count: usize,
    total_block_count: usize,
    conditioned_sample_count: usize,
    sign_mismatches: usize,
    publication_confidence: f32,
    quality_confidence: f32,
    adaptive_sample_count: usize,
    estimated_cache_bytes: usize,
}

fn aircraft_script_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(AIRCRAFT_SCRIPT)
}

fn read_aircraft_script() -> (String, PathBuf) {
    let path = aircraft_script_path();
    let source = std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()));
    (source, path)
}

fn apply_replacements(mut source: String, replacements: &[Replacement]) -> String {
    for replacement in replacements {
        assert!(
            source.contains(replacement.old),
            "replacement anchor not found: {}",
            replacement.old
        );
        source = source.replace(replacement.old, replacement.new);
    }
    source
}

fn eval_canonical(source: &str, project_dir: Option<&Path>) -> Arc<dyn Sdf> {
    scripting::evaluate_script_full(
        source,
        None,
        None,
        None,
        None,
        &IndexMap::new(),
        project_dir,
        None,
        &[],
    )
    .expect("aircraft script should evaluate")
    .canonical_sdf
}

fn assert_clean_metadata(label: &str, sdf: &Arc<dyn Sdf>) {
    let report = audit_conditionable_geometry(&sdf.metadata());
    assert!(
        report.is_clean(),
        "{label} metadata audit failed: {:?}",
        report.issues
    );
}

fn bounds_volume(bounds: &SdfBounds) -> f32 {
    let size = bounds.size();
    size.x.max(0.0) * size.y.max(0.0) * size.z.max(0.0)
}

fn affected_block_count(
    previous: &Arc<dyn Sdf>,
    next: &Arc<dyn Sdf>,
) -> Option<(usize, usize, f32)> {
    let kernel = conditioned_kernel_ref(previous)?;
    let previous_metadata = kernel.canonical().metadata();
    let next_metadata = next.metadata();
    let cache = previous.metadata().conditioned_cache?;
    let halo = cache.interface_band.max(cache.grid_spacing) + cache.grid_spacing * 2.0;
    let edit = GeometryEdit::metadata_changed(
        Some("canonical_graph".to_string()),
        &previous_metadata,
        &next_metadata,
        halo,
    )?;
    let update_bounds = edit.dirty_region.update_bounds();
    let domain = kernel.conditioning().domain();
    let domain_volume = bounds_volume(domain).max(1.0e-6);
    let affected = kernel
        .conditioning()
        .block_indexes_intersecting(&update_bounds)
        .len();
    Some((
        affected,
        kernel.conditioning().block_count(),
        bounds_volume(&update_bounds.clamped_to(domain)) / domain_volume,
    ))
}

fn run_edit_case(
    previous_conditioned: &Arc<dyn Sdf>,
    next_canonical: Arc<dyn Sdf>,
) -> ConditionedEditResult {
    let predicted_blocks = affected_block_count(previous_conditioned, &next_canonical);
    let start = Instant::now();
    let conditioned = condition_sdf_after_backend_edit_with_sample_budget(
        Some(previous_conditioned),
        Arc::clone(&next_canonical),
        REGRESSION_SAMPLE_BUDGET,
    );
    let elapsed_ms = start.elapsed().as_millis();
    let kernel = conditioned_kernel_ref(&conditioned)
        .expect("conditioned edit should return a conditioned geometry kernel");
    let diagnostics = kernel
        .conditioning()
        .last_diagnostics()
        .expect("conditioned edit should record diagnostics");
    let cache = conditioned
        .metadata()
        .conditioned_cache
        .expect("conditioned edit should publish cache metadata");
    let (invalidated_block_count, total_block_count, _domain_fraction) =
        predicted_blocks.unwrap_or((0, kernel.conditioning().block_count(), 0.0));

    ConditionedEditResult {
        elapsed_ms,
        update_mode: diagnostics.update_mode,
        cache_state: diagnostics.cache_state,
        invalidated_block_count,
        total_block_count,
        conditioned_sample_count: diagnostics.conditioned_sample_count,
        sign_mismatches: diagnostics.sign_mismatch_count_near_interface,
        publication_confidence: diagnostics.publication_confidence,
        quality_confidence: diagnostics.quality_confidence,
        adaptive_sample_count: cache.adaptive_sample_count,
        estimated_cache_bytes: cache.estimated_total_bytes,
    }
}

fn assert_publishable(case: &AircraftEditCase, result: &ConditionedEditResult) {
    assert_ne!(
        result.cache_state,
        ConditionedCacheState::Rejected,
        "{} should not publish a rejected cache",
        case.name
    );
    assert_eq!(
        result.sign_mismatches, 0,
        "{} should not introduce near-interface sign mismatches",
        case.name
    );
    assert!(
        result.publication_confidence >= 1.0,
        "{} should have publication confidence, got {:.3}",
        case.name,
        result.publication_confidence
    );
    assert!(
        result.elapsed_ms <= case.max_elapsed_ms,
        "{} took {}ms, over the {}ms regression budget",
        case.name,
        result.elapsed_ms,
        case.max_elapsed_ms
    );
    assert!(
        result.update_mode == ConditioningUpdateMode::FullRebuild
            || result.conditioned_sample_count > 0,
        "{} should validate at least one conditioned sample for local updates",
        case.name
    );
    assert!(
        result.quality_confidence.is_finite(),
        "{} should publish finite quality confidence",
        case.name
    );
    if let Some(max_fraction) = case.max_affected_block_fraction {
        let affected_fraction =
            result.invalidated_block_count as f32 / result.total_block_count.max(1) as f32;
        assert!(
            affected_fraction <= max_fraction,
            "{} affected {:.3} of blocks, over the {:.3} regression gate",
            case.name,
            affected_fraction,
            max_fraction
        );
    }
    if let Some(max_conditioned_samples) = case.max_conditioned_sample_count {
        assert!(
            result.conditioned_sample_count <= max_conditioned_samples,
            "{} conditioned {} samples, over the {} regression gate",
            case.name,
            result.conditioned_sample_count,
            max_conditioned_samples
        );
    }
    if let Some(max_adaptive_samples) = case.max_adaptive_sample_count {
        assert!(
            result.adaptive_sample_count <= max_adaptive_samples,
            "{} allocated {} adaptive samples, over the {} regression gate",
            case.name,
            result.adaptive_sample_count,
            max_adaptive_samples
        );
    }
    if let Some(max_cache_bytes) = case.max_estimated_cache_bytes {
        assert!(
            result.estimated_cache_bytes <= max_cache_bytes,
            "{} estimated cache is {} bytes, over the {} byte regression gate",
            case.name,
            result.estimated_cache_bytes,
            max_cache_bytes
        );
    }
}

fn section_point(plane: &str, coord: f32, a: f32, b: f32) -> Vec3 {
    match plane {
        "xz" => Vec3::new(a, coord, b),
        "yz" => Vec3::new(coord, a, b),
        "xy" => Vec3::new(a, b, coord),
        _ => panic!("unsupported section plane: {plane}"),
    }
}

fn crosses_zero(a: f32, b: f32) -> bool {
    if !a.is_finite() || !b.is_finite() {
        return false;
    }
    a == 0.0 || b == 0.0 || (a < 0.0 && b > 0.0) || (a > 0.0 && b < 0.0)
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
            if let Some(t) = zero_crossing_t(value_at(ia, ib), value_at(ia + 1, ib)) {
                points.push([a0 + t * da, b]);
            }
        }
    }
    for ib in 0..nb.saturating_sub(1) {
        let b0 = bmin + ib as f32 * db;
        for ia in 0..na {
            let a = amin + ia as f32 * da;
            if let Some(t) = zero_crossing_t(value_at(ia, ib), value_at(ia, ib + 1)) {
                points.push([a, b0 + t * db]);
            }
        }
    }
    points
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
    to.iter()
        .map(|to_point| {
            let da = point[0] - to_point[0];
            let db = point[1] - to_point[1];
            (da * da + db * db).sqrt()
        })
        .filter(|distance| distance.is_finite())
        .min_by(|a, b| a.total_cmp(b))
}

fn percentile(sorted: &[f32], percentile: f32) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let index = ((sorted.len() - 1) as f32 * percentile).round() as usize;
    sorted[index.min(sorted.len() - 1)]
}

#[allow(clippy::too_many_arguments)]
fn assert_reinitialized_band_p95_below(
    label: &str,
    canonical: &Arc<dyn Sdf>,
    conditioned: &Arc<dyn Sdf>,
    plane: &str,
    coord: f32,
    amin: f32,
    amax: f32,
    bmin: f32,
    bmax: f32,
    na: usize,
    nb: usize,
    band: f32,
    min_coverage: f32,
    threshold: f32,
) {
    let kernel = conditioned_kernel_ref(conditioned)
        .expect("conditioned aircraft should expose a conditioned kernel");
    let da = (amax - amin) / (na as f32 - 1.0);
    let db = (bmax - bmin) / (nb as f32 - 1.0);
    let mut canonical_values = Vec::with_capacity(na * nb);
    for ib in 0..nb {
        let b = bmin + ib as f32 * db;
        for ia in 0..na {
            let a = amin + ia as f32 * da;
            canonical_values.push(canonical.distance(section_point(plane, coord, a, b)));
        }
    }
    let canonical_points = zero_contour_points(&canonical_values, na, nb, amin, bmin, da, db);
    assert!(
        !canonical_points.is_empty(),
        "{label} should produce a canonical zero contour"
    );

    let mut residuals = Vec::new();
    let mut sample_count = 0_usize;
    let mut finite_count = 0_usize;
    let mut sign_mismatch_count = 0_usize;
    for ib in 0..nb {
        let b = bmin + ib as f32 * db;
        for ia in 0..na {
            let a = amin + ia as f32 * da;
            let Some(reference_distance) = nearest_distance([a, b], &canonical_points) else {
                continue;
            };
            if reference_distance > band {
                continue;
            }
            sample_count += 1;
            let index = ib * na + ia;
            let canonical_d = canonical_values[index];
            let Some(conditioned_d) = kernel
                .raw_surface_refined_reinitialized_distance(section_point(plane, coord, a, b))
            else {
                continue;
            };
            if !canonical_d.is_finite() || !conditioned_d.is_finite() {
                continue;
            }
            finite_count += 1;
            let expected_sign = if canonical_d < 0.0 { -1.0 } else { 1.0 };
            if canonical_d.abs() > 1.0e-4 && conditioned_d.signum() != expected_sign {
                sign_mismatch_count += 1;
            }
            residuals.push((conditioned_d - expected_sign * reference_distance).abs());
        }
    }
    assert!(sample_count > 0, "{label} should sample the trusted band");
    let coverage = finite_count as f32 / sample_count as f32;
    residuals.sort_by(|a, b| a.total_cmp(b));
    let p95 = percentile(&residuals, 0.95);
    eprintln!(
        "{label}: reinitialized_band={band:.1}mm finite={finite_count}/{sample_count} coverage={coverage:.3} sign_mismatches={sign_mismatch_count} p95={p95:.3}mm"
    );
    assert_eq!(
        sign_mismatch_count, 0,
        "{label} reinitialized band should preserve canonical inside/outside sign"
    );
    assert!(
        coverage >= min_coverage,
        "{label} reinitialized band coverage {coverage:.3} below {min_coverage:.3}"
    );
    assert!(
        p95 <= threshold,
        "{label} reinitialized band p95 {p95:.3}mm exceeds {threshold:.3}mm"
    );
}

#[allow(clippy::too_many_arguments)]
fn assert_reinitialized_band_query_quality(
    label: &str,
    canonical: &Arc<dyn Sdf>,
    conditioned: &Arc<dyn Sdf>,
    plane: &str,
    coord: f32,
    amin: f32,
    amax: f32,
    bmin: f32,
    bmax: f32,
    na: usize,
    nb: usize,
    band: f32,
    stride: usize,
) {
    let kernel = conditioned_kernel_ref(conditioned)
        .expect("conditioned aircraft should expose a conditioned kernel");
    let da = (amax - amin) / (na as f32 - 1.0);
    let db = (bmax - bmin) / (nb as f32 - 1.0);
    let mut canonical_values = Vec::with_capacity(na * nb);
    for ib in 0..nb {
        let b = bmin + ib as f32 * db;
        for ia in 0..na {
            let a = amin + ia as f32 * da;
            canonical_values.push(canonical.distance(section_point(plane, coord, a, b)));
        }
    }
    let canonical_points = zero_contour_points(&canonical_values, na, nb, amin, bmin, da, db);
    assert!(
        !canonical_points.is_empty(),
        "{label} should produce a canonical zero contour"
    );

    let mut gradient_errors = Vec::new();
    let mut projection_residuals = Vec::new();
    let mut normal_alignments = Vec::new();
    let mut sample_count = 0_usize;
    let mut finite_count = 0_usize;
    let mut projection_count = 0_usize;
    let stride = stride.max(1);
    for ib in (0..nb).step_by(stride) {
        let b = bmin + ib as f32 * db;
        for ia in (0..na).step_by(stride) {
            let a = amin + ia as f32 * da;
            let Some(reference_distance) = nearest_distance([a, b], &canonical_points) else {
                continue;
            };
            if reference_distance <= 0.5 || reference_distance > band {
                continue;
            }
            sample_count += 1;
            let point = section_point(plane, coord, a, b);
            let Some(conditioned_d) = kernel.raw_surface_refined_reinitialized_distance(point)
            else {
                continue;
            };
            if !conditioned_d.is_finite() {
                continue;
            }
            finite_count += 1;
            let gradient = kernel.gradient(point, 1.5);
            let gradient_length = gradient.length();
            if gradient_length.is_finite() && gradient_length > 1.0e-6 {
                gradient_errors.push((gradient_length - 1.0).abs());
                if let Some(projected) = kernel.project_to_surface(point, 0.25, 16) {
                    let residual = canonical.distance(projected).abs();
                    if residual.is_finite() {
                        projection_residuals.push(residual);
                        projection_count += 1;
                    }
                    let outward = gradient / gradient_length;
                    let radial = (point - projected).normalize_or_zero();
                    if radial.length_squared() > 1.0e-6 {
                        let expected_sign = if canonical_values[ib * na + ia] < 0.0 {
                            -1.0
                        } else {
                            1.0
                        };
                        normal_alignments.push(outward.dot(radial * expected_sign));
                    }
                }
            }
        }
    }

    assert!(sample_count > 0, "{label} should sample band query quality");
    let coverage = finite_count as f32 / sample_count as f32;
    gradient_errors.sort_by(|a, b| a.total_cmp(b));
    projection_residuals.sort_by(|a, b| a.total_cmp(b));
    normal_alignments.sort_by(|a, b| a.total_cmp(b));
    let gradient_p95 = percentile(&gradient_errors, 0.95);
    let projection_p95 = percentile(&projection_residuals, 0.95);
    let normal_p05 = percentile(&normal_alignments, 0.05);
    eprintln!(
        "{label}: band_query_quality band={band:.1}mm finite={finite_count}/{sample_count} coverage={coverage:.3} gradient_error_p95={gradient_p95:.3} projection_count={projection_count} projection_residual_p95={projection_p95:.3}mm normal_alignment_p05={normal_p05:.3}"
    );
    assert!(
        coverage >= MIN_REINITIALIZED_BAND_COVERAGE,
        "{label} band query coverage {coverage:.3} below {MIN_REINITIALIZED_BAND_COVERAGE:.3}"
    );
    assert!(
        gradient_p95 <= MAX_BAND_GRADIENT_ABS_ERROR_P95,
        "{label} band gradient p95 error {gradient_p95:.3} exceeds {MAX_BAND_GRADIENT_ABS_ERROR_P95:.3}"
    );
    assert!(
        projection_count > 0 && projection_p95 <= MAX_BAND_PROJECTION_RESIDUAL_P95_MM,
        "{label} projection residual p95 {projection_p95:.3}mm exceeds {MAX_BAND_PROJECTION_RESIDUAL_P95_MM:.3}mm"
    );
    assert!(
        normal_p05 >= MIN_BAND_NORMAL_ALIGNMENT_P05,
        "{label} normal alignment p05 {normal_p05:.3} below {MIN_BAND_NORMAL_ALIGNMENT_P05:.3}"
    );
}

#[allow(clippy::too_many_arguments)]
fn assert_raw_zero_contour_p95_below(
    label: &str,
    canonical: &Arc<dyn Sdf>,
    conditioned: &Arc<dyn Sdf>,
    plane: &str,
    coord: f32,
    amin: f32,
    amax: f32,
    bmin: f32,
    bmax: f32,
    na: usize,
    nb: usize,
    threshold: f32,
) {
    let kernel = conditioned_kernel_ref(conditioned)
        .expect("conditioned aircraft should expose a conditioned kernel");
    let cache = conditioned
        .metadata()
        .conditioned_cache
        .expect("conditioned aircraft should publish cache metadata");
    assert!(
        cache.surface_refinement_cell_count > 0,
        "{label} should build a derived surface-refinement cache"
    );

    let da = (amax - amin) / (na as f32 - 1.0);
    let db = (bmax - bmin) / (nb as f32 - 1.0);
    let mut canonical_values = Vec::with_capacity(na * nb);
    let mut raw_values = Vec::with_capacity(na * nb);
    let mut surface_refinement_hits = 0_usize;

    for ib in 0..nb {
        let b = bmin + ib as f32 * db;
        for ia in 0..na {
            let a = amin + ia as f32 * da;
            let point = section_point(plane, coord, a, b);
            canonical_values.push(canonical.distance(point));
            raw_values.push(kernel.raw_conditioned_distance(point).unwrap_or(f32::NAN));
            if kernel.raw_surface_refined_distance(point).is_some() {
                surface_refinement_hits += 1;
            }
        }
    }

    assert!(
        surface_refinement_hits > 0,
        "{label} should exercise the derived surface-refinement cache"
    );
    let canonical_points = zero_contour_points(&canonical_values, na, nb, amin, bmin, da, db);
    let raw_points = zero_contour_points(&raw_values, na, nb, amin, bmin, da, db);
    assert!(
        !canonical_points.is_empty() && !raw_points.is_empty(),
        "{label} should produce comparable zero contours"
    );
    let mut distances = nearest_distances(&canonical_points, &raw_points);
    distances.sort_by(|a, b| a.total_cmp(b));
    let p95 = percentile(&distances, 0.95);
    eprintln!(
        "{label}: raw_zero_contour_p95={p95:.3}mm surface_refinement_hits={surface_refinement_hits}"
    );
    assert!(
        p95 <= threshold,
        "{label} raw zero-contour p95 {p95:.3}mm exceeds {threshold:.3}mm"
    );
}

#[test]
fn curated_aircraft_conditioning_regression_suite() {
    let _guard = aircraft_conditioning_test_guard();
    let (source, script_path) = read_aircraft_script();
    let project_dir = script_path.parent();
    let base_canonical = eval_canonical(&source, project_dir);
    assert_clean_metadata("base aircraft", &base_canonical);
    let coverage = summarize_conditioning_metadata(&base_canonical.metadata());
    assert!(
        coverage.is_conditioning_ready(),
        "base aircraft metadata should be conditionable: {coverage:?}"
    );
    assert!(
        coverage.has_locality_hints(),
        "base aircraft metadata should expose locality hints: {coverage:?}"
    );
    assert!(
        coverage.total_node_count >= 10,
        "base aircraft should expose a real metadata tree: {coverage:?}"
    );
    assert!(
        coverage.feature_region_count >= 4,
        "base aircraft should expose feature regions for adaptive conditioning: {coverage:?}"
    );
    assert!(
        coverage.child_interface_node_count >= 2,
        "base aircraft should mark smooth interface dependencies: {coverage:?}"
    );
    assert!(
        coverage.nodes_with_parameter_fingerprint >= 8,
        "base aircraft should fingerprint parameterized nodes: {coverage:?}"
    );

    let base_conditioned = condition_sdf_for_backend_with_sample_budget(
        Arc::clone(&base_canonical),
        REGRESSION_SAMPLE_BUDGET,
    );
    let base_kernel =
        conditioned_kernel_ref(&base_conditioned).expect("base aircraft should be conditionable");
    let base_cache = base_conditioned
        .metadata()
        .conditioned_cache
        .expect("base aircraft should publish cache metadata");
    assert_eq!(base_cache.state, ConditionedCacheState::Ready);
    assert!(base_kernel.conditioning().block_count() > 1);
    assert!(base_cache.publication_confidence >= 1.0);
    assert!(base_cache.block_sample_count > 0);
    assert!(base_cache.total_stored_sample_count >= base_cache.block_sample_count);
    assert!(base_cache.total_stored_sample_bytes >= base_cache.block_sample_bytes);
    assert!(base_cache.estimated_total_bytes >= base_cache.total_stored_sample_bytes);
    assert!(base_cache.surface_refinement_cell_count > 0);
    assert_raw_zero_contour_p95_below(
        "base_aircraft_centerline",
        &base_canonical,
        &base_conditioned,
        "xz",
        0.0,
        -60.0,
        780.0,
        -120.0,
        210.0,
        561,
        281,
        MAX_RAW_ZERO_CONTOUR_P95_MM,
    );
    assert_reinitialized_band_p95_below(
        "base_aircraft_centerline",
        &base_canonical,
        &base_conditioned,
        "xz",
        0.0,
        -60.0,
        780.0,
        -120.0,
        210.0,
        561,
        281,
        TRUSTED_REINITIALIZED_BAND_MM,
        MIN_REINITIALIZED_BAND_COVERAGE,
        MAX_REINITIALIZED_BAND_P95_MM,
    );
    assert_reinitialized_band_p95_below(
        "base_aircraft_centerline_broad",
        &base_canonical,
        &base_conditioned,
        "xz",
        0.0,
        -60.0,
        780.0,
        -120.0,
        210.0,
        561,
        281,
        BROAD_REINITIALIZED_BAND_MM,
        MIN_BROAD_REINITIALIZED_BAND_COVERAGE,
        MAX_BROAD_REINITIALIZED_BAND_P95_MM,
    );
    assert_reinitialized_band_query_quality(
        "base_aircraft_centerline",
        &base_canonical,
        &base_conditioned,
        "xz",
        0.0,
        -60.0,
        780.0,
        -120.0,
        210.0,
        141,
        71,
        TRUSTED_REINITIALIZED_BAND_MM,
        2,
    );

    let cases = [
        AircraftEditCase {
            name: "wing_root_blend_local",
            replacements: &[Replacement {
                old: "let wing_root_blend_k = 16.0;",
                new: "let wing_root_blend_k = 17.0;",
            }],
            expected_mode: Some(ConditioningUpdateMode::LocalIncremental),
            max_elapsed_ms: MAX_LOCAL_EDIT_MS,
            require_partial_blocks: true,
            max_affected_block_fraction: Some(MAX_LOCAL_AFFECTED_BLOCK_FRACTION),
            max_conditioned_sample_count: Some(MAX_LOCAL_CONDITIONED_SAMPLES),
            max_adaptive_sample_count: Some(MAX_LOCAL_ADAPTIVE_SAMPLES),
            max_estimated_cache_bytes: Some(MAX_AIRCRAFT_CACHE_BYTES),
        },
        AircraftEditCase {
            name: "tail_blend_local",
            replacements: &[Replacement {
                old: "let tail_blend_k = 10.0;",
                new: "let tail_blend_k = 11.0;",
            }],
            expected_mode: Some(ConditioningUpdateMode::LocalIncremental),
            max_elapsed_ms: MAX_LOCAL_EDIT_MS,
            require_partial_blocks: true,
            max_affected_block_fraction: Some(MAX_LOCAL_AFFECTED_BLOCK_FRACTION),
            max_conditioned_sample_count: Some(MAX_LOCAL_CONDITIONED_SAMPLES),
            max_adaptive_sample_count: Some(MAX_LOCAL_ADAPTIVE_SAMPLES),
            max_estimated_cache_bytes: Some(MAX_AIRCRAFT_CACHE_BYTES),
        },
        AircraftEditCase {
            name: "wing_sweep_broad",
            replacements: &[
                Replacement {
                    old: "let wing_sweep_deg = 15.0;",
                    new: "let wing_sweep_deg = 20.0;",
                },
                Replacement {
                    old: "let wing_sweep_tan = 0.26794919243;",
                    new: "let wing_sweep_tan = 0.36397023427;",
                },
            ],
            expected_mode: None,
            max_elapsed_ms: MAX_FULL_EDIT_MS,
            require_partial_blocks: false,
            max_affected_block_fraction: None,
            max_conditioned_sample_count: None,
            max_adaptive_sample_count: None,
            max_estimated_cache_bytes: Some(MAX_AIRCRAFT_CACHE_BYTES),
        },
        AircraftEditCase {
            name: "wing_tip_chord",
            replacements: &[Replacement {
                old: "let wing_tip_chord = 65.0;",
                new: "let wing_tip_chord = 72.0;",
            }],
            expected_mode: None,
            max_elapsed_ms: MAX_FULL_EDIT_MS,
            require_partial_blocks: false,
            max_affected_block_fraction: None,
            max_conditioned_sample_count: None,
            max_adaptive_sample_count: None,
            max_estimated_cache_bytes: Some(MAX_AIRCRAFT_CACHE_BYTES),
        },
        AircraftEditCase {
            name: "fuselage_width_broad",
            replacements: &[Replacement {
                old: "let fuselage_width = 175.0;",
                new: "let fuselage_width = 185.0;",
            }],
            expected_mode: None,
            max_elapsed_ms: MAX_FULL_EDIT_MS,
            require_partial_blocks: false,
            max_affected_block_fraction: None,
            max_conditioned_sample_count: None,
            max_adaptive_sample_count: None,
            max_estimated_cache_bytes: Some(MAX_AIRCRAFT_CACHE_BYTES),
        },
    ];

    for case in cases {
        let edited_source = apply_replacements(source.clone(), case.replacements);
        let edit_canonical = eval_canonical(&edited_source, project_dir);
        assert_clean_metadata(case.name, &edit_canonical);
        let result = run_edit_case(&base_conditioned, edit_canonical);
        assert_publishable(&case, &result);

        if let Some(expected_mode) = case.expected_mode {
            assert_eq!(
                result.update_mode, expected_mode,
                "{} should use the expected update strategy",
                case.name
            );
        }
        if case.require_partial_blocks {
            assert!(
                result.invalidated_block_count > 0
                    && result.invalidated_block_count < result.total_block_count,
                "{} should only invalidate a subset of blocks; affected={} total={}",
                case.name,
                result.invalidated_block_count,
                result.total_block_count
            );
        }

        eprintln!(
            "{}: mode={:?} elapsed_ms={} affected_blocks={}/{} adaptive_samples={} estimated_cache_bytes={}",
            case.name,
            result.update_mode,
            result.elapsed_ms,
            result.invalidated_block_count,
            result.total_block_count,
            result.adaptive_sample_count,
            result.estimated_cache_bytes
        );
    }
}

#[test]
fn curated_aircraft_conditioning_query_api_smoke_test() {
    let _guard = aircraft_conditioning_test_guard();
    let (source, script_path) = read_aircraft_script();
    let canonical = eval_canonical(&source, script_path.parent());
    let conditioned = condition_sdf_for_backend_with_sample_budget(
        Arc::clone(&canonical),
        REGRESSION_SAMPLE_BUDGET,
    );
    let kernel = conditioned_kernel_ref(&conditioned)
        .expect("base aircraft should be queryable as a conditioned geometry kernel");
    let metadata = conditioned.metadata();
    let bounds = metadata
        .support_bounds
        .as_ref()
        .expect("conditioned aircraft should retain support bounds");
    let center = (bounds.min + bounds.max) * 0.5;
    let grid_spacing = metadata
        .conditioned_cache
        .as_ref()
        .expect("conditioned aircraft should publish cache metadata")
        .grid_spacing;
    let mut near_surface = center;
    let mut near_surface_abs_distance = f32::INFINITY;
    let steps = 18;
    for ix in 0..=steps {
        for iy in 0..=steps {
            for iz in 0..=steps {
                let t = Vec3::new(
                    ix as f32 / steps as f32,
                    iy as f32 / steps as f32,
                    iz as f32 / steps as f32,
                );
                let point = bounds.min + bounds.size() * t;
                let distance = kernel.distance(point).abs();
                if distance.is_finite() && distance < near_surface_abs_distance {
                    near_surface = point;
                    near_surface_abs_distance = distance;
                }
            }
        }
    }

    let normal = kernel.surface_normal(near_surface, grid_spacing);
    assert!(normal.is_finite());
    assert!(
        normal.length_squared() > 0.25,
        "near-surface query should produce a usable normal"
    );

    let query_point = near_surface + normal * grid_spacing * 3.0;
    let projected = kernel
        .project_to_surface(query_point, grid_spacing, 48)
        .expect("project_to_surface should converge from a nearby surface offset");
    let closest = kernel
        .closest_surface_point(query_point, grid_spacing, 48)
        .expect("closest_surface_point should converge from a nearby surface offset");
    let hit = kernel
        .ray_intersection(
            projected + normal * grid_spacing * 4.0,
            -normal,
            grid_spacing * 12.0,
            grid_spacing,
            128,
        )
        .or_else(|| {
            kernel.ray_intersection(
                projected - normal * grid_spacing * 4.0,
                normal,
                grid_spacing * 12.0,
                grid_spacing,
                128,
            )
        });

    assert!(normal.is_finite());
    assert!(projected.is_finite());
    assert!(closest.is_finite());
    assert!(
        kernel.distance(projected).abs() <= grid_spacing * 2.0,
        "projected point should land close to the conditioned surface"
    );
    assert!(
        hit.is_some(),
        "conditioned query API should report a local ray hit"
    );
}

#[test]
fn curated_aircraft_local_blend_matches_direct_full_conditioning() {
    let _guard = aircraft_conditioning_test_guard();
    let (source, script_path) = read_aircraft_script();
    let project_dir = script_path.parent();
    let base_canonical = eval_canonical(&source, project_dir);
    let base_conditioned = condition_sdf_for_backend_with_sample_budget(
        Arc::clone(&base_canonical),
        REGRESSION_SAMPLE_BUDGET,
    );
    let edited_source = apply_replacements(
        source,
        &[Replacement {
            old: "let wing_root_blend_k = 16.0;",
            new: "let wing_root_blend_k = 17.0;",
        }],
    );
    let edit_canonical = eval_canonical(&edited_source, project_dir);
    let local_conditioned = condition_sdf_after_backend_edit_with_sample_budget(
        Some(&base_conditioned),
        Arc::clone(&edit_canonical),
        REGRESSION_SAMPLE_BUDGET,
    );
    let direct_full_conditioned =
        condition_sdf_for_backend_with_sample_budget(edit_canonical, REGRESSION_SAMPLE_BUDGET);
    let local_kernel =
        conditioned_kernel_ref(&local_conditioned).expect("local update should be conditioned");
    let full_kernel = conditioned_kernel_ref(&direct_full_conditioned)
        .expect("direct full rebuild should be conditioned");
    let diagnostics = diagnose_conditioned_equivalence(local_kernel, full_kernel, None, None);

    eprintln!("local_vs_full_blend_equivalence={diagnostics:?}");
    assert!(
        diagnostics.compared_sample_count > 0,
        "equivalence diagnostic should compare samples"
    );
    assert_eq!(
        diagnostics.sign_mismatch_count_near_interface, 0,
        "local conditioning should not change near-interface signs relative to full conditioning"
    );
    assert!(
        diagnostics.accepted(
            diagnostics.spacing * MAX_LOCAL_VS_FULL_DISTANCE_P95_SPACING_MULTIPLIER,
            MAX_LOCAL_VS_FULL_GRADIENT_P95_DELTA,
        ),
        "local conditioning should match full conditioning within diagnostic tolerances: {diagnostics:?}"
    );
}
