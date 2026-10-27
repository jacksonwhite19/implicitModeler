use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::conditioning::{
    GeometryEdit, SdfBounds, audit_conditionable_geometry, condition_sdf_after_backend_edit,
    condition_sdf_for_backend, conditioned_kernel_ref,
};
use indexmap::IndexMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

fn usage() -> String {
    "usage: conditioning_edit_bench <script.rhai> [--audit-metadata] [--compare-direct-full] [--diagnose-quality] [--explain-edit] [old=>new ...]\n\
     example: conditioning_edit_bench aircraft.rhai \"let wing_sweep_deg = 15.0;=>let wing_sweep_deg = 20.0;\""
        .to_string()
}

fn apply_replacements(mut source: String, replacements: &[String]) -> Result<String, String> {
    if replacements.is_empty() {
        let defaults = [
            ("let wing_sweep_deg = 15.0;", "let wing_sweep_deg = 20.0;"),
            (
                "let wing_sweep_tan = 0.26794919243;",
                "let wing_sweep_tan = 0.36397023427;",
            ),
        ];
        for (old, new) in defaults {
            if source.contains(old) {
                source = source.replace(old, new);
            }
        }
        return Ok(source);
    }

    for replacement in replacements {
        let Some((old, new)) = replacement.split_once("=>") else {
            return Err(format!(
                "replacement must use old=>new syntax: {replacement}"
            ));
        };
        if !source.contains(old) {
            return Err(format!("replacement text not found: {old}"));
        }
        source = source.replace(old, new);
    }
    Ok(source)
}

fn eval_canonical(
    label: &str,
    source: &str,
    project_dir: Option<&Path>,
) -> Result<(Arc<dyn Sdf>, u128), String> {
    let dimensions = IndexMap::new();
    let start = Instant::now();
    let result = scripting::evaluate_script_full(
        source,
        None,
        None,
        None,
        None,
        &dimensions,
        project_dir,
        None,
        &[],
    )?;
    let elapsed_ms = start.elapsed().as_millis();
    println!("{label}_script_eval_ms={elapsed_ms}");
    Ok((result.canonical_sdf, elapsed_ms))
}

fn print_conditioning_summary(label: &str, sdf: &Arc<dyn Sdf>, elapsed_ms: u128) {
    let metadata = sdf.metadata();
    let cache = metadata.conditioned_cache.as_ref();
    let update_mode = conditioned_kernel_ref(sdf)
        .and_then(|kernel| kernel.conditioning().last_diagnostics())
        .map(|diagnostics| diagnostics.update_mode);
    let dirty_samples = conditioned_kernel_ref(sdf)
        .and_then(|kernel| kernel.conditioning().last_diagnostics())
        .map(|diagnostics| diagnostics.dirty_sample_count);
    let conditioned_samples = conditioned_kernel_ref(sdf)
        .and_then(|kernel| kernel.conditioning().last_diagnostics())
        .map(|diagnostics| diagnostics.conditioned_sample_count);
    let rejected = conditioned_kernel_ref(sdf)
        .and_then(|kernel| kernel.conditioning().last_rejected_diagnostics());

    if let Some(cache) = cache {
        println!(
            "{label}_conditioning_ms={elapsed_ms} state={:?} generation={} update_mode={:?} dirty_samples={} conditioned_samples={} blocks={} block_samples={} adaptive_regions={} adaptive_samples={} adaptive_stored_samples={} adaptive_min_spacing={:?} total_stored_samples={} estimated_cache_bytes={} confidence={:.3} publication_confidence={:.3} quality_confidence={:.3}",
            cache.state,
            cache.generation,
            update_mode,
            dirty_samples.unwrap_or(0),
            conditioned_samples.unwrap_or(0),
            cache.block_count,
            cache.block_sample_count,
            cache.adaptive_region_count,
            cache.adaptive_sample_count,
            cache.adaptive_stored_sample_count,
            cache.adaptive_min_grid_spacing,
            cache.total_stored_sample_count,
            cache.estimated_total_bytes,
            cache.confidence,
            cache.publication_confidence,
            cache.quality_confidence
        );
    } else {
        println!(
            "{label}_conditioning_ms={elapsed_ms} state=Unavailable update_mode={:?} dirty_samples={} conditioned_samples={}",
            update_mode,
            dirty_samples.unwrap_or(0),
            conditioned_samples.unwrap_or(0)
        );
    }
    if let Some(rejected) = rejected {
        println!(
            "{label}_rejected_local state={:?} confidence={:.3} grid_points={} dirty_samples={} conditioned_samples={} sign_mismatches={} surface_error={:.6} gradient_p95={:.6}",
            rejected.cache_state,
            rejected.confidence,
            rejected.grid_point_count,
            rejected.dirty_sample_count,
            rejected.conditioned_sample_count,
            rejected.sign_mismatch_count_near_interface,
            rejected.max_abs_error_near_interface,
            rejected.gradient_abs_error_p95
        );
    }
}

fn print_metadata_audit(label: &str, sdf: &Arc<dyn Sdf>) {
    let report = audit_conditionable_geometry(&sdf.metadata());
    println!("{label}_metadata_audit_issues={}", report.issues.len());
    for issue in report.issues.iter().take(20) {
        println!(
            "{label}_metadata_issue path={} node_kind={} issue={:?}",
            issue.path, issue.node_kind, issue.issue
        );
    }
    if report.issues.len() > 20 {
        println!(
            "{label}_metadata_issue_truncated={}",
            report.issues.len() - 20
        );
    }
}

fn print_cache_quality(label: &str, conditioned: &Arc<dyn Sdf>, canonical: &Arc<dyn Sdf>) {
    let Some(kernel) = conditioned_kernel_ref(conditioned) else {
        println!("{label}_quality unavailable=not_conditioned");
        return;
    };
    let quality = kernel
        .conditioning()
        .diagnose_cache_quality(canonical.as_ref());
    println!(
        "{label}_quality confidence={:.3} grid_points={} near_interface={} sign_mismatches={} surface_error={:.6} projection_error={:.6} distance_p95={:.6} gradient_p95={:.6}",
        quality.confidence,
        quality.grid_point_count,
        quality.near_interface_sample_count,
        quality.sign_mismatch_count_near_interface,
        quality.max_surface_residual,
        quality.max_projection_residual,
        quality.distance_abs_error_p95,
        quality.gradient_abs_error_p95
    );
}

fn bounds_volume(bounds: &SdfBounds) -> f32 {
    let size = bounds.size();
    (size.x.max(0.0)) * (size.y.max(0.0)) * (size.z.max(0.0))
}

fn print_edit_explanation(base_conditioned: &Arc<dyn Sdf>, edit_canonical: &Arc<dyn Sdf>) {
    let previous_metadata = base_conditioned
        .as_ref()
        .as_any()
        .downcast_ref::<implicit_cad::sdf::conditioning::ConditionedGeometryKernel>()
        .map(|kernel| kernel.canonical().metadata())
        .unwrap_or_else(|| base_conditioned.metadata());
    let next_metadata = edit_canonical.metadata();
    let cache = base_conditioned.metadata().conditioned_cache;
    let halo = cache
        .as_ref()
        .map(|cache| cache.interface_band.max(cache.grid_spacing) + cache.grid_spacing * 2.0)
        .unwrap_or(0.0);
    let Some(edit) = GeometryEdit::metadata_changed(
        Some("canonical_graph".to_string()),
        &previous_metadata,
        &next_metadata,
        halo,
    ) else {
        println!("edit_dirty_region=none");
        return;
    };

    let update_bounds = edit.dirty_region.update_bounds();
    let (domain_volume_fraction, affected_blocks, total_blocks) =
        conditioned_kernel_ref(base_conditioned)
            .map(|kernel| {
                let domain = kernel.conditioning().domain();
                let domain_volume = bounds_volume(domain).max(1.0e-6);
                let affected_blocks = kernel
                    .conditioning()
                    .block_indexes_intersecting(&update_bounds)
                    .len();
                (
                    bounds_volume(&update_bounds.clamped_to(domain)) / domain_volume,
                    affected_blocks,
                    kernel.conditioning().block_count(),
                )
            })
            .unwrap_or((0.0, 0, 0));

    println!(
        "edit_dirty_region source={:?} halo={:.3} recommended_spacing={:?} feature_ids={} bounds_min=({:.3},{:.3},{:.3}) bounds_max=({:.3},{:.3},{:.3}) update_size=({:.3},{:.3},{:.3}) domain_volume_fraction={:.3} affected_blocks={} total_blocks={}",
        edit.dirty_region.source,
        edit.dirty_region.halo,
        edit.dirty_region.recommended_grid_spacing,
        edit.dirty_region.feature_ids.len(),
        edit.dirty_region.bounds.min.x,
        edit.dirty_region.bounds.min.y,
        edit.dirty_region.bounds.min.z,
        edit.dirty_region.bounds.max.x,
        edit.dirty_region.bounds.max.y,
        edit.dirty_region.bounds.max.z,
        update_bounds.size().x,
        update_bounds.size().y,
        update_bounds.size().z,
        domain_volume_fraction,
        affected_blocks,
        total_blocks,
    );
}

fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let script_path = args.next().ok_or_else(usage).map(PathBuf::from)?;
    let mut audit_metadata = false;
    let mut compare_direct_full = false;
    let mut diagnose_quality = false;
    let mut explain_edit = false;
    let mut replacements = Vec::new();
    for arg in args {
        match arg.as_str() {
            "--audit-metadata" => audit_metadata = true,
            "--compare-direct-full" => compare_direct_full = true,
            "--diagnose-quality" => diagnose_quality = true,
            "--explain-edit" => explain_edit = true,
            _ => replacements.push(arg),
        }
    }
    let source = std::fs::read_to_string(&script_path)
        .map_err(|error| format!("failed to read {}: {error}", script_path.display()))?;
    let edited = apply_replacements(source.clone(), &replacements)?;
    let project_dir = script_path.parent();

    let (base_canonical, _) = eval_canonical("base", &source, project_dir)?;
    if audit_metadata {
        print_metadata_audit("base", &base_canonical);
    }
    let start = Instant::now();
    let base_conditioned = condition_sdf_for_backend(Arc::clone(&base_canonical));
    print_conditioning_summary("base_full", &base_conditioned, start.elapsed().as_millis());
    if diagnose_quality {
        print_cache_quality("base_full", &base_conditioned, &base_canonical);
    }

    let (edit_canonical, _) = eval_canonical("edit", &edited, project_dir)?;
    if audit_metadata {
        print_metadata_audit("edit", &edit_canonical);
    }
    if explain_edit {
        print_edit_explanation(&base_conditioned, &edit_canonical);
    }
    if compare_direct_full {
        let start = Instant::now();
        let edit_direct_conditioned = condition_sdf_for_backend(Arc::clone(&edit_canonical));
        print_conditioning_summary(
            "edit_direct_full",
            &edit_direct_conditioned,
            start.elapsed().as_millis(),
        );
        if diagnose_quality {
            print_cache_quality(
                "edit_direct_full",
                &edit_direct_conditioned,
                &edit_canonical,
            );
        }
    }
    let start = Instant::now();
    let edit_conditioned =
        condition_sdf_after_backend_edit(Some(&base_conditioned), Arc::clone(&edit_canonical));
    print_conditioning_summary("edit_local", &edit_conditioned, start.elapsed().as_millis());
    if diagnose_quality {
        print_cache_quality("edit_local", &edit_conditioned, &edit_canonical);
    }

    Ok(())
}
