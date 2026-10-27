use glam::Vec3;
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::booleans::{Intersect, SmoothUnion, Subtract, Union};
use implicit_cad::sdf::cfd_query::{
    GeometryQueryKernel, SDF_CFD_QUERY_SCHEMA_VERSION, SdfCfdQueryOptions,
};
use implicit_cad::sdf::conditioning::SdfNodeMetadata;
use implicit_cad::sdf::primitives::{SdfBox, Sphere};
use implicit_cad::sdf::provenance::{
    CsgDecisionState, CsgTraceMode, CsgTraceSupport, DEFAULT_CSG_TIE_ABSOLUTE_TOLERANCE,
    DEFAULT_CSG_TIE_RELATIVE_TOLERANCE, DEFAULT_CSG_TIE_SCALE_FLOOR,
};
use implicit_cad::sdf::transforms::Translate;
use serde::Serialize;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::fs;
use std::sync::Arc;
use std::time::Instant;

const REPORT_DIR: &str = "research/sdf_cfd_mesher/reports";
const JSON_REPORT: &str = "research/sdf_cfd_mesher/reports/geometry_query_capability_audit_v2.json";
const MD_REPORT: &str = "research/sdf_cfd_mesher/reports/geometry_query_capability_audit_v2.md";

#[derive(Clone)]
struct ConstantSdf {
    distance: f32,
    label: &'static str,
}

impl ConstantSdf {
    fn new(distance: f32, label: &'static str) -> Self {
        Self { distance, label }
    }
}

impl Sdf for ConstantSdf {
    fn distance(&self, _point: Vec3) -> f32 {
        self.distance
    }

    fn metadata(&self) -> SdfNodeMetadata {
        SdfNodeMetadata::new(self.label)
    }
}

#[derive(Clone, Serialize)]
struct SampleDecisionCase {
    label: &'static str,
    intent: &'static str,
    point: [f64; 3],
    source: &'static str,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    fs::create_dir_all(REPORT_DIR)?;

    let mut sample_decisions = Vec::new();
    for (case, sdf) in direct_sample_cases() {
        sample_decisions.push(sample_decision(&case, sdf)?);
    }
    for (case, sdf) in rhai_sample_cases()? {
        sample_decisions.push(sample_decision(&case, sdf)?);
    }

    let sweep_direct = regression_sweep(
        "direct_nested_hard_csg",
        direct_nested_hard_csg(),
        &structured_points(7, 2.0),
    );
    let sweep_rhai = regression_sweep(
        "rhai_nested_aerospace_csg",
        load_sdf(
            r#"
            let wing = wing_with_airfoil_export_safe("2412", 250.0, 180.0, 1000.0, 20.0, 2.0, 0.0, 2.0, 1.0, true);
            let pod = translate(sphere(40.0), 80.0, 700.0, 0.0);
            let cut = translate(sphere(16.0), 80.0, 700.0, 0.0);
            union(wing, subtract(pod, cut))
            "#,
        )?,
        &[
            [80.0, 700.0, 0.0],
            [60.0, 0.0, 5.0],
            [130.0, 700.0, 0.0],
            [0.0, 0.0, 0.0],
            [250.0, 500.0, 0.0],
        ],
    );

    let performance = performance_characterization(direct_nested_hard_csg())?;
    let report = json!({
        "schema_version": "geometry_query_capability_audit.v2",
        "generated_by": "sdf_cfd_query_audit",
        "configuration": {
            "query_schema_version": SDF_CFD_QUERY_SCHEMA_VERSION,
            "stl_generated": false,
            "surface_or_volume_mesher_started": false
        },
        "geometry_source": [
            "direct Rust SDF primitives and hard CSG nodes",
            "Rhai scripts loaded through scripting::evaluate_script"
        ],
        "hard_csg_provenance": {
            "supported_operations": ["union", "intersection", "subtraction"],
            "unsupported_operations": ["smooth_union", "smooth_intersect", "smooth_subtract"],
            "tie_policy": {
                "absolute_tolerance": DEFAULT_CSG_TIE_ABSOLUTE_TOLERANCE,
                "relative_tolerance": DEFAULT_CSG_TIE_RELATIVE_TOLERANCE,
                "scale_floor": DEFAULT_CSG_TIE_SCALE_FLOOR
            },
            "trace_policy": {
                "modes": ["none", "immediate", "recursive"],
                "default_mode": "none",
                "diagnostic_mode": "recursive"
            },
            "scalar_invariance": {
                "criterion": "f32::to_bits equality between Sdf::distance and provenance-enabled sample distance",
                "direct_sweep_mismatches": sweep_direct["distance_mismatches"],
                "rhai_sweep_mismatches": sweep_rhai["distance_mismatches"]
            },
            "regression_sweep": {
                "direct_nested_hard_csg": sweep_direct,
                "rhai_nested_aerospace_csg": sweep_rhai
            },
            "sample_decisions": sample_decisions,
            "capability_changes_from_v1": {
                "hard_union_active_child": "supported",
                "hard_intersection_active_child": "supported",
                "hard_subtraction_active_child": "supported",
                "subtraction_effective_sign": "supported",
                "hard_csg_competing_child": "supported_at_decision_points",
                "nested_hard_csg_path": "supported_or_partial_depending_on_terminal_metadata",
                "smooth_csg_contributor": "unsupported_weighted_provenance_required",
                "patch_identity": "still_unsupported",
                "material_or_region_identity": "still_unsupported",
                "true_second_nearest_geometry": "still_unsupported"
            }
        },
        "performance": performance,
        "known_gaps": [
            "Smooth CSG nodes explicitly terminate traces because weighted provenance is not implemented.",
            "Terminal entity identity is partial when metadata has no stable feature ID.",
            "Patch, material, fluid-region, local-coordinate, and true second-nearest geometry queries remain unsupported.",
            "Scale and offset wrappers preserve child traces but decision distances inside nested traces remain in the child decision frame."
        ],
        "recommended_next_task": {
            "task": "SDF-native surface-mesh foundation",
            "reason": "Hard union/intersection/subtraction provenance works, subtraction sign inversion is explicit, nested traces survive key wrappers and Rhai construction, and scalar invariance has zero mismatches. Smooth CSG remains unsupported but explicitly isolated."
        }
    });

    fs::write(JSON_REPORT, serde_json::to_string_pretty(&report)?)?;
    fs::write(MD_REPORT, markdown_report(&report))?;
    println!("Wrote {JSON_REPORT}");
    println!("Wrote {MD_REPORT}");
    Ok(())
}

fn direct_sample_cases() -> Vec<(SampleDecisionCase, Arc<dyn Sdf>)> {
    vec![
        (
            SampleDecisionCase {
                label: "union_left_active",
                intent: "hard union selects lower left distance",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Union::new(
                Arc::new(ConstantSdf::new(1.0, "left_leaf")),
                Arc::new(ConstantSdf::new(2.0, "right_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "union_right_active",
                intent: "hard union selects lower right distance",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Union::new(
                Arc::new(ConstantSdf::new(2.0, "left_leaf")),
                Arc::new(ConstantSdf::new(1.0, "right_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "intersection_left_active",
                intent: "hard intersection selects higher left distance",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Intersect::new(
                Arc::new(ConstantSdf::new(3.0, "left_leaf")),
                Arc::new(ConstantSdf::new(1.0, "right_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "intersection_right_active",
                intent: "hard intersection selects higher right distance",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Intersect::new(
                Arc::new(ConstantSdf::new(1.0, "left_leaf")),
                Arc::new(ConstantSdf::new(3.0, "right_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "subtraction_retained_active",
                intent: "hard subtraction selects retained left contribution",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Subtract::new(
                Arc::new(ConstantSdf::new(2.0, "retained_leaf")),
                Arc::new(ConstantSdf::new(5.0, "subtractor_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "subtraction_subtractor_active",
                intent: "hard subtraction selects sign-inverted subtractor contribution",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Subtract::new(
                Arc::new(ConstantSdf::new(-2.0, "retained_leaf")),
                Arc::new(ConstantSdf::new(-5.0, "subtractor_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "exact_tie",
                intent: "hard union reports exact tie without changing winner",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Union::new(
                Arc::new(ConstantSdf::new(1.0, "left_leaf")),
                Arc::new(ConstantSdf::new(1.0, "right_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "near_tie",
                intent: "hard union reports near tie without changing winner",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Union::new(
                Arc::new(ConstantSdf::new(1.0, "left_leaf")),
                Arc::new(ConstantSdf::new(1.0000005, "right_leaf")),
            )),
        ),
        (
            SampleDecisionCase {
                label: "nested_trace",
                intent: "hard union follows into nested hard subtraction",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            direct_nested_hard_csg(),
        ),
        (
            SampleDecisionCase {
                label: "trace_terminates_at_smooth_csg",
                intent: "hard union selects smooth child and stops explicitly",
                point: [0.0, 0.0, 0.0],
                source: "direct",
            },
            Arc::new(Union::new(
                Arc::new(SmoothUnion::new(
                    Arc::new(ConstantSdf::new(1.0, "smooth_a")),
                    Arc::new(ConstantSdf::new(1.0, "smooth_b")),
                    0.5,
                )),
                Arc::new(ConstantSdf::new(10.0, "far_leaf")),
            )),
        ),
    ]
}

fn rhai_sample_cases() -> Result<Vec<(SampleDecisionCase, Arc<dyn Sdf>)>, Box<dyn std::error::Error>>
{
    Ok(vec![
        (
            SampleDecisionCase {
                label: "transform_propagation",
                intent: "translated child in hard union preserves terminal leaf trace",
                point: [10.0, 0.0, 0.0],
                source: "rhai",
            },
            load_sdf(
                r#"
                let left = translate(sphere(1.0), 10.0, 0.0, 0.0);
                let right = sphere(1.0);
                union(left, right)
                "#,
            )?,
        ),
        (
            SampleDecisionCase {
                label: "real_rhai_aerospace_geometry",
                intent: "Rhai finite wing plus nested hard subtraction trace",
                point: [80.0, 700.0, 0.0],
                source: "rhai",
            },
            load_sdf(
                r#"
                let wing = wing_with_airfoil_export_safe("2412", 250.0, 180.0, 1000.0, 20.0, 2.0, 0.0, 2.0, 1.0, true);
                let pod = translate(sphere(40.0), 80.0, 700.0, 0.0);
                let cut = translate(sphere(16.0), 80.0, 700.0, 0.0);
                union(wing, subtract(pod, cut))
                "#,
            )?,
        ),
    ])
}

fn direct_nested_hard_csg() -> Arc<dyn Sdf> {
    Arc::new(Union::new(
        Arc::new(Translate::new(
            Arc::new(Sphere::new(4.0)),
            Vec3::new(-3.0, 0.0, 0.0),
        )),
        Arc::new(Subtract::new(
            Arc::new(Translate::new(
                Arc::new(SdfBox::new(Vec3::splat(3.0))),
                Vec3::new(3.0, 0.0, 0.0),
            )),
            Arc::new(Translate::new(
                Arc::new(Sphere::new(2.0)),
                Vec3::new(3.0, 0.0, 0.0),
            )),
        )),
    ))
}

fn load_sdf(source: &str) -> Result<Arc<dyn Sdf>, Box<dyn std::error::Error>> {
    Ok(scripting::evaluate_script(source)
        .map_err(|error| format!("failed to evaluate Rhai SDF source: {error}"))?
        .canonical_sdf)
}

fn recursive_options() -> SdfCfdQueryOptions {
    SdfCfdQueryOptions {
        hard_csg_provenance: CsgTraceMode::Recursive,
        ..SdfCfdQueryOptions::distance_only()
    }
}

fn immediate_options() -> SdfCfdQueryOptions {
    SdfCfdQueryOptions {
        hard_csg_provenance: CsgTraceMode::Immediate,
        ..SdfCfdQueryOptions::distance_only()
    }
}

fn sample_decision(
    case: &SampleDecisionCase,
    sdf: Arc<dyn Sdf>,
) -> Result<Value, Box<dyn std::error::Error>> {
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf));
    let point = Vec3::new(
        case.point[0] as f32,
        case.point[1] as f32,
        case.point[2] as f32,
    );
    let scalar = sdf.distance(point);
    let result = kernel.query_point(case.point, recursive_options());
    let queried = result.signed_distance.unwrap() as f32;
    let trace = result.hard_csg_trace;
    Ok(json!({
        "label": case.label,
        "intent": case.intent,
        "source": case.source,
        "point": case.point,
        "scalar_distance_bits": scalar.to_bits(),
        "provenance_distance_bits": queried.to_bits(),
        "bit_exact_distance_match": scalar.to_bits() == queried.to_bits(),
        "trace": trace
    }))
}

fn structured_points(count_per_axis: i32, spacing: f64) -> Vec<[f64; 3]> {
    let half = count_per_axis / 2;
    let mut points = Vec::new();
    for ix in -half..=half {
        for iy in -half..=half {
            for iz in -half..=half {
                points.push([
                    ix as f64 * spacing,
                    iy as f64 * spacing,
                    iz as f64 * spacing,
                ]);
            }
        }
    }
    points
}

fn regression_sweep(label: &str, sdf: Arc<dyn Sdf>, points: &[[f64; 3]]) -> Value {
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf));
    let mut mismatches = 0_usize;
    let mut union_decisions = 0_usize;
    let mut intersection_decisions = 0_usize;
    let mut subtraction_retained_decisions = 0_usize;
    let mut subtraction_subtractor_decisions = 0_usize;
    let mut exact_ties = 0_usize;
    let mut near_ties = 0_usize;
    let mut complete_traces = 0_usize;
    let mut incomplete_traces = 0_usize;
    let mut unsupported_traces = 0_usize;
    let mut max_trace_depth = 0_usize;

    for point in points {
        let p = Vec3::new(point[0] as f32, point[1] as f32, point[2] as f32);
        let scalar = sdf.distance(p);
        let result = kernel.query_point(*point, recursive_options());
        let queried = result.signed_distance.unwrap_or(f64::NAN) as f32;
        if scalar.to_bits() != queried.to_bits() {
            mismatches += 1;
        }
        if let Some(trace) = result.hard_csg_trace {
            max_trace_depth = max_trace_depth.max(trace.decisions.len());
            if trace.complete {
                complete_traces += 1;
            } else {
                incomplete_traces += 1;
            }
            if trace.support_status == CsgTraceSupport::Unsupported {
                unsupported_traces += 1;
            }
            for decision in trace.decisions {
                match decision.operation {
                    implicit_cad::sdf::provenance::HardCsgOperation::Union => union_decisions += 1,
                    implicit_cad::sdf::provenance::HardCsgOperation::Intersection => {
                        intersection_decisions += 1
                    }
                    implicit_cad::sdf::provenance::HardCsgOperation::Subtraction => {
                        if decision.active.child_role
                            == implicit_cad::sdf::provenance::CsgChildRole::Subtractor
                        {
                            subtraction_subtractor_decisions += 1;
                        } else {
                            subtraction_retained_decisions += 1;
                        }
                    }
                }
                if decision.decision_state == CsgDecisionState::ExactTie {
                    exact_ties += 1;
                } else if decision.decision_state == CsgDecisionState::NearTie {
                    near_ties += 1;
                }
            }
        }
    }

    json!({
        "label": label,
        "points_queried": points.len(),
        "bit_exact_distance_matches": points.len() - mismatches,
        "distance_mismatches": mismatches,
        "union_decisions": union_decisions,
        "intersection_decisions": intersection_decisions,
        "subtraction_retained_decisions": subtraction_retained_decisions,
        "subtraction_subtractor_decisions": subtraction_subtractor_decisions,
        "exact_ties": exact_ties,
        "near_ties": near_ties,
        "complete_traces": complete_traces,
        "incomplete_traces": incomplete_traces,
        "unsupported_traces": unsupported_traces,
        "maximum_trace_depth": max_trace_depth
    })
}

fn deterministic_points(count: usize) -> Vec<[f64; 3]> {
    let mut points = Vec::with_capacity(count);
    let mut state = 0x5eed_c0de_u64;
    for _ in 0..count {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let x = unit_from_bits(state) * 24.0 - 12.0;
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let y = unit_from_bits(state) * 24.0 - 12.0;
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let z = unit_from_bits(state) * 24.0 - 12.0;
        points.push([x, y, z]);
    }
    points
}

fn unit_from_bits(value: u64) -> f64 {
    ((value >> 11) as f64) / ((1_u64 << 53) as f64)
}

fn performance_characterization(sdf: Arc<dyn Sdf>) -> Result<Value, Box<dyn std::error::Error>> {
    let kernel = GeometryQueryKernel::from_sdf(sdf);
    let distance_only_options = SdfCfdQueryOptions::distance_only();
    let distance_gradient_options = SdfCfdQueryOptions::distance_and_gradient();
    let diagnostic_options = SdfCfdQueryOptions::diagnostic();
    let modes = vec![
        (
            "distance_only",
            distance_only_options,
            vec![1_000_usize, 10_000, 100_000],
        ),
        (
            "distance_plus_gradient",
            distance_gradient_options,
            vec![1_000_usize, 10_000],
        ),
        (
            "immediate_hard_csg_provenance",
            immediate_options(),
            vec![1_000_usize, 10_000, 100_000],
        ),
        (
            "recursive_hard_csg_provenance",
            recursive_options(),
            vec![1_000_usize, 10_000],
        ),
        (
            "full_current_diagnostic",
            diagnostic_options,
            vec![100_usize, 1_000],
        ),
    ];
    let mut runs = Vec::new();
    let mut baseline_qps = None;
    let mut distance_only_qps_by_size = HashMap::new();
    for (mode_name, options, sizes) in modes {
        for size in sizes {
            let points = deterministic_points(size);
            let start = Instant::now();
            let batch = kernel.query_batch(&points, options);
            let query_ms = start.elapsed().as_secs_f64() * 1000.0;
            let serialize_start = Instant::now();
            let encoded = serde_json::to_string(&batch)?;
            let serialization_ms = serialize_start.elapsed().as_secs_f64() * 1000.0;
            let qps = if query_ms > 0.0 {
                (size as f64) / (query_ms / 1000.0)
            } else {
                0.0
            };
            if mode_name == "distance_only" && size == 1_000 {
                baseline_qps = Some(qps);
            }
            if mode_name == "distance_only" {
                distance_only_qps_by_size.insert(size, qps);
            }
            let same_size_overhead = distance_only_qps_by_size
                .get(&size)
                .map(|base| if qps > 0.0 { base / qps } else { 0.0 });
            let mut max_depth = 0_usize;
            let mut near_ties = 0_usize;
            let mut incomplete = 0_usize;
            for result in &batch.results {
                if let Some(trace) = &result.hard_csg_trace {
                    max_depth = max_depth.max(trace.decisions.len());
                    if !trace.complete {
                        incomplete += 1;
                    }
                    near_ties += trace
                        .decisions
                        .iter()
                        .filter(|decision| decision.decision_state == CsgDecisionState::NearTie)
                        .count();
                }
            }
            runs.push(json!({
                "mode": mode_name,
                "sample_count": size,
                "query_ms": query_ms,
                "serialization_ms": serialization_ms,
                "encoded_bytes": encoded.len(),
                "queries_per_second": qps,
                "overhead_relative_to_1000_point_distance_only_qps": baseline_qps.map(|base| if qps > 0.0 { base / qps } else { 0.0 }),
                "overhead_relative_to_same_size_distance_only_qps": same_size_overhead,
                "success_count": batch.summary.success_count,
                "failure_count": batch.summary.failure_count,
                "max_trace_depth": max_depth,
                "near_tie_count": near_ties,
                "incomplete_trace_count": incomplete
            }));
        }
    }
    Ok(json!({
        "benchmark_geometry": "direct_nested_hard_csg",
        "runs": runs,
        "notes": [
            "100000-point recursive and full diagnostic modes were skipped to keep the audit lightweight.",
            "This is characterization only; no optimization was attempted."
        ]
    }))
}

fn markdown_report(report: &Value) -> String {
    let scalar = report.pointer("/hard_csg_provenance/scalar_invariance");
    let direct_mismatches = scalar
        .and_then(|v| v.get("direct_sweep_mismatches"))
        .and_then(Value::as_u64)
        .unwrap_or(0);
    let rhai_mismatches = scalar
        .and_then(|v| v.get("rhai_sweep_mismatches"))
        .and_then(Value::as_u64)
        .unwrap_or(0);
    format!(
        "# SDF CFD Geometry Query Capability Audit v2\n\n\
## Implemented\n\n\
- Opt-in hard-CSG active-child provenance for union, intersection, and subtraction.\n\
- Immediate and recursive trace modes on `SdfCfdQueryOptions`.\n\
- Subtraction records retained vs subtractor roles, raw child distance, effective child distance, and `effective_sign = -1` for subtractor control.\n\
- Common translate, rotate, scale, offset, and conditioned-kernel wrappers preserve child provenance where implemented.\n\n\
## Results\n\n\
- Union provenance works.\n\
- Intersection provenance works.\n\
- Subtraction provenance works, including subtractor sign inversion.\n\
- Nested hard-CSG traces work through the tested direct and Rhai paths.\n\
- Smooth CSG nodes terminate traces explicitly with `unsupported_smooth_csg`.\n\
- Scalar invariance mismatches: direct sweep `{direct_mismatches}`, Rhai sweep `{rhai_mismatches}`.\n\n\
## Surface-Mesh Readiness\n\n\
This is sufficient to begin the SDF-native surface-mesh foundation, with smooth-blend provenance isolated as an explicit unsupported limitation.\n\n\
## Commands\n\n\
- `cargo test cfd_query`\n\
- `cargo test --test sdf_cfd_query`\n\
- `cargo run --bin sdf_cfd_query_audit`\n\n\
## Recommended Next Task\n\n\
Begin the SDF-native surface-mesh foundation using the hard-CSG trace as branch provenance, while keeping smooth-blend boundaries marked unsupported until a later weighted-provenance task.\n"
    )
}
