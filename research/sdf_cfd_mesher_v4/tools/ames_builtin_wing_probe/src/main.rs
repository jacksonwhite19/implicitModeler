use glam::{Quat, Vec3};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::aerospace::{
    Airfoil, AirfoilExportOptions, LoftedWing, wing_with_airfoil_export_safe,
};
use implicit_cad::sdf::transforms::{Rotate, Translate};
use serde::Serialize;
use serde_json::{Value, json};
use std::fs;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

const CHORD_MM: f32 = 1219.2;
const CONSTANT_SPAN_MM: f32 = 841.248;
const BUILTIN_TOTAL_SPAN_MM: f32 = 2.0 * CONSTANT_SPAN_MM;
const TARGET_TE_MM: f32 = 2.54;

#[derive(Serialize)]
struct SectionMeasurements {
    chord_parameter_mm: f64,
    x_min_mm: f64,
    x_max_mm: f64,
    chord_extent_mm: f64,
    leading_edge_mm: [f64; 2],
    leading_edge_upper_endpoint_mm: [f64; 2],
    leading_edge_lower_endpoint_mm: [f64; 2],
    leading_edge_closure_segment_length_mm: f64,
    trailing_edge_upper_mm: [f64; 2],
    trailing_edge_lower_mm: [f64; 2],
    trailing_edge_full_thickness_mm: f64,
    maximum_vertex_full_thickness_mm: f64,
    maximum_vertex_thickness_x_mm: f64,
    authoritative_representation: &'static str,
    preview_point_count_including_closure: usize,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let started = Instant::now();
    let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../..").canonicalize()?;
    let script_path = repo.join("research/sdf_cfd_mesher_v4/tools/ames_wingtip_builtin_probe.rhai");
    let output = repo.join("research/sdf_cfd_mesher_v4/results/ames_builtin_wing_tool_audit");
    fs::create_dir_all(&output)?;
    let source = fs::read_to_string(&script_path)?;

    let first = scripting::evaluate_script(&source)?.canonical_sdf;
    let reloaded_source = String::from_utf8(fs::read(&script_path)?)?;
    let second = scripting::evaluate_script(&reloaded_source)?.canonical_sdf;
    let wing = downcast_wing(&first)?;
    let _reloaded_wing = downcast_wing(&second)?;

    let direct = wing_with_airfoil_export_safe(
        "0012", CHORD_MM, CHORD_MM, BUILTIN_TOTAL_SPAN_MM, 0.0, 0.0, 0.0,
        AirfoilExportOptions {
            min_trailing_edge_thickness_mm: TARGET_TE_MM,
            min_leading_edge_radius_mm: 0.0,
            clip_trailing_edge: true,
        },
    );
    let unclipped = wing_with_airfoil_export_safe(
        "0012", CHORD_MM, CHORD_MM, BUILTIN_TOTAL_SPAN_MM, 0.0, 0.0, 0.0,
        AirfoilExportOptions {
            min_trailing_edge_thickness_mm: TARGET_TE_MM,
            min_leading_edge_radius_mm: 0.0,
            clip_trailing_edge: false,
        },
    );

    let clipped_section = center_airfoil(wing)?;
    let unclipped_section = center_airfoil(&unclipped)?;
    let clipped = section_measurements(clipped_section);
    let unclipped_m = section_measurements(unclipped_section);

    let half_span = wing.sections.iter().map(|s| s.position.y.abs()).fold(0.0_f32, f32::max);
    let sample_points = vec![
        Vec3::new(0.5 * CHORD_MM, 0.0, 0.0),
        Vec3::new(CHORD_MM + 0.5, 0.0, 0.0),
        Vec3::new(CHORD_MM, 0.0, 0.0),
        Vec3::new(0.5 * CHORD_MM, half_span + 0.5, 0.0),
        Vec3::new(-0.5, 0.0, 0.0),
        Vec3::new(0.3 * CHORD_MM, 0.25 * half_span, 0.2 * CHORD_MM),
    ];
    let sample_rows = sample_points.iter().map(|&p| {
        let a = first.distance(p);
        let b = second.distance(p);
        let c = direct.distance(p);
        json!({
            "point_mm": p.to_array(),
            "rhai_distance_mm": a,
            "reload_distance_mm": b,
            "direct_constructor_distance_mm": c,
            "reload_bit_exact": a.to_bits() == b.to_bits(),
            "direct_bit_exact": a.to_bits() == c.to_bits()
        })
    }).collect::<Vec<_>>();

    let metadata = first.metadata();
    let metadata_json = json!({
        "node_kind": metadata.node_kind,
        "support_bounds": metadata.support_bounds.as_ref().map(|b| json!({"min": b.min.to_array(), "max": b.max.to_array()})),
        "feature_ids": metadata.feature_ids,
        "feature_regions": metadata.feature_regions.iter().map(|r| json!({
            "feature_id": r.feature_id,
            "bounds": {"min": r.bounds.min.to_array(), "max": r.bounds.max.to_array()},
            "min_feature_size": r.min_feature_size
        })).collect::<Vec<_>>(),
        "min_feature_size": metadata.min_feature_size,
        "bounds_are_approximate": metadata.bounds_are_approximate,
        "bound_quality": format!("{:?}", metadata.bound_quality),
        "parameter_fingerprint": metadata.parameter_fingerprint,
        "conditioned_cache_present": metadata.conditioned_cache.is_some()
    });
    let inventory = first.procedural_feature_inventory();
    let query_points = diagnostic_points(clipped_section, half_span);
    let query_rows = query_points.iter().map(|(label, point)| {
        direct_projection_diagnostic(first.as_ref(), label, Vec3::from_array(*point))
    }).collect::<Vec<_>>();

    let span_positions = wing.sections.iter().map(|s| s.position.to_array()).collect::<Vec<_>>();
    let feature_inventory = serde_json::to_value(inventory)?;
    let transform = rigid_transform_diagnostic();
    let report = json!({
        "schema": "SDFCFDMesherV4BuiltinWingProbe/1",
        "scope": "diagnostic only; no surface mesh generated",
        "units": "millimetres",
        "input": {
            "script": script_path.strip_prefix(&repo)?.to_string_lossy(),
            "airfoil": "0012",
            "root_chord_mm": CHORD_MM,
            "tip_chord_mm": CHORD_MM,
            "builtin_total_span_mm": BUILTIN_TOTAL_SPAN_MM,
            "intended_constant_semispan_mm": CONSTANT_SPAN_MM,
            "sweep_deg": 0.0,
            "dihedral_deg": 0.0,
            "twist_deg": 0.0,
            "requested_trailing_edge_full_thickness_mm": TARGET_TE_MM,
            "minimum_leading_edge_radius_mm": 0.0,
            "clip_trailing_edge": true
        },
        "constructor": {
            "type": "LoftedWing",
            "section_count": wing.sections.len(),
            "section_positions_mm": span_positions,
            "full_span_semantics": "span/2 at each planar end",
            "tip_topology": "planar section cap from max(section_distance, interval_distance)",
            "root_at_y_zero": "interior section, not a closure face"
        },
        "clipped_section": clipped,
        "unclipped_section": unclipped_m,
        "trailing_edge_operation": {
            "measured_chord_loss_mm": clipped.chord_extent_mm - unclipped_m.chord_extent_mm,
            "measured_te_change_mm": clipped.trailing_edge_full_thickness_mm - unclipped_m.trailing_edge_full_thickness_mm,
            "rescale_observed": false
        },
        "distance_samples": sample_rows,
        "metadata": metadata_json,
        "procedural_feature_inventory": feature_inventory,
        "geometry_query": {
            "public_conditioned_query_attempt": {
                "completed": false,
                "stop_reason": "bounded diagnostic stopped during eager whole-domain conditioning",
                "peak_working_set_bytes": 2430365696_u64,
                "aggregate_process_cpu_seconds_at_stop": 1174.0625,
                "peak_virtual_memory_bytes": 6919696384_u64,
                "retained_conditioned_field": false
            },
            "replacement_method": "direct canonical Sdf::distance plus central differences and eight unqualified Newton steps; diagnostic only",
            "finite_difference_step_mm": 0.001,
            "samples": query_rows
        },
        "serialization_reload": {
            "mechanism": "Rhai source bytes saved and evaluated again",
            "source_bytes_equal": source.as_bytes() == reloaded_source.as_bytes(),
            "sample_count": sample_points.len(),
            "all_distance_bits_equal": sample_points.iter().all(|&p| first.distance(p).to_bits() == second.distance(p).to_bits())
        },
        "rigid_transform": transform,
        "elapsed_seconds": started.elapsed().as_secs_f64()
    });
    fs::write(output.join("PROBE_RESULT.json"), serde_json::to_string_pretty(&report)? + "\n")?;
    fs::write(output.join("builtin_wing_geometry.svg"), render_svg(wing, clipped_section, &clipped))?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn downcast_wing(sdf: &Arc<dyn Sdf>) -> Result<&LoftedWing, Box<dyn std::error::Error>> {
    sdf.as_ref().as_any().downcast_ref::<LoftedWing>().ok_or_else(|| "Rhai result is not LoftedWing".into())
}

fn center_airfoil(wing: &LoftedWing) -> Result<&Airfoil, Box<dyn std::error::Error>> {
    let section = wing.sections.iter().min_by(|a, b| a.position.y.abs().total_cmp(&b.position.y.abs())).ok_or("wing has no section")?;
    section.section.as_any().downcast_ref::<Airfoil>().ok_or_else(|| "center section is not Airfoil".into())
}

fn section_measurements(airfoil: &Airfoil) -> SectionMeasurements {
    let scale = f64::from(airfoil.chord);
    let data = airfoil.points.iter().map(|&(x, y)| [f64::from(x) * scale, f64::from(y) * scale]).collect::<Vec<_>>();
    let x_min = data.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let x_max = data.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
    let le = *data.iter().min_by(|a, b| a[0].total_cmp(&b[0])).unwrap();
    let leading_idx = data.iter().enumerate().min_by(|(_, a), (_, b)| a[0].total_cmp(&b[0])).map(|(i, _)| i).unwrap();
    let le_upper = data[leading_idx];
    let le_lower = if airfoil.spline().is_some() {
        le_upper
    } else {
        data[(leading_idx + 1).min(data.len() - 2)]
    };
    let upper = data[0];
    let lower = data[data.len() - 2];
    let (max_idx, max_y) = data.iter().enumerate().take((data.len() - 1) / 2 + 1)
        .max_by(|(_, a), (_, b)| a[1].total_cmp(&b[1])).map(|(i, p)| (i, p[1])).unwrap();
    let representation = if airfoil.spline().is_some() { "smooth_naca_spline" } else { "closed_piecewise_linear_polygon" };
    SectionMeasurements {
        chord_parameter_mm: scale,
        x_min_mm: x_min,
        x_max_mm: x_max,
        chord_extent_mm: x_max - x_min,
        leading_edge_mm: le,
        leading_edge_upper_endpoint_mm: le_upper,
        leading_edge_lower_endpoint_mm: le_lower,
        leading_edge_closure_segment_length_mm: ((le_upper[0] - le_lower[0]).powi(2) + (le_upper[1] - le_lower[1]).powi(2)).sqrt(),
        trailing_edge_upper_mm: upper,
        trailing_edge_lower_mm: lower,
        trailing_edge_full_thickness_mm: (upper[1] - lower[1]).abs(),
        maximum_vertex_full_thickness_mm: 2.0 * max_y,
        maximum_vertex_thickness_x_mm: data[max_idx][0],
        authoritative_representation: representation,
        preview_point_count_including_closure: data.len(),
    }
}

fn rigid_transform_diagnostic() -> Value {
    let base: Arc<dyn Sdf> = Arc::new(wing_with_airfoil_export_safe(
        "0012", CHORD_MM, CHORD_MM, BUILTIN_TOTAL_SPAN_MM, 0.0, 0.0, 0.0,
        AirfoilExportOptions {
            min_trailing_edge_thickness_mm: TARGET_TE_MM,
            min_leading_edge_radius_mm: 0.0,
            clip_trailing_edge: true,
        },
    ));
    let pivot = Vec3::new(0.25 * CHORD_MM, 0.0, 0.0);
    let rotation = Quat::from_rotation_y(10.0_f32.to_radians());
    let shifted: Arc<dyn Sdf> = Arc::new(Translate::new(Arc::clone(&base), -pivot));
    let rotated: Arc<dyn Sdf> = Arc::new(Rotate::new(shifted, rotation));
    let transformed: Arc<dyn Sdf> = Arc::new(Translate::new(rotated, pivot));
    let local = Vec3::new(0.42 * CHORD_MM, 0.31 * CONSTANT_SPAN_MM, 90.0);
    let world = pivot + rotation * (local - pivot);
    let before = base.distance(local);
    let after = transformed.distance(world);
    json!({
        "operation": "translate(-quarter_chord), rotate_y(+10 degrees), translate(+quarter_chord)",
        "pivot_mm": pivot.to_array(),
        "local_probe_mm": local.to_array(),
        "world_probe_mm": world.to_array(),
        "distance_before_mm": before,
        "distance_after_mm": after,
        "absolute_difference_mm": (after - before).abs(),
        "bit_exact": before.to_bits() == after.to_bits(),
        "defined_semantics": "Translate and Rotate evaluate the child at the inverse rigid transform; rigid distances are preserved subject to binary32 arithmetic"
    })
}

fn diagnostic_points(airfoil: &Airfoil, half_span: f32) -> Vec<(&'static str, [f32; 3])> {
    let chord = airfoil.chord;
    let upper = airfoil.points.iter().filter(|(_, y)| *y >= 0.0).min_by(|a, b| (a.0 - 0.3).abs().total_cmp(&(b.0 - 0.3).abs())).unwrap();
    let upper_point = [upper.0 * chord, 0.0, upper.1 * chord];
    vec![
        ("upper_surface_offset", [upper_point[0], upper_point[1], upper_point[2] + 0.25]),
        ("leading_edge_offset", [-0.25, 0.0, 0.0]),
        ("trailing_face_offset", [chord + 0.25, 0.0, 0.0]),
        ("positive_planar_tip_offset", [0.5 * chord, half_span + 0.25, 0.0]),
        ("root_upper_surface_offset", [upper_point[0], 0.0, upper_point[2] + 0.25]),
    ]
}

fn finite_gradient(sdf: &dyn Sdf, p: Vec3, h: f32) -> Vec3 {
    let ex = Vec3::new(h, 0.0, 0.0);
    let ey = Vec3::new(0.0, h, 0.0);
    let ez = Vec3::new(0.0, 0.0, h);
    Vec3::new(
        (sdf.distance(p + ex) - sdf.distance(p - ex)) / (2.0 * h),
        (sdf.distance(p + ey) - sdf.distance(p - ey)) / (2.0 * h),
        (sdf.distance(p + ez) - sdf.distance(p - ez)) / (2.0 * h),
    )
}

fn direct_projection_diagnostic(sdf: &dyn Sdf, label: &str, input: Vec3) -> Value {
    let h = 0.001_f32;
    let mut p = input;
    let initial = sdf.distance(p);
    let initial_gradient = finite_gradient(sdf, p, h);
    let mut completed_steps = 0;
    let mut failure = None;
    for _ in 0..8 {
        let d = sdf.distance(p);
        let g = finite_gradient(sdf, p, h);
        let g2 = g.length_squared();
        if !d.is_finite() || !g.is_finite() || g2 <= 1.0e-16 {
            failure = Some("nonfinite or zero finite-difference gradient");
            break;
        }
        p -= g * (d / g2);
        completed_steps += 1;
    }
    let final_distance = sdf.distance(p);
    let final_gradient = finite_gradient(sdf, p, h);
    json!({
        "label": label,
        "input_mm": input.to_array(),
        "initial_signed_distance_mm": initial,
        "initial_gradient": initial_gradient.to_array(),
        "initial_gradient_magnitude": initial_gradient.length(),
        "projected_point_mm": p.to_array(),
        "displacement_mm": (p - input).length(),
        "final_signed_distance_mm": final_distance,
        "final_gradient": final_gradient.to_array(),
        "final_gradient_magnitude": final_gradient.length(),
        "steps_completed": completed_steps,
        "failure": failure,
        "qualification": "diagnostic only; no projection or normal pass gate exists for this geometry"
    })
}

fn render_svg(wing: &LoftedWing, airfoil: &Airfoil, m: &SectionMeasurements) -> String {
    let width = 1700.0;
    let height = 1040.0;
    let min_y = wing.sections.iter().map(|s| s.position.y).fold(f32::INFINITY, f32::min);
    let max_y = wing.sections.iter().map(|s| s.position.y).fold(f32::NEG_INFINITY, f32::max);
    let plan_x = |x: f32| 80.0 + f64::from(x / CHORD_MM) * 580.0;
    let plan_y = |y: f32| 325.0 - f64::from((y - min_y) / (max_y - min_y)) * 250.0;
    let section_points = airfoil.points.iter().map(|&(x, z)| format!("{:.3},{:.3}", 80.0 + f64::from(x) * 580.0, 650.0 - f64::from(z) * 2400.0)).collect::<Vec<_>>().join(" ");
    let te_points = airfoil.points.iter().filter(|(x, _)| *x > 0.985).map(|&(x, z)| format!("{:.3},{:.3}", 900.0 + f64::from((x - 0.985) / 0.015) * 450.0, 750.0 - f64::from(z) * 14000.0)).collect::<Vec<_>>().join(" ");
    format!(r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#f7f8fa"/>
<style>text{{font-family:Arial,sans-serif;fill:#17202a}} .axis{{stroke:#536878;stroke-width:2}} .geom{{fill:#8ecae6;stroke:#174a66;stroke-width:2}} .note{{font-size:18px}} .small{{font-size:15px}}</style>
<text x="50" y="38" font-size="27" font-weight="bold">Built-in wing diagnostic: actual LoftedWing output</text>
<text x="50" y="65" class="small">NACA 0012, chord 1219.2 mm, built-in span 1682.496 mm, requested TE 2.54 mm. Not an Ames production geometry.</text>
<text x="80" y="100" class="note" font-weight="bold">Planform (x-y)</text>
<polygon class="geom" points="{x0},{y0} {x1},{y0} {x1},{y1} {x0},{y1}"/>
<line class="axis" x1="80" y1="350" x2="680" y2="350"/><text x="685" y="355" class="note">+x chord</text>
<line class="axis" x1="50" y1="325" x2="50" y2="75"/><text x="12" y="135" class="note">+y</text><text x="12" y="158" class="note">span</text>
<text x="720" y="145" class="note">Both ends are planar section caps.</text>
<text x="720" y="175" class="note">No rounded-tip option is applied.</text>
<text x="720" y="205" class="note">y=0 is an interior root section.</text>
<text x="50" y="420" class="note" font-weight="bold">Section (x-z)</text>
<polyline class="geom" points="{section_points}"/>
<line class="axis" x1="80" y1="690" x2="680" y2="690"/><text x="685" y="695" class="note">+x</text>
<line class="axis" x1="60" y1="780" x2="60" y2="500"/><text x="35" y="490" class="note">+z</text>
<text x="50" y="825" class="small">Measured chord extent: {chord:.9} mm; max vertex thickness: {thick:.9} mm at x={thick_x:.9} mm.</text>
<text x="850" y="420" class="note" font-weight="bold">Trailing-edge detail</text>
<polyline fill="none" stroke="#174a66" stroke-width="3" points="{te_points}"/>
<line class="axis" x1="900" y1="790" x2="1370" y2="790"/><text x="1375" y="795" class="note">+x</text>
<line class="axis" x1="875" y1="900" x2="875" y2="535"/><text x="850" y="525" class="note">+z</text>
<text x="850" y="925" class="small">Endpoint face thickness: {te:.9} mm.</text>
<text x="850" y="948" class="small">The operation moves endpoint ordinates at x/c=1; it does not shorten or rescale the chord.</text>
<text x="50" y="1010" class="small">Coordinates and curves in this figure were read from the built-in LoftedWing/Airfoil object. This is diagnostic visual evidence, not a surface mesh.</text>
</svg>"##,
        x0=plan_x(0.0), x1=plan_x(CHORD_MM), y0=plan_y(min_y), y1=plan_y(max_y),
        chord=m.chord_extent_mm, thick=m.maximum_vertex_full_thickness_mm,
        thick_x=m.maximum_vertex_thickness_x_mm, te=m.trailing_edge_full_thickness_mm)
}
