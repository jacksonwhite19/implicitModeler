use glam::Vec3;
use implicit_historical::sdf::Sdf;
use implicit_historical::sdf::aerospace::{
    Airfoil, AirfoilExportOptions, get_naca_airfoil_export_safe,
};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

#[derive(Serialize)]
struct BoundsResult {
    min_mm: [f32; 3],
    max_mm: [f32; 3],
    note: &'static str,
}

#[derive(Serialize)]
struct AirfoilResult {
    designation: &'static str,
    chord_mm: f32,
    point_count: usize,
    x_min_mm: f32,
    x_max_mm: f32,
    chord_extent_mm: f32,
    z_min_mm: f32,
    z_max_mm: f32,
    vertical_extent_mm: f32,
    trailing_edge_upper_mm: [f32; 2],
    trailing_edge_lower_mm: [f32; 2],
    trailing_edge_full_thickness_mm: f32,
    leading_edge_min_x_mm: f32,
    leading_edge_z_mm: f32,
    maximum_abs_z_mm: f32,
    maximum_abs_z_x_mm: f32,
    metadata_trailing_edge_thickness_mm: Option<f32>,
    metadata_leading_edge_radius_mm: Option<f32>,
}

#[derive(Clone, Copy)]
struct Panel {
    name: &'static str,
    u_label: &'static str,
    v_label: &'static str,
    u0: f32,
    u1: f32,
    v0: f32,
    v1: f32,
    nu: usize,
    nv: usize,
    map: fn(f32, f32) -> Vec3,
}

#[derive(Serialize)]
struct PanelResult {
    name: &'static str,
    plane: String,
    sample_count: usize,
    inside_sample_count: usize,
    nonfinite_sample_count: usize,
    boundary_root_count: usize,
    max_abs_boundary_residual_mm: f32,
    gradient_magnitude_min: Option<f32>,
    gradient_magnitude_max: Option<f32>,
    occupied_u_min_mm: Option<f32>,
    occupied_u_max_mm: Option<f32>,
    occupied_v_min_mm: Option<f32>,
    occupied_v_max_mm: Option<f32>,
}

#[derive(Serialize)]
struct VariantDifference {
    name: &'static str,
    evaluated_points: usize,
    nonzero_bitwise_differences: usize,
    base_nonfinite_count: usize,
    variant_nonfinite_count: usize,
    maximum_absolute_field_difference_mm: f32,
}

#[derive(Serialize)]
struct ProbeResult {
    schema: &'static str,
    exact_script_path: String,
    exact_script_bytes: usize,
    current_root_execution: String,
    current_root_elapsed_seconds: f64,
    historical_execution: String,
    historical_elapsed_seconds: f64,
    historical_sdf_type: String,
    historical_bounds: Option<BoundsResult>,
    source_reload_sample_count: usize,
    source_reload_bitwise_match_count: usize,
    source_reload_max_abs_difference_mm: f32,
    airfoil_sections: Vec<AirfoilResult>,
    panels: Vec<PanelResult>,
    control_variants: Vec<VariantDifference>,
    sdf_evaluations: usize,
    limitations: Vec<&'static str>,
}

#[derive(Deserialize)]
struct CurrentExecution {
    execution: String,
    elapsed_seconds: f64,
}

fn airfoil_result(
    designation: &'static str,
    chord_mm: f32,
    te_mm: f32,
    le_mm: f32,
) -> AirfoilResult {
    let airfoil: Arc<Airfoil> = get_naca_airfoil_export_safe(
        designation,
        chord_mm,
        AirfoilExportOptions {
            min_trailing_edge_thickness_mm: te_mm,
            min_leading_edge_radius_mm: le_mm,
        },
    );
    let scaled: Vec<(f32, f32)> = airfoil
        .points
        .iter()
        .map(|&(x, z)| (x * chord_mm, z * chord_mm))
        .collect();
    let x_min = scaled.iter().map(|p| p.0).fold(f32::INFINITY, f32::min);
    let x_max = scaled.iter().map(|p| p.0).fold(f32::NEG_INFINITY, f32::max);
    let z_min = scaled.iter().map(|p| p.1).fold(f32::INFINITY, f32::min);
    let z_max = scaled.iter().map(|p| p.1).fold(f32::NEG_INFINITY, f32::max);
    let le = scaled
        .iter()
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .copied()
        .unwrap();
    let max_abs = scaled
        .iter()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .copied()
        .unwrap();
    let upper_te = scaled[0];
    let lower_te = scaled[scaled.len() - 2];
    AirfoilResult {
        designation,
        chord_mm,
        point_count: scaled.len(),
        x_min_mm: x_min,
        x_max_mm: x_max,
        chord_extent_mm: x_max - x_min,
        z_min_mm: z_min,
        z_max_mm: z_max,
        vertical_extent_mm: z_max - z_min,
        trailing_edge_upper_mm: [upper_te.0, upper_te.1],
        trailing_edge_lower_mm: [lower_te.0, lower_te.1],
        trailing_edge_full_thickness_mm: (upper_te.1 - lower_te.1).abs(),
        leading_edge_min_x_mm: le.0,
        leading_edge_z_mm: le.1,
        maximum_abs_z_mm: max_abs.1.abs(),
        maximum_abs_z_x_mm: max_abs.0,
        metadata_trailing_edge_thickness_mm: airfoil
            .feature_metadata
            .map(|m| m.trailing_edge_thickness_mm),
        metadata_leading_edge_radius_mm: airfoil.feature_metadata.map(|m| m.leading_edge_radius_mm),
    }
}

fn bisection_root(sdf: &dyn Sdf, a: Vec3, b: Vec3) -> (Vec3, f32, usize) {
    let mut lo = a;
    let mut hi = b;
    let mut dlo = sdf.distance(lo);
    let mut evals = 1usize;
    for _ in 0..40 {
        let mid = (lo + hi) * 0.5;
        let dm = sdf.distance(mid);
        evals += 1;
        if (dm <= 0.0) == (dlo <= 0.0) {
            lo = mid;
            dlo = dm;
        } else {
            hi = mid;
        }
    }
    let p = (lo + hi) * 0.5;
    let residual = sdf.distance(p).abs();
    (p, residual, evals + 1)
}

fn gradient_magnitude(sdf: &dyn Sdf, p: Vec3, h: f32) -> (f32, usize) {
    let ex = Vec3::new(h, 0.0, 0.0);
    let ey = Vec3::new(0.0, h, 0.0);
    let ez = Vec3::new(0.0, 0.0, h);
    let g = Vec3::new(
        (sdf.distance(p + ex) - sdf.distance(p - ex)) / (2.0 * h),
        (sdf.distance(p + ey) - sdf.distance(p - ey)) / (2.0 * h),
        (sdf.distance(p + ez) - sdf.distance(p - ez)) / (2.0 * h),
    );
    (g.length(), 6)
}

fn inspect_panel(sdf: &dyn Sdf, panel: Panel) -> (PanelResult, Vec<(usize, usize, usize)>, usize) {
    let mut values = vec![0.0f32; panel.nu * panel.nv];
    let mut inside = 0usize;
    let mut nonfinite = 0usize;
    let mut u_min = f32::INFINITY;
    let mut u_max = f32::NEG_INFINITY;
    let mut v_min = f32::INFINITY;
    let mut v_max = f32::NEG_INFINITY;
    let mut evals = 0usize;
    for j in 0..panel.nv {
        let v = panel.v0 + (panel.v1 - panel.v0) * j as f32 / (panel.nv - 1) as f32;
        for i in 0..panel.nu {
            let u = panel.u0 + (panel.u1 - panel.u0) * i as f32 / (panel.nu - 1) as f32;
            let d = sdf.distance((panel.map)(u, v));
            evals += 1;
            values[j * panel.nu + i] = d;
            if !d.is_finite() {
                nonfinite += 1;
            }
            if d <= 0.0 {
                inside += 1;
                u_min = u_min.min(u);
                u_max = u_max.max(u);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }

    let mut roots = Vec::new();
    let mut max_residual = 0.0f32;
    let mut grad_min = f32::INFINITY;
    let mut grad_max = f32::NEG_INFINITY;
    for j in 0..panel.nv {
        let v = panel.v0 + (panel.v1 - panel.v0) * j as f32 / (panel.nv - 1) as f32;
        for i in 0..panel.nu - 1 {
            let d0 = values[j * panel.nu + i];
            let d1 = values[j * panel.nu + i + 1];
            if (d0 <= 0.0) != (d1 <= 0.0) {
                let u0 = panel.u0 + (panel.u1 - panel.u0) * i as f32 / (panel.nu - 1) as f32;
                let u1 = panel.u0 + (panel.u1 - panel.u0) * (i + 1) as f32 / (panel.nu - 1) as f32;
                let (root, residual, n) =
                    bisection_root(sdf, (panel.map)(u0, v), (panel.map)(u1, v));
                evals += n;
                max_residual = max_residual.max(residual);
                let (g, n) = gradient_magnitude(sdf, root, 0.01);
                evals += n;
                grad_min = grad_min.min(g);
                grad_max = grad_max.max(g);
                roots.push((j, i, i + 1));
            }
        }
    }

    let some_inside = inside > 0;
    (
        PanelResult {
            name: panel.name,
            plane: format!("{} by {}", panel.u_label, panel.v_label),
            sample_count: panel.nu * panel.nv,
            inside_sample_count: inside,
            nonfinite_sample_count: nonfinite,
            boundary_root_count: roots.len(),
            max_abs_boundary_residual_mm: max_residual,
            gradient_magnitude_min: (grad_min.is_finite()).then_some(grad_min),
            gradient_magnitude_max: (grad_max.is_finite()).then_some(grad_max),
            occupied_u_min_mm: some_inside.then_some(u_min),
            occupied_u_max_mm: some_inside.then_some(u_max),
            occupied_v_min_mm: some_inside.then_some(v_min),
            occupied_v_max_mm: some_inside.then_some(v_max),
        },
        roots,
        evals,
    )
}

fn svg_panel(
    sdf: &dyn Sdf,
    panel: Panel,
    x0: f32,
    y0: f32,
    width: f32,
    height: f32,
) -> (String, usize) {
    let mut body = String::new();
    let mut evals = 0usize;
    for j in 0..panel.nv {
        let v = panel.v1 - (panel.v1 - panel.v0) * j as f32 / (panel.nv - 1) as f32;
        let mut run_start: Option<usize> = None;
        for i in 0..panel.nu {
            let u = panel.u0 + (panel.u1 - panel.u0) * i as f32 / (panel.nu - 1) as f32;
            let inside = sdf.distance((panel.map)(u, v)) <= 0.0;
            evals += 1;
            if inside && run_start.is_none() {
                run_start = Some(i);
            }
            if (!inside || i + 1 == panel.nu) && run_start.is_some() {
                let start = run_start.take().unwrap();
                let end = if inside && i + 1 == panel.nu {
                    i
                } else {
                    i.saturating_sub(1)
                };
                let sx = x0 + width * start as f32 / (panel.nu - 1) as f32;
                let ex = x0 + width * end as f32 / (panel.nu - 1) as f32;
                let sy = y0 + height * j as f32 / (panel.nv - 1) as f32;
                body.push_str(&format!("<line x1=\"{sx:.3}\" y1=\"{sy:.3}\" x2=\"{ex:.3}\" y2=\"{sy:.3}\" stroke=\"#b8d8f4\" stroke-width=\"2.4\"/>\n"));
            }
        }
    }
    body.push_str(&format!("<rect x=\"{x0}\" y=\"{y0}\" width=\"{width}\" height=\"{height}\" fill=\"none\" stroke=\"#30475e\"/>\n"));
    body.push_str(&format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" class=\"title\">{}</text>\n",
        x0,
        y0 - 12.0,
        panel.name
    ));
    body.push_str(&format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" class=\"axis\">{}: {:.1} to {:.1} mm</text>\n",
        x0,
        y0 + height + 18.0,
        panel.u_label,
        panel.u0,
        panel.u1
    ));
    body.push_str(&format!(
        "<text x=\"{:.1}\" y=\"{:.1}\" class=\"axis\">{}: {:.1} to {:.1} mm</text>\n",
        x0,
        y0 + height + 34.0,
        panel.v_label,
        panel.v0,
        panel.v1
    ));
    (body, evals)
}

fn compare_variant(
    base: &dyn Sdf,
    variant: &dyn Sdf,
    name: &'static str,
) -> (VariantDifference, usize) {
    let mut differences = 0usize;
    let mut base_nonfinite = 0usize;
    let mut variant_nonfinite = 0usize;
    let mut max_abs = 0.0f32;
    let mut count = 0usize;
    for iz in 0..17 {
        let z = -70.0 + iz as f32 * 140.0 / 16.0;
        for iy in 0..17 {
            let y = 325.0 + iy as f32 * 50.0 / 16.0;
            for ix in 0..17 {
                let x = 300.0 + ix as f32 * 140.0 / 16.0;
                let p = Vec3::new(x, y, z);
                let a = base.distance(p);
                let b = variant.distance(p);
                base_nonfinite += usize::from(!a.is_finite());
                variant_nonfinite += usize::from(!b.is_finite());
                count += 1;
                if a.to_bits() != b.to_bits() {
                    differences += 1;
                }
                max_abs = max_abs.max((a - b).abs());
            }
        }
    }
    (
        VariantDifference {
            name,
            evaluated_points: count,
            nonzero_bitwise_differences: differences,
            base_nonfinite_count: base_nonfinite,
            variant_nonfinite_count: variant_nonfinite,
            maximum_absolute_field_difference_mm: max_abs,
        },
        count * 2,
    )
}

fn variant_source(source: &str, needle: &str, replacement: &str) -> Result<String, String> {
    let count = source.matches(needle).count();
    if count != 1 {
        return Err(format!(
            "expected one occurrence of '{needle}', found {count}"
        ));
    }
    Ok(source.replacen(needle, replacement, 1))
}

fn write_svg(path: &Path, sdf: &dyn Sdf, panels: &[Panel]) -> Result<usize, String> {
    let canvas_width = 20 + panels.len() * 390;
    let mut svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{canvas_width}\" height=\"470\" viewBox=\"0 0 {canvas_width} 470\"><style>.title{{font:600 16px sans-serif;fill:#102a43}}.axis{{font:12px monospace;fill:#486581}}.note{{font:13px sans-serif;fill:#243b53}}</style><rect width=\"100%\" height=\"100%\" fill=\"#f7fafc\"/><text x=\"20\" y=\"28\" class=\"title\">Exact curated built-in wingtip workflow: direct signed-field sections</text><text x=\"20\" y=\"48\" class=\"note\">Blue fill is sampled d &lt;= 0. This is diagnostic evidence, not a mesh or geometric certificate.</text>\n"
    );
    let mut evals = 0usize;
    for (idx, panel) in panels.iter().copied().enumerate() {
        let (part, n) = svg_panel(sdf, panel, 20.0 + idx as f32 * 400.0, 86.0, 360.0, 300.0);
        evals += n;
        svg.push_str(&part);
    }
    svg.push_str("</svg>\n");
    fs::write(path, svg).map_err(|e| format!("write {}: {e}", path.display()))?;
    Ok(evals)
}

fn main() -> Result<(), String> {
    let mut args = std::env::args_os().skip(1);
    let script_path = PathBuf::from(
        args.next()
            .ok_or("usage: probe <script.rhai> <output-dir> <current-result.json>")?,
    );
    let output_dir = PathBuf::from(
        args.next()
            .ok_or("usage: probe <script.rhai> <output-dir> <current-result.json>")?,
    );
    let current_result_path = PathBuf::from(
        args.next()
            .ok_or("usage: probe <script.rhai> <output-dir> <current-result.json>")?,
    );
    if args.next().is_some() {
        return Err("usage: probe <script.rhai> <output-dir> <current-result.json>".to_string());
    }
    fs::create_dir_all(&output_dir).map_err(|e| format!("create output: {e}"))?;
    let source = fs::read_to_string(&script_path).map_err(|e| format!("read script: {e}"))?;

    let current_execution: CurrentExecution = serde_json::from_str(
        &fs::read_to_string(&current_result_path)
            .map_err(|e| format!("read current-root result: {e}"))?,
    )
    .map_err(|e| format!("parse current-root result: {e}"))?;

    let started = Instant::now();
    let historical = implicit_historical::scripting::evaluate_script(&source)
        .map_err(|e| format!("historical evaluator failed: {e}"))?;
    let historical_elapsed = started.elapsed().as_secs_f64();
    let sdf = historical.sdf;

    let started = Instant::now();
    let reload = implicit_historical::scripting::evaluate_script(&source)
        .map_err(|e| format!("historical evaluator reload failed: {e}"))?;
    let reload_points = [
        Vec3::new(230.0, 0.0, 0.0),
        Vec3::new(300.0, 300.0, 0.0),
        Vec3::new(323.78223, 350.0, 0.0),
        Vec3::new(350.0, 350.0, 20.0),
        Vec3::new(380.0, 350.0, 50.0),
        Vec3::new(420.0, 350.0, 0.0),
        Vec3::new(350.0, 360.0, 0.0),
        Vec3::new(350.0, 340.0, 0.0),
        Vec3::new(350.0, -350.0, 20.0),
        Vec3::new(350.0, -360.0, 0.0),
        Vec3::new(100.0, 0.0, 0.0),
        Vec3::new(600.0, 0.0, 0.0),
    ];
    let mut reload_matches = 0usize;
    let mut reload_max_abs = 0.0f32;
    for p in reload_points {
        let a = sdf.distance(p);
        let b = reload.sdf.distance(p);
        if a.to_bits() == b.to_bits() {
            reload_matches += 1;
        }
        reload_max_abs = reload_max_abs.max((a - b).abs());
    }
    let _reload_elapsed = started.elapsed().as_secs_f64();

    let panels = [
        Panel {
            name: "Complete aircraft planform (Z = 0)",
            u_label: "X",
            v_label: "Y",
            u0: -80.0,
            u1: 780.0,
            v0: -420.0,
            v1: 420.0,
            nu: 121,
            nv: 91,
            map: |x, y| Vec3::new(x, y, 0.0),
        },
        Panel {
            name: "Planform near +Y wingtip (Z = 0)",
            u_label: "X",
            v_label: "Y",
            u0: 260.0,
            u1: 455.0,
            v0: 285.0,
            v1: 380.0,
            nu: 121,
            nv: 91,
            map: |x, y| Vec3::new(x, y, 0.0),
        },
        Panel {
            name: "Vertical wingtip section (Y = 350)",
            u_label: "X",
            v_label: "Z",
            u0: 290.0,
            u1: 455.0,
            v0: -75.0,
            v1: 75.0,
            nu: 121,
            nv: 91,
            map: |x, z| Vec3::new(x, 350.0, z),
        },
        Panel {
            name: "Root blend section (X = 350)",
            u_label: "Y",
            v_label: "Z",
            u0: 315.0,
            u1: 385.0,
            v0: -75.0,
            v1: 75.0,
            nu: 121,
            nv: 91,
            map: |y, z| Vec3::new(350.0, y, z),
        },
    ];
    let mut panel_results = Vec::new();
    let mut evals = reload_points.len() * 2;
    for panel in panels {
        let (result, _, n) = inspect_panel(sdf.as_ref(), panel);
        evals += n;
        panel_results.push(result);
    }
    evals += write_svg(
        &output_dir.join("BUILTIN_WING_WORKFLOW_SECTIONS.svg"),
        sdf.as_ref(),
        &panels,
    )?;

    let inner_zero_source = variant_source(
        &source,
        "let wingtip_inner_fillet_radius_mm = 7.000000;",
        "let wingtip_inner_fillet_radius_mm = 0.000000;",
    )?;
    let outer_zero_source = variant_source(
        &source,
        "let wingtip_outer_fillet_radius_mm = 12.800000;",
        "let wingtip_outer_fillet_radius_mm = 1.000000;",
    )?;
    let inner_zero = implicit_historical::scripting::evaluate_script(&inner_zero_source)
        .map_err(|e| format!("inner-zero control variant failed: {e}"))?;
    let outer_zero = implicit_historical::scripting::evaluate_script(&outer_zero_source)
        .map_err(|e| format!("outer-zero control variant failed: {e}"))?;
    let (inner_diff, n) = compare_variant(
        sdf.as_ref(),
        inner_zero.sdf.as_ref(),
        "inner_fillet_radius_mm: 7.0 -> 0.0",
    );
    evals += n;
    let (outer_diff, n) = compare_variant(
        sdf.as_ref(),
        outer_zero.sdf.as_ref(),
        "outer_fillet_radius_mm: 12.8 -> 1.0",
    );
    evals += n;

    let bounds = sdf.bounds().map(|b| BoundsResult {
        min_mm: b.min.to_array(),
        max_mm: b.max.to_array(),
        note: "Conservative SDF-tree bounds; not a measured occupied extent.",
    });
    let result = ProbeResult {
        schema: "wing-workflow-reproduction-probe-v1",
        exact_script_path: script_path.display().to_string(),
        exact_script_bytes: source.len(),
        current_root_execution: current_execution.execution,
        current_root_elapsed_seconds: current_execution.elapsed_seconds,
        historical_execution: "SUCCESS through dual_contouring scripting::evaluate_script"
            .to_string(),
        historical_elapsed_seconds: historical_elapsed,
        historical_sdf_type: sdf.sdf_type_name().to_string(),
        historical_bounds: bounds,
        source_reload_sample_count: reload_points.len(),
        source_reload_bitwise_match_count: reload_matches,
        source_reload_max_abs_difference_mm: reload_max_abs,
        airfoil_sections: vec![
            airfoil_result("2412", 170.0, 1.5, 1.5),
            airfoil_result("2412", 65.0, 1.5, 1.5),
            airfoil_result("0010", 58.0, 1.5, 1.5),
            airfoil_result("0010", 36.0, 1.5, 1.5),
        ],
        panels: panel_results,
        control_variants: vec![inner_diff, outer_diff],
        sdf_evaluations: evals,
        limitations: vec![
            "The curated script is unchanged; only control variants replace one exact assignment each.",
            "Panel extents are discrete signed-field samples, not analytic extrema.",
            "Boundary roots use 40 bisection iterations on sign-changing panel rows.",
            "Gradient magnitudes use central differences with h = 0.01 mm and are diagnostic only.",
            "The visualization samples the signed field directly; it is not a surface mesh.",
            "Source reload equality is checked only at the listed 12 points.",
        ],
    };
    let json = serde_json::to_string_pretty(&result).map_err(|e| format!("serialize: {e}"))?;
    fs::write(output_dir.join("PROBE_RESULT.json"), format!("{json}\n"))
        .map_err(|e| format!("write result: {e}"))?;
    println!("{json}");
    Ok(())
}
