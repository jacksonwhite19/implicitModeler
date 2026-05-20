use glam::Vec3;
use implicit_cad::export::dual_contouring::{DualContouringTelemetry, analyze_mesh_topology};
use implicit_cad::export::{
    ExportBackend, build_export_mesh, build_export_mesh_dual_contouring_adaptive_with_telemetry,
    build_export_mesh_with_backend, export_stl,
};
use implicit_cad::mesh::{Mesh, MeshQuality};
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::aerospace::{AirfoilExportOptions, wing_with_airfoil_export_safe};
use implicit_cad::sdf::booleans::{Intersect, Subtract, Union};
use implicit_cad::sdf::primitives::SdfBox;
use implicit_cad::sdf::transforms::{Shell, Translate};
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn make_wing_fixture() -> Arc<dyn Sdf> {
    let outer: Arc<dyn Sdf> = Arc::new(wing_with_airfoil_export_safe(
        "2212",
        180.0,
        110.0,
        700.0,
        0.0,
        0.0,
        0.0,
        AirfoilExportOptions {
            min_trailing_edge_thickness_mm: 4.0,
            min_leading_edge_radius_mm: 2.0,
        },
    ));
    let wall_mm = 2.0;
    let span_mm = 700.0;
    let tip_cap_thickness_mm = wall_mm * 2.0;
    let hollow_shell: Arc<dyn Sdf> = Arc::new(Shell::new(Arc::clone(&outer), wall_mm));
    let cap_slab: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(
        5_000.0,
        tip_cap_thickness_mm * 0.5,
        5_000.0,
    )));
    let positive_tip_cap: Arc<dyn Sdf> = Arc::new(Intersect::new(
        Arc::clone(&outer),
        Arc::new(Translate::new(
            Arc::clone(&cap_slab),
            Vec3::new(0.0, span_mm * 0.5 - tip_cap_thickness_mm * 0.5, 0.0),
        )),
    ));
    let negative_tip_cap: Arc<dyn Sdf> = Arc::new(Intersect::new(
        outer,
        Arc::new(Translate::new(
            cap_slab,
            Vec3::new(0.0, -span_mm * 0.5 + tip_cap_thickness_mm * 0.5, 0.0),
        )),
    ));

    Arc::new(Union::new(
        Arc::new(Union::new(hollow_shell, positive_tip_cap)),
        negative_tip_cap,
    ))
}

fn mesh_bounds(mesh: &Mesh) -> Option<(Vec3, Vec3)> {
    let mut vertices = mesh.vertices.iter();
    let first = vertices.next()?;
    let mut min = Vec3::from_array(first.position);
    let mut max = min;
    for vertex in vertices {
        let p = Vec3::from_array(vertex.position);
        min = min.min(p);
        max = max.max(p);
    }
    Some((min, max))
}

fn print_report(
    label: &str,
    mesh: &Mesh,
    path: &Path,
    build_time: Duration,
    write_time: Duration,
) -> Result<(), Box<dyn Error>> {
    let topology = analyze_mesh_topology(mesh);
    let bytes = fs::metadata(path)?.len();
    let (bounds_min, bounds_max) =
        mesh_bounds(mesh).unwrap_or((Vec3::splat(f32::NAN), Vec3::splat(f32::NAN)));
    println!(
        "{label}: build_ms={} write_ms={} total_ms={} vertices={} triangles={} file_bytes={} boundary_edges={} non_manifold_edges={} degenerate_triangles={} duplicate_triangles={} bounds_min=({:.3},{:.3},{:.3}) bounds_max=({:.3},{:.3},{:.3}) path={}",
        build_time.as_millis(),
        write_time.as_millis(),
        (build_time + write_time).as_millis(),
        mesh.vertices.len(),
        mesh.indices.len() / 3,
        bytes,
        topology.boundary_edges,
        topology.non_manifold_edges,
        topology.degenerate_triangles,
        topology.duplicate_triangles,
        bounds_min.x,
        bounds_min.y,
        bounds_min.z,
        bounds_max.x,
        bounds_max.y,
        bounds_max.z,
        path.display()
    );
    Ok(())
}

fn percent(count: usize, total: usize) -> f32 {
    if total == 0 {
        0.0
    } else {
        count as f32 * 100.0 / total as f32
    }
}

fn write_telemetry_report(
    telemetry: &DualContouringTelemetry,
    total_time: Duration,
) -> Result<(), Box<dyn Error>> {
    let mut depths: Vec<_> = telemetry.leaf_counts_per_depth.iter().collect();
    depths.sort_by_key(|(depth, _)| **depth);
    let depth_lines = depths
        .into_iter()
        .map(|(depth, count)| format!("  - Depth {depth}: {count} leaves"))
        .collect::<Vec<_>>()
        .join("\n");
    let report = format!(
        concat!(
            "# Dual Contouring Exporter Telemetry Report\n\n",
            "## Execution Profile\n",
            "- **Total Execution Time:** {} ms\n",
            "  - Octree Leaf Traversal: {} ms\n",
            "  - QEF Optimization Pass: {} ms\n",
            "  - Face Emission & Indexing: {} ms\n\n",
            "## Octree Structure\n",
            "- **Total Active Leaf Nodes:** {}\n",
            "- **Depth Distribution:**\n{}\n\n",
            "## QEF Solver Integrity (Faceting Diagnostic)\n",
            "- **Total Optimizations Attempted:** {}\n",
            "- **Successful Continuous Fits:** {} ({:.2}%)\n",
            "- **Ill-Conditioned Fallbacks (Snapped to Mass Center):** {} ({:.2}%) \n",
            "- **Aggressive Bounds Clamps Triggered:** {} ({:.2}%)\n\n",
            "## Geometric Fidelity\n",
            "- **Active Edge Crossings Detected:** {}\n",
            "- **Degenerate Roots (Hard Midpoints):** {}\n",
            "- **Grid-Snapped/Axis-Aligned Normals:** {}\n",
            "- **Degenerate Gradient Guards Triggered:** {}\n",
        ),
        total_time.as_millis(),
        telemetry.duration_octree_walk_ms,
        telemetry.duration_qef_solve_ms,
        telemetry.duration_edge_emission_ms,
        telemetry.total_leaf_nodes,
        if depth_lines.is_empty() {
            String::from("  - Depth 0: 0 leaves")
        } else {
            depth_lines
        },
        telemetry.qef_total_solves,
        telemetry.qef_success_count,
        percent(telemetry.qef_success_count, telemetry.qef_total_solves),
        telemetry.qef_singular_matrix_fallbacks,
        percent(
            telemetry.qef_singular_matrix_fallbacks,
            telemetry.qef_total_solves
        ),
        telemetry.qef_out_of_bounds_clamped,
        percent(
            telemetry.qef_out_of_bounds_clamped,
            telemetry.qef_total_solves
        ),
        telemetry.total_active_edges,
        telemetry.exact_midpoint_fallbacks,
        telemetry.axis_snapped_normals,
        telemetry.degenerate_gradient_guards,
    );
    use std::io::Write;
    let mut file = fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open("output.md")?;
    file.write_all(report.as_bytes())?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut backend = String::from("current");
    let mut target_cell_mm = 1.0;
    let mut script_path: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--backend" => {
                backend = args
                    .next()
                    .ok_or("--backend requires current, uniform-dc, legacy, or both")?;
            }
            "--cell" => {
                target_cell_mm = args
                    .next()
                    .ok_or("--cell requires a millimeter value")?
                    .parse::<f32>()?;
            }
            "--script" => {
                script_path = Some(args.next().ok_or("--script requires a path")?);
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }

    fs::write("output.md", "# Vertex-Index Integrity Audit\n\n")?;

    let (shape, bounds_min, bounds_max, current_path) = if let Some(path) = script_path {
        if path.ends_with("wingTest.rhai") {
            let script_path = Path::new(&path);
            let output_dir = script_path.parent().unwrap_or_else(|| Path::new("."));
            (
                make_wing_fixture(),
                Vec3::new(-20.0, -370.0, -45.0),
                Vec3::new(200.0, 370.0, 45.0),
                output_dir.join("wingTest.stl"),
            )
        } else if path.ends_with("rectangular_prism_shell_400x650x200_2mm.rhai") {
            let script_path = Path::new(&path);
            let output_dir = script_path.parent().unwrap_or_else(|| Path::new("."));
            let output_file = output_dir.join("cubeTest.stl");
            let outer: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(200.0, 325.0, 100.0)));
            let inner: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(198.0, 323.0, 98.0)));
            (
                Arc::new(Subtract::new(outer, inner)) as Arc<dyn Sdf>,
                Vec3::new(-205.0, -330.0, -105.0),
                Vec3::new(205.0, 330.0, 105.0),
                output_file,
            )
        } else {
            return Err(format!("unsupported benchmark --script fixture: {path}").into());
        }
    } else {
        (
            make_wing_fixture(),
            Vec3::new(-20.0, -370.0, -45.0),
            Vec3::new(200.0, 370.0, 45.0),
            Path::new("wingTest.stl").to_path_buf(),
        )
    };
    let extent = bounds_max - bounds_min;
    let edge_scan_estimate = (extent.x / target_cell_mm).ceil() as u64
        * (extent.y / target_cell_mm).ceil() as u64
        * (extent.z / target_cell_mm).ceil() as u64
        * 3;
    println!(
        "fixture_bounds=({:.1},{:.1},{:.1})..({:.1},{:.1},{:.1}) target_cell_mm={target_cell_mm:.3} finest_edge_scan_estimate={edge_scan_estimate}",
        bounds_min.x, bounds_min.y, bounds_min.z, bounds_max.x, bounds_max.y, bounds_max.z
    );

    let script_relative_paths = current_path
        .file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| name == "cubeTest.stl");
    let sibling_output = |script_name: &str, default_name: &str| -> PathBuf {
        if script_relative_paths {
            current_path.with_file_name(script_name)
        } else {
            Path::new(default_name).to_path_buf()
        }
    };
    let legacy_path = sibling_output("cubeTest_legacy.stl", "wingTest_legacy.stl");

    if backend == "current" || backend == "both" {
        let build_start = Instant::now();
        let (current, mut telemetry) = build_export_mesh_dual_contouring_adaptive_with_telemetry(
            shape.as_ref(),
            bounds_min,
            bounds_max,
            target_cell_mm,
        );
        let current_build_time = build_start.elapsed();
        let write_start = Instant::now();
        export_stl(
            &current,
            current_path.to_str().ok_or("invalid current path")?,
        )?;
        let current_write_time = write_start.elapsed();
        telemetry.duration_write_io_ms = current_write_time.as_millis();
        print_report(
            "current_adaptive_dual_contouring",
            &current,
            &current_path,
            current_build_time,
            current_write_time,
        )?;
        write_telemetry_report(&telemetry, current_build_time + current_write_time)?;
    }

    if backend == "uniform-dc" || backend == "both" {
        let uniform_path = sibling_output("cubeTest_uniform_dc.stl", "wingTest_uniform_dc.stl");
        let build_start = Instant::now();
        let uniform = build_export_mesh_with_backend(
            shape.as_ref(),
            bounds_min,
            bounds_max,
            target_cell_mm,
            ExportBackend::UniformDualContouring,
            false,
        )
        .mesh;
        let uniform_build_time = build_start.elapsed();
        let write_start = Instant::now();
        export_stl(
            &uniform,
            uniform_path.to_str().ok_or("invalid uniform path")?,
        )?;
        let uniform_write_time = write_start.elapsed();
        print_report(
            "uniform_dual_contouring_reference",
            &uniform,
            &uniform_path,
            uniform_build_time,
            uniform_write_time,
        )?;
    }

    if backend == "legacy" || backend == "both" {
        let build_start = Instant::now();
        let legacy = build_export_mesh(
            shape.as_ref(),
            bounds_min,
            bounds_max,
            MeshQuality::Normal,
            false,
        );
        let legacy_build_time = build_start.elapsed();
        let write_start = Instant::now();
        export_stl(&legacy, legacy_path.to_str().ok_or("invalid legacy path")?)?;
        let legacy_write_time = write_start.elapsed();
        print_report(
            "legacy_uniform_marching_cubes",
            &legacy,
            &legacy_path,
            legacy_build_time,
            legacy_write_time,
        )?;
    }

    if backend != "current" && backend != "uniform-dc" && backend != "legacy" && backend != "both" {
        return Err("--backend must be current, uniform-dc, legacy, or both".into());
    }

    Ok(())
}
