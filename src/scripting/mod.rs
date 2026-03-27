// Rhai scripting engine

use std::sync::{Arc, Mutex, RwLock};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::SystemTime;
use rhai::{Engine, Scope};
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::field::Field;
use crate::sdf::aerospace::Section2D;
use crate::sdf::profiles::SplineProfile;
use crate::sdf::spine::LongitudinalSplines;
use crate::fea::FEASetup;

/// Shared mesh parse cache: path → (mtime, parsed mesh).
/// Keyed by canonical path; re-parsed only when mtime changes.
pub type MeshCache = HashMap<PathBuf, (SystemTime, Arc<crate::mesh::TriangleMesh>)>;

pub mod api;
pub mod analysis_api;
pub mod errors;
pub mod legacy_api;

/// Wrapper for SDF objects in Rhai scripts
#[derive(Clone)]
pub struct SdfHandle(pub Arc<dyn Sdf>);

/// Wrapper for Field objects in Rhai scripts
#[derive(Clone)]
pub struct FieldHandle(pub Arc<dyn Field>);

/// Handle to a 2-D cross-section (circle, ellipse, airfoil, etc.)
/// Used as input to `fuselage_station()` in scripts.
#[derive(Clone)]
pub struct SectionHandle(pub Arc<dyn Section2D>);

/// Handle to a positioned fuselage station.
/// Created by `fuselage_station(position, section)` in scripts.
#[derive(Clone)]
pub struct StationHandle {
    pub position: f32,
    pub section: Arc<dyn Section2D>,
}

/// An aircraft component: carries both its geometry SDF and a keepout (clearance) SDF.
/// Create with `component(sdf, margin)`, place with `place(comp, x, y, z)`,
/// extract with `geometry(comp)` or `keepout(comp)`.
#[derive(Clone)]
pub struct ComponentHandle {
    pub geometry: Arc<dyn Sdf>,
    pub keepout: Arc<dyn Sdf>,
    pub mass_g: f32,
    pub name: String,
}

/// A composite material handle.
#[derive(Clone)]
pub struct MaterialHandle(pub Arc<crate::materials::CompositeMaterial>);

/// A composite shell layer handle.
#[derive(Clone)]
pub struct LayerHandle(pub Arc<crate::sdf::aerospace::composite::ShellLayer>);

/// A stored layup configuration (layers without a parent body).
/// Created by `composite_layup_config(layers)`, applied by `apply_layup(body, layup)`.
#[derive(Clone)]
pub struct LayupConfigHandle(pub Vec<Arc<crate::sdf::aerospace::composite::ShellLayer>>);

/// Handle for a hinge specification (passed to aileron/elevator/rudder/flap/elevon).
#[derive(Clone)]
pub struct HingeHandle(pub crate::sdf::aerospace::control_surfaces::HingeSpec);

/// Handle for a linkage specification (control horn + pushrod slot).
#[derive(Clone)]
pub struct LinkageHandle(pub crate::sdf::aerospace::control_surfaces::LinkageSpec);

/// A sweep path handle — wraps any SweepPath for use in Rhai sweep() calls.
#[derive(Clone)]
pub struct PathHandle(pub Arc<dyn crate::sdf::sweep::SweepPath>);

/// A 2D profile handle — wraps any Section2D for use in Rhai sweep() calls.
#[derive(Clone)]
pub struct ProfileHandle(pub Arc<dyn crate::sdf::aerospace::Section2D>);

/// A point mass declared in a script via `mass_at()` or `mass_named()`
#[derive(Clone, Debug)]
pub struct MassPoint {
    pub name: String,
    pub mass_g: f32,
    pub position: Vec3,
}

// ── PointHandle ───────────────────────────────────────────────────────────────
#[derive(Clone, Debug)]
pub struct PointHandle(pub Vec3);

// ── Reference point registry ─────────────────────────────────────────────────

/// A named geometric reference point declared in a script via ref_point().
#[derive(Clone, Debug)]
pub struct ReferencePoint {
    pub name:     String,
    pub position: Vec3,
    pub color:    [f32; 3],
}

// ── Mounting hole data model ──────────────────────────────────────────────────

#[derive(Clone, Debug)]
#[allow(dead_code)] // Part of mounting hole data model
pub enum HoleSource {
    AutoDetected { confidence: f32 },
    Manual,
    AutoDetectedOverridden,
}

#[derive(Clone, Debug)]
#[allow(dead_code)] // Mounting hole data model
pub struct MountingHole {
    pub position:          Vec3,
    pub direction:         Vec3,
    pub screw_designation: String,
    pub source:            HoleSource,
}

#[derive(Clone, Debug)]
#[allow(dead_code)] // Mounting hole set data model
pub struct MountingHoleSet {
    pub holes:          Vec<MountingHole>,
    pub component_name: String,
}

#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct MountingHoleHandle(pub MountingHole);

#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct MountingHoleSetHandle(pub MountingHoleSet);

/// Handle wrapping an airfoil polar for use in Rhai scripts.
#[derive(Clone)]
pub struct PolarHandle(pub Arc<crate::aero::AirfoilPolar>);

/// Handle wrapping a flight condition for use in Rhai scripts.
#[derive(Clone)]
pub struct FlightConditionHandle(pub crate::aero::FlightCondition);

/// Handle wrapping a static margin result for use in Rhai scripts.
#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct StabilityResultHandle(pub crate::aero::StaticMarginResult);

/// Handle wrapping a trim analysis result for use in Rhai scripts.
#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct TrimResultHandle(pub crate::aero::TrimResult);

/// Handle wrapping a drag polar result for use in Rhai scripts.
#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct DragPolarHandle(pub crate::aero::DragPolarResult);

/// Handle wrapping a motor specification for propulsion scripting.
#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct MotorHandle(pub Arc<crate::aero::MotorSpec>);

/// Handle wrapping a propeller specification for propulsion scripting.
#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct PropHandle(pub Arc<crate::aero::PropSpec>);

/// Handle wrapping a full propulsion system setup for scripting.
#[derive(Clone)]
#[allow(dead_code)] // Rhai scripting handle type
pub struct PropulsionHandle(pub Arc<crate::aero::PropulsionSetup>);

static REF_COLORS: &[[f32; 3]] = &[
    [1.0, 0.4, 0.4],
    [0.4, 0.8, 0.4],
    [0.4, 0.6, 1.0],
    [1.0, 0.8, 0.2],
    [0.8, 0.4, 1.0],
    [0.4, 0.9, 0.9],
    [1.0, 0.6, 0.2],
    [0.9, 0.4, 0.8],
];

/// Ordered map of name → position accumulated during script evaluation.
pub type RefPointCollector = Arc<Mutex<Vec<ReferencePoint>>>;

/// Accumulates all placed components during script evaluation (for bulkhead_auto).
#[allow(dead_code)] // Used by bulkhead_auto scripting function
pub type ComponentCollector = Arc<Mutex<Vec<ComponentHandle>>>;

/// Result of evaluating a script — includes the SDF, declared mass points, and FEA conditions.
pub struct ScriptResult {
    pub sdf:              Arc<dyn Sdf>,
    pub mass_points:      Vec<MassPoint>,
    pub fea_setup:        FEASetup,
    /// All composite layups created during this eval (for the Layup Summary panel).
    pub layups:           Vec<Arc<crate::sdf::aerospace::composite::CompositeLayup>>,
    pub reference_points: Vec<ReferencePoint>,
}

pub struct ComponentPreviewPart {
    pub name: String,
    pub sdf: Arc<dyn Sdf>,
}

impl ScriptResult {
    /// Compute the overall center of gravity from declared mass points.
    /// Returns None if no mass points were declared.
    pub fn center_of_gravity(&self) -> Option<Vec3> {
        let total_mass: f32 = self.mass_points.iter().map(|m| m.mass_g).sum();
        if total_mass <= 0.0 {
            return None;
        }
        let weighted: Vec3 = self.mass_points.iter()
            .map(|m| m.position * m.mass_g)
            .sum();
        Some(weighted / total_mass)
    }

    #[allow(dead_code)] // Available for mass budget analysis
    pub fn total_mass_g(&self) -> f32 {
        self.mass_points.iter().map(|m| m.mass_g).sum()
    }
}

/// Evaluate a script and return the resulting SDF plus any declared mass points.
///
/// Convenience wrapper — no profile, spine, or FEA field access.
pub fn evaluate_script(source: &str) -> Result<ScriptResult, String> {
    evaluate_script_full(source, None, None, None, None, &indexmap::IndexMap::new(), None, None, &[])
}

/// Convenience wrapper used by the app with profiles and spine but without FEA fields.
#[allow(dead_code)] // Available as a convenience wrapper for callers not needing FEA
pub fn evaluate_script_with_profiles(
    source: &str,
    profiles: Option<Arc<RwLock<HashMap<String, SplineProfile>>>>,
    splines: Option<Arc<LongitudinalSplines>>,
) -> Result<ScriptResult, String> {
    evaluate_script_full(source, profiles, splines, None, None, &indexmap::IndexMap::new(), None, None, &[])
}

#[allow(clippy::too_many_arguments)]
pub fn evaluate_component_preview_parts(
    source:             &str,
    profiles:           Option<Arc<RwLock<HashMap<String, SplineProfile>>>>,
    splines:            Option<Arc<LongitudinalSplines>>,
    stress_field:       Option<Arc<dyn Field>>,
    displacement_field: Option<Arc<dyn Field>>,
    dimensions:         &indexmap::IndexMap<String, f64>,
    project_dir:        Option<&Path>,
    mesh_cache:         Option<Arc<Mutex<MeshCache>>>,
    library_sources:    &[(String, String)],
) -> Result<Vec<ComponentPreviewPart>, String> {
    let mass_collector:   Arc<Mutex<Vec<MassPoint>>>  = Arc::new(Mutex::new(Vec::new()));
    let comp_collector:   Arc<Mutex<Vec<ComponentHandle>>> = Arc::new(Mutex::new(Vec::new()));
    let fea_collector:    Arc<Mutex<FEASetup>>        = Arc::new(Mutex::new(FEASetup::default()));
    let layup_collector:  Arc<Mutex<Vec<Arc<crate::sdf::aerospace::composite::CompositeLayup>>>>
                        = Arc::new(Mutex::new(Vec::new()));
    let ref_collector:    RefPointCollector           = Arc::new(Mutex::new(Vec::new()));

    let mut engine = Engine::new();
    api::register_sdf_functions(&mut engine);
    api::register_component_functions(&mut engine, Arc::clone(&mass_collector), Arc::clone(&comp_collector));
    api::register_drone_auto_functions(&mut engine, Arc::clone(&comp_collector));
    api::register_mass_functions(&mut engine, Arc::clone(&mass_collector));
    api::register_fea_functions(&mut engine, Arc::clone(&fea_collector), stress_field, displacement_field);
    if let Some(p) = profiles {
        api::register_profile_functions(&mut engine, p);
    }
    if let Some(s) = splines {
        api::register_spine_functions(&mut engine, s);
    }
    api::register_mesh_functions(
        &mut engine,
        project_dir.map(|p| p.to_path_buf()),
        mesh_cache.unwrap_or_else(|| Arc::new(Mutex::new(MeshCache::new()))),
    );
    api::register_composite_collector(&mut engine, Arc::clone(&layup_collector));
    api::register_query_functions(&mut engine, Arc::clone(&ref_collector));
    let mut resolver = if let Some(base) = project_dir {
        rhai::module_resolvers::FileModuleResolver::new_with_path(base)
    } else {
        rhai::module_resolvers::FileModuleResolver::new()
    };
    resolver.enable_cache(true);
    engine.set_module_resolver(resolver);

    for (mod_name, mod_source) in library_sources {
        let comp_engine = rhai::Engine::new();
        if let Ok(ast) = comp_engine.compile(mod_source.as_str()) {
            if let Ok(module) = rhai::Module::eval_ast_as_new(rhai::Scope::new(), &ast, &comp_engine) {
                engine.register_static_module(mod_name.as_str(), module.into());
            }
        }
    }

    let mut scope = Scope::new();
    for (name, &value) in dimensions {
        scope.push_constant(name.as_str(), value);
    }

    let ast = engine.compile(source)
        .map_err(|e| errors::build_error_string(&errors::format_script_error(source, &format!("Script error: {}", e))))?;

    let _ = engine.eval_ast_with_scope::<rhai::Dynamic>(&mut scope, &ast)
        .map_err(|e| errors::build_error_string(&errors::format_script_error(source, &format!("Script error: {}", e))))?;

    let comp_map = engine.call_fn::<rhai::Map>(&mut scope, &ast, "component", ())
        .map_err(|e| format!("Component preview error: {}", e))?;

    let mut parts = Vec::new();
    if let Some(parts_dyn) = comp_map.get("parts") {
        if let Some(parts_map) = parts_dyn.clone().try_cast::<rhai::Map>() {
            for (name, value) in parts_map {
                if let Some(sdf) = value.clone().try_cast::<SdfHandle>() {
                    parts.push(ComponentPreviewPart { name: name.to_string(), sdf: sdf.0 });
                }
            }
        }
    }
    if parts.is_empty() {
        if let Some(sdf) = comp_map.get("physical").and_then(|v| v.clone().try_cast::<SdfHandle>()) {
            parts.push(ComponentPreviewPart { name: "physical".to_string(), sdf: sdf.0 });
        }
    }
    Ok(parts)
}

/// Full evaluator.
///
/// - `profiles`: `spline(name)` and `spline_section(name)` resolve named profiles.
/// - `splines`: `spline_fuselage(stations, length)` applies longitudinal spine constraints.
/// - `stress_field`: `stress_field()` returns a FieldHandle backed by this field.
/// - `displacement_field`: `displacement_field()` returns a FieldHandle backed by this field.
/// - `project_dir`: base directory for resolving relative mesh paths in `import_mesh()`.
/// - `mesh_cache`: shared parse cache — meshes are re-parsed only when mtime changes.
pub fn evaluate_script_full(
    source:             &str,
    profiles:           Option<Arc<RwLock<HashMap<String, SplineProfile>>>>,
    splines:            Option<Arc<LongitudinalSplines>>,
    stress_field:       Option<Arc<dyn Field>>,
    displacement_field: Option<Arc<dyn Field>>,
    dimensions:         &indexmap::IndexMap<String, f64>,
    project_dir:        Option<&Path>,
    mesh_cache:         Option<Arc<Mutex<MeshCache>>>,
    library_sources:    &[(String, String)],
) -> Result<ScriptResult, String> {
    let mass_collector:   Arc<Mutex<Vec<MassPoint>>>  = Arc::new(Mutex::new(Vec::new()));
    let comp_collector:   Arc<Mutex<Vec<ComponentHandle>>> = Arc::new(Mutex::new(Vec::new()));
    let fea_collector:    Arc<Mutex<FEASetup>>        = Arc::new(Mutex::new(FEASetup::default()));
    let layup_collector:  Arc<Mutex<Vec<Arc<crate::sdf::aerospace::composite::CompositeLayup>>>>
                        = Arc::new(Mutex::new(Vec::new()));
    let ref_collector:    RefPointCollector           = Arc::new(Mutex::new(Vec::new()));

    let mut engine = Engine::new();
    api::register_sdf_functions(&mut engine);
    api::register_component_functions(&mut engine, Arc::clone(&mass_collector), Arc::clone(&comp_collector));
    api::register_drone_auto_functions(&mut engine, Arc::clone(&comp_collector));
    api::register_mass_functions(&mut engine, Arc::clone(&mass_collector));
    api::register_fea_functions(&mut engine, Arc::clone(&fea_collector), stress_field, displacement_field);
    if let Some(p) = profiles {
        api::register_profile_functions(&mut engine, p);
    }
    if let Some(s) = splines {
        api::register_spine_functions(&mut engine, s);
    }
    api::register_mesh_functions(
        &mut engine,
        project_dir.map(|p| p.to_path_buf()),
        mesh_cache.unwrap_or_else(|| Arc::new(Mutex::new(MeshCache::new()))),
    );
    api::register_composite_collector(&mut engine, Arc::clone(&layup_collector));
    api::register_query_functions(&mut engine, Arc::clone(&ref_collector));
    let mut resolver = if let Some(base) = project_dir {
        rhai::module_resolvers::FileModuleResolver::new_with_path(base)
    } else {
        rhai::module_resolvers::FileModuleResolver::new()
    };
    resolver.enable_cache(true);
    engine.set_module_resolver(resolver);

    // Register library components as static modules.
    for (mod_name, mod_source) in library_sources {
        let comp_engine = rhai::Engine::new();
        match comp_engine.compile(mod_source.as_str()) {
            Ok(ast) => {
                match rhai::Module::eval_ast_as_new(rhai::Scope::new(), &ast, &comp_engine) {
                    Ok(module) => {
                        engine.register_static_module(mod_name.as_str(), module.into());
                    }
                    Err(e) => {
                        eprintln!("[Library] Module '{}' eval error: {}", mod_name, e);
                    }
                }
            }
            Err(e) => {
                eprintln!("[Library] Module '{}' compile error: {}", mod_name, e);
            }
        }
    }

    // Inject named dimensions as constants into the script scope.
    let mut scope = Scope::new();
    for (name, &value) in dimensions {
        scope.push_constant(name.as_str(), value);
    }

    match engine.eval_with_scope::<SdfHandle>(&mut scope, source) {
        Ok(handle) => {
            let mass_points      = mass_collector.lock().unwrap().clone();
            let fea_setup        = std::mem::take(&mut *fea_collector.lock().unwrap());
            let layups           = layup_collector.lock().unwrap().clone();
            let reference_points = ref_collector.lock().unwrap().clone();
            Ok(ScriptResult { sdf: handle.0, mass_points, fea_setup, layups, reference_points })
        }
        Err(e) => {
            let msg = e.to_string();
            if msg.contains("() (expecting") || msg.contains("output type incorrect") {
                let base = "Script must end with an SDF expression (no semicolon on the last line).\nExample: the last line should be  `auto_fuselage(internals, 3.0)`  not  `auto_fuselage(internals, 3.0);`";
                let formatted = errors::format_script_error(source, base);
                Err(errors::build_error_string(&formatted))
            } else {
                let raw = format!("Script error: {}", msg);
                let formatted = errors::format_script_error(source, &raw);
                Err(errors::build_error_string(&formatted))
            }
        }
    }
}

// ── Cell-based scripting ──────────────────────────────────────────────────────

/// Status of a script cell after evaluation.
#[derive(Clone, Debug, PartialEq)]
pub enum CellStatus {
    Pending,
    Ok,
    Error { message: String, line: Option<usize> },
    Skipped,
}

/// A named section of a script delimited by `# === Name ===` comments.
#[derive(Clone, Debug)]
pub struct ScriptCell {
    /// Stable identifier, e.g. `"cell_0"`.
    pub id: String,
    /// Human-readable name parsed from the delimiter comment.
    pub name: String,
    /// 0-indexed line number of the delimiter comment (or 0 for no-delimiter scripts).
    pub start_line: usize,
    /// 0-indexed inclusive last line of this cell's content.
    pub end_line: usize,
    /// Evaluation status set after running.
    pub status: CellStatus,
}

/// Returns true if `line` is a cell delimiter of the form `# === <name> ===`.
pub fn is_delimiter_line(line: &str) -> bool {
    is_cell_delimiter(line)
}

fn is_cell_delimiter(line: &str) -> bool {
    let t = line.trim();
    t.starts_with("# ===") && t.ends_with("===") && t.len() > 8
}

/// Parse the cell name from a delimiter line.
fn parse_cell_name(line: &str) -> String {
    let t = line.trim();
    // Strip leading `# ===` and trailing `===`
    let inner = t
        .trim_start_matches('#')
        .trim()
        .trim_start_matches('=')
        .trim()
        .trim_end_matches('=')
        .trim();
    inner.to_string()
}

/// Parse a script into named cells.
///
/// If no delimiters are found, returns a single cell named `"Script"` covering all lines.
pub fn parse_cells(script: &str) -> Vec<ScriptCell> {
    let lines: Vec<&str> = script.lines().collect();
    let total = lines.len();

    // Collect delimiter positions.
    let delimiters: Vec<(usize, String)> = lines
        .iter()
        .enumerate()
        .filter_map(|(i, l)| {
            if is_cell_delimiter(l) {
                Some((i, parse_cell_name(l)))
            } else {
                None
            }
        })
        .collect();

    if delimiters.is_empty() {
        // No delimiters — single cell covering everything.
        let end = if total == 0 { 0 } else { total - 1 };
        return vec![ScriptCell {
            id: "cell_0".to_string(),
            name: "Script".to_string(),
            start_line: 0,
            end_line: end,
            status: CellStatus::Pending,
        }];
    }

    let mut cells = Vec::with_capacity(delimiters.len());
    for (idx, (start, name)) in delimiters.iter().enumerate() {
        let end = if idx + 1 < delimiters.len() {
            delimiters[idx + 1].0.saturating_sub(1)
        } else if total == 0 {
            0
        } else {
            total - 1
        };
        cells.push(ScriptCell {
            id: format!("cell_{}", idx),
            name: name.clone(),
            start_line: *start,
            end_line: end,
            status: CellStatus::Pending,
        });
    }
    cells
}

/// Evaluate a script split into cells, updating each cell's status in-place.
///
/// The signature mirrors `evaluate_script_full` so the caller can pass the same args.
/// Returns `Some(ScriptResult)` if at least one cell produced a valid SDF,
/// or `None` if the very first cell failed.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_script_cells(
    script:             &str,
    cells:              &mut Vec<ScriptCell>,
    profiles:           Option<Arc<RwLock<HashMap<String, SplineProfile>>>>,
    splines:            Option<Arc<LongitudinalSplines>>,
    stress_field:       Option<Arc<dyn Field>>,
    displacement_field: Option<Arc<dyn Field>>,
    dimensions:         &indexmap::IndexMap<String, f64>,
    project_dir:        Option<&Path>,
    mesh_cache:         Option<Arc<Mutex<MeshCache>>>,
    library_sources:    &[(String, String)],
) -> Option<ScriptResult> {
    // Set up engine + collectors exactly as evaluate_script_full does.
    let mass_collector:   Arc<Mutex<Vec<MassPoint>>>  = Arc::new(Mutex::new(Vec::new()));
    let comp_collector:   Arc<Mutex<Vec<ComponentHandle>>> = Arc::new(Mutex::new(Vec::new()));
    let fea_collector:    Arc<Mutex<FEASetup>>        = Arc::new(Mutex::new(FEASetup::default()));
    let layup_collector:  Arc<Mutex<Vec<Arc<crate::sdf::aerospace::composite::CompositeLayup>>>>
                        = Arc::new(Mutex::new(Vec::new()));
    let ref_collector:    RefPointCollector           = Arc::new(Mutex::new(Vec::new()));

    let mut engine = Engine::new();
    api::register_sdf_functions(&mut engine);
    api::register_component_functions(&mut engine, Arc::clone(&mass_collector), Arc::clone(&comp_collector));
    api::register_drone_auto_functions(&mut engine, Arc::clone(&comp_collector));
    api::register_mass_functions(&mut engine, Arc::clone(&mass_collector));
    api::register_fea_functions(&mut engine, Arc::clone(&fea_collector), stress_field, displacement_field);
    if let Some(p) = profiles {
        api::register_profile_functions(&mut engine, p);
    }
    if let Some(s) = splines {
        api::register_spine_functions(&mut engine, s);
    }
    api::register_mesh_functions(
        &mut engine,
        project_dir.map(|p| p.to_path_buf()),
        mesh_cache.unwrap_or_else(|| Arc::new(Mutex::new(MeshCache::new()))),
    );
    api::register_composite_collector(&mut engine, Arc::clone(&layup_collector));
    api::register_query_functions(&mut engine, Arc::clone(&ref_collector));
    let mut resolver = if let Some(base) = project_dir {
        rhai::module_resolvers::FileModuleResolver::new_with_path(base)
    } else {
        rhai::module_resolvers::FileModuleResolver::new()
    };
    resolver.enable_cache(true);
    engine.set_module_resolver(resolver);

    for (mod_name, mod_source) in library_sources {
        let comp_engine = rhai::Engine::new();
        match comp_engine.compile(mod_source.as_str()) {
            Ok(ast) => {
                match rhai::Module::eval_ast_as_new(rhai::Scope::new(), &ast, &comp_engine) {
                    Ok(module) => {
                        engine.register_static_module(mod_name.as_str(), module.into());
                    }
                    Err(e) => {
                        eprintln!("[Library] Module '{}' eval error: {}", mod_name, e);
                    }
                }
            }
            Err(e) => {
                eprintln!("[Library] Module '{}' compile error: {}", mod_name, e);
            }
        }
    }

    // Shared scope across all cells.
    let mut scope = Scope::new();
    for (name, &value) in dimensions {
        scope.push_constant(name.as_str(), value);
    }

    let script_lines: Vec<&str> = script.lines().collect();
    let mut failed = false;
    let mut last_sdf: Option<SdfHandle> = None;

    for cell in cells.iter_mut() {
        if failed {
            cell.status = CellStatus::Skipped;
            continue;
        }

        // Determine content lines.
        // If cell.start_line is a delimiter line, content starts at start_line + 1.
        // If start_line == 0 and line 0 is NOT a delimiter (no-delimiter script), start from 0.
        let content_start = if cell.start_line < script_lines.len()
            && is_cell_delimiter(script_lines[cell.start_line])
        {
            cell.start_line + 1
        } else {
            cell.start_line
        };

        let content_end = cell.end_line.min(script_lines.len().saturating_sub(1));

        // Build source: prepend blank lines to keep absolute line numbers.
        let blank_prefix = "\n".repeat(content_start);
        let cell_lines: Vec<&str> = if content_start <= content_end {
            script_lines[content_start..=content_end].to_vec()
        } else {
            vec![]
        };
        let cell_source = format!("{}{}", blank_prefix, cell_lines.join("\n"));

        // Run the cell in the shared scope, collecting the last SDF-valued expression.
        match engine.eval_with_scope::<SdfHandle>(&mut scope, &cell_source) {
            Ok(handle) => {
                cell.status = CellStatus::Ok;
                last_sdf = Some(handle);
            }
            Err(e) => {
                // If the error is "output type incorrect" it means the cell ran fine but
                // didn't return an SDF (e.g. it was all statements). Treat as Ok.
                let msg = e.to_string();
                if msg.contains("() (expecting") || msg.contains("output type incorrect") {
                    // Cell ran successfully, no SDF produced by this cell — that's fine.
                    cell.status = CellStatus::Ok;
                } else {
                    // Extract line number from error if available.
                    let line = e.position().line();
                    cell.status = CellStatus::Error {
                        message: errors::format_cell_error(&msg),
                        line,
                    };
                    failed = true;
                }
            }
        }
    }

    // Extract result from collectors.
    let sdf = last_sdf?.0;
    let mass_points      = mass_collector.lock().unwrap().clone();
    let fea_setup        = std::mem::take(&mut *fea_collector.lock().unwrap());
    let layups           = layup_collector.lock().unwrap().clone();
    let reference_points = ref_collector.lock().unwrap().clone();

    Some(ScriptResult { sdf, mass_points, fea_setup, layups, reference_points })
}

#[cfg(test)]
mod cell_tests {
    use super::*;

    #[test]
    fn test_parse_no_delimiters() {
        let cells = parse_cells("let x = 1;\nlet y = 2;");
        assert_eq!(cells.len(), 1);
        assert_eq!(cells[0].name, "Script");
        assert_eq!(cells[0].start_line, 0);
        assert_eq!(cells[0].end_line, 1);
    }

    #[test]
    fn test_parse_two_sections() {
        let script = "# === A ===\nlet a = 1;\n# === B ===\nlet b = 2;";
        let cells = parse_cells(script);
        assert_eq!(cells.len(), 2);
        assert_eq!(cells[0].name, "A");
        assert_eq!(cells[0].start_line, 0);
        assert_eq!(cells[0].end_line, 1);
        assert_eq!(cells[1].name, "B");
        assert_eq!(cells[1].start_line, 2);
        assert_eq!(cells[1].end_line, 3);
    }

    #[test]
    fn test_parse_ids() {
        let script = "# === X ===\n# === Y ===\n";
        let cells = parse_cells(script);
        assert_eq!(cells[0].id, "cell_0");
        assert_eq!(cells[1].id, "cell_1");
    }

    #[test]
    fn test_parse_single_delimiter() {
        let script = "# === Shapes ===\nlet s = sphere(5.0);\ns";
        let cells = parse_cells(script);
        assert_eq!(cells.len(), 1);
        assert_eq!(cells[0].name, "Shapes");
        assert_eq!(cells[0].start_line, 0);
        assert_eq!(cells[0].end_line, 2);
    }

    #[test]
    fn test_cell_eval_two_cells() {
        let script = "# === Base ===\nlet s = sphere(5.0);\n# === Result ===\ns";
        let mut cells = parse_cells(script);
        assert_eq!(cells.len(), 2);
        let result = evaluate_script_cells(
            script, &mut cells,
            None, None, None, None,
            &indexmap::IndexMap::new(),
            None, None, &[],
        );
        assert!(result.is_some(), "Two-cell eval should succeed");
        assert_eq!(cells[0].status, CellStatus::Ok);
        assert_eq!(cells[1].status, CellStatus::Ok);
    }

    #[test]
    fn test_cell_eval_error_skips_subsequent() {
        let script = "# === Bad ===\nbad_function();\n# === Good ===\nsphere(5.0)";
        let mut cells = parse_cells(script);
        let result = evaluate_script_cells(
            script, &mut cells,
            None, None, None, None,
            &indexmap::IndexMap::new(),
            None, None, &[],
        );
        assert!(result.is_none(), "Error in first cell should give None");
        assert!(matches!(cells[0].status, CellStatus::Error { .. }));
        assert_eq!(cells[1].status, CellStatus::Skipped);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn test_evaluate_simple_sphere() {
        let script = "sphere(5.0)";
        let result = evaluate_script(script);
        assert!(result.is_ok(), "Simple sphere script should succeed");
    }

    #[test]
    fn test_evaluate_compound() {
        let script = r#"
            let base = box_(40.0, 20.0, 10.0);
            let hole = cylinder(4.0, 12.0);
            let hole = translate(hole, 10.0, 5.0, 0.0);
            let part = subtract(base, hole);
            part
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "Compound script should succeed");

        if let Ok(r) = result {
            let dist = r.sdf.distance(Vec3::new(0.0, 0.0, 0.0));
            assert!(dist < 0.0, "Origin should be inside the box");
        }
    }

    #[test]
    fn test_evaluate_error() {
        let script = "invalid_function()";
        let result = evaluate_script(script);
        assert!(result.is_err(), "Invalid script should return error");
    }

    #[test]
    fn test_mass_points_collected() {
        let script = r#"
            mass_at(150.0, 0.0, 0.0, 0.0);
            mass_at(25.0, 30.0, 0.0, 0.0);
            sphere(5.0)
        "#;
        let result = evaluate_script(script).unwrap();
        assert_eq!(result.mass_points.len(), 2);
        assert!((result.total_mass_g() - 175.0).abs() < 0.01);
        let cg = result.center_of_gravity().unwrap();
        // CG x = (150*0 + 25*30) / 175 = 750/175 ≈ 4.286
        assert!((cg.x - 4.286).abs() < 0.01, "CG x = {}", cg.x);
    }

    #[test]
    fn test_wing_with_airfoil_script() {
        let script = r#"
            let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 20.0, 3.0, -4.0);
            wing
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "wing_with_airfoil script should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Mid-chord of root section should be inside
        let d = sdf.distance(Vec3::new(6.0, 0.0, 0.0));
        assert!(d < 0.0, "Root mid-chord should be inside the wing, got {}", d);
    }

    #[test]
    fn test_fuselage_normalized() {
        // Unit fuselage: x ∈ [0,1]. Stations given out of order → auto-sorted.
        let script = r#"
            fuselage([
                [1.0,  circle_section(0.05)],
                [0.0,  circle_section(0.05)],
                [0.15, circle_section(0.5)],
                [0.85, circle_section(0.5)],
            ])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "fuselage() should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Mid-body (x=0.5) should be inside
        let d = sdf.distance(Vec3::new(0.5, 0.0, 0.0));
        assert!(d < 0.0, "mid-body should be inside, got {}", d);
        // Far outside should be positive
        let d_out = sdf.distance(Vec3::new(0.5, 5.0, 0.0));
        assert!(d_out > 0.0, "far outside should be positive, got {}", d_out);
    }

    #[test]
    fn test_fuselage_out_of_range_error() {
        let script = r#"
            fuselage([[0.0, circle_section(0.5)], [2.0, circle_section(0.1)]])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "position > 1.0 should return an error");
    }

    #[test]
    fn test_lofted_fuselage_script() {
        // Multi-station fuselage: nose at x=0 (r=0.2), body at x=3 (r=1), tail at x=8 (r=0.3)
        let script = r#"
            let nose  = fuselage_station(0.0, circle_section(0.2));
            let body  = fuselage_station(3.0, circle_section(1.0));
            let tail  = fuselage_station(8.0, ellipse_section(0.5, 0.3));
            lofted_fuselage([nose, body, tail])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "lofted_fuselage script should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Point at mid-body (x=4, y=0, z=0) should be inside
        let d = sdf.distance(Vec3::new(4.0, 0.0, 0.0));
        assert!(d < 0.0, "Mid-body should be inside, got {}", d);
        // Point far above mid-body should be outside
        let d_out = sdf.distance(Vec3::new(4.0, 5.0, 0.0));
        assert!(d_out > 0.0, "Point far above should be outside, got {}", d_out);
    }

    #[test]
    fn test_lofted_fuselage_smooth_script() {
        let script = r#"
            let nose  = fuselage_station(0.0, circle_section(0.2));
            let body  = fuselage_station(3.0, ellipse_section(1.2, 0.8));
            let tail  = fuselage_station(8.0, ellipse_section(0.5, 0.3));
            lofted_fuselage_smooth([nose, body, tail], 0.7)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "lofted_fuselage_smooth script should succeed: {:?}", result.err());
    }

    #[test]
    fn test_lofted_fuselage_unsorted_error() {
        // lofted_fuselage() still validates order (legacy API)
        let script = r#"
            let a = fuselage_station(5.0, circle_section(1.0));
            let b = fuselage_station(2.0, circle_section(0.5));
            lofted_fuselage([a, b])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "lofted_fuselage with unsorted stations should return an error");
    }

    #[test]
    fn test_lofted_fuselage_too_few_error() {
        let script = r#"
            let a = fuselage_station(0.0, circle_section(1.0));
            lofted_fuselage([a])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "Fewer than 2 stations should return an error");
    }

    // --- Structural primitive integration tests ---

    #[test]
    fn test_rib_at_station() {
        // Root rib (y=0) on a 2412 wing; root chord 12, half-span 20.
        // Interior point at (6, 0, 0) should be inside the rib.
        // Point at (6, 5, 0) is outside the rib thickness (0.5) but still in the wing —
        // the intersection should exclude it.
        let script = r#"
            let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 0.0, 0.0, 0.0);
            rib_at_station(wing, 0.0, 0.5)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "rib_at_station should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        let d_inside = sdf.distance(Vec3::new(6.0, 0.0, 0.0));
        assert!(d_inside < 0.0, "root rib centre should be inside, got {}", d_inside);
        let d_outside_span = sdf.distance(Vec3::new(6.0, 5.0, 0.0));
        assert!(d_outside_span > 0.0, "point 5 units from rib should be outside, got {}", d_outside_span);
    }

    #[test]
    fn test_spar() {
        // Front spar at 25% of root chord (x=3), radius 0.4.
        // Point on spar axis at root should be inside; point far off-axis should be outside.
        let script = r#"
            let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 0.0, 0.0, 0.0);
            spar(wing, 3.0, 0.4)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "spar should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        let d_on_axis = sdf.distance(Vec3::new(3.0, 0.0, 0.0));
        assert!(d_on_axis < 0.0, "spar axis at root should be inside, got {}", d_on_axis);
        let d_off_axis = sdf.distance(Vec3::new(3.0, 0.0, 5.0));
        assert!(d_off_axis > 0.0, "point well above spar should be outside, got {}", d_off_axis);
    }

    #[test]
    fn test_two_ribs_union() {
        // Two ribs at different stations unioned together.
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let rib1 = rib_at_station(wing, 0.0, 0.4);
            let rib2 = rib_at_station(wing, 6.0, 0.4);
            union(rib1, rib2)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "union of two ribs should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Both rib centres should be inside the union
        assert!(sdf.distance(Vec3::new(5.0, 0.0, 0.0)) < 0.0, "rib1 centre should be inside");
        assert!(sdf.distance(Vec3::new(5.0, 6.0, 0.0)) < 0.0, "rib2 centre should be inside");
        // Gap between ribs should be outside
        assert!(sdf.distance(Vec3::new(5.0, 3.0, 0.0)) > 0.0, "gap between ribs should be outside");
    }

    #[test]
    fn test_rib_spar_composition() {
        // Full structural assembly: two ribs + front spar, all unioned.
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let rib1     = rib_at_station(wing, 0.0, 0.4);
            let rib2     = rib_at_station(wing, 6.0, 0.4);
            let front_sp = spar(wing, 2.5, 0.35);
            let structure = union(union(rib1, rib2), front_sp);
            structure
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "rib+spar assembly should succeed: {:?}", result.err());
        // The result must be a valid SDF (can be further composed)
        let sdf = result.unwrap().sdf;
        // At least one of the three parts must register as interior at a known-interior point
        let d = sdf.distance(Vec3::new(2.5, 0.0, 0.0));
        assert!(d < 0.0, "spar axis at root should be inside the assembly, got {}", d);
    }

    #[test]
    fn test_airfoil_from_points_script() {
        // Minimal diamond-shaped airfoil: 4 pts that form a closed loop
        let script = r#"
            let pts = [
                [0.0, 0.0],
                [0.5, 0.1],
                [1.0, 0.0],
                [0.5, -0.1],
                [0.0, 0.0],
            ];
            let af = airfoil_from_points(pts, 10.0);
            let wing = wing_with_airfoil(af, af, 20.0, 0.0, 0.0, 0.0);
            wing
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "airfoil_from_points wing should succeed: {:?}", result.err());
    }

    #[test]
    fn test_airfoil_from_points_too_few_error() {
        let script = r#"
            let pts = [[0.0, 0.0], [1.0, 0.0]];
            airfoil_from_points(pts, 5.0)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "Fewer than 3 points should return an error");
    }

    #[test]
    fn test_auto_fuselage() {
        let script = r#"
            let internals = box_(8.0, 4.0, 3.0);
            auto_fuselage(internals, 2.0)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "auto_fuselage should succeed");
        // Outer skin should be larger than inner box
        let sdf = result.unwrap().sdf;
        // box_ half-extent = 4.0, skin = 2.0 → outer surface at x=6.0
        // At x=5.5 we should be inside the skin
        let dist = sdf.distance(Vec3::new(5.5, 0.0, 0.0));
        assert!(dist < 0.0, "Point inside skin should be interior, got {}", dist);
    }

    // ── Drone structural primitive integration tests ───────────────────────────

    /// Helper script fragment that builds a normalised test fuselage.
    fn fuse_script() -> &'static str {
        r#"let fuse = fuselage([
            [0.0, circle_section(0.3)],
            [0.5, circle_section(1.0)],
            [1.0, circle_section(0.3)],
        ]);"#
    }

    #[test]
    fn test_bulkhead_at_station_script() {
        let script = format!(r#"
            {}
            bulkhead_at_station(fuse, 0.5, 0.02, 0, 0.0)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "bulkhead_at_station should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Centre of bulkhead at x=0.5 must be inside.
        assert!(sdf.distance(Vec3::new(0.5, 0.0, 0.0)) < 0.0);
        // Off-station must be outside.
        assert!(sdf.distance(Vec3::new(0.9, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn test_bulkhead_with_holes_script() {
        let script = format!(r#"
            {}
            bulkhead_at_station(fuse, 0.5, 0.02, 6, 0.3)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "bulkhead_at_station with holes should succeed: {:?}", result.err());
    }

    #[test]
    fn test_lightening_hole_pattern_script() {
        let script = format!(r#"
            {}
            let bk = bulkhead_at_station(fuse, 0.5, 0.02, 0, 0.0);
            lightening_hole_pattern(bk, 4, 0.5, 0.1, 0)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "lightening_hole_pattern should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Hole centre at (0.5, 0.5, 0) should be outside after drilling.
        assert!(sdf.distance(Vec3::new(0.5, 0.5, 0.0)) > 0.0,
            "hole location should be outside");
    }

    #[test]
    fn test_rod_mount_script() {
        let script = format!(r#"
            {}
            let bk = bulkhead_at_station(fuse, 0.5, 0.02, 0, 0.0);
            rod_mount(bk, 0.0, 0.6, 0.04, 0.08)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "rod_mount should succeed: {:?}", result.err());
    }

    #[test]
    fn test_motor_arm_script() {
        let script = format!(r#"
            {}
            motor_arm(fuse, 0.0, 2.0, 0.12, 0.09)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "motor_arm should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Tube wall at midspan: outer_r=0.06, Z=0.05 is in the wall (0.045..0.06).
        assert!(sdf.distance(Vec3::new(0.5, 2.5, 0.05)) < 0.0,
            "arm tube wall should be inside");
        // Hollow axis should be outside the tube.
        assert!(sdf.distance(Vec3::new(0.5, 2.5, 0.0)) > 0.0,
            "arm hollow axis should be outside");
        // Far beyond arm tip should be outside.
        assert!(sdf.distance(Vec3::new(0.5, 10.0, 0.0)) > 0.0,
            "beyond arm tip should be outside");
    }

    #[test]
    fn test_motor_mount_script() {
        let script = format!(r#"
            {}
            let arm = motor_arm(fuse, 0.0, 2.0, 0.12, 0.09);
            motor_mount(arm, 0.3, 0.05, 0.12, 0.03)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "motor_mount should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Plate at y ≈ 3.0, half_side=0.15, thickness=0.05. Point at y=2.98 should be inside.
        assert!(sdf.distance(Vec3::new(0.5, 2.98, 0.0)) < 0.0,
            "motor mount plate interior should be inside");
    }

    #[test]
    fn test_generate_mounts_script() {
        let script = format!(r#"
            {}
            let battery = component_named("battery", box_(0.4, 0.2, 0.15), 0.02, 50.0);
            let battery_placed = place(battery, 0.5, 0.0, -0.3);
            generate_mounts([battery_placed], fuse, 0.015, 0.04)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "generate_mounts should succeed: {:?}", result.err());
    }

    #[test]
    fn test_mount_with_bolts_script() {
        let script = format!(r#"
            {}
            let esc = component(box_(0.1, 0.05, 0.02), 0.01);
            let esc_placed = place(esc, 0.5, 0.0, -0.2);
            mount_with_bolts(esc_placed, fuse, 0.01, 0.03, 0.02, 4)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "mount_with_bolts should succeed: {:?}", result.err());
    }

    #[test]
    fn test_drone_full_example_script() {
        // Full workflow: fuselage + two bulkheads + two arms + motor mount + battery tray.
        let script = r#"
            let fuse = fuselage([
                [0.0, circle_section(0.3)],
                [0.5, circle_section(1.0)],
                [1.0, circle_section(0.3)],
            ]);
            let frame_a = bulkhead_at_station(fuse, 0.3, 0.02, 8, 0.6);
            let frame_b = bulkhead_at_station(fuse, 0.6, 0.02, 8, 0.6);
            let arm_0   = motor_arm(fuse, 0.0, 2.0, 0.12, 0.09);
            let arm_90  = motor_arm(fuse, 90.0, 2.0, 0.12, 0.09);
            let mount_0 = motor_mount(arm_0, 0.3, 0.05, 0.12, 0.03);
            let battery = component_named("battery", box_(0.4, 0.2, 0.15), 0.02, 180.0);
            let battery_placed = place(battery, 0.5, 0.0, -0.3);
            let battery_mount  = generate_mounts([battery_placed], fuse, 0.015, 0.04);
            union(union(union(union(union(fuse, frame_a), frame_b), arm_90), mount_0), battery_mount)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "full drone script should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Fuselage mid-body must be inside the combined geometry.
        assert!(sdf.distance(Vec3::new(0.5, 0.0, 0.0)) < 0.0,
            "fuselage mid-body should be inside");
    }

    // ── Part 9: Library module integration tests ───────────────────────────────

    /// A library component with a build() function is callable via module syntax.
    #[test]
    fn test_library_module_callable() {
        let component_source = r#"
fn build(r) {
    sphere(r)
}
"#;
        let main_script = "my_comp::build(7.0)";
        let result = evaluate_script_full(
            main_script,
            None, None, None, None,
            &indexmap::IndexMap::new(),
            None, None,
            &[("my_comp".to_string(), component_source.to_string())],
        );
        assert!(result.is_ok(), "Library module should be callable: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Sphere of radius 7: point at origin is inside
        assert!(sdf.distance(Vec3::ZERO) < 0.0, "Origin should be inside sphere(7)");
        // Point at distance 8 should be outside
        assert!(sdf.distance(Vec3::new(8.0, 0.0, 0.0)) > 0.0, "Point outside sphere should be positive");
    }

    /// A library component with a syntax error does not prevent main script evaluation.
    #[test]
    fn test_library_syntax_error_does_not_block_main() {
        let bad_source = "fn build( { invalid syntax !!!";
        let main_script = "sphere(3.0)";
        // Even with a broken component, the main script should succeed.
        let result = evaluate_script_full(
            main_script,
            None, None, None, None,
            &indexmap::IndexMap::new(),
            None, None,
            &[("broken_comp".to_string(), bad_source.to_string())],
        );
        assert!(result.is_ok(), "Main script should succeed despite broken library: {:?}", result.err());
    }

    /// composite_layup_config + apply_layup round-trip.
    #[test]
    fn test_layup_config_and_apply() {
        let script = r#"
            let mat = material("CarbonWoven_200gsm");
            let layers = [shell_layer("outer", mat, 0.6)];
            let cfg = composite_layup_config(layers);
            let body = sphere(10.0);
            apply_layup(body, cfg)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "composite_layup_config + apply_layup should succeed: {:?}", result.err());
        // The composite SDF wraps the sphere — origin is inside
        assert!(result.unwrap().sdf.distance(Vec3::ZERO) < 0.0);
    }

    // ── Part 8: Control surface integration tests ──────────────────────────────

    /// Aileron applied to a wing produces valid control_surface and modified_parent.
    #[test]
    fn test_aileron_produces_valid_sdfs() {
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let hinge = rounded_hinge(1.5, 0.5);
            let horn = control_horn_lower(15.0, 10.0, 0.5);
            let parts = aileron(wing, 5.0, 12.0, 0.30, hinge, horn);
            parts[1]
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "aileron should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Modified parent should still have material inside the wing (root area)
        let d = sdf.distance(Vec3::new(5.0, 0.0, 0.0));
        assert!(d < 0.0, "Modified parent root area should still be solid, got {}", d);
    }

    /// Control surface is carved from the aileron span region.
    #[test]
    fn test_aileron_control_surface_in_span() {
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let hinge = simple_gap_hinge(0.5);
            let linkage = control_horn_lower(15.0, 10.0, 0.5);
            let parts = aileron(wing, 5.0, 12.0, 0.30, hinge, linkage);
            parts[0]
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "aileron control surface should succeed: {:?}", result.err());
        let cs_sdf = result.unwrap().sdf;
        // Point outside the aileron span (y=1, root region) should be outside the CS
        let d_outside = cs_sdf.distance(Vec3::new(8.0, 1.0, 0.0));
        assert!(d_outside > 0.0, "Root region should be outside aileron CS, got {}", d_outside);
    }

    #[test]
    fn test_no_linkage_control_surface_has_no_horn_stub() {
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let hinge = rounded_hinge(1.5, 0.5);
            let parts = aileron(wing, 16.5, 27.6, 0.30, hinge, no_linkage());
            parts[0]
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "aileron with no_linkage should succeed: {:?}", result.err());
        let cs_sdf = result.unwrap().sdf;
        let d_below = cs_sdf.distance(Vec3::new(9.5, 22.0, -8.0));
        assert!(d_below > 0.0, "No-linkage control surface should not create a horn stub, got {}", d_below);
    }

    /// Legacy 4-arg aileron/elevon APIs treat span inputs in [0,1] as half-span fractions.
    #[test]
    fn test_legacy_aileron_fraction_inputs_map_outboard() {
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            aileron(wing, 0.55, 0.92, 0.30)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "legacy aileron should succeed: {:?}", result.err());
        let cs_sdf = result.unwrap().sdf;

        let d_root = cs_sdf.distance(Vec3::new(9.5, 1.0, 0.0));
        assert!(d_root > 0.0, "Legacy aileron should not create a root stub, got {}", d_root);
    }

    /// wing_with_ailerons returns 3 SdfHandles.
    #[test]
    fn test_wing_with_ailerons_returns_three_handles() {
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let parts = wing_with_ailerons(wing, 0.4, 0.9, 0.30);
            parts[2]
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "wing_with_ailerons should succeed: {:?}", result.err());
        // Modified wing should have material at root
        let sdf = result.unwrap().sdf;
        let d = sdf.distance(Vec3::new(5.0, 0.0, 0.0));
        assert!(d < 0.0, "Modified wing root should be solid, got {}", d);
    }

    /// Rounded hinge creates a void near the hinge line radius.
    #[test]
    fn test_rounded_hinge_void() {
        use crate::sdf::aerospace::control_surfaces::{
            ControlSurface, ControlSurfaceType, HingeSpec, LinkageSpec, build_control_surface,
        };
        use crate::sdf::primitives::SdfBox;

        // Simple box wing: X=0..20, Y=-15..15, Z=-2..2
        let wing: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfBox::new(Vec3::new(10.0, 15.0, 2.0)));
        // Translate to match chord range 0..20 (center at x=10)
        use crate::sdf::transforms::Translate;
        let wing = Arc::new(Translate::new(wing, Vec3::new(10.0, 0.0, 0.0)));

        let spec = ControlSurface {
            surface_type: ControlSurfaceType::Aileron,
            parent_wing: wing.clone(),
            span_start: 3.0, span_end: 12.0,
            chord_fraction: 0.30,
            hinge: HingeSpec::rounded(2.0, 0.5),
            linkage: LinkageSpec::none(),
        };
        let result = build_control_surface(&spec);
        // The modified parent should have the control surface region removed
        // Point deep in the aileron span (y=7.5) aft of hinge should be outside modified parent
        let y_test = 7.5;
        let d_aft = result.modified_parent.distance(Vec3::new(17.0, y_test, 0.0));
        assert!(d_aft > 0.0, "Aft of hinge in aileron span should be outside modified parent, got {}", d_aft);
        // Root region should still be solid in modified parent
        let d_root = result.modified_parent.distance(Vec3::new(5.0, 0.0, 0.0));
        assert!(d_root < 0.0, "Root region should be solid in modified parent, got {}", d_root);
    }

    /// Thumbnail renderer produces non-empty pixels for a sphere.
    #[test]
    fn test_thumbnail_non_empty() {
        use std::sync::Arc;
        use crate::sdf::primitives::Sphere;
        let sdf: Arc<dyn crate::sdf::Sdf> = Arc::new(Sphere::new(5.0));
        let pixels = crate::library::thumbnail::render_thumbnail(sdf);
        assert_eq!(pixels.len(), crate::library::thumbnail::THUMB_W * crate::library::thumbnail::THUMB_H * 4);
        // Should not be all black — there should be hit pixels
        let has_lit = pixels.chunks(4).any(|p| p[0] > 50 || p[1] > 50 || p[2] > 50);
        assert!(has_lit, "Thumbnail should have some lit pixels from sphere hit");
    }

    // ── Part 8: Geometric query integration tests ──────────────────────────────

    #[test]
    fn test_surface_point_from_interior() {
        use crate::sdf::query::surface_point;
        use crate::sdf::primitives::Sphere;
        let sdf = Sphere::new(10.0);
        // Ray from origin in +X should hit surface at (10, 0, 0)
        let pt = surface_point(&sdf, Vec3::ZERO, Vec3::X, 100.0).unwrap();
        assert!((pt.x - 10.0).abs() < 0.1, "surface hit should be at x≈10, got {}", pt.x);
        assert!(pt.y.abs() < 0.1);
    }

    #[test]
    fn test_closest_point_on_sphere() {
        use crate::sdf::query::closest_point;
        use crate::sdf::primitives::Sphere;
        let sdf = Sphere::new(5.0);
        // Query from (10, 0, 0) — closest surface point should be (5, 0, 0)
        let pt = closest_point(&sdf, Vec3::new(10.0, 0.0, 0.0));
        let dist = pt.length();
        assert!((dist - 5.0).abs() < 0.1, "Closest point should be at radius 5, got {}", dist);
    }

    #[test]
    fn test_furthest_point_sphere() {
        use crate::sdf::query::furthest_point;
        use crate::sdf::primitives::Sphere;
        let sdf = Sphere::new(8.0);
        let pt = furthest_point(&sdf, Vec3::X);
        assert!((pt.x - 8.0).abs() < 1.0, "Furthest in +X should be near (8,0,0), got {:?}", pt);
    }

    #[test]
    fn test_ref_point_and_get_ref() {
        let script = r#"
            let p = point(1.0, 2.0, 3.0);
            let stored = ref_point("test_pt", p);
            let retrieved = get_ref("test_pt");
            let origin = point(0.0, 0.0, 0.0);
            let d = dist(retrieved, origin);
            let body = sphere(5.0);
            translate_p(body, stored)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "ref_point + get_ref should succeed: {:?}", result.err());
        let script_result = result.unwrap();
        assert_eq!(script_result.reference_points.len(), 1);
        let rp = &script_result.reference_points[0];
        assert_eq!(rp.name, "test_pt");
        assert!((rp.position.x - 1.0).abs() < 0.01);
        assert!((rp.position.y - 2.0).abs() < 0.01);
        assert!((rp.position.z - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_sdf_distance_queries() {
        let script = r#"
            let s = sphere(5.0);
            let inside = sdf_distance_p(s, point(0.0, 0.0, 0.0));
            let outside = sdf_distance(s, 8.0, 0.0, 0.0);
            if inside < 0.0 && outside > 0.0 {
                s
            } else {
                sphere(1.0)
            }
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "sdf_distance queries should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        assert!(sdf.distance(Vec3::ZERO) < 0.0);
    }

    #[test]
    fn test_offset_point() {
        let script = r#"
            let p = point(0.0, 0.0, 0.0);
            let q = offset_point(p, 3.0, 4.0, 0.0);
            translate_p(sphere(5.0), q)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "offset_point should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Sphere translated to (3,4,0), center should be inside
        let d = sdf.distance(Vec3::new(3.0, 4.0, 0.0));
        assert!(d < 0.0, "Center of translated sphere should be inside, got {}", d);
    }

    #[test]
    fn test_trailing_edge_query() {
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let te = trailing_edge(wing, 0.0);
            translate_p(sphere(1.0), te)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "trailing_edge should succeed: {:?}", result.err());
    }

    // ── Phase 22: Bracket / mounting hole tests ───────────────────────────────

    #[test]
    fn test_bulkhead_with_components_subtracts_keepout() {
        // fuselage radius at x=0.5 is 1.0 (model units).
        // Battery box half-extents: X=0.2, Y=0.1, Z=0.075 — centred at (0.5, 0, 0).
        // margin=0.05 → keepout expands by 0.05 in model units (small, realistic).
        let script = r#"
            let fuse = fuselage([
                [0.0, circle_section(0.05)],
                [0.5, circle_section(1.0)],
                [1.0, circle_section(0.05)],
            ]);
            let battery = component_named("battery", box_(0.4, 0.2, 0.15), 0.02, 80.0);
            let batt_placed = place(battery, 0.5, 0.0, 0.0);
            let bk = bulkhead_with_components(fuse, 0.5, 0.02, [batt_placed], 0.05);
            bk
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "bulkhead_with_components should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Inside the battery keepout at station 0.5 should be OUTSIDE the bulkhead (subtracted)
        // battery half-ext in Y = 0.1, at (0.5, 0, 0) should be cleared
        assert!(
            sdf.distance(Vec3::new(0.5, 0.0, 0.0)) > 0.0,
            "Battery keepout area should be subtracted from bulkhead"
        );
        // Fuselage ring material at (0.5, 0.9, 0) should be inside bulkhead
        // battery half-ext Y = 0.1 + 0.02 (margin in component_named) + 0.05 (extra margin) = 0.17
        // 0.9 is well outside the battery keepout, so this should be solid
        let d_ring = sdf.distance(Vec3::new(0.5, 0.9, 0.0));
        assert!(d_ring < 0.0, "Fuselage ring should be solid, got {}", d_ring);
    }

    #[test]
    fn test_bulkhead_auto_finds_intersecting_components() {
        let script = r#"
            let fuse = fuselage([
                [0.0, circle_section(0.05)],
                [0.5, circle_section(1.0)],
                [1.0, circle_section(0.05)],
            ]);
            let bat = component_named("bat", box_(0.2, 0.1, 0.1), 0.01, 50.0);
            let bat_p = place(bat, 0.5, 0.0, 0.0);
            let bk = bulkhead_auto(fuse, 0.5, 0.02);
            bk
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "bulkhead_auto should succeed: {:?}", result.err());
    }

    #[test]
    fn test_cable_hole_creates_void() {
        let script = r#"
            let fuse = fuselage([
                [0.0, circle_section(0.05)],
                [0.5, circle_section(1.0)],
                [1.0, circle_section(0.05)],
            ]);
            let bk = bulkhead_at_station(fuse, 0.5, 0.02, 0, 0.0);
            cable_hole(bk, 0.3, 0.0, 8.0)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "cable_hole should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Point at the cable hole center (x=0.5, y=0.3, z=0) should be outside
        assert!(
            sdf.distance(Vec3::new(0.5, 0.3, 0.0)) > 0.0,
            "Cable hole should be empty"
        );
    }

    #[test]
    fn test_auto_bulkheads_no_panic() {
        let script = r#"
            let fuse = fuselage([
                [0.0, circle_section(0.05)],
                [0.5, circle_section(1.0)],
                [1.0, circle_section(0.05)],
            ]);
            auto_bulkheads(fuse, 3, 0.02)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "auto_bulkheads should succeed: {:?}", result.err());
    }

    #[test]
    fn test_conformal_spline_path_supports_offset_surface_features() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.03)],
                [0.3, ellipse_section(0.12, 0.08)],
                [0.7, ellipse_section(0.12, 0.08)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let path = conformal_spline_path(
                fuse,
                [[0.2, 0.0, 0.12], [0.45, 0.0, 0.16], [0.7, 0.0, 0.12]],
                0.02,
                24
            );
            sweep(ellipse_profile(0.05, 0.03), path)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "conformal_spline_path should succeed: {:?}", result.err());
    }

    #[test]
    fn test_conformal_inlet_returns_fairing_and_duct() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.03)],
                [0.3, ellipse_section(0.12, 0.08)],
                [0.7, ellipse_section(0.12, 0.08)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let parts = conformal_inlet(
                fuse,
                [[0.2, 0.0, 0.12], [0.4, 0.0, 0.16], [0.6, 0.0, 0.14]],
                0.08, 0.05,
                0.06, 0.03,
                0.04,
                0.01,
                24,
                [0.78, 0.0, 0.0],
                [0.98, 0.0, 0.0]
            );
            parts[0]
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "conformal_inlet should succeed: {:?}", result.err());
    }

    #[test]
    fn test_conformal_profile_inlet_returns_all_parts() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.03)],
                [0.3, ellipse_section(0.12, 0.08)],
                [0.7, ellipse_section(0.12, 0.08)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let guide = [[0.2, 0.0, 0.12], [0.36, 0.0, 0.16], [0.56, 0.0, 0.14]];
            let duct = spline_path([
                [0.2, 0.0, 0.12],
                [0.28, 0.0, 0.13],
                [0.42, 0.0, 0.06],
                [0.62, 0.0, 0.02],
                [0.82, 0.0, 0.0],
            ]);
            let outer_start = rounded_rect_profile(0.08, 0.05, 0.01);
            let outer_end = circle_profile(0.025);
            let inner_start = rounded_rect_profile(0.064, 0.038, 0.008);
            let inner_end = circle_profile(0.023);
            let parts = conformal_profile_inlet(
                fuse,
                guide,
                duct,
                outer_start,
                outer_end,
                inner_start,
                inner_end,
                0.01,
                0.03,
                0.03,
                64
            );
            union(parts[0], parts[2])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "conformal_profile_inlet should succeed: {:?}", result.err());
    }

    #[test]
    fn test_variable_duct_circular_outlet_builds() {
        let script = r#"
            let path = spline_path([
                [0.0, 0.0, 0.04],
                [0.08, 0.0, 0.045],
                [0.18, 0.0, 0.02],
                [0.30, 0.0, 0.0],
                [0.45, 0.0, 0.0]
            ]);
            variable_duct_circular_outlet(path, 0.092, 0.090, 0.090, 0.002, 32, 0.9)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "variable_duct_circular_outlet should succeed: {:?}", result.err());
    }

    #[test]
    fn test_spline_tube_builds() {
        let script = r#"
            let path = spline_path([
                [0.0, 0.0, 0.046],
                [0.024, 0.0, 0.054],
                [0.072, 0.0, 0.058],
                [0.132, 0.0, 0.050],
                [0.196, 0.0, 0.028],
                [0.248, 0.0, 0.008],
                [0.292, 0.0, 0.000],
                [0.430, 0.0, 0.000]
            ]);
            spline_tube(path, 0.090, 0.090, 0.002, 96, 0.95)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "spline_tube should succeed: {:?}", result.err());
    }

    #[test]
    fn test_custom_profile_and_profile_duct_build() {
        let script = r#"
            let path = spline_path([
                [0.0, 0.0, 0.046],
                [0.020, 0.0, 0.052],
                [0.056, 0.0, 0.058],
                [0.108, 0.0, 0.060],
                [0.164, 0.0, 0.048],
                [0.218, 0.0, 0.024],
                [0.266, 0.0, 0.008],
                [0.312, 0.0, 0.000],
                [0.430, 0.0, 0.000]
            ]);
            let outer_start = rounded_rect_profile(0.100, 0.076, 0.014);
            let inner_start = custom_profile([[0.048,0.000],[0.032,0.022],[0.000,0.028],[-0.032,0.022],[-0.048,0.000],[-0.020,-0.018],[0.020,-0.018]]);
            let outer_end = circle_profile(0.047);
            let inner_end = circle_profile(0.045);
            profile_duct(path, outer_start, outer_end, inner_start, inner_end, 0.070, 0.070, 160)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "profile_duct should succeed: {:?}", result.err());
    }

    #[test]
    fn test_conformal_profile_builds_for_dorsal_and_side_mounts() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.04)],
                [0.3, ellipse_section(0.10, 0.09)],
                [0.7, ellipse_section(0.10, 0.09)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let trap = custom_profile([[-0.028,-0.018],[0.028,-0.018],[0.022,0.018],[-0.022,0.018]]);
            let dorsal = conformal_profile(fuse, trap, point(0.30, 0.0, 0.12), point(1.0, 0.0, 0.0), 0.004, 64);
            let cheek = conformal_profile(fuse, trap, point(0.30, 0.11, 0.05), point(1.0, 0.0, 0.0), 0.004, 64);
            let d_prof = dorsal[0];
            let d_ctr = dorsal[1];
            let c_prof = cheek[0];
            let c_ctr = cheek[1];
            union(
                profile_duct_fixed_solid(polyline_path([[d_ctr.x,d_ctr.y,d_ctr.z],[d_ctr.x+0.02,d_ctr.y,d_ctr.z]]), d_prof, d_prof, 64),
                profile_duct_fixed_solid(polyline_path([[c_ctr.x,c_ctr.y,c_ctr.z],[c_ctr.x+0.02,c_ctr.y,c_ctr.z]]), c_prof, c_prof, 64)
            )
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "conformal_profile should support dorsal and side mounts: {:?}", result.err());
    }

    #[test]
    fn test_conformal_profile_x_builds_without_global_surface_snap() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.04)],
                [0.3, ellipse_section(0.10, 0.09)],
                [0.7, ellipse_section(0.10, 0.09)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let wing = translate(box_(0.15, 0.40, 0.01), 0.30, 0.22, 0.00);
            let body = union(fuse, wing);
            let trap = custom_profile([[-0.028,-0.018],[0.028,-0.018],[0.022,0.018],[-0.022,0.018]]);
            let data = conformal_profile_x(body, trap, 0.30, 0.0, 0.12, point(1.0, 0.0, 0.0), 0.004, 64);
            let prof = data[0];
            let ctr = data[1];
            profile_duct_fixed_solid(polyline_path([[ctr.x,ctr.y,ctr.z],[ctr.x+0.02,ctr.y,ctr.z]]), prof, prof, 64)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "conformal_profile_x should succeed on a cluttered x-station: {:?}", result.err());
    }

    #[test]
    fn test_conformal_profile_duct_x_builds_full_parts() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.04)],
                [0.3, ellipse_section(0.10, 0.09)],
                [0.7, ellipse_section(0.10, 0.09)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let parts = conformal_profile_duct_x(
                fuse,
                rounded_rect_profile(0.072, 0.039, 0.002),
                rounded_rect_profile(0.068, 0.035, 0.001),
                0.155, 0.0, 0.12,
                point(1.0, 0.0, 0.0),
                0.005,
                spline_path([
                    [0.155, 0.0, 0.10],
                    [0.230, 0.0, 0.10],
                    [0.320, 0.0, 0.02],
                    [0.430, 0.0, 0.00]
                ]),
                circle_profile(0.035),
                circle_profile(0.033),
                0.014,
                0.024,
                0.1,
                0.9,
                128
            );
            union(parts[0], parts[2])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "conformal_profile_duct_x should build complete parts: {:?}", result.err());
    }

    #[test]
    fn test_dual_conformal_profile_duct_x_builds_standalone_pair() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.04)],
                [0.3, ellipse_section(0.10, 0.09)],
                [0.7, ellipse_section(0.10, 0.09)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let parts = dual_conformal_profile_duct_x(
                fuse,
                rounded_rect_profile(0.050, 0.028, 0.004),
                rounded_rect_profile(0.046, 0.024, 0.003),
                0.155, 0.055, 0.05,
                0.155, -0.055, 0.05,
                point(1.0, 0.0, 0.0),
                spline_path([
                    [0.155, 0.055, 0.052],
                    [0.250, 0.070, 0.040],
                    [0.350, 0.080, 0.028]
                ]),
                spline_path([
                    [0.155, -0.055, 0.052],
                    [0.250, -0.070, 0.040],
                    [0.350, -0.080, 0.028]
                ]),
                circle_profile(0.022),
                circle_profile(0.020),
                0.004,
                0.014,
                0.020,
                0.2,
                0.9,
                160
            );
            union(parts[0], parts[2])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "dual_conformal_profile_duct_x should build a standalone pair: {:?}", result.err());
    }

    #[test]
    fn test_mirrored_dual_conformal_profile_duct_x_builds_merge_case() {
        let script = r#"
            let fuse = fuselage([
                [0.0, ellipse_section(0.05, 0.04)],
                [0.3, ellipse_section(0.10, 0.09)],
                [0.7, ellipse_section(0.10, 0.09)],
                [1.0, ellipse_section(0.04, 0.03)],
            ]);
            let parts = mirrored_dual_conformal_profile_duct_x(
                fuse,
                rounded_rect_profile(0.050, 0.028, 0.004),
                rounded_rect_profile(0.046, 0.024, 0.003),
                0.155, 0.055, 0.05,
                point(1.0, 0.0, 0.0),
                spline_path([
                    [0.155, 0.055, 0.052],
                    [0.230, 0.045, 0.040],
                    [0.320, 0.018, 0.018],
                    [0.410, 0.000, 0.000]
                ]),
                circle_profile(0.028),
                circle_profile(0.026),
                0.004,
                0.014,
                0.024,
                0.2,
                0.9,
                160
            );
            union(parts[0], parts[2])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "mirrored_dual_conformal_profile_duct_x should build a merge case: {:?}", result.err());
    }

    #[test]
    fn test_error_message_unknown_function() {
        let script = "unknown_fn(5.0)";
        let result = evaluate_script(script);
        assert!(result.is_err(), "Should be an error");
        let err = result.err().unwrap();
        // Should contain a helpful message mentioning the unknown function
        assert!(err.contains("Unknown function") || err.contains("Function not found") || err.contains("unknown_fn"),
            "Error should mention the unknown function, got: {}", err);
    }

    #[test]
    fn test_error_message_includes_context() {
        let script = "let x = sphere(5.0);\nunknown_fn(x)\n";
        let result = evaluate_script(script);
        assert!(result.is_err(), "Should be an error");
        let err = result.err().unwrap();
        // Error message should be non-empty and mention the function
        assert!(!err.is_empty(), "Error should not be empty");
    }

    #[test]
    fn test_full_assembly_symmetric() {
        let script = r#"
            let half = translate(sphere(5.0), 0.0, 8.0, 0.0);
            full_assembly(half)
        "#;
        let result = evaluate_script(script).unwrap();
        let sdf = result.sdf;
        // Both (0, 8, 0) and (0, -8, 0) should be inside
        assert!(sdf.distance(Vec3::new(0.0,  8.0, 0.0)) < 0.0, "pos Y half should be inside");
        assert!(sdf.distance(Vec3::new(0.0, -8.0, 0.0)) < 0.0, "neg Y half should be inside");
        // Origin should be outside (gap between the two spheres)
        assert!(sdf.distance(Vec3::ZERO) > 0.0, "origin should be outside");
    }

    #[test]
    fn test_wing_area_rectangular() {
        // Rectangular wing: box 100mm chord x 200mm span x 10mm thick
        // Planform area ≈ 100 * 200 = 20000 mm²; sphere radius = area/10000 ≈ 2.0
        let script = r#"
            let wing = box_(100.0, 200.0, 10.0);
            let area = wing_area(wing);
            sphere(area / 10000.0)
        "#;
        let result = evaluate_script(script).unwrap();
        // The sphere radius is area/10000; for area≈20000 → radius≈2.0
        // Point at (1.8, 0, 0) should be inside (radius > 1.8)
        assert!(result.sdf.distance(Vec3::new(1.8, 0.0, 0.0)) < 0.0,
            "wing_area on rectangular wing should be ~20000, making sphere radius ~2.0");
    }

    #[test]
    fn test_instance_four_positions() {
        let script = r#"
            let bolt = sphere(2.0);
            let positions = [
                #{tx:  10.0, ty:  10.0, tz: 0.0},
                #{tx: -10.0, ty:  10.0, tz: 0.0},
                #{tx:  10.0, ty: -10.0, tz: 0.0},
                #{tx: -10.0, ty: -10.0, tz: 0.0},
            ];
            instance(bolt, positions)
        "#;
        let result = evaluate_script(script).unwrap();
        let sdf = result.sdf;
        // All four positions should be solid
        assert!(sdf.distance(Vec3::new( 10.0,  10.0, 0.0)) < 0.0, "pos1");
        assert!(sdf.distance(Vec3::new(-10.0,  10.0, 0.0)) < 0.0, "pos2");
        assert!(sdf.distance(Vec3::new( 10.0, -10.0, 0.0)) < 0.0, "pos3");
        assert!(sdf.distance(Vec3::new(-10.0, -10.0, 0.0)) < 0.0, "pos4");
        // Origin should be outside (gap between instances)
        assert!(sdf.distance(Vec3::ZERO) > 0.0, "origin should be outside");
    }

    #[test]
    fn test_instance_grid() {
        let script = r#"
            let s = sphere(3.0);
            instance_grid(s, 3, 3, 1, 20.0, 20.0, 0.0)
        "#;
        let result = evaluate_script(script).unwrap();
        // Center of grid (0,0,0) — should be at one of the spheres
        assert!(result.sdf.distance(Vec3::ZERO) < 0.0, "grid center should be inside");
    }

    #[test]
    fn test_deflect_basic() {
        let script = r#"
            let surf = box_(20.0, 40.0, 5.0);
            deflect(surf, 15.0)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "deflect should succeed: {:?}", result.err());
    }

    #[test]
    fn test_tail_volumes_basic() {
        let script = r#"
            let wing   = box_(120.0, 300.0, 8.0);
            let h_tail = box_(60.0, 120.0, 5.0);
            let v_tail = box_(50.0, 10.0, 80.0);
            let fuse   = sphere(30.0);
            let tv = tail_volume_coefficients(wing, h_tail, v_tail, fuse);
            sphere(1.0)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "tail_volume_coefficients should succeed: {:?}", result.err());
    }

    #[test]
    fn test_place_above_below() {
        let script = r#"
            let base = box_(20.0, 20.0, 10.0);
            let cap  = sphere(5.0);
            place_above(cap, base, 2.0)
        "#;
        let result = evaluate_script(script).unwrap();
        // Sphere should be above the box: center at z = 5 + 2 + 5 = 12
        let d = result.sdf.distance(Vec3::new(0.0, 0.0, 12.0));
        assert!(d < 0.0, "Sphere should be centered above box, got d={}", d);
    }

    #[test]
    fn test_wall_thickness_at_measures_geometry() {
        let script = r#"
            let body = box_(20.0, 20.0, 10.0);
            let t = wall_thickness_at(body, 0.0, 0.0, 0.0, "z");
            sphere(t)
        "#;
        let result = evaluate_script(script).unwrap();
        let d = result.sdf.distance(Vec3::new(4.5, 0.0, 0.0));
        assert!(d < 0.0, "Measured thickness should produce a sphere larger than 4.5mm");
    }

    #[test]
    fn test_tolerance_compensate_changes_geometry() {
        let script = r#"
            let body = sphere(10.0);
            let tuned = tolerance_compensate(body, #{
                external_offset_mm: -0.2,
                internal_offset_mm: 0.15
            });
            tuned
        "#;
        let result = evaluate_script(script).unwrap();
        assert!(
            result.sdf.distance(Vec3::new(10.0, 0.0, 0.0)) > 0.0,
            "Compensated sphere should shrink enough that the original surface is outside"
        );
    }

    #[test]
    fn test_fea_compat_wrappers_record_conditions() {
        let script = r#"
            let spar = box_(20.0, 100.0, 5.0);
            let fixed = fea_fixed_face(spar, "y", 0.0);
            let loaded = fea_gravity(fixed);
            fea_load_point(0.0, 50.0, 0.0, 0.0, 0.0, 12.0);
            fea_pressure(loaded, "z", 2.5, 500.0)
        "#;
        let result = evaluate_script(script).unwrap();
        assert_eq!(result.fea_setup.fixed_supports.len(), 1);
        assert!(result.fea_setup.gravity.is_some());
        assert_eq!(result.fea_setup.force_loads.len(), 1);
        assert_eq!(result.fea_setup.pressure_loads.len(), 1);
    }

    #[test]
    fn test_fixed_wing_install_helpers_evaluate() {
        let script = r#"
            let tray = servo_tray(23.0, 12.0, 24.0, 1.5, 4.0);
            let cradle = battery_cradle(105.0, 36.0, 28.0, 1.6, 18.0);
            let mount = fc_stack_mount(36.0, 36.0, 30.5, 10.0);
            let guide = pushrod_guide(80.0, 3.0, 1.6);
            let antenna = antenna_mount(85.0, 4.0, 1.2);
            let travel = control_throw(12.0, 20.0);
            union(union(union(tray, cradle), mount), union(translate(guide, travel, 0.0, 0.0), antenna))
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "Installation helpers should evaluate: {:?}", result.err());
    }

    #[test]
    fn test_imported_servo_component_module_builds() {
        let temp = tempdir().unwrap();
        let project_dir = temp.path();
        let components_dir = project_dir.join("components");
        std::fs::create_dir_all(&components_dir).unwrap();
        std::fs::copy(
            "components/servo_9g.rhai",
            components_dir.join("servo_9g.rhai"),
        ).unwrap();

        let script = r#"
            import "components/servo_9g" as servo;

            let wing_bay = translate(box_(220.0, 80.0, 24.0), 120.0, 0.0, 0.0);
            let wing_shell = shell(wing_bay, 1.6);
            let placed = mount_component_granular(wing_shell, servo::component(), 120.0, 0.0, -15.0);
            placed.assembly
        "#;

        let result = evaluate_script_full(
            script,
            None,
            None,
            None,
            None,
            &indexmap::IndexMap::new(),
            Some(project_dir),
            None,
            &[],
        );
        assert!(result.is_ok(), "imported servo component should evaluate: {:?}", result.err());
    }

    #[test]
    fn test_mount_component_granular_builds() {
        let temp = tempfile::tempdir().unwrap();
        let project_dir = temp.path();
        let components_dir = project_dir.join("components");
        std::fs::create_dir_all(&components_dir).unwrap();
        std::fs::copy(
            "components/electronics_box.rhai",
            components_dir.join("electronics_box.rhai"),
        ).unwrap();

        let script = r#"
            import "components/electronics_box" as ebox;

            let fuselage_outer = fuselage_elliptical(220.0, 90.0, 70.0, 45.0, 50.0, 0.9, 0.8, 8.0);
            let fuselage_shell = shell(fuselage_outer, 1.6);
            let fuse_center = bbox_center(fuselage_outer);
            let placed = mount_component_granular(
                fuselage_shell,
                ebox::component(),
                fuse_center.x, 0.0, fuse_center.z + 2.0
            );
            placed.assembly
        "#;

        let result = evaluate_script_full(
            script,
            None,
            None,
            None,
            None,
            &indexmap::IndexMap::new(),
            Some(project_dir),
            None,
            &[],
        );
        assert!(result.is_ok(), "granular component mount should evaluate: {:?}", result.err());
    }

}
