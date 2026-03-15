// Application state and UI layout

use eframe::{egui, egui_wgpu, wgpu};
use std::sync::{Arc, RwLock};
use std::collections::HashMap;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::profiles::SplineProfile;
use crate::scripting;
use crate::mesh::{Mesh, adaptive_mc, MeshQuality};
use crate::render::{Camera, RenderState, GridRenderer, AxesRenderer, WireframeRenderer, RaymarchRenderer, SdfGrid, SectionUniforms, ThicknessUniforms};
use crate::analysis::thickness::{compute_thickness, ThicknessResult};
use crate::project::{SectionView, SectionPlane, Axis};
use rayon::prelude::*;
use crate::components::{ComponentRegistry, ComponentInstance};
use crate::node_graph::{EditorMode, NodeGraph, GraphUiState};
use crate::notebook::Notebook;
use crate::scripting::MassPoint;
use crate::mesh::compute_volume;
use crate::ui::spline_editor::{SplineEditorState, show_spline_editor};
use crate::ui::spine_editor::{SpineEditorState, SpineEditorTarget, show_spine_editor};
use crate::ui::project_tree::{ProjectTree, show_project_tree, find_name_in_script,
                               count_occurrences, rename_in_script};
use crate::sdf::spine::LongitudinalSplines;
use crate::settings::AppSettings;

#[derive(Clone, Copy, PartialEq, Default)]
pub enum FEAOverlayMode { #[default] None, Stress, Displacement }

pub struct App {
    script_text: String,
    current_sdf: Option<Arc<dyn Sdf>>,
    current_mesh: Option<Mesh>,
    current_sdf_grid: Option<Arc<SdfGrid>>,
    error_message: Option<String>,

    // Rendering state
    camera: Camera,

    // Mouse state for camera control
    last_mouse_pos: Option<egui::Pos2>,
    is_dragging_left: bool,
    is_dragging_right: bool,

    // Status feedback
    eval_time_ms: Option<f64>,
    mesh_time_ms: Option<f64>,
    is_processing: bool,

    // Mesh quality controls
    resolution: u32,
    smooth_normals: bool,
    mesh_quality: MeshQuality,
    show_wireframe: bool,

    // Project management
    current_file_path: Option<std::path::PathBuf>,
    status_message: Option<String>,

    // Undo/redo
    history: Vec<String>,
    history_index: usize,

    // Auto-save
    last_auto_save: std::time::Instant,
    auto_save_interval: std::time::Duration,

    // Component library
    component_registry: ComponentRegistry,

    // Parameter editor modal
    param_editor_open: bool,
    param_editor_instance: Option<ComponentInstance>,

    // Save as component modal
    save_component_open: bool,
    save_component_name: String,
    save_component_category: String,
    save_component_description: String,

    // Node graph editor
    editor_mode: EditorMode,
    node_graph: NodeGraph,
    graph_ui_state: GraphUiState,
    node_graph_history: Vec<NodeGraph>,
    node_graph_history_index: usize,

    // Notebook editor
    notebook: Notebook,
    // Debounce: time of last notebook edit (re-render fires 600ms after last change)
    last_nb_edit: Option<std::time::Instant>,

    // Mass / CG analysis
    density_g_per_cm3: f32,
    mass_points: Vec<MassPoint>,
    cg: Option<glam::Vec3>,
    mesh_volume_mm3: Option<f32>,

    // Background export
    export_in_progress: bool,
    export_progress: String,
    export_receiver: Option<std::sync::mpsc::Receiver<String>>,

    // Spline cross-section profiles
    /// Editor state (serialised with project).
    profiles: HashMap<String, SplineEditorState>,
    /// Name of the profile currently open in the spline editor, or None.
    active_profile: Option<String>,
    /// Thread-safe runtime view shared with the Rhai engine.
    profiles_shared: Arc<RwLock<HashMap<String, SplineProfile>>>,

    // Longitudinal spine
    splines: LongitudinalSplines,
    /// UI state for the spine editor.
    spine_editor_state: SpineEditorState,
    /// Whether the spine editor panel is open.
    spine_editor_open: bool,
    /// Fuselage length used by the spine editor canvas domain.
    spine_fuselage_length: f32,

    // Section view clipping
    section_view: SectionView,

    // Wall thickness analysis
    thickness_max_display:   f32,
    thickness_running:       bool,
    thickness_result:        Option<ThicknessResult>,
    thickness_overlay_on:    bool,
    thickness_receiver:      Option<std::sync::mpsc::Receiver<ThicknessResult>>,
    /// Set when new thickness data should be uploaded to the GPU on the next frame.
    thickness_needs_upload:  bool,

    // FEA integration
    fea_setup:               crate::fea::FEASetup,
    fea_config:              crate::fea::FEAConfig,
    fea_running:             bool,
    fea_log:                 Vec<String>,
    fea_result:              Option<Box<crate::fea::FEAGridResult>>,
    fea_overlay_mode:        FEAOverlayMode,
    fea_receiver:            Option<std::sync::mpsc::Receiver<crate::fea::FEAMessage>>,
    fea_needs_upload:        bool,
    /// Cached stress field for use by re-run scripts.
    fea_stress_field:        Option<Arc<dyn crate::sdf::field::Field>>,
    /// Cached displacement field for use by re-run scripts.
    fea_displacement_field:  Option<Arc<dyn crate::sdf::field::Field>>,
    /// Pre-computed BC visualization geometry (recomputed each script eval).
    fea_viz:                 Option<crate::fea::FEAVizData>,
    /// Whether to draw BC overlays in the viewport.
    fea_show_conditions:     bool,

    // Application settings
    settings:                AppSettings,
    /// Whether the Settings modal is open.
    settings_open:           bool,

    // Floating panel visibility
    section_view_open:       bool,
    thickness_open:          bool,
    fea_open:                bool,

    // Project tree panel
    project_tree:            ProjectTree,
    /// Character offset in script_text to place the cursor on the next frame.
    pending_cursor_offset:   Option<usize>,
    /// Line number (0-indexed) to scroll the editor to on the next frame.
    pending_scroll_line:     Option<usize>,

    // Measurement tools
    measure_open:            bool,
    measurements:            crate::analysis::MeasurementResults,
    measure_density:         f32,   // g/cm³
    measurements_stale:      bool,
    measure_running:         bool,
    measure_receiver:        Option<std::sync::mpsc::Receiver<(f32, f32, glam::Vec3)>>,
    cs_axis:                 crate::project::Axis,
    cs_position:             f32,
    cs_running:              bool,
    cs_receiver:             Option<std::sync::mpsc::Receiver<f32>>,
    /// Which point is being picked: Some(0) = A, Some(1) = B.
    pick_mode:               Option<u8>,
    point_a:                 Option<glam::Vec3>,
    point_b:                 Option<glam::Vec3>,
    dist_kind:               crate::analysis::DistanceMeasureKind,
    /// Custom projection vector (user-editable, normalized on use).
    custom_proj_vec:         glam::Vec3,
    show_cg_marker:          bool,
    /// Last known viewport rect, used for viewport picking.
    last_viewport_rect:      egui::Rect,
    /// Next auto-label index for saved distance measurements.
    distance_label_idx:      usize,

    // Script editor enhancements
    /// Autocomplete popup state.
    autocomplete_state:      crate::ui::AutocompleteState,
    /// Scroll offset of the code editor last frame (used for autocomplete positioning).
    last_editor_scroll_y:    f32,
}

impl App {
    pub fn new(_cc: &eframe::CreationContext) -> Self {
        let mut app = Self {
            script_text: Self::get_default_example(),
            current_sdf: None,
            current_mesh: None,
            current_sdf_grid: None,
            error_message: None,
            camera: Camera::new(16.0 / 9.0),
            last_mouse_pos: None,
            is_dragging_left: false,
            is_dragging_right: false,
            eval_time_ms: None,
            mesh_time_ms: None,
            is_processing: false,
            resolution: 96,
            smooth_normals: true,
            mesh_quality: MeshQuality::Normal,
            show_wireframe: false,
            current_file_path: None,
            status_message: None,
            export_in_progress: false,
            export_progress: String::new(),
            export_receiver: None,
            history: vec![Self::get_default_example()],
            history_index: 0,
            last_auto_save: std::time::Instant::now(),
            auto_save_interval: std::time::Duration::from_secs(30),
            component_registry: {
                let mut registry = ComponentRegistry::new();

                // Try multiple locations for components directory
                let paths_to_try = vec![
                    "components",  // Current directory
                    "./components", // Explicitly current directory
                    "../components", // Parent directory
                ];

                let mut loaded = false;
                for path in paths_to_try {
                    if let Ok(count) = registry.load_from_directory(path) {
                        if count > 0 {
                            eprintln!("✓ Loaded {} components from: {}", count, path);
                            loaded = true;
                            break;
                        }
                    }
                }

                if !loaded {
                    eprintln!("⚠ No components found. Create components/ directory with .json files");
                    if let Ok(cwd) = std::env::current_dir() {
                        eprintln!("  Current directory: {}", cwd.display());
                    }
                }

                registry
            },
            param_editor_open: false,
            param_editor_instance: None,
            save_component_open: false,
            save_component_name: String::new(),
            save_component_category: String::from("custom"),
            save_component_description: String::new(),
            editor_mode: EditorMode::Script,
            node_graph: NodeGraph::with_output(),
            graph_ui_state: GraphUiState::default(),
            node_graph_history: vec![NodeGraph::with_output()],
            node_graph_history_index: 0,
            notebook: Notebook::default(),
            last_nb_edit: None,
            density_g_per_cm3: 1.0,
            mass_points: Vec::new(),
            cg: None,
            mesh_volume_mm3: None,
            profiles: HashMap::new(),
            active_profile: None,
            profiles_shared: Arc::new(RwLock::new(HashMap::new())),
            splines: LongitudinalSplines::default(),
            spine_editor_state: SpineEditorState::default(),
            spine_editor_open: false,
            spine_fuselage_length: 10.0,
            section_view: SectionView::default(),
            thickness_max_display:  10.0,
            thickness_running:      false,
            thickness_result:       None,
            thickness_overlay_on:   false,
            thickness_receiver:     None,
            thickness_needs_upload: false,
            fea_setup:              crate::fea::FEASetup::default(),
            fea_config:             crate::fea::FEAConfig::default(),
            fea_running:            false,
            fea_log:                Vec::new(),
            fea_result:             None,
            fea_overlay_mode:       FEAOverlayMode::None,
            fea_receiver:           None,
            fea_needs_upload:       false,
            fea_stress_field:       None,
            fea_displacement_field: None,
            fea_viz:                None,
            fea_show_conditions:    true,
            settings:               AppSettings::load(),
            settings_open:          false,
            section_view_open:      false,
            thickness_open:         false,
            fea_open:               false,
            project_tree:           ProjectTree::default(),
            pending_cursor_offset:  None,
            pending_scroll_line:    None,
            measure_open:           false,
            measurements:           crate::analysis::MeasurementResults::default(),
            measure_density:        1.24,
            measurements_stale:     false,
            measure_running:        false,
            measure_receiver:       None,
            cs_axis:                crate::project::Axis::X,
            cs_position:            0.0,
            cs_running:             false,
            cs_receiver:            None,
            pick_mode:              None,
            point_a:                None,
            point_b:                None,
            dist_kind:              crate::analysis::DistanceMeasureKind::Distance3D,
            custom_proj_vec:        glam::Vec3::X,
            show_cg_marker:         false,
            last_viewport_rect:     egui::Rect::NOTHING,
            distance_label_idx:     1,
            autocomplete_state:     crate::ui::AutocompleteState::default(),
            last_editor_scroll_y:   0.0,
        };

        // Try to restore from auto-save
        app.try_restore_auto_save();
        app
    }

    fn get_default_example() -> String {
        "// Simple sphere\nlet s = sphere(10.0);\ns".to_string()
    }

    fn get_insert_snippets() -> Vec<(&'static str, &'static str, &'static str)> {
        // (category, name, code)
        vec![
            // Primitives
            ("Primitives", "Sphere", "sphere(10.0)"),
            ("Primitives", "Box", "box_(20.0, 15.0, 10.0)"),
            ("Primitives", "Cylinder", "cylinder(5.0, 20.0)"),
            ("Primitives", "Torus", "torus(15.0, 3.0)"),
            ("Primitives", "Cone", "cone(5.0, 10.0)"),
            ("Primitives", "Plane", "plane(0.0, 0.0, 1.0, 0.0)"),

            // Operations
            ("Operations", "Union", "union(<a>, <b>)"),
            ("Operations", "Subtract", "subtract(<base>, <hole>)"),
            ("Operations", "Intersect", "intersect(<a>, <b>)"),
            ("Operations", "Smooth Union", "smooth_union(<a>, <b>, 2.0)"),
            ("Operations", "Offset", "offset(<shape>, 2.0)"),
            ("Operations", "Shell", "shell(<shape>, 1.0)"),

            // Transforms
            ("Transforms", "Translate", "translate(<shape>, 0.0, 0.0, 0.0)"),
            ("Transforms", "Rotate", "rotate(<shape>, 0.0, 0.0, 45.0)"),
            ("Transforms", "Scale", "scale(<shape>, 1.0, 1.0, 1.0)"),

            // Patterns (examples)
            ("Patterns", "Bracket (base with holes)", "let base = box_(40.0, 20.0, 10.0);\nlet hole = cylinder(4.0, 12.0);\nlet hole = translate(hole, 10.0, 5.0, 0.0);\nsubtract(base, hole)"),
            ("Patterns", "Smooth blend", "let s1 = sphere(8.0);\nlet s2 = translate(sphere(8.0), 10.0, 0.0, 0.0);\nsmooth_union(s1, s2, 3.0)"),
            ("Patterns", "Hollow shape", "let t = torus(15.0, 3.0);\nshell(t, 1.0)"),

            // Arrays & Mirrors
            ("Arrays", "Linear Array (X)", "linear_array(<shape>, 4, 10.0, 0.0, 0.0)"),
            ("Arrays", "Linear Array (Y)", "linear_array(<shape>, 4, 0.0, 10.0, 0.0)"),
            ("Arrays", "Polar Array (Z)", "polar_array(<shape>, 6)"),
            ("Arrays", "Polar Array (custom axis)", "polar_array_axis(<shape>, 6, 0.0, 0.0, 1.0)"),
            ("Arrays", "Mirror X", "mirror_x(<shape>)"),
            ("Arrays", "Mirror Y", "mirror_y(<shape>)"),
            ("Arrays", "Mirror Z", "mirror_z(<shape>)"),

            // Constraint Design (scratchpad pattern)
            ("Constraints", "component(sdf, margin)", "let bat = component(box_(80.0, 30.0, 20.0), 5.0);\nlet bat = place(bat, 0.0, 0.0, 0.0);\ngeometry(bat)"),
            ("Constraints", "component_named(name, sdf, margin, mass)", "let bat = component_named(\"battery\", box_(80.0, 30.0, 20.0), 5.0, 180.0);\nlet bat = place(bat, 0.0, 0.0, 0.0);\ngeometry(bat)"),
            ("Constraints", "keepout(comp)", "let c = component(sphere(10.0), 3.0);\nlet c = place(c, 0.0, 0.0, 0.0);\nkeepout(c)"),
            ("Constraints", "smooth_subtract", "smooth_subtract(<base>, <tool>, 5.0)"),
            ("Constraints", "Fuselage wraps battery", "// Battery defines the fuselage envelope\nlet bat = component_named(\"battery\", box_(80.0, 30.0, 20.0), 5.0, 180.0);\nlet bat = place(bat, 0.0, 0.0, 0.0);\nlet shell = offset(keepout(bat), 2.0);\nlet aero = fuselage_parametric(120.0, 50.0, 0.7, 0.5);\nsmooth_union(aero, shell, 10.0)"),
            // Legacy mass_at
            ("Constraints", "mass_at (standalone)", "mass_at(150.0, 0.0, 0.0, 0.0);\nsphere(5.0)"),

            // Scripting helpers
            ("Scripting", "User function", "fn make_part(r) {\n    sphere(r)\n}\nmake_part(10.0)"),
            ("Scripting", "For loop (union)", "let result = sphere(3.0);\nfor i in 1..5 {\n    let s = translate(sphere(2.0), i * 8.0, 0.0, 0.0);\n    result = union(result, s);\n}\nresult"),
            ("Scripting", "to_rad / PI", "let r = 20.0 * sin(PI / 6.0);\nsphere(r)"),
            ("Scripting", "clamp", "clamp(<value>, 0.0, 1.0)"),
            ("Scripting", "lerp", "lerp(<a>, <b>, 0.5)"),
        ]
    }

    fn get_example_list() -> Vec<(&'static str, &'static str)> {
        vec![
            ("Simple Sphere", "// Simple sphere\nlet s = sphere(10.0);\ns"),
            ("Box", "// Box primitive\nlet b = box_(20.0, 15.0, 10.0);\nb"),
            ("Cylinder", "// Cylinder primitive\nlet c = cylinder(5.0, 20.0);\nc"),
            ("Torus", "// Torus primitive\nlet t = torus(15.0, 3.0);\nt"),
            ("Boolean Union", "// Two overlapping spheres\nlet s1 = sphere(8.0);\nlet s2 = translate(sphere(8.0), 10.0, 0.0, 0.0);\nunion(s1, s2)"),
            ("Boolean Subtract", "// Sphere with cylindrical hole\nlet s = sphere(12.0);\nlet c = cylinder(4.0, 30.0);\nsubtract(s, c)"),
            ("Smooth Blend", "// Smooth blended spheres\nlet s1 = sphere(8.0);\nlet s2 = translate(sphere(8.0), 10.0, 0.0, 0.0);\nsmooth_union(s1, s2, 3.0)"),
            ("Hollow Shell", "// Hollow torus\nlet t = torus(15.0, 3.0);\nshell(t, 1.0)"),
            ("Bracket", "// Simple bracket\nlet base = box_(40.0, 20.0, 10.0);\nlet hole = cylinder(4.0, 12.0);\nlet hole = translate(hole, 10.0, 5.0, 0.0);\nlet part = subtract(base, hole);\nlet part = offset(part, 1.0);\npart"),
            ("Rotated Box", "// Rotated box\nlet b = box_(15.0, 8.0, 5.0);\nlet b = rotate(b, 0.0, 0.0, 45.0);\nb"),
            ("Wing-Body Blend", "\
// Smooth wing-body fairing
// fuselage_parametric(length, diameter, nose_sharpness, tail_sharpness)
let body = fuselage_parametric(60.0, 8.0, 0.7, 0.1);

// wing_with_airfoil(naca, root_chord, tip_chord, span, sweep_deg, dihedral_deg, twist_deg)
let wing = wing_with_airfoil(\"2412\", 15.0, 8.0, 80.0, 12.0, 3.0, -1.5);

// Translate leading edge to ~mid-fuselage
let wing = translate(wing, 20.0, 0.0, 0.0);

// blend(a, b, radius) = smooth union — radius controls fillet size
blend(body, wing, 3.0)
"),
            ("Constraint Fuselage", "// Constraint-driven fuselage:\n// Battery is source of truth. Fuselage is a function of battery placement.\n// Move the battery -> keepout updates -> shell updates -> OML updates.\nlet bat  = component_named(\"battery\",  box_(80.0, 30.0, 20.0), 5.0, 180.0);\nlet fc   = component_named(\"avionics\", box_(36.0, 36.0,  8.0), 3.0,  25.0);\nlet motor= component_named(\"motor\",    cylinder(15.0, 30.0),   5.0,  45.0);\n\nlet bat   = place(bat,   0.0,  0.0, 0.0);\nlet fc    = place(fc,   50.0,  0.0, 0.0);\nlet motor = place(motor, -70.0, 0.0, 0.0);\n\n// Internal envelope: smooth union of all keepout zones\nlet envelope = smooth_union(keepout(bat), keepout(fc), 8.0);\nlet envelope = smooth_union(envelope, keepout(motor), 8.0);\n\n// Structural shell wraps the envelope\nlet shell = offset(envelope, 2.0);\n\n// OML: aero profile smoothly blended with the constraint shell\nlet aero = fuselage_parametric(200.0, 60.0, 0.7, 0.5);\nsmooth_union(aero, shell, 12.0)"),
            ("Hard-Chine Hull", "\
// ─────────────────────────────────────────────────────────────────────────────
// Hard-chine hull  —  longitudinal spine demo
// ─────────────────────────────────────────────────────────────────────────────
//
// QUICK-START (no editor setup needed)
//   Runs immediately using ellipse sections to approximate hull volume.
//
// FULL CHINE WORKFLOW
//   1. Open \"✏ Profiles\" → New Profile → name it \"hull_bow\", \"hull_mid\", \"hull_aft\"
//   2. Shape the cross-section in the spline editor.
//      Right-click bottom-most point → Role: Keel
//      Right-click top-most point    → Role: Deck
//      Right-click widest point      → Role: Chine  (check Sharp corner for hard chine)
//   3. Open \"📐 Longitudinal Spine\" (set length = 8.0).
//      Draw Keel curve: dips ~0.15 at midship, rises to 0 at bow/stern.
//      Draw Deck curve: roughly flat at 0.5.
//      Draw Chine Y curve: widens from 0.05 at bow to 0.6 amidships.
//   4. Uncomment spline_fuselage() below and comment out fuselage().
// ─────────────────────────────────────────────────────────────────────────────

let LENGTH = 8.0;
let BEAM   = 1.4;
let DEPTH  = 0.65;

// Cross-sections — replace with spline_section(\"hull_*\") once profiles are drawn
let s_stem    = ellipse_section(0.04, 0.12);
let s_bow     = ellipse_section(0.25, 0.42);
let s_fwd     = ellipse_section(0.55, 0.52);
let s_mid     = ellipse_section(0.72, 0.50);
let s_aft     = ellipse_section(0.68, 0.48);
let s_transom = ellipse_section(0.58, 0.42);

let hull_unit = fuselage([
    [0.00, s_stem   ],
    [0.07, s_bow    ],
    [0.22, s_fwd    ],
    [0.45, s_mid    ],
    [0.65, s_mid    ],
    [0.82, s_aft    ],
    [1.00, s_transom],
]);

// Spine-constrained version — uncomment once profiles + spine are configured:
// let hull_unit = spline_fuselage([
//     [0.00, spline_section(\"hull_bow\")],
//     [0.22, spline_section(\"hull_mid\")],
//     [0.65, spline_section(\"hull_mid\")],
//     [1.00, spline_section(\"hull_aft\")],
// ], LENGTH);

let hull = scale(hull_unit, LENGTH, BEAM, DEPTH);

// Keel fin — thin slab along the bottom centreline
let keel = box_(LENGTH * 0.5, 0.06, DEPTH * 0.30);
let keel = translate(keel, LENGTH * 0.60, 0.0, -(DEPTH * 0.65));
smooth_union(hull, keel, 0.04)
"),
            ("Drone Bulkheads", "\
// Two structural bulkheads inside a fuselage.
// The fuselage shape is used only as a cutting tool — only the rings are returned.

let fuse_norm = fuselage([
    [0.0, circle_section(0.05)],
    [0.2, circle_section(0.18)],
    [0.5, circle_section(0.22)],
    [0.8, circle_section(0.18)],
    [1.0, circle_section(0.05)],
]);
// Scale to real units: 100 long, 20 wide, 20 tall
let fuse = scale(fuse_norm, 100.0, 20.0, 20.0);

// bulkhead_at_station(fuselage, x_position, thickness, num_holes, hole_radius_fraction)
let frame_a = bulkhead_at_station(fuse, 30.0, 2.0, 6, 0.35);
let frame_b = bulkhead_at_station(fuse, 70.0, 2.0, 6, 0.35);

union(frame_a, frame_b)
"),
            ("Wing Conformal Lattice", "\
// NACA 2412 wing — 100 unit span, filled with a conformal gyroid lattice.
// The lattice follows the wing's curved surface so struts stay parallel
// to the skin, with no Cartesian frame discontinuities.

// Outer wing skin (root chord 20, tip chord 10, span 100)
let wing = wing_with_airfoil(\"2412\", 20.0, 10.0, 100.0, 12.0, 3.0, -1.5);

// conformal_gyroid(parent, cell_size, strut_thickness)
// — confined to the wing interior; skin is preserved outside the lattice
let lattice = conformal_gyroid(wing, 6.0, 1.2);
lattice
"),
            ("FEA Stress-Driven Lattice", "\
// FEA + stress-field-driven conformal lattice demo.
//
// Workflow:
//   1. Run script  →  geometry appears, FEA conditions are registered.
//   2. Click \"Run FEA\" in the FEA panel  →  CalculiX solves the model.
//   3. Re-run the script  →  the lattice density now follows the stress field.
//
// Requires CalculiX (ccx) to be installed or bundled in assets/calculix/.

let fuse = fuselage([
    [0.0, circle_section(0.3)],
    [0.5, circle_section(1.0)],
    [1.0, circle_section(0.3)],
]);
let arm   = motor_arm(fuse, 0.0, 2.0, 0.12, 0.09);
let mount = motor_mount(arm, 0.3, 0.05, 0.12, 0.03);
let structure = union(union(fuse, arm), mount);

// Boundary conditions (collected; FEA run is triggered manually)
fixed_support(sphere(0.1), \"fuselage_root\");
motor_thrust(sphere(0.08), \"motor_0\", 12.0, 800.0, 0.0, 0.0, 1.0);
gravity_load(0.0, 0.0, -9810.0);

// After a successful FEA run, stress_field() returns the solved Von Mises field.
// The conformal lattice becomes denser where stress is highest.
let lattice = conformal_gyroid_field(
    structure, 0.15, 0.3,
    multiply_fields(stress_field(), constant_field(0.5))
);
union(structure, lattice)
"),
            ("Project Tree Demo", "\
// Project Tree Demo
// Shows Components, FEA Conditions, and Mass points in the project tree.
// Run this script, then inspect the Project panel on the left.

// ── Components ────────────────────────────────────────────────────────────────
let battery  = component_named(\"battery\",  box_(80.0, 30.0, 20.0), 5.0, 180.0);
let avionics = component_named(\"avionics\", box_(36.0, 36.0,  8.0), 3.0,  25.0);
let motor    = component_named(\"motor\",    cylinder(15.0, 30.0),   5.0,  45.0);

let battery  = place(battery,   0.0,  0.0,  0.0);
let avionics = place(avionics, 50.0,  0.0,  0.0);
let motor    = place(motor,   -70.0,  0.0,  0.0);

// ── Structural shell ──────────────────────────────────────────────────────────
let envelope = smooth_union(keepout(battery), keepout(avionics), 8.0);
let envelope = smooth_union(envelope, keepout(motor), 8.0);
let shell    = offset(envelope, 2.0);

// ── FEA boundary conditions ───────────────────────────────────────────────────
let mount_region = box_(10.0, 10.0, 10.0);
let mount_region = translate(mount_region, -70.0, 0.0, 0.0);
fixed_support(mount_region, \"motor_mount\");

let aero_load = box_(80.0, 40.0, 5.0);
let aero_load = translate(aero_load, 0.0, 0.0, 15.0);
force_load(aero_load, \"aero_lift\", 0.0, 0.0, -50.0);

gravity_load(0.0, 0.0, -9810.0);

// ── OML ───────────────────────────────────────────────────────────────────────
let aero = fuselage_parametric(200.0, 60.0, 0.7, 0.5);
smooth_union(aero, shell, 12.0)
"),
        ]
    }

    fn execute_notebook(&mut self) {
        let script = crate::notebook::notebook_to_rhai(&self.notebook);
        self.evaluate_and_render(&script);
    }

    fn execute_script(&mut self) {
        let script = self.script_text.clone();

        // Handle empty script - clear mesh
        let trimmed = script.trim();
        if trimmed.is_empty() || trimmed.lines().all(|line| line.trim().starts_with("//")) {
            self.current_sdf = None;
            self.current_mesh = None;
            self.error_message = None;
            self.eval_time_ms = None;
            self.mesh_time_ms = None;
            self.is_processing = false;
            return;
        }

        self.evaluate_and_render(&script);
    }

    fn evaluate_and_render(&mut self, script: &str) {
        use std::time::Instant;
        self.is_processing = true;

        let start_eval = Instant::now();
        let profiles_ref = Arc::clone(&self.profiles_shared);
        let splines_ref = Arc::new(self.splines.clone());
        let sf = self.fea_stress_field.clone();
        let df = self.fea_displacement_field.clone();
        match scripting::evaluate_script_full(script, Some(profiles_ref), Some(splines_ref), sf, df) {
            Ok(result) => {
                let eval_time = start_eval.elapsed().as_secs_f64() * 1000.0;

                // Extract CG/mass data before moving sdf out of result
                let cg = result.center_of_gravity();
                let mass_points = result.mass_points;
                self.fea_setup = result.fea_setup;
                let sdf = result.sdf;

                // Compute bounds once, reuse for both SDF grid and mesh.
                let start_mesh = Instant::now();
                let (bounds_min, bounds_max) = Self::auto_bounds(sdf.as_ref());

                // Build SDF grid for sphere-tracing viewport (resolution³, parallel).
                let sdf_grid = Arc::new(Self::compute_sdf_grid(
                    sdf.as_ref(), bounds_min, bounds_max, self.resolution,
                ));
                let mesh_time = start_mesh.elapsed().as_secs_f64() * 1000.0;

                // Estimate volume from negative voxel count (no marching cubes needed).
                let step = (bounds_max - bounds_min) / self.resolution as f32;
                let voxel_vol = step.x * step.y * step.z;
                let inside = sdf_grid.data.iter().filter(|&&d| d < 0.0).count();
                let vol = inside as f32 * voxel_vol;
                self.mesh_volume_mm3 = Some(vol);
                self.cg = cg;
                self.mass_points = mass_points;

                // Frame camera on tight bounds of negative (interior) voxels.
                let res = self.resolution as usize;
                let mut tight_min = bounds_max;
                let mut tight_max = bounds_min;
                for iz in 0..res { for iy in 0..res { for ix in 0..res {
                    if sdf_grid.data[ix + iy * res + iz * res * res] < 0.0 {
                        let p = bounds_min + Vec3::new(
                            (ix as f32 + 0.5) * step.x,
                            (iy as f32 + 0.5) * step.y,
                            (iz as f32 + 0.5) * step.z,
                        );
                        tight_min = tight_min.min(p);
                        tight_max = tight_max.max(p);
                    }
                }}}
                if tight_min.x <= tight_max.x {
                    self.camera.frame_bounds(tight_min, tight_max);
                } else {
                    self.camera.frame_bounds(bounds_min, bounds_max);
                }

                // Pre-compute FEA BC visualization geometry (cheap 20³ sample).
                if !self.fea_setup.is_empty() {
                    self.fea_viz = Some(crate::fea::compute_fea_viz(
                        &self.fea_setup, bounds_min, bounds_max,
                    ));
                } else {
                    self.fea_viz = None;
                }

                self.current_sdf = Some(sdf);
                self.current_mesh = None; // mesh generated on export only
                self.current_sdf_grid = Some(sdf_grid);
                self.error_message = None;
                self.eval_time_ms = Some(eval_time);
                self.mesh_time_ms = Some(mesh_time);

                // Mark measurements stale (model changed).
                if self.measurements.volume_mm3.is_some() {
                    self.measurements_stale = true;
                }

                // Rebuild project tree from latest data.
                let profile_names: Vec<String> = self.profiles.keys().cloned().collect();
                self.project_tree.rebuild(
                    &self.mass_points,
                    &self.fea_setup,
                    &profile_names,
                    &self.splines,
                );
            }
            Err(e) => {
                self.error_message = Some(e);
                self.eval_time_ms = None;
                self.mesh_time_ms = None;
                // Keep previous mesh visible on error
            }
        }

        self.is_processing = false;
    }

    fn execute_script_from_text(&mut self) {
        // Alias kept for call-sites that reference old name pattern
        self.execute_script();
    }

    fn save_project(&mut self) {
        if let Some(path) = &self.current_file_path {
            self.save_project_to_path(path.clone());
        } else {
            self.save_project_as();
        }
    }

    fn save_project_as(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Implicit CAD Project", &["icad"])
            .save_file()
        {
            self.save_project_to_path(path);
        }
    }

    fn save_project_to_path(&mut self, path: std::path::PathBuf) {
        let profiles_opt = if self.profiles.is_empty() {
            None
        } else {
            Some(self.profiles.clone())
        };
        let splines_opt = if self.splines == LongitudinalSplines::default() {
            None
        } else {
            Some(self.splines.clone())
        };
        let section_opt = Some(self.section_view.clone());
        let project = crate::project::Project::new(
            self.script_text.clone(),
            self.resolution,
            self.smooth_normals,
            self.show_wireframe,
            [self.camera.eye.x, self.camera.eye.y, self.camera.eye.z],
            [self.camera.target.x, self.camera.target.y, self.camera.target.z],
            Some(self.node_graph.clone()),
            Some(self.notebook.clone()),
            profiles_opt,
            splines_opt,
            section_opt,
            Some(self.fea_config.clone()),
        );

        match project.save(&path) {
            Ok(_) => {
                self.current_file_path = Some(path.clone());
                self.status_message = Some(format!("Saved to {}", path.display()));
                // Clear auto-save after explicit save
                Self::clear_auto_save();
            }
            Err(e) => {
                self.error_message = Some(format!("Failed to save: {}", e));
            }
        }
    }

    fn load_project(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Implicit CAD Project", &["icad"])
            .pick_file()
        {
            match crate::project::Project::load(&path) {
                Ok(project) => {
                    self.script_text = project.script;
                    self.resolution = project.resolution;
                    self.smooth_normals = project.smooth_normals;
                    self.show_wireframe = project.show_wireframe;

                    // Restore camera position
                    self.camera.eye = Vec3::from_array(project.camera_position);
                    self.camera.target = Vec3::from_array(project.camera_target);

                    // Restore node graph if present
                    if let Some(ng) = project.node_graph {
                        self.node_graph = ng.clone();
                        self.node_graph_history = vec![ng];
                        self.node_graph_history_index = 0;
                    }

                    // Restore notebook if present
                    if let Some(nb) = project.notebook {
                        self.notebook = nb;
                    }

                    // Restore profiles if present (orphaned profiles are preserved
                    // by merging — existing profiles not in the file are kept).
                    if let Some(loaded_profiles) = project.profiles {
                        for (name, state) in loaded_profiles {
                            self.profiles.insert(name, state);
                        }
                        self.sync_profiles_shared();
                    }

                    // Restore spine constraints if present
                    if let Some(splines) = project.splines {
                        self.splines = splines;
                    }

                    // Restore section view if present
                    if let Some(sv) = project.section_view {
                        self.section_view = sv;
                    }

                    // Restore FEA config if present
                    if let Some(fc) = project.fea_config {
                        self.fea_config = fc;
                    }

                    self.current_file_path = Some(path.clone());
                    self.status_message = Some(format!("Loaded {}", path.display()));

                    // Execute the loaded script
                    self.execute_script();
                }
                Err(e) => {
                    self.error_message = Some(format!("Failed to load: {}", e));
                }
            }
        }
    }

    fn push_history(&mut self) {
        // If we're not at the end of history, truncate forward history
        if self.history_index < self.history.len() - 1 {
            self.history.truncate(self.history_index + 1);
        }

        // Add new state
        self.history.push(self.script_text.clone());

        // Limit history to 50 states
        if self.history.len() > 50 {
            self.history.remove(0);
        } else {
            self.history_index += 1;
        }
    }

    fn undo(&mut self) {
        if self.history_index > 0 {
            self.history_index -= 1;
            self.script_text = self.history[self.history_index].clone();
        }
    }

    fn redo(&mut self) {
        if self.history_index < self.history.len() - 1 {
            self.history_index += 1;
            self.script_text = self.history[self.history_index].clone();
        }
    }

    fn push_graph_history(&mut self) {
        if self.node_graph_history_index < self.node_graph_history.len() - 1 {
            self.node_graph_history.truncate(self.node_graph_history_index + 1);
        }
        self.node_graph_history.push(self.node_graph.clone());
        if self.node_graph_history.len() > 50 {
            self.node_graph_history.remove(0);
        } else {
            self.node_graph_history_index += 1;
        }
    }

    fn undo_graph(&mut self) {
        if self.node_graph_history_index > 0 {
            self.node_graph_history_index -= 1;
            self.node_graph = self.node_graph_history[self.node_graph_history_index].clone();
        }
    }

    fn redo_graph(&mut self) {
        if self.node_graph_history_index < self.node_graph_history.len() - 1 {
            self.node_graph_history_index += 1;
            self.node_graph = self.node_graph_history[self.node_graph_history_index].clone();
        }
    }

    /// Detect the SDF's spatial extent. Returns a tight asymmetric bounding box so that
    /// marching-cubes voxels are distributed proportionally across each axis.
    fn auto_bounds(sdf: &dyn Sdf) -> (Vec3, Vec3) {
        // Fine-Z scan: coarse XY (step=10, ±150) but fine Z (step=1, ±80).
        // This reliably catches thin airfoil cross-sections (thickness ~1.8 units)
        // at any spanwise position, including swept/dihedraled wings where the
        // airfoil centerline drifts in Z. Parallelized over all sample points.
        let nx = 31i32; // -150 to +150, step=10
        let ny = 31i32;
        let nz = 161i32; // -80 to +80, step=1
        let total = (nx * ny * nz) as usize;

        let (lo, hi) = (0..total).into_par_iter().map(|idx| {
            let iz = (idx / (nx * ny) as usize) as i32;
            let iy = ((idx / nx as usize) % ny as usize) as i32;
            let ix = (idx % nx as usize) as i32;
            let p = Vec3::new(
                -150.0 + ix as f32 * 10.0,
                -150.0 + iy as f32 * 10.0,
                -80.0  + iz as f32 * 1.0,
            );
            if sdf.distance(p) < 0.0 { (p, p) }
            else { (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)) }
        }).reduce(
            || (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
            |(lo1, hi1), (lo2, hi2)| (lo1.min(lo2), hi1.max(hi2)),
        );

        let mut bb_min = lo;
        let mut bb_max = hi;

        // Coarse fallback for geometry outside ±150 XY or ±80 Z range
        if bb_min.x == f32::MAX {
            let n = 20i32;
            let total2 = ((n + 1) * (n + 1) * (n + 1)) as usize;
            let (lo2, hi2) = (0..total2).into_par_iter().map(|idx| {
                let iz = (idx / ((n+1)*(n+1)) as usize) as i32;
                let iy = ((idx / (n+1) as usize) % (n+1) as usize) as i32;
                let ix = (idx % (n+1) as usize) as i32;
                let p = Vec3::new(
                    -300.0 + ix as f32 * 30.0,
                    -300.0 + iy as f32 * 30.0,
                    -300.0 + iz as f32 * 30.0,
                );
                if sdf.distance(p) < 0.0 { (p, p) }
                else { (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)) }
            }).reduce(
                || (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
                |(lo1, hi1), (lo2, hi2)| (lo1.min(lo2), hi1.max(hi2)),
            );
            bb_min = lo2;
            bb_max = hi2;
        }

        if bb_min.x == f32::MAX {
            return (Vec3::splat(-100.0), Vec3::splat(100.0));
        }

        // Probe all 6 directions from the centroid for exact extents.
        let center = (bb_min + bb_max) * 0.5;
        let seed = if sdf.distance(center) < 0.0 {
            center
        } else {
            // Find nearest known interior point (the scan min corner is inside)
            bb_min
        };

        if sdf.distance(seed) < 0.0 {
            for &dir in &[Vec3::X, Vec3::NEG_X, Vec3::Y, Vec3::NEG_Y, Vec3::Z, Vec3::NEG_Z] {
                let mut last_inside = seed;
                let mut r = 0.05f32;
                while r < 600.0 {
                    let p = seed + dir * r;
                    if sdf.distance(p) < 0.0 { last_inside = p; }
                    r *= 1.15;
                }
                bb_min = bb_min.min(last_inside);
                bb_max = bb_max.max(last_inside);
            }
        }

        // Pad 10% each side so the surface isn't flush with the grid boundary.
        let span = (bb_max - bb_min).max(Vec3::splat(1.0));
        let pad  = span * 0.10 + Vec3::splat(0.5);
        let lo = (bb_min - pad).max(Vec3::splat(-600.0));
        let hi = (bb_max + pad).min(Vec3::splat( 600.0));
        (lo, hi)
    }

    fn compute_sdf_grid(
        sdf:        &dyn Sdf,
        bounds_min: Vec3,
        bounds_max: Vec3,
        res:        u32,
    ) -> SdfGrid {
        let step = (bounds_max - bounds_min) / res as f32;
        let total = (res * res * res) as usize;

        // Evaluate in parallel; index layout: x + y*res + z*res² (X fastest for wgpu 3D texture).
        let data: Vec<f32> = (0..total)
            .into_par_iter()
            .map(|idx| {
                let x = (idx % res as usize) as u32;
                let y = ((idx / res as usize) % res as usize) as u32;
                let z = (idx / (res * res) as usize) as u32;
                let p = bounds_min + Vec3::new(
                    (x as f32 + 0.5) * step.x,
                    (y as f32 + 0.5) * step.y,
                    (z as f32 + 0.5) * step.z,
                );
                sdf.distance(p)
            })
            .collect();

        SdfGrid { data, resolution: res, bounds_min, bounds_max }
    }

    fn start_export_async(&mut self, path: String, is_obj: bool) {
        let Some(ref sdf) = self.current_sdf else { return };
        let Some(ref grid) = self.current_sdf_grid else { return };

        let sdf = Arc::clone(sdf);
        let bounds_min = grid.bounds_min;
        let bounds_max = grid.bounds_max;
        let quality = self.mesh_quality;
        let smooth = self.smooth_normals;

        let (tx, rx) = std::sync::mpsc::channel::<String>();
        self.export_receiver = Some(rx);
        self.export_in_progress = true;
        self.export_progress = format!("Building mesh at {} quality…", quality.label());

        std::thread::spawn(move || {
            tx.send(format!("Building {} mesh (cell size {:.3} units)…",
                quality.label(), quality.target_cell_size())).ok();

            let mesh = adaptive_mc::extract_mesh_adaptive(
                sdf.as_ref(), bounds_min, bounds_max,
                quality.target_cell_size(), smooth,
            );
            let tri_count = mesh.indices.len() / 3;

            tx.send(format!("Writing {} ({} triangles)…",
                std::path::Path::new(&path).file_name()
                    .and_then(|n| n.to_str()).unwrap_or(&path),
                tri_count)).ok();

            let result = if is_obj {
                crate::export::export_obj(&mesh, &path)
            } else {
                crate::export::export_stl(&mesh, &path)
            };

            match result {
                Ok(_) => tx.send(format!("✓ Saved {} — {} triangles", path, tri_count)).ok(),
                Err(e) => tx.send(format!("✗ Export failed: {}", e)).ok(),
            };
        });
    }

    fn get_auto_save_path() -> std::path::PathBuf {
        let mut path = std::env::temp_dir();
        path.push("implicit_cad_autosave.rhai");
        path
    }

    fn auto_save(&mut self) {
        let path = Self::get_auto_save_path();
        if let Err(e) = std::fs::write(&path, &self.script_text) {
            eprintln!("Auto-save failed: {}", e);
        }
        self.last_auto_save = std::time::Instant::now();
    }

    fn try_restore_auto_save(&mut self) -> bool {
        let path = Self::get_auto_save_path();
        if path.exists() {
            if let Ok(content) = std::fs::read_to_string(&path) {
                if !content.is_empty() && content != Self::get_default_example() {
                    self.script_text = content;
                    self.history = vec![self.script_text.clone()];
                    self.history_index = 0;
                    self.status_message = Some("Restored from auto-save".to_string());
                    return true;
                }
            }
        }
        false
    }

    fn clear_auto_save() {
        let path = Self::get_auto_save_path();
        let _ = std::fs::remove_file(&path);
    }

    /// Rebuild the shared SplineProfile map from the editor states.
    /// Must be called after any profile edit or project load.
    fn sync_profiles_shared(&self) {
        if let Ok(mut write) = self.profiles_shared.write() {
            write.clear();
            for (name, state) in &self.profiles {
                write.insert(name.clone(), state.to_profile());
            }
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Auto-save periodically
        if self.last_auto_save.elapsed() >= self.auto_save_interval {
            self.auto_save();
        }

        // Notebook debounce: re-render 600 ms after the last keystroke
        if let Some(t) = self.last_nb_edit {
            let debounce = std::time::Duration::from_millis(600);
            if t.elapsed() >= debounce {
                self.last_nb_edit = None;
                self.execute_notebook();
            } else {
                ctx.request_repaint_after(debounce - t.elapsed());
            }
        }

        // Request repaint for auto-save timer
        ctx.request_repaint_after(std::time::Duration::from_secs(1));

        // Poll background export thread
        if self.export_in_progress {
            let mut done = false;
            if let Some(ref rx) = self.export_receiver {
                while let Ok(msg) = rx.try_recv() {
                    let finished = msg.starts_with('✓') || msg.starts_with('✗');
                    if msg.starts_with('✗') {
                        self.error_message = Some(msg.clone());
                    } else if msg.starts_with('✓') {
                        self.status_message = Some(msg.clone());
                    }
                    self.export_progress = msg;
                    if finished { done = true; }
                }
            }
            if done {
                self.export_in_progress = false;
                self.export_receiver = None;
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }

        // Poll background measurement analysis thread
        if self.measure_running {
            if let Some(ref rx) = self.measure_receiver {
                if let Ok((vol, sa, com)) = rx.try_recv() {
                    let mass = vol / 1000.0 * self.measure_density;  // mm³ → cm³ → g
                    self.measurements.volume_mm3       = Some(vol);
                    self.measurements.surface_area_mm2 = Some(sa);
                    self.measurements.center_of_mass   = Some(com);
                    self.measurements.print_mass_g     = Some(mass);
                    self.measurements_stale            = false;
                    self.measure_running               = false;
                    self.measure_receiver              = None;
                }
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }

        // Poll background cross-section thread
        if self.cs_running {
            if let Some(ref rx) = self.cs_receiver {
                if let Ok(area) = rx.try_recv() {
                    let label = format!("{:?}={:.1}", self.cs_axis, self.cs_position);
                    self.measurements.cross_sections.push(
                        crate::analysis::CrossSectionMeasurement {
                            label,
                            axis:     self.cs_axis.clone(),
                            position: self.cs_position,
                            area_mm2: area,
                        }
                    );
                    self.cs_running  = false;
                    self.cs_receiver = None;
                }
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }

        // Poll background thickness analysis thread
        if self.thickness_running {
            let mut done = false;
            let mut result = None;
            if let Some(ref rx) = self.thickness_receiver {
                if let Ok(r) = rx.try_recv() {
                    result = Some(r);
                    done   = true;
                }
            }
            if done {
                self.thickness_running  = false;
                self.thickness_receiver = None;
                if let Some(r) = result {
                    self.thickness_result       = Some(r);
                    self.thickness_needs_upload = true;
                    self.thickness_overlay_on   = true;
                }
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }

        // Poll FEA background thread
        if self.fea_running {
            let mut done = false;
            let mut messages = Vec::new();
            if let Some(ref rx) = self.fea_receiver {
                loop {
                    match rx.try_recv() {
                        Ok(msg) => messages.push(msg),
                        Err(_)  => break,
                    }
                }
            }
            for msg in messages {
                use crate::fea::FEAMessage;
                match msg {
                    FEAMessage::Log(s) => self.fea_log.push(s),
                    FEAMessage::Error(e) => {
                        self.fea_log.push(format!("ERROR: {e}"));
                        self.error_message = Some(format!("FEA failed: {e}"));
                        done = true;
                    }
                    FEAMessage::Done(result) => {
                        // Build Field wrappers for stress and displacement
                        let res = self.fea_config.mesh_resolution;
                        let bmin = result.bounds_min;
                        let bmax = result.bounds_max;
                        self.fea_stress_field = Some(Arc::new(crate::fea::GridField {
                            data:       Arc::new(result.von_mises.clone()),
                            bounds_min: bmin,
                            bounds_max: bmax,
                            resolution: res,
                        }));
                        self.fea_displacement_field = Some(Arc::new(crate::fea::GridField {
                            data:       Arc::new(result.displacement.clone()),
                            bounds_min: bmin,
                            bounds_max: bmax,
                            resolution: res,
                        }));
                        self.fea_log.push(format!(
                            "✓ FEA complete. Max stress: {:.2} MPa, Max disp: {:.4} mm",
                            result.max_von_mises, result.max_displacement,
                        ));
                        self.fea_overlay_mode  = FEAOverlayMode::Stress;
                        self.fea_result        = Some(result);
                        self.fea_needs_upload  = true;
                        done = true;
                    }
                }
            }
            if done {
                self.fea_running   = false;
                self.fea_receiver  = None;
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }

        // Handle keyboard shortcuts
        if ctx.input(|i| i.key_pressed(egui::Key::F5) || (i.modifiers.ctrl && i.key_pressed(egui::Key::R))) {
            match self.editor_mode {
                EditorMode::Notebook => self.execute_notebook(),
                _ => self.execute_script(),
            }
        }

        // Save project (Ctrl+S)
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::S)) {
            self.save_project();
        }

        // Load project (Ctrl+O)
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::O)) {
            self.load_project();
        }

        // Undo (Ctrl+Z)
        if ctx.input(|i| i.modifiers.ctrl && !i.modifiers.shift && i.key_pressed(egui::Key::Z)) {
            if self.editor_mode == EditorMode::Graph {
                self.undo_graph();
            } else {
                self.undo();
            }
        }

        // Redo (Ctrl+Y or Ctrl+Shift+Z)
        if ctx.input(|i| (i.modifiers.ctrl && i.key_pressed(egui::Key::Y)) ||
                          (i.modifiers.ctrl && i.modifiers.shift && i.key_pressed(egui::Key::Z))) {
            if self.editor_mode == EditorMode::Graph {
                self.redo_graph();
            } else {
                self.redo();
            }
        }

        // Camera reset (Home key)
        if ctx.input(|i| i.key_pressed(egui::Key::Home)) {
            self.camera.reset();
            // Re-frame mesh if we have one
            if let Some(ref mesh) = self.current_mesh {
                if !mesh.vertices.is_empty() {
                    let mut mesh_min = Vec3::splat(f32::INFINITY);
                    let mut mesh_max = Vec3::splat(f32::NEG_INFINITY);
                    for vertex in &mesh.vertices {
                        let pos = Vec3::from_array(vertex.position);
                        mesh_min = mesh_min.min(pos);
                        mesh_max = mesh_max.max(pos);
                    }
                    self.camera.frame_bounds(mesh_min, mesh_max);
                }
            }
        }

        // ── Top menu bar ────────────────────────────────────────────────────
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                // File menu
                ui.menu_button("File", |ui| {
                    if ui.button("New").clicked() {
                        self.script_text = Self::get_default_example();
                        self.current_sdf = None;
                        self.current_mesh = None;
                        self.current_sdf_grid = None;
                        self.current_file_path = None;
                        ui.close_menu();
                    }
                    if ui.button("Open…  Ctrl+O").clicked() {
                        self.load_project();
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Save    Ctrl+S").clicked() {
                        self.save_project();
                        ui.close_menu();
                    }
                    if ui.button("Save As…").clicked() {
                        self.save_project_as();
                        ui.close_menu();
                    }
                });

                // Export menu — generates mesh on-demand at the chosen quality
                ui.menu_button("Export", |ui| {
                    ui.label("Quality:");
                    for &q in MeshQuality::all() {
                        if ui.selectable_label(self.mesh_quality == q, q.label()).clicked() {
                            self.mesh_quality = q;
                        }
                    }
                    ui.separator();
                    let has_sdf = self.current_sdf.is_some() && !self.export_in_progress;
                    if ui.add_enabled(has_sdf, egui::Button::new("Export STL…")).clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("STL", &["stl"])
                            .set_file_name("output.stl")
                            .save_file()
                        {
                            self.start_export_async(path.to_str().unwrap_or("output.stl").to_string(), false);
                        }
                        ui.close_menu();
                    }
                    if ui.add_enabled(has_sdf, egui::Button::new("Export OBJ…")).clicked() {
                        if let Some(path) = rfd::FileDialog::new()
                            .add_filter("OBJ", &["obj"])
                            .set_file_name("output.obj")
                            .save_file()
                        {
                            self.start_export_async(path.to_str().unwrap_or("output.obj").to_string(), true);
                        }
                        ui.close_menu();
                    }
                });

                // View menu
                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.smooth_normals, "Smooth Normals");
                    ui.checkbox(&mut self.show_wireframe, "Wireframe");
                    ui.separator();
                    if ui.button("Reset Camera  Home").clicked() {
                        self.camera.reset();
                        ui.close_menu();
                    }
                });

                if ui.button("Settings").clicked() {
                    self.settings_open = true;
                }

                ui.separator();

                let sv_label = if self.section_view_open { "✂ Section ✓" } else { "✂ Section" };
                if ui.button(sv_label).clicked() { self.section_view_open = !self.section_view_open; }

                let th_label = if self.thickness_open { "🔍 Thickness ✓" } else { "🔍 Thickness" };
                if ui.button(th_label).clicked() { self.thickness_open = !self.thickness_open; }

                let fea_label = if self.fea_open { "🔬 FEA ✓" } else { "🔬 FEA" };
                if ui.button(fea_label).clicked() { self.fea_open = !self.fea_open; }

                let measure_label = if self.measure_open { "📐 Measure ✓" } else { "📐 Measure" };
                if ui.button(measure_label).clicked() {
                    self.measure_open = !self.measure_open;
                }

                ui.separator();

                // Quick-access run button in menu bar
                let run_label = if self.is_processing { "⏳ Running…" } else { "▶ Run  F5" };
                if ui.button(run_label).clicked() && !self.is_processing {
                    match self.editor_mode {
                        EditorMode::Notebook => self.execute_notebook(),
                        _ => self.execute_script(),
                    }
                }

                // Status / timing on right side
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(ref msg) = self.status_message.clone() {
                        ui.label(egui::RichText::new(msg).small());
                    }
                    if let Some(ms) = self.mesh_time_ms {
                        ui.label(egui::RichText::new(format!("{:.0}ms", ms)).small().weak());
                    }
                });
            });
        });

        // ── Bottom status bar ────────────────────────────────────────────────
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                if self.export_in_progress {
                    ui.spinner();
                    ui.label(egui::RichText::new(&self.export_progress)
                        .color(egui::Color32::from_rgb(100, 180, 255)));
                } else if let Some(ref msg) = self.error_message.clone() {
                    ui.label(egui::RichText::new(format!("⚠ {}", msg))
                        .color(egui::Color32::from_rgb(255, 100, 80)));
                    if ui.small_button("✕").clicked() {
                        self.error_message = None;
                    }
                } else if let Some(ref msg) = self.status_message.clone() {
                    ui.label(egui::RichText::new(msg)
                        .color(egui::Color32::from_rgb(120, 220, 120)));
                } else {
                    ui.label(egui::RichText::new("Ready").weak());
                }

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(ms) = self.eval_time_ms {
                        ui.label(egui::RichText::new(format!("eval {:.0}ms", ms)).small().weak());
                    }
                    if let Some(ms) = self.mesh_time_ms {
                        ui.label(egui::RichText::new(format!("  grid {:.0}ms", ms)).small().weak());
                    }
                });
            });
        });

        // Project tree panel (leftmost)
        egui::SidePanel::left("project_tree_panel")
            .default_width(220.0)
            .min_width(120.0)
            .resizable(true)
            .show(ctx, |ui| {
                ui.heading("Project");
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let interaction = show_project_tree(ui, &mut self.project_tree);

                    // Jump to script definition on node click
                    if let Some(name) = interaction.jump_to_name {
                        if let Some((offset, line)) = find_name_in_script(&self.script_text, &name) {
                            self.pending_cursor_offset = Some(offset);
                            self.pending_scroll_line   = Some(line);
                        }
                    }

                    // Start rename on right-click
                    if let Some((node_id, old_name)) = interaction.start_rename {
                        let count = count_occurrences(&self.script_text, &old_name);
                        self.project_tree.rename = Some(crate::ui::project_tree::RenameState {
                            node_id,
                            old_name: old_name.clone(),
                            buffer: old_name,
                            needs_confirm: count > 3,
                            occurrence_count: count,
                        });
                    }

                    // Apply confirmed rename immediately
                    if let Some((old_name, new_name)) = interaction.confirmed_rename {
                        let new_script = rename_in_script(&self.script_text, &old_name, &new_name);
                        if new_script != self.script_text {
                            self.script_text = new_script;
                            self.push_history();
                            self.execute_script();
                        }
                    }
                });
            });

        // Left panel: Code editor (or spline editor when active_profile is Some)
        egui::SidePanel::left("code_panel")
            .default_width(500.0)
            .resizable(true)
            .show(ctx, |ui| {
                ui.heading("Implicit CAD");

                // ── Spline editor mode ─────────────────────────────────────
                if let Some(ref profile_name) = self.active_profile.clone() {
                    ui.horizontal(|ui| {
                        ui.strong(format!("✏ Profile: {}", profile_name));
                        if ui.button("✕ Close").clicked() {
                            self.active_profile = None;
                            self.execute_script();
                        }
                    });
                    ui.separator();

                    let state = self.profiles
                        .entry(profile_name.clone())
                        .or_insert_with(SplineEditorState::default);

                    if show_spline_editor(ui, state, profile_name) {
                        // Profile changed — push to shared map and rebuild
                        self.sync_profiles_shared();
                        self.execute_script();
                    }
                    return; // Skip the rest of the panel
                }

                // ── Spine editor mode ──────────────────────────────────────
                if self.spine_editor_open {
                    ui.horizontal(|ui| {
                        ui.strong("📐 Longitudinal Spine");
                        ui.separator();
                        ui.label("len:");
                        ui.add(egui::DragValue::new(&mut self.spine_fuselage_length)
                            .range(0.1f32..=500.0).speed(0.1).suffix(" u"));
                        if ui.button("✕ Close").clicked() {
                            self.spine_editor_open = false;
                            self.execute_script();
                        }
                    });
                    ui.separator();

                    let mut editor_state = std::mem::take(&mut self.spine_editor_state);
                    let changed = show_spine_editor(
                        ui,
                        &mut editor_state,
                        &mut self.splines,
                        self.spine_fuselage_length,
                    );
                    self.spine_editor_state = editor_state;
                    if changed {
                        self.execute_script();
                    }
                    return;
                }

                // ── Normal editor mode toggle ──────────────────────────────
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.editor_mode, EditorMode::Script, "📝 Script");
                    ui.selectable_value(&mut self.editor_mode, EditorMode::Notebook, "📓 Notebook");
                });
                ui.separator();

                if self.editor_mode == EditorMode::Notebook {
                    // ── Notebook Mode ────────────────────────────────────────
                    let nb_changed = crate::notebook::show_notebook_panel(ui, &mut self.notebook);
                    if nb_changed {
                        // Debounce: don't re-render on every keystroke — fire 600ms after last change
                        self.last_nb_edit = Some(std::time::Instant::now());
                    }
                } else {
                // ── Script Mode ───────────────────────────────────────────────

                // File operations
                ui.horizontal(|ui| {
                    if ui.button("📁 Open (Ctrl+O)").clicked() {
                        self.load_project();
                    }
                    if ui.button("💾 Save (Ctrl+S)").clicked() {
                        self.save_project();
                    }
                    if ui.button("💾 Save As...").clicked() {
                        self.save_project_as();
                    }
                });

                ui.horizontal(|ui| {
                    if ui.button("📦 Save as Component...").clicked() {
                        self.save_component_open = true;
                    }
                });

                // ── Spine editor button ────────────────────────────────────
                if ui.button("📐 Longitudinal Spine…").clicked() {
                    self.spine_editor_open = true;
                }

                // ── Spline profiles ────────────────────────────────────────
                ui.collapsing("✏ Profiles", |ui| {
                    // List existing profiles
                    let names: Vec<String> = self.profiles.keys().cloned().collect();
                    for name in &names {
                        ui.horizontal(|ui| {
                            if ui.button("Edit").clicked() {
                                self.active_profile = Some(name.clone());
                            }
                            if ui.label(name).secondary_clicked() {
                                // Right-click label → delete (future: context menu)
                            }
                            if ui.small_button("✕").clicked() {
                                self.profiles.remove(name);
                                self.sync_profiles_shared();
                                if self.active_profile.as_deref() == Some(name.as_str()) {
                                    self.active_profile = None;
                                }
                            }
                            if ui.small_button("Insert").clicked() {
                                let snippet = format!("spline_section(\"{}\")", name);
                                if !self.script_text.is_empty() && !self.script_text.ends_with('\n') {
                                    self.script_text.push('\n');
                                }
                                self.script_text.push_str(&snippet);
                            }
                        });
                    }
                    ui.separator();
                    if ui.button("+ New Profile").clicked() {
                        let name = format!("profile{}", self.profiles.len() + 1);
                        self.profiles.insert(name.clone(), SplineEditorState::default());
                        self.sync_profiles_shared();
                        self.active_profile = Some(name);
                    }
                });

                // Examples and Insert dropdowns
                ui.horizontal(|ui| {
                    ui.label("Examples:");
                    egui::ComboBox::from_id_salt("examples")
                        .selected_text("Load Example...")
                        .show_ui(ui, |ui| {
                            for (name, code) in Self::get_example_list() {
                                if ui.selectable_label(false, name).clicked() {
                                    self.script_text = code.to_string();
                                }
                            }
                        });

                    ui.label("Insert:");
                    egui::ComboBox::from_id_salt("insert")
                        .selected_text("Insert Code...")
                        .show_ui(ui, |ui| {
                            let snippets = Self::get_insert_snippets();
                            let mut current_category = "";

                            for (category, name, code) in snippets {
                                if category != current_category {
                                    if !current_category.is_empty() {
                                        ui.separator();
                                    }
                                    ui.label(egui::RichText::new(category).strong());
                                    current_category = category;
                                }

                                if ui.selectable_label(false, name).clicked() {
                                    // Insert at end with newline
                                    if !self.script_text.is_empty() && !self.script_text.ends_with('\n') {
                                        self.script_text.push('\n');
                                    }
                                    self.script_text.push_str(code);
                                }
                            }
                        });
                });

                // Component Library
                ui.collapsing("📦 Component Library", |ui| {
                    let categories = self.component_registry.list_categories();

                    if categories.is_empty() {
                        ui.label(egui::RichText::new("No components found").italics().color(egui::Color32::GRAY));
                        ui.label(egui::RichText::new("Create components/ directory with JSON files").size(10.0).color(egui::Color32::GRAY));
                    } else {
                        for category in categories {
                            ui.collapsing(&category, |ui| {
                                let components = self.component_registry.list_by_category(&category);
                                for component in components {
                                    let button_text = format!("  • {}", component.name);
                                    if ui.button(button_text)
                                        .on_hover_text(&component.description)
                                        .clicked()
                                    {
                                        // Open parameter editor modal
                                        self.param_editor_instance = Some(ComponentInstance::new(component.clone()));
                                        self.param_editor_open = true;
                                    }
                                }
                            });
                        }
                    }
                });

                // Status message
                if let Some(status) = &self.status_message {
                    ui.label(egui::RichText::new(status).color(egui::Color32::from_rgb(100, 200, 100)));
                }

                ui.separator();

                // Multiline text editor with stats
                ui.label(egui::RichText::new(format!("Lines: {} | Characters: {}",
                    self.script_text.lines().count(),
                    self.script_text.len()))
                    .size(11.0)
                    .color(egui::Color32::GRAY));

                // Scroll to target line if a tree node was clicked.
                let line_height = ui.text_style_height(&egui::TextStyle::Monospace);
                let scroll_area = if let Some(line) = self.pending_scroll_line.take() {
                    let visible_rows = (ui.available_height() / line_height).floor() as usize;
                    let target_y = (line.saturating_sub(visible_rows / 2)) as f32 * line_height;
                    egui::ScrollArea::vertical().vertical_scroll_offset(target_y)
                } else {
                    egui::ScrollArea::vertical()
                };

                // Intercept autocomplete navigation keys before TextEdit sees them.
                let (ac_up, ac_down, ac_confirm, ac_dismiss) = if self.autocomplete_state.visible {
                    ctx.input_mut(|i| {
                        let up      = i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp);
                        let down    = i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown);
                        let confirm = i.consume_key(egui::Modifiers::NONE, egui::Key::Tab)
                                   || i.consume_key(egui::Modifiers::NONE, egui::Key::Enter);
                        let dismiss = i.consume_key(egui::Modifiers::NONE, egui::Key::Escape);
                        (up, down, confirm, dismiss)
                    })
                } else {
                    (false, false, false, false)
                };

                if ac_dismiss { self.autocomplete_state.visible = false; }
                if ac_up && self.autocomplete_state.selected_index > 0 {
                    self.autocomplete_state.selected_index -= 1;
                }
                if ac_down {
                    let max = self.autocomplete_state.match_indices.len().saturating_sub(1);
                    if self.autocomplete_state.selected_index < max {
                        self.autocomplete_state.selected_index += 1;
                    }
                }

                // Build syntax-highlighting layouter (no per-frame allocation in the common case:
                // egui caches galleys internally keyed by LayoutJob content).
                let mut layouter = |ui: &egui::Ui, text: &str, wrap_width: f32| {
                    let font_id = ui.style()
                        .text_styles[&egui::TextStyle::Monospace]
                        .clone();
                    let mut job = crate::ui::syntax::highlight_script(text, font_id);
                    job.wrap.max_width = wrap_width;
                    ui.fonts(|f| f.layout_job(job))
                };

                let editor_top = ui.cursor().min.y;
                let scroll_out = scroll_area.show(ui, |ui| {
                    let response = ui.add(
                        egui::TextEdit::multiline(&mut self.script_text)
                            .desired_width(f32::INFINITY)
                            .desired_rows(30)
                            .code_editor()
                            .font(egui::TextStyle::Monospace)
                            .layouter(&mut layouter),
                    );

                    // Apply pending cursor (from project tree node click).
                    if let Some(offset) = self.pending_cursor_offset.take() {
                        let mut state = egui::text_edit::TextEditState::load(ctx, response.id)
                            .unwrap_or_default();
                        let cursor = egui::text::CCursor::new(offset);
                        state.cursor.set_char_range(Some(egui::text::CCursorRange::one(cursor)));
                        state.store(ctx, response.id);
                        response.request_focus();
                    }

                    // Push to history when editor loses focus and text changed
                    if response.lost_focus() && response.changed() {
                        self.push_history();
                    }

                    // Update autocomplete from cursor position.
                    if let Some(te_state) = egui::text_edit::TextEditState::load(ctx, response.id) {
                        if let Some(range) = te_state.cursor.char_range() {
                            let cursor_offset = range.primary.index;

                            // Apply confirmed autocomplete from previous frame.
                            if ac_confirm && self.autocomplete_state.visible {
                                let sel_list_i = self.autocomplete_state.selected_index;
                                if let Some(&global_i) = self.autocomplete_state.match_indices.get(sel_list_i) {
                                    let new_cursor = crate::ui::autocomplete::apply_completion(
                                        &mut self.script_text,
                                        self.autocomplete_state.token_start,
                                        cursor_offset,
                                        &crate::ui::autocomplete::ALL_COMPLETIONS[global_i],
                                    );
                                    // Move cursor to inside the parens.
                                    let mut st = egui::text_edit::TextEditState::load(ctx, response.id)
                                        .unwrap_or_default();
                                    let c = egui::text::CCursor::new(new_cursor);
                                    st.cursor.set_char_range(Some(egui::text::CCursorRange::one(c)));
                                    st.store(ctx, response.id);
                                    self.autocomplete_state.visible = false;
                                }
                            } else {
                                // Compute anchor position for autocomplete popup.
                                let cursor_line = self.script_text[..cursor_offset.min(self.script_text.len())]
                                    .chars().filter(|&c| c == '\n').count();
                                let anchor = egui::pos2(
                                    response.rect.min.x + 20.0,
                                    editor_top + (cursor_line + 1) as f32 * line_height
                                        - self.last_editor_scroll_y,
                                );
                                crate::ui::autocomplete::update_completions(
                                    &mut self.autocomplete_state,
                                    &self.script_text,
                                    cursor_offset,
                                    anchor,
                                );
                            }

                            // Signature tooltip anchor (above cursor line).
                            let cursor_line = self.script_text[..cursor_offset.min(self.script_text.len())]
                                .chars().filter(|&c| c == '\n').count();
                            let tip_anchor = egui::pos2(
                                response.rect.min.x + 20.0,
                                editor_top + cursor_line as f32 * line_height
                                    - self.last_editor_scroll_y - 28.0,
                            );
                            crate::ui::autocomplete::show_signature_tooltip(
                                ctx,
                                &self.script_text,
                                cursor_offset,
                                tip_anchor,
                            );
                        }
                    }

                    response
                });

                self.last_editor_scroll_y = scroll_out.state.offset.y;

                // Show autocomplete popup (outside scroll area so it floats freely).
                let action = crate::ui::autocomplete::show_autocomplete(
                    ctx, &mut self.autocomplete_state,
                );
                if let crate::ui::autocomplete::AutocompleteAction::Confirm(global_i) = action {
                    // Mouse click on completion — need cursor from last known state.
                    // We can't easily get cursor here, so use token_start + token_len as cursor approx.
                    let cursor_approx = self.autocomplete_state.token_start
                        + self.autocomplete_state.token.len();
                    let new_cursor = crate::ui::autocomplete::apply_completion(
                        &mut self.script_text,
                        self.autocomplete_state.token_start,
                        cursor_approx,
                        &crate::ui::autocomplete::ALL_COMPLETIONS[global_i],
                    );
                    // Request a cursor update on next frame via pending_cursor_offset.
                    self.pending_cursor_offset = Some(new_cursor);
                    self.autocomplete_state.visible = false;
                }

                ui.separator();

                // Resolution slider
                ui.horizontal(|ui| {
                    ui.label("Resolution:");
                    if ui.add(egui::Slider::new(&mut self.resolution, 16..=256).logarithmic(true)).changed() {
                        // Re-mesh if we have a current SDF
                        if self.current_sdf.is_some() {
                            self.execute_script();
                        }
                    }
                });

                // Rendering options
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut self.smooth_normals, "Smooth Normals").changed() {
                        // Re-mesh if we have a current SDF
                        if self.current_sdf.is_some() {
                            self.execute_script();
                        }
                    }
                    ui.checkbox(&mut self.show_wireframe, "Show Wireframe");
                });

                // Run button and status
                ui.horizontal(|ui| {
                    if ui.button("Run (F5)").clicked() {
                        self.push_history();
                        self.execute_script();
                    }

                    if self.is_processing {
                        ui.spinner();
                        ui.label("Processing...");
                    }
                });

                ui.horizontal(|ui| {
                    if ui.button("Reset Camera (Home)").clicked() {
                        self.camera.reset();
                        // Re-frame mesh if we have one
                        if let Some(ref mesh) = self.current_mesh {
                            if !mesh.vertices.is_empty() {
                                let mut mesh_min = Vec3::splat(f32::INFINITY);
                                let mut mesh_max = Vec3::splat(f32::NEG_INFINITY);
                                for vertex in &mesh.vertices {
                                    let pos = Vec3::from_array(vertex.position);
                                    mesh_min = mesh_min.min(pos);
                                    mesh_max = mesh_max.max(pos);
                                }
                                self.camera.frame_bounds(mesh_min, mesh_max);
                            }
                        }
                    }
                });

                // Display errors
                if let Some(error) = self.error_message.clone() {
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.colored_label(egui::Color32::RED, "❌ Error");
                        if ui.button("Clear").clicked() {
                            self.error_message = None;
                        }
                    });

                    egui::Frame::none()
                        .fill(egui::Color32::from_rgb(40, 20, 20))
                        .inner_margin(8.0)
                        .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(255, 100, 100)))
                        .show(ui, |ui| {
                            ui.colored_label(egui::Color32::from_rgb(255, 150, 150), &error);

                            // Show syntax hint if it's a common error
                            if error.contains("unexpected") || error.contains("expected") {
                                ui.label(egui::RichText::new("💡 Tip: Check for missing semicolons, parentheses, or variable names")
                                    .color(egui::Color32::from_rgb(200, 200, 150))
                                    .size(12.0));
                            }
                        });
                }

                // Stats and export — shown whenever we have an SDF grid
                if let Some(ref grid) = self.current_sdf_grid {
                    ui.separator();
                    ui.heading("Shape Info");

                    egui::Grid::new("mesh_stats")
                        .num_columns(2)
                        .spacing([20.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("Grid Res:");
                            ui.label(format!("{}³", grid.resolution));
                            ui.end_row();

                            if let (Some(eval_time), Some(mesh_time)) = (self.eval_time_ms, self.mesh_time_ms) {
                                ui.label("Eval Time:");
                                ui.label(format!("{:.1}ms", eval_time));
                                ui.end_row();

                                ui.label("Grid Time:");
                                ui.label(format!("{:.1}ms", mesh_time));
                                ui.end_row();
                            }

                            if let Some(vol_mm3) = self.mesh_volume_mm3 {
                                let vol_cm3 = vol_mm3 / 1000.0;
                                ui.label("Volume ~:");
                                ui.label(format!("{:.2} cm³", vol_cm3));
                                ui.end_row();

                                let mass_g = vol_cm3 * self.density_g_per_cm3;
                                ui.label("Mass ~:");
                                ui.label(format!("{:.1} g", mass_g));
                                ui.end_row();
                            }
                        });

                    // Density control
                    ui.horizontal(|ui| {
                        ui.label("Density:");
                        ui.add(egui::DragValue::new(&mut self.density_g_per_cm3)
                            .speed(0.01)
                            .range(0.001..=20.0)
                            .suffix(" g/cm³"));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Preset:");
                        if ui.small_button("Foam").clicked()  { self.density_g_per_cm3 = 0.05; }
                        if ui.small_button("PLA").clicked()   { self.density_g_per_cm3 = 1.25; }
                        if ui.small_button("CF").clicked()    { self.density_g_per_cm3 = 1.60; }
                        if ui.small_button("Alum").clicked()  { self.density_g_per_cm3 = 2.70; }
                    });

                    // CG / mass budget
                    if !self.mass_points.is_empty() {
                        ui.separator();
                        ui.heading("Mass Budget");
                        let total: f32 = self.mass_points.iter().map(|m| m.mass_g).sum();
                        egui::Grid::new("mass_budget")
                            .num_columns(2)
                            .spacing([20.0, 4.0])
                            .show(ui, |ui| {
                                for mp in &self.mass_points {
                                    let label = if mp.name.is_empty() {
                                        format!("({:.0},{:.0},{:.0})", mp.position.x, mp.position.y, mp.position.z)
                                    } else { mp.name.clone() };
                                    ui.label(label);
                                    ui.label(format!("{:.1} g", mp.mass_g));
                                    ui.end_row();
                                }
                                ui.label("Total:");
                                ui.label(format!("{:.1} g", total));
                                ui.end_row();
                            });
                        if let Some(cg) = self.cg {
                            ui.label(format!("CG: ({:.1}, {:.1}, {:.1})", cg.x, cg.y, cg.z));
                        }
                    }

                    // Export — generates mesh on demand from stored SDF
                    ui.separator();
                    egui::CollapsingHeader::new("Export")
                        .default_open(true)
                        .show(ui, |ui| {
                            ui.label(egui::RichText::new("Mesh generated at export time")
                                .color(egui::Color32::GRAY).size(11.0));
                            // Mesh quality selector for export
                            ui.horizontal(|ui| {
                                ui.label("Export Quality:");
                                for &q in MeshQuality::all() {
                                    let selected = self.mesh_quality == q;
                                    if ui.selectable_label(selected, q.label()).clicked() {
                                        self.mesh_quality = q;
                                    }
                                }
                            });
                            ui.horizontal(|ui| {
                                let has_sdf = self.current_sdf.is_some() && !self.export_in_progress;
                                if ui.add_enabled(has_sdf, egui::Button::new("Export STL")).clicked() {
                                    self.start_export_async("output.stl".to_string(), false);
                                }
                                if ui.add_enabled(has_sdf, egui::Button::new("Export OBJ")).clicked() {
                                    self.start_export_async("output.obj".to_string(), true);
                                }
                            });
                            ui.label("Files saved to current directory");
                        });
                }

                } // end EditorMode::Script
            });

        // Section View floating window
        if self.section_view_open {
            let mut open = true;
            egui::Window::new("✂ Section View")
                .open(&mut open)
                .resizable(true)
                .default_width(220.0)
                .show(ctx, |ui| {
                    self.render_section_view_panel(ui);
                });
            self.section_view_open = open;
        }

        // Wall Thickness floating window
        if self.thickness_open {
            let mut open = true;
            egui::Window::new("🔍 Wall Thickness")
                .open(&mut open)
                .resizable(true)
                .default_width(220.0)
                .show(ctx, |ui| {
                    self.render_thickness_panel(ui);
                });
            self.thickness_open = open;
        }

        // FEA floating window
        if self.fea_open {
            let mut open = true;
            egui::Window::new("🔬 FEA")
                .open(&mut open)
                .resizable(true)
                .default_width(280.0)
                .show(ctx, |ui| {
                    self.render_fea_panel(ui);
                });
            self.fea_open = open;
        }

        // Center panel: 3D Viewport
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::Frame::canvas(ui.style()).show(ui, |ui| {
                self.render_3d_viewport(ui);
            });
        });

        // Parameter editor modal (rendered on top)
        self.render_param_editor_modal(ctx);

        // Save as component modal
        self.render_save_component_modal(ctx);

        // Settings modal
        self.show_settings_modal(ctx);

        // Measurement popup
        self.show_measure_popup(ctx);
    }
}

impl App {
    fn render_param_editor_modal(&mut self, ctx: &egui::Context) {
        if !self.param_editor_open {
            return;
        }

        let mut should_close = false;
        let mut should_apply = false;

        egui::Window::new("Component Parameters")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                if let Some(ref mut instance) = self.param_editor_instance {
                    ui.heading(&instance.component_def.name);
                    ui.label(&instance.component_def.description);
                    ui.add_space(10.0);

                    ui.separator();
                    ui.add_space(5.0);

                    // Render parameter controls
                    let mut param_names: Vec<String> = instance.param_values.keys().cloned().collect();
                    param_names.sort(); // Alphabetical order

                    for param_name in param_names {
                        if let Some(param_def) = instance.component_def.parameters.get(&param_name) {
                            if let Some(param_value) = instance.param_values.get_mut(&param_name) {
                                ui.horizontal(|ui| {
                                    ui.label(&param_name);
                                    if let Some(desc) = &param_def.description {
                                        ui.label(egui::RichText::new(format!("({})", desc)).italics().small().color(egui::Color32::GRAY));
                                    }
                                });

                                match param_value {
                                    crate::components::ParamValue::Float(value) => {
                                        let min = param_def.min.unwrap_or(0.0) as f64;
                                        let max = param_def.max.unwrap_or(100.0) as f64;
                                        ui.add(egui::Slider::new(value, min..=max).text(&param_name));
                                    }
                                    crate::components::ParamValue::Int(value) => {
                                        let min = param_def.min.unwrap_or(0.0) as i64;
                                        let max = param_def.max.unwrap_or(100.0) as i64;
                                        ui.add(egui::Slider::new(value, min..=max).text(&param_name));
                                    }
                                    crate::components::ParamValue::Bool(value) => {
                                        ui.checkbox(value, "");
                                    }
                                    crate::components::ParamValue::String(value) => {
                                        ui.text_edit_singleline(value);
                                    }
                                }

                                ui.add_space(5.0);
                            }
                        }
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(5.0);

                    // Action buttons
                    ui.horizontal(|ui| {
                        if ui.button("✓ Apply").clicked() {
                            should_apply = true;
                        }
                        if ui.button("✗ Cancel").clicked() {
                            should_close = true;
                        }
                    });
                }
            });

        // Handle apply action
        if should_apply {
            if let Some(ref instance) = self.param_editor_instance {
                match instance.generate_script() {
                    Ok(script) => {
                        // Ensure existing script's last expression ends with ';'
                        // so the component becomes the new return value
                        self.script_text = Self::terminate_last_expression(self.script_text.clone());
                        if !self.script_text.is_empty() && !self.script_text.ends_with('\n') {
                            self.script_text.push('\n');
                        }
                        self.script_text.push_str(&script);
                        should_close = true;
                    }
                    Err(e) => {
                        self.status_message = Some(format!("Error generating component: {}", e));
                    }
                }
            }
        }

        // Handle close
        if should_close {
            self.param_editor_open = false;
            self.param_editor_instance = None;
        }
    }

    fn render_save_component_modal(&mut self, ctx: &egui::Context) {
        if !self.save_component_open {
            return;
        }

        let mut should_close = false;
        let mut should_save = false;

        egui::Window::new("Save as Component")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.label("Create a reusable component from the current script.");
                ui.label(egui::RichText::new("Tip: Use #{param_name} in your script for parameters").small().italics());
                ui.add_space(10.0);

                ui.horizontal(|ui| {
                    ui.label("Name:");
                    ui.text_edit_singleline(&mut self.save_component_name);
                });

                ui.horizontal(|ui| {
                    ui.label("Category:");
                    ui.text_edit_singleline(&mut self.save_component_category);
                });

                ui.horizontal(|ui| {
                    ui.label("Description:");
                    ui.text_edit_singleline(&mut self.save_component_description);
                });

                ui.add_space(10.0);
                ui.separator();
                ui.add_space(5.0);

                ui.horizontal(|ui| {
                    if ui.button("💾 Save").clicked() {
                        if !self.save_component_name.is_empty() && !self.save_component_category.is_empty() {
                            should_save = true;
                        } else {
                            self.status_message = Some("Name and category are required".to_string());
                        }
                    }
                    if ui.button("✗ Cancel").clicked() {
                        should_close = true;
                    }
                });
            });

        if should_save {
            match self.save_current_script_as_component() {
                Ok(path) => {
                    self.status_message = Some(format!("Component saved to {}", path));
                    // Reload registry
                    let paths_to_try = vec!["components", "./components", "../components"];
                    for path in paths_to_try {
                        if let Ok(count) = self.component_registry.load_from_directory(path) {
                            if count > 0 {
                                break;
                            }
                        }
                    }
                    should_close = true;
                }
                Err(e) => {
                    self.status_message = Some(format!("Error saving component: {}", e));
                }
            }
        }

        if should_close {
            self.save_component_open = false;
            self.save_component_name.clear();
            self.save_component_category = String::from("custom");
            self.save_component_description.clear();
        }
    }

    fn show_settings_modal(&mut self, ctx: &egui::Context) {
        if !self.settings_open { return; }

        let mut open = self.settings_open;
        egui::Window::new("Settings")
            .open(&mut open)
            .resizable(false)
            .collapsible(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.set_min_width(360.0);

                ui.heading("CalculiX");
                ui.separator();
                ui.label("Path to ccx binary (leave blank to auto-detect):");

                let mut path_str = self.settings.ccx_path.clone().unwrap_or_default();
                let changed = ui.horizontal(|ui| {
                    let r = ui.add(
                        egui::TextEdit::singleline(&mut path_str)
                            .hint_text("Auto (bundled or PATH)")
                            .desired_width(260.0),
                    );
                    if ui.button("Browse…").clicked() {
                        if let Some(p) = rfd::FileDialog::new().pick_file() {
                            path_str = p.to_string_lossy().into_owned();
                        }
                    }
                    r.changed()
                }).inner;

                if changed || !path_str.is_empty() {
                    self.settings.ccx_path = if path_str.trim().is_empty() {
                        None
                    } else {
                        Some(path_str)
                    };
                }

                // Show resolved binary
                let resolved = crate::fea::calculix::find_calculix(
                    self.settings.ccx_path.as_deref()
                );
                ui.add_space(4.0);
                match resolved {
                    Some(p) => ui.label(
                        egui::RichText::new(format!("Found: {}", p.display()))
                            .small().color(egui::Color32::GREEN)
                    ),
                    None => ui.label(
                        egui::RichText::new("Not found — FEA will be unavailable")
                            .small().color(egui::Color32::RED)
                    ),
                };

                ui.add_space(8.0);
                ui.separator();
                if ui.button("Save").clicked() {
                    self.settings.save();
                    self.settings_open = false;
                }
            });

        self.settings_open = open;
    }

    /// Add ';' to the last meaningful line if it doesn't already end with one.
    /// This prevents Rhai from treating an existing bare expression as the final return
    /// when more code is appended after it.
    fn terminate_last_expression(script: String) -> String {
        let lines: Vec<&str> = script.lines().collect();
        let last_idx = lines.iter().rposition(|l| {
            let t = l.trim();
            !t.is_empty() && !t.starts_with("//")
        });
        if let Some(idx) = last_idx {
            let line = lines[idx].trim_end();
            if !line.ends_with(';') && !line.ends_with('{') && !line.ends_with('}') {
                let mut new_lines: Vec<String> = lines.iter().map(|l| l.to_string()).collect();
                new_lines[idx] = format!("{};", line);
                let mut result = new_lines.join("\n");
                if script.ends_with('\n') {
                    result.push('\n');
                }
                return result;
            }
        }
        script
    }

    fn save_current_script_as_component(&self) -> Result<String, String> {
        use std::collections::HashMap;
        use crate::components::ComponentDef;

        // Create component directory if it doesn't exist
        let category_dir = format!("components/{}", self.save_component_category);
        std::fs::create_dir_all(&category_dir)
            .map_err(|e| format!("Failed to create directory: {}", e))?;

        // Build component definition
        let component = ComponentDef {
            name: self.save_component_name.clone(),
            category: self.save_component_category.clone(),
            description: self.save_component_description.clone(),
            parameters: HashMap::new(), // Empty for now - user can edit JSON to add params
            script_template: self.script_text.clone(),
        };

        // Serialize to JSON
        let json = serde_json::to_string_pretty(&component)
            .map_err(|e| format!("Failed to serialize: {}", e))?;

        // Write to file
        let file_path = format!("{}/{}.json", category_dir, self.save_component_name);
        std::fs::write(&file_path, json)
            .map_err(|e| format!("Failed to write file: {}", e))?;

        Ok(file_path)
    }

    fn render_3d_viewport(&mut self, ui: &mut egui::Ui) {
        let (rect, response) = ui.allocate_exact_size(
            ui.available_size(),
            egui::Sense::click_and_drag(),
        );
        self.last_viewport_rect = rect;

        // Handle camera controls
        if response.dragged_by(egui::PointerButton::Primary) {
            if let Some(last_pos) = self.last_mouse_pos {
                if let Some(current_pos) = response.hover_pos() {
                    let delta = current_pos - last_pos;
                    self.camera.orbit(delta.x, -delta.y);
                }
            }
            self.is_dragging_left = true;
        } else {
            self.is_dragging_left = false;
        }

        if response.dragged_by(egui::PointerButton::Secondary) {
            if let Some(last_pos) = self.last_mouse_pos {
                if let Some(current_pos) = response.hover_pos() {
                    let delta = current_pos - last_pos;
                    self.camera.pan(delta.x, -delta.y);
                }
            }
            self.is_dragging_right = true;
        } else {
            self.is_dragging_right = false;
        }

        self.last_mouse_pos = response.hover_pos();

        // Viewport picking for measurement point A/B
        if let Some(pick) = self.pick_mode {
            if response.clicked() && !self.is_dragging_left {
                if let (Some(click_pos), Some(grid)) = (response.interact_pointer_pos(), &self.current_sdf_grid) {
                    let hit = self.unproject_and_march(click_pos, rect, grid.as_ref());
                    if let Some(p) = hit {
                        if pick == 0 { self.point_a = Some(p); }
                        else         { self.point_b = Some(p); }
                        self.pick_mode = None;
                    }
                }
            }
        }

        // Handle zoom
        if response.hovered() {
            let scroll_delta = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll_delta.abs() > 0.01 {
                self.camera.zoom(scroll_delta * 0.1);
            }
        }

        // Update camera aspect ratio
        if rect.width() > 0.0 && rect.height() > 0.0 {
            self.camera.aspect = rect.width() / rect.height();
        }

        // Custom painting callback
        let camera = self.camera.clone();
        let mesh = self.current_mesh.clone();
        let sdf_grid = self.current_sdf_grid.clone();
        let show_wireframe = self.show_wireframe;

        let section   = self.section_view_to_uniforms();
        let thickness = self.thickness_uniforms();

        // Pass thickness grid data if a new upload is pending.
        let thickness_upload: Option<(Arc<Vec<f32>>, u32)> = if self.thickness_needs_upload {
            self.thickness_result.as_ref().map(|r| (Arc::clone(&r.analysis_grid), r.resolution))
        } else {
            None
        };
        if self.thickness_needs_upload { self.thickness_needs_upload = false; }
        let clear_thickness = !self.thickness_overlay_on && self.thickness_result.is_none()
            && self.fea_overlay_mode == FEAOverlayMode::None;

        // FEA overlay upload: stress or displacement data
        let fea_upload: Option<(Arc<Vec<f32>>, u32)> = if self.fea_needs_upload {
            if let Some(ref result) = self.fea_result {
                let data = match self.fea_overlay_mode {
                    FEAOverlayMode::Stress | FEAOverlayMode::None =>
                        Arc::new(result.von_mises.clone()),
                    FEAOverlayMode::Displacement =>
                        Arc::new(result.displacement.clone()),
                };
                Some((data, result.resolution))
            } else { None }
        } else { None };
        if self.fea_needs_upload { self.fea_needs_upload = false; }

        let cb = CustomPaint {
            camera, mesh, sdf_grid, show_wireframe, viewport_rect: rect,
            section, thickness, thickness_upload, fea_upload, clear_thickness,
        };

        ui.painter().add(egui_wgpu::Callback::new_paint_callback(
            rect,
            cb,
        ));

        // Draw FEA BC overlays on top using egui painter (screen-space 2D).
        if self.fea_show_conditions {
            if let Some(ref viz) = self.fea_viz {
                let painter = ui.painter().with_clip_rect(rect);
                Self::draw_fea_overlays(&painter, viz, &self.camera, rect);
            }
        }

        // Draw measurement overlays (picked points, CG marker).
        {
            let painter = ui.painter().with_clip_rect(rect);
            if let Some(p) = self.point_a {
                if let Some(s) = Self::project_to_screen(p, &self.camera, rect) {
                    painter.circle_filled(s, 6.0, egui::Color32::from_rgb(220, 60, 60));
                    painter.circle_stroke(s, 6.0, egui::Stroke::new(1.5, egui::Color32::WHITE));
                    painter.text(s + egui::vec2(8.0, -8.0), egui::Align2::LEFT_BOTTOM,
                        "A", egui::FontId::proportional(12.0), egui::Color32::from_rgb(220,60,60));
                }
            }
            if let Some(p) = self.point_b {
                if let Some(s) = Self::project_to_screen(p, &self.camera, rect) {
                    painter.circle_filled(s, 6.0, egui::Color32::from_rgb(60, 120, 220));
                    painter.circle_stroke(s, 6.0, egui::Stroke::new(1.5, egui::Color32::WHITE));
                    painter.text(s + egui::vec2(8.0, -8.0), egui::Align2::LEFT_BOTTOM,
                        "B", egui::FontId::proportional(12.0), egui::Color32::from_rgb(60,120,220));
                }
            }
            if self.show_cg_marker {
                if let Some(cg) = self.measurements.center_of_mass {
                    Self::draw_cg_crosshair(&painter, cg, &self.camera, rect);
                }
                // Also draw script-defined CG if available.
                if let Some(cg) = self.cg {
                    if self.measurements.center_of_mass != Some(cg) {
                        Self::draw_cg_crosshair_color(&painter, cg, egui::Color32::from_rgb(255,200,0), &self.camera, rect);
                    }
                }
            }
        }
    }

    /// Project a world-space point onto the egui viewport rect.
    /// Returns `None` if the point is behind the camera.
    fn project_to_screen(p: Vec3, camera: &Camera, rect: egui::Rect) -> Option<egui::Pos2> {
        let vp = camera.view_projection();
        let clip = vp * glam::Vec4::new(p.x, p.y, p.z, 1.0);
        if clip.w <= 0.0 { return None; }
        let ndc_x =  clip.x / clip.w;
        let ndc_y = -clip.y / clip.w;  // flip Y: egui Y goes down
        if ndc_x.abs() > 1.5 || ndc_y.abs() > 1.5 { return None; }
        Some(egui::pos2(
            rect.left() + (ndc_x + 1.0) * 0.5 * rect.width(),
            rect.top()  + (ndc_y + 1.0) * 0.5 * rect.height(),
        ))
    }

    /// Draw a 3-D arrow as a 2-D screen-space line + arrowhead.
    fn draw_arrow_3d(
        painter:  &egui::Painter,
        origin:   Vec3,
        dir:      Vec3,      // normalized
        length:   f32,
        color:    egui::Color32,
        camera:   &Camera,
        rect:     egui::Rect,
    ) {
        let tip = origin + dir * length;
        let p0 = Self::project_to_screen(origin, camera, rect);
        let p1 = Self::project_to_screen(tip, camera, rect);
        if let (Some(a), Some(b)) = (p0, p1) {
            painter.line_segment([a, b], egui::Stroke::new(2.5, color));
            // Arrowhead: small filled triangle at tip
            let shaft = (a - b).normalized();
            let perp  = egui::vec2(-shaft.y, shaft.x) * 5.0;
            let head  = [b, b + shaft * 12.0 + perp, b + shaft * 12.0 - perp];
            painter.add(egui::Shape::convex_polygon(
                head.to_vec(), color, egui::Stroke::NONE,
            ));
        }
    }

    /// Draw all FEA BC overlays.
    fn draw_fea_overlays(
        painter: &egui::Painter,
        viz:     &crate::fea::FEAVizData,
        camera:  &Camera,
        rect:    egui::Rect,
    ) {
        // Arrow scale relative to viewport size
        let arrow_len = rect.size().min_elem() * 0.12;

        // Fixed supports — blue dots
        let blue = egui::Color32::from_rgba_unmultiplied(60, 120, 255, 200);
        for region in &viz.fixed_supports {
            for &p in &region.points {
                if let Some(sp) = Self::project_to_screen(p, camera, rect) {
                    painter.circle_filled(sp, 3.0, blue);
                }
            }
        }

        // Force loads — yellow arrows from centroid
        let yellow = egui::Color32::from_rgba_unmultiplied(255, 220, 40, 230);
        for ((region, dir), _mag) in viz.force_loads.iter().zip(viz.force_magnitudes.iter()) {
            Self::draw_arrow_3d(painter, region.centroid, *dir, arrow_len, yellow, camera, rect);
        }

        // Pressure loads — red dots
        let red = egui::Color32::from_rgba_unmultiplied(220, 60, 60, 180);
        for region in &viz.pressure_loads {
            for &p in &region.points {
                if let Some(sp) = Self::project_to_screen(p, camera, rect) {
                    painter.circle_filled(sp, 3.0, red);
                }
            }
        }

        // Torque loads — cyan arc approximation (two opposing arrows around centroid)
        let cyan = egui::Color32::from_rgba_unmultiplied(40, 200, 200, 220);
        for (region, axis) in &viz.torque_loads {
            // Draw two tangential arrows indicating rotation direction
            let perp = axis.any_orthonormal_vector();
            let r = arrow_len * 0.5;
            Self::draw_arrow_3d(painter, region.centroid + perp * r,
                axis.cross(perp).normalize_or_zero(), arrow_len * 0.6, cyan, camera, rect);
            Self::draw_arrow_3d(painter, region.centroid - perp * r,
                -axis.cross(perp).normalize_or_zero(), arrow_len * 0.6, cyan, camera, rect);
        }

        // Motor thrusts — green arrows
        let green = egui::Color32::from_rgba_unmultiplied(60, 200, 80, 230);
        for (region, dir) in &viz.motor_thrusts {
            Self::draw_arrow_3d(painter, region.centroid, *dir, arrow_len, green, camera, rect);
        }

        // Gravity — large grey downward arrow in bottom-left corner of viewport
        if let Some(grav_dir) = viz.gravity {
            let grey = egui::Color32::from_rgba_unmultiplied(160, 160, 160, 200);
            let corner = rect.left_bottom() + egui::vec2(40.0, -40.0);
            let shaft_px = arrow_len;
            let end = corner + egui::vec2(grav_dir.x, -grav_dir.z) * shaft_px;
            painter.line_segment([corner, end], egui::Stroke::new(3.0, grey));
            let dir2d = (end - corner).normalized();
            let perp  = egui::vec2(-dir2d.y, dir2d.x) * 6.0;
            painter.add(egui::Shape::convex_polygon(
                vec![end, end - dir2d * 14.0 + perp, end - dir2d * 14.0 - perp],
                grey, egui::Stroke::NONE,
            ));
            painter.text(
                corner + egui::vec2(8.0, -10.0),
                egui::Align2::LEFT_BOTTOM,
                "g",
                egui::FontId::proportional(11.0),
                grey,
            );
        }
    }

    // ── Measurement helpers ───────────────────────────────────────────────────

    fn unproject_and_march(
        &self,
        screen_pos: egui::Pos2,
        rect:       egui::Rect,
        grid:       &crate::render::SdfGrid,
    ) -> Option<Vec3> {
        // Convert screen pos to NDC [-1, 1].
        let nx =  (screen_pos.x - rect.left()) / rect.width()  * 2.0 - 1.0;
        let ny = -((screen_pos.y - rect.top())  / rect.height() * 2.0 - 1.0);

        // Unproject near and far planes through the inverse VP matrix.
        let inv_vp = self.camera.view_projection().inverse();
        let near4 = inv_vp * glam::Vec4::new(nx, ny, -1.0, 1.0);
        let far4  = inv_vp * glam::Vec4::new(nx, ny,  1.0, 1.0);
        let near_w = Vec3::new(near4.x / near4.w, near4.y / near4.w, near4.z / near4.w);
        let far_w  = Vec3::new(far4.x  / far4.w,  far4.y  / far4.w,  far4.z  / far4.w);
        let dir = (far_w - near_w).normalize_or_zero();
        if dir.length_squared() < 0.0001 { return None; }
        let max_dist = (far_w - near_w).length();
        crate::analysis::ray_march_grid(grid, near_w, dir, max_dist)
    }

    fn draw_cg_crosshair(
        painter: &egui::Painter,
        cg:      Vec3,
        camera:  &Camera,
        rect:    egui::Rect,
    ) {
        Self::draw_cg_crosshair_color(painter, cg, egui::Color32::from_rgb(100, 220, 255), camera, rect);
    }

    fn draw_cg_crosshair_color(
        painter: &egui::Painter,
        cg:      Vec3,
        color:   egui::Color32,
        camera:  &Camera,
        rect:    egui::Rect,
    ) {
        let arm = 6.0f32;
        let axes = [
            (Vec3::X * arm, Vec3::NEG_X * arm),
            (Vec3::Y * arm, Vec3::NEG_Y * arm),
            (Vec3::Z * arm, Vec3::NEG_Z * arm),
        ];
        for (a, b) in axes {
            if let (Some(pa), Some(pb)) = (
                Self::project_to_screen(cg + a, camera, rect),
                Self::project_to_screen(cg + b, camera, rect),
            ) {
                painter.line_segment([pa, pb], egui::Stroke::new(2.0, color));
            }
        }
        if let Some(center) = Self::project_to_screen(cg, camera, rect) {
            painter.circle_filled(center, 4.0, color);
        }
    }

    // ── Measurement popup ─────────────────────────────────────────────────────

    fn show_measure_popup(&mut self, ctx: &egui::Context) {
        if !self.measure_open { return; }

        let mut open = true;
        egui::Window::new("📐 Measurements")
            .open(&mut open)
            .resizable(true)
            .default_width(340.0)
            .show(ctx, |ui| {
                // Stale warning
                if self.measurements_stale {
                    egui::Frame::none()
                        .fill(egui::Color32::from_rgba_unmultiplied(180, 150, 0, 60))
                        .inner_margin(egui::Margin::symmetric(8.0, 4.0))
                        .show(ui, |ui| {
                            ui.label(egui::RichText::new(
                                "⚠ Results may be outdated — re-run analysis after the model changes."
                            ).color(egui::Color32::from_rgb(255, 220, 0)).small());
                        });
                    ui.add_space(4.0);
                }

                // ── Section 1: Model Properties ──────────────────────────────
                ui.strong("Model Properties");
                ui.horizontal(|ui| {
                    let btn_label = if self.measure_running { "⏳ Running…" } else { "Run Analysis" };
                    if ui.button(btn_label).clicked() && !self.measure_running {
                        if let Some(ref grid) = self.current_sdf_grid {
                            let grid = grid.as_ref().clone_data();
                            let (tx, rx) = std::sync::mpsc::channel();
                            self.measure_receiver = Some(rx);
                            self.measure_running  = true;
                            std::thread::spawn(move || {
                                let (v, s, c) = crate::analysis::compute_model_properties(&grid);
                                let _ = tx.send((v, s, c));
                            });
                        }
                    }
                    if self.measure_running { ui.spinner(); }
                });

                if let Some(v) = self.measurements.volume_mm3 {
                    egui::Grid::new("model_props").num_columns(2).show(ui, |ui| {
                        ui.label("Volume:");
                        ui.label(format!("{:.3} cm³", v / 1000.0));
                        ui.end_row();
                        if let Some(sa) = self.measurements.surface_area_mm2 {
                            ui.label("Surface Area:");
                            ui.label(format!("{:.2} cm²", sa / 100.0));
                            ui.end_row();
                        }
                        if let Some(com) = self.measurements.center_of_mass {
                            ui.label("Center of Mass:");
                            ui.label(format!("({:.1}, {:.1}, {:.1}) mm", com.x, com.y, com.z));
                            ui.end_row();
                        }
                        if let Some(mass) = self.measurements.print_mass_g {
                            ui.label("Print Mass:");
                            ui.label(format!("{:.1} g", mass));
                            ui.end_row();
                        }
                    });

                    // Density input + presets
                    ui.horizontal(|ui| {
                        ui.label("Density:");
                        let changed = ui.add(
                            egui::DragValue::new(&mut self.measure_density)
                                .speed(0.01)
                                .range(0.1..=20.0)
                                .suffix(" g/cm³"),
                        ).changed();
                        egui::ComboBox::from_id_salt("density_preset")
                            .selected_text("Preset")
                            .show_ui(ui, |ui| {
                                if ui.selectable_label(false, "PLA 1.24").clicked()  { self.measure_density = 1.24; }
                                if ui.selectable_label(false, "PETG 1.27").clicked() { self.measure_density = 1.27; }
                                if ui.selectable_label(false, "ABS 1.05").clicked()  { self.measure_density = 1.05; }
                                if ui.selectable_label(false, "Resin 1.10").clicked(){ self.measure_density = 1.10; }
                            });
                        if changed {
                            if let Some(v) = self.measurements.volume_mm3 {
                                self.measurements.print_mass_g = Some(v / 1000.0 * self.measure_density);
                            }
                        }
                    });
                }

                ui.separator();

                // ── Section 2: Cross-Section Area ─────────────────────────────
                ui.strong("Cross-Section Area");
                ui.horizontal(|ui| {
                    ui.label("Axis:");
                    ui.radio_value(&mut self.cs_axis, crate::project::Axis::X, "X");
                    ui.radio_value(&mut self.cs_axis, crate::project::Axis::Y, "Y");
                    ui.radio_value(&mut self.cs_axis, crate::project::Axis::Z, "Z");
                });
                if let Some(ref grid) = self.current_sdf_grid {
                    let (lo, hi) = match self.cs_axis {
                        crate::project::Axis::X => (grid.bounds_min.x, grid.bounds_max.x),
                        crate::project::Axis::Y => (grid.bounds_min.y, grid.bounds_max.y),
                        crate::project::Axis::Z => (grid.bounds_min.z, grid.bounds_max.z),
                    };
                    self.cs_position = self.cs_position.clamp(lo, hi);
                    ui.add(egui::Slider::new(&mut self.cs_position, lo..=hi).text("Position"));
                }
                ui.horizontal(|ui| {
                    let label = if self.cs_running { "⏳ Running…" } else { "Measure" };
                    if ui.button(label).clicked() && !self.cs_running {
                        if let (Some(sdf), Some(grid)) = (&self.current_sdf, &self.current_sdf_grid) {
                            let sdf   = sdf.clone();
                            let axis  = self.cs_axis.clone();
                            let pos   = self.cs_position;
                            let bmin  = grid.bounds_min;
                            let bmax  = grid.bounds_max;
                            let (tx, rx) = std::sync::mpsc::channel();
                            self.cs_receiver = Some(rx);
                            self.cs_running  = true;
                            std::thread::spawn(move || {
                                let area = crate::analysis::measure_cross_section(
                                    sdf, &axis, pos, bmin, bmax, 256,
                                );
                                let _ = tx.send(area);
                            });
                        }
                    }
                    if self.cs_running { ui.spinner(); }
                });
                egui::ScrollArea::vertical().max_height(120.0).id_salt("cs_scroll").show(ui, |ui| {
                    let mut to_remove = None;
                    for (i, cs) in self.measurements.cross_sections.iter().enumerate() {
                        ui.horizontal(|ui| {
                            if ui.small_button("✕").clicked() { to_remove = Some(i); }
                            ui.label(format!("{} → {:.2} mm²", cs.label, cs.area_mm2));
                        });
                    }
                    if let Some(i) = to_remove { self.measurements.cross_sections.remove(i); }
                });
                if !self.measurements.cross_sections.is_empty() {
                    if ui.small_button("Clear All").clicked() {
                        self.measurements.cross_sections.clear();
                    }
                }

                ui.separator();

                // ── Section 3: Point Distance ─────────────────────────────────
                ui.strong("Point Distance");

                // Point A / B pickers
                let sdf_for_snap = self.current_sdf.clone();
                for (lbl, pt_field, pick_idx, color) in [
                    ("A", 0usize, 0u8, egui::Color32::from_rgb(220, 60, 60)),
                    ("B", 1usize, 1u8, egui::Color32::from_rgb(60, 120, 220)),
                ] {
                    ui.horizontal(|ui| {
                        ui.colored_label(color, lbl);

                        let pt_ref = if pt_field == 0 { &mut self.point_a } else { &mut self.point_b };
                        let mut xyz = pt_ref.unwrap_or(Vec3::ZERO);
                        let mut changed = false;
                        changed |= ui.add(egui::DragValue::new(&mut xyz.x).speed(0.1).prefix("x ").max_decimals(2)).changed();
                        changed |= ui.add(egui::DragValue::new(&mut xyz.y).speed(0.1).prefix("y ").max_decimals(2)).changed();
                        changed |= ui.add(egui::DragValue::new(&mut xyz.z).speed(0.1).prefix("z ").max_decimals(2)).changed();
                        if changed { *pt_ref = Some(xyz); }

                        // Pick from viewport
                        let picking = self.pick_mode == Some(pick_idx);
                        let pick_btn = if picking {
                            egui::Button::new("Picking…").fill(egui::Color32::from_rgb(40, 100, 40))
                        } else {
                            egui::Button::new("Pick")
                        };
                        if ui.add(pick_btn).on_hover_text("Click in the 3D viewport to place this point").clicked() {
                            self.pick_mode = if picking { None } else { Some(pick_idx) };
                        }

                        // Snap to nearest surface
                        let snap_enabled = pt_ref.is_some() && sdf_for_snap.is_some();
                        if ui.add_enabled(snap_enabled, egui::Button::new("→ Surface"))
                            .on_hover_text("Move point to nearest point on the SDF surface")
                            .clicked()
                        {
                            if let (Some(p), Some(sdf)) = (*pt_ref, &sdf_for_snap) {
                                *pt_ref = Some(crate::analysis::snap_to_surface(sdf.as_ref(), p, 64));
                            }
                        }
                    });
                }

                ui.add_space(4.0);

                // Measurement type selector
                ui.horizontal(|ui| {
                    ui.label("Type:");
                    egui::ComboBox::from_id_salt("dist_kind")
                        .selected_text(self.dist_kind.label())
                        .width(160.0)
                        .show_ui(ui, |ui| {
                            for k in crate::analysis::DistanceMeasureKind::all() {
                                ui.selectable_value(&mut self.dist_kind, k.clone(), k.label());
                            }
                        });
                });

                // Custom vector input
                if self.dist_kind == crate::analysis::DistanceMeasureKind::CustomVector {
                    ui.horizontal(|ui| {
                        ui.label("Vector:");
                        ui.add(egui::DragValue::new(&mut self.custom_proj_vec.x).speed(0.01).prefix("x ").max_decimals(3));
                        ui.add(egui::DragValue::new(&mut self.custom_proj_vec.y).speed(0.01).prefix("y ").max_decimals(3));
                        ui.add(egui::DragValue::new(&mut self.custom_proj_vec.z).speed(0.01).prefix("z ").max_decimals(3));
                        if ui.small_button("Normalize").clicked() {
                            let n = self.custom_proj_vec.normalize_or(Vec3::X);
                            self.custom_proj_vec = n;
                        }
                    });
                }

                // Live results
                if let (Some(a), Some(b)) = (self.point_a, self.point_b) {
                    let custom = if self.dist_kind == crate::analysis::DistanceMeasureKind::CustomVector {
                        Some(self.custom_proj_vec)
                    } else { None };
                    let sdf_ref = self.current_sdf.as_ref().map(|s| s.as_ref());
                    let m = crate::analysis::PointDistanceMeasurement::compute(
                        String::new(), a, b, self.dist_kind.clone(), custom, sdf_ref,
                    );

                    // Always show full breakdown
                    egui::Frame::none()
                        .fill(egui::Color32::from_rgba_unmultiplied(0, 0, 0, 60))
                        .inner_margin(egui::Margin::symmetric(8.0, 6.0))
                        .show(ui, |ui| {
                            egui::Grid::new("dist_results").num_columns(2).spacing([20.0, 2.0]).show(ui, |ui| {
                                ui.label("3D Distance:");
                                ui.label(format!("{:.4} mm", m.distance_3d));
                                ui.end_row();
                                ui.label("ΔX:");
                                ui.label(format!("{:.4} mm", m.delta.x));
                                ui.end_row();
                                ui.label("ΔY:");
                                ui.label(format!("{:.4} mm", m.delta.y));
                                ui.end_row();
                                ui.label("ΔZ:");
                                ui.label(format!("{:.4} mm", m.delta.z));
                                ui.end_row();

                                // Primary derived value
                                match &self.dist_kind {
                                    crate::analysis::DistanceMeasureKind::Distance3D => {}
                                    crate::analysis::DistanceMeasureKind::Angle => {
                                        if let Some(angs) = m.angles_deg {
                                            ui.label("Angle from X:");
                                            ui.label(format!("{:.3}°", angs[0]));
                                            ui.end_row();
                                            ui.label("Angle from Y:");
                                            ui.label(format!("{:.3}°", angs[1]));
                                            ui.end_row();
                                            ui.label("Angle from Z:");
                                            ui.label(format!("{:.3}°", angs[2]));
                                            ui.end_row();
                                        }
                                    }
                                    k => {
                                        ui.label(egui::RichText::new(format!("{}:", k.label())).strong());
                                        ui.label(egui::RichText::new(format!("{:.4} mm", m.primary_mm)).strong());
                                        ui.end_row();
                                    }
                                }

                                // Midpoint
                                let mid = (a + b) * 0.5;
                                ui.label("Midpoint:");
                                ui.label(format!("({:.2}, {:.2}, {:.2})", mid.x, mid.y, mid.z));
                                ui.end_row();
                            });
                        });

                    ui.horizontal(|ui| {
                        if ui.button("Save").clicked() {
                            let label = format!("D{}", self.distance_label_idx);
                            self.distance_label_idx += 1;
                            let custom2 = if self.dist_kind == crate::analysis::DistanceMeasureKind::CustomVector {
                                Some(self.custom_proj_vec)
                            } else { None };
                            let sdf_ref2 = self.current_sdf.as_ref().map(|s| s.as_ref());
                            self.measurements.point_distances.push(
                                crate::analysis::PointDistanceMeasurement::compute(
                                    label, a, b, self.dist_kind.clone(), custom2, sdf_ref2,
                                )
                            );
                        }
                        if ui.button("Swap A↔B").clicked() {
                            std::mem::swap(&mut self.point_a, &mut self.point_b);
                        }
                        if ui.button("Clear").clicked() {
                            self.point_a = None;
                            self.point_b = None;
                            self.pick_mode = None;
                        }
                    });
                }

                // Saved measurements list
                if !self.measurements.point_distances.is_empty() {
                    ui.add_space(4.0);
                    egui::ScrollArea::vertical().max_height(120.0).id_salt("dist_scroll").show(ui, |ui| {
                        let mut to_remove = None;
                        for (i, pd) in self.measurements.point_distances.iter().enumerate() {
                            ui.horizontal(|ui| {
                                if ui.small_button("✕").clicked() { to_remove = Some(i); }
                                let summary = match &pd.kind {
                                    crate::analysis::DistanceMeasureKind::Angle =>
                                        if let Some(a) = pd.angles_deg {
                                            format!("{} — ∠X:{:.1}° ∠Y:{:.1}° ∠Z:{:.1}°", pd.label, a[0], a[1], a[2])
                                        } else { format!("{} — {:.3} mm", pd.label, pd.primary_mm) },
                                    k => {
                                        if *k == crate::analysis::DistanceMeasureKind::Distance3D {
                                            format!("{} — {:.4} mm", pd.label, pd.distance_3d)
                                        } else {
                                            format!("{} — 3D:{:.3}  {}:{:.3} mm", pd.label, pd.distance_3d, k.label(), pd.primary_mm)
                                        }
                                    }
                                };
                                ui.label(summary).on_hover_text(
                                    format!("A({:.2},{:.2},{:.2})  B({:.2},{:.2},{:.2})\nΔ({:.3},{:.3},{:.3})",
                                        pd.point_a.x, pd.point_a.y, pd.point_a.z,
                                        pd.point_b.x, pd.point_b.y, pd.point_b.z,
                                        pd.delta.x, pd.delta.y, pd.delta.z)
                                );
                            });
                        }
                        if let Some(i) = to_remove { self.measurements.point_distances.remove(i); }
                    });
                    if ui.small_button("Clear All").clicked() {
                        self.measurements.point_distances.clear();
                    }
                }

                ui.separator();

                // ── Section 4: CG Visualization ───────────────────────────────
                ui.strong("CG Visualization");
                ui.checkbox(&mut self.show_cg_marker, "Show Center of Mass in viewport");
                if self.show_cg_marker {
                    if let Some(cg) = self.measurements.center_of_mass {
                        ui.label(egui::RichText::new(
                            format!("Geometric CG: ({:.1}, {:.1}, {:.1}) mm", cg.x, cg.y, cg.z)
                        ).color(egui::Color32::from_rgb(100, 220, 255)).small());
                    }
                    if let Some(cg) = self.cg {
                        ui.label(egui::RichText::new(
                            format!("Script CG: ({:.1}, {:.1}, {:.1}) mm", cg.x, cg.y, cg.z)
                        ).color(egui::Color32::from_rgb(255, 200, 0)).small());
                    }
                }

                ui.separator();

                // ── Section 5: Export ─────────────────────────────────────────
                ui.strong("Export");
                if ui.button("Copy to Clipboard").clicked() {
                    let text = self.format_measurements_text();
                    ctx.copy_text(text);
                }
            });

        self.measure_open = open;
    }

    fn format_measurements_text(&self) -> String {
        let mut s = String::from("=== Measurements ===\n");
        if let Some(v) = self.measurements.volume_mm3 {
            s += &format!("Volume: {:.3} cm³\n", v / 1000.0);
        }
        if let Some(sa) = self.measurements.surface_area_mm2 {
            s += &format!("Surface Area: {:.2} cm²\n", sa / 100.0);
        }
        if let Some(com) = self.measurements.center_of_mass {
            s += &format!("Center of Mass: ({:.1}, {:.1}, {:.1}) mm\n", com.x, com.y, com.z);
        }
        if let Some(mass) = self.measurements.print_mass_g {
            s += &format!("Print Mass: {:.1} g (density {:.2} g/cm³)\n", mass, self.measure_density);
        }
        if !self.measurements.cross_sections.is_empty() {
            s += "\n--- Cross-Sections ---\n";
            for cs in &self.measurements.cross_sections {
                s += &format!("{}: {:.2} mm²\n", cs.label, cs.area_mm2);
            }
        }
        if !self.measurements.point_distances.is_empty() {
            s += "\n--- Point Distances ---\n";
            for pd in &self.measurements.point_distances {
                s += &format!("{}: {:.3} mm  A({:.1},{:.1},{:.1})  B({:.1},{:.1},{:.1})\n",
                    pd.label, pd.primary_mm,
                    pd.point_a.x, pd.point_a.y, pd.point_a.z,
                    pd.point_b.x, pd.point_b.y, pd.point_b.z);
            }
        }
        s
    }
}

struct CustomPaint {
    camera:           Camera,
    mesh:             Option<Mesh>,
    sdf_grid:         Option<Arc<SdfGrid>>,
    show_wireframe:   bool,
    viewport_rect:    egui::Rect,
    section:          SectionUniforms,
    thickness:        ThicknessUniforms,
    /// Some(data, resolution) when a new thickness texture must be uploaded this frame.
    thickness_upload: Option<(Arc<Vec<f32>>, u32)>,
    /// Some(data, resolution) when FEA stress/displacement data must be uploaded this frame.
    fea_upload:       Option<(Arc<Vec<f32>>, u32)>,
    /// True when the thickness overlay was just cleared and the dummy texture should be rebound.
    clear_thickness:  bool,
}

impl egui_wgpu::CallbackTrait for CustomPaint {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let fmt = wgpu::TextureFormat::Bgra8Unorm;

        if callback_resources.get::<RenderState>().is_none() {
            callback_resources.insert(RenderState::new(device, fmt));
        }
        if callback_resources.get::<GridRenderer>().is_none() {
            callback_resources.insert(GridRenderer::new(device, fmt));
        }
        if callback_resources.get::<AxesRenderer>().is_none() {
            callback_resources.insert(AxesRenderer::new(device, fmt));
        }
        if callback_resources.get::<WireframeRenderer>().is_none() {
            callback_resources.insert(WireframeRenderer::new(device, fmt));
        }
        if callback_resources.get::<RaymarchRenderer>().is_none() {
            callback_resources.insert(RaymarchRenderer::new(device, fmt));
        }

        // Mesh render state (kept for export / wireframe overlay)
        let render_state: &mut RenderState = callback_resources.get_mut().unwrap();
        if let Some(ref mesh) = self.mesh {
            render_state.upload_mesh(device, mesh);
        }
        render_state.update_uniforms(queue, &self.camera);

        let grid_renderer: &GridRenderer = callback_resources.get().unwrap();
        grid_renderer.update_uniforms(queue, &self.camera);

        let axes_renderer: &AxesRenderer = callback_resources.get().unwrap();
        axes_renderer.update_uniforms(queue, &self.camera);

        let wireframe_renderer: &mut WireframeRenderer = callback_resources.get_mut().unwrap();
        if let Some(ref mesh) = self.mesh {
            wireframe_renderer.upload_mesh(device, mesh);
        }
        wireframe_renderer.update_uniforms(queue, &self.camera);

        // Sphere-tracing renderer
        let ppp = screen_descriptor.pixels_per_point;
        let vp_offset = [self.viewport_rect.min.x * ppp, self.viewport_rect.min.y * ppp];
        let vp_size   = [self.viewport_rect.width() * ppp, self.viewport_rect.height() * ppp];

        let rm: &mut RaymarchRenderer = callback_resources.get_mut().unwrap();
        if let Some(ref grid) = self.sdf_grid {
            let needs_upload = rm.last_grid.as_ref()
                .map(|last| !Arc::ptr_eq(last, grid))
                .unwrap_or(true);
            if needs_upload {
                rm.upload_grid(device, queue, grid);
            }
        }
        // FEA upload takes priority over thickness upload for the overlay texture
        if let Some((ref data, resolution)) = self.fea_upload {
            rm.upload_thickness(device, queue, data, resolution);
        } else if let Some((ref data, resolution)) = self.thickness_upload {
            rm.upload_thickness(device, queue, data, resolution);
        }
        if self.clear_thickness {
            rm.clear_thickness(device);
        }
        rm.update_uniforms(queue, &self.camera, vp_offset, vp_size, &self.section, &self.thickness);

        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &egui_wgpu::CallbackResources,
    ) {
        let grid_renderer:      &GridRenderer      = callback_resources.get().unwrap();
        let axes_renderer:      &AxesRenderer      = callback_resources.get().unwrap();
        let wireframe_renderer: &WireframeRenderer = callback_resources.get().unwrap();
        let raymarch_renderer:  &RaymarchRenderer  = callback_resources.get().unwrap();

        // Grid first (background)
        grid_renderer.render(render_pass);

        // Sphere-traced shape (replaces mesh triangles in viewport)
        if self.sdf_grid.is_some() {
            raymarch_renderer.render(render_pass);
        } else {
            // Fallback: rasterised mesh if no SDF grid available
            let render_state: &RenderState = callback_resources.get().unwrap();
            render_pass.set_pipeline(&render_state.render_pipeline);
            render_pass.set_bind_group(0, &render_state.uniform_bind_group, &[]);
            if let (Some(vb), Some(ib)) = (&render_state.vertex_buffer, &render_state.index_buffer) {
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..render_state.num_indices, 0, 0..1);
            }
        }

        // Wireframe overlay if enabled
        if self.show_wireframe {
            wireframe_renderer.render(render_pass);
        }

        // Render axes indicator last (on top)
        axes_renderer.render(render_pass);
    }
}

impl App {
    /// Convert the app-level `SectionView` into the flat `SectionUniforms` needed by the shader.
    fn section_view_to_uniforms(&self) -> SectionUniforms {
        let sv = &self.section_view;
        let a = sv.plane_a.as_ref();
        let b = sv.plane_b.as_ref();
        SectionUniforms {
            plane_a_enabled:  a.map(|p| p.enabled).unwrap_or(false),
            plane_a_axis:     a.map(|p| p.axis.to_index()).unwrap_or(0),
            plane_a_position: a.map(|p| p.position).unwrap_or(0.0),
            plane_a_flip:     a.map(|p| p.flip).unwrap_or(false),
            plane_b_enabled:  b.map(|p| p.enabled).unwrap_or(false),
            plane_b_axis:     b.map(|p| p.axis.to_index()).unwrap_or(0),
            plane_b_position: b.map(|p| p.position).unwrap_or(0.0),
            plane_b_flip:     b.map(|p| p.flip).unwrap_or(false),
        }
    }

    fn thickness_uniforms(&self) -> ThicknessUniforms {
        // FEA overlay takes priority over thickness overlay.
        if self.fea_overlay_mode != FEAOverlayMode::None && self.fea_result.is_some() {
            let (max_val, invert) = match self.fea_overlay_mode {
                FEAOverlayMode::Stress => (
                    self.fea_result.as_ref().map(|r| r.max_von_mises).unwrap_or(1.0).max(0.001),
                    true,  // red = high stress, green = safe
                ),
                FEAOverlayMode::Displacement => (
                    self.fea_result.as_ref().map(|r| r.max_displacement).unwrap_or(1.0).max(0.001),
                    false, // green = zero displacement, red = max
                ),
                FEAOverlayMode::None => unreachable!(),
            };
            return ThicknessUniforms { enabled: true, min_display: 0.0, max_display: max_val, invert };
        }
        ThicknessUniforms {
            enabled:     self.thickness_overlay_on && self.thickness_result.is_some(),
            min_display: 0.0,
            max_display: self.thickness_max_display,
            invert:      false,
        }
    }

    fn render_section_view_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Section View");
        ui.separator();

        // Pre-extract bounds so we can use them while holding mutable refs to planes.
        let (xlo, xhi, ylo, yhi, zlo, zhi) = if let Some(ref grid) = self.current_sdf_grid {
            (grid.bounds_min.x, grid.bounds_max.x,
             grid.bounds_min.y, grid.bounds_max.y,
             grid.bounds_min.z, grid.bounds_max.z)
        } else {
            (-100.0, 100.0, -100.0, 100.0, -100.0, 100.0)
        };
        let range_for = |axis: &Axis| -> (f32, f32) {
            match axis {
                Axis::X => (xlo, xhi),
                Axis::Y => (ylo, yhi),
                Axis::Z => (zlo, zhi),
            }
        };

        // Plane A (always present — initialized in Default)
        if let Some(ref mut plane_a) = self.section_view.plane_a {
            ui.checkbox(&mut plane_a.enabled, "Plane A");
            if plane_a.enabled {
                ui.horizontal(|ui| {
                    ui.label("Axis:");
                    ui.selectable_value(&mut plane_a.axis, Axis::X, "X");
                    ui.selectable_value(&mut plane_a.axis, Axis::Y, "Y");
                    ui.selectable_value(&mut plane_a.axis, Axis::Z, "Z");
                });
                let (lo, hi) = range_for(&plane_a.axis);
                ui.add(egui::Slider::new(&mut plane_a.position, lo..=hi).text("Pos"));
                ui.checkbox(&mut plane_a.flip, "Flip direction");
            }
        }

        ui.separator();

        let has_b = self.section_view.plane_b.is_some();
        if has_b {
            if let Some(ref mut plane_b) = self.section_view.plane_b {
                ui.checkbox(&mut plane_b.enabled, "Plane B");
                if plane_b.enabled {
                    ui.horizontal(|ui| {
                        ui.label("Axis:");
                        ui.selectable_value(&mut plane_b.axis, Axis::X, "X");
                        ui.selectable_value(&mut plane_b.axis, Axis::Y, "Y");
                        ui.selectable_value(&mut plane_b.axis, Axis::Z, "Z");
                    });
                    let (lo, hi) = range_for(&plane_b.axis);
                    ui.add(egui::Slider::new(&mut plane_b.position, lo..=hi).text("Pos"));
                    ui.checkbox(&mut plane_b.flip, "Flip direction");
                }
            }
            if ui.button("Remove Plane B").clicked() {
                self.section_view.plane_b = None;
            }
        } else if ui.button("Add Plane B").clicked() {
            self.section_view.plane_b = Some(SectionPlane {
                axis: Axis::Y,
                position: 0.0,
                enabled: false,
                flip: false,
            });
        }

    }

    fn render_thickness_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Wall Thickness");
        ui.separator();

        ui.add(egui::Slider::new(&mut self.thickness_max_display, 0.5..=100.0)
            .text("Max display (mm)"));

        ui.add_space(4.0);

        let has_grid = self.current_sdf_grid.is_some();
        let running  = self.thickness_running;

        ui.horizontal(|ui| {
            let run_btn = ui.add_enabled(has_grid && !running, egui::Button::new("Run"));
            if run_btn.clicked() {
                self.start_thickness_analysis();
            }

            let clear_btn = ui.add_enabled(self.thickness_result.is_some(), egui::Button::new("Clear"));
            if clear_btn.clicked() {
                self.thickness_result       = None;
                self.thickness_overlay_on   = false;
                self.thickness_needs_upload = false;
            }
        });

        if running {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("Analysing...");
            });
        }

        if let Some(ref result) = self.thickness_result {
            ui.add_space(4.0);
            ui.checkbox(&mut self.thickness_overlay_on, "Show overlay");

            ui.add_space(4.0);
            let min = result.min_thickness;
            let warn = min < self.thickness_max_display * 0.25;
            if warn {
                ui.colored_label(egui::Color32::from_rgb(220, 60, 60),
                    format!("⚠ Min: {:.2} mm", min));
            } else {
                ui.colored_label(egui::Color32::from_rgb(60, 180, 60),
                    format!("✓ Min: {:.2} mm", min));
            }
            let loc = result.min_location;
            ui.label(format!("  at ({:.1}, {:.1}, {:.1})", loc.x, loc.y, loc.z));
        } else if !running {
            ui.label("No analysis data.");
        }
    }

    fn start_thickness_analysis(&mut self) {
        let Some(ref grid) = self.current_sdf_grid else { return };
        let grid_clone     = Arc::clone(grid);
        let max_display    = self.thickness_max_display;

        let (tx, rx) = std::sync::mpsc::channel::<ThicknessResult>();
        self.thickness_receiver = Some(rx);
        self.thickness_running  = true;

        std::thread::spawn(move || {
            let result = compute_thickness(&grid_clone, max_display);
            let _ = tx.send(result);
        });
    }

    fn start_fea_analysis(&mut self) {
        let Some(ref sdf) = self.current_sdf else {
            self.error_message = Some("Run the script first to generate geometry.".into());
            return;
        };
        let Some(ref grid) = self.current_sdf_grid else { return };

        self.fea_log.clear();
        self.fea_running = true;

        let pipeline = crate::fea::FEAPipeline {
            sdf:          Arc::clone(sdf),
            bounds_min:   grid.bounds_min,
            bounds_max:   grid.bounds_max,
            setup:        std::mem::take(&mut self.fea_setup),
            config:       self.fea_config.clone(),
            ccx_override: self.settings.ccx_path.clone(),
        };

        let (tx, rx) = std::sync::mpsc::channel::<crate::fea::FEAMessage>();
        self.fea_receiver = Some(rx);

        std::thread::spawn(move || {
            pipeline.run(tx);
        });
    }

    fn render_fea_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("FEA");
        ui.separator();

        // BC overlay toggle
        let has_viz = self.fea_viz.is_some();
        ui.add_enabled_ui(has_viz, |ui| {
            ui.checkbox(&mut self.fea_show_conditions, "Show FEA Conditions");
        });
        if !has_viz { self.fea_show_conditions = false; }

        ui.separator();

        // Material preset
        ui.horizontal(|ui| {
            ui.label("Material:");
            egui::ComboBox::from_id_salt("fea_material")
                .selected_text(format!("{:?}", self.fea_config.material.preset))
                .show_ui(ui, |ui| {
                    use crate::fea::MaterialPreset;
                    for preset in [MaterialPreset::PLA, MaterialPreset::PETG, MaterialPreset::ABS,
                                   MaterialPreset::CarbonFiber, MaterialPreset::Custom] {
                        let label = format!("{preset:?}");
                        if ui.selectable_value(&mut self.fea_config.material.preset, preset.clone(), &label).clicked() {
                            if preset != MaterialPreset::Custom {
                                self.fea_config.material = crate::fea::MaterialProperties::from_preset(preset);
                            }
                        }
                    }
                });
        });

        // Mesh resolution
        ui.horizontal(|ui| {
            ui.label("Mesh resolution:");
            ui.add(egui::Slider::new(&mut self.fea_config.mesh_resolution, 8..=64));
        });

        ui.separator();

        // Run / Cancel button
        let has_sdf = self.current_sdf.is_some();
        if self.fea_running {
            if ui.button("Cancel FEA").clicked() {
                // Drop the receiver — the thread will finish but results are ignored
                self.fea_receiver  = None;
                self.fea_running   = false;
            }
        } else if ui.add_enabled(has_sdf, egui::Button::new("Run FEA")).clicked() {
            self.start_fea_analysis();
        }

        // Results
        if let Some(ref result) = self.fea_result {
            ui.separator();
            ui.label(egui::RichText::new("Results").strong());
            ui.label(format!("Max Von Mises: {:.2} MPa", result.max_von_mises));
            ui.label(format!("Max displacement: {:.4} mm", result.max_displacement));

            // Safety factor
            let yield_mpa = self.fea_config.material.yield_strength_mpa;
            let sf = if result.max_von_mises > 0.0 { yield_mpa / result.max_von_mises } else { f32::INFINITY };
            let sf_color = if sf >= 2.0 { egui::Color32::GREEN }
                           else if sf >= 1.0 { egui::Color32::YELLOW }
                           else { egui::Color32::RED };
            ui.horizontal(|ui| {
                ui.label("Safety factor:");
                ui.colored_label(sf_color, format!("{sf:.2}"));
            });

            ui.separator();
            ui.label("Overlay:");
            let prev_mode = self.fea_overlay_mode;
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.fea_overlay_mode, FEAOverlayMode::None, "Off");
                ui.selectable_value(&mut self.fea_overlay_mode, FEAOverlayMode::Stress, "Stress");
                ui.selectable_value(&mut self.fea_overlay_mode, FEAOverlayMode::Displacement, "Displacement");
            });
            if self.fea_overlay_mode != prev_mode && self.fea_overlay_mode != FEAOverlayMode::None {
                self.fea_needs_upload = true;  // re-upload correct channel on mode change
            }

            // Turning on FEA overlay turns off thickness overlay
            if self.fea_overlay_mode != FEAOverlayMode::None {
                self.thickness_overlay_on = false;
            }
        }

        // Log output
        if !self.fea_log.is_empty() {
            ui.separator();
            ui.label("Log:");
            egui::ScrollArea::vertical().max_height(120.0).show(ui, |ui| {
                for line in &self.fea_log {
                    ui.label(egui::RichText::new(line).monospace().size(10.0));
                }
            });
        }

        // Clear button
        if self.fea_result.is_some() {
            ui.separator();
            if ui.button("Clear Results").clicked() {
                self.fea_result        = None;
                self.fea_overlay_mode  = FEAOverlayMode::None;
                self.fea_stress_field       = None;
                self.fea_displacement_field = None;
                self.fea_needs_upload  = false;
            }
        }
    }
}
