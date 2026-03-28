// Application state and UI layout

use eframe::{egui, egui_wgpu, wgpu};
use std::sync::{Arc, RwLock};
use std::collections::HashMap;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::profiles::SplineProfile;
use crate::scripting;
use crate::mesh::{Mesh, MeshQuality};
use crate::render::{Camera, StandardView, RenderState, GridRenderer, AxesRenderer, WireframeRenderer, RaymarchRenderer, SdfGrid, SectionUniforms, ThicknessUniforms};
use crate::analysis::thickness::{compute_thickness, ThicknessResult};
use crate::project::{SectionView, SectionPlane, Axis};
use crate::components::{ComponentRegistry, ComponentInstance};
use crate::scripting::MassPoint;
use crate::ui::spline_editor::{SplineEditorState, show_spline_editor};
use crate::ui::spine_editor::{SpineEditorState, show_spine_editor};
use crate::ui::project_tree::{ProjectTree, show_project_tree, find_name_in_script,
                               count_occurrences, rename_in_script};
use crate::undo::{AppState, UndoHistory, ScriptTextCommand, SplineShapeResetCommand,
                  LongitudinalSpineEditCommand, RenameCommand};
use crate::sdf::spine::LongitudinalSplines;
use crate::settings::AppSettings;
use crate::library::LibraryManager;
use crate::ui::library_panel::{LibraryPanelState, show_library_panel};
use crate::ui::examples;
use crate::ui::project_wizard::{show_wizard, WizardState};

struct ScriptEvalSuccess {
    sdf: Arc<dyn Sdf>,
    sdf_grid: Arc<SdfGrid>,
    mesh_volume_mm3: f32,
    cg: Option<Vec3>,
    mass_points: Vec<MassPoint>,
    fea_setup: crate::fea::FEASetup,
    layups: Vec<Arc<crate::sdf::aerospace::composite::CompositeLayup>>,
    reference_points: Vec<crate::scripting::ReferencePoint>,
    bounds_min: Vec3,
    bounds_max: Vec3,
    eval_time_ms: f64,
    mesh_time_ms: f64,
    component_preview_parts: Vec<(String, Arc<dyn Sdf>)>,
}

struct ScriptEvalResponse {
    job_id: u64,
    cells: Option<Vec<scripting::ScriptCell>>,
    outcome: Result<ScriptEvalSuccess, String>,
}

#[derive(Clone, Copy, PartialEq, Default)]
pub enum FEAOverlayMode { #[default] None, Stress, Displacement }

#[derive(Clone, Copy, PartialEq, Default)]
#[allow(dead_code)] // Arbitrary variant is matched but not constructed via a UI button yet
pub enum SplitAxisUi { #[default] Z, X, Y, Arbitrary }

#[derive(Clone, Copy, PartialEq, Default)]
pub enum SplitAlignUi { #[default] None, Pins, Groove, Dovetail, BoltHoles }

#[derive(Clone, Copy, PartialEq, Default)]
enum ComponentSelectionMode {
    #[default]
    Highlight,
    Isolate,
}

pub struct App {
    /// All user-editable state (participates in undo/redo).
    state:       AppState,
    /// Undo/redo history stack.
    undo_history: UndoHistory,
    /// Feedback text for status bar (shown 3 s after undo/redo).
    undo_feedback: Option<(String, std::time::Instant)>,
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
    processing_status: Option<String>,
    script_eval_receiver: Option<std::sync::mpsc::Receiver<ScriptEvalResponse>>,
    script_eval_job_id: u64,

    // Mesh quality controls
    resolution: u32,
    smooth_normals: bool,
    mesh_quality: MeshQuality,
    show_wireframe: bool,

    // Project management
    current_file_path: Option<std::path::PathBuf>,
    status_message: Option<String>,
    workflow_config: Option<crate::project::WorkflowConfig>,
    workflow_auto_apply_constraints: bool,
    workflow_variant_name: String,
    workflow_variant_description: String,
    project_wizard: WizardState,

    // (script text undo is managed by self.undo_history / AppState)

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


    // Mass / CG analysis
    density_g_per_cm3: f32,
    mass_points: Vec<MassPoint>,
    cg: Option<glam::Vec3>,
    mesh_volume_mm3: Option<f32>,

    // Background export
    export_in_progress: bool,
    export_progress: String,
    export_receiver: Option<std::sync::mpsc::Receiver<String>>,

    // Spline cross-section profiles (editor state lives in self.state.profiles)
    /// Thread-safe runtime view shared with the Rhai engine.
    profiles_shared: Arc<RwLock<HashMap<String, SplineProfile>>>,

    // Longitudinal spine (data lives in self.state.splines)
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

    // FEA integration (setup populated by scripts lives in self.state.fea_setup)
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

    // Dimensions panel
    dimensions_state:        crate::ui::DimensionsState,
    /// Set to true when a dimension changes; triggers re-eval on next frame.
    dimensions_pending_eval: bool,

    // Mesh import cache (shared with the Rhai engine).
    mesh_cache: Arc<std::sync::Mutex<scripting::MeshCache>>,

    // Composite layups collected from the last script eval (for the Layup Summary panel).
    current_layups: Vec<Arc<crate::sdf::aerospace::composite::CompositeLayup>>,

    // Print analysis
    print_analysis_open:     bool,
    print_analysis_settings: crate::analysis::print_analysis::PrintAnalysisSettings,
    print_analysis_result:   Option<crate::analysis::print_analysis::PrintAnalysisResult>,
    print_analysis_running:  bool,
    print_analysis_receiver: Option<std::sync::mpsc::Receiver<crate::analysis::print_analysis::PrintAnalysisResult>>,
    print_overhang_overlay:  bool,
    print_overhang_needs_upload: bool,

    // Split body tool
    split_axis:         SplitAxisUi,
    split_offset:       f32,
    split_align_type:   SplitAlignUi,
    split_pin_radius:   f32,
    split_pin_height:   f32,
    split_pin_count:    usize,
    split_pattern_r:    f32,
    split_groove_width: f32,
    split_groove_height: f32,
    split_dovetail_angle: f32,
    split_bolt_radius:  f32,
    split_boss_radius:  f32,
    split_bolt_count:   usize,
    split_preview:      Option<(std::sync::Arc<dyn crate::sdf::Sdf>, std::sync::Arc<dyn crate::sdf::Sdf>)>,
    split_fit_result:   Option<crate::sdf::print::SplitFitResult>,

    // Tolerance compensation
    tolerance_settings:   crate::sdf::print::ToleranceSettings,
    tolerance_preset:     crate::sdf::print::TolerancePreset,
    /// Whether to wrap geometry in ToleranceCompensated automatically at export time.
    tolerance_on_export:  bool,

    // Reference points from script evaluation
    current_ref_points: Vec<crate::scripting::ReferencePoint>,
    show_ref_points:    bool,
    show_dim_lines:     bool,

    // Project-local component library
    library_manager:     Option<LibraryManager>,
    library_panel_state: LibraryPanelState,

    // Aerodynamic analysis (Phase 26)
    current_flight_condition:    Option<crate::aero::FlightCondition>,
    current_lifting_line_result: Option<crate::aero::LiftingLineResult>,
    /// UI inputs for flight condition panel.
    aero_airspeed_ms:            f32,
    aero_altitude_m:             f32,
    aero_aoa_deg:                f32,
    aero_panel_open:             bool,

    // Aerodynamic analysis (Phase 27) — stability and drag polar
    current_neutral_point:       Option<crate::aero::NeutralPointResult>,
    current_static_margin:       Option<crate::aero::StaticMarginResult>,
    current_trim_result:         Option<crate::aero::TrimResult>,
    current_drag_polar:          Option<crate::aero::DragPolarResult>,
    /// CG x-position input (mm) for stability panel.
    aero_cg_x_mm:                f32,
    /// Aircraft weight (N) for trim and performance.
    aero_weight_n:               f32,

    // Cell-based script execution (Phase 28)
    /// Parsed cells from the current script_text.
    script_cells: Vec<scripting::ScriptCell>,

    // Phase 30 — CG sensitivity and interference analysis
    current_cg_sensitivity:       Option<crate::analysis::CgSensitivityResult>,
    current_interference_result:  Option<crate::analysis::InterferenceResult>,

    // Version control
    version_control:     crate::version_control::VersionControlState,
    vc_panel_open:       bool,
    vc_discard_confirm:  bool,
    vc_commit_message:   String,
    vc_new_branch_name:  String,
    vc_new_branch_desc:  String,
    vc_new_branch_dialog: bool,
    vc_panel_state:      crate::ui::VCPanelState,

    // Help / function reference panel
    help_panel_open:     bool,
    help_state:          crate::ui::HelpSearchState,
    /// Byte offset of the script editor cursor (updated each frame for "Insert at Cursor").
    script_cursor_byte:  usize,

    // Example browser / tab system
    /// Indices into examples::EXAMPLES for open read-only example tabs.
    open_example_tabs:     Vec<usize>,
    /// None = Main Script active; Some(idx) = example tab active.
    active_example_tab:    Option<usize>,
    /// Search string for the example browser dropdown.
    example_search:        String,
    /// Pending copy-to-main action from right-click context menu.
    pending_copy_example:  Option<usize>,
    component_preview_parts: Vec<(String, Arc<dyn Sdf>)>,
    selected_component_part: Option<String>,
    component_selection_mode: ComponentSelectionMode,
    selected_component_bounds: Option<(Vec3, Vec3)>,
}

impl App {
    pub fn new(_cc: &eframe::CreationContext) -> Self {
        let default_script = Self::get_default_example();
        let mut app = Self {
            state: AppState::new(default_script.clone()),
            undo_history: UndoHistory::default(),
            undo_feedback: None,
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
            processing_status: None,
            script_eval_receiver: None,
            script_eval_job_id: 0,
            resolution: 96,
            smooth_normals: true,
            mesh_quality: MeshQuality::Normal,
            show_wireframe: false,
            current_file_path: None,
            status_message: None,
            workflow_config: None,
            workflow_auto_apply_constraints: true,
            workflow_variant_name: "New Variant".to_string(),
            workflow_variant_description: String::new(),
            project_wizard: WizardState::default(),
            export_in_progress: false,
            export_progress: String::new(),
            export_receiver: None,
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
            density_g_per_cm3: 1.0,
            mass_points: Vec::new(),
            cg: None,
            mesh_volume_mm3: None,
            profiles_shared: Arc::new(RwLock::new(HashMap::new())),
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
            dimensions_state:        crate::ui::DimensionsState::default(),
            dimensions_pending_eval: false,
            mesh_cache:              Arc::new(std::sync::Mutex::new(scripting::MeshCache::new())),
            current_layups:          Vec::new(),
            print_analysis_open:     false,
            print_analysis_settings: crate::analysis::print_analysis::PrintAnalysisSettings::default(),
            print_analysis_result:   None,
            print_analysis_running:  false,
            print_analysis_receiver: None,
            print_overhang_overlay:  false,
            print_overhang_needs_upload: false,
            split_axis:          SplitAxisUi::Z,
            split_offset:        0.0,
            split_align_type:    SplitAlignUi::None,
            split_pin_radius:    1.5,
            split_pin_height:    3.0,
            split_pin_count:     4,
            split_pattern_r:     8.0,
            split_groove_width:  10.0,
            split_groove_height: 3.0,
            split_dovetail_angle: 15.0,
            split_bolt_radius:   1.5,
            split_boss_radius:   3.0,
            split_bolt_count:    4,
            split_preview:       None,
            split_fit_result:    None,
            tolerance_settings:  crate::sdf::print::ToleranceSettings::default(),
            tolerance_preset:    crate::sdf::print::TolerancePreset::StandardFDM,
            tolerance_on_export: false,
            current_ref_points:  Vec::new(),
            show_ref_points:     true,
            show_dim_lines:      false,
            library_manager:     None,
            library_panel_state: LibraryPanelState::default(),
            current_flight_condition:    None,
            current_lifting_line_result: None,
            aero_airspeed_ms:            50.0,
            aero_altitude_m:             0.0,
            aero_aoa_deg:                5.0,
            aero_panel_open:             false,
            current_neutral_point:       None,
            current_static_margin:       None,
            current_trim_result:         None,
            current_drag_polar:          None,
            aero_cg_x_mm:               100.0,
            aero_weight_n:              10.0,
            script_cells:               Vec::new(),
            current_cg_sensitivity:       None,
            current_interference_result:  None,
            version_control:     crate::version_control::VersionControlState::new_with_root(
                &AppState::new(default_script.clone())
            ),
            vc_panel_open:       false,
            vc_discard_confirm:  false,
            vc_commit_message:   String::new(),
            vc_new_branch_name:  String::new(),
            vc_new_branch_desc:  String::new(),
            vc_new_branch_dialog: false,
            vc_panel_state:      crate::ui::VCPanelState::default(),
            help_panel_open:     false,
            help_state:          crate::ui::HelpSearchState::new(),
            script_cursor_byte:  0,
            open_example_tabs:   Vec::new(),
            active_example_tab:  None,
            example_search:      String::new(),
            pending_copy_example: None,
            component_preview_parts: Vec::new(),
            selected_component_part: None,
            component_selection_mode: ComponentSelectionMode::Highlight,
            selected_component_bounds: None,
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

    #[allow(dead_code)]
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
            ("Script Variables Demo", "\
# === Parameters ===
// All numeric literals in each section appear as live sliders on the left.
// Drag a slider or type a new value — the shape updates instantly.

let wingspan    = 200.0;
let root_chord  = 40.0;
let tip_chord   = 20.0;
let sweep_deg   = 15.0;
let dihedral_deg = 3.0;
let twist_deg   = -2.0;

# === Fuselage ===
let fuse_length   = 150.0;
let fuse_diameter = 22.0;
let fuse = fuselage_parametric(fuse_length, fuse_diameter, 0.7, 0.4);

# === Wing ===
let wing = wing_with_airfoil(\"2412\", root_chord, tip_chord, wingspan, sweep_deg, dihedral_deg, twist_deg);
let wing = translate(wing, fuse_length * 0.35, 0.0, 0.0);
let full_wing = full_assembly(wing);

# === Assembly ===
blend(fuse, full_wing, 5.0)
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

    fn update_vc_working_changes(&mut self) {
        if let Some(ref head_id) = self.version_control.head_commit_id.clone() {
            if let Some(head_commit) = self.version_control.commits.get(head_id) {
                let head_state = head_commit.state.clone();
                self.version_control.working_changes =
                    crate::version_control::operations::has_working_changes(
                        &self.state, &head_state);
            }
        }
    }

    fn execute_script(&mut self) {
        let script = self.state.script_text.clone();

        // Re-detect script variables for live manipulation sliders (Phase 31).
        self.dimensions_state.detected_variables =
            crate::ui::script_variable_detector::detect_script_variables(&script);

        // Handle empty script - clear mesh
        let trimmed = script.trim();
        if trimmed.is_empty() || trimmed.lines().all(|line| line.trim().starts_with("//")) {
            self.current_sdf = None;
            self.current_mesh = None;
            self.error_message = None;
            self.eval_time_ms = None;
            self.mesh_time_ms = None;
            self.is_processing = false;
            self.processing_status = None;
            self.script_eval_receiver = None;
            return;
        }

        // Re-parse cells; preserve prior statuses by id.
        let mut new_cells = scripting::parse_cells(&script);
        for cell in &mut new_cells {
            if let Some(old) = self.script_cells.iter().find(|c| c.id == cell.id) {
                cell.status = old.status.clone();
            }
        }
        // Reset statuses to Pending for fresh eval.
        for cell in &mut new_cells {
            cell.status = scripting::CellStatus::Pending;
        }

        // Use cell-aware evaluation when more than one cell exists.
        if new_cells.len() > 1 {
            self.script_cells = new_cells;
            self.start_background_script_eval(script, Some(self.script_cells.clone()));
        } else {
            self.script_cells = new_cells;
            self.start_background_script_eval(script, None);
        }

        // Update working changes indicator.
        self.update_vc_working_changes();
    }

    fn start_background_script_eval(&mut self, script: String, cells: Option<Vec<scripting::ScriptCell>>) {
        use std::time::Instant;

        self.script_eval_job_id = self.script_eval_job_id.wrapping_add(1);
        let job_id = self.script_eval_job_id;
        self.is_processing = true;
        self.processing_status = Some(if cells.is_some() {
            "Evaluating script cells and rebuilding viewport…".to_string()
        } else {
            "Evaluating script and rebuilding viewport…".to_string()
        });
        self.error_message = None;

        let profiles_ref = Arc::clone(&self.profiles_shared);
        let splines_ref = Arc::new(self.state.splines.clone());
        let sf = self.fea_stress_field.clone();
        let df = self.fea_displacement_field.clone();
        let dimensions = self.state.dimensions.clone();
        let project_dir = self.current_file_path.as_deref().and_then(|p| p.parent()).map(|p| p.to_path_buf());
        let library_sources = self.library_manager.as_ref()
            .map(|m| m.module_sources())
            .unwrap_or_default();
        let mesh_cache = Arc::clone(&self.mesh_cache);
        let viewport_resolution = self.resolution.clamp(48, 256);
        let is_component_script = self.current_file_path.as_ref()
            .and_then(|p| p.parent().and_then(|parent| parent.file_name()).map(|n| n == "components"))
            .unwrap_or(false);
        let (tx, rx) = std::sync::mpsc::channel();
        self.script_eval_receiver = Some(rx);

        std::thread::spawn(move || {
            let start_eval = Instant::now();
            let response = if let Some(mut worker_cells) = cells {
                let result_opt = scripting::evaluate_script_cells(
                    &script,
                    &mut worker_cells,
                    Some(profiles_ref),
                    Some(splines_ref),
                    sf, df,
                    &dimensions,
                    project_dir.as_deref(),
                    Some(mesh_cache),
                    &library_sources,
                );
                let eval_time_ms = start_eval.elapsed().as_secs_f64() * 1000.0;
                let outcome = if let Some(result) = result_opt {
                    build_script_eval_success(result, viewport_resolution, eval_time_ms, Vec::new())
                } else {
                    let err_msg = worker_cells.iter()
                        .filter_map(|c| {
                            if let scripting::CellStatus::Error { message, line } = &c.status {
                                let line_info = line.map(|l| format!(" (line {})", l)).unwrap_or_default();
                                Some(format!("[{}]{}: {}", c.name, line_info, message))
                            } else {
                                None
                            }
                        })
                        .collect::<Vec<_>>()
                        .join("\n");
                    Err(err_msg)
                };
                ScriptEvalResponse { job_id, cells: Some(worker_cells), outcome }
            } else {
                let outcome = match scripting::evaluate_script_full(
                    &script,
                    Some(profiles_ref),
                    Some(splines_ref),
                    sf.clone(), df.clone(),
                    &dimensions,
                    project_dir.as_deref(),
                    Some(Arc::clone(&mesh_cache)),
                    &library_sources,
                ) {
                    Ok(result) => {
                        let eval_time_ms = start_eval.elapsed().as_secs_f64() * 1000.0;
                        let component_preview_parts = scripting::evaluate_preview_parts(
                            &script,
                            None,
                            None,
                            sf.clone(),
                            df.clone(),
                            &dimensions,
                            project_dir.as_deref(),
                            Some(Arc::clone(&mesh_cache)),
                            &library_sources,
                        ).unwrap_or_else(|_| {
                            if is_component_script {
                                scripting::evaluate_component_preview_parts(
                                    &script,
                                    None,
                                    None,
                                    sf.clone(),
                                    df.clone(),
                                    &dimensions,
                                    project_dir.as_deref(),
                                    Some(Arc::clone(&mesh_cache)),
                                    &library_sources,
                                ).unwrap_or_default()
                            } else {
                                Vec::new()
                            }
                        });
                        build_script_eval_success(result, viewport_resolution, eval_time_ms, component_preview_parts)
                    }
                    Err(e) => Err(e),
                };
                ScriptEvalResponse { job_id, cells: None, outcome }
            };
            let _ = tx.send(response);
        });
    }

    fn apply_script_eval_success(&mut self, success: ScriptEvalSuccess) {
        let should_frame_camera = self.current_sdf_grid.is_none();
        self.component_preview_parts = success.component_preview_parts;
        if self.component_preview_parts.is_empty() {
            self.selected_component_part = None;
            self.selected_component_bounds = None;
        } else if let Some(selected) = self.selected_component_part.clone() {
            if !self.component_preview_parts.iter().any(|(name, _)| *name == selected) {
                self.selected_component_part = None;
                self.selected_component_bounds = None;
            }
        }
        self.mesh_volume_mm3 = Some(success.mesh_volume_mm3);
        self.cg = success.cg;
        self.mass_points = success.mass_points;
        if let Some(cg) = self.cg {
            self.aero_cg_x_mm = cg.x;
        }

        if should_frame_camera {
            if let Some((tight_min, tight_max)) = crate::pipeline::tight_bounds_from_grid(&success.sdf_grid) {
                self.camera.frame_bounds(tight_min, tight_max);
            } else {
                self.camera.frame_bounds(success.bounds_min, success.bounds_max);
            }
        }

        self.state.fea_setup = success.fea_setup;
        if !self.state.fea_setup.is_empty() {
            self.fea_viz = Some(crate::fea::compute_fea_viz(
                &self.state.fea_setup, success.bounds_min, success.bounds_max,
            ));
        } else {
            self.fea_viz = None;
        }

        self.current_layups = success.layups;
        self.current_ref_points = success.reference_points;
        self.current_sdf = Some(success.sdf);
        self.current_mesh = None;
        self.current_sdf_grid = Some(success.sdf_grid);
        self.error_message = None;
        self.eval_time_ms = Some(success.eval_time_ms);
        self.mesh_time_ms = Some(success.mesh_time_ms);

        if self.measurements.volume_mm3.is_some() {
            self.measurements_stale = true;
        }
        if self.print_analysis_result.is_some() {
            self.print_analysis_result       = None;
            self.print_overhang_overlay      = false;
            self.print_overhang_needs_upload = false;
        }

        let profile_names: Vec<String> = self.state.profiles.keys().cloned().collect();
        self.project_tree.rebuild(
            &self.mass_points,
            &self.state.fea_setup,
            &profile_names,
            &self.state.splines,
            &self.current_ref_points,
        );

        if let Some(selected) = self.selected_component_part.clone() {
            self.set_component_preview_selection(Some(selected), false);
        }
    }

    fn set_component_preview_selection(&mut self, selected: Option<String>, jump_to_code: bool) {
        self.selected_component_part = selected.clone();
        let viewport_resolution = self.resolution.clamp(48, 256);

        if let Some(name) = selected.clone() {
            let selected_sdf = self.component_preview_parts.iter()
                .find(|(n, _)| *n == name)
                .map(|(_, sdf)| Arc::clone(sdf));

            if let Some(part_sdf) = selected_sdf {
                let (part_min, part_max) = crate::pipeline::auto_bounds(part_sdf.as_ref());
                self.selected_component_bounds = Some((part_min, part_max));

                let display_sdf = match self.component_selection_mode {
                    ComponentSelectionMode::Highlight => self.current_sdf.clone(),
                    ComponentSelectionMode::Isolate => Some(part_sdf),
                };

                if let Some(sdf) = display_sdf {
                    let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(sdf.as_ref());
                    self.current_sdf_grid = Some(crate::pipeline::compute_sdf_grid_cached_arc(
                        &sdf, bounds_min, bounds_max, viewport_resolution,
                    ));
                }

                self.status_message = Some(format!(
                    "{} preview part '{}'",
                    match self.component_selection_mode {
                        ComponentSelectionMode::Highlight => "Highlighted",
                        ComponentSelectionMode::Isolate => "Isolated",
                    },
                    name
                ));
                if jump_to_code {
                    if let Some((offset, line)) = find_name_in_script(&self.state.script_text, &name) {
                        self.pending_cursor_offset = Some(offset);
                        self.pending_scroll_line = Some(line);
                    }
                }
            }
        } else if let Some(sdf) = self.current_sdf.clone() {
            self.selected_component_bounds = None;
            let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(sdf.as_ref());
            self.current_sdf_grid = Some(crate::pipeline::compute_sdf_grid_cached_arc(
                &sdf, bounds_min, bounds_max, viewport_resolution,
            ));
            self.status_message = Some("Showing full preview".to_string());
        }
    }

    fn evaluate_and_render(&mut self, script: &str) {
        use std::time::Instant;
        self.is_processing = true;

        let start_eval = Instant::now();
        let profiles_ref = Arc::clone(&self.profiles_shared);
        let splines_ref = Arc::new(self.state.splines.clone());
        let sf = self.fea_stress_field.clone();
        let df = self.fea_displacement_field.clone();
        let project_dir = self.current_file_path.as_deref().and_then(|p| p.parent()).map(|p| p.to_path_buf());
        let library_sources = self.library_manager.as_ref()
            .map(|m| m.module_sources())
            .unwrap_or_default();
        match scripting::evaluate_script_full(
            script,
            Some(profiles_ref),
            Some(splines_ref),
            sf, df,
            &self.state.dimensions,
            project_dir.as_deref(),
            Some(Arc::clone(&self.mesh_cache)),
            &library_sources,
        ) {
            Ok(result) => {
                let should_frame_camera = self.current_sdf_grid.is_none();
                let eval_time = start_eval.elapsed().as_secs_f64() * 1000.0;

                // Extract CG/mass data before moving sdf out of result
                let cg = result.center_of_gravity();
                let mass_points = result.mass_points;
                self.state.fea_setup    = result.fea_setup;
                self.current_layups     = result.layups;
                self.current_ref_points = result.reference_points;
                let sdf = result.sdf;

                // Compute bounds once, reuse for both SDF grid and mesh.
                let start_mesh = Instant::now();
                let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(sdf.as_ref());
                let viewport_resolution = self.resolution.clamp(48, 256);

                let sdf_grid = crate::pipeline::compute_sdf_grid_cached_arc(
                    &sdf, bounds_min, bounds_max, viewport_resolution,
                );
                let mesh_time = start_mesh.elapsed().as_secs_f64() * 1000.0;

                // Estimate volume from negative voxel count (no marching cubes needed).
                let step = (bounds_max - bounds_min) / viewport_resolution as f32;
                let voxel_vol = step.x * step.y * step.z;
                let inside = sdf_grid.data.iter().filter(|&&d| d < 0.0).count();
                let vol = inside as f32 * voxel_vol;
                self.mesh_volume_mm3 = Some(vol);
                self.cg = cg;
                self.mass_points = mass_points;
                if let Some(cg) = self.cg {
                    self.aero_cg_x_mm = cg.x;
                }

                // Frame camera on tight bounds of negative (interior) voxels.
                if should_frame_camera {
                    if let Some((tight_min, tight_max)) = crate::pipeline::tight_bounds_from_grid(&sdf_grid) {
                        self.camera.frame_bounds(tight_min, tight_max);
                    } else {
                        self.camera.frame_bounds(bounds_min, bounds_max);
                    }
                }

                // Pre-compute FEA BC visualization geometry (cheap 20³ sample).
                if !self.state.fea_setup.is_empty() {
                    self.fea_viz = Some(crate::fea::compute_fea_viz(
                        &self.state.fea_setup, bounds_min, bounds_max,
                    ));
                } else {
                    self.fea_viz = None;
                }

                self.current_sdf = Some(sdf);
                self.current_mesh = None;
                self.current_sdf_grid = Some(sdf_grid);
                self.error_message = None;
                self.eval_time_ms = Some(eval_time);
                self.mesh_time_ms = Some(mesh_time);

                // Mark measurements stale (model changed).
                if self.measurements.volume_mm3.is_some() {
                    self.measurements_stale = true;
                }

                // Print analysis results are stale when the model changes.
                if self.print_analysis_result.is_some() {
                    self.print_analysis_result       = None;
                    self.print_overhang_overlay      = false;
                    self.print_overhang_needs_upload = false;
                }

                // Rebuild project tree from latest data.
                let profile_names: Vec<String> = self.state.profiles.keys().cloned().collect();
                self.project_tree.rebuild(
                    &self.mass_points,
                    &self.state.fea_setup,
                    &profile_names,
                    &self.state.splines,
                    &self.current_ref_points,
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

    /// Cell-aware evaluation: runs each cell sequentially in a shared Rhai scope.
    /// Updates `self.script_cells` statuses and, on success, renders as usual.
    fn evaluate_and_render_cells(&mut self, script: &str) {
        use std::time::Instant;
        self.is_processing = true;

        let start_eval = Instant::now();
        let profiles_ref = Arc::clone(&self.profiles_shared);
        let splines_ref = Arc::new(self.state.splines.clone());
        let sf = self.fea_stress_field.clone();
        let df = self.fea_displacement_field.clone();
        let project_dir = self.current_file_path.as_deref().and_then(|p| p.parent()).map(|p| p.to_path_buf());
        let library_sources = self.library_manager.as_ref()
            .map(|m| m.module_sources())
            .unwrap_or_default();

        let result_opt = scripting::evaluate_script_cells(
            script,
            &mut self.script_cells,
            Some(profiles_ref),
            Some(splines_ref),
            sf, df,
            &self.state.dimensions,
            project_dir.as_deref(),
            Some(Arc::clone(&self.mesh_cache)),
            &library_sources,
        );

        let eval_time = start_eval.elapsed().as_secs_f64() * 1000.0;

        match result_opt {
            Some(result) => {
                let should_frame_camera = self.current_sdf_grid.is_none();
                let cg = result.center_of_gravity();
                let mass_points = result.mass_points;
                self.state.fea_setup    = result.fea_setup;
                self.current_layups     = result.layups;
                self.current_ref_points = result.reference_points;
                let sdf = result.sdf;

                let start_mesh = Instant::now();
                let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(sdf.as_ref());
                let viewport_resolution = self.resolution.clamp(48, 256);

                let sdf_grid = crate::pipeline::compute_sdf_grid_cached_arc(
                    &sdf, bounds_min, bounds_max, viewport_resolution,
                );
                let mesh_time = start_mesh.elapsed().as_secs_f64() * 1000.0;

                let step = (bounds_max - bounds_min) / viewport_resolution as f32;
                let voxel_vol = step.x * step.y * step.z;
                let inside = sdf_grid.data.iter().filter(|&&d| d < 0.0).count();
                self.mesh_volume_mm3 = Some(inside as f32 * voxel_vol);
                self.cg = cg;
                self.mass_points = mass_points;
                if let Some(cg) = self.cg {
                    self.aero_cg_x_mm = cg.x;
                }

                if should_frame_camera {
                    if let Some((tight_min, tight_max)) = crate::pipeline::tight_bounds_from_grid(&sdf_grid) {
                        self.camera.frame_bounds(tight_min, tight_max);
                    } else {
                        self.camera.frame_bounds(bounds_min, bounds_max);
                    }
                }

                if !self.state.fea_setup.is_empty() {
                    self.fea_viz = Some(crate::fea::compute_fea_viz(
                        &self.state.fea_setup, bounds_min, bounds_max,
                    ));
                } else {
                    self.fea_viz = None;
                }

                self.current_sdf = Some(sdf);
                self.current_mesh = None;
                self.current_sdf_grid = Some(sdf_grid);
                self.error_message = None;
                self.eval_time_ms = Some(eval_time);
                self.mesh_time_ms = Some(mesh_time);

                if self.measurements.volume_mm3.is_some() {
                    self.measurements_stale = true;
                }
                if self.print_analysis_result.is_some() {
                    self.print_analysis_result       = None;
                    self.print_overhang_overlay      = false;
                    self.print_overhang_needs_upload = false;
                }

                let profile_names: Vec<String> = self.state.profiles.keys().cloned().collect();
                self.project_tree.rebuild(
                    &self.mass_points,
                    &self.state.fea_setup,
                    &profile_names,
                    &self.state.splines,
                    &self.current_ref_points,
                );
            }
            None => {
                // Build error message from failed cells.
                let err_msg = self.script_cells.iter()
                    .filter_map(|c| {
                        if let scripting::CellStatus::Error { message, line } = &c.status {
                            let line_info = line.map(|l| format!(" (line {})", l)).unwrap_or_default();
                            Some(format!("[{}]{}: {}", c.name, line_info, message))
                        } else {
                            None
                        }
                    })
                    .collect::<Vec<_>>()
                    .join("\n");
                self.error_message = Some(err_msg);
                self.eval_time_ms = None;
                self.mesh_time_ms = None;
            }
        }

        self.is_processing = false;
    }

    fn apply_workflow_constraints(&mut self) -> usize {
        let applied = self.workflow_config.as_ref()
            .map(|cfg| crate::project::apply_assembly_constraints(
                &mut self.state.dimensions,
                &cfg.assembly_constraints,
            ))
            .unwrap_or(0);
        if applied > 0 {
            self.dimensions_pending_eval = true;
        }
        applied
    }

    fn run_workflow_flight_analysis(&mut self) {
        let Some(sdf) = self.current_sdf.clone() else {
            self.error_message = Some("Run the script first to generate geometry.".into());
            return;
        };

        use crate::aero::{compute_drag_polar, compute_neutral_point, compute_static_margin, compute_trim, FlightCondition, PolarDatabase};
        use crate::sdf::query::bounding_points;

        let fc = FlightCondition::new(self.aero_airspeed_ms, self.aero_altitude_m, self.aero_aoa_deg);
        let db = PolarDatabase::new();
        let np = compute_neutral_point(&sdf, &sdf, &sdf, &db, &fc);
        let bbox = bounding_points(sdf.as_ref());
        let root_chord = bbox.size.x.max(1.0);
        let mac = (root_chord + root_chord * 0.5) * 0.5;
        let cg_x = self.cg.map(|cg| cg.x).unwrap_or(self.aero_cg_x_mm);
        let cg = glam::Vec3::new(cg_x, 0.0, 0.0);
        let sm = compute_static_margin(&np, cg, mac);
        let trim = compute_trim(&np, &sm, &sdf, &sdf, &sdf, &db, &fc, self.aero_weight_n);
        let drag = compute_drag_polar(&sdf, &sdf, &sdf, &sdf, &db, &fc, Some(self.aero_weight_n));

        self.current_neutral_point = Some(np.clone());
        self.current_static_margin = Some(sm.clone());
        self.current_trim_result = Some(trim);
        self.current_drag_polar = Some(drag);
        self.current_flight_condition = Some(fc);

        if !self.mass_points.is_empty() {
            let comps: Vec<(String, glam::Vec3, f32)> = self.mass_points
                .iter()
                .map(|m| (m.name.clone(), m.position, m.mass_g))
                .collect();
            let fwd = np.neutral_point_x_mm - 0.25 * mac;
            self.current_cg_sensitivity = Some(
                crate::analysis::compute_cg_sensitivity(
                    &comps,
                    &self.state.dimensions,
                    np.neutral_point_x_mm,
                    mac,
                    fwd,
                )
            );
        }
    }

    fn save_project(&mut self) {
        if let Some(path) = &self.current_file_path {
            if Self::is_rhai_path(path) {
                self.save_script_to_path(path.clone());
            } else {
                self.save_project_to_path(path.clone());
            }
        } else {
            self.save_project_as();
        }
    }

    fn save_project_as(&mut self) {
        let save_script = self.current_file_path
            .as_ref()
            .map(|p| Self::is_rhai_path(p.as_path()))
            .unwrap_or(false);
        let dialog = rfd::FileDialog::new()
            .add_filter("Rhai Script", &["rhai"])
            .add_filter("Implicit CAD Project", &["icad"])
            .set_file_name(if save_script { "script.rhai" } else { "project.icad" });
        if let Some(path) = dialog.save_file() {
            if save_script || Self::is_rhai_path(&path) {
                self.save_script_to_path(path);
            } else {
                self.save_project_to_path(path);
            }
        }
    }

    fn save_script_to_path(&mut self, path: std::path::PathBuf) {
        match std::fs::write(&path, &self.state.script_text) {
            Ok(_) => {
                self.current_file_path = Some(path.clone());
                self.status_message = Some(format!("Saved script to {}", path.display()));
                Self::clear_auto_save();
            }
            Err(e) => {
                self.error_message = Some(format!("Failed to save script: {}", e));
            }
        }
    }

    fn save_project_to_path(&mut self, path: std::path::PathBuf) {
        let profiles_opt = if self.state.profiles.is_empty() {
            None
        } else {
            Some(self.state.profiles.clone())
        };
        let splines_opt = if self.state.splines == LongitudinalSplines::default() {
            None
        } else {
            Some(self.state.splines.clone())
        };
        let section_opt = Some(self.section_view.clone());
        let project = crate::project::Project::new(
            self.state.script_text.clone(),
            self.resolution,
            self.smooth_normals,
            self.show_wireframe,
            [self.camera.eye.x, self.camera.eye.y, self.camera.eye.z],
            [self.camera.target.x, self.camera.target.y, self.camera.target.z],
            profiles_opt,
            splines_opt,
            section_opt,
            Some(self.fea_config.clone()),
            self.state.dimensions.clone(),
            Some(self.print_analysis_settings.clone()),
            Some(self.tolerance_settings.clone()),
            self.workflow_config.clone(),
        );

        let mut project_with_vc = project;
        project_with_vc.version_control = Some(self.version_control.clone());

        match project_with_vc.save(&path) {
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
            .add_filter("Rhai Script", &["rhai"])
            .pick_file()
        {
            if Self::is_rhai_path(&path) {
                match std::fs::read_to_string(&path) {
                    Ok(script) => {
                        self.state.script_text = script;
                        self.current_file_path = Some(path.clone());
                        self.status_message = Some(format!("Loaded script {}", path.display()));
                        self.undo_history.clear();
                        self.version_control = crate::version_control::VersionControlState::new_with_root(&self.state);
                        if let Some(dir) = path.parent() {
                            let lib_dir = dir.join("lib");
                            let mut mgr = LibraryManager::new(lib_dir);
                            mgr.scan();
                            self.library_manager = Some(mgr);
                        }
                        self.execute_script();
                    }
                    Err(e) => {
                        self.error_message = Some(format!("Failed to load script: {}", e));
                    }
                }
            } else {
                match crate::project::Project::load(&path) {
                    Ok(project) => {
                        self.state.script_text = project.script;
                        self.resolution = project.resolution;
                        self.smooth_normals = project.smooth_normals;
                        self.show_wireframe = project.show_wireframe;

                        // Restore camera position
                        self.camera.eye = Vec3::from_array(project.camera_position);
                        self.camera.target = Vec3::from_array(project.camera_target);

                        // Restore profiles if present (orphaned profiles are preserved
                        // by merging — existing profiles not in the file are kept).
                        if let Some(loaded_profiles) = project.profiles {
                            for (name, state) in loaded_profiles {
                                self.state.profiles.insert(name, state);
                            }
                            self.sync_profiles_shared();
                        }

                        // Restore spine constraints if present
                        if let Some(splines) = project.splines {
                            self.state.splines = splines;
                        }

                        // Restore section view if present
                        if let Some(sv) = project.section_view {
                            self.section_view = sv;
                        }

                        // Restore FEA config if present
                        if let Some(fc) = project.fea_config {
                            self.fea_config = fc;
                        }

                        // Restore named dimensions
                        if !project.dimensions.is_empty() {
                            self.state.dimensions = project.dimensions;
                        }

                        // Restore print analysis settings if present
                        if let Some(pa) = project.print_analysis_settings {
                            self.print_analysis_settings = pa;
                        }

                        // Restore tolerance settings if present
                        if let Some(ts) = project.tolerance_settings {
                            self.tolerance_settings = ts;
                        }
                        self.workflow_config = project.workflow_config;

                        // Restore version control state (or create fresh from loaded state)
                        self.version_control = project.version_control.unwrap_or_else(|| {
                            crate::version_control::VersionControlState::new_with_root(&self.state)
                        });

                        self.current_file_path = Some(path.clone());
                        self.status_message = Some(format!("Loaded {}", path.display()));
                        // Undo history is session-only — clear on load.
                        self.undo_history.clear();

                        // Initialize library manager for project lib/ dir
                        if let Some(dir) = path.parent() {
                            let lib_dir = dir.join("lib");
                            let mut mgr = LibraryManager::new(lib_dir);
                            mgr.scan();
                            self.library_manager = Some(mgr);
                        }

                        // Execute the loaded script
                        self.execute_script();
                    }
                    Err(e) => {
                        self.error_message = Some(format!("Failed to load: {}", e));
                    }
                }
            }
        }
    }

    fn is_rhai_path(path: &std::path::Path) -> bool {
        path.extension()
            .and_then(|s| s.to_str())
            .map(|ext| ext.eq_ignore_ascii_case("rhai"))
            .unwrap_or(false)
    }

    // push_history replaced by undo_history.execute(ScriptTextCommand) at each change site.

    fn undo_app(&mut self) {
        // Split borrow: undo_history needs &mut AppState separately.
        let state = &mut self.state;
        self.undo_history.undo(state);
        if let Some((msg, _)) = &self.undo_history.last_action {
            self.undo_feedback = Some((msg.clone(), std::time::Instant::now()));
        }
        self.dimensions_pending_eval = true;
    }

    fn redo_app(&mut self) {
        let state = &mut self.state;
        self.undo_history.redo(state);
        if let Some((msg, _)) = &self.undo_history.last_action {
            self.undo_feedback = Some((msg.clone(), std::time::Instant::now()));
        }
        self.dimensions_pending_eval = true;
    }

    /// Detect the SDF's spatial extent. Returns a tight asymmetric bounding box so that
    /// marching-cubes voxels are distributed proportionally across each axis.
    fn start_export_async(&mut self, path: String, is_obj: bool) {
        let Some(ref sdf) = self.current_sdf else { return };
        let Some(ref grid) = self.current_sdf_grid else { return };

        // Optionally wrap with tolerance compensation.
        let sdf: Arc<dyn crate::sdf::Sdf> = if self.tolerance_on_export {
            Arc::new(crate::sdf::print::ToleranceCompensated::new(
                Arc::clone(sdf),
                self.tolerance_settings.clone(),
            ))
        } else {
            Arc::clone(sdf)
        };
        let bounds_min = grid.bounds_min;
        let bounds_max = grid.bounds_max;
        let smooth = self.smooth_normals;
        let quality = self.mesh_quality;

        let (tx, rx) = std::sync::mpsc::channel::<String>();
        self.export_receiver = Some(rx);
        self.export_in_progress = true;
        self.export_progress = format!("Building mesh at {} quality…", quality.label());

        std::thread::spawn(move || {
            tx.send(format!("Building {} mesh ({:.2} mm cells)…", quality.label(), quality.target_cell_size_mm())).ok();

            let mesh = crate::export::build_export_mesh(sdf.as_ref(), bounds_min, bounds_max, quality, smooth);
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

    fn start_export_package_async(&mut self, output_dir: String) {
        let Some(ref sdf) = self.current_sdf else { return };
        let Some(ref grid) = self.current_sdf_grid else { return };

        let sdf: Arc<dyn crate::sdf::Sdf> = if self.tolerance_on_export {
            Arc::new(crate::sdf::print::ToleranceCompensated::new(
                Arc::clone(sdf),
                self.tolerance_settings.clone(),
            ))
        } else {
            Arc::clone(sdf)
        };

        let bounds_min = grid.bounds_min;
        let bounds_max = grid.bounds_max;
        let smooth = self.smooth_normals;
        let quality = self.mesh_quality;
        let project_name = self.current_file_path.as_ref()
            .and_then(|p| p.file_stem())
            .and_then(|s| s.to_str())
            .unwrap_or("implicit_cad_project")
            .to_string();

        let (tx, rx) = std::sync::mpsc::channel::<String>();
        self.export_receiver = Some(rx);
        self.export_in_progress = true;
        self.export_progress = "Building manufacturing package…".to_string();

        std::thread::spawn(move || {
            tx.send(format!("Building package mesh ({:.2} mm cells)…", quality.target_cell_size_mm())).ok();
            let mesh = crate::export::build_export_mesh(sdf.as_ref(), bounds_min, bounds_max, quality, smooth);
            match crate::export::export_manufacturing_package(&mesh, &project_name, std::path::Path::new(&output_dir)) {
                Ok(pkg) => {
                    tx.send(format!("✓ Package saved to {} ({} STL file(s))", output_dir, pkg.stl_files.len())).ok();
                }
                Err(e) => {
                    tx.send(format!("✗ Package export failed: {}", e)).ok();
                }
            }
        });
    }

    fn get_auto_save_path() -> std::path::PathBuf {
        let mut path = std::env::temp_dir();
        path.push("implicit_cad_autosave.rhai");
        path
    }

    fn auto_save(&mut self) {
        let path = Self::get_auto_save_path();
        if let Err(e) = std::fs::write(&path, &self.state.script_text) {
            eprintln!("Auto-save failed: {}", e);
        }
        self.last_auto_save = std::time::Instant::now();
    }

    fn try_restore_auto_save(&mut self) -> bool {
        let path = Self::get_auto_save_path();
        if path.exists() {
            if let Ok(content) = std::fs::read_to_string(&path) {
                if !content.is_empty() && content != Self::get_default_example() {
                    self.state.script_text = content;
                    self.undo_history.clear();
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
            for (name, state) in &self.state.profiles {
                write.insert(name.clone(), state.to_profile());
            }
        }
    }
}

fn build_script_eval_success(
    result: scripting::ScriptResult,
    viewport_resolution: u32,
    eval_time_ms: f64,
    component_preview_parts: Vec<scripting::ComponentPreviewPart>,
) -> Result<ScriptEvalSuccess, String> {
    let cg = result.center_of_gravity();
    let mass_points = result.mass_points;
    let fea_setup = result.fea_setup;
    let layups = result.layups;
    let reference_points = result.reference_points;
    let sdf = result.sdf;

    let start_mesh = std::time::Instant::now();
    let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(sdf.as_ref());
    let sdf_grid = crate::pipeline::compute_sdf_grid_cached_arc(
        &sdf, bounds_min, bounds_max, viewport_resolution,
    );
    let mesh_time_ms = start_mesh.elapsed().as_secs_f64() * 1000.0;

    let step = (bounds_max - bounds_min) / viewport_resolution as f32;
    let voxel_vol = step.x * step.y * step.z;
    let inside = sdf_grid.data.iter().filter(|&&d| d < 0.0).count();
    let mesh_volume_mm3 = inside as f32 * voxel_vol;

    Ok(ScriptEvalSuccess {
        sdf,
        sdf_grid,
        mesh_volume_mm3,
        cg,
        mass_points,
        fea_setup,
        layups,
        reference_points,
        bounds_min,
        bounds_max,
        eval_time_ms,
        mesh_time_ms,
        component_preview_parts: component_preview_parts.into_iter()
            .map(|p| (p.name, p.sdf))
            .collect(),
    })
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Auto-save periodically
        if self.last_auto_save.elapsed() >= self.auto_save_interval {
            self.auto_save();
        }

        if self.is_processing {
            let mut latest_response: Option<ScriptEvalResponse> = None;
            if let Some(ref rx) = self.script_eval_receiver {
                while let Ok(resp) = rx.try_recv() {
                    latest_response = Some(resp);
                }
            }
            if let Some(resp) = latest_response {
                if resp.job_id == self.script_eval_job_id {
                    if let Some(cells) = resp.cells {
                        self.script_cells = cells;
                    }
                    match resp.outcome {
                        Ok(success) => {
                            self.apply_script_eval_success(success);
                            self.status_message = Some("Script rebuilt".to_string());
                        }
                        Err(e) => {
                            self.error_message = Some(e);
                            self.eval_time_ms = None;
                            self.mesh_time_ms = None;
                        }
                    }
                    self.is_processing = false;
                    self.processing_status = None;
                    self.script_eval_receiver = None;
                }
            }
            if self.is_processing {
                ctx.request_repaint_after(std::time::Duration::from_millis(50));
            }
        }

        // Trigger re-eval when dimensions changed last frame.
        if self.dimensions_pending_eval {
            self.dimensions_pending_eval = false;
            self.execute_script();
            // execute_script calls update_vc_working_changes internally
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

        // Poll print analysis background thread
        if self.print_analysis_running {
            let mut done = false;
            let mut result = None;
            if let Some(ref rx) = self.print_analysis_receiver {
                if let Ok(r) = rx.try_recv() {
                    result = Some(r);
                    done   = true;
                }
            }
            if done {
                self.print_analysis_running  = false;
                self.print_analysis_receiver = None;
                if let Some(r) = result {
                    self.print_analysis_result       = Some(r);
                    self.print_overhang_needs_upload = true;
                    self.print_overhang_overlay      = true;
                }
            }
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }

        // Handle keyboard shortcuts
        if ctx.input(|i| i.key_pressed(egui::Key::F1)) {
            self.help_panel_open = !self.help_panel_open;
        }

        if ctx.input(|i| i.key_pressed(egui::Key::F5) || (i.modifiers.ctrl && i.key_pressed(egui::Key::R))) {
            self.execute_script();
        }

        // Save project (Ctrl+S)
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::S)) {
            self.save_project();
        }

        // Load project (Ctrl+O)
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::O)) {
            self.load_project();
        }

        // Undo (Ctrl+Z) — consume key so egui TextEdit does NOT handle it.
        let ctrl_z = ctx.input_mut(|i| {
            i.consume_key(egui::Modifiers::CTRL, egui::Key::Z) &&
            !i.modifiers.shift
        });
        if ctrl_z {
            self.undo_app();
        }

        // Redo (Ctrl+Y or Ctrl+Shift+Z)
        let ctrl_y = ctx.input_mut(|i| {
            i.consume_key(egui::Modifiers::CTRL, egui::Key::Y) ||
            (i.modifiers.ctrl && i.modifiers.shift &&
             i.consume_key(egui::Modifiers::CTRL | egui::Modifiers::SHIFT, egui::Key::Z))
        });
        if ctrl_y {
            self.redo_app();
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
                    if ui.button("New Project…").clicked() {
                        self.project_wizard.open = true;
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("New").clicked() {
                        self.state.script_text = Self::get_default_example();
                        self.current_sdf = None;
                        self.current_mesh = None;
                        self.current_sdf_grid = None;
                        self.current_file_path = None;
                        self.workflow_config = None;
                        self.undo_history.clear();
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

                // Edit menu
                ui.menu_button("Edit", |ui| {
                    let can_undo = self.undo_history.can_undo();
                    let undo_label = match self.undo_history.undo_description() {
                        Some(d) => format!("Undo: {}  Ctrl+Z", d),
                        None    => "Nothing to undo".to_owned(),
                    };
                    if ui.add_enabled(can_undo, egui::Button::new(undo_label)).clicked() {
                        self.undo_app();
                        ui.close_menu();
                    }
                    let can_redo = self.undo_history.can_redo();
                    let redo_label = match self.undo_history.redo_description() {
                        Some(d) => format!("Redo: {}  Ctrl+Shift+Z", d),
                        None    => "Nothing to redo".to_owned(),
                    };
                    if ui.add_enabled(can_redo, egui::Button::new(redo_label)).clicked() {
                        self.redo_app();
                        ui.close_menu();
                    }
                    ui.separator();
                    ui.menu_button("Undo History…", |ui| {
                        ui.set_min_width(280.0);
                        let descs: Vec<&str> = self.undo_history.past_descriptions(20).collect();
                        if descs.is_empty() {
                            ui.label(egui::RichText::new("(empty)").color(egui::Color32::GRAY));
                        }
                        for (i, d) in descs.iter().enumerate() {
                            let is_top = i == 0;
                            let text = egui::RichText::new(format!("{}  {}", if is_top { "▶" } else { "  " }, d));
                            ui.label(if is_top { text.strong() } else { text.color(egui::Color32::GRAY) });
                        }
                    });
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
                    ui.checkbox(&mut self.tolerance_on_export, "Apply tolerance compensation");
                    if !self.tolerance_on_export {
                        ui.colored_label(egui::Color32::from_rgb(200, 160, 40),
                            "⚠ Tolerance compensation not applied");
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
                    if ui.add_enabled(has_sdf, egui::Button::new("Export Package…")).clicked() {
                        if let Some(path) = rfd::FileDialog::new().pick_folder() {
                            self.start_export_package_async(path.to_string_lossy().to_string());
                        }
                        ui.close_menu();
                    }
                });

                // View menu
                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.smooth_normals, "Smooth Normals");
                    ui.checkbox(&mut self.show_wireframe, "Wireframe");
                    ui.separator();
                    for (label, view) in [
                        ("Front", StandardView::Front),
                        ("Back", StandardView::Back),
                        ("Left", StandardView::Left),
                        ("Right", StandardView::Right),
                        ("Top", StandardView::Top),
                        ("Bottom", StandardView::Bottom),
                        ("Isometric", StandardView::Isometric),
                    ] {
                        if ui.button(label).clicked() {
                            self.snap_camera_to_view(view);
                            ui.close_menu();
                        }
                    }
                    if ui.button("Frame Geometry").clicked() {
                        self.frame_current_geometry();
                        ui.close_menu();
                    }
                    if ui.button("Reset Camera  Home").clicked() {
                        self.camera.reset();
                        self.status_message = Some("Reset viewport camera".into());
                        ui.close_menu();
                    }
                });

                if ui.button("Settings").clicked() {
                    self.settings_open = true;
                }

                ui.separator();

                ui.label("Viewport");
                if ui.add(
                    egui::Slider::new(&mut self.resolution, 16..=256)
                        .logarithmic(true)
                        .text("Res")
                ).changed() && self.current_sdf.is_some() {
                    self.execute_script();
                }

                ui.separator();

                egui::ComboBox::from_id_salt("menu_mesh_quality")
                    .selected_text(format!("Mesh {}", self.mesh_quality.label()))
                    .show_ui(ui, |ui| {
                        for &q in MeshQuality::all() {
                            if ui.selectable_label(self.mesh_quality == q, q.label()).clicked() {
                                self.mesh_quality = q;
                            }
                        }
                    });

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

                let pa_label = if self.print_analysis_open { "🖨 Print ✓" } else { "🖨 Print" };
                if ui.button(pa_label).clicked() {
                    self.print_analysis_open = !self.print_analysis_open;
                }

                let aero_label = if self.aero_panel_open { "✈ Aero ✓" } else { "✈ Aero" };
                if ui.button(aero_label).clicked() {
                    self.aero_panel_open = !self.aero_panel_open;
                }

                ui.separator();

                // Version control indicator
                let vc_label = if self.version_control.detached_head {
                    egui::RichText::new("⚠ DETACHED HEAD")
                        .color(egui::Color32::from_rgb(255, 150, 50))
                } else if self.version_control.working_changes {
                    egui::RichText::new(format!("⌥ {}*", self.version_control.current_branch))
                        .color(egui::Color32::YELLOW)
                } else {
                    egui::RichText::new(format!("⌥ {}", self.version_control.current_branch))
                        .color(egui::Color32::GRAY)
                };
                ui.label(vc_label);
                if ui.small_button("VC").clicked() {
                    self.vc_panel_open = !self.vc_panel_open;
                }

                ui.separator();

                // Function reference help panel
                if ui.button("?").on_hover_text("Function Reference (F1)").clicked() {
                    self.help_panel_open = !self.help_panel_open;
                }

                ui.separator();

                // Quick-access run button in menu bar
                let run_label = if self.is_processing { "⏳ Running…" } else { "▶ Run  F5" };
                if ui.button(run_label).clicked() && !self.is_processing {
                    self.execute_script();
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
                if self.is_processing {
                    ui.spinner();
                    ui.label(egui::RichText::new(
                        self.processing_status.as_deref().unwrap_or("Evaluating…")
                    ).color(egui::Color32::from_rgb(100, 180, 255)));
                } else if self.export_in_progress {
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

        if let Some((instance, project_name, project_path)) = show_wizard(ctx, &mut self.project_wizard) {
            self.state.script_text = instance.script;
            self.state.dimensions = instance.dimensions;
            self.workflow_config = instance.workflow_config;
            if let Some(settings) = instance.print_analysis_settings {
                self.print_analysis_settings = settings;
            }
            if let Some(tol) = instance.tolerance_settings {
                self.tolerance_settings = tol;
                self.tolerance_preset = crate::sdf::print::TolerancePreset::StandardFDM;
            }
            self.current_file_path = None;
            self.current_sdf = None;
            self.current_mesh = None;
            self.current_sdf_grid = None;
            self.undo_history.clear();
            self.status_message = Some(format!("Initialized project '{}'", project_name));
            self.execute_script();

            if !project_path.trim().is_empty() {
                let save_dir = std::path::PathBuf::from(project_path.trim());
                let filename = format!(
                    "{}.icad",
                    project_name.replace(['\\', '/', ':', '*', '?', '\"', '<', '>', '|'], "_")
                );
                self.save_project_to_path(save_dir.join(filename));
            }
        }

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
                        if let Some((offset, line)) = find_name_in_script(&self.state.script_text, &name) {
                            self.pending_cursor_offset = Some(offset);
                            self.pending_scroll_line   = Some(line);
                        }
                    }

                    // Start rename on right-click
                    if let Some((node_id, old_name)) = interaction.start_rename {
                        let count = count_occurrences(&self.state.script_text, &old_name);
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
                        let new_script = rename_in_script(&self.state.script_text, &old_name, &new_name);
                        if new_script != self.state.script_text {
                            let cmd = Box::new(RenameCommand {
                                old_name:      old_name.clone(),
                                new_name:      new_name.clone(),
                                script_before: self.state.script_text.clone(),
                                script_after:  new_script.clone(),
                            });
                            self.state.script_text = new_script;
                            // Update profiles key if needed.
                            if let Some(profile) = self.state.profiles.remove(&old_name) {
                                self.state.profiles.insert(new_name.clone(), profile);
                            }
                            if self.state.active_profile.as_deref() == Some(&old_name) {
                                self.state.active_profile = Some(new_name);
                            }
                            self.undo_history.push_executed(cmd);
                            self.execute_script();
                        }
                    }

                    // ── Dimensions panel ─────────────────────────────────────
                    let dims_changed = crate::ui::dimensions::show_dimensions_panel(
                        ui,
                        &mut self.state,
                        &mut self.dimensions_state,
                        &mut self.undo_history,
                    );
                    if dims_changed {
                        if self.workflow_auto_apply_constraints {
                            self.apply_workflow_constraints();
                        }
                        self.dimensions_pending_eval = true;
                    }

                    let mut workflow_apply_constraints = false;
                    let mut workflow_run_checks = false;
                    let mut workflow_run_flight = false;
                    let mut variant_to_apply: Option<usize> = None;
                    let mut variant_to_delete: Option<usize> = None;
                    let mut save_variant = false;
                    let mut workflow_status_message: Option<String> = None;
                    if let Some(cfg) = &mut self.workflow_config {
                        ui.separator();
                        ui.collapsing("Workflow Config", |ui| {
                            ui.label(format!("Template: {}", cfg.template_id));
                            ui.label(format!("Vehicle: {}", cfg.vehicle_type));
                            ui.checkbox(&mut self.workflow_auto_apply_constraints, "Auto-apply assembly constraints");

                            let mut selected = cfg.manufacturing_preset.clone();
                            egui::ComboBox::from_id_salt("workflow_manufacturing_preset")
                                .selected_text(selected.label())
                                .show_ui(ui, |ui| {
                                    for preset in [
                                        crate::project::ManufacturingPreset::Foamboard,
                                        crate::project::ManufacturingPreset::LwPlaShell,
                                        crate::project::ManufacturingPreset::CarbonTubeSpar,
                                        crate::project::ManufacturingPreset::BalsaHybrid,
                                        crate::project::ManufacturingPreset::MoldedShell,
                                    ] {
                                        ui.selectable_value(&mut selected, preset.clone(), preset.label());
                                    }
                                });

                            if selected != cfg.manufacturing_preset {
                                cfg.manufacturing_preset = selected.clone();
                                for (name, value) in crate::ui::templates::preset_dimension_defaults(&selected) {
                                    self.state.dimensions.insert(name, value);
                                }
                                self.print_analysis_settings = crate::ui::templates::preset_print_analysis_settings(&selected);
                                self.tolerance_settings = crate::ui::templates::preset_tolerance_settings(&selected);
                                self.tolerance_preset = crate::sdf::print::TolerancePreset::StandardFDM;
                                workflow_apply_constraints = true;
                                self.dimensions_pending_eval = true;
                            }

                            if !cfg.parameter_groups.is_empty() {
                                ui.separator();
                                for (group, names) in &cfg.parameter_groups {
                                    ui.label(egui::RichText::new(group).small().strong());
                                    ui.label(
                                        egui::RichText::new(names.join(", "))
                                            .small()
                                            .color(egui::Color32::GRAY),
                                    );
                                }
                            }

                            if !cfg.assembly_constraints.is_empty() {
                                ui.separator();
                                ui.collapsing("Assembly Constraints", |ui| {
                                    if ui.button("Apply Constraints").clicked() {
                                        workflow_apply_constraints = true;
                                    }
                                    for constraint in &mut cfg.assembly_constraints {
                                        ui.horizontal(|ui| {
                                            ui.checkbox(&mut constraint.enabled, "");
                                            ui.label(&constraint.label);
                                            ui.label(
                                                egui::RichText::new(format!("-> {}", constraint.driven))
                                                    .small()
                                                    .color(egui::Color32::GRAY),
                                            );
                                        });
                                    }
                                });
                            }

                            ui.separator();
                            ui.collapsing("Workflow Checks", |ui| {
                                ui.horizontal(|ui| {
                                    if ui.button("Run Manufacturing Checks").clicked() {
                                        workflow_run_checks = true;
                                    }
                                    if ui.button("Run Flight Summary").clicked() {
                                        workflow_run_flight = true;
                                    }
                                });

                                let mfg = crate::analysis::workflow_summary::summarize_manufacturing(
                                    self.thickness_result.as_ref(),
                                    self.print_analysis_result.as_ref(),
                                    &self.print_analysis_settings,
                                );
                                let flight = crate::analysis::workflow_summary::summarize_flight(
                                    &self.mass_points,
                                    self.cg.map(|cg| cg.x),
                                    self.current_static_margin.as_ref(),
                                    self.current_trim_result.as_ref(),
                                    self.current_drag_polar.as_ref(),
                                    self.current_cg_sensitivity.as_ref(),
                                );
                                let status_color = |status| match status {
                                    crate::analysis::workflow_summary::SummaryStatus::Pass => egui::Color32::from_rgb(60, 180, 90),
                                    crate::analysis::workflow_summary::SummaryStatus::Warning => egui::Color32::from_rgb(220, 170, 60),
                                    crate::analysis::workflow_summary::SummaryStatus::Fail => egui::Color32::from_rgb(210, 70, 70),
                                };

                                ui.colored_label(status_color(mfg.status), format!("Manufacturing: {}", mfg.status.label()));
                                if let Some(min_wall) = mfg.min_wall_mm {
                                    ui.label(format!("Min wall: {:.2} mm", min_wall));
                                }
                                if let Some(area) = mfg.overhang_area_mm2 {
                                    ui.label(format!("Overhang area: {:.0} mm²", area));
                                }
                                if let Some(critical_area) = mfg.critical_overhang_area_mm2 {
                                    ui.label(format!("Critical overhang area: {:.0} mm²", critical_area));
                                }
                                if mfg.issue_errors > 0 || mfg.issue_warnings > 0 {
                                    ui.label(format!("Issues: {} errors, {} warnings", mfg.issue_errors, mfg.issue_warnings));
                                }
                                for note in mfg.notes.iter().take(2) {
                                    ui.label(note);
                                }

                                ui.separator();
                                ui.colored_label(status_color(flight.status), format!("Flight: {}", flight.status.label()));
                                ui.label(format!("Total component mass: {:.1} g", flight.total_mass_g));
                                if let Some(cg_x) = flight.cg_x_mm {
                                    ui.label(format!("CG x: {:.1} mm", cg_x));
                                }
                                if let Some(sm) = flight.static_margin_mac {
                                    ui.label(format!("Static margin: {:.1}% MAC", sm * 100.0));
                                }
                                if let Some(ld) = flight.best_glide_ratio {
                                    ui.label(format!("Best L/D: {:.1}", ld));
                                }
                                if let Some(speed) = flight.best_glide_speed_ms {
                                    ui.label(format!("Best glide speed: {:.1} m/s", speed));
                                }
                                if let Some(trim_aoa) = flight.trim_aoa_deg {
                                    ui.label(format!("Trim AoA: {:.1}°", trim_aoa));
                                }
                                for note in flight.notes.iter().take(2) {
                                    ui.label(note);
                                }
                            });

                            ui.separator();
                            ui.collapsing("Design Variants", |ui| {
                                ui.horizontal(|ui| {
                                    ui.label("Name");
                                    ui.text_edit_singleline(&mut self.workflow_variant_name);
                                });
                                ui.text_edit_singleline(&mut self.workflow_variant_description);
                                if ui.button("Save Current as Variant").clicked() {
                                    save_variant = true;
                                }
                                for (idx, variant) in cfg.variants.iter().enumerate() {
                                    ui.horizontal(|ui| {
                                        ui.label(&variant.name);
                                        if !variant.description.is_empty() {
                                            ui.label(
                                                egui::RichText::new(&variant.description)
                                                    .small()
                                                    .color(egui::Color32::GRAY),
                                            );
                                        }
                                        if ui.small_button("Apply").clicked() {
                                            variant_to_apply = Some(idx);
                                        }
                                        if ui.small_button("Delete").clicked() {
                                            variant_to_delete = Some(idx);
                                        }
                                    });
                                }
                            });
                        });
                    }
                    if workflow_apply_constraints {
                        let applied = self.apply_workflow_constraints();
                        workflow_status_message = Some(format!("Applied {} assembly constraints", applied));
                    }
                    if workflow_run_checks {
                        if self.thickness_result.is_none() {
                            self.start_thickness_analysis();
                        }
                        self.start_print_analysis();
                        workflow_status_message = Some("Started manufacturing checks".into());
                    }
                    if workflow_run_flight {
                        self.run_workflow_flight_analysis();
                        workflow_status_message = Some("Updated workflow flight summary".into());
                    }
                    if let Some(idx) = variant_to_apply {
                        let selected_variant = self.workflow_config.as_ref()
                            .and_then(|cfg| cfg.variants.get(idx).cloned());
                        if let Some(variant) = selected_variant {
                            self.state.dimensions = variant.dimensions.clone();
                            if self.workflow_auto_apply_constraints {
                                self.apply_workflow_constraints();
                            }
                            self.dimensions_pending_eval = true;
                            workflow_status_message = Some(format!("Applied variant '{}'", variant.name));
                        }
                    }
                    if save_variant {
                        if let Some(cfg) = &mut self.workflow_config {
                            let name = self.workflow_variant_name.trim();
                            if !name.is_empty() {
                                cfg.variants.retain(|v| v.name != name);
                                cfg.variants.push(crate::project::DesignVariant {
                                    name: name.to_string(),
                                    description: self.workflow_variant_description.trim().to_string(),
                                    dimensions: self.state.dimensions.clone(),
                                });
                                workflow_status_message = Some(format!("Saved variant '{}'", name));
                            }
                        }
                    }
                    if let Some(idx) = variant_to_delete {
                        if let Some(cfg) = &mut self.workflow_config {
                            if idx < cfg.variants.len() {
                                let removed = cfg.variants.remove(idx);
                                workflow_status_message = Some(format!("Deleted variant '{}'", removed.name));
                            }
                        }
                    }
                    if let Some(msg) = workflow_status_message {
                        self.status_message = Some(msg);
                    }

                    // ── Imported meshes ───────────────────────────────────────
                    let cache_guard = self.mesh_cache.lock().unwrap();
                    if !cache_guard.is_empty() {
                        ui.separator();
                        ui.collapsing("📦 Meshes", |ui| {
                            for (path, (_mtime, mesh)) in cache_guard.iter() {
                                let filename = path.file_name()
                                    .and_then(|n| n.to_str())
                                    .unwrap_or("(unknown)");
                                use crate::mesh::import::validate_mesh;
                                let v = validate_mesh(mesh);
                                let icon = if v.has_open_boundary { "🔴" }
                                           else if v.has_non_manifold { "🟡" }
                                           else { "🟢" };
                                ui.label(format!(
                                    "{} {} — {} tri",
                                    icon, filename, mesh.triangle_count()
                                ));
                            }
                        });
                    }
                    drop(cache_guard);

                    // ── Composite layup summary ───────────────────────────────
                    // ── Library panel ─────────────────────────────────────────
                    if let Some(ref mut mgr) = self.library_manager {
                        ui.separator();
                        let result = show_library_panel(
                            ui,
                            ctx,
                            &mut self.library_panel_state,
                            mgr,
                        );
                        if let Some(action) = result {
                            if action == "__NEW_COMPONENT__" {
                                // Create new component with a default name
                                match mgr.create_new_component("new_component") {
                                    Ok(path) => {
                                        self.library_panel_state.editing_library = Some(path);
                                    }
                                    Err(e) => {
                                        self.error_message = Some(format!("Failed to create component: {}", e));
                                    }
                                }
                            } else {
                                // Insert snippet into script
                                self.state.script_text.push_str(&action);
                                self.execute_script();
                            }
                        }
                    }

                    if !self.current_layups.is_empty() {
                        ui.separator();
                        ui.collapsing("🧱 Layup Summary", |ui| {
                            for (li, layup) in self.current_layups.iter().enumerate() {
                                ui.strong(format!("Layup {}", li + 1));
                                let total_t: f32 = layup.layers.iter().map(|l| l.thickness).sum();
                                ui.label(format!(
                                    "{} layers, {:.2} mm total wall",
                                    layup.layers.len(), total_t
                                ));
                                egui::Grid::new(format!("layup_grid_{}", li))
                                    .num_columns(3)
                                    .striped(true)
                                    .show(ui, |ui| {
                                        ui.strong("Layer");
                                        ui.strong("Material");
                                        ui.strong("t (mm)");
                                        ui.end_row();
                                        for layer in &layup.layers {
                                            let label = if layer.is_core { "⬛ core" } else { "▪" };
                                            ui.label(format!("{} {}", label, &layer.name));
                                            ui.label(&layer.material.name);
                                            ui.label(format!("{:.2}", layer.thickness));
                                            ui.end_row();
                                        }
                                    });
                            }
                        });
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
                if let Some(ref profile_name) = self.state.active_profile.clone() {
                    ui.horizontal(|ui| {
                        ui.strong(format!("✏ Profile: {}", profile_name));
                        if ui.button("✕ Close").clicked() {
                            self.state.active_profile = None;
                            self.execute_script();
                        }
                    });
                    ui.separator();

                    let state = self.state.profiles
                        .entry(profile_name.clone())
                        .or_insert_with(SplineEditorState::default);
                    let spline_before = state.clone();

                    if show_spline_editor(ui, state, profile_name) {
                        // Push undo command for the spline change.
                        let after = state.clone();
                        self.undo_history.push_executed(Box::new(SplineShapeResetCommand {
                            profile_name: profile_name.clone(),
                            before:       spline_before,
                            after,
                            desc:         "Edit profile".to_owned(),
                        }));
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

                    let spines_before = self.state.splines.clone();
                    let mut editor_state = std::mem::take(&mut self.spine_editor_state);
                    let changed = show_spine_editor(
                        ui,
                        &mut editor_state,
                        &mut self.state.splines,
                        self.spine_fuselage_length,
                    );
                    self.spine_editor_state = editor_state;
                    if changed {
                        let spines_after = self.state.splines.clone();
                        self.undo_history.push_executed(Box::new(
                            LongitudinalSpineEditCommand::new(spines_before, spines_after)
                        ));
                        self.execute_script();
                    }
                    return;
                }

                // ── Script Mode ───────────────────────────────────────────────

                // File operations
                if !self.component_preview_parts.is_empty() && self.active_example_tab.is_none() {
                    ui.group(|ui| {
                        ui.strong("Preview Parts");
                        ui.horizontal(|ui| {
                            ui.label("Mode:");
                            let changed_highlight = ui.selectable_value(
                                &mut self.component_selection_mode,
                                ComponentSelectionMode::Highlight,
                                "Highlight",
                            ).changed();
                            let changed_isolate = ui.selectable_value(
                                &mut self.component_selection_mode,
                                ComponentSelectionMode::Isolate,
                                "Isolate",
                            ).changed();
                            if changed_highlight || changed_isolate {
                                self.set_component_preview_selection(self.selected_component_part.clone(), false);
                            }
                        });
                        ui.separator();
                        ui.horizontal_wrapped(|ui| {
                            let show_all = self.selected_component_part.is_none();
                            if ui.selectable_label(show_all, "Show All").clicked() {
                                self.set_component_preview_selection(None, false);
                            }
                        });
                        ui.separator();
                        ui.label(egui::RichText::new("preview_parts() / component()").strong());
                        ui.indent("component_parts_tree", |ui| {
                            for (name, _) in self.component_preview_parts.clone() {
                                let selected = self.selected_component_part.as_deref() == Some(name.as_str());
                                if ui.selectable_label(selected, &name).clicked() {
                                    self.set_component_preview_selection(Some(name.clone()), true);
                                }
                            }
                        });
                    });
                    ui.separator();
                }

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

                // ── Import Mesh button ─────────────────────────────────────
                if ui.button("📂 Import Mesh…").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("3D Mesh", &["stl", "obj"])
                        .pick_file()
                    {
                        let path_str = path.to_string_lossy().replace('\\', "/");
                        let snippet  = format!("import_mesh(\"{}\")", path_str);
                        if !self.state.script_text.is_empty()
                            && !self.state.script_text.ends_with('\n')
                        {
                            self.state.script_text.push('\n');
                        }
                        self.state.script_text.push_str(&snippet);
                    }
                }

                // ── Spline profiles ────────────────────────────────────────
                ui.collapsing("✏ Profiles", |ui| {
                    // List existing profiles
                    let names: Vec<String> = self.state.profiles.keys().cloned().collect();
                    for name in &names {
                        ui.horizontal(|ui| {
                            if ui.button("Edit").clicked() {
                                self.state.active_profile = Some(name.clone());
                            }
                            if ui.label(name).secondary_clicked() {
                                // Right-click label → delete (future: context menu)
                            }
                            if ui.small_button("✕").clicked() {
                                self.state.profiles.remove(name);
                                self.sync_profiles_shared();
                                if self.state.active_profile.as_deref() == Some(name.as_str()) {
                                    self.state.active_profile = None;
                                }
                            }
                            if ui.small_button("Insert").clicked() {
                                let snippet = format!("spline_section(\"{}\")", name);
                                if !self.state.script_text.is_empty() && !self.state.script_text.ends_with('\n') {
                                    self.state.script_text.push('\n');
                                }
                                self.state.script_text.push_str(&snippet);
                            }
                        });
                    }
                    ui.separator();
                    if ui.button("+ New Profile").clicked() {
                        let name = format!("profile{}", self.state.profiles.len() + 1);
                        self.state.profiles.insert(name.clone(), SplineEditorState::default());
                        self.sync_profiles_shared();
                        self.state.active_profile = Some(name);
                    }
                });

                // Examples and Insert dropdowns
                ui.horizontal(|ui| {
                    ui.label("Examples:");
                    egui::ComboBox::from_id_salt("examples_browser")
                        .selected_text("Browse Examples...")
                        .width(200.0)
                        .show_ui(ui, |ui| {
                            ui.add(
                                egui::TextEdit::singleline(&mut self.example_search)
                                    .hint_text("Search...")
                                    .desired_width(f32::INFINITY),
                            );
                            ui.separator();

                            let search_lower = self.example_search.to_lowercase();
                            let all_examples = examples::get_examples();

                            for cat in examples::CATEGORIES {
                                let cat_examples: Vec<(usize, &examples::ExampleScript)> = all_examples
                                    .iter()
                                    .enumerate()
                                    .filter(|(_, e)| e.category == *cat)
                                    .filter(|(_, e)| {
                                        if search_lower.is_empty() { return true; }
                                        e.title.to_lowercase().contains(&search_lower)
                                            || e.description.to_lowercase().contains(&search_lower)
                                            || e.tags.iter().any(|t| t.to_lowercase().contains(&search_lower))
                                    })
                                    .collect();

                                if cat_examples.is_empty() { continue; }

                                ui.label(
                                    egui::RichText::new(*cat)
                                        .strong()
                                        .color(egui::Color32::from_rgb(150, 150, 200)),
                                );

                                for (idx, ex) in cat_examples {
                                    let diff_color = match ex.difficulty {
                                        examples::Difficulty::Beginner     => egui::Color32::from_rgb(80, 180, 80),
                                        examples::Difficulty::Intermediate => egui::Color32::from_rgb(200, 180, 50),
                                        examples::Difficulty::Advanced     => egui::Color32::from_rgb(200, 80, 80),
                                    };
                                    let maturity = ex.maturity();
                                    let maturity_color = match maturity {
                                        examples::Maturity::Stable => egui::Color32::from_rgb(90, 160, 220),
                                        examples::Maturity::Experimental => egui::Color32::from_rgb(220, 150, 60),
                                        examples::Maturity::Legacy => egui::Color32::from_rgb(150, 150, 150),
                                    };
                                    let already_open = self.open_example_tabs.contains(&idx);
                                    ui.horizontal(|ui| {
                                        ui.colored_label(diff_color, "●");
                                        ui.colored_label(maturity_color, format!("[{}]", maturity.label()));
                                        if ui.selectable_label(
                                            already_open,
                                            format!("{} — {}", ex.title, ex.description),
                                        ).clicked() {
                                            if !already_open {
                                                self.open_example_tabs.push(idx);
                                            }
                                            self.active_example_tab = Some(idx);
                                            ui.close_menu();
                                        }
                                    });
                                }
                                ui.separator();
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
                                    if !self.state.script_text.is_empty() && !self.state.script_text.ends_with('\n') {
                                        self.state.script_text.push('\n');
                                    }
                                    self.state.script_text.push_str(code);
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
                // Undo/redo feedback (shown 3 s after action).
                if let Some((msg, when)) = &self.undo_feedback {
                    if when.elapsed() < std::time::Duration::from_secs(3) {
                        ui.label(egui::RichText::new(msg).italics()
                            .color(egui::Color32::GRAY).size(12.0));
                        ctx.request_repaint_after(std::time::Duration::from_millis(500));
                    } else {
                        self.undo_feedback = None;
                    }
                }

                ui.separator();

                // ── Example tab bar ──────────────────────────────────────────────
                {
                    // Handle pending copy-to-main from context menu (outside borrow).
                    if let Some(copy_idx) = self.pending_copy_example.take() {
                        if let Some(ex) = examples::get_examples().get(copy_idx) {
                            self.state.script_text = ex.script.to_string();
                            self.active_example_tab = None;
                            self.status_message = Some("Example copied to main script".to_string());
                        }
                    }

                    ui.horizontal(|ui| {
                        // Main Script tab
                        let main_active = self.active_example_tab.is_none();
                        let main_label = egui::RichText::new("Main Script");
                        let main_label = if main_active { main_label.strong() } else { main_label };
                        if ui.selectable_label(main_active, main_label).clicked() {
                            self.active_example_tab = None;
                        }

                        // Example tabs
                        let mut to_close: Option<usize> = None;
                        let mut to_activate: Option<usize> = None;
                        let mut to_copy: Option<usize> = None;

                        let tab_indices: Vec<usize> = self.open_example_tabs.clone();
                        for tab_idx in tab_indices {
                            let is_active = self.active_example_tab == Some(tab_idx);
                            let title = examples::get_examples()
                                .get(tab_idx)
                                .map(|e| e.title)
                                .unwrap_or("?");

                            ui.separator();

                            let tab_resp = ui.selectable_label(
                                is_active,
                                egui::RichText::new(title),
                            );
                            if tab_resp.clicked() {
                                to_activate = Some(tab_idx);
                            }

                            // Right-click context menu
                            tab_resp.context_menu(|ui| {
                                if ui.button("Copy to Main Script").clicked() {
                                    to_copy = Some(tab_idx);
                                    ui.close_menu();
                                }
                                if ui.button("Close Tab").clicked() {
                                    to_close = Some(tab_idx);
                                    ui.close_menu();
                                }
                            });

                            // × close button
                            if ui.small_button("×").clicked() {
                                to_close = Some(tab_idx);
                            }
                        }

                        if let Some(idx) = to_activate {
                            self.active_example_tab = Some(idx);
                        }
                        if let Some(idx) = to_copy {
                            self.pending_copy_example = Some(idx);
                        }
                        if let Some(idx) = to_close {
                            self.open_example_tabs.retain(|&i| i != idx);
                            if self.active_example_tab == Some(idx) {
                                self.active_example_tab = None;
                            }
                        }
                    });
                    ui.separator();
                }

                // Multiline text editor with stats
                ui.label(egui::RichText::new(format!("Lines: {} | Characters: {}",
                    self.state.script_text.lines().count(),
                    self.state.script_text.len()))
                    .size(11.0)
                    .color(egui::Color32::GRAY));

                // ── Cell-based editor (when script has multiple delimited sections) ──
                let use_cell_view = self.script_cells.len() > 1;

                if use_cell_view {
                    // Collect cell data for rendering (we must avoid borrowing self.script_cells
                    // while also mutating self.state.script_text and self.script_cells).
                    let cell_count = self.script_cells.len();
                    let mut needs_eval = false;

                    egui::ScrollArea::vertical().show(ui, |ui| {
                        for cell_idx in 0..cell_count {
                            let cell_id = self.script_cells[cell_idx].id.clone();
                            let cell_name = self.script_cells[cell_idx].name.clone();
                            let cell_start = self.script_cells[cell_idx].start_line;
                            let cell_end   = self.script_cells[cell_idx].end_line;
                            let cell_status = self.script_cells[cell_idx].status.clone();
                            let is_skipped = cell_status == scripting::CellStatus::Skipped;

                            // ── Header bar ──────────────────────────────────────
                            ui.horizontal(|ui| {
                                // Status dot.
                                let (dot_color, dot_tip) = match &cell_status {
                                    scripting::CellStatus::Ok      => (egui::Color32::from_rgb(80, 200, 80),  "OK"),
                                    scripting::CellStatus::Error{..}=> (egui::Color32::from_rgb(220, 60, 60),  "Error"),
                                    scripting::CellStatus::Skipped => (egui::Color32::GRAY,                   "Skipped"),
                                    scripting::CellStatus::Pending => (egui::Color32::from_rgb(210, 150, 40), "Pending"),
                                };
                                ui.label(egui::RichText::new("●").color(dot_color))
                                    .on_hover_text(dot_tip);

                                // Editable cell name.
                                let mut name_buf = cell_name.clone();
                                let name_resp = ui.add(
                                    egui::TextEdit::singleline(&mut name_buf)
                                        .desired_width(160.0)
                                        .font(egui::TextStyle::Monospace),
                                );
                                if name_resp.changed() && !name_buf.is_empty() {
                                    // Rewrite delimiter line in script_text.
                                    let new_delim = format!("# === {} ===", name_buf.trim());
                                    let mut lines: Vec<String> = self.state.script_text
                                        .lines().map(|l| l.to_string()).collect();
                                    if cell_start < lines.len() {
                                        lines[cell_start] = new_delim;
                                    }
                                    self.state.script_text = lines.join("\n");
                                    // Re-parse cells (no re-eval).
                                    self.script_cells = scripting::parse_cells(&self.state.script_text);
                                }
                            });

                            // ── Code region ──────────────────────────────────────
                            // Extract cell content lines (excluding delimiter).
                            let lines: Vec<String> = self.state.script_text
                                .lines().map(|l| l.to_string()).collect();

                            // Content range: skip delimiter line.
                            let content_start = if cell_start < lines.len()
                                && scripting::is_delimiter_line(&lines[cell_start])
                            {
                                cell_start + 1
                            } else {
                                cell_start
                            };
                            let content_end_idx = cell_end.min(lines.len().saturating_sub(1));

                            let mut cell_text = if content_start <= content_end_idx {
                                lines[content_start..=content_end_idx].join("\n")
                            } else {
                                String::new()
                            };

                            let script_before = self.state.script_text.clone();
                            ui.add_enabled_ui(!is_skipped, |ui| {
                                let mut cell_layouter = |ui: &egui::Ui, text: &str, wrap_width: f32| {
                                    let font_id = ui.style()
                                        .text_styles[&egui::TextStyle::Monospace]
                                        .clone();
                                    let mut job = crate::ui::syntax::highlight_script(text, font_id);
                                    job.wrap.max_width = wrap_width;
                                    ui.fonts(|f| f.layout_job(job))
                                };
                                let resp = ui.add(
                                    egui::TextEdit::multiline(&mut cell_text)
                                        .id(egui::Id::new(format!("cell_editor_{}", cell_id)))
                                        .desired_width(f32::INFINITY)
                                        .desired_rows(4)
                                        .code_editor()
                                        .font(egui::TextStyle::Monospace)
                                        .layouter(&mut cell_layouter),
                                );
                                if resp.changed() {
                                    // Reconstruct full script_text.
                                    let mut all_lines: Vec<String> = script_before
                                        .lines().map(|l| l.to_string()).collect();
                                    let new_cell_lines: Vec<String> = cell_text
                                        .lines().map(|l| l.to_string()).collect();
                                    // Replace lines content_start..=content_end_idx.
                                    let before: Vec<String> = all_lines[..content_start].to_vec();
                                    let after_start = (content_end_idx + 1).min(all_lines.len());
                                    let after: Vec<String> = all_lines[after_start..].to_vec();
                                    all_lines = before;
                                    all_lines.extend(new_cell_lines);
                                    all_lines.extend(after);
                                    self.state.script_text = all_lines.join("\n");
                                    self.script_cells = scripting::parse_cells(&self.state.script_text);
                                    self.undo_history.push_executed(Box::new(
                                        ScriptTextCommand::new(script_before.clone(), self.state.script_text.clone())
                                    ));
                                    needs_eval = true;
                                }
                            });

                            // ── Error display ────────────────────────────────────
                            if let scripting::CellStatus::Error { message, .. } = &cell_status {
                                ui.colored_label(egui::Color32::from_rgb(220, 60, 60),
                                    format!("⚠ {}", message));
                            }

                            // ── Add Section button ────────────────────────────────
                            if ui.small_button("+ Add Section").clicked() {
                                let mut all_lines: Vec<String> = self.state.script_text
                                    .lines().map(|l| l.to_string()).collect();
                                let insert_at = (cell_end + 1).min(all_lines.len());
                                // Insert new delimiter after cell_end.
                                all_lines.insert(insert_at, "# === New Section ===".to_string());
                                self.state.script_text = all_lines.join("\n");
                                self.script_cells = scripting::parse_cells(&self.state.script_text);
                                needs_eval = true;
                            }

                            ui.separator();
                        }
                    });

                    if needs_eval {
                        self.execute_script();
                    }
                } else if let Some(ex_idx) = self.active_example_tab {
                // ── Read-only example tab view ────────────────────────────────────
                if let Some(ex) = examples::get_examples().get(ex_idx) {
                    let maturity = ex.maturity();
                    let maturity_color = match maturity {
                        examples::Maturity::Stable => egui::Color32::from_rgb(90, 160, 220),
                        examples::Maturity::Experimental => egui::Color32::from_rgb(220, 150, 60),
                        examples::Maturity::Legacy => egui::Color32::from_rgb(150, 150, 150),
                    };
                    ui.label(
                        egui::RichText::new("Read-only example — right-click tab to copy to main script")
                            .color(egui::Color32::GRAY)
                            .size(11.0),
                    );
                    ui.colored_label(
                        maturity_color,
                        format!("Feature status: {}", maturity.label()),
                    );
                    // Related functions
                    if !ex.related_functions.is_empty() {
                        ui.horizontal_wrapped(|ui| {
                            ui.label(egui::RichText::new("Functions:").size(10.0).color(egui::Color32::GRAY));
                            for fn_name in ex.related_functions {
                                if ui.small_button(*fn_name).clicked() {
                                    self.help_state.query = fn_name.to_string();
                                    self.help_state.update_results();
                                    self.help_panel_open = true;
                                }
                            }
                        });
                    }
                    let mut example_text = ex.script.to_string();
                    egui::ScrollArea::vertical()
                        .id_salt("example_editor")
                        .show(ui, |ui| {
                            ui.add(
                                egui::TextEdit::multiline(&mut example_text)
                                    .font(egui::TextStyle::Monospace)
                                    .desired_rows(30)
                                    .desired_width(f32::INFINITY)
                                    .code_editor(),
                            );
                        });
                }

                } else {
                // ── Single-cell / no-delimiter plain editor (existing code) ────

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

                // Snapshot text before editing so we can push an undo command if it changes.
                let script_before_edit = self.state.script_text.clone();

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
                        egui::TextEdit::multiline(&mut self.state.script_text)
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

                    // Push undo command whenever text changes (coalesced within 800ms).
                    if response.changed() {
                        let after = self.state.script_text.clone();
                        self.undo_history.push_executed(Box::new(
                            ScriptTextCommand::new(script_before_edit.clone(), after)
                        ));
                        // Re-detect script variables immediately so sliders update while typing.
                        self.dimensions_state.detected_variables =
                            crate::ui::script_variable_detector::detect_script_variables(&self.state.script_text);
                    }

                    // Update autocomplete from cursor position.
                    if let Some(te_state) = egui::text_edit::TextEditState::load(ctx, response.id) {
                        if let Some(range) = te_state.cursor.char_range() {
                            let cursor_offset = range.primary.index;
                            // cursor_offset is a CHARACTER index; convert to byte offset for string slicing.
                            let cursor_byte = self.state.script_text.char_indices()
                                .nth(cursor_offset)
                                .map(|(b, _)| b)
                                .unwrap_or(self.state.script_text.len());

                            // Apply confirmed autocomplete from previous frame.
                            if ac_confirm && self.autocomplete_state.visible {
                                let sel_list_i = self.autocomplete_state.selected_index;
                                if let Some(&global_i) = self.autocomplete_state.match_indices.get(sel_list_i) {
                                    let new_cursor = crate::ui::autocomplete::apply_completion(
                                        &mut self.state.script_text,
                                        self.autocomplete_state.token_start,
                                        cursor_byte,
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
                                let cursor_line = self.state.script_text[..cursor_byte]
                                    .chars().filter(|&c| c == '\n').count();
                                let anchor = egui::pos2(
                                    response.rect.min.x + 20.0,
                                    editor_top + (cursor_line + 1) as f32 * line_height
                                        - self.last_editor_scroll_y,
                                );
                                crate::ui::autocomplete::update_completions(
                                    &mut self.autocomplete_state,
                                    &self.state.script_text,
                                    cursor_offset,
                                    anchor,
                                );
                            }

                            // Signature tooltip anchor (above cursor line).
                            let cursor_line = self.state.script_text[..cursor_byte]
                                .chars().filter(|&c| c == '\n').count();
                            let tip_anchor = egui::pos2(
                                response.rect.min.x + 20.0,
                                editor_top + cursor_line as f32 * line_height
                                    - self.last_editor_scroll_y - 28.0,
                            );
                            crate::ui::autocomplete::show_signature_tooltip(
                                ctx,
                                &self.state.script_text,
                                cursor_offset,
                                tip_anchor,
                            );
                        }
                    }

                    response
                });

                self.last_editor_scroll_y = scroll_out.state.offset.y;

                // Phase 31 — draw margin indicator dots for detected script variables.
                {
                    const LINE_HEIGHT: f32 = 14.0;
                    let scroll_y = self.last_editor_scroll_y;
                    let editor_rect = scroll_out.inner_rect;
                    // Only draw if the editor has been laid out (non-zero size).
                    if editor_rect.width() > 1.0 && editor_rect.height() > 1.0 {
                        let dot_x = editor_rect.min.x - 6.0;
                        let painter = ui.painter();
                        for var in &self.dimensions_state.detected_variables {
                            let dot_y = editor_rect.min.y + (var.line as f32 + 0.5) * LINE_HEIGHT - scroll_y;
                            if dot_y >= editor_rect.min.y && dot_y <= editor_rect.max.y {
                                let color = match &var.detection_type {
                                    crate::ui::DetectionType::LetBinding { .. } =>
                                        egui::Color32::from_rgb(100, 150, 255),
                                    crate::ui::DetectionType::InlineLiteral { .. } =>
                                        egui::Color32::from_rgb(120, 120, 120),
                                };
                                painter.circle_filled(egui::pos2(dot_x, dot_y), 3.0, color);
                            }
                        }
                    }
                }

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
                        &mut self.state.script_text,
                        self.autocomplete_state.token_start,
                        cursor_approx,
                        &crate::ui::autocomplete::ALL_COMPLETIONS[global_i],
                    );
                    // Request a cursor update on next frame via pending_cursor_offset.
                    self.pending_cursor_offset = Some(new_cursor);
                    self.autocomplete_state.visible = false;
                }

                } // end single-cell else branch

                ui.separator();

                // Rendering options
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut self.smooth_normals, "Smooth Normals").changed() {
                        if self.current_sdf.is_some() {
                            self.execute_script();
                        }
                    }
                    ui.checkbox(&mut self.show_wireframe, "Show Wireframe");
                    ui.checkbox(&mut self.show_ref_points, "Ref Points");
                    ui.checkbox(&mut self.show_dim_lines, "Dim Lines");
                });

                // Run button and status
                ui.horizontal(|ui| {
                    if ui.button("Run (F5)").clicked() {
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
                        if ui.button("Copy").on_hover_text("Copy error to clipboard").clicked() {
                            ui.output_mut(|o| o.copied_text = error.clone());
                        }
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

        // Print Analysis floating window
        if self.print_analysis_open {
            let mut open = true;
            egui::Window::new("🖨 Print Analysis")
                .open(&mut open)
                .resizable(true)
                .default_width(300.0)
                .show(ctx, |ui| {
                    self.render_print_analysis_panel(ui);
                });
            self.print_analysis_open = open;
        }

        // Aerodynamic analysis floating window
        if self.aero_panel_open {
            let mut open = true;
            egui::Window::new("✈ Aero Analysis")
                .open(&mut open)
                .resizable(true)
                .default_width(320.0)
                .show(ctx, |ui| {
                    self.render_aero_panel(ui);
                });
            self.aero_panel_open = open;
        }

        // Version control panel (floating window)
        if self.vc_panel_open {
            let mut needs_eval = false;
            crate::ui::version_control_panel::show_vc_panel(
                ctx,
                &mut self.vc_panel_open,
                &mut self.version_control,
                &mut self.state,
                &mut self.vc_panel_state,
                &mut self.vc_commit_message,
                &mut self.vc_new_branch_name,
                &mut self.vc_new_branch_desc,
                &mut self.vc_new_branch_dialog,
                &mut self.vc_discard_confirm,
                &mut needs_eval,
            );
            if needs_eval {
                self.execute_script();
            }
        }

        // Help / function reference panel (floating window)
        crate::ui::help_panel::show_help_panel(
            ctx,
            &mut self.help_panel_open,
            &mut self.help_state,
            &mut self.state.script_text,
            Some(self.script_cursor_byte),
            false,
        );

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
                        self.state.script_text = Self::terminate_last_expression(self.state.script_text.clone());
                        if !self.state.script_text.is_empty() && !self.state.script_text.ends_with('\n') {
                            self.state.script_text.push('\n');
                        }
                        self.state.script_text.push_str(&script);
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
            script_template: self.state.script_text.clone(),
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

        response.context_menu(|ui| {
            self.show_viewport_view_menu(ui);
        });

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
        else if response.clicked() && !self.is_dragging_left && !self.component_preview_parts.is_empty() {
            if let (Some(click_pos), Some(grid)) = (response.interact_pointer_pos(), &self.current_sdf_grid) {
                if let Some(hit) = self.unproject_and_march(click_pos, rect, grid.as_ref()) {
                    let mut best_name: Option<String> = None;
                    let mut best_d = f32::INFINITY;
                    for (name, sdf) in &self.component_preview_parts {
                        let d = sdf.distance(hit).abs();
                        if d < best_d {
                            best_d = d;
                            best_name = Some(name.clone());
                        }
                    }
                    if let Some(name) = best_name {
                        self.set_component_preview_selection(Some(name), true);
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
        // Overhang overlay takes priority when active.
        let thickness_upload: Option<(Arc<Vec<f32>>, u32)> = if self.print_overhang_needs_upload && self.print_overhang_overlay {
            self.print_analysis_result.as_ref().map(|r| {
                (Arc::new(r.overhang.overhang_grid.clone()), r.overhang.resolution)
            })
        } else if self.thickness_needs_upload {
            self.thickness_result.as_ref().map(|r| (Arc::clone(&r.analysis_grid), r.resolution))
        } else {
            None
        };
        if self.print_overhang_needs_upload { self.print_overhang_needs_upload = false; }
        if self.thickness_needs_upload { self.thickness_needs_upload = false; }
        let clear_thickness = !self.thickness_overlay_on && self.thickness_result.is_none()
            && !self.print_overhang_overlay
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

            if self.component_selection_mode == ComponentSelectionMode::Highlight {
                if let Some((bmin, bmax)) = self.selected_component_bounds {
                    Self::draw_world_bbox(
                        &painter,
                        bmin,
                        bmax,
                        egui::Color32::from_rgb(255, 170, 40),
                        &self.camera,
                        rect,
                    );
                }
            }

            self.draw_view_cube(ui, rect);

            // Draw reference point overlays.
            if self.show_ref_points && !self.current_ref_points.is_empty() {
                let ref_pts: Vec<_> = self.current_ref_points.iter()
                    .filter_map(|pt| {
                        Self::project_to_screen(pt.position, &self.camera, rect)
                            .map(|s| (pt, s))
                    })
                    .collect();

                for (pt, screen) in &ref_pts {
                    let color = egui::Color32::from_rgb(
                        (pt.color[0] * 255.0) as u8,
                        (pt.color[1] * 255.0) as u8,
                        (pt.color[2] * 255.0) as u8,
                    );
                    painter.circle_filled(*screen, 5.0, color);
                    painter.circle_stroke(*screen, 5.0, egui::Stroke::new(1.0, egui::Color32::WHITE));
                    painter.text(
                        *screen + egui::Vec2::new(8.0, -8.0),
                        egui::Align2::LEFT_BOTTOM,
                        &pt.name,
                        egui::FontId::proportional(11.0),
                        color,
                    );
                }

                // Dimension lines between nearby pairs.
                if self.show_dim_lines {
                    for i in 0..ref_pts.len() {
                        for j in (i + 1)..ref_pts.len() {
                            let dist_mm = (ref_pts[i].0.position - ref_pts[j].0.position).length();
                            if dist_mm < 200.0 {
                                painter.line_segment(
                                    [ref_pts[i].1, ref_pts[j].1],
                                    egui::Stroke::new(1.0, egui::Color32::from_gray(140)),
                                );
                                let mid = (ref_pts[i].1.to_vec2() + ref_pts[j].1.to_vec2()) * 0.5;
                                painter.text(
                                    mid.to_pos2(),
                                    egui::Align2::CENTER_CENTER,
                                    format!("{:.1}mm", dist_mm),
                                    egui::FontId::proportional(10.0),
                                    egui::Color32::from_gray(180),
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    fn show_viewport_view_menu(&mut self, ui: &mut egui::Ui) {
        ui.label("Viewport Views");
        ui.separator();

        for (label, view) in [
            ("Front", StandardView::Front),
            ("Back", StandardView::Back),
            ("Left", StandardView::Left),
            ("Right", StandardView::Right),
            ("Top", StandardView::Top),
            ("Bottom", StandardView::Bottom),
            ("Isometric", StandardView::Isometric),
        ] {
            if ui.button(label).clicked() {
                self.snap_camera_to_view(view);
                ui.close_menu();
            }
        }

        ui.separator();
        if ui.button("Frame Selection").clicked() {
            self.frame_current_geometry();
            ui.close_menu();
        }
        if ui.button("Reset Camera").clicked() {
            self.camera.reset();
            self.status_message = Some("Reset viewport camera".into());
            ui.close_menu();
        }
    }

    fn draw_view_cube(&mut self, ui: &mut egui::Ui, viewport_rect: egui::Rect) {
        let panel_size = egui::vec2(124.0, 136.0);
        let panel_rect = egui::Rect::from_min_size(
            egui::pos2(viewport_rect.left() + 14.0, viewport_rect.bottom() - panel_size.y - 14.0),
            panel_size,
        );
        let painter = ui.painter().with_clip_rect(viewport_rect);

        painter.rect_filled(
            panel_rect,
            10.0,
            egui::Color32::from_rgba_unmultiplied(18, 22, 28, 180),
        );
        painter.rect_stroke(
            panel_rect,
            10.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 48)),
        );
        let cube_rect = egui::Rect::from_min_max(
            panel_rect.min + egui::vec2(10.0, 8.0),
            panel_rect.min + egui::vec2(panel_rect.width() - 10.0, 94.0),
        );
        self.draw_projected_view_cube(ui, cube_rect);

        let iso_rect = egui::Rect::from_min_size(
            egui::pos2(panel_rect.left() + 12.0, panel_rect.bottom() - 28.0),
            egui::vec2(46.0, 18.0),
        );
        let frame_rect = egui::Rect::from_min_size(
            egui::pos2(panel_rect.left() + 64.0, panel_rect.bottom() - 28.0),
            egui::vec2(46.0, 18.0),
        );
        for (label, rect, action) in [
            ("Iso", iso_rect, Some(StandardView::Isometric)),
            ("Fit", frame_rect, None),
        ] {
            let id = ui.make_persistent_id(("viewport_view_cube_action", label));
            let resp = ui.interact(rect, id, egui::Sense::click());
            let fill = if resp.hovered() {
                egui::Color32::from_rgb(120, 86, 40)
            } else {
                egui::Color32::from_rgb(72, 54, 28)
            };
            painter.rect_filled(rect, 6.0, fill);
            painter.rect_stroke(
                rect,
                6.0,
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 80)),
            );
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::proportional(10.0),
                egui::Color32::WHITE,
            );
            if resp.clicked() {
                if let Some(view) = action {
                    self.snap_camera_to_view(view);
                } else {
                    self.frame_current_geometry();
                }
            }
        }
    }

    fn draw_projected_view_cube(&mut self, ui: &mut egui::Ui, cube_rect: egui::Rect) {
        let painter = ui.painter().with_clip_rect(cube_rect.expand(8.0));
        let center = cube_rect.center();
        let scale = cube_rect.width().min(cube_rect.height()) * 0.24;
        let forward = (self.camera.target - self.camera.eye).normalize_or_zero();
        let mut right = forward.cross(self.camera.up).normalize_or_zero();
        if right.length_squared() < 1e-6 {
            right = Vec3::X;
        }
        let mut up = right.cross(forward).normalize_or_zero();
        if up.length_squared() < 1e-6 {
            up = Vec3::Z;
        }

        let vertices = [
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new( 1.0, -1.0, -1.0),
            Vec3::new( 1.0,  1.0, -1.0),
            Vec3::new(-1.0,  1.0, -1.0),
            Vec3::new(-1.0, -1.0,  1.0),
            Vec3::new( 1.0, -1.0,  1.0),
            Vec3::new( 1.0,  1.0,  1.0),
            Vec3::new(-1.0,  1.0,  1.0),
        ];

        let projected: Vec<(egui::Pos2, f32)> = vertices.iter().map(|v| {
            let x = v.dot(right) * scale;
            let y = -v.dot(up) * scale;
            let z = v.dot(forward);
            (egui::pos2(center.x + x, center.y + y), z)
        }).collect();

        struct CubeFace {
            label: &'static str,
            view: StandardView,
            normal: Vec3,
            indices: [usize; 4],
            base: egui::Color32,
        }

        let faces = [
            CubeFace {
                label: "Front",
                view: StandardView::Front,
                normal: Vec3::X,
                indices: [1, 2, 6, 5],
                base: egui::Color32::from_rgb(86, 112, 170),
            },
            CubeFace {
                label: "Back",
                view: StandardView::Back,
                normal: -Vec3::X,
                indices: [0, 4, 7, 3],
                base: egui::Color32::from_rgb(63, 78, 110),
            },
            CubeFace {
                label: "Left",
                view: StandardView::Left,
                normal: Vec3::Y,
                indices: [2, 3, 7, 6],
                base: egui::Color32::from_rgb(74, 138, 174),
            },
            CubeFace {
                label: "Right",
                view: StandardView::Right,
                normal: -Vec3::Y,
                indices: [0, 1, 5, 4],
                base: egui::Color32::from_rgb(64, 116, 146),
            },
            CubeFace {
                label: "Top",
                view: StandardView::Top,
                normal: Vec3::Z,
                indices: [4, 5, 6, 7],
                base: egui::Color32::from_rgb(156, 122, 76),
            },
            CubeFace {
                label: "Bottom",
                view: StandardView::Bottom,
                normal: -Vec3::Z,
                indices: [0, 3, 2, 1],
                base: egui::Color32::from_rgb(108, 78, 50),
            },
        ];

        let pointer_pos = ui.input(|i| i.pointer.interact_pos());
        let pointer_clicked = ui.input(|i| i.pointer.primary_clicked());
        let mut hovered_view = None;
        let mut clicked_view = None;

        let mut visible_faces: Vec<(f32, &CubeFace, Vec<egui::Pos2>)> = faces.iter()
            .filter_map(|face| {
                let facing = face.normal.dot(forward);
                if facing <= 0.0 {
                    return None;
                }
                let poly: Vec<egui::Pos2> = face.indices.iter().map(|&idx| projected[idx].0).collect();
                let depth = face.indices.iter().map(|&idx| projected[idx].1).sum::<f32>() / 4.0;
                Some((depth, face, poly))
            })
            .collect();
        visible_faces.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        painter.rect_filled(cube_rect, 8.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 10));

        for (_, face, poly) in &visible_faces {
            let hovered = pointer_pos.map(|p| Self::point_in_polygon(p, poly)).unwrap_or(false);
            if hovered {
                hovered_view = Some(face.view);
                if pointer_clicked {
                    clicked_view = Some(face.view);
                }
            }
        }

        for (_, face, poly) in &visible_faces {
            let fill = if hovered_view == Some(face.view) {
                face.base.gamma_multiply(1.35)
            } else {
                face.base
            };
            painter.add(egui::Shape::convex_polygon(
                poly.clone(),
                fill,
                egui::Stroke::new(1.2, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 120)),
            ));
            let center = Self::polygon_center(poly);
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                face.label,
                egui::FontId::proportional(11.0),
                egui::Color32::WHITE,
            );
        }

        let edges = [
            (0usize, 1usize), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ];
        for (a, b) in edges {
            painter.line_segment(
                [projected[a].0, projected[b].0],
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 70)),
            );
        }

        if let Some(view) = clicked_view {
            self.snap_camera_to_view(view);
        }
    }

    fn snap_camera_to_view(&mut self, view: StandardView) {
        self.camera.snap_to_view(view);
        let label = match view {
            StandardView::Front => "Front",
            StandardView::Back => "Back",
            StandardView::Left => "Left",
            StandardView::Right => "Right",
            StandardView::Top => "Top",
            StandardView::Bottom => "Bottom",
            StandardView::Isometric => "Isometric",
        };
        self.status_message = Some(format!("Snapped viewport to {} view", label));
    }

    fn frame_current_geometry(&mut self) {
        if let Some(mesh) = &self.current_mesh {
            if let Some((mesh_min, mesh_max)) = Self::mesh_bounds(mesh) {
                self.camera.frame_bounds(mesh_min, mesh_max);
                self.status_message = Some("Framed current geometry".into());
                return;
            }
        }

        if let Some(grid) = &self.current_sdf_grid {
            self.camera.frame_bounds(grid.bounds_min, grid.bounds_max);
            self.status_message = Some("Framed current geometry".into());
        }
    }

    fn mesh_bounds(mesh: &Mesh) -> Option<(Vec3, Vec3)> {
        let first = mesh.vertices.first()?;
        let mut min = Vec3::from_array(first.position);
        let mut max = min;
        for vertex in &mesh.vertices[1..] {
            let p = Vec3::from_array(vertex.position);
            min = min.min(p);
            max = max.max(p);
        }
        Some((min, max))
    }

    fn point_in_polygon(point: egui::Pos2, polygon: &[egui::Pos2]) -> bool {
        let mut inside = false;
        let mut j = polygon.len().saturating_sub(1);
        for i in 0..polygon.len() {
            let pi = polygon[i];
            let pj = polygon[j];
            let intersects = ((pi.y > point.y) != (pj.y > point.y))
                && (point.x < (pj.x - pi.x) * (point.y - pi.y) / ((pj.y - pi.y).abs().max(1e-6)) + pi.x);
            if intersects {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    fn polygon_center(polygon: &[egui::Pos2]) -> egui::Pos2 {
        let mut sum = egui::Vec2::ZERO;
        for p in polygon {
            sum += p.to_vec2();
        }
        let avg = sum / polygon.len().max(1) as f32;
        egui::pos2(avg.x, avg.y)
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

    fn draw_world_bbox(
        painter: &egui::Painter,
        bounds_min: Vec3,
        bounds_max: Vec3,
        color: egui::Color32,
        camera: &Camera,
        rect: egui::Rect,
    ) {
        let corners = [
            Vec3::new(bounds_min.x, bounds_min.y, bounds_min.z),
            Vec3::new(bounds_max.x, bounds_min.y, bounds_min.z),
            Vec3::new(bounds_max.x, bounds_max.y, bounds_min.z),
            Vec3::new(bounds_min.x, bounds_max.y, bounds_min.z),
            Vec3::new(bounds_min.x, bounds_min.y, bounds_max.z),
            Vec3::new(bounds_max.x, bounds_min.y, bounds_max.z),
            Vec3::new(bounds_max.x, bounds_max.y, bounds_max.z),
            Vec3::new(bounds_min.x, bounds_max.y, bounds_max.z),
        ];
        let projected: Vec<Option<egui::Pos2>> = corners.iter()
            .map(|&p| Self::project_to_screen(p, camera, rect))
            .collect();
        let edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ];
        for (a, b) in edges {
            if let (Some(pa), Some(pb)) = (projected[a], projected[b]) {
                painter.line_segment([pa, pb], egui::Stroke::new(2.0, color));
            }
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
        // FEA overlay takes priority over everything else.
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
        // Overhang overlay: values 1.0 (Good) … 4.0 (Critical).
        if self.print_overhang_overlay && self.print_analysis_result.is_some() {
            return ThicknessUniforms { enabled: true, min_display: 1.0, max_display: 4.0, invert: false };
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

    fn render_print_analysis_panel(&mut self, ui: &mut egui::Ui) {
        use crate::analysis::print_analysis::{PrinterPreset, IssueType, IssueSeverity};

        ui.heading("Print Analysis");
        ui.separator();

        // --- Printer preset dropdown ---
        ui.horizontal(|ui| {
            ui.label("Printer:");
            let current_preset = self.print_analysis_settings.preset.clone();
            let preset_name = |p: &PrinterPreset| match p {
                PrinterPreset::GenericFDM    => "Generic FDM (220×220×250)",
                PrinterPreset::BambuX1C     => "Bambu X1C (256×256×256)",
                PrinterPreset::PrusaMK4     => "Prusa MK4 (250×210×220)",
                PrinterPreset::Voron24_300  => "Voron 2.4 300 (300×300×300)",
                PrinterPreset::Custom       => "Custom",
            };
            egui::ComboBox::from_id_salt("printer_preset")
                .selected_text(preset_name(&current_preset))
                .show_ui(ui, |ui| {
                    for p in [
                        PrinterPreset::GenericFDM,
                        PrinterPreset::BambuX1C,
                        PrinterPreset::PrusaMK4,
                        PrinterPreset::Voron24_300,
                        PrinterPreset::Custom,
                    ] {
                        let label = preset_name(&p);
                        if ui.selectable_label(current_preset == p, label).clicked() {
                            self.print_analysis_settings.apply_preset(p);
                        }
                    }
                });
        });

        ui.add_space(4.0);

        // --- Settings ---
        ui.collapsing("Settings", |ui| {
            ui.add(egui::Slider::new(&mut self.print_analysis_settings.overhang_threshold_deg, 20.0..=70.0)
                .text("Overhang threshold (°)"));
            ui.add(egui::Slider::new(&mut self.print_analysis_settings.min_wall_thickness, 0.2..=5.0)
                .text("Min wall (mm)"));
            ui.add(egui::Slider::new(&mut self.print_analysis_settings.min_feature_size, 0.1..=2.0)
                .text("Min feature (mm)"));
            ui.add(egui::Slider::new(&mut self.print_analysis_settings.max_aspect_ratio, 2.0..=20.0)
                .text("Max aspect ratio"));
            let bv = &mut self.print_analysis_settings.build_volume;
            ui.horizontal(|ui| {
                ui.label("Build volume:");
                ui.add(egui::DragValue::new(&mut bv.x).suffix("mm").speed(1.0));
                ui.label("×");
                ui.add(egui::DragValue::new(&mut bv.y).suffix("mm").speed(1.0));
                ui.label("×");
                ui.add(egui::DragValue::new(&mut bv.z).suffix("mm").speed(1.0));
            });
        });

        ui.add_space(4.0);

        // --- Run / Clear ---
        let has_grid = self.current_sdf_grid.is_some();
        let running  = self.print_analysis_running;
        ui.horizontal(|ui| {
            if ui.add_enabled(has_grid && !running, egui::Button::new("Run Print Analysis")).clicked() {
                self.start_print_analysis();
            }
            if ui.add_enabled(self.print_analysis_result.is_some(), egui::Button::new("Clear")).clicked() {
                self.print_analysis_result       = None;
                self.print_overhang_overlay      = false;
                self.print_overhang_needs_upload = false;
            }
        });

        if running {
            ui.horizontal(|ui| { ui.spinner(); ui.label("Analysing…"); });
        }

        if let Some(ref result) = self.print_analysis_result {
            ui.add_space(6.0);
            ui.separator();

            // --- Overhang summary ---
            ui.strong("Overhang");
            let prev_overlay = self.print_overhang_overlay;
            ui.checkbox(&mut self.print_overhang_overlay, "Show overhang overlay");
            if self.print_overhang_overlay && !prev_overlay {
                self.print_overhang_needs_upload = true;
            }
            ui.label(format!("Overhang area: {:.1} mm²", result.overhang.overhang_area_mm2));
            ui.label(format!("Critical area:  {:.1} mm²", result.overhang.critical_overhang_area_mm2));
            ui.label(format!("Support vol est: {:.0} mm³", result.overhang.support_volume_estimate_mm3));

            ui.add_space(6.0);
            ui.separator();

            // --- Orientation advisor ---
            ui.strong("Orientation Advisor");
            let rec_idx = result.orientation.recommended;
            for (i, cand) in result.orientation.candidates.iter().enumerate() {
                let d = cand.build_direction;
                let label = format!(
                    "{}{} ({:.2},{:.2},{:.2})  score={:.2}",
                    if i == rec_idx { "★ " } else { "  " },
                    if cand.fits_build_volume { "✓" } else { "✗" },
                    d.x, d.y, d.z,
                    cand.score
                );
                let color = if i == rec_idx {
                    egui::Color32::from_rgb(80, 200, 80)
                } else {
                    ui.style().visuals.text_color()
                };
                ui.horizontal(|ui| {
                    ui.colored_label(color, &label);
                    if ui.small_button("Apply").clicked() {
                        self.print_analysis_settings.build_direction = cand.build_direction;
                    }
                });
            }

            ui.add_space(6.0);
            ui.separator();

            // --- Feature / printability issues ---
            let issues = &result.features.issues;
            let errors: usize   = issues.iter().filter(|i| matches!(i.severity, IssueSeverity::Error)).count();
            let warnings: usize = issues.iter().filter(|i| matches!(i.severity, IssueSeverity::Warning)).count();
            ui.horizontal(|ui| {
                ui.strong("Printability Issues");
                if errors > 0 {
                    ui.colored_label(egui::Color32::from_rgb(220, 60, 60),
                        format!("{} errors", errors));
                }
                if warnings > 0 {
                    ui.colored_label(egui::Color32::from_rgb(220, 160, 40),
                        format!("{} warnings", warnings));
                }
                if errors == 0 && warnings == 0 {
                    ui.colored_label(egui::Color32::from_rgb(60, 200, 60), "✓ No issues");
                }
            });

            egui::ScrollArea::vertical()
                .max_height(200.0)
                .id_salt("print_issues")
                .show(ui, |ui| {
                    for issue in issues.iter().take(50) {
                        let (icon, color) = match issue.severity {
                            IssueSeverity::Error   => ("✗", egui::Color32::from_rgb(220, 60, 60)),
                            IssueSeverity::Warning => ("⚠", egui::Color32::from_rgb(220, 160, 40)),
                        };
                        let type_str = match issue.issue_type {
                            IssueType::ThinWall           => "ThinWall",
                            IssueType::TinyFeature        => "TinyFeature",
                            IssueType::HighAspectRatio    => "HighAspectRatio",
                            IssueType::BridgeSpan         => "BridgeSpan",
                            IssueType::SharpInternalCorner => "SharpCorner",
                        };
                        ui.colored_label(color, format!("{} [{}] {}", icon, type_str, issue.description));
                    }
                    if issues.len() > 50 {
                        ui.label(format!("… and {} more", issues.len() - 50));
                    }
                });
        } else if !running {
            ui.add_space(4.0);
            ui.label("No analysis data. Press Run Print Analysis.");
        }

        // ── Tolerance Compensation section ────────────────────────────────────
        ui.add_space(8.0);
        ui.separator();
        self.render_tolerance_section(ui);

        // ── Split Body section (always available when there's a model) ────────
        if self.current_sdf.is_some() {
            ui.add_space(8.0);
            ui.separator();
            self.render_split_body_section(ui);
        }
    }

    fn render_tolerance_section(&mut self, ui: &mut egui::Ui) {
        use crate::sdf::print::TolerancePreset;

        ui.collapsing("Tolerance Compensation", |ui| {
            // Preset dropdown
            ui.horizontal(|ui| {
                ui.label("Preset:");
                let cur = self.tolerance_preset.clone();
                let label = match &cur {
                    TolerancePreset::TightFit    => "Tight Fit (0.1 mm)",
                    TolerancePreset::StandardFDM => "Standard FDM (0.15 mm)",
                    TolerancePreset::LooseFit    => "Loose Fit (0.2 mm)",
                    TolerancePreset::Custom      => "Custom",
                };
                egui::ComboBox::from_id_salt("tol_preset")
                    .selected_text(label)
                    .show_ui(ui, |ui| {
                        for p in [
                            TolerancePreset::TightFit,
                            TolerancePreset::StandardFDM,
                            TolerancePreset::LooseFit,
                            TolerancePreset::Custom,
                        ] {
                            let lbl = match &p {
                                TolerancePreset::TightFit    => "Tight Fit (0.1 mm)",
                                TolerancePreset::StandardFDM => "Standard FDM (0.15 mm)",
                                TolerancePreset::LooseFit    => "Loose Fit (0.2 mm)",
                                TolerancePreset::Custom      => "Custom",
                            };
                            if ui.selectable_label(cur == p, lbl).clicked() {
                                self.tolerance_settings.apply_preset(&p);
                                self.tolerance_preset = p;
                            }
                        }
                    });
            });

            ui.add_space(4.0);
            ui.add(egui::Slider::new(&mut self.tolerance_settings.external_offset_mm, -0.5..=0.5)
                .text("External offset (mm)").step_by(0.01));
            ui.add(egui::Slider::new(&mut self.tolerance_settings.internal_offset_mm, -0.5..=0.5)
                .text("Internal offset (mm)").step_by(0.01));
            ui.add(egui::Slider::new(&mut self.tolerance_settings.min_hole_diameter_mm, 0.5..=10.0)
                .text("Small hole threshold (mm)"));
            ui.add(egui::Slider::new(&mut self.tolerance_settings.small_hole_bonus_mm, 0.0..=0.3)
                .text("Small hole bonus (mm)").step_by(0.01));

            ui.add_space(4.0);
            ui.checkbox(&mut self.tolerance_on_export,
                "Apply tolerance compensation on export");
            if self.tolerance_on_export {
                ui.colored_label(egui::Color32::from_rgb(100, 180, 100),
                    "✓ Geometry will be wrapped with tolerance offsets at export time");
            }
        });
    }

    fn render_split_body_section(&mut self, ui: &mut egui::Ui) {
        use crate::sdf::print::{SplitPlane, AlignmentFeature, split_body, verify_split_fit};
        use std::sync::Arc;

        ui.collapsing("Split Body", |ui| {
            // --- Axis selector ---
            ui.horizontal(|ui| {
                ui.label("Axis:");
                ui.selectable_value(&mut self.split_axis, SplitAxisUi::X, "X");
                ui.selectable_value(&mut self.split_axis, SplitAxisUi::Y, "Y");
                ui.selectable_value(&mut self.split_axis, SplitAxisUi::Z, "Z");
            });

            // --- Position slider ---
            let range = if let Some(ref g) = self.current_sdf_grid {
                let (lo, hi) = match self.split_axis {
                    SplitAxisUi::X => (g.bounds_min.x, g.bounds_max.x),
                    SplitAxisUi::Y => (g.bounds_min.y, g.bounds_max.y),
                    SplitAxisUi::Z | SplitAxisUi::Arbitrary => (g.bounds_min.z, g.bounds_max.z),
                };
                (lo, hi)
            } else {
                (-100.0_f32, 100.0_f32)
            };
            ui.add(egui::Slider::new(&mut self.split_offset, range.0..=range.1).text("Position (mm)"));

            // --- Alignment type ---
            ui.horizontal(|ui| {
                ui.label("Alignment:");
                egui::ComboBox::from_id_salt("split_align")
                    .selected_text(match self.split_align_type {
                        SplitAlignUi::None     => "None",
                        SplitAlignUi::Pins     => "Pins & Sockets",
                        SplitAlignUi::Groove   => "Tongue & Groove",
                        SplitAlignUi::Dovetail => "Dovetail",
                        SplitAlignUi::BoltHoles => "Bolt Holes",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.split_align_type, SplitAlignUi::None, "None");
                        ui.selectable_value(&mut self.split_align_type, SplitAlignUi::Pins, "Pins & Sockets");
                        ui.selectable_value(&mut self.split_align_type, SplitAlignUi::Groove, "Tongue & Groove");
                        ui.selectable_value(&mut self.split_align_type, SplitAlignUi::Dovetail, "Dovetail");
                        ui.selectable_value(&mut self.split_align_type, SplitAlignUi::BoltHoles, "Bolt Holes");
                    });
            });

            // --- Alignment parameters ---
            match self.split_align_type {
                SplitAlignUi::None => {}
                SplitAlignUi::Pins => {
                    ui.add(egui::Slider::new(&mut self.split_pin_radius, 0.5..=5.0).text("Pin radius (mm)"));
                    ui.add(egui::Slider::new(&mut self.split_pin_height, 1.0..=10.0).text("Pin height (mm)"));
                    ui.add(egui::Slider::new(&mut self.split_pin_count, 2..=8).text("Count"));
                    ui.add(egui::Slider::new(&mut self.split_pattern_r, 2.0..=50.0).text("Pattern radius (mm)"));
                }
                SplitAlignUi::Groove => {
                    ui.add(egui::Slider::new(&mut self.split_groove_width, 2.0..=30.0).text("Width (mm)"));
                    ui.add(egui::Slider::new(&mut self.split_groove_height, 1.0..=10.0).text("Height (mm)"));
                }
                SplitAlignUi::Dovetail => {
                    ui.add(egui::Slider::new(&mut self.split_groove_width, 2.0..=30.0).text("Width (mm)"));
                    ui.add(egui::Slider::new(&mut self.split_groove_height, 1.0..=10.0).text("Height (mm)"));
                    ui.add(egui::Slider::new(&mut self.split_dovetail_angle, 5.0..=30.0).text("Angle (°)"));
                }
                SplitAlignUi::BoltHoles => {
                    ui.add(egui::Slider::new(&mut self.split_bolt_radius, 0.5..=5.0).text("Bolt radius (mm)"));
                    ui.add(egui::Slider::new(&mut self.split_boss_radius, 1.0..=10.0).text("Boss radius (mm)"));
                    ui.add(egui::Slider::new(&mut self.split_bolt_count, 2..=8).text("Count"));
                    ui.add(egui::Slider::new(&mut self.split_pattern_r, 2.0..=50.0).text("Pattern radius (mm)"));
                }
            }

            ui.add_space(4.0);

            // --- Build the SplitPlane and AlignmentFeature from current UI state ---
            let plane = match self.split_axis {
                SplitAxisUi::X => SplitPlane::X(self.split_offset),
                SplitAxisUi::Y => SplitPlane::Y(self.split_offset),
                SplitAxisUi::Z | SplitAxisUi::Arbitrary => SplitPlane::Z(self.split_offset),
            };

            let alignment: AlignmentFeature = match self.split_align_type {
                SplitAlignUi::None => AlignmentFeature::None,
                SplitAlignUi::Pins => AlignmentFeature::PinsAndSockets {
                    pin_radius:       self.split_pin_radius,
                    pin_height:       self.split_pin_height,
                    socket_clearance: 0.15,
                    count:            self.split_pin_count,
                    pattern_radius:   self.split_pattern_r,
                },
                SplitAlignUi::Groove => AlignmentFeature::TongueAndGroove {
                    tongue_width:    self.split_groove_width,
                    tongue_height:   self.split_groove_height,
                    groove_clearance: 0.15,
                },
                SplitAlignUi::Dovetail => AlignmentFeature::Dovetail {
                    width:     self.split_groove_width,
                    height:    self.split_groove_height,
                    angle_deg: self.split_dovetail_angle,
                    clearance: 0.15,
                },
                SplitAlignUi::BoltHoles => AlignmentFeature::BoltHoles {
                    bolt_radius:    self.split_bolt_radius,
                    boss_radius:    self.split_boss_radius,
                    boss_height:    3.0,
                    count:          self.split_bolt_count,
                    pattern_radius: self.split_pattern_r,
                    countersink:    false,
                },
            };

            ui.horizontal(|ui| {
                let has_sdf = self.current_sdf.is_some();
                if ui.add_enabled(has_sdf, egui::Button::new("Preview Split")).clicked() {
                    if let Some(ref sdf) = self.current_sdf {
                        let result = split_body(Arc::clone(sdf), &plane, &alignment);
                        self.split_preview = Some((result.part_a, result.part_b));
                        self.split_fit_result = None;
                    }
                }
                if self.split_preview.is_some() {
                    if ui.button("Verify Fit").clicked() {
                        if let Some(ref sdf) = self.current_sdf {
                            let result = split_body(Arc::clone(sdf), &plane, &alignment);
                            let half_size = self.current_sdf_grid.as_ref()
                                .map(|g| (g.bounds_max - g.bounds_min).max_element() * 0.6)
                                .unwrap_or(30.0);
                            self.split_fit_result = Some(verify_split_fit(&result, half_size));
                        }
                    }
                    if ui.button("Clear").clicked() {
                        self.split_preview   = None;
                        self.split_fit_result = None;
                    }
                }
            });

            // --- Fit result summary ---
            if let Some(ref fit) = self.split_fit_result {
                ui.add_space(4.0);
                if fit.fits {
                    ui.colored_label(egui::Color32::from_rgb(60, 200, 60), "✓ Fit OK");
                } else {
                    ui.colored_label(egui::Color32::from_rgb(220, 60, 60), "✗ Fit issues:");
                    for w in &fit.warnings {
                        ui.label(format!("  {}", w));
                    }
                }
                ui.label(format!("Interference: {:.3} mm³   Gap: {:.3} mm³",
                    fit.interference_volume_mm3, fit.gap_volume_mm3));
            }

            // --- Preview info ---
            if let Some(ref _preview) = self.split_preview {
                ui.add_space(4.0);
                ui.colored_label(egui::Color32::from_rgb(100, 140, 220), "Part A (top/positive side)");
                ui.colored_label(egui::Color32::from_rgb(220, 130, 60), "Part B (bottom/negative side)");
                ui.label("Use split() in script to produce these as separate SdfHandles.");
            }
        });
    }

    fn start_print_analysis(&mut self) {
        let Some(ref grid) = self.current_sdf_grid else { return };
        let grid_clone    = Arc::clone(grid);
        let settings      = self.print_analysis_settings.clone();
        // Pass a thin-wall result if available for feature detection
        let thickness_opt = self.thickness_result.clone();
        let fea_stress: Option<Vec<f32>> = self.fea_result.as_ref().map(|r| r.von_mises.clone());

        let (tx, rx) = std::sync::mpsc::channel();
        self.print_analysis_receiver = Some(rx);
        self.print_analysis_running  = true;

        std::thread::spawn(move || {
            let result = crate::analysis::print_analysis::compute_print_analysis(
                &grid_clone,
                &settings,
                thickness_opt.as_ref(),
                fea_stress.as_deref(),
            );
            let _ = tx.send(result);
        });
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
            setup:        std::mem::take(&mut self.state.fea_setup),
            config:       self.fea_config.clone(),
            ccx_override: self.settings.ccx_path.clone(),
            layups:       self.current_layups.clone(),
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

    // ── Aerodynamic Analysis Panel ────────────────────────────────────────────

    fn render_aero_panel(&mut self, ui: &mut egui::Ui) {
        use crate::aero::{FlightCondition, PolarDatabase, solve_lifting_line};

        ui.heading("Flight Conditions");
        ui.separator();

        egui::Grid::new("aero_fc_grid")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Airspeed (m/s):");
                ui.add(egui::DragValue::new(&mut self.aero_airspeed_ms)
                    .range(1.0..=400.0).speed(0.5));
                ui.end_row();

                ui.label("Altitude (m):");
                ui.add(egui::DragValue::new(&mut self.aero_altitude_m)
                    .range(0.0..=20000.0).speed(10.0));
                ui.end_row();

                ui.label("AoA (deg):");
                ui.add(egui::DragValue::new(&mut self.aero_aoa_deg)
                    .range(-20.0..=25.0).speed(0.1));
                ui.end_row();

                // Density readout (computed from ISA).
                let fc_preview = FlightCondition::new(
                    self.aero_airspeed_ms, self.aero_altitude_m, self.aero_aoa_deg);
                ui.label("Air density (kg/m³):");
                ui.label(format!("{:.4}", fc_preview.air_density_kg_m3));
                ui.end_row();
                ui.label("Dyn. pressure (Pa):");
                ui.label(format!("{:.1}", fc_preview.dynamic_pressure_pa));
                ui.end_row();
            });

        ui.separator();

        if ui.button("▶  Run Lifting Line").clicked() {
            if let Some(sdf) = self.current_sdf.clone() {
                let fc  = FlightCondition::new(
                    self.aero_airspeed_ms, self.aero_altitude_m, self.aero_aoa_deg);
                let db  = PolarDatabase::new();
                let res = solve_lifting_line(&sdf, &db, &fc, 20);
                self.current_flight_condition    = Some(fc);
                self.current_lifting_line_result = Some(res);
            } else {
                ui.label("⚠ No SDF loaded — run script first.");
            }
        }

        if let Some(ref res) = self.current_lifting_line_result {
            ui.separator();
            ui.heading("Lifting Line Results");

            egui::Grid::new("aero_results_grid")
                .num_columns(2)
                .spacing([8.0, 4.0])
                .show(ui, |ui| {
                    ui.label("CL:");      ui.label(format!("{:.4}", res.cl_total));        ui.end_row();
                    ui.label("CDi:");     ui.label(format!("{:.5}", res.cd_induced));      ui.end_row();
                    ui.label("Oswald e:"); ui.label(format!("{:.4}", res.oswald_efficiency)); ui.end_row();
                    ui.label("Lift (N):");  ui.label(format!("{:.2}", res.lift_total_n));  ui.end_row();
                    ui.label("Di (N):");    ui.label(format!("{:.3}", res.induced_drag_total_n)); ui.end_row();
                });

            // Stall warnings.
            if res.tip_stall_risk {
                ui.colored_label(egui::Color32::from_rgb(220, 60, 60), "⚠ Tip stall risk!");
            }
            if !res.stall_stations.is_empty() {
                let label = if res.root_stall_first {
                    format!("⚠ Stall at {} stations (root first)", res.stall_stations.len())
                } else {
                    format!("⚠ Stall at {} stations", res.stall_stations.len())
                };
                ui.colored_label(egui::Color32::from_rgb(220, 140, 0), &label);
            } else {
                ui.colored_label(egui::Color32::from_rgb(60, 180, 60), "✓ No stall");
            }

            // Spanwise CL distribution.
            ui.separator();
            ui.label("Spanwise CL distribution:");
            let n = res.span_stations.len();
            if n > 0 {
                let cl_max = res.local_cl.iter().cloned().fold(0.0_f32, f32::max).max(0.1);
                let bar_w  = (ui.available_width() / n as f32).max(2.0);
                let bar_h  = 60.0;
                let (resp, painter) = ui.allocate_painter(
                    egui::Vec2::new(ui.available_width(), bar_h),
                    egui::Sense::hover(),
                );
                let rect = resp.rect;
                for i in 0..n {
                    let x_frac  = i as f32 / n as f32;
                    let cl_frac = (res.local_cl[i] / cl_max).clamp(0.0, 1.0);
                    let margin  = res.stall_margin[i];
                    let color   = if margin < 0.0 {
                        egui::Color32::from_rgb(200, 50, 50)
                    } else if margin < 0.3 {
                        egui::Color32::from_rgb(200, 160, 30)
                    } else {
                        egui::Color32::from_rgb(40, 160, 80)
                    };
                    let x0 = rect.left() + x_frac * rect.width();
                    let x1 = x0 + bar_w;
                    let y1 = rect.bottom();
                    let y0 = y1 - cl_frac * bar_h;
                    painter.rect_filled(egui::Rect::from_min_max(
                        egui::Pos2::new(x0, y0), egui::Pos2::new(x1, y1)), 0.0, color);
                }
            }

            if ui.button("Clear").clicked() {
                self.current_lifting_line_result = None;
                self.current_flight_condition    = None;
            }
        }

        // ── Stability Analysis ─────────────────────────────────────────────────
        ui.separator();
        ui.heading("Stability Analysis");

        egui::Grid::new("aero_stability_inputs")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("CG x (mm):");
                ui.add(egui::DragValue::new(&mut self.aero_cg_x_mm)
                    .range(-1000.0..=5000.0).speed(1.0));
                ui.end_row();
            });

        if ui.button("▶  Run Stability Analysis").clicked() {
            if let Some(sdf) = self.current_sdf.clone() {
                use crate::aero::{PolarDatabase, compute_neutral_point, compute_static_margin};
                use crate::sdf::query::bounding_points;
                let fc   = FlightCondition::new(
                    self.aero_airspeed_ms, self.aero_altitude_m, self.aero_aoa_deg);
                let db   = PolarDatabase::new();
                let np   = compute_neutral_point(&sdf, &sdf, &sdf, &db, &fc);
                let bbox = bounding_points(sdf.as_ref());
                let root_chord = bbox.size.x;
                let mac  = (root_chord + root_chord * 0.5) * 0.5;
                let cg   = glam::Vec3::new(self.aero_cg_x_mm, 0.0, 0.0);
                let sm   = compute_static_margin(&np, cg, mac);
                self.current_neutral_point = Some(np);
                self.current_static_margin = Some(sm);
                self.current_flight_condition = Some(fc);
            } else {
                ui.label("⚠ No SDF loaded — run script first.");
            }
        }

        if let Some(ref np) = self.current_neutral_point.clone() {
            if let Some(ref sm) = self.current_static_margin.clone() {
                egui::Grid::new("aero_stability_results")
                    .num_columns(2)
                    .spacing([8.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("NP x (mm):"); ui.label(format!("{:.1}", np.neutral_point_x_mm)); ui.end_row();
                        ui.label("Wing AC (mm):"); ui.label(format!("{:.1}", np.wing_ac_x_mm)); ui.end_row();
                        ui.label("Tail AC (mm):"); ui.label(format!("{:.1}", np.htail_ac_x_mm)); ui.end_row();
                        ui.label("CG (mm):"); ui.label(format!("{:.1}", sm.cg_x_mm)); ui.end_row();
                        ui.label("Static Margin:"); ui.label(format!("{:.1}% MAC", sm.static_margin_mac * 100.0)); ui.end_row();
                        ui.label("CG Range:"); ui.label(format!("{:.1}–{:.1} mm ({:.1} mm)", sm.cg_forward_limit_mm, sm.cg_aft_limit_mm, sm.cg_range_mm)); ui.end_row();
                    });

                // Stability badge.
                let (badge_color, badge_text) = match sm.stability_category {
                    crate::aero::StabilityCategory::VeryStable => (egui::Color32::from_rgb(40, 160, 80),  "Very Stable"),
                    crate::aero::StabilityCategory::Stable     => (egui::Color32::from_rgb(80, 200, 80),  "Stable"),
                    crate::aero::StabilityCategory::Marginal   => (egui::Color32::from_rgb(200, 160, 30), "Marginal"),
                    crate::aero::StabilityCategory::Neutral    => (egui::Color32::from_rgb(200, 100, 30), "Neutral"),
                    crate::aero::StabilityCategory::Unstable   => (egui::Color32::from_rgb(200, 50, 50),  "Unstable"),
                };
                ui.colored_label(badge_color, badge_text);

                // CG position bar (painter-based, no egui_plot needed).
                let bar_w = ui.available_width();
                let bar_h = 18.0;
                let (resp, painter) = ui.allocate_painter(
                    egui::Vec2::new(bar_w, bar_h), egui::Sense::hover());
                let rect = resp.rect;
                let x_min_vis = sm.cg_forward_limit_mm - 20.0;
                let x_max_vis = np.htail_ac_x_mm + 20.0;
                let x_range   = (x_max_vis - x_min_vis).max(1.0);
                let to_screen  = |x: f32| rect.left() + (x - x_min_vis) / x_range * rect.width();

                // Background bar.
                painter.rect_filled(rect, 2.0, egui::Color32::from_gray(40));
                // CG range (green zone).
                let x0 = to_screen(sm.cg_forward_limit_mm).clamp(rect.left(), rect.right());
                let x1 = to_screen(sm.cg_aft_limit_mm).clamp(rect.left(), rect.right());
                painter.rect_filled(
                    egui::Rect::from_min_max(egui::Pos2::new(x0, rect.top()), egui::Pos2::new(x1, rect.bottom())),
                    0.0, egui::Color32::from_rgba_unmultiplied(80, 180, 80, 80));
                // NP marker (blue line).
                let xnp = to_screen(np.neutral_point_x_mm).clamp(rect.left(), rect.right());
                painter.line_segment(
                    [egui::Pos2::new(xnp, rect.top()), egui::Pos2::new(xnp, rect.bottom())],
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 160, 255)));
                // CG marker (orange triangle).
                let xcg = to_screen(sm.cg_x_mm).clamp(rect.left(), rect.right());
                let mid_y = rect.center().y;
                painter.add(egui::Shape::convex_polygon(
                    vec![
                        egui::Pos2::new(xcg, mid_y - 6.0),
                        egui::Pos2::new(xcg - 5.0, mid_y + 4.0),
                        egui::Pos2::new(xcg + 5.0, mid_y + 4.0),
                    ],
                    egui::Color32::from_rgb(240, 140, 30),
                    egui::Stroke::NONE,
                ));
            }
        }

        // ── Trim Analysis ──────────────────────────────────────────────────────
        ui.separator();
        ui.heading("Trim Analysis");

        egui::Grid::new("aero_trim_inputs")
            .num_columns(2)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                ui.label("Weight (N):");
                ui.add(egui::DragValue::new(&mut self.aero_weight_n)
                    .range(0.1..=10000.0).speed(0.1));
                ui.end_row();
            });

        if ui.button("▶  Run Trim Analysis").clicked() {
            if let Some(sdf) = self.current_sdf.clone() {
                use crate::aero::{PolarDatabase, compute_neutral_point, compute_static_margin, compute_trim};
                use crate::sdf::query::bounding_points;
                let fc   = FlightCondition::new(
                    self.aero_airspeed_ms, self.aero_altitude_m, self.aero_aoa_deg);
                let db   = PolarDatabase::new();
                let np   = compute_neutral_point(&sdf, &sdf, &sdf, &db, &fc);
                let bbox = bounding_points(sdf.as_ref());
                let root_chord = bbox.size.x;
                let mac  = (root_chord + root_chord * 0.5) * 0.5;
                let cg   = glam::Vec3::new(self.aero_cg_x_mm, 0.0, 0.0);
                let sm   = compute_static_margin(&np, cg, mac);
                let trim = compute_trim(&np, &sm, &sdf, &sdf, &sdf, &db, &fc, self.aero_weight_n);
                self.current_neutral_point = Some(np);
                self.current_static_margin = Some(sm);
                self.current_trim_result   = Some(trim);
                self.current_flight_condition = Some(fc);
            } else {
                ui.label("⚠ No SDF loaded — run script first.");
            }
        }

        if let Some(ref trim) = self.current_trim_result.clone() {
            egui::Grid::new("aero_trim_results")
                .num_columns(2)
                .spacing([8.0, 4.0])
                .show(ui, |ui| {
                    ui.label("Trim AoA:");  ui.label(format!("{:.1}°", trim.trim_aoa_deg)); ui.end_row();
                    ui.label("Trim CL:");   ui.label(format!("{:.3}", trim.trim_cl)); ui.end_row();
                    ui.label("Trim Speed:"); ui.label(format!("{:.1} m/s", trim.trim_airspeed_ms)); ui.end_row();
                    ui.label("Stall Margin:"); ui.label(format!("{:.1}°", trim.trim_margin_deg)); ui.end_row();
                });
            if trim.is_trimmed {
                ui.colored_label(egui::Color32::from_rgb(60, 180, 60), "✓ Trim found");
            } else {
                ui.colored_label(egui::Color32::from_rgb(200, 140, 30), "⚠ Approx trim (no zero crossing)");
            }
        }

        // ── Drag Polar ──────────────────────────────────────────────────────────
        ui.separator();
        ui.heading("Drag Polar");

        if ui.button("▶  Run Drag Polar").clicked() {
            if let Some(sdf) = self.current_sdf.clone() {
                use crate::aero::{PolarDatabase, compute_drag_polar};
                let fc     = FlightCondition::new(
                    self.aero_airspeed_ms, self.aero_altitude_m, self.aero_aoa_deg);
                let db     = PolarDatabase::new();
                let result = compute_drag_polar(&sdf, &sdf, &sdf, &sdf, &db, &fc, Some(self.aero_weight_n));
                self.current_drag_polar = Some(result);
                self.current_flight_condition = Some(fc);
            } else {
                ui.label("⚠ No SDF loaded — run script first.");
            }
        }

        if let Some(ref dp) = self.current_drag_polar.clone() {
            egui::Grid::new("aero_drag_results")
                .num_columns(2)
                .spacing([8.0, 4.0])
                .show(ui, |ui| {
                    ui.label("CD0:");       ui.label(format!("{:.4}", dp.cd0)); ui.end_row();
                    ui.label("k:");         ui.label(format!("{:.4}", dp.k)); ui.end_row();
                    ui.label("L/D max:");   ui.label(format!("{:.1}", dp.ld_max)); ui.end_row();
                    ui.label("Best glide:"); ui.label(format!("{:.1} m/s", dp.best_glide_airspeed_ms)); ui.end_row();
                    ui.label("  Wing:");    ui.label(format!("{:.4}", dp.cd0_breakdown.wing)); ui.end_row();
                    ui.label("  Fuselage:"); ui.label(format!("{:.4}", dp.cd0_breakdown.fuselage)); ui.end_row();
                    ui.label("  H-tail:");  ui.label(format!("{:.4}", dp.cd0_breakdown.h_tail)); ui.end_row();
                    ui.label("  V-tail:");  ui.label(format!("{:.4}", dp.cd0_breakdown.v_tail)); ui.end_row();
                });

            // Simple text table of polar points (no egui_plot dependency).
            ui.separator();
            ui.label("CD vs CL polar:");
            egui::ScrollArea::vertical()
                .max_height(120.0)
                .id_salt("drag_polar_scroll")
                .show(ui, |ui| {
                    for (cl, cd) in &dp.polar_points {
                        ui.label(format!("CL={:.3}  CD={:.5}  L/D={:.1}",
                            cl, cd, if *cd > 0.0 { *cl / *cd } else { 0.0 }));
                    }
                });
        }

        // ── Flight Envelope Summary ────────────────────────────────────────────
        let all_done = self.current_static_margin.is_some()
            && self.current_trim_result.is_some()
            && self.current_drag_polar.is_some();

        if all_done {
            ui.separator();
            ui.heading("Flight Envelope Summary");

            let sm   = self.current_static_margin.as_ref().unwrap();
            let trim = self.current_trim_result.as_ref().unwrap();
            let dp   = self.current_drag_polar.as_ref().unwrap();

            // Stall speed from CL_max.
            let db = PolarDatabase::new();
            if let Some(sdf) = self.current_sdf.clone() {
                use crate::sdf::query::bounding_points;
                let bbox = bounding_points(sdf.as_ref());
                let root_chord = bbox.size.x;
                let mac  = (root_chord + root_chord * 0.5) * 0.5;
                let b_m  = bbox.size.y / 1000.0;
                let s_m2 = b_m * mac / 1000.0;
                let fc   = FlightCondition::new(self.aero_airspeed_ms, self.aero_altitude_m, 0.0);
                let polar = db.get_interpolated("NACA 0012", fc.reynolds_for_chord(mac))
                    .or_else(|| db.get_interpolated("NACA 0012", 500_000.0));
                if let Some(ref p) = polar {
                    let rho  = fc.air_density_kg_m3;
                    let v_stall = if s_m2 > 1e-9 && p.cl_max > 0.0 {
                        (2.0 * self.aero_weight_n / (rho * s_m2 * p.cl_max)).sqrt()
                    } else {
                        0.0
                    };
                    egui::Grid::new("aero_envelope_grid")
                        .num_columns(2)
                        .spacing([8.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("Stall Speed:");
                            ui.label(format!("{:.1} m/s  ({:.0} km/h)", v_stall, v_stall * 3.6));
                            ui.end_row();
                            ui.label("Best L/D:");
                            ui.label(format!("{:.1} at {:.1} m/s", dp.ld_max, dp.best_glide_airspeed_ms));
                            ui.end_row();
                            ui.label("Static Margin:");
                            let sm_pct = sm.static_margin_mac * 100.0;
                            let sm_color = if sm.is_stable {
                                egui::Color32::from_rgb(60, 180, 60)
                            } else {
                                egui::Color32::from_rgb(200, 50, 50)
                            };
                            ui.colored_label(sm_color, format!("{:.1}% MAC", sm_pct));
                            ui.end_row();
                            ui.label("Trim AoA:");
                            ui.label(format!("{:.1}°", trim.trim_aoa_deg));
                            ui.end_row();
                            ui.label("CG Range:");
                            ui.label(format!("{:.1}–{:.1} mm", sm.cg_forward_limit_mm, sm.cg_aft_limit_mm));
                            ui.end_row();
                        });
                }
            }
        }

        // ── CG Sensitivity (Phase 30) ────────────────────────────────────────
        ui.separator();
        ui.collapsing("CG Sensitivity", |ui| {
            if let Some(ref result) = self.current_cg_sensitivity {
                let env = &result.cg_envelope;
                ui.label(format!(
                    "CG: {:.1}mm  ({:.0}% through envelope)",
                    env.current_x_mm, env.percent_through_envelope
                ));
                ui.label(format!(
                    "Forward limit: {:.1}mm  |  Aft limit: {:.1}mm",
                    env.forward_limit_x_mm, env.aft_limit_x_mm
                ));
                ui.label(format!(
                    "Static margin: {:.2} MAC",
                    result.baseline_static_margin_mac
                ));

                ui.separator();
                ui.label("Component influence:");
                for comp in &result.component_sensitivities {
                    ui.horizontal(|ui| {
                        let bar_w = (comp.influence_fraction * 160.0).max(4.0);
                        let (rect, _) = ui.allocate_exact_size(
                            egui::vec2(bar_w, 10.0),
                            egui::Sense::hover(),
                        );
                        ui.painter().rect_filled(
                            rect, 2.0, egui::Color32::from_rgb(100, 150, 220),
                        );
                        ui.label(format!(
                            "{}: {:.0}%",
                            comp.component_name,
                            comp.influence_fraction * 100.0
                        ));
                    });
                }

                if !result.recommendations.is_empty() {
                    ui.separator();
                    ui.collapsing("Recommendations", |ui| {
                        for rec in &result.recommendations {
                            ui.label(format!("• {}", rec));
                        }
                    });
                }

                if ui.button("Clear CG Sensitivity").clicked() {
                    self.current_cg_sensitivity = None;
                }
            } else {
                if ui.button("Run CG Sensitivity").clicked() {
                    let mp = self.mass_points.clone();
                    if !mp.is_empty() {
                        let comps: Vec<(String, glam::Vec3, f32)> = mp
                            .iter()
                            .map(|m| (m.name.clone(), m.position, m.mass_g))
                            .collect();
                        let np_x = self
                            .current_neutral_point
                            .as_ref()
                            .map(|np| np.neutral_point_x_mm)
                            .unwrap_or(100.0);
                        let mac = 30.0_f32;
                        let fwd = np_x - 0.25 * mac;
                        let dims = indexmap::IndexMap::new();
                        self.current_cg_sensitivity = Some(
                            crate::analysis::compute_cg_sensitivity(&comps, &dims, np_x, mac, fwd),
                        );
                    } else {
                        self.error_message = Some(
                            "No mass points found. Use mass_point() in your script.".to_string(),
                        );
                    }
                }
                ui.label("Requires mass_point() calls in script.");
            }
        });

        // ── Interference Check (Phase 30) ────────────────────────────────────
        ui.separator();
        ui.collapsing("Interference Check", |ui| {
            if let Some(ref result) = self.current_interference_result {
                if result.has_critical_interference {
                    let crit_count = result
                        .pairs
                        .iter()
                        .filter(|p| p.severity == crate::analysis::InterferenceSeverity::Critical)
                        .count();
                    ui.colored_label(
                        egui::Color32::RED,
                        format!("WARNING: {} critical interference(s)!", crit_count),
                    );
                } else if result.total_interference_count == 0 {
                    ui.colored_label(egui::Color32::GREEN, "No interference detected");
                } else {
                    ui.colored_label(
                        egui::Color32::YELLOW,
                        format!("{} interference(s) found", result.total_interference_count),
                    );
                }

                for pair in &result.pairs {
                    let color = match pair.severity {
                        crate::analysis::InterferenceSeverity::Critical  => egui::Color32::RED,
                        crate::analysis::InterferenceSeverity::Moderate  => egui::Color32::from_rgb(220, 120, 0),
                        crate::analysis::InterferenceSeverity::Minor     => egui::Color32::YELLOW,
                        crate::analysis::InterferenceSeverity::Negligible => egui::Color32::GRAY,
                    };
                    ui.colored_label(color, &pair.description);
                }

                if !result.outside_parent.is_empty() {
                    ui.separator();
                    ui.label("Components outside parent structure:");
                    for name in &result.outside_parent {
                        ui.colored_label(egui::Color32::RED, format!("  WARNING: {}", name));
                    }
                }

                if ui.button("Clear Interference Results").clicked() {
                    self.current_interference_result = None;
                }
            } else {
                ui.label("Interference check runs via script.");
                ui.label("Call interference_check(names, keepouts, parent) in script.");
                ui.label("Or interference_check_no_parent(names, keepouts).");
            }
        });

        ui.separator();
        if ui.button("Clear All Aero Results").clicked() {
            self.current_lifting_line_result = None;
            self.current_flight_condition    = None;
            self.current_neutral_point       = None;
            self.current_static_margin       = None;
            self.current_trim_result         = None;
            self.current_drag_polar          = None;
            self.current_cg_sensitivity      = None;
            self.current_interference_result = None;
        }
    }
}
