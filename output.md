# Codebase Audit — 2026-03-17

## 1. src/ File Tree

```
src/
├── aero/  [8 files]
│   ├── drag.rs (270 lines)
│   ├── flight_condition.rs (82 lines)
│   ├── inlet_analysis.rs (93 lines)
│   ├── lifting_line.rs (530 lines)
│   ├── mod.rs (18 lines)
│   ├── polar_data.rs (198 lines)
│   ├── polars.rs (277 lines)
│   └── stability.rs (470 lines)
├── analysis/  [5 files]
│   ├── aero/  [2 files]
│   │   ├── inlet_analysis.rs (93 lines)
│   │   └── mod.rs (5 lines)
│   ├── hole_detection.rs (306 lines)
│   ├── measurements.rs (333 lines)
│   ├── mod.rs (16 lines)
│   ├── print_analysis.rs (915 lines)
│   └── thickness.rs (168 lines)
├── app.rs (5497 lines)
├── components/  [3 files]
│   ├── library.rs (187 lines)
│   ├── mod.rs (265 lines)
│   └── registry.rs (226 lines)
├── export/  [1 file]
│   └── mod.rs (104 lines)
├── fea/  [8 files]
│   ├── calculix.rs (81 lines)
│   ├── frd.rs (175 lines)
│   ├── inp.rs (439 lines)
│   ├── meshing.rs (135 lines)
│   ├── mod.rs (16 lines)
│   ├── pipeline.rs (287 lines)
│   ├── setup.rs (144 lines)
│   └── viz.rs (122 lines)
├── geometry_analysis/  [3 files]
│   ├── cg_sensitivity.rs (352 lines)
│   ├── interference.rs (259 lines)
│   └── mod.rs (14 lines)
├── headless.rs (446 lines)
├── lib.rs (13 lines)
├── library/  [4 files]
│   ├── manager.rs (161 lines)
│   ├── metadata.rs (114 lines)
│   ├── mod.rs (12 lines)
│   └── thumbnail.rs (93 lines)
├── main.rs (128 lines)
├── materials/  [2 files]
│   ├── composite.rs (134 lines)
│   └── mod.rs (5 lines)
├── mesh/  [4 files]
│   ├── adaptive_mc.rs (430 lines)
│   ├── import.rs (305 lines)
│   ├── marching_cubes.rs (681 lines)
│   └── mod.rs (38 lines)
├── node_graph/  [4 files]
│   ├── codegen.rs (186 lines)
│   ├── mod.rs (16 lines)
│   ├── types.rs (268 lines)
│   └── ui.rs (546 lines)
├── notebook/  [4 files]
│   ├── codegen.rs (128 lines)
│   ├── mod.rs (9 lines)
│   ├── types.rs (230 lines)
│   └── ui.rs (434 lines)
├── project.rs (141 lines)
├── render/  [7 files]
│   ├── axes.rs (178 lines)
│   ├── camera.rs (121 lines)
│   ├── grid.rs (177 lines)
│   ├── mod.rs (15 lines)
│   ├── pipeline.rs (174 lines)
│   ├── raymarch.rs (436 lines)
│   └── wireframe.rs (196 lines)
├── scripting/  [3 files]
│   ├── api.rs (4235 lines)
│   ├── errors.rs (300 lines)
│   └── mod.rs (1531 lines)
├── sdf/  [48 files]
│   ├── aerospace/  [14 files]
│   │   ├── airfoil.rs (490 lines)
│   │   ├── composite.rs (369 lines)
│   │   ├── control_surfaces.rs (386 lines)
│   │   ├── fuselage.rs (320 lines)
│   │   ├── inlets.rs (269 lines)
│   │   ├── mechanical.rs (250 lines)
│   │   ├── mod.rs (65 lines)
│   │   ├── nacelle.rs (85 lines)
│   │   ├── nose_tail.rs (332 lines)
│   │   ├── section.rs (32 lines)
│   │   ├── stability_geometry.rs (157 lines)
│   │   ├── structural.rs (87 lines)
│   │   ├── structural_drone.rs (683 lines)
│   │   └── wing.rs (284 lines)
│   ├── field/  [6 files]
│   │   ├── arithmetic.rs (291 lines)
│   │   ├── gradients.rs (384 lines)
│   │   ├── lattice.rs (317 lines)
│   │   ├── mod.rs (25 lines)
│   │   ├── operations.rs (250 lines)
│   │   └── primitives.rs (165 lines)
│   ├── lattice/  [2 files]
│   │   ├── conformal.rs (327 lines)
│   │   └── mod.rs (3 lines)
│   ├── print/  [8 files]
│   │   ├── alignment.rs (369 lines)
│   │   ├── bracket.rs (315 lines)
│   │   ├── fasteners.rs (316 lines)
│   │   ├── joints.rs (411 lines)
│   │   ├── mod.rs (20 lines)
│   │   ├── panels.rs (397 lines)
│   │   ├── split.rs (345 lines)
│   │   └── tolerance.rs (261 lines)
│   ├── booleans.rs (248 lines)
│   ├── mesh_import.rs (570 lines)
│   ├── mod.rs (59 lines)
│   ├── patterns.rs (160 lines)
│   ├── primitives.rs (250 lines)
│   ├── profiles.rs (328 lines)
│   ├── query.rs (215 lines)
│   ├── spine.rs (239 lines)
│   ├── sweep.rs (462 lines)
│   └── transforms.rs (294 lines)
├── settings.rs (50 lines)
├── ui/  [12 files]
│   ├── autocomplete.rs (402 lines)
│   ├── dimensions.rs (609 lines)
│   ├── library_panel.rs (285 lines)
│   ├── mod.rs (31 lines)
│   ├── project_tree.rs (531 lines)
│   ├── project_wizard.rs (193 lines)
│   ├── script_variable_detector.rs (556 lines)
│   ├── spine_editor.rs (499 lines)
│   ├── spline_editor.rs (772 lines)
│   ├── syntax.rs (203 lines)
│   ├── templates.rs (154 lines)
│   └── version_control_panel.rs (851 lines)
├── undo.rs (341 lines)
└── version_control/  [2 files]
    ├── mod.rs (146 lines)
    └── operations.rs (757 lines)
```

**Total: 119 .rs files, ~43,500 lines**

---

## 2. Feature Implementation Status

### Lifting Line Theory — IMPLEMENTED
File: `src/aero/lifting_line.rs` (530 lines)

```rust
pub fn solve_lifting_line(
    wing_sdf: &Arc<dyn Sdf>,
    polar_db: &PolarDatabase,
    flight: &FlightCondition,
    n_stations: usize,
) -> LiftingLineResult

pub struct LiftingLineResult {
    pub cl_total: f32,
    pub cd_induced: f32,
    pub oswald_efficiency: f32,
    pub span_efficiency: f32,
    pub lift_total_n: f32,
    pub induced_drag_total_n: f32,
    pub lift_distribution: Vec<f32>,
    pub induced_aoa: Vec<f32>,
    pub effective_aoa: Vec<f32>,
    pub local_cl: Vec<f32>,
    pub stall_stations: Vec<usize>,
    pub stall_margin: Vec<f32>,
    pub span_stations: Vec<f32>,
    pub tip_stall_risk: bool,
    pub root_stall_first: bool,
}
```
Full Glauert Fourier series method. Geometric extraction from SDF. Gaussian elimination solver. Oswald efficiency. Spanwise distributions. Stall analysis.

---

### Static Stability and Neutral Point — IMPLEMENTED
File: `src/aero/stability.rs` (470 lines)

```rust
pub fn compute_neutral_point(
    wing_sdf: &Arc<dyn Sdf>,
    htail_sdf: &Arc<dyn Sdf>,
    fuselage_sdf: &Arc<dyn Sdf>,
    polar_db: &PolarDatabase,
    flight: &FlightCondition,
) -> NeutralPointResult

pub fn compute_static_margin(
    np: &NeutralPointResult,
    cg: Vec3,
    wing_mac: f32,
) -> StaticMarginResult

pub struct StaticMarginResult {
    pub static_margin_mac: f32,
    pub is_stable: bool,
    pub stability_category: StabilityCategory,
    pub cg_forward_limit_mm: f32,
    pub cg_aft_limit_mm: f32,
    pub pitch_stiffness: f32,
}

pub fn compute_trim(...) -> TrimResult
```
Wing/H-tail contributions, downwash factor, fuselage destabilizing effect. CG limits. Trim point solver.

---

### Drag Polar — IMPLEMENTED
File: `src/aero/drag.rs` (270 lines)

```rust
pub fn compute_drag_polar(
    wing_sdf: &Arc<dyn Sdf>,
    fuselage_sdf: &Arc<dyn Sdf>,
    htail_sdf: &Arc<dyn Sdf>,
    vtail_sdf: &Arc<dyn Sdf>,
    polar_db: &PolarDatabase,
    flight: &FlightCondition,
    weight_n: Option<f32>,
) -> DragPolarResult

pub struct CD0Breakdown {
    pub wing: f32,
    pub fuselage: f32,
    pub h_tail: f32,
    pub v_tail: f32,
    pub interference: f32,
    pub total: f32,
}
```
Zero-lift drag buildup by component. Schlichting turbulent skin friction. Parabolic polar. Best L/D and endurance speeds.

---

### Propulsion Analysis — PARTIAL
No dedicated propulsion module. Basic `motor_thrust()` registered in scripting API. No motor efficiency curves, propeller design, or battery/energy analysis.

---

### Control Authority — IMPLEMENTED
File: `src/sdf/aerospace/control_surfaces.rs` (386 lines)

```rust
pub enum ControlSurfaceType { Aileron, Elevator, Rudder, Flap, Elevon }

pub fn create_control_surface(
    wing: &Arc<dyn Sdf>,
    surface_type: ControlSurfaceType,
    span_start: f32,
    span_end: f32,
    chord_fraction: f32,
    hinge_spec: &HingeSpec,
    linkage_spec: &LinkageSpec,
) -> ControlSurfaceResult
```
Gap/hinge modeling. Deflection. Control horn and pushrod linkage integration.

---

### Range and Endurance — PARTIAL
Best glide speed (max L/D) and best endurance speed computed in `src/aero/drag.rs`. No full Breguet range/endurance tables or battery integration.

---

### Inlet Geometry (NACA and EDF) — IMPLEMENTED
File: `src/sdf/aerospace/inlets.rs` (269 lines)

```rust
pub struct NacaInlet {
    pub width: f32,
    pub length: f32,
    pub depth: f32,
    pub ramp_angle_deg: f32,
    pub position: Vec3,
    pub normal: Vec3,
    pub flow_direction: Vec3,
}

pub enum InletShape {
    Circular { diameter: f32 },
    Elliptical { width: f32, height: f32 },
    DShaped { width: f32, height: f32, flat_fraction: f32 },
}
// Also: chin_inlet, side_inlet, edf_duct, edf_duct_s, edf_exhaust, convergent_nozzle
```

---

### Haack/Von Karman Nose Primitives — IMPLEMENTED
File: `src/sdf/aerospace/nose_tail.rs` (332 lines)

```rust
pub struct HaackNose {
    pub length: f32,
    pub base_radius: f32,
    pub c_parameter: f32,
    profile: Vec<(f32, f32)>,  // 256-point precomputed
}
// Von Karman = c=0, LV-Haack = c=1/3
// Also: HaackTail, TangentOgive, EllipsoidNose
```

---

### Conformal Lattice — IMPLEMENTED
File: `src/sdf/lattice/conformal.rs` (327 lines)

```rust
pub struct ConformalGyroid {
    pub parent: Arc<dyn Sdf>,
    pub cell_size: f32,
    pub thickness: f32,
    pub density_field: Option<Arc<dyn Field>>,
    pub region_mask: Option<Arc<dyn Sdf>>,
}
// Also: ConformalDiamond, ConformalSchwarzP
```
Surface-aligned orthonormal frames via Newton iteration (28 SDF evals/query). Density field modulation. Spatial masking.

---

### Sweep Operation — IMPLEMENTED
File: `src/sdf/sweep.rs` (462 lines)

```rust
pub trait SweepPath: Send + Sync {
    fn evaluate(&self, t: f32) -> Vec3;
    fn tangent(&self, t: f32) -> Vec3;
    fn arc_length(&self) -> f32;
}

pub struct Sweep {
    pub path: Arc<dyn SweepPath>,
    pub profile: Arc<dyn Section2D>,
    pub twist_start: f32,
    pub twist_end: f32,
}
// Paths: LinePath, PolylinePath, SplinePath, SurfacePath
```
Rotation-minimizing (Bishop) frame. Optional linear twist.

---

### Mesh Import to SDF — IMPLEMENTED
File: `src/sdf/mesh_import.rs` (570 lines)

```rust
pub enum MeshSdfMode {
    Approximate,  // 64³ voxel grid, trilinear interpolation
    Accurate,     // BVH + generalized winding number
}

pub fn mesh_to_sdf(mesh: Arc<TriangleMesh>, mode: MeshSdfMode) -> Arc<dyn Sdf>
```

---

### Multi-Shell Composite Layup — IMPLEMENTED
Files: `src/sdf/aerospace/composite.rs` (369 lines), `src/materials/composite.rs` (134 lines)

```rust
pub struct ShellLayer {
    pub name: String,
    pub material: Arc<CompositeMaterial>,
    pub thickness: f32,
    pub thickness_field: Option<Arc<dyn Field>>,
    pub is_core: bool,
    pub core_infill: Option<Arc<dyn Sdf>>,
}

pub struct CompositeLayup {
    pub parent: Arc<dyn Sdf>,
    pub layers: Vec<ShellLayer>,
}
```
Outermost-to-innermost layer ordering. Per-layer thickness fields. Lattice infill for cores.

---

### Control Surfaces (Aileron, Elevator, Rudder) — IMPLEMENTED
File: `src/sdf/aerospace/control_surfaces.rs` (386 lines)

```rust
pub fn create_aileron(wing: &Arc<dyn Sdf>, span_start: f32, span_end: f32, chord_fraction: f32, hinge_spec: &HingeSpec, linkage_spec: &LinkageSpec) -> ControlSurfaceResult
pub fn create_elevator(htail: &Arc<dyn Sdf>, ...) -> ControlSurfaceResult
pub fn create_rudder(vtail: &Arc<dyn Sdf>, ...) -> ControlSurfaceResult
```

---

### Auto Bracket Generation — IMPLEMENTED
File: `src/sdf/print/bracket.rs` (315 lines)

```rust
pub enum BracketType {
    FlatPlate { plate_thickness: f32, tab_width: f32, tab_extension: f32 },
    Saddle { wall_thickness: f32, conform_radius: f32 },
    Cantilever { arm_thickness: f32, arm_width: f32, face: BracketFace },
    FullTray { wall_thickness: f32, floor_thickness: f32, open_face: BracketFace },
}

pub fn generate_bracket(keepout: &Arc<dyn Sdf>, bracket_type: &BracketType, mounting_holes: &[BracketHole], face: &BracketFace) -> BracketGeometry
```
Heat-set boss and clearance hole drilling. Keepout volume avoidance.

---

### Split Body with Alignment Features — IMPLEMENTED
Files: `src/sdf/print/split.rs` (345 lines), `src/sdf/print/alignment.rs` (369 lines)

```rust
pub enum AlignmentFeatureType {
    Dowels { diameter: f32, depth: f32 },
    Pins { diameter: f32 },
    Pockets { depth: f32 },
    Dovetail { angle: f32 },
}

pub fn split_with_alignment(body: &Arc<dyn Sdf>, plane: &SplitPlane, feature_type: AlignmentFeatureType, feature_spacing: f32) -> SplitResult
```

---

### Version Control System — IMPLEMENTED
Files: `src/version_control/mod.rs` (146 lines), `src/version_control/operations.rs` (757 lines), `src/ui/version_control_panel.rs` (851 lines)

```rust
pub struct CommitId(pub String);
// SHA-256-style hash, 8-char short display

pub struct Commit {
    pub id: CommitId,
    pub parent_ids: Vec<CommitId>,
    pub author: String,
    pub timestamp: chrono::DateTime<chrono::Utc>,
    pub message: String,
    pub state: ProjectState,
    pub thumbnail: Option<Vec<u8>>,
}

pub struct VersionControlState {
    pub commits: HashMap<CommitId, Commit>,
    pub branches: HashMap<String, Branch>,
    pub current_branch: String,
    pub head_commit_id: Option<CommitId>,
    pub detached_head: bool,
    pub working_changes: bool,
}
```
Operations: `commit`, `create_branch`, `checkout_branch`, `checkout_commit`, `discard_changes`, `merge` (3-way with conflict detection), `delete_branch`, `get_commit_graph`, `has_working_changes`. BFS merge base finder. Line-level diff. Branch indicator in title bar. 3-tab UI panel (Branches, History, Changes).

---

### CG Sensitivity — IMPLEMENTED
File: `src/geometry_analysis/cg_sensitivity.rs` (352 lines)

```rust
pub fn analyze_cg_sensitivity(
    components: &[(String, Vec3, f32)],
    dimensions: &IndexMap<String, f64>,
    neutral_point_x_mm: f32,
    wing_mac: f32,
    forward_limit_x_mm: f32,
) -> CgSensitivityResult

pub struct ComponentCgSensitivity {
    pub dcg_dx_mm_per_mm: f32,
    pub dcg_dy_mm_per_mm: f32,
    pub dcg_dz_mm_per_mm: f32,
    pub influence_fraction: f32,
    pub forward_limit_mm: f32,
    pub aft_limit_mm: f32,
}
```
Analytical partial derivatives ∂CG/∂position and ∂CG/∂dimension. CG envelope.

---

### Assembly Interference Check — IMPLEMENTED
File: `src/geometry_analysis/interference.rs` (259 lines)

```rust
pub fn check_assembly_interference(
    components: &[(String, Arc<dyn Sdf>, Arc<dyn Sdf>)],
    parent_sdf: Option<Arc<dyn Sdf>>,
    resolution: usize,
) -> InterferenceResult

pub enum InterferenceSeverity { Negligible, Minor, Moderate, Critical }
```
Voxel-based O(n²) overlap volume estimation. AABB pre-filtering. Rayon parallelism.

---

### Manufacturing Export Package — PARTIAL
File: `src/export/mod.rs` (104 lines). Basic STL export only. No nested part export, BOM generation, or assembly drawing output.

---

## 3. All Registered Rhai Functions (src/scripting/api.rs)

### Primitives
`sphere`, `box_`, `cylinder`, `torus`, `cone`, `plane`

### Booleans
`union`, `subtract`, `intersect`, `smooth_union`, `smooth_subtract`, `smooth_intersect`

### Transforms
`translate`, `rotate`, `scale`, `offset`, `shell`, `twist`, `bend`, `linear_array`, `polar_array`, `polar_array_axis`, `mirror_x`, `mirror_y`, `mirror_z`

### Assembly
`full_assembly`, `mirror_wing`, `mirror_assembly_y`, `mirror_assembly_x`, `mirror_assembly_z`

### Math Utilities
`to_rad`, `to_deg`, `clamp`, `lerp`

### Airfoil & Wing
`naca`, `wing_with_airfoil` (2 variants), `blend`, `fuselage_parametric`, `fuselage`, `rib_at_station`, `spar`, `wing_lattice`, `fuselage_lattice`, `fuselage_lattice_graded`

### Wing Measurements
`wing_span`, `wing_area`, `wing_mac`, `wing_aspect_ratio`, `wing_taper_ratio`, `wing_mac_location`, `wing_incidence`, `set_wing_incidence`

### Control Surfaces
`deflect`, `tail_volume_coefficients`, `size_horizontal_tail`, `size_vertical_tail`, `aileron`, `elevator`, `rudder`, `flap`, `elevon`, `wing_with_ailerons`, `wing_with_ailerons_custom`

### Nose / Tail Cones
`von_karman_nose`, `lv_haack_nose`, `haack_nose`, `tangent_ogive`, `ellipsoid_nose`, `haack_tail`, `von_karman_tail`, `nose_fuselage_union`, `tail_fuselage_union`

### Inlets
`naca_inlet`, `naca_inlet_surface`, `circular_inlet`, `elliptical_inlet`, `dshaped_inlet`, `chin_inlet`, `side_inlet`, `edf_duct`, `edf_duct_s`, `edf_exhaust`, `convergent_nozzle`, `inlet_performance`

### Structural Panels
`surface_panel`, `surface_panel_at_station`, `bulkhead_at_station`, `lightening_hole_pattern`, `rod_mount`, `motor_arm`, `motor_mount`, `generate_mounts`, `mount_with_bolts`

### Component System
`component`, `component_mass`, `component_named`, `place`, `geometry`, `keepout`, `mass_g`

### Fasteners & Holes
`cable_hole`, `bolt_circle`, `bolt_square`, `bolt_rect`, `countersink`, `counterbore`, `slot`, `chamfer_edge`, `thread_hole`

### Fuselage & Sections
`circle_section`, `ellipse_section`, `fuselage_station`, `lofted_fuselage`, `airfoil_from_points`, `airfoil_from_dat`, `auto_fuselage`

### Fields & Density
`constant_field`, `sdf_as_field`, `position_x_field`, `position_y_field`, `position_z_field`, `add_fields`, `multiply_fields`, `min_fields`, `max_fields`, `abs_field`, `gradient_field`, `radial_field`, `axial_radial_field`

### Lattice
`offset_by_field`, `shell_with_field`, `blend_by_field`, `gyroid`, `cubic_lattice`, `diamond_lattice`, `gyroid_with_field`, `conformal_gyroid`, `conformal_gyroid_field`, `conformal_gyroid_region`, `conformal_diamond`, `conformal_diamond_field`, `conformal_schwarz_p`

### Splines & Sweeps
`spline_fuselage`, `spline`, `spline_section`, `line_path`, `polyline_path`, `spline_path`, `surface_path`, `circle_profile`, `ellipse_profile`, `rect_profile`, `ngon_profile`, `sweep`, `sweep_twisted`

### Structural
`cable_channel`, `carbon_rod`, `control_rod`

### FEA & Loads
`fixed_support`, `fixed_axis`, `force_load`, `pressure_load`, `gravity_load`, `torque_load`, `motor_thrust`, `stress_field`, `displacement_field`

### Mesh Import
`import_mesh`, `import_mesh_scaled`, `import_mesh_transform`, `mesh_info`

### Composite Layup
`composite_layup`, `material`, `custom_material`, `shell_layer`, `shell_layer_field`, `core_layer`, `solid_core_layer`, `wing_composite`, `fuselage_composite`, `printed_shell`

### Print / Split
`split_x`, `split_y`, `split_z`, `split_plane`, `pins_and_sockets`, `tongue_and_groove`, `dovetail`, `bolt_holes`, `bolt_holes_countersunk`, `split`, `split_body_x`, `split_body_y`, `split_body_z`

### Joints
`magnet_retention`, `access_panel`, `battery_hatch`, `fc_access_panel`, `apply_joint_delta`, `dovetail_joint`, `finger_joint`, `press_fit`, `snap_connector`, `living_hinge_strip`, `simple_gap_hinge`, `rounded_hinge`

### Layup Config
`composite_layup_config`, `apply_layup`

### Control Linkage
`control_horn`, `control_horn_lower`, `pushrod_slot`, `horn_and_slot`

### Aerodynamic Analysis
`flight_condition`, `flight_condition_sl`, `dynamic_pressure`, `reynolds`, `get_polar`, `get_polar_re`, `cl_at`, `cd_at`, `cl_alpha`, `cl_max`, `alpha_stall`, `run_lifting_line`, `run_lifting_line_polar`, `neutral_point`, `static_margin`, `required_cg_range`, `trim_analysis`, `drag_polar`, `drag_polar_weighted`, `ld_max`, `best_glide_speed`

### Geometry Analysis
`cg_sensitivity`, `interference_check`, `interference_check_no_parent`

### Points & Geometry Queries
`point`, `+`, `-`, `dist`, `midpoint`, `lerp_point`, `direction`, `to_array`, `offset_point`, `offset_along`, `offset_toward`, `project_to_plane`, `translate_p`, `place_p`, `surface_point`, `surface_point_p`, `closest_point`, `closest_point_p`, `furthest_point`, `centroid`, `bbox_min`, `bbox_max`, `bbox_center`, `bbox_size`, `cross_section_center`, `ref_point` (dynamic), `get_ref` (dynamic)

### Wing Geometry Points
`leading_edge`, `trailing_edge`, `chord_point`, `wing_tip`, `wing_root`, `span_station`

### Placement
`place_at_ref` (dynamic), `mounting_hole` (2 variants), `auto_bracket`, `auto_bracket_flat`, `auto_bracket_saddle`, `auto_bracket_cantilever`, `auto_bracket_tray`, `auto_bracket_detect`, `place_behind`, `place_above`, `place_below`, `place_beside`, `attach_to_fuselage_station`, `attach_to_trailing_edge`

### Instancing
`instance`, `instance_grid`, `instance_along_path`

**Total: ~179 registered functions**

---

## 4. Compilation Warnings / Errors

Build result: **0 errors, 218 warnings** (all pre-existing dead-code / unused variable warnings — no functional issues).

No `unimplemented!()`, `todo!()`, or `panic!()` stubs found anywhere in the codebase.

---

## 5. Approximate Line Count

| Module | Lines |
|--------|-------|
| scripting/api.rs | 4,235 |
| app.rs | 5,497 |
| ui/ (all) | ~5,035 |
| sdf/ (all) | ~9,500 |
| aero/ (all) | ~1,920 |
| fea/ (all) | ~1,399 |
| analysis/ (all) | ~1,738 |
| mesh/ (all) | ~1,454 |
| geometry_analysis/ | ~625 |
| version_control/ | ~903 |
| render/ | ~1,297 |
| node_graph/ | ~1,016 |
| notebook/ | ~801 |
| scripting/mod.rs + errors.rs | ~1,831 |
| components/ | ~678 |
| library/ | ~380 |
| materials/ | ~139 |
| other | ~750 |
| **Total** | **~43,500** |

---

## Summary

| Area | Status |
|------|--------|
| Lifting line theory | Implemented |
| Static stability / neutral point | Implemented |
| Drag polar | Implemented |
| Propulsion analysis | Partial (no motor/prop module) |
| Control authority | Implemented |
| Range and endurance | Partial (speeds only, no Breguet tables) |
| Inlet geometry (NACA, EDF) | Implemented |
| Haack/Von Karman nose | Implemented |
| Conformal lattice | Implemented |
| Sweep operation | Implemented |
| Mesh import to SDF | Implemented |
| Multi-shell composite layup | Implemented |
| Control surfaces | Implemented |
| Auto bracket generation | Implemented |
| Split body with alignment | Implemented |
| Version control system | Implemented |
| CG sensitivity | Implemented |
| Assembly interference check | Implemented |
| Manufacturing export package | Partial (STL only, no BOM/nested export) |
