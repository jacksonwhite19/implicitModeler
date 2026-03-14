```
Implement full automated FEA integration using CalculiX. This is a complete pipeline from geometry to stress field, triggered by a single UI action.

Part 1 — Rhai boundary condition API. Add the following functions to a new register_fea_functions() call in src/scripting/api.rs. These define FEA setup within the script and are collected by a FEA-specific script collector analogous to the existing mass/component collectors.

fixed_support(region: SdfHandle, name: &str) — marks a region of the geometry as fully fixed (zero displacement in all axes). Any mesh node whose position is inside region.distance(p) < 0.0 receives an ENCASTRE boundary condition in the CalculiX input.

fixed_axis(region: SdfHandle, name: &str, x: bool, y: bool, z: bool) — partial fixity, constrains only the specified axes. Useful for symmetry planes and pin joints.

force_load(region: SdfHandle, name: &str, fx: f64, fy: f64, fz: f64) — applies a distributed force in Newtons over the region surface. Total force magnitude is (fx, fy, fz), distributed evenly across all nodes in the region.

pressure_load(region: SdfHandle, name: &str, magnitude: f64) — applies a normal pressure load in Pascals to the region surface. Positive magnitude pushes inward.

gravity_load(gx: f64, gy: f64, gz: f64) — applies gravitational body force in m/s² to the entire model. For standard gravity use gravity_load(0.0, 0.0, -9810.0) in mm units.

torque_load(region: SdfHandle, name: &str, axis_x: f64, axis_y: f64, axis_z: f64, magnitude: f64) — applies a moment load in N·mm about the given axis at the centroid of the region. Implemented as a couple of equal and opposite forces distributed across the region surface.

motor_thrust(region: SdfHandle, name: &str, thrust_n: f64, torque_nmm: f64, direction_x: f64, direction_y: f64, direction_z: f64) — drone-specific convenience wrapper that combines a force_load in the thrust direction with a torque_load about the same axis. Covers the full load state of a single motor mount.

Store all collected FEA conditions in FEASetup on App:

pub struct FEASetup {
    pub fixed_supports: Vec<FEARegion>,
    pub fixed_axes: Vec<FEAAxisRegion>,
    pub force_loads: Vec<FEAForceRegion>,
    pub pressure_loads: Vec<FEAPressureRegion>,
    pub gravity: Option<Vec3>,
    pub torque_loads: Vec<FEATorqueRegion>,
    pub motor_thrusts: Vec<FEAMotorRegion>,
}

Serialize FEASetup into the .ntop project file.

Part 2 — Viewport visualization of boundary conditions. When FEA conditions exist in the script, visualize them as overlays in the 3D viewport. Do not modify the raymarching shader for this — render them as egui_wgpu geometry overlays drawn after the raymarching pass.

Fixed supports render as a blue transparent surface overlay on the constrained region. Force loads render as yellow arrows at the region centroid, scaled to force magnitude, pointing in the force direction. Pressure loads render as a red transparent surface overlay. Motor thrust regions render as green arrows for thrust and a circular arc arrow for torque. Gravity renders as a single large grey downward arrow in the corner of the viewport. Add a toggle Show FEA Conditions in the FEA panel to enable or disable these overlays.

Part 3 — CalculiX bundling. Bundle the CalculiX ccx binary for the target platform in assets/calculix/. Add a function in src/fea/calculix.rs to locate the bundled binary relative to the executable path. On startup verify the binary exists and is executable, log a warning if not. Add a CalculiX path override in application settings (src/settings.rs) that lets the user point to a different binary if needed.

Part 4 — FEA pipeline. Implement the full pipeline in src/fea/pipeline.rs as an async operation that runs on a background thread so the UI remains responsive.

Step 1 — mesh extraction. Extract a mesh from the current SDF using adaptive_mc at a fixed FEA quality level (target_cell_size = 0.3 or user-configurable). This mesh must be manifold and watertight. Add a mesh validation step that checks for open edges and degenerate elements and returns a clear error if the mesh is not suitable for FEA.

Step 2 — material assignment. Use a default linear elastic isotropic material appropriate for PLA/PETG 3D printed parts: E = 2100 MPa (Young's modulus), nu = 0.35 (Poisson's ratio), density = 1.24e-9 tonne/mm³. Expose these as configurable fields in the FEA panel: Material Preset dropdown with PLA, PETG, ABS, Carbon Fiber (unidirectional), and Custom. Custom allows direct entry of E, nu, and density.

Step 3 — .inp file generation. Write a CalculiX .inp file to a temporary directory. The file must contain: node definitions (*NODE), element definitions as C3D4 linear tetrahedra (*ELEMENT, TYPE=C3D4), a single material definition (*MATERIAL, *ELASTIC, *DENSITY), element set and surface set definitions for each named FEA region, boundary conditions derived from fixed_support and fixed_axis regions (*BOUNDARY), load steps for all force, pressure, gravity, and torque loads (*STEP, *CLOAD, *DLOAD, *DFLUX), and output requests for nodal stress (*NODE FILE, S) and displacement (*NODE FILE, U).

Step 4 — CalculiX execution. Spawn the bundled ccx binary as a subprocess using std::process::Command. Capture stdout and stderr. Stream stderr to a log panel in the UI in real time so the user can see solver progress. Wait for completion. If the exit code is nonzero, parse the stderr for the first ERROR line and surface it as a clear error message in the UI.

Step 5 — result parsing. Parse the .frd output file from CalculiX. Extract nodal Von Mises stress values and nodal displacement magnitudes. Interpolate both onto the SDF grid using inverse distance weighting from the nearest mesh nodes to each voxel center. Register the results as two named fields on App: fea_stress_field: Option<Arc<dyn Field>> and fea_displacement_field: Option<Arc<dyn Field>>. These are immediately available in Rhai as stress_field() -> FieldHandle and displacement_field() -> FieldHandle after a successful FEA run.

Part 5 — FEA results visualization. Add a Results Overlay section to the FEA panel. After a successful run show: maximum Von Mises stress in MPa, maximum displacement in mm, a safety factor estimate based on material yield strength (yield strengths: PLA 50MPa, PETG 45MPa, ABS 40MPa, Carbon Fiber 600MPa). Add a stress overlay mode to the viewport that colors the surface by Von Mises stress using the same color map infrastructure as the wall thickness analysis (red = at or above yield, green = below 25% of yield, smooth interpolation between). Add a displacement overlay mode that colors by displacement magnitude. Both overlays work correctly in combination with section view.

Part 6 — Rhai integration example. Add the following as a documented example script showing the complete FEA-driven design workflow:

let fuse = fuselage([station(0.0, ellipse(0.3, 0.3)), station(0.5, ellipse(1.0, 0.8)), station(1.0, ellipse(0.3, 0.3))]);
let arm = motor_arm(fuse, 0.0, 150.0, 12.0, 9.0);
let mount = motor_mount(arm, 16.0, 3.0, 15.0, 3.0);
let structure = union(union(fuse, arm), mount);
fixed_support(sphere_at(0.0, 0.0, 0.0, 20.0), "fuselage_root");
motor_thrust(sphere_at(150.0, 0.0, 0.0, 15.0), "motor_0", 12.0, 800.0, 0.0, 0.0, 1.0);
gravity_load(0.0, 0.0, -9810.0);
let lattice = conformal_gyroid_field(structure, 4.0, 0.8, multiply_fields(stress_field(), constant_field(0.5)));
union(shell(structure, 1.5), lattice)

This example shows stress-field-driven lattice grading — after FEA runs, re-running the script produces a lattice that is denser where stress is highest.

Add integration tests for the .inp file generator that verify correct node count, element count, boundary condition application, and load step format without requiring CalculiX to be present.
```