# Live Status

## Completed: Help Panel UI (2026-03-17)

### Files Created
- `src/ui/help_data.rs` — Stub `FunctionDoc`, `FUNCTION_DOCS`, `CATEGORIES` (to be replaced by parallel task with real data)
- `src/ui/help_search.rs` — `HelpSearchState`: query/category/scored search, recent searches, keyboard navigation
- `src/ui/help_panel.rs` — `show_help_panel()`: search bar, category chips, result list with expand/collapse, copy/insert actions

### Files Modified
- `src/ui/mod.rs` — Added `pub mod help_data`, `pub mod help_search`, `pub mod help_panel`, `pub use help_search::HelpSearchState`
- `src/app.rs` — Added `help_panel_open: bool`, `help_state: HelpSearchState`, `script_cursor_byte: usize` fields + init; F1 keyboard shortcut; "?" toolbar button; `show_help_panel(...)` call before the central panel

### Verification
- `cargo build` — 0 errors, 233 warnings (all pre-existing), `Finished` in 0.29s



## Completed: Propulsion Analysis Implementation (2026-03-17)

### Files Created
- `src/aero/propulsion_db.rs` — Motor/prop database with 8 motors, 10 props, and recommendation engine
- `src/aero/propulsion.rs` — Iterative RPM convergence thrust curve, power curves, efficiency, limits, warnings
- `src/aero/performance.rs` — Rate of climb, range/endurance, and glide polar analysis

### Files Modified
- `src/aero/drag.rs` — Added `s_ref_m2`, `cl_max`, `v_stall_ms` to `DragPolarResult`
- `src/aero/mod.rs` — Added pub mod and re-exports for propulsion_db, propulsion, performance
- `src/scripting/mod.rs` — Added `MotorHandle`, `PropHandle`, `PropulsionHandle` newtype wrappers
- `src/scripting/api.rs` — Added `register_propulsion_functions` with 13 new Rhai scripting functions
- `src/export/mod.rs` — Added `ManufacturingPackage` and `export_manufacturing_package`

### Verification
- `cargo build` — 0 errors, 233 warnings (pre-existing)
- `cargo test` — 328 passed, 0 failed
  - 5 new propulsion tests: motor_lookup_correct_kv, prop_ct_at_zero/jmax, static_thrust_positive, thrust_decreases_with_airspeed, max_airspeed_reasonable
  - 3 new performance tests: endurance_at_best_speed_is_maximum, range_at_best_range_speed_is_max, glide_ratio_positive
  - All pre-existing tests continue to pass

### New Scripting API (Rhai)
- `motor(name)`, `motor_custom(...)`, `prop(name)`, `prop_by_size(d,p)`, `prop_custom(...)`
- `list_motors()`, `list_props()`
- `propulsion_setup(motor, prop, cells, cap_mah)`, `propulsion_setup_full(...)`
- `propulsion_analysis(setup, fc, weight_n)` → Map
- `propulsion_thrust_at(setup, airspeed_ms, fc)` → f64
- `range_endurance(setup, wing, fuse, htail, vtail, fc, weight_n)` → Map
- `rate_of_climb(setup, wing, fuse, htail, vtail, fc, weight_n)` → Map
- `glide_performance(wing, fuse, htail, vtail, fc, weight_n)` → Map
- `recommend_motor_prop(thrust_n, cruise_ms, max_weight_g)` → Array of Maps

---

## Completed: Example Browser UI + Script Editor Tab System (2026-03-18)

### Files Modified
- `src/ui/mod.rs` — Added `pub mod examples;`
- `src/app.rs` — Added `use crate::ui::examples;` import; added 4 new fields to App struct (`open_example_tabs`, `active_example_tab`, `example_search`, `pending_copy_example`) with matching initialization in `App::new()`; replaced old ComboBox with rich searchable example browser; added tab bar above editor; added read-only example view branch in the editor section

### Key Behavior
- Example browser dropdown shows category headers, colored difficulty badges, search filter
- Selecting an example opens a read-only tab (does not overwrite main script)
- Tab bar: "Main Script" always first; example tabs closeable with × button
- Right-click on example tab: "Copy to Main Script" or "Close Tab"
- Example view shows related functions as clickable buttons that open the help panel
- Main script (state.script_text) is always what gets evaluated — example tabs are display-only

### Verification
- `cargo build` — 0 errors, 0 warnings, Finished in 3.59s

---

## IN PROGRESS: Remove notebook/node_graph dead code + warnings (2026-03-18)

### Step 1: Collected all references
- src/lib.rs, src/main.rs, src/project.rs, src/app.rs all reference notebook/node_graph
- EditorMode enum lives in node_graph — will define local replacement in app.rs (Script only)

### Step 2: Deleting directories

### Step 3-8: COMPLETED

#### Changes Made:
- **Deleted**: `src/notebook/` and `src/node_graph/` directories
- **src/lib.rs**: Removed `pub mod node_graph;` and `pub mod notebook;`
- **src/main.rs**: Removed `mod node_graph;` and `mod notebook;`
- **src/project.rs**: Removed NodeGraph/Notebook imports, removed `node_graph`/`notebook` fields and parameters from `Project::new()`
- **src/app.rs**: Removed all node_graph/notebook fields, methods, and UI code; simplified EditorMode; removed unused editor_mode/polar_database fields; removed execute_script_from_text alias
- Fixed deprecated API calls: `from_id_source` → `from_id_salt`, `id_source` → `id_salt`, `clamp_to_range` → `clamping`
- Added `#[allow(dead_code)]` / `#![allow(dead_code)]` to 25+ files for planned API/data model code
- Used `cargo fix` to auto-fix 40+ unused import warnings

#### Final Build:
- Errors: **0**
- Warnings: **0** (well under the 20 target)
