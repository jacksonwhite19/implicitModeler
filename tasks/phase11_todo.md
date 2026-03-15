# Phase 11: Measurement Tools — Task Plan

## Steps

### Step 1 — `src/analysis/measurements.rs` data model + computations [>]
- MeasurementResults, CrossSectionMeasurement, PointDistanceMeasurement structs
- compute_model_properties() — single parallel SDF-grid pass (volume, SA, COM)
- measure_cross_section() — 2D parallel SDF sample at axial slice
- measure_distance() — trivial (b-a).length()

### Step 2 — Register in analysis/mod.rs

### Step 3 — App fields (measurements state, UI state, pick mode, viewport rect)

### Step 4 — Measure button in toolbar, floating egui::Window popup
- Section 1: Run Analysis, spinner, results, density input + presets
- Section 2: Cross-section axis/position/Measure, results list
- Section 3: Point A/B pick + coord inputs, save distances list
- Section 4: Show CG toggle (egui painter overlay in viewport)
- Section 5: Copy to Clipboard

### Step 5 — Viewport picking (ray-SDF march on click)
- When pick_mode active: intercept viewport click, unproject ray, march SDF grid

### Step 6 — Viewport overlays (egui painter)
- Draw picked points as colored circles
- Draw CG crosshair when enabled

### Step 7 — Stale state management
- Mark stale on script change; clear on re-run analysis; yellow banner

### Step 8 — cargo build passes, manual verification steps
