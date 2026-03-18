# Phase 16: Print Analysis System

## Steps
- [x] Step 1: Create src/analysis/print_analysis.rs (settings, surface points, overhang, orientation, features)
- [x] Step 2: Update src/analysis/mod.rs (pub mod print_analysis)
- [x] Step 3: Update src/project.rs (serialize PrintAnalysisSettings)
- [x] Step 4: Update src/app.rs (fields, toolbar button, panel, overlay upload)
- [x] Step 5: Build and run integration tests

## Acceptance Criteria
- Sphere Z-up → zero overhang area
- Flat plate → 100% overhang on bottom face
- Orientation advisor recommends Z-up for tall thin cylinder
- Wall 0.5mm flagged as ThinWall error with min_wall 1.2mm
