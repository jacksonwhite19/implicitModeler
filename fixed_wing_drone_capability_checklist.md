# Fixed-Wing Drone Capability Checklist

Use this file as the working build sheet for making the app credible for end-to-end design of an 800 mm fixed-wing drone.

Key:
- `[ ]` not started
- `[/]` partial
- `[x]` complete
- `[n]` not needed

## Summary

The main conclusion from the review is:

- Several important capabilities already exist, but are fragmented across scripting, UI, examples, and headless paths.
- The highest-value work is not "add more random features." It is consolidating existing capability into coherent workflows the user can trust.
- The true top-priority build items are:
  - Parametric full-aircraft workflow
  - Strong project-level dimensions/configs
  - Reliable export pipeline
  - End-to-end workflow tests
  - Electronics installation helpers
  - Control system design helpers
  - Shared GUI/headless pipeline everywhere
  - Hardware presets by manufacturing style

## Bucket A: Must Build / Highest Priority

- [x] Parametric full-aircraft workflow
  - Goal: one coherent path for fuselage, wing, tail, control surfaces, internals, mounts, splits, and export without stitching together ad hoc scripts.
- [x] Strong project-level dimensions/configs
  - Goal: one place to drive span, chord, battery size, servo choice, spar size, and other top-level parameters with reliable regeneration.
- [x] Reliable export pipeline
  - Goal: project to mesh/STL/3MF/STEP-equivalent outputs with consistent units, bounds, and part naming.
- [x] End-to-end workflow tests
  - Goal: representative full-aircraft examples that load, regenerate, export, and pass expected checks.
- [x] Electronics installation helpers
  - Goal: servo trays, motor firewall patterns, FC stacks, battery cradles, antenna mounts, and hatch systems.
- [x] Control system design helpers
  - Goal: hinge geometry, horn placement, pushrod routing, servo throw estimation, and linkage interference checks.
- [x] Shared GUI/headless pipeline everywhere
  - Goal: both execution paths evaluate and export identically.
- [x] Hardware presets by manufacturing style
  - Goal: foamboard, LW-PLA shell, carbon tube spar, balsa hybrid, and molded shell presets.

## Bucket B: Already Exists But Needs Consolidation

- [x] Assembly constraints
  - Exists today: placement, mirroring, reference points, `attach_to_*`, dimension-driven regeneration.
  - Consolidated: lightweight project-level assembly rules and auto-apply relationships are now part of the workflow config and variant flow.
- [x] Print/manufacturing readiness checks
  - Exists today: wall-thickness query, overhang analysis, split-fit tools, tolerance compensation, fastener helpers.
  - Consolidated: unified manufacturing summary in the workflow panel with pass/warn/fail status, wall checks, overhang totals, and issue counts.
- [x] Structural layout tools
  - Exists today: spars, ribs, bulkheads, shells, composite and reinforcement-oriented tooling.
- [x] CG and balance workflow
  - Exists today: mass points, CG, CG sensitivity, neutral point, static margin, required CG range.
  - Consolidated: project-level flight summary now rolls up component mass, CG, static margin, trim, glide ratio, and glide speed in one place.
- [x] Airframe partitioning tools
  - Exists today: split-body UI, alignment features, multi-split, pins/sockets, groove, dovetail, bolt-hole alignment.
- [x] Better help/status signaling
  - Exists today: README status section, help-panel status labels, example maturity labels.
  - Consolidated: high-visibility workflow helpers are explicitly tagged stable, experimental, or legacy in help data.
- [x] Flight envelope estimates
  - Exists today: propulsion analysis, drag polar, range/endurance, rate of climb, glide performance, stability helpers.
  - Consolidated: these analyses now feed the project-level flight summary workflow instead of living only as scattered standalone tools.
- [x] Versioned design variants
  - Exists today: dimensions, projects, internal version-control concepts.
  - Consolidated: workflow panel now supports named design variants with save/apply/delete behavior.

## Bucket C: Not Needed

- [n] Real component library
- [n] Internal packaging validation
- [n] Wiring and routing model
- [n] BOM output
- [n] Canonical aircraft templates
- [n] Cost estimation and weight budget rollup
- [n] Guided design wizard for non-expert users

## Bucket D: Explicit Cleanup Deferred

- [x] Shrink and organize `src/scripting/api.rs`
- [x] Remove or fully retire the leftover in-file legacy block in `src/scripting/api.rs`
- [x] Make help, docs, and examples reflect actual maturity consistently across UI and README
- [x] Add at least 2-3 gold-standard aircraft examples that prove the tool can do a full design coherently

These cleanup tasks are now complete and folded into the current fixed-wing workflow baseline.

## Build Plan

Build order is from highest leverage to lowest. Each phase should land in a usable state before moving on.

### Phase 1: Parametric Aircraft Workflow

- [x] Create a canonical fixed-wing project script/template that defines:
  - fuselage
  - wing
  - horizontal tail
  - vertical tail
  - control surfaces
  - key internal components
  - printable splits
- [x] Define the minimum parameter set for the template:
  - wingspan
  - root chord
  - tip chord
  - fuselage length
  - fuselage diameter or station set
  - tail moment
  - battery envelope
  - servo envelope
  - spar size
- [x] Ensure the template works in both GUI and headless paths.
- [x] Add a "fixed-wing aircraft" starting workflow in the app, either as:
  - a new example
  - a template
  - or a new project preset
- [x] Define success criteria:
  - changing a top-level dimension regenerates the whole aircraft correctly
  - geometry remains valid
  - export still works

### Phase 2: Project-Level Dimensions and Configs

- [x] Audit current dimensions workflow and document the current source of truth.
- [x] Separate global aircraft dimensions from incidental script locals.
- [x] Add a structured config grouping for at least:
  - airframe
  - structure
  - control system
  - manufacturing preset
- [x] Ensure dimension/config changes trigger consistent re-evaluation.
- [x] Make saved `.icad` projects preserve the full parameter/config state cleanly.
- [x] Define success criteria:
  - a user can reopen a project and get the same geometry
  - headless overrides work predictably
  - parameter names are stable and readable

### Phase 3: Shared Export Pipeline

- [x] Audit all export paths and identify remaining GUI/headless drift.
- [x] Route all export operations through the shared pipeline layer where possible.
- [x] Standardize:
  - units
  - bounds selection
  - output naming
  - part naming
  - metrics output
- [x] Verify split-part export behaves the same between GUI and headless.
- [x] Define success criteria:
  - same project and same dimensions produce the same output regardless of launch path

### Phase 4: End-to-End Workflow Tests

- [x] Add one canonical "complete 800 mm fixed-wing" integration test.
- [x] Add one canonical "split printable aircraft section" integration test.
- [x] Add one canonical "dimension change regenerates aircraft correctly" integration test.
- [x] Add one canonical "headless export from `.icad` project" integration test.
- [x] Add artifact-level assertions where realistic:
  - output file exists
  - metrics file exists
  - dimensions used are correct
  - key geometry extents stay in expected ranges

### Phase 5: Electronics Installation Helpers

- [x] Inventory what already exists:
  - motor mounts
  - servo/component placement
  - trays
  - hatches
  - access panels
- [x] Add any missing helpers required for the fixed-wing workflow:
  - servo tray helper
  - flight-controller mount helper
  - battery cradle helper
  - antenna or receiver mount helper
- [x] Make sure each helper is usable parametrically and works with splits/shells.
- [x] Add at least one integrated example using these helpers in a full aircraft.

### Phase 6: Control System Design Helpers

- [x] Inventory existing hinge/control-horn/pushrod functionality.
- [x] Add missing utilities for:
  - control horn placement
  - pushrod path clearance
  - servo-to-surface linkage geometry
  - basic throw estimation
- [x] Connect these helpers into the fixed-wing reference workflow.
- [x] Define success criteria:
  - a user can place and reason about basic actuation geometry without ad hoc script hacks

### Phase 7: Manufacturing Presets

- [x] Define the preset schema:
  - shell thickness
  - split preferences
  - alignment defaults
  - spar assumptions
  - tolerance compensation defaults
- [x] Add presets for:
  - foamboard
  - LW-PLA shell
  - carbon tube spar
  - balsa hybrid
  - molded shell
- [x] Make the fixed-wing template consume these presets.
- [x] Define success criteria:
  - changing preset updates manufacturing-oriented defaults without rewriting the design

### Phase 8: Consolidate Partial Capabilities

- [x] Assembly constraints
  - Landed a light project-level constraint model with workflow-panel control and optional auto-apply behavior.
- [x] Manufacturing checks
  - Added a consolidated manufacturability summary with workflow status, wall thickness, overhang totals, and issue counts.
- [x] CG and balance workflow
  - Added a direct project-level flight summary tied to component mass, CG, stability, trim, and glide metrics.
- [x] Help/status signaling
  - Expanded help status tagging for high-visibility workflow functions.
- [x] Flight envelope
  - Routed existing performance analysis outputs into the project-level workflow summary.
- [x] Design variants
  - Added named workflow variants with save/apply/delete UX.

## Immediate Execution Order

Work these in order unless a blocker forces reordering:

1. [x] Phase 1: Parametric Aircraft Workflow
2. [x] Phase 2: Project-Level Dimensions and Configs
3. [x] Phase 3: Shared Export Pipeline
4. [x] Phase 4: End-to-End Workflow Tests
5. [x] Phase 5: Electronics Installation Helpers
6. [x] Phase 6: Control System Design Helpers
7. [x] Phase 7: Manufacturing Presets
8. [x] Phase 8: Consolidate Partial Capabilities

## Current Recommended Next Step

- [ ] Choose the next net-new product claim to build beyond the current fixed-wing workflow baseline.
