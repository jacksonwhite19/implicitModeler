# Pre-Export and Pre-CFD Gate Catalog

This catalog lists early gates intended to reject obviously poor candidates before expensive export, meshing, or CFD.

## Pre-Definition Gates

These run before geometry generation. They use the design vector plus non-exported virtual components.

- Variable schema bounds.
- Normalized design vector validity.
- Planform area bounds.
- Aspect ratio bounds.
- Taper ratio bounds.
- Sweep bounds.
- Mean chord bounds.
- Minimum tip chord.
- Span/root chord ratio.
- Wing loading proxy.
- Virtual component mass estimate.
- Virtual component CG estimate.
- Target CG error after movable component placement.
- Export risk score.
- CFD design-rule limits from `cfd_geometry_design_rules.md`, including early
  limits for taper, sweep, tip chord, aspect ratio, and local feature size.

Virtual components are not geometry. They are pre-definition engineering assumptions used for mass/CG and packaging screening. Current v0.1 components assume mostly 3D-printed construction: printed wing shell/ribs/spar, printed fuselage shell/bulkheads, battery, payload, avionics, and EDF propulsion.

The internal layout model is configurable. It records:

- print shell areal density
- rib spacing
- spar allowance
- bulkhead count
- fuselage usable length
- fuselage inner diameter
- component masses
- component dimensions
- component keepout from the inner fuselage surface
- movable x-ranges for battery and avionics
- fixed EDF station
- fixed payload station
- target CG station

The layout pass checks whether internal components can fit in the fuselage envelope, including each component's keepout from the inner fuselage surface, and then moves movable components along their allowed x-ranges to reduce CG error. This is a screening approximation, not a detailed packaging CAD model.

Static margin is not calculated or gated at this stage. It requires an aerodynamic neutral point from later CFD/stability analysis. After that neutral point is available, the optimizer may rerun layout placement to shift movable components and improve static margin where possible. EDF remains fixed.

See `post_cfd_stability_layout_contract.md` for the post-CFD adjustment path.

## Pre-Export Geometry-Definition Gates

These should run after a parameterized aircraft definition is assembled but before STL export.

- Required reference frames exist.
- Required named features exist.
- Bounding box is within campaign limits.
- Wing root/tip stations are ordered correctly.
- Candidate has no known impossible packaging conflict.
- Expected mesh cell count is below campaign budget.
- Expected narrow-feature risk is below threshold.
- Geometry-provider parameter trace is complete.
- Trailing edges satisfy minimum thickness/radius policy.
- Leading edges satisfy minimum radius policy.
- Wing/fuselage and tail/fuselage blends are large enough for the target
  surface edge length.
- Inlet lip/cap fairing radii and clearances are large enough for meshing.
- No local CFD feature is smaller than the configured minimum feature size
  unless it is explicitly marked for local refinement or removed from the CFD
  OML.

## Post-Export Pre-CFD Gates

These run after STL export and before CFD case generation.

- STL exists and is hashable.
- Triangle and vertex counts are within limits.
- Boundary edge count is zero for CFD-ready exports.
- Nonmanifold edge count is zero.
- Connected component count is one unless policy explicitly allows more.
- Duplicate triangle count is zero.
- Degenerate triangle count is below threshold.
- Normals are consistently oriented.
- Bounding box and units match the candidate definition.
- Section audit has no long-chord anomalies.
- Mesh size and triangle/vertex count fit the CFD budget.
- High-aspect and edge-length metrics are within CFD-readiness policy.
- BL surface gate metrics are within policy or have an explicit repair/remesh
  path. Current diagnostics include triangle quality, minimum angle, aspect
  ratio, minimum altitude, and bad-triangle feature clusters.

## Pre-CFD Analysis Gates

These run before solver launch.

- Low-fidelity required lift coefficient is plausible.
- Estimated stall speed is within mission limit.
- L/D estimate is above minimum.
- Static margin is within campaign policy after aerodynamic neutral point is available.
- Required thrust/power proxy is within propulsion budget.
- Mesh/cell count fits runtime budget.
- Solver setup has valid freestream, reference area, reference length, and boundary names.
