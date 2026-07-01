# SDF Conditioning Backend

Status: kernel-integrated conditioning state and live SDF graph metadata
foundation.

The conditioning effort belongs to the core SDF backend. Exporters, meshers,
visualization, automation, and future simulation tools are downstream clients
of the same geometry kernel.

## Current Anchor

Backend code:

- `src/sdf/conditioning.rs`

Kernel export:

- `src/sdf/mod.rs`

The first implementation provides:

- live SDF-node metadata through `Sdf::metadata()`,
- explicit support-bound quality labels: exact, conservative, estimated, and
  unknown,
- support bounds propagated by primitives, transforms, booleans, patterns, field
  operations, and composite wrappers,
- cross-section support bounds through `Section2D::bounds_2d()`,
- centerline support bounds through `SweepPath::support_bounds()`,
- live support metadata for sweeps, lofted wings, lofted fuselages, inlet
  primitives, and circular/elliptical/profile duct nodes,
- conservative support metadata for twist and bend transforms derived from
  child support invariants,
- feature ownership tagging through `FeatureTaggedSdf`,
- dependency links between composed SDF nodes,
- dirty-region bounds,
- dirty-region source metadata,
- conditioning policy thresholds,
- local validation mode for real-time cache-quality validation versus
  full-comparison regression validation,
- a dense-region sample budget for splitting oversized reconditioning
  components,
- cache state/status vocabulary,
- `ConditionedGeometryKernel` as the live backend owner of the canonical SDF
  graph plus its conditioned cache,
- `ConditionedGeometryModel` as kernel-owned live conditioning state,
- connected-region cache generation and regeneration for full and local
  updates,
- trilinear conditioned-distance queries over generated cache blocks,
- sparse derived surface-refinement cells inside conditioned blocks for
  near-interface OML fidelity on coarse runtime caches,
- split surface-refinement storage: raw fine cells preserve the published
  zero-contour, while separately reinitialized fine cells provide an explicit
  trusted finite-distance band,
- kernel distance queries that prefer the conditioned cache and fall back to the
  canonical graph outside the cache domain or inside the published fidelity
  band,
- conditioned gradient, surface-normal, and projection-to-surface queries,
- first-class conditioned-kernel inside/outside, closest-surface-point, and
  ray-intersection queries,
- first-class conditioned-kernel local feature size, wall thickness,
  curvature, principal curvature, cross-section, volume, wetted-area, and
  frontal-area queries,
- sign-preserving runtime distance semantics so the conditioned cache cannot
  override the canonical graph's inside/outside classification,
- canonical fallback projection when cache-based projection cannot converge,
- narrow-band fast-sweeping reconditioning for generated region samples,
- sparse narrow-band fallback for oversized regions that exceed the dense
  region sample budget,
- edge-crossing interface anchors derived from canonical SDF samples,
- projection/gradient-corrected near-interface anchors derived from
  `|phi| / |grad phi|`,
- per-block reconditioning statistics for anchor count, iteration count, and
  maximum sweep change,
- aggregate reconditioning statistics in update summaries,
- metadata-first aero/export patch bounds before sampling fallback,
- live cache-quality diagnostics for sign mismatches, surface residual,
  gradient error, publication confidence, and raw quality confidence,
- metadata-driven conditioned-kernel construction for conditionable SDF trees,
- canonical Rhai script evaluation with explicit backend conditioning for
  headless export, published UI previews, sampling tools, aero export parts, and
  refinement SDFs,
- auxiliary backend geometry conditioning for component preview parts and FEA
  support/load/pressure/torque/thrust regions,
- live conditioned-cache metadata embedded in `SdfNodeMetadata`,
- headless metrics schema v5 with persisted conditioning state and confidence
  split,
- cache storage accounting for dense block samples, sparse block surface
  refinement cells, reinitialized refinement cells, surface point anchors,
  adaptive stored samples, sparse/dense adaptive values, total stored sample
  bytes, metadata bytes, and estimated total cache bytes,
- local edit application through `GeometryEdit`,
- multi-edit live updates through `GeometryEditTransaction`,
- background conditioning publication through latest-job scheduling with
  queued-edit coalescing,
- generous app-side geometry edit debounce before background conditioning
  submission,
- fail-closed full rebuild when local dirty coverage is insufficient,
- finite-difference gradient diagnostics,
- local-update versus full-recompute comparison diagnostics,
- operation-emitted edit helpers for primitive radius/dimension changes,
  translation changes, blend radius, offset distance, and shell thickness
  changes,
- generic metadata-change edit emission through `GeometryEdit::metadata_changed`
  for profile/path/section-driven tree updates,
- metadata-tree dirty-region localization for canonical graph changes, with
  conservative parent-space fallback through transform-like nodes,
- parameter fingerprints for same-envelope shape changes in primitives,
  transforms, airfoils, lofts, sweeps, paths, and ducts,
- operation-aware dirty propagation so boolean and blend nodes expand child
  edits into sibling/interface influence bands without aircraft-specific
  wing/fuselage logic,
- persistent app/backend cache advancement through
  `condition_sdf_after_backend_edit(...)`,
- canonical graph retention in `ScriptResult` so callers can replace the active
  canonical graph without discarding the previous conditioned cache,
- metadata-first backend bounds in `pipeline::auto_bounds(...)`, plus
  exact-metadata fast paths in `sdf::query::bounding_points(...)`,
- live app diagnostics for conditioned-cache state, generation, publication
  confidence, block count, spacing, and anchor counts,
- conditionable-geometry metadata audit APIs for rejecting missing bounds,
  invalid bounds, and unknown bound quality before local conditioning depends on
  the tree,
- regression tests for covered local edits, under-covered dirty regions,
  blend-radius reconditioning, disjoint edit transactions, and live canonical
  replacement,
- regression tests proving scaled/non-distance fields are restored toward
  unit-distance behavior by the fast-sweeping conditioner across same-sign
  neighboring blocks,
- regression tests proving default kernel SDF queries, projection, and normals
  use the conditioned field,
- regression tests for conditioned cache queries and operation-emitted dirty
  regions,
- real-aircraft regression tests against the curated direct sparse OML aircraft
  script, covering local blend edits, broad shape edits, shell edits,
  conditioned query API smoke tests, local-vs-full equivalence diagnostics,
  metadata coverage, and performance/memory gates.

## Near-Term Direction

The backend direction is a fully conditioned model state:

```text
Geometry kernel
  -> canonical SDF graph
  -> edit transaction
  -> dirty-region tracker
  -> conditioned block field
  -> adaptive local overlay regions
  -> connected-region local regeneration / full rebuild fallback
  -> conditioned geometry-service queries
  -> diagnostics
  -> geometry queries and exporters
```

The conditioned block field is integral kernel state, not downstream-owned
state.
It remains derived and disposable; the canonical SDF graph remains the source of
truth.

## Live Graph Metadata

Conditioning metadata is now part of the SDF graph contract. Each SDF node can
report:

- node kind,
- support bounds,
- feature IDs owned by or inherited through that node,
- child dependencies and their roles,
- support-bound quality.

This is not an after-the-fact wrapper around exported geometry. The tree itself
exposes enough metadata for the kernel to reason about affected regions while
the model changes. Primitive nodes provide analytic bounds; transforms move or
expand child bounds; booleans and patterns compose child bounds; feature tagging
preserves ownership through the tree.

Profile-driven aircraft geometry now exposes metadata through the same live
path. Cross-section implementors provide local 2-D bounds via
`Section2D::bounds_2d()`, and path implementors provide world-space centerline
bounds via `SweepPath::support_bounds()`. Sweeps, lofted wings, lofted
fuselages, NACA/buried inlets, inlet lips, EDF ducts, S-ducts, spline tubes,
variable ducts, and arbitrary-profile ducts consume those contracts directly.
That means a section edit, path edit, blend-radius change, or duct-size change
can produce a dirty region from the active SDF node without waiting for STL
export or an external metadata pass.

Twist and bend transforms also preserve conservative live support bounds. Twist
converts the child support box into an axial interval plus swept radial extent
around the twist axis. Bend converts the child support box into a preserved
perpendicular interval plus bend-plane radial envelope. These bounds are
intentionally approximate, but they keep deformed subtrees available to local
conditioning instead of forcing unknown global coverage.

Support-bound quality is explicit:

- `Exact`: analytic bounds supplied by primitives or other exact nodes.
- `Conservative`: a proven finite envelope that may be wider than the true
  active surface region.
- `Estimated`: a useful but not yet proven bound, typically from mesh-backed or
  field-driven nodes.
- `Unknown`: no trustworthy local conditioning envelope.

Local dirty-region inference requires conditionable metadata: finite support
bounds and non-unknown bound quality. `audit_conditionable_geometry(...)` walks
the metadata dependency tree and reports missing bounds, invalid bounds, and
unknown quality with dependency paths. `assert_conditionable_geometry(...)` is
the production-facing gate for code that needs to reject weak geometry trees
before relying on local cache invalidation.

Supported parameter edit emitters now use live child metadata where practical.
For example, translation, offset, and shell edits derive their dirty regions
from the current child support instead of requiring a caller-maintained box.
For geometry nodes that do not have a specialized edit helper yet,
`GeometryEdit::metadata_changed(...)` compares the previous and next
`SdfNodeMetadata`, localizes the dirty region through changed dependency
metadata where parent/child spaces are compatible, carries feature IDs forward,
and emits a feature dirty region. Transform-like nodes (`translate`, `rotate`,
`scale`, `twist`, `bend`) intentionally fall back to the parent node's
world-space support union when a child differs, because child dependency bounds
are stored in local space. If either side lacks support bounds, it returns
`None` so the caller can fall back to full conditioning instead of guessing. It
also refuses invalid bounds or unknown bound quality; bounded-but-untrusted
metadata is not allowed to drive local conditioning.

Metadata is not the field definition. The canonical `distance(point)` graph
remains the source of truth. Metadata is locality evidence. Nodes can now report
a `parameter_fingerprint` so edits that keep the same support envelope still
produce a dirty region instead of silently reusing stale cache samples. The
default section and path fingerprint hooks fall back to bounds and length, while
airfoils, explicit paths, lofted wings/fuselages, extruded airfoils, sweeps, and
ducts contribute richer shape-driving parameters.

Parent operation metadata handles interface-locality. Boolean and smooth
boolean nodes mark child changes as potentially affecting siblings. The
conditioner takes the changed child dirty bounds, expands by the operation's
interface radius, intersects nearby sibling support bounds, and unions that
interface band into the propagated dirty region. This is deliberately
location-aware and operation-aware, not aircraft-semantic: a fuselage/wing blend
and any other smooth union use the same rule.

Some nodes intentionally report conservative or estimated metadata. Field-driven
operations and mesh-backed SDFs are marked estimated until field range or mesh
validity contracts exist. Twist, bend, booleans, sweeps, lofts, and duct
compositions report conservative envelopes when they can prove finite coverage
but not exact active support.

`ConditionedGeometryKernel` is the current integration point. It owns the active
canonical SDF and `ConditionedGeometryModel`; replacing the canonical graph
requires a `GeometryEditTransaction`, so cache invalidation and conditioning are
part of the geometry edit itself. Its normal `distance()`, `conditioned_distance()`,
and `Sdf` implementation expose the published conditioned field: cache values
are used only when doing so does not degrade runtime geometry fidelity. If the
cache would flip the canonical inside/outside sign, if the query lies outside
the cache domain, or if the query lies inside the published fidelity band, the
published field falls back to the canonical graph. The fidelity band is wider
than the solver's interface band because aircraft viewers and downstream
geometry tools usually care about a near-body SDF region, not only the exact
zero crossing. This keeps the visible OML and section cuts locked to the
canonical graph while the cache continues to provide conditioned samples and
diagnostics.
Raw source-field access remains available through `canonical_distance(...)`.
Raw cache access remains available separately through
`raw_conditioned_distance(...)` for diagnostics and before/after cache-quality
plots.

The high-fidelity surface layer is not a fake query fallback. Coarse runtime
blocks may be too sparse to preserve aircraft OML details directly, so each
conditioned block can own a fine surface-refinement map at bounded spacing.
That map is derived from the canonical field during conditioning and stores the
raw high-resolution samples that preserve the local zero contour. A separate
reinitialized refinement map stores distance-restored samples for an explicit
finite band. Callers that need to audit the derived storage can query
`raw_surface_refined_distance(...)`, while callers that need the finite trusted
band can query `raw_surface_refined_reinitialized_distance(...)`. Normal
published geometry queries still arbitrate against canonical sign and fidelity
rules so they cannot silently publish a blocky or sign-flipped OML.

The runtime path keeps script evaluation canonical and moves conditioning into
the geometry backend boundary. `evaluate_script_full(...)` and
`evaluate_script_cells(...)` return the canonical graph as `canonical_sdf`; the
backend-facing `sdf` handle starts as the same graph until a caller explicitly
invokes `ScriptResult::condition_for_backend(...)` or submits it to the
background conditioning scheduler. App-side edit paths pass the new canonical
graph and the previous conditioned SDF to `condition_sdf_after_backend_edit(...)`,
allowing script, dimension, profile, spine, component, and snippet edits to
advance the existing cache with a dirty-region transaction. Headless export,
aero export, standalone sampling tools, and UI preview publication now use the
same kernel-owned conditioned representation while the procedural graph remains
the source of truth.

Backend bounds queries now consume live metadata before sampling where that is
safe. `pipeline::auto_bounds(...)` uses finite support metadata with padding
before broad SDF scans. `sdf::query::bounding_points(...)` uses metadata only
when the bound quality is `Exact`, preserving tighter directional searches for
conservative aircraft envelopes used by analysis code.

Because the cache is derived state, it reports itself through metadata rather
than replacing canonical graph metadata. `SdfNodeMetadata` has an optional
`conditioned_cache` payload with cache state, generation, block count, grid
spacing, interface band, publication confidence, raw quality confidence, anchor
counts, narrow-band block count, adaptive overlay counts, max iteration count,
and storage accounting. Storage accounting includes dense block sample count,
raw surface-refinement cell count, reinitialized surface-refinement cell count,
surface point anchor count, adaptive stored sample count, sparse/dense adaptive
value counts, total stored sample bytes, metadata bytes, and estimated total
cache bytes.
`ConditionedGeometryKernel::metadata()` preserves the canonical node kind,
support bounds, dependencies, and feature IDs, then attaches that live cache
summary. Headless metrics copy this into the metrics JSON as `conditioning` in
schema version 5.

Kernel-level geometry queries now hang off the same conditioned field:

- `inside_outside(...)`
- `closest_surface_point(...)`
- `project_to_surface(...)`
- `ray_intersection(...)`
- `gradient(...)`
- `surface_normal(...)`
- `local_feature_size(...)`
- `wall_thickness(...)`
- `curvature(...)`
- `principal_curvature(...)`
- `cross_section(...)`
- `volume(...)`
- `wetted_area(...)`
- `frontal_area(...)`

These are intentionally backend methods, not exporter-owned helpers. Downstream
clients can still use `Sdf` trait utilities, but the conditioned kernel now
exposes the core query contract directly.

## Runtime Operating Rules

Every backend-facing geometry edit should flow through the conditioned kernel:

```text
previous conditioned kernel + next canonical graph
  -> metadata-derived GeometryEditTransaction
  -> latest-job background scheduling for UI/editor paths
  -> local cache regeneration where coverage is trustworthy
  -> full rebuild when coverage is missing, rejected, or too broad
  -> publish conditioned kernel as the runtime geometry handle
```

The canonical graph is never mutated by conditioning. The runtime handle can be
a `ConditionedGeometryKernel`, but graph identity, source hashes, feature
ownership, and parameter semantics remain canonical graph concerns.

The background scheduler is intentionally latest-job oriented. Slider-heavy UI
edits should be debounced before submission; queued intermediate jobs can be
superseded so the worker spends time publishing the newest geometry state
rather than conditioning stale intermediate slider positions.

Backend call sites should not create unconditioned auxiliary geometry after the
main SDF is conditioned. `ScriptResult::condition_for_backend(...)` conditions
the main runtime SDF and auxiliary FEA regions. Component preview parts are
conditioned with the same backend helper before publication.

Regression gates for the curated aircraft model currently enforce:

- raw surface-refinement zero-contour drift remains below 1 mm on audited
  aircraft sections,
- reinitialized refinement-band residual remains below 1 mm p95 inside the
  current 5 mm trusted band,
- the trusted band reports explicit finite coverage rather than falling back to
  canonical values,
- local blend edits use local incremental conditioning,
- local blend edits affect no more than 20% of cache blocks,
- local blend edits stay under bounded conditioned-sample and adaptive-sample
  budgets,
- all curated edit cases remain under the cache memory envelope,
- near-interface sign mismatches remain zero,
- publication confidence remains publishable,
- local blend conditioning matches direct full conditioning within configured
  distance and gradient tolerances.

## Reconditioning Engine

Region regeneration now performs actual SDF reconditioning instead of only
copying canonical samples into the cache:

```text
connected block component bounds + ghost-cell halo
  -> canonical SDF samples
  -> zero/interface edge anchors
  -> projection/gradient-corrected near-interface anchors
  -> fast-sweeping solve for |grad phi| = 1
  -> restore sign from canonical samples
  -> preserve canonical samples where conditioning would move the surface away
  -> split conditioned region samples back into cache blocks
  -> store per-block and aggregate reconditioning stats
```

The implementation uses three anchor sources. Exact zero samples are fixed
anchors. If two adjacent canonical samples have opposite signs, the zero
crossing is estimated by linear interpolation along that grid edge and the two
edge endpoints become fixed anchors with physical distances to that crossing.
Near-interface samples also get a projection-style correction from the local
grid gradient: `abs(phi) / abs(grad phi)`. This lets same-sign near-interface
regions seed the Eikonal solve even when no sampled edge crosses the surface,
and it directly corrects scaled SDF fields whose gradient magnitude is not one.

All other samples are initialized to infinity and solved with repeated
eight-direction fast-sweeping passes over the connected region grid. Each update
uses the standard upwind Eikonal update from the best neighboring values along
x/y/z. After sweeping, the absolute distance solution is signed using the
canonical sample sign. A single isolated region with no interface anchors keeps
its canonical samples, but same-sign blocks inside a connected dirty component
can now condition from neighboring interface blocks instead of remaining raw.
Before conditioned samples are stored into dense blocks or adaptive overlays,
the cache applies a narrow fidelity-preserving selector: exact/near-zero
canonical samples and sign-conflicting samples keep the canonical value, while
other near-surface samples can still be reconditioned toward a unit-distance
field. At query time the
published kernel can apply a wider canonical-distance fidelity band around the
body for coarse runtime caches. Fine/default caches use the narrow cache
fidelity band; coarse caches use the maximum of that band and seven cache grid
spacings. It still only uses the smaller interface band when rejecting spurious
cache-near-surface values, so deliberately reconditioned non-distance fields
can publish improved values away from the real surface. This is an intentional
publication rule: raw cache quality remains measurable through
`raw_conditioned_distance(...)`, but normal `distance(...)` calls do not expose
coarse trilinear cache drift in the visible/useful SDF band. Adaptive overlays
are also arbitrated against their parent block near the interface so a local
overlay cannot publish a conflicting sign over a stable coarse block.

The current reconditioner is deliberately local and deterministic. It is the
MVP production direction for making dirty regions distance-like after blends,
offsets, shells, and other operations distort the raw composed field. Full
rebuilds and accepted local edits both use the connected-region sweeper.
Disjoint dirty block components are solved separately so local work scales with
modified volume instead of the bounding box between unrelated edits.

Coarse runtime caches also build a sparse surface-refinement layer inside each
conditioned block. This layer is derived during conditioning, not queried from
the canonical graph at publication time. It stores fine near-interface cells at
a bounded pitch and is queried before adaptive overlays and coarse block
trilinear interpolation. The purpose is OML fidelity: the zero contour should
remain close to the canonical surface even when the broader coarse SDF band is
still approximate.

The surface-refinement layer is intentionally split into two maps. The raw
surface map stores canonical high-resolution samples and is the publication
path for preserving the OML. The reinitialized map stores a signed-distance
restoration from interface points and is exposed as a finite-band diagnostic
and future query substrate. Current curated-aircraft testing gates the
reinitialized map over a 5 mm trusted band. A 10 mm band is tracked as a
diagnostic target, but it is not yet claimed as sub-millimeter accurate on the
full aircraft at the current spacing and memory budget.

Full cache initialization and full rebuilds generate both raw surface cells and
the reinitialized distance band. Local synchronous edits regenerate the
publication-critical raw surface layer first so section cuts and OML queries do
not become blocky after blend, sweep, twist, or dimensional edits. Wider
reinitialized-band refresh for local dirty regions is the next background
scheduler step; until that lands, code that needs a proven finite-distance band
should check the distance-cell metadata and use the explicit
`raw_surface_refined_reinitialized_distance(...)` query instead of assuming the
surface layer implies full-band conditioning.

Live cache diagnostics can now be run against the conditioned field itself.
`diagnose_cache_quality(...)` samples the current cache and reports
near-interface sign mismatches, surface residual, gradient error percentiles,
publication confidence, and raw quality confidence. Update summaries also
expose `ReconditioningStats` so callers can inspect block count, anchor count,
maximum sweep iteration count, and maximum reconditioning change immediately
after a model edit.

Section comparison diagnostics intentionally separate published geometry
fidelity from raw storage quality. `conditioning_section_compare` reports
published-vs-canonical deltas, raw-storage-vs-canonical deltas, inferred
publication guard/fallback fraction, raw surface-refinement hit counts,
reinitialized-band hit counts, finite-band coverage, and zero-contour drift for
published, raw-storage, and surface-refinement fields. A zero published delta is
therefore not treated as proof that the raw cache matched the canonical graph;
it may mean the publication guard protected that part of the section. For
current coarse runtime conditioning, the primary OML acceptance metric is raw
zero-contour drift. The finite-distance acceptance metric is the explicit
reinitialized-band residual inside the declared trusted band.

Local edit acceptance is no longer forced to compare against a full-domain
recompute. `ConditioningPolicy::local_validation_mode` controls this:

- `LocalCacheQuality` is the default runtime path. It regenerates affected
  blocks first, validates only the affected local cache against the new
  canonical graph, and reports publication confidence separately from raw
  quality telemetry so ordinary coarse-grid residuals do not force a full
  rebuild.
- `FullComparison` keeps the strict regression/debug path. It compares the
  proposed local update against a full recomputation over the domain and falls
  back to full rebuild when coverage is under-specified.

The local runtime validator rejects strong sign/topology disagreement, but
treats grid-scale near-surface ambiguity, gradient percentiles, and residuals
as confidence telemetry. This keeps normal parameter edits real-time while
preserving a stricter mode for tests and research comparisons.

Large connected components are bounded by `ConditioningPolicy::
max_region_sample_count` and `max_blocks_per_region`. If a proposed connected
region would exceed either budget, it is split deterministically along its
widest block-index axis and each half is reconnected before recursive solving.
Single blocks are allowed to exceed the budget because they are the minimum
cache unit. Runtime conditioning currently uses one block per region so local
validation and reconditioning parallelize across the affected block set instead
of serializing a large connected dirty component.

Oversized regions that still exceed the dense sample budget use a sparse
narrow-band backing store. The region scanner keeps samples near the interface,
edge-crossing anchors, and a small grid halo around them. Fast sweeping then
runs over that active sparse set. When extracting dense cache blocks from a
sparse region, samples outside the active band fall back to canonical distances.
That keeps the cache complete while limiting the expensive conditioning solve
to the area where SDF quality matters most for projection, normals, meshing, and
ray/surface queries.

Adaptive overlays now distinguish full rebuild seeding from local edit refresh.
Full rebuilds seed broad feature-sized overlays. Local edits invalidate and
refresh only the feature dirty bounds plus a small adaptive-spacing halo, not the
larger block-conditioning halo used to protect cache continuity. This keeps a
blend-radius edit from resampling a large aircraft-scale box at fine spacing.

`ReconditioningReport` and aggregate `ReconditioningStats` expose projection
anchor count, whether a region/block used narrow-band mode, and how many active
sparse samples were conditioned. This makes gradient-corrected anchoring and
budget-triggered fallback visible immediately after an edit or full rebuild.

Aero patch export now also consumes live SDF metadata for patch bounds before
falling back to sampling-based `auto_bounds(...)`. This keeps small components
such as inlet ducts from being meshed against a broad fallback cube when the
geometry graph already knows tight support bounds.

The next accuracy/performance step is extending the verified finite-distance
band beyond the current 5 mm guarantee without corrupting the raw OML surface
map. That likely requires a spatially seeded or tree-backed narrow-band update,
an OpenVDB-style sparse volume, or a higher-quality local surface primitive than
the current edge-crossing point cloud. The current point-cloud band is fast and
keeps the 5 mm residual sub-millimeter, but it is not accurate enough to claim a
10 mm sub-millimeter band on the curated aircraft.

Example backend flow:

```text
blend radius changes
  -> SmoothUnion emits GeometryEdit
  -> GeometryEditTransaction collects all dirty regions for the model edit
  -> ConditionedGeometryKernel replaces the canonical graph
  -> affected connected block components regenerate locally
  -> diagnostics compare the local result against full recomputation
  -> if coverage is rejected, the cache performs a full rebuild
```

Primitive parameter edits now follow the same pattern for supported nodes. For
example, `Sphere::edit_for_radius_change(...)` emits a dirty region based on
the old/new support radius before the kernel replaces the canonical graph.

## Non-Goals For This Backend Step

- No CFD solver work.
- No surrogate model work.
- No mesh workflow rewrite.
- No GPU acceleration.
- No replacement of the canonical SDF graph.

The immediate objective is a more reliable geometry kernel: stable gradients,
bounded local updates, clear diagnostics, and a safe fallback to full
recomputation when local coverage is not trustworthy.
