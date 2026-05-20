The code audit of `adaptive_mc.rs` has answered all five prerequisite questions. The structural gap is confirmed: edge-contact T-junctions are detected but not retained in a usable form, so no downstream emission or repair pass can address them.

The next step is to build `EdgeTransitionRecord` collection during adjacency construction and wire it into a new emission pass.

Before writing any code, read `src/mesh/adaptive_mc.rs` in full, then implement the following in order. Write findings and results to `output.md` as you go.

## Step 1: Add `EdgeTransitionRecord` collection inside `FaceAdjacency::build(...)`

When `FaceContactKind::Edge` is detected between two faces of different cell sizes with a 2:1 ratio, construct and push an `EdgeTransitionRecord` with these fields:

- `coarse_cell_idx` and `fine_cell_idx`
- `coarse_face_id` and `fine_face_id`
- `axis`, `sign`, `plane_coord`
- `coarse_cell_size`, `fine_cell_size`
- `edge_start` and `edge_end`: the two world-space endpoints of the shared edge, derived from the intersection of the two face spans on the face plane
- `overlap_axis`: which in-face axis (u or v) strictly overlaps
- `touch_axis`: which in-face axis only touches

Also add a case classifier:
- `TrueHalfEdge`: 2:1 ratio, fine span equals exactly one half of coarse span on the overlap axis, contact at the coarse face midpoint
- `CornerTouch`: contact is only at a single point
- `IrregularEdge`: everything else

Only retain `TrueHalfEdge` cases for emission. Log counts of all three cases.

## Step 2: Add `emit_edge_transitions(...)` function

Slot it immediately after `emit_transition_faces(...)` in `emit_mesh_from_cells(...)`.

For each `TrueHalfEdge` record, emit one bridging triangle that connects:
- the midpoint vertex of the coarse face edge (interpolated from coarse cell SDF corners)
- the two endpoints of the fine half-edge (already emitted as part of the fine cell's MC triangles)

Use the same `edge_vertices` dedup map for vertex reuse. Do not add a new vertex if the endpoint already exists within numerical tolerance.

Apply these acceptance gates before emitting each triangle:
- all three vertices are distinct
- the triangle is not degenerate (area > threshold)
- emitting it would not create a non-manifold edge (check edge incidence before committing)

Log: `edge_transitions_attempted`, `edge_transitions_emitted`, `edge_transitions_rejected`, rejection reasons.

## Step 3: Narrow retest on `inlet_only` only

Run standalone inlet repro only:
- script: `tmp_inlet_only_debug.rhai`
- cell: `3.0 mm`
- report topology before and after the new edge-transition pass at `adaptive-mc boundary_checkpoint: stage=post_edge_transitions`
- compare to current baseline: `boundary_edges=441`, `non_manifold_edges=469`

Do not run the full component sweep yet. Only run inlet.

If inlet boundary edges decrease and non-manifold edges do not increase, proceed to report the result and stop.

If inlet regresses, disable the emission pass and report what `edge_transitions_rejected` shows as the dominant rejection reason. Do not attempt a fix — just report.

Write all results to `output.md`.