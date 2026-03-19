# Viewport Preview Plan

## Goal

Provide reliable, low-latency viewing for sub-1 m aircraft models, including thin trailing edges and wing-body intersections, without forcing export-grade meshing on every script run.

## Constraints

- The current uniform cubic SDF preview wastes too many samples in empty space.
- Aircraft models are highly anisotropic: span is much larger than thickness and height.
- Existing analysis and picking paths depend on the current cubic `SdfGrid`.
- Export accuracy is already acceptable; the main gap is live preview fidelity.

## Recommended Architecture

1. Keep a fast analysis/picking grid.
   - Continue using the existing cubic `SdfGrid` for picking, thickness, and print analysis.
   - Keep this grid modest in resolution so script execution stays responsive.

2. Add a separate viewport-only preview grid.
   - Use an anisotropic 3D texture sized per axis from world-space bounds.
   - Target about 1.0 mm cells for sub-1 m aircraft.
   - Cap total voxel count so latency remains acceptable.

3. Render the viewport from the anisotropic preview grid.
   - Use the current raymarcher, but feed it non-cubic texture dimensions.
   - Use per-axis voxel size in the shader for normals and stepping.

4. Future refinement path.
   - Add interaction-aware LOD: coarse while orbiting, refine when idle.
   - Add chunked/narrow-band caching around the isosurface instead of full-volume refinement.
   - Add view-dependent refinement near the camera and silhouette edges.

## First Implementation Slice

- Add `PreviewGrid` with per-axis resolution.
- Add `compute_preview_grid()` in the pipeline with aircraft-oriented sizing rules.
- Route viewport rendering to `PreviewGrid`.
- Reduce the analysis/picking grid back to a moderate cubic resolution.

## Why This First

This addresses the main product problem directly:

- better trailing-edge preservation than a cubic grid at the same cost
- much lower latency than brute-force cubic upsampling
- minimal disruption to existing analysis code

