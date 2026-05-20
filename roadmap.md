# Wing Shell Export Artifact Roadmap

## Status

Completed for the current export path.

The wing fixture now exports to:

```text
test_scripts/wingTest.stl
```

Validation result from `python scripts/check_mesh.py test_scripts/wingTest.stl`:

```text
is_watertight: True
boundary_edges: 0
non_manifold_edges: 0
connected_components: 1
n_triangles: 898694
n_vertices: 448547
```

The previous internal closed artifact islands were caused by the shell SDF producing multiple disconnected closed zero-level surfaces. The dominant component is the usable physical wing shell; the smaller components were internal extraction islands. The fallback exporter now logs component diagnostics and prunes to the dominant shell component for this export path.

## Implemented Actions

1. Added component diagnostics for fallback export meshes.
2. Added generalized triangle-component pruning.
3. Tightened wing fallback pruning from `keep largest 2 plus >= 1%` to `keep dominant component plus >= 5%`, which removes the internal islands seen in the wing export.
4. Kept the user-facing script API as `shell(outer, wall_mm)`.
5. Updated `test_scripts/wingTest.rhai` to use `shell(outer, wall_mm)` instead of a manually defined inner wing.
6. Re-exported the 700 mm NACA 2212 wing to `test_scripts/wingTest.stl`.

## Remaining Engineering Caveat

Adaptive dual contouring is still not the production export result. The adaptive DC diagnostics still show unresolved boundary/non-manifold topology, so the production export path falls back to the uniform reference mesher and cleanup path.

Current adaptive DC failure signature:

```text
adaptive_dc_boundary_edges: ~30k
adaptive_dc_non_manifold_edges: ~1k
fallback: legacy_uniform_marching_cubes
```

That is acceptable for this milestone because the user-facing STL export is watertight and artifact-pruned, but the adaptive DC emitter still needs its own closure work before it should replace the fallback.

## Next Roadmap

1. Keep the current fallback as the export safety path.
2. Add a mesh validation gate after build and before save so UI/headless export can report watertightness, components, and non-manifold counts.
3. Resume adaptive DC work separately by fixing the native adaptive edge emitter and transition ownership logic.
4. Once adaptive DC reaches zero boundary edges on sphere, prism, and wing fixtures, compare output quality and speed against the current fallback before switching defaults.
