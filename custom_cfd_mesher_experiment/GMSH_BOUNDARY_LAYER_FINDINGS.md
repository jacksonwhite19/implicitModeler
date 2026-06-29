# Gmsh Boundary-Layer Findings

Verdict: conditional for tetrahedral meshing, fail for near-term prism boundary-layer scoring.

Gmsh remains the best current off-the-shelf full-aircraft volume mesher in this
experiment, but the topological prism boundary-layer path is not robust enough
on the current discrete aircraft STL. The tetrahedral Gmsh baseline remains the
recommended near-term optimizer mesh path.

## Reference

The Gmsh manual describes topological boundary layers as built-in-kernel
extrusions from surfaces. It also notes that this method is a simple extrusion
without fan or reentrant-corner treatment, and that the `BoundaryLayer` field is
2D-only. That matches the failures seen around the aircraft wing/body and
leading/trailing-edge region.

Source: https://gmsh.info/doc/texinfo/

## Best Passing Baseline

`runs/improve_gmsh_10k_hxt_netgen_far011`

- Mesh type: tetrahedral external-flow volume.
- Input STL: `runs/wing_span_plus20_v3_transition/aircraft_surface.stl`
- Aircraft wall faces: 10,000.
- Cells: 122,351 tetrahedra.
- Points: 23,996.
- Aircraft patch faces: 10,000.
- Farfield patch faces: 3,270.
- OpenFOAM `checkMesh`: `Mesh OK.`
- SU2 validation: valid.
- Surface deviation: p95 0.0977 mm, p99 0.1432 mm.
- `potentialFoam`: completed, continuity error `1.0511682e-05`.
- Report: `runs/improve_gmsh_10k_hxt_netgen_far011/GMSH_IMPROVEMENT_REPORT.md`

This is still plumbing/early-CFD evidence, not final scoring CFD.

## Boundary-Layer Attempts

| Run | Result | Notes |
|---|---|---|
| `bl_gmsh_aircraft_2k_angle90_recombine` | Failed OpenFOAM strict checks | 2,000 prisms + 36,721 tets; SU2 valid; OpenFOAM had 2 negative-volume cells, 12 wrong-oriented faces, 18 `defaultFaces`. |
| `bl_gmsh_aircraft_2k_angle90_h0010_recombine` | Worse | 2,000 prisms + 36,639 tets; 3 negative-volume cells, 15 wrong-oriented faces, 30 `defaultFaces`. |
| `bl_gmsh_aircraft_5k_angle90_h0005_recombine` | Failed OpenFOAM strict checks | 5,000 prisms + 77,519 tets; SU2 valid; `gmshToFoam -keepOrientation` removed `defaultFaces` but left 3 negative-volume cells and 13 wrong-oriented faces. |
| `bl_gmsh_aircraft_5k_angle50_h0005_recombine` | Same localized failure | 5,000 prisms + 77,796 tets; SU2 valid; `gmshToFoam -keepOrientation` still left 3 negative-volume cells and 13 wrong-oriented faces. |
| `bl_gmsh_aircraft_5k_angle50_h0005_suppress_badbox010` | Rejected | Local scalar-view layer suppression created pyramids/polyhedra, 1,682 `defaultFaces`, invalid vertex faces, zero-area faces, and 329 negative-volume cells. |

## Useful Automation Findings

- For Gmsh prism/hex-containing meshes, OpenFOAM conversion should use
  `gmshToFoam -keepOrientation`.
- Without `-keepOrientation`, Gmsh prism meshes can create `defaultFaces` and
  extra skew/wrong-oriented artifacts during conversion.
- `-keepOrientation` is not enough to make the current prism BL path pass. It
  improves conversion but does not fix local degenerate/inverted cells produced
  by the topological extrusion.
- The bad cells are localized around approximately:
  - `x = 0.4188..0.5976 m`
  - `y = -0.0097..0.0309 m`
  - `z = 0.2020..0.3252 m`
- This region overlaps the wing-side/wing-body area on the classified aircraft
  surface, so excluding a whole classified surface would remove too much useful
  boundary-layer coverage.

## Commands

Best prism attempt:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\gmsh_boundary_layer_mesher.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\bl_gmsh_aircraft_5k_angle50_h0005_recombine `
  --target-faces 5000 `
  --surface-size 0.009 `
  --farfield-size 0.11 `
  --padding 0.7,0.5,0.5 `
  --layer-heights 0.0005 `
  --layer-elements 1 `
  --angle-deg 50 `
  --optimize default,Relocate3D
```

OpenFOAM handoff used for prism meshes:

```bash
source /opt/openfoam13/etc/bashrc
gmshToFoam -keepOrientation ../mesh.msh
checkMesh -writeSets -writeSurfaces
```

Rejected scalar-view suppression attempt:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\gmsh_boundary_layer_mesher.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\bl_gmsh_aircraft_5k_angle50_h0005_suppress_badbox010 `
  --target-faces 5000 `
  --surface-size 0.009 `
  --farfield-size 0.11 `
  --padding 0.7,0.5,0.5 `
  --layer-heights 0.0005 `
  --layer-elements 1 `
  --angle-deg 50 `
  --thickness-suppression-boxes 0.40,0.66,-0.04,0.04,0.16,0.34,0.1 `
  --optimize default,Relocate3D
```

## Recommendation

Use `runs/improve_gmsh_10k_hxt_netgen_far011` as the near-term optimizer
integration mesh baseline. Keep prism boundary-layer generation experimental
until one of these changes is available:

- cleaner CAD/BRep or feature-aware surface patches instead of STL-only
  topological extrusion,
- a dedicated BL-capable mesher that handles wing trailing edges, blends, and
  reentrant regions with fans/corner treatment,
- or a custom near-wall shell plus constrained tet transition that passes
  strict OpenFOAM checks.

Do not use the current Gmsh prism BL output for scoring CFD.
