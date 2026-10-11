# SDF CFD Mesher V4 code backup

This branch preserves the qualified V4 code and its runnable native dependencies as exact bytes. It starts with a new root commit so old meshes and examples are not inherited through Git history. The original application and research branch remain separate.

## Restore and run

On Windows, enable Git long-path support for this checkout (`git config core.longpaths true`, or `git -c core.longpaths=true clone ...`). Some authenticated native filenames exceed the usual Windows path limit. Use the backup branch `backup/sdf-cfd-mesher-v4-code-20261011` in `jacksonwhite19/implicitModeler`.

The qualified deployment is Windows with Python 3.12 and the existing WSL Linux environment. Keep the repository layout. Native build authentication binds the exact original path: `C:\Users\Jackson\Desktop\02_Projects\09b_Implicit_CAD_claude`. Restore the backed-up V4 subtree there for the existing qualified runtime. A fresh clone at another path passes release integrity and CLI help but its complete background preflight fails closed with `UnreviewedSizingLeafBuild`. Relocation requires a separately reviewed packaging/authentication change; no checks were bypassed for this backup. Do not overwrite an active checkout without first preserving its changes. The public entry point is `research/sdf_cfd_mesher_v4/mesh.py`. No mesh or example input is bundled. Supply your own case JSON, SDF/Rhai geometry and explicit sizing, prism schedule, resource limits and background policy.

```powershell
py -3.12 -m pip install -r requirements-windows.txt
py -3.12 -B research/sdf_cfd_mesher_v4/mesh.py --help
py -3.12 -B research/sdf_cfd_mesher_v4/mesh.py --case C:\path\to\case.json --from-layer 0 --through-layer 2 --mode fastprod --surface-assurance numerical --check
```

`--check` validates inputs and dependencies without meshing. Use the layer count and assurance mode admitted by your case. For generation, omit `--check` and supply `--output` outside this checkout. An output mesh is not restart authority or a CFD solver validation.

Windows Python package versions are recorded in `requirements-windows.txt`. WSL provides `python3`, NumPy, SciPy and SymPy, the Linux dynamic loader and system libraries needed by the bundled Linux binaries. The native build records also authenticate the existing compiler, dependency headers, shared libraries and NumPy arithmetic binary. This Git backup is not an image of that WSL environment. `ENVIRONMENT.json` records the observed environment and backend probe. Existing Gmsh 4.15.2, MMGS and the canonical kernel binaries are bundled under `runtime`; no alternative mesher is installed. Netgen optimization is invoked through the bundled Gmsh library. This is a backup of the tested Windows/WSL deployment, not a claim of portability to every operating system.

## What is included

`BACKUP_MANIFEST.json` enumerates every copied payload file and its SHA-256. The 1,205-file conservative release closure includes shared and historical code required by the current integrity inventory. Native binaries, libraries, dependency licenses, build metadata, the runtime manifests and two small release-authentication records are included. The two JSON files under `results/source_qualification_20261011` are required by `verify_release()`; they are not mesh examples or experiment output data.

V4-owned Rust crates and the native callback header are also included. The canonical kernel crate's source build refers to the parent Implicit CAD application. That application is not copied into this code-only mesher backup. The already-qualified kernel host binary is included and runs without rebuilding the application. A source rebuild requires the parent source and separate qualification; this snapshot does not claim that a rebuild reproduces the pinned binaries.

Examples, case inputs, generated meshes, test data, experiment reports, caches, target directories and unrelated application code are excluded. Do not add/remove files under `tools`, `src` or `runtime` casually: the runtime checks the complete release inventory and exact file bytes. Source line endings are preserved by `.gitattributes`.

## Supported scope

One shared generic SDF/Rhai tool with explicit user controls remains the entry point. The last qualified smooth wing-body case passed fresh surface, two configured prism layers and background mixed-mesh acceptance. Numerical surface mode does not prove whole-source completeness or the formal continuous surface contract. Explicit `backend_trusted` background mode omits continuous new-core collision, independently enumerated collision coverage, exact total volume balance and full global degree proofs. Prism obligations and the current quality gates remain unchanged. This backup adds no supported geometry, assurance contract, scientific setting or performance claim.
