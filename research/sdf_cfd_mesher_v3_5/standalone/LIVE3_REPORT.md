# LIVE3: fresh normal CLI connection passes

The normal V3.5 CLI now connects freshly generated V3 prisms to the established V4 direct outer workflow, including bounded background conditioning before the original bulk-quality/shared-apex stage. There is no separate transition band. One fresh original thick-blend run completed source/surface/prism admission, the genuine live background handoff and terminal lease, all twelve joined checks, exact OpenFOAM export, literal Mesh OK, and a separate-process semantic readback. No historical mesh was supplied as input. This is a complete engineering result for this fixture, not CFD or general-geometry qualification.

## What changed

Shared CORE1 existing-vertex repair runs only for current deficient bulk cells; CORE2 constrained cavities follow only when needed within the same fixed family. Passing bulk takes a no-search/no-native no-op. The normal controller propagates inserted points, changed cells, persistent IDs and owners atomically. Cold readback reconstructs the current fresh native baseline and validates committed edits without rerunning proposal search or Gmsh. The original live lease, quality floors and twelve joined checks remain required. Native cavity calls share the live lease deadline and cumulative native budget. This adds a separately identified V3.5 conditioning method to the established V4 construction.

## Fresh settings and achieved prisms

Stock asymmetric_blend.rhai through the copied Rhai/SDF pipeline; scale .001 metres/model-unit; surface size .0002 m, size-led; first true normal height20um; five rows; nominal growth1.3. Original case SHA256: 9db8b51421a1c4494b36c6b04075d56fe9466c1d5dcae8cde591234331b0350b, byte-identical to FULL5. Fresh live prism identity: 5448d2afef5681ad163ec0fce26172fb95dee08741eb83224575416d6fd2d208.

Surface: 4,659 vertices / 9,314 triangles. Admitted shell: 46,570 prisms. Actual first normal height is 19.99999999999979–20.000000000000213um, max error2.1006417091906648e-19m. Achieved scalar total58.5731–180.862um versus nominal180.862um. Per-row scalar increment minima are20,20,13.52,2.197,2.8561um; maxima20,26,33.8,43.94,57.122um. There are2,429 scalar growth reversals and2,559 path growth reversals. Nominal1.3 growth is not uniformly achieved. All fresh schedule/native-plan bytes equal the original comparison; execution, live authority and custody establish freshness.

## Background conditioning and mesh

The fresh native background again contained72,849 tetrahedra and two deficient cells, minimum0.248752321. CORE1 evaluated57 proposals, committed one3-to-2 swap and left one deficient cell. CORE2 used8 local native calls/15 snapshots and stopped at the first passing candidate: depth3, seed-point rule, optimized, replacing31 tetrahedra with54 and inserting4 strictly interior points. All38,864 original point coordinates, every prism and constrained cap/farfield facet stayed fixed. No shared-apex motion was needed.

Final mesh: **38,868 points;46,570 prisms;72,871 tetrahedra;119,441 cells**. All float/exact bulk labels pass the unchanged0.25 floor; minimum **0.26732849523888347**. Replacement minimum0.2784489302575291. Interface minimum0.2969626531058274 passes0.25; cap minimum0.3034890879831677 passes0.30. The cap and interface percentage statuses remain TARGET_NOT_MET under the existing NONBLOCKING_ENGINEERING_DIAGNOSTIC role; bulk percentage targets are met. Exact cavity volume/boundary, new-point degree, contact coverage and actual-neighbour QA passed. Independent review recomputed the complete bulk exact floor, joined owner incidence, original-coordinate preservation and cavity volume/new-point degrees. It did not rerun native predicates.

All12 original joined checks passed across268,660 unique faces. Exact conversion preserves point/cell bijections, face incidence/winding and wall/farfield labels. checkMesh exited0 with literal Mesh OK. **Warning:41 faces exceed70 degrees nonorthogonality; maximum77.26599753823804 degrees.** No applicable approved mesh-quality dictionary was run. Continuous SDF fluid proof remains NOT_PERFORMED and CFD_qualified=false.

## Timing, tests and custody

Conditioning total: 750.266s; CORE1 23.062s; CORE2 708.438s. The background controller spent754.665s in conditioning,217.168s in joined proofs,55.033s in export/live replay and227.065s in separate readback. Original fresh farfield and bulk generation took85.903s and110.366s; shared-apex stage7.609s. Root-stage timings are in LIVE3_METRICS.json. Fresh worker total 2871.605s; supervisor 2873.921s. Peak sampled Windows process-tree RSS 2,234,241,024bytes. This is not aggregate Linux RSS. No resource/monitor stop. Source custody closed clean across 1,857files, identity 0b4a5bfedb9b7fd502441c2a8979f709cd3e57c665eab8fe8a9765b40f079a81. Native command and Linux-scope records are preserved in the fresh evidence package.

Existing regression suite passed121 tests in102.750s. After a final provenance-validator hardening and addition of the nonzero-apex custody control, all11 focused controls passed in1.120s. The entire suite was not rerun after that last narrow change. Two independent source reviews cleared this single fresh run. Final independent component and complete-path reviews accompany the evidence. No tuning, retries, threshold changes, cap replanning, additional shapes or production changes were made.

Whole-run bounds21600s/16GiB/4GB output; unchanged background10800s and cumulative native7200s. CORE1:512proposals/8commits/40cells/1800s. CORE2:depth1..4,256cells/512facets,12nativecalls/24snapshots perseed,at most2seeds/2commits/128newpoints/4096newtets,360s pernative/7200s repair. These are ceilings, not runtime targets.

## Reproduction

Run from the extracted Mesher_V3_5_Standalone directory with the configured Windows Python3.12/NumPy/SciPy/SymPy/CasADi/psutil, WSL Ubuntu22.04 Python3.10 and its numerical libraries/Gmsh4.15.2, and Foundation OpenFOAM13. See FULL5_INSTALL.md for detailed runtime requirements; fresh-machine portability has not been validated. Original production/research V3 and V4 trees are not runtime source dependencies. Use a new output folder; the recorded command is:

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/asymmetric_blend.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --background fixtures/background_direct_v4.json --output results/asymmetric_blend_live3_01 --stop-after complete --timeout-seconds 21600 --memory-limit-gib 16
```

Prior FULL5, CORE1 and CORE2 artifacts remain preserved. Updated source and fresh evidence archives include per-member hashes. Stop boundary: advisor callback and review; no further campaign is started.
