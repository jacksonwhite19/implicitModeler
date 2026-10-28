> Verification status (2026-10-27): sphere/adaptive and thick-blend/adaptive sequences passed on unchanged runtime714b83dd. Passage/nodal actual full audit passed, then external parent-token comparison stopped; production/offline NOT RUN. Retained physical state matches reference; first failed gate is preserved. See [stop report](../qualification/passage_verification_stop_20261027/REPORT.md). Frozen eff955ed table below remains historical and unchanged.

# Frozen-build qualification and exact commands

Closed three-case consolidation. Build **5e38fe034bb6f90fd23f5fee854293960e3ead35c289450ebe8d4f5a935f6666**. The mode and achieved-layer distinctions are part of the supported scope.

| Fixture | Explicit carrier mode | Current frozen-build status | Cells / points |
|---|---|---|---:|
| Passage | nodal | Complete engineering pass, NODAL5 | 546,464 / 207,705 |
| Sphere | adaptive | Complete pass; baseline-identical arrays | 53,228 / 14,798 |
| Original thick blend | adaptive | Complete pass; baseline-identical arrays | 119,441 / 38,868 |

Run from the standalone directory. These are the exact recorded commands, not a promise of acceptance on other geometry or environments. Fresh output is required; no retry or fallback occurs after scientific refusal.

### Sphere: fresh same-build regression

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/sphere.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/consolidate1_sphere_01 --stop-after complete --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

### Original thick blend: fresh same-build regression

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/asymmetric_blend.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/consolidate1_blend_01 --stop-after complete --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

### Passage: accepted NODAL5 run

```powershell
python -B mesh.py --case fixtures/passage_historical_case.json --geometry fixtures/passage.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/passage_nodal5_complete_01 --stop-after complete --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

## Limits

Surface size is a nominal target, not a hard maximum edge or final metric-ratio guarantee. Surface geometry is sampled numerical acceptance, with no whole-source completeness, continuous face accuracy, exact root/normal, native/real-target equivalence or determinism proof. Continuous SDF fluid clearance, CFD/solver/y-plus/flow accuracy and general-shape qualification are unclaimed. The parser supports settings beyond the tested rows, but those settings are not qualified by this matrix. Historical or closed FastProd authority is not reusable as fresh live authority.

Passage has all five nominal rows, total 180.862 um, no compression or growth reversals. Sphere likewise has no compression. The original thick-blend baseline uses local compression, with total thickness 58.5731–180.862 um and 2,429 scalar / 2,559 physical-path growth reversals; nominal growth 1.3 is not achieved everywhere. The current blend regression confirms those achieved values on the frozen build.

DirectFillEngineering/1 requires bulk/interface tetrahedron minima 0.25 and cap triangle minimum 0.30, plus actual-neighbour QA and all other current gates. Cap/interface population percentage targets can remain nonblocking TARGET_NOT_MET. A complete engineering pass is not a claim that every quality distribution target is met. checkMesh -meshQuality was not run because no applicable approved dictionary exists. Clean-machine portability is not tested.
