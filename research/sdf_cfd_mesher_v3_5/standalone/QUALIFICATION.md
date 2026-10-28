# Tested commands and qualification

One audit, one production and one separate offline audit passed for each row on the unchanged reviewed runtime. All five complete mesh arrays and eight prism-plan arrays match their frozen reference and paired run in dtype, shape and bytes. Complete retained repair/native records also match. Each production bundle stayed unchanged through offline replay (sphere374 files, blend548, passage372).

| Fixture | Carrier | Points | Cells | Audit seconds | Production seconds | Offline seconds | Production + offline seconds |
|---|---|---:|---:|---:|---:|---:|---:|
| Sphere | adaptive | 14,798 | 53,228 | 524.486 | 436.574 | 87.811 | 524.385 |
| Thick blend | adaptive | 38,868 | 119,441 | 2850.049 | 2637.364 | 229.903 | 2867.267 |
| Passage | nodal | 207,705 | 546,464 | 6998.994 | 6124.915 | 1029.549 | 7154.465 |

These are single sequential observations, not stable or causally isolated speedups. Standalone production defers replay cost. Production followed by offline can be slower overall: blend was 17.218 seconds higher than its audit run; passage was 155.471 seconds higher. Stage times are nested and must not be added to total CLI time. Memory samples cover the Windows process tree, not aggregate WSL RSS; physical allocation and total finalizer cost are unmeasured.

## Run the tested workflows

Run from the standalone directory. The commands below preserve the tested arguments, using `python` for your configured interpreter and extraction-relative output paths. They do not run on installation. Use a fresh output path each time. The exact original executable/absolute argument vectors, including the offline commands and their captured expected roots, are retained in `../qualification/verification_candidate_20261028/TESTED_COMMANDS.json` as historical provenance, not a script to replay old output locations.

### Sphere

Audit:

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/sphere.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/sphere_verify1_audit_01 --stop-after complete --verification audit --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

Production:

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/sphere.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/sphere_verify1_production_01 --stop-after complete --verification production --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

### Thick blend

Audit:

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/asymmetric_blend.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/blend_verify1_audit_01 --stop-after complete --verification audit --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

Production:

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/asymmetric_blend.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/blend_verify1_production_01 --stop-after complete --verification production --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

### Passage

Audit:

```powershell
python -B mesh.py --case fixtures/passage_historical_case.json --geometry fixtures/passage.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/passage_verify1_audit_01 --stop-after complete --verification audit --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

Production:

```powershell
python -B mesh.py --case fixtures/passage_historical_case.json --geometry fixtures/passage.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/passage_verify1_production_01 --stop-after complete --verification production --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

Omitting `--verification` selects audit. Omitting `--surface-carrier` selects adaptive; passage intentionally specifies nodal. Verification policy and carrier are independent. Production requires complete direct background execution and fails unsupported partial/non-direct combinations. Audit failure does not fall back to production. Other accepted parser settings are not qualified by these rows.

## What passed, and what remains limited

Both normal modes keep twelve mandatory producer obligations, physical/shape/domain/collision/filling/actual-neighbour checks, exact native export correspondence, literal checkMesh result and clean source/resource closure. Audit additionally runs separate semantic replay. Production records `audit: not_run`; later offline `audit_passed` is a separate result and leaves the production receipt unchanged. Offline recomputes twelve obligations, actual shape populations and retained committed edits. It reuses global collision classifications with fresh coverage, and grants no fresh live geometry authority. See README.md for evidence retention and trusted-root capture.

Sphere has 12,780 prisms and 40,448 tetrahedra, no edits/compression and no checkMesh warnings. Thick blend has 46,570 prisms and 72,871 tetrahedra, one existing-vertex 3-to-2 edit plus one cavity commit, four inserted interior points and eight constrained native calls. It retains 1,663 compressed vertices, local growth reversals and 41 faces above 70 degrees nonorthogonality (maximum77.266 degrees; one warning entry). Nominal growth1.3 is not achieved everywhere.

Passage has 298,660 prisms and 247,804 tetrahedra. All five nominal rows total180.862 micrometres, without compression or growth reversals. Conditioning is an exact no-op AlreadyAboveFloor; two shared-apex moves of2.2977053665821368 and1.686693085253412 micrometres preserve181,034 protected vertex coordinate bits and connectivity/IDs. Interface minimum weight0.05687894314413293 exceeds0.055 with zero failed faces. Literal Mesh OK has no warnings. Mandatory shape minima pass: bulk0.2716482611054878>=0.25, interface0.32041083317825586>=0.25, cap0.3304947841289329>=0.30.

Nonblocking cap/interface distribution misses remain for blend/passage. Optional checkMesh meshQuality was not run without an approved dictionary. Repair classification remains UNQUALIFIED: changed rows get source-float plus exact-binary checks; untouched rows source-float only. This is configured-host engineering acceptance, not global exact-Q or continuous SDF/fluid-clearance proof. Arbitrary geometry/carrier/settings combinations, clean-machine portability, CFD/solver/y-plus accuracy and repeatable performance are unqualified.

## Comparison history

Two external comparison refusals delayed passage closure without a normal mesh rerun. V1 treated an authority-dependent parent token as physical identity; V2 authenticates each own-run chain and compares actual physical/native operands. V2 later treated a three-byte difference in an all-scratch-files disk-space counter as a scientific metric input change. V3 reports that operational difference while independently pinning complete metric.bin (4,132,292 bytes), INITIAL, native raw output and final surface/quality/reference bytes. Counts, source, policies, native method and complete mesh/repair data remain strict. A bounded static inventory found no other varying clear category error. All29 focused external tests and independent source/result reviews passed. Original failed receipts and comparison sources remain byte-for-byte in the compact evidence history. Those scripts document qualification history; they are not normal product steps or live geometry authority.
