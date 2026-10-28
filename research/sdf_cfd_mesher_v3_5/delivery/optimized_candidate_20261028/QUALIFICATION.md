# Tested commands and fixed-case results

All three fixed cases passed production, exact baseline comparison, separate offline audit and independent review on the unchanged runtime/profile. No new audit-mode mesh was run on this optimized runtime; the audit command variant below is supported behavior, not a new numerical qualification result.

| Case / carrier | Points / prisms / tetrahedra | Archived -> fresh production (s) | Separate offline (s) | Unchanged production files |
|---|---|---:|---:|---:|
| Sphere / adaptive | 14,798 / 12,780 / 40,448 | 436.5744 -> 430.8123 | 88.0422 | 375 |
| Blend / adaptive | 38,868 / 46,570 / 72,871 | 2637.3637 -> 2595.0691 | 229.2823 | 549 |
| Passage / nodal | 207,705 / 298,660 / 247,804 | 6124.9154 -> 4637.0778 | 1036.1075 | 373 |

All five complete physical arrays, eight prism-plan arrays, native prefixes, canonical metadata and full source proof payloads match accepted baselines. Each run has its own authenticated root/profile chain. qualification/PERF11_CUMULATIVE_IDENTITY_MATRIX.json records exact identities and roots; qualification/TESTED_COMMANDS.json records original absolute argument vectors. These are historical provenance, not scripts to overwrite retained runs. Performance values are single fresh-versus-archived comparisons, not stable causal speedups. Offline cost is separate; nested stage timers and earlier component savings must not be added to whole-run savings.

## Production commands

Run from standalone, only when you intend to start a fresh mesh. These commands preserve tested numerical arguments and change only the output location and configured Python invocation. Each output must be fresh. Do not run all rows as an installation check.

### Sphere

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/sphere.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/sphere_optimized_production_01 --stop-after complete --verification production --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

### Blend

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/asymmetric_blend.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/blend_optimized_production_01 --stop-after complete --verification production --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

### Passage

```powershell
python -B mesh.py --case fixtures/passage_historical_case.json --geometry fixtures/passage.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier nodal --background fixtures/background_direct_v4.json --output results/passage_optimized_production_01 --stop-after complete --verification production --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

For audit mode, use the same case arguments, change --verification production to --verification audit and use a different fresh output name. Omitting --verification also selects audit. This recomputes source segments and adds separate replay. It does not fall back after failure. Production followed by later offline replay pays the deferred audit cost; it is not a claim of lower combined cost than audit mode.

The commands use 0.001 metres/model unit, 20-micrometre first height, five rows, growth 1.3, nominal 200-micrometre size-led surface and the original direct background. Passage uses its historical case and explicit nodal carrier; sphere and blend use the template and adaptive carrier. Do not change parameters or lower gates to obtain acceptance after a scientific failure. README.md gives trusted-root capture and a fresh separate offline command. Each fresh run needs its own trusted root; do not reuse the historical roots.

## What the results establish

Twelve producer checks and twelve separate offline semantic obligations passed for each row. All production bundles stayed unchanged through offline verification. Final preparation authenticated 6,400 / 23,295 / 149,330 source segments respectively, with zero recomputation at that stage. Earlier source/collision predicates still ran. The archived full source file is a locally pinned historical comparison input; its old production manifest did not independently bind the entire file. Offline authenticates original source evidence and provides no fresh source authority.

Sphere has no compression or MeshOK warning. Blend retains 1,663 compressed vertices, local growth reversals and the 41-face severe-nonorthogonality warning (maximum 77.266 degrees). Nominal growth 1.3 is not achieved everywhere. It retains two committed edits and four inserted points. Passage has zero compression, nominal 180.862-micrometre stack thickness, two small apex moves, 181,034 protected coordinates unchanged and no MeshOK warning. Mandatory minima pass; retained nonblocking interface/cap percentage target misses remain.

Shared-apex classification remains UNQUALIFIED. Untouched rows have source-float evidence only. No global exact-Q population, continuous whole-source/SDF-fluid proof, hard 200-micrometre surface guarantee, CFD/y-plus accuracy, arbitrary-geometry coverage, clean-machine portability, release qualification or repeatable speedup is established. This is numerical engineering evidence for the fixed cases/settings and configured host/exact isolated profile.

Detailed accepted evidence and independent result reviews are in qualification. Multi-GB raw result bundles are intentionally excluded; those must be retained separately for replay. The runtime ZIP cannot substitute for them. This package is a separate local delivery and does not replace the original frozen or published verification candidates.
