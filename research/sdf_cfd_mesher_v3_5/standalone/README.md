> Verification qualification (2026-10-27): runtime714b83dd passed one complete audit, production and separate offline audit each for the fixed sphere and thick blend. All physical arrays match frozen references. Blend compression, 41 faces above 70 degrees nonorthogonality and nonblocking cap/interface misses are retained. Passage/general production remains unqualified. The table below remains the frozen eff955ed reference. See [blend report](../qualification/blend_verification_20261027/REPORT.md) and [sphere report](../qualification/sphere_verification_20261027/REPORT.md).

# Mesher V3.5 — frozen working candidate

Closed three-case consolidation. Scientific/runtime identity: **5e38fe034bb6f90fd23f5fee854293960e3ead35c289450ebe8d4f5a935f6666**, 1,866 protected files. This is the same build that passed the complete NODAL5 passage run. No runtime source, numerical gate or verification mode was changed for consolidation.

The path is **Rhai/SDF surface -> V3 complete-stack prisms -> existing V4 direct outer workflow -> joined acceptance/export/readback**. There is no separate transition band in this workflow.

## Qualification matrix

| Fixture | Explicit carrier mode | Current frozen-build status | Cells / points |
|---|---|---|---:|
| Passage | nodal | Complete engineering pass, NODAL5 | 546,464 / 207,705 |
| Sphere | adaptive | Complete pass; baseline-identical arrays | 53,228 / 14,798 |
| Original thick blend | adaptive | Complete pass; baseline-identical arrays | 119,441 / 38,868 |

Modes are public generic CLI choices, declared in the commands. This matrix does not claim one shared default was tested for all three. There is no fixture-name dispatch or silent fallback.

## Run and install

Read [INSTALL_FROZEN.md](INSTALL_FROZEN.md) for the configured environment and [QUALIFICATION.md](QUALIFICATION.md) for the exact three commands and limits. Use a fresh output directory. The package is a runnable snapshot for the configured Windows/WSL host; it does not bundle operating systems or prove clean-machine portability.

`--surface-carrier adaptive|nodal` selects construction only. Omission retains the V3.5 **adaptive** default. Adaptive evaluates the optional construction carrier; nodal uses the existing carrier-optional V4 FastProd route and records carrier NotRun. Both require unchanged final surface and downstream acceptance. A refusal never triggers automatic fallback.

`--surface-size` is nominal sizing. Requests for first height, row count and nominal growth must be read together with achieved thickness/compression. See [CONSOLIDATE1_REPORT.md](CONSOLIDATE1_REPORT.md).

## Known limits

Surface size is a nominal target, not a hard maximum edge or final metric-ratio guarantee. Surface geometry is sampled numerical acceptance, with no whole-source completeness, continuous face accuracy, exact root/normal, native/real-target equivalence or determinism proof. Continuous SDF fluid clearance, CFD/solver/y-plus/flow accuracy and general-shape qualification are unclaimed. The parser supports settings beyond the tested rows, but those settings are not qualified by this matrix. Historical or closed FastProd authority is not reusable as fresh live authority.

Passage has all five nominal rows, total 180.862 um, no compression or growth reversals. Sphere likewise has no compression. The original thick-blend baseline uses local compression, with total thickness 58.5731–180.862 um and 2,429 scalar / 2,559 physical-path growth reversals; nominal growth 1.3 is not achieved everywhere. The current blend regression confirms those achieved values on the frozen build.

DirectFillEngineering/1 requires bulk/interface tetrahedron minima 0.25 and cap triangle minimum 0.30, plus actual-neighbour QA and all other current gates. Cap/interface population percentage targets can remain nonblocking TARGET_NOT_MET. A complete engineering pass is not a claim that every quality distribution target is met. checkMesh -meshQuality was not run because no applicable approved dictionary exists. Clean-machine portability is not tested.

The current source inventory and compact accepted evidence are in `../qualification/consolidation_20261027/`. Copied historical manifests and commands remain unchanged as pinned provenance; the current qualification matrix governs current claims. Results, caches and multi-GB evidence remain local and are excluded from the Git freeze. No performance optimization or new verification-policy control is included.
