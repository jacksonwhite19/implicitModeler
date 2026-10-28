# Mesher V3.5 current qualification

V3 complete-stack prisms feed the existing V4 direct transition/background, then joined acceptance/export. There is no separate transition band. Runtime714b83dd /8e428c2a5e7bee83dc71c70660c40cbd1452f08ad99674ed5e4755456768c518 has passed one audit -> production -> offline sequence each for the fixed sphere and thick blend. All five physical arrays and eight prism-plan arrays match their accepted frozen cases exactly. The blend also reproduces both committed conditioning edits and four inserted points.

Audit remains default. Production retains twelve producer checks and actual native fidelity, omitting only separate semantic replay with audit not_run. Separate offline audits passed twelve semantic checks each without changing the production bundles or creating fresh source authority. Source/resource closure passed.

Blend: 119,441 cells / 38,868 points; 1,663 compressed prism columns; 41 faces above 70 degrees nonorthogonality (max 77.266 degrees); cap/interface distribution targets remain nonblocking misses while mandatory minima pass. Settings: explicit adaptive, 20 um first height, five rows, growth 1.3, 200 um surface size, original direct/background policy. Observed blend CLI seconds: audit 2850.049, production 2637.364, offline 229.903. One pair does not establish repeatable speedup.

See [blend report](qualification/blend_verification_20261027/REPORT.md), [blend status](qualification/blend_verification_20261027/STATUS.json), [sphere report](qualification/sphere_verification_20261027/REPORT.md) and [verification contract](PHASE1B_CONTRACT.md). Dispatch 3/5: one attempt each, no runtime implementation changes/retries/full-shape expansion. Stop after callback.

Only these fixed fixtures/settings/configured-host routes are qualified. Passage production/offline, arbitrary geometry/settings/carriers, clean-machine portability, continuous geometry and CFD/y-plus accuracy remain unqualified. Frozen eff955ed retains its prior three-case audit reference and is unchanged. No push or frozen package replacement.
