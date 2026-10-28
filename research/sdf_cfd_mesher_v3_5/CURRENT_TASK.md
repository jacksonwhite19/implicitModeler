# Mesher V3.5 current qualification

The fixed sphere audit -> production -> offline sequence passed once each on runtime714b83dd /8e428c2a5e7bee83dc71c70660c40cbd1452f08ad99674ed5e4755456768c518. All five physical arrays and eight prism-plan arrays match the accepted frozen sphere exactly; counts53,228cells/14,798points. Explicit adaptive,20um firstheight,five rows,growth1.3,200um nominal surface size; original direct/background and operational settings.

V3 prisms feed the existing V4 transition/background. Audit is default. Production retains all twelve producer checks and native fidelity, omitting only separate semantic replay with audit not_run. Genuine offline audit subsequently passed twelve semantic checks without modifying any of374production files or creating fresh source authority. Source/resource closure passed. CheckMesh warnings:none. No compression or growth reversals.

Observed CLI times:audit524.486s,production436.574s,offline87.811s; production alone87.911s/16.76%lower in this pair. Production+offline524.385s versus audit524.486s is essentially equal. This is not a statistical speedup claim. Full finalizer cost is not separately instrumented; Windows sampled RSS excludes aggregate WSL Linux RSS.

See [closed qualification report](qualification/sphere_verification_20261027/REPORT.md), [compact status](qualification/sphere_verification_20261027/STATUS.json) and [verification contract](PHASE1B_CONTRACT.md). Authorization:advisor dispatch2/5 on human-authorized verification work; one attempt each, no runtime implementation changes/retries/full-shape expansion. The current milestone stops after the closed report callback.

Only this sphere/settings/configured-host production and offline route is qualified. Passage/blend production, arbitrary geometry/settings, clean-machine portability, continuous SDF/CFD/y-plus accuracy and repeatable performance remain unqualified. Frozen eff955ed retains its prior three-case audit qualification and is unchanged. No push, package replacement or larger fixture is authorized by this record.
