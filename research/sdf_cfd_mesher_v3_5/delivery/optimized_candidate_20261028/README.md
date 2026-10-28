# Mesher V3.5 optimized verification candidate

This separate candidate connects V3 complete-stack prisms to the existing V4 direct transition/background, joined checks and OpenFOAM export. No separate transition mesher is added. It includes the reviewed source and owned binaries. Full result bundles and caches are excluded.

Use this package on the existing configured Windows/WSL host. It is not a self-contained or clean-machine portable ZIP. Start with [installation and dependencies](INSTALL.md), then [tested commands and results](QUALIFICATION.md). All run commands below start in `standalone`.

The runtime remains commit `d14d7592d35e36914e2c5d3fcaa6f664843d0a96`, identity `3bdd14455b74a67e6fa1405efc945956f69bcb03925283ee1d6481b9815b82a8`, across 1,885 files. Release and package manifests are outside standalone so they do not change verifier identity. Numerical settings and runtime bytes must remain unchanged to retain this qualification.

Audit is the default (`--verification audit`). It runs producer checks and separate semantic replay, with source-segment recomputation. Explicit `--verification production` supports only complete direct execution. It keeps all twelve producer obligations, construction/quality/domain/collision/actual-neighbour checks, exact native export fidelity, literal checkMesh acceptance, artifact binding and source/resource closure. It defers the separate replay and records `audit: not_run`. An offline pass is a separate result and leaves that production receipt unchanged. Audit failure does not fall back to production.

Production final preparation may authenticate the complete immutable source-proof population from the same live run when policy, method, arithmetic, coordinates and final operands match. Changed or unavailable live evidence takes the original evaluation path. Earlier source/collision checks and independent face coverage still run. SOURCE_EVALUATION.json identifies the actual method. This is not historical cache reuse or an independent source-predicate rerun. The accepted parent and completion records bind the full retained evidence. Offline authenticates that original evidence without granting fresh live source authority.

Carrier and verification are independent. Adaptive is the carrier default and was tested for sphere and blend. Passage must specify nodal. Other settings/carrier combinations have no qualification from these rows.

## Keep a complete production bundle

Keep the whole successful production output, including VERIFICATION.json, source/closure records, manifests, raw/native operands and nested background evidence. The exported mesh alone is insufficient. Preserve the matching verifier runtime. Capture the trusted root immediately after exit zero and closed production acceptance, outside the bundle. Do not create a replacement trusted root after an unexplained modification.

Example after the fresh sphere production command in QUALIFICATION.md succeeds:

```powershell
$bundle = (Resolve-Path 'results/sphere_optimized_production_01').Path
$verification = Get-Content -Raw (Join-Path $bundle 'VERIFICATION.json') | ConvertFrom-Json
if ($verification.acceptance_status -ne 'production_accepted') { throw 'Production did not close' }
$root = (Get-FileHash -LiteralPath (Join-Path $bundle 'VERIFICATION.json') -Algorithm SHA256).Hash.ToLowerInvariant()
$verifier = (python -B offline_audit.py --print-verifier-identity).Trim()
if ($LASTEXITCODE -ne 0 -or $verifier -ne '3bdd14455b74a67e6fa1405efc945956f69bcb03925283ee1d6481b9815b82a8') { throw 'Incompatible verifier' }
if (Test-Path '../sphere-production-trust.json') { throw 'Use a fresh trust record' }
@{ bundle=$bundle; expected_root=$root; verifier_identity=$verifier } | ConvertTo-Json | Set-Content -Encoding utf8 '../sphere-production-trust.json'
```

Later, from standalone, use a fresh output outside both the production bundle and standalone:

```powershell
$trust = Get-Content -Raw '../sphere-production-trust.json' | ConvertFrom-Json
python -B offline_audit.py --bundle "$($trust.bundle)" --expected-root "$($trust.expected_root)" --verifier-identity "$($trust.verifier_identity)" --output '../offline-audits/sphere_optimized_offline_01' --timeout-seconds 43200
```

Require exit zero, AUDIT_RESULT.json status audit_passed, all twelve semantic replays and closed verifier custody. Preserve both the unchanged production bundle and the separate audit result. A wrong root, changed evidence, incompatible verifier or overlapping/existing output is refused. Offline does not remesh, repeat search, convert or run checkMesh again. It reuses global collision classifications with fresh coverage. Actual committed edits are replayed as implemented; passage's two apex moves replay in process, without native changed-versus-all recertification. No new per-move replay files are created for passage. Blend has its own native repair replay. No global exact-quality or continuous fluid proof is implied.
