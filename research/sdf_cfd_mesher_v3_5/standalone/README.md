# Mesher V3.5 verification candidate

V3 complete-stack prisms connect to the existing V4 direct transition/background, followed by joined checks and OpenFOAM export. There is no separate transition band. This is a separate local verification candidate. The original frozen release is unchanged.

Start with [installation](INSTALL_VERIFICATION.md), then the [tested commands and results](QUALIFICATION.md). Run commands from this directory. The archive includes built native components and their runtime/source closure. It excludes result bundles and caches.

## Choose verification and carrier independently

`--verification audit` is the default. It runs all twelve mandatory producer obligations, native export/fidelity checks, source/resource closure and the separate twelve-obligation semantic replay.

`--verification production` is explicit and supports only `--stop-after complete` with the direct background policy in `fixtures/background_direct_v4.json`. It keeps the mathematical producer, construction, quality, export and closure obligations. The isolated performance candidate adds `V35LiveFinalSourcePopulation/1`: final preparation can authenticate complete source-segment evidence from the same live run and identical final operands, instead of recomputing those predicates. Changed or unavailable evidence takes the original evaluation path. `SOURCE_EVALUATION.json` states which method ran. Audit mode recomputes source segments, and independent face coverage always runs. Production also omits separate full semantic replay, records `production_accepted` and `audit: not_run`, and retains evidence for later audit. It does not fall back after an audit failure. This new profile is not release qualified. Unsupported partial/non-direct production requests fail with `ProductionRequiresCompleteDirectWorkflow`; invalid policies/modes fail validation. Parser support for another setting is not qualification of that setting.

`--surface-carrier adaptive|nodal` controls construction, independently of verification. The carrier default is adaptive. The tested sphere and thick blend use adaptive; passage explicitly uses nodal. Do not infer that other carrier/geometry combinations are qualified.

Production saves the cost of the deferred replay. Running a separate offline audit later pays that cost again and can be slower overall than one audit-mode run. A later offline pass is a separate result; the original production receipt remains `audit: not_run`.

## Keep the evidence if you want a later audit

After a successful production run, keep its entire output directory intact, including source/closure records, manifests, raw/native operands and nested background evidence. An exported OpenFOAM mesh alone is insufficient. Capture and preserve the trusted `VERIFICATION.json` hash immediately after successful production closure, outside the bundle. Do not establish a new trusted root by rehashing a bundle after an unexplained modification. Keep this compatible runtime too.

Example after the sphere production command in QUALIFICATION.md succeeds:

```powershell
$bundle = (Resolve-Path 'results/sphere_verify1_production_01').Path
$root = (Get-FileHash -LiteralPath (Join-Path $bundle 'VERIFICATION.json') -Algorithm SHA256).Hash.ToLowerInvariant()
$verifier = (python -B offline_audit.py --print-verifier-identity).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify verifier' }
$expectedVerifier = (Get-Content -Raw '../RELEASE.json' | ConvertFrom-Json).runtime_identity
if ($verifier -ne $expectedVerifier) { throw 'Incompatible verifier runtime' }
@{ bundle=$bundle; expected_root=$root; verifier_identity=$verifier } | ConvertTo-Json | Set-Content -Encoding utf8 '../sphere-production-trust.json'
```

Run this only after `mesh.py` exits zero and its closed `VERIFICATION.json` says `acceptance_status: production_accepted`. The retained tested roots in the qualification records identify those original runs; a new run needs its own captured root. Keep the trust record in a safe location and use a new name for each run.

Later, from this same runtime directory, load that preserved record and use a fresh, separate output path:

```powershell
$trust = Get-Content -Raw '../sphere-production-trust.json' | ConvertFrom-Json
python -B offline_audit.py --bundle "$($trust.bundle)" --expected-root "$($trust.expected_root)" --verifier-identity "$($trust.verifier_identity)" --output '../offline-audits/sphere_offline_01' --timeout-seconds 10800
```

Keep offline audit output outside the standalone runtime directory, as shown above. The output must be fresh and outside the production bundle. Require exit zero, `AUDIT_RESULT.json` status `audit_passed`, all twelve replay obligations passed and closed verifier source custody. A wrong root, changed retained evidence, incompatible verifier or overlapping/existing output is refused. Offline replay does not remesh, repeat the optimizer search, create fresh geometry authority, rerun OpenFOAM conversion or rerun checkMesh. It does replay retained commits using the required numerical/native predicates; zero new subprocess scopes does not mean zero native work.

Qualification is limited to the three fixed cases, settings and configured host. See QUALIFICATION.md for retained compression, warnings and scientific limits. [Detailed qualification](../qualification/verification_candidate_20261028/records/PASSAGE_VERIFY3_REPORT.md) and [compact evidence index](../qualification/verification_candidate_20261028/README.md) accompany the runtime. The archive CRC/member hashes and help smoke checks establish package integrity only. Clean-machine portability, arbitrary geometry, CFD accuracy and stable performance are not qualified.
