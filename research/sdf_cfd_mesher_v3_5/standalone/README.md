# Mesher V3.5 verification candidate

PERF9 dispatch 4/5 is complete: trusted provenance, isolated exact two-library restoration, 1,916-file early external preflight and nine focused controls pass. The first tiny smoke has a preserved malformed-scope limitation; reviewed V2 formatter repair and fresh tiny smoke pass with valid exact argv, closed 1,885-file custody, Gmsh 4.15.2 one-point synchronization and zero generation/optimization. Runtime commit d14d7592; full-mesh qualification remains pending. PERF8 failed evidence, original/published/frozen packages and shared updated system remain unchanged. No full mesh, passage, push, promotion or automation ran. Recommend NEW sphere production/offline then blend production/offline only after review and authorization. Full report: chat outputs/PERF9_REPORT.md. Callback before final, then stop; counter 4/5, next 5/5 Bird's Eye.

This PERF9 candidate uses `V35IsolatedBrotliRuntime/1`: two authenticated original decoder/common libraries under `v4/runtime/isolated_brotli`. Original external hashes and manifests stay unchanged. Candidate child processes prepend only this library directory to their inherited loader path; the system package stays updated. The normal supervisor checks all 1,916 effective external byte pins and loader resolution before geometry, while holding local source/binary custody. Failed or unknown profiles refuse before the worker launches. The late runtime gate also requires actual decoder/common process mappings at the exact candidate paths and rejects foreign/deleted mappings or preloads. Production profile identity changes. This restores original bytes at new paths; historical environment qualification is not inherited. Full sphere/blend qualification remains pending. Use the normal CLI, not its hidden worker transport.

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
# PERF8 retained source-evidence binding

Production now requires `V35RetainedSourceEvidence/1`. Prepare writes and holds the exact source evaluation and full source-proof population through publication. The parent binds the accepted source-proof identity. The completion manifest also authenticates both shell files and the actual native sealed prism positions. Completion and offline audit validate method, mode, population, coordinates, source identity and first height. Offline reports this as authenticated original-run evidence; it does not recompute live source predicates. The twelve joined checks retain their separate replay obligations. This changes the production profile identity. Existing accepted packages require their own compatible verifier. The one PERF8 sphere production attempt reached shell admission, then stopped at an existing external Brotli-library identity mismatch before farfield construction. No accepted production root exists; the changed runtime/profile remains unqualified. No offline or blend run started. Published566a remains authoritative. See chat outputs/PERF8_REPORT.md.
