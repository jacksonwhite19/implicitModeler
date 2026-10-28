> Qualification update: the staged fixed-sphere sequence described below has now passed on runtime714b83dd. See [closed report](qualification/sphere_verification_20261027/REPORT.md). Other proposed fixture work remains unrun and unauthorized; original implementation-stage limits below are dated history.

# Phase 1B verification contract and handoff

## Scope and behavior

Phase 1B adds an explicit verification choice to the existing generic Rhai/SDF -> V3 complete prism stack -> V4 direct transition/background -> export path. No extra transition is constructed. Audit remains default and preserves the legacy cold-required result contract. Production is supported only for `--stop-after complete` with validated `V4DirectFillEngineeringPolicy/1`. Partial stops, historical finite-transition backgrounds, unknown policies, profiles and mode combinations refuse before surface/source queries. Carrier adaptive/nodal selection remains independent.

Production changes one verification obligation: it does not invoke the separate complete saved-artifact semantic replay. It records audit `not_run`; a failed audit is never retried or promoted as production. All existing construction, prism planning, native meshing, conditioning, apex repair, twelve joined producer checks, physical minima, quality, source, collision, domain, filling, native export and source/resource closure remain. No storage optimization or witness removal is included. Runtime cost and net savings are unmeasured.

## Composed profile and request

`verification_policy.py` defines `V35DirectProducerAcceptance/1`. Its identity composes the explicit production publication/replay obligations with the unchanged physical shape contract identity and backend method identity. The old `DirectFillEngineering/1` and policy `all_twelve_fresh` remain component inputs; their cold-required whole-path acceptance is not claimed for production. Shape applicability records retain their original component scope. No historical approval is relabelled as production approval.

The resolved CASE contains a strictly validated `V35VerificationSelection/1` extension with requested/effective mode, origin, profile, background identity and stop target. The copied CASE loader accepts exactly this validated optional extension; unknown fields/versions still refuse and legacy cases still load. This is a local package extension, not a frozen reference modification. Selection is written before construction and bound through the CASE hash into parent/provenance and final roots.

The selected mode does not change construction settings. CASE/provenance hashes necessarily differ between modes; later qualification must compare physical arrays while allowing only understood policy/provenance differences. No deterministic or equivalent-output claim follows from these focused tests.

## Schemas and acceptance consumers

| Artifact | Audit | Production |
|---|---|---|
| Direct result | `V35V4DirectComplete/1`; legacy cold-required success | `V35ProductionDirectResult/1` |
| Root result | Legacy audited fields | `V35ProductionMeshResult/1` |
| Finalization | `V35AuditedArtifactFinalization/1` | `V35ProductionArtifactFinalization/1` |
| Artifact manifest | `V35DirectArtifactManifest/1` | `V35ProductionArtifactManifest/1` |
| Handoff request/ACK | `V35AuditGuardHandoff/1` | `V35ProductionGuardHandoff/1` |
| Final verification | `V35VerificationManifest/1`, audit_accepted | `V35ProductionVerificationManifest/1`, production_accepted |
| Offline result | Existing normal cold READBACK path | `V35RetainedDirectAudit/1`, audit_passed or audit_failed, in separate output |

Production root/direct results omit `passed`, `full_mesh_complete`, fake READBACK hashes and `all_twelve_fresh` aliases. They carry `geometry_complete`, `producer_checks_passed`, `artifact_fidelity_passed`, `verification_policy`, `independent_audit_status=not_run` and `acceptance_status=pending_supervisor`. Before artifact finalization the direct provisional has `acceptance_status=provisional`. Only final VERIFICATION carries `acceptance_status=production_accepted`. A schema-aware downstream consumer must require that final root and expected profile; generic legacy success readers must fail closed. The legacy reader explicitly rejects production manifests.

Final verification records original producer runtime identity, effective selection and per-check receipt/geometry provenance. Preservation is `reused_with_binding` from the earlier within-run result; other joined checkpoints recompute their existing predicates. Collision rows describe native classification plus fresh coverage and persisted witness readback. Native export is recomputed, artifact binding is derived, and only separate semantic replay is `not_run`. This does not expand the old policy name into a claim that every primitive independently reruns.

## Finalization and closure

Phase 1A guards and byte binding remain. Eight JOINED arrays, canonical metadata, ownership and producer proof roots are compared; five COMPLETE arrays are written and checked; actual export input, mapping, raw native files and polyMesh are held and pinned. Native guards precede both contractAudit and checkMesh. All twelve receipts are present and authenticated. Producer collision-witness/filling/repair verification and real live-parent origin checks still execute before sealing.

Production additionally checks the identities formerly supplied only by cold reconstruction: canonical core metadata from actual tet rows/IDs, prefix order, parent cell identity, inherited points/prisms/IDs/wall/cap hashes, policy and CASE equality, parent/provenance generation and construction counts, and exact handoff/proof/shape/export/provisional agreement against caller-owned expected roots. A missing bridge refuses sealing. These are identity/conformity checks, not a new geometric acceptance path.

Worker guards overlap supervisor guards while the worker is alive. The supervisor binds the requested mode, validates new root/direct schemas against sealed provisional contents and refuses any production READBACK masquerade. Final publication requires worker exit zero, clean source closure with no additions/changes, and no resource/monitor stop. Production copies source/supervisor closure records into the bundle and hashes them before the atomic verification marker. Failed workers revoke legacy success both before and after ACK; late mutation/publication failures prevent final acceptance.

The pre-replay artifact manifest does not cover new cold diagnostic files. Audit pins the cold summary through handoff; production has no cold diagnostics. Existing configured Windows file guards remain the custody model, without an adversarial host-security claim.

## Offline audit

The caller supplies a complete production bundle, trusted expected SHA256 of its VERIFICATION, explicit supported verifier runtime SHA256 and a fresh separate output. `--print-verifier-identity` reports this installed verifier's selected source/native/runtime byte identity. Offline execution holds those bytes and requires exact equality to the caller's identity; unknown or changed verifiers refuse. This supports this installed implementation and known profile, not arbitrary future verifier compatibility.

The auditor holds and authenticates the final root, completion files, request, self-contained original closure snapshots, artifact manifest and all retained witnesses. It reconstructs parent/joined state and invokes the existing semantic checks, committed conditioning/apex edit replay, shape recomputation and collision-classification/coverage witness review. It does not remesh, rerun search, issue a live lease or grant fresh source authority. As in legacy cold audit, it does not rerun native conversion/checkMesh or native collision classification. A successful audit is evidence about retained operands under the stated implementation, not a new source qualification.

`background_conditioning.replay_output_scope` changes only diagnostic output placement. It routes replay admission diagnostics outside the source bundle and resets on exceptions; legacy/live output selection is unchanged. No source bundle file is modified, and no proof files are copied back. The separate `AUDIT_RESULT.json` records pass/fail, expected production root, verifier and producer identities, and `fresh_source_authority=false`. Failure is recorded and re-raised; production remains `audit=not_run`. Existing output, overlap, missing/tampered witnesses, unknown profiles, changed roots and failed replay refuse. All witnesses remain retained initially.

## Source and consumer map

| File | Change |
|---|---|
| `standalone/verification_policy.py` | Profile, strict selection, truthful execution record, new result dispatch; sole audit/production replay branch. |
| `standalone/mesh.py` | CLI/default/origin, early selection, separate production root schema, policy-aware supervisor handoff. |
| `standalone/v4/tools/standalone_case.py` | Strict optional versioned extension validation; legacy field sets remain supported. |
| `standalone/direct_connection.py` | Same producer construction/check pipeline; bind expected production roots, finalize, select replay obligation. |
| `standalone/artifact_finalization.py` | Cold-only identity bridges and separate production finalizer/manifest. |
| `standalone/audit_completion.py` | Strict production root/handoff validation, self-contained closure snapshots, final acceptance last. |
| `standalone/direct_readback.py` | Preserve legacy entry; explicit production replay entry/output; reject unknown/production schema in legacy path. |
| `standalone/background_conditioning.py` | Scoped replay diagnostic output; no acceptance-predicate change. |
| `standalone/offline_audit.py` | Trusted-root/verifier custody, input authentication, isolated replay and failure recording. |
| `standalone/tests/test_verification_policy.py` | Mode/configuration, real finalization/serialization/child closure, version/tamper/offline faults. |
| `standalone/tests/test_background_conditioning.py` | Actual unchanged replay predicate with separate diagnostic output and source hash preservation. |

Historical retained diagnostic tools and public_cli_reference fixtures are not consumers of the normal new completion contract. They remain unchanged and do not gain production support. Existing legacy result/contract validators are not relaxed to accept production aliases.

## CLI examples (prepared, not executed as mesh campaigns)

From standalone, audit is selected by omission or explicit `--verification audit`. Example production invocation with the retained sphere construction settings:

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/sphere.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --surface-carrier adaptive --background fixtures/background_direct_v4.json --output results/phase1b_sphere_production_01 --stop-after complete --verification production --timeout-seconds 43200 --memory-limit-gib 16 --output-limit-bytes 16000000000
```

Use the identical command and fresh audit output with `--verification audit` for a later pair. Retain the trusted publication root externally before offline inspection. The following placeholders must be replaced with trusted values:

```powershell
python -B offline_audit.py --print-verifier-identity
python -B offline_audit.py --bundle results/phase1b_sphere_production_01 --expected-root <trusted-verification-sha256> --verifier-identity <supported-runtime-sha256> --output results/phase1b_sphere_offline_01 --timeout-seconds 10800
```

## Validation and limits

Focused controls use real serialized arrays, expected-root comparisons, file guards, actual tiny native export and real child/supervisor closure. They cover omitted/audit selection, carrier independence, early unsupported refusal, strict actual CASE loading, production skipping only intended replay, audit failure without downgrade, required binding, parent/policy/proof faults, schema mismatch, tampering/missing witnesses, late write denial, offline output isolation, verifier refusal and failed-audit recording.

The synthetic complete producer and successful replay premises in protocol tests are explicit. A mocked successful offline replay tests the wrapper only. Real replay of the deliberately incomplete tiny bundle fails and records failure. A separate actual conditioning component replay verifies output isolation with unchanged predicates. No successful full production offline replay, candidate full mesh, full audit/production equivalence, new supported shape or performance result is qualified.

The combined regression attempt ran 68 tests:67 passed; one legacy expected artifact-tamper error was pre-empted by new schema checking. The check order was restored with strict schema refusal preserved, then four targeted reader controls were run. Exact final results and timings are retained outside runtime in PHASE1B_TESTS.* and PHASE1B_READER_TESTS.*. Source and saved-result review must pass before local promotion.

## Proposed next qualification sequence (not authorized or run here)

1. Start with one smallest established complete control: sphere, explicit adaptive, five rows, 20 micrometre H1, growth1.3, surface size0.0002m, existing direct policy and unchanged declared operational bounds. Run candidate audit first in a fresh output. Require all producer predicates, actual native fidelity, full cold replay and final source/resource closure. Stop on failure; no fallback or tuning.
2. Only after that control passes and further execution is authorized, run production with identical construction/mode/settings in another fresh output. Require all same producer/fidelity/closure obligations and audit `not_run`. Compare all five COMPLETE physical arrays by dtype/shape/bytes, plus inherited stack/shape populations/warnings/compression. Expected verification/provenance hashes can differ; any unexplained physical difference stops work.
3. Audit that complete production bundle offline using the externally retained root and pinned supported verifier identity into fresh separate output. Require successful existing semantic replay and unchanged source-bundle inventory/hashes. Compare relevant retained acceptance populations and proofs, without claiming a live source lease or native conversion rerun.
4. Review the sphere pair and offline result before proposing passage nodal and thick-blend adaptive pairs. Preserve each retained construction mode/settings exactly. Only later measure stage costs and net savings on passing comparable runs. Do not infer speedup from removed-call timing alone.

Decision: source/focused-protocol candidate only. No production-use or numerical-equivalence promotion follows from this milestone. The frozen package remains the last qualified complete reference.
