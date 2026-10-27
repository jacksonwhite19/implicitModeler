# Phase 1A audited artifact finalization

## Behavior and boundary

V3.5 already connects the V3 complete prism stack to the existing V4 direct outer workflow. Phase 1A binds the producer-checked joined state to saved operands, native export and final supervisor acceptance. It does not change prism planning, transition construction, background construction or scientific acceptance predicates.

Audit remains the sole behavior. Full cold replay and all twelve producer checks remain mandatory. `V35V4DirectComplete/1.full_mesh_complete` retains its old meaning: cold replay passed. No production verification selector or bypass exists.

## Consumer and change map

| File | Responsibility / consumer |
|---|---|
| `standalone/artifact_finalization.py` | Compare eight JOINED arrays and canonical metadata against the checked producer state; rebuild ownership from producer cells after serialized equality; authenticate twelve proof receipts; compare five COMPLETE arrays; bind pinned export input and actual native output; seal manifest; always invoke existing cold reader. |
| `standalone/export_mesh.py` | Accept private custody scope and expected archive hash. Guard archive before reading, polyMesh immediately after conversion and before native readers, raw native receipts before decoding, mapping and export report. Add streaming native-file inventory hashes. Existing fidelity predicates remain unchanged. |
| `standalone/direct_connection.py` | Require caller-owned finalization lifetime; establish bundle after producer audit; use guarded export; seal before live lease closes; run mandatory cold reader after lease closure; return finalization descriptor only after cold success. |
| `standalone/audit_completion.py` | Hold an overlapping worker/supervisor custody interval; verify completion roots and cold receipt; publish final acceptance only after worker exit and clean source/resource closure. |
| `standalone/mesh.py` | Own worker lifetime; monitor handoff; revoke legacy success on any nonzero worker exit; persist closure records; publish final verification last. |
| `standalone/tests/test_artifact_finalization.py` | Nine serialization, proof-root, export and corruption/refusal controls, including actual tiny native export. |
| `standalone/tests/test_audit_completion.py` | Nine process/handoff/publication controls, including failed worker before handoff and after ACK. |

`direct_readback.py`, the twelve check implementations and legacy manifest schema are unchanged. Existing consumers accept the additive export fields and finalization payload. The ordinary CLI passes the private finalization lifetime; direct internal callers must supply it. The V4 direct route now requires supervisor handoff for accepted completion.

## Binding and publication contract

1. Snapshot checked geometry, proof, arrays, canonical metadata and owners. Compare real saved JOINED operands and authenticated twelve-receipt population while file handles are held.
2. Write COMPLETE from producer operands; compare dtype, shape and array hashes; hold and hash the actual archive. Export receives this expected archive pin.
3. Run existing conversion, exact point/cell/face/owner/patch fidelity and checkMesh. Hold native inputs/outputs at their producer/reader boundaries and inventory their bytes. Compare COMPLETE again.
4. Write `FINALIZATION.json` (`V35AuditedArtifactFinalization/1`) with audit-only identities and payload root. Write legacy `MANIFEST.json` (`V35DirectArtifactManifest/1`) including the finalization receipt. Preserve held handles through cold replay.
5. Launch actual legacy cold reader. Require exit zero, passed receipt, expected manifest hash and all twelve semantic replays. Cold replay is not a separate algorithm; its collision review reuses certified native classifications with fresh population/coverage checks. It does not rerun conversion/checkMesh.
6. Worker guards CASE, root RESULT, direct RESULT and READBACK, then creates `FINALIZATION_HANDOFF.json`. Supervisor must acquire the same expected roots while the worker is alive before writing ACK. Late acquisition cannot repair a missing overlap.
7. Require worker exit zero, source custody closed/passed without additions or changes, and no supervisor resource stop/monitor error. Guard persisted source/supervisor closure records. Atomically write `VERIFICATION.json` (`V35VerificationManifest/1`, `audit_accepted`) last. Publication failure refuses acceptance and revokes legacy root success.

The final verification file is the acceptance commit marker. Legacy RESULT may appear earlier while work is provisional; downstream candidate consumers must require final VERIFICATION acceptance. Any failed worker, source closure, resource limit, missing handoff, changed root or publication failure prevents this marker. Existing failed-root and cleanup behavior remains active.

## Coverage and explicit limits

The pre-cold manifest covers the payload present when sealed. New `readback_audit` diagnostic artifacts generated during cold replay are outside that base manifest. The cold summary and completion roots are pinned by the later handoff. Do not claim that all cold-generated raw diagnostics share the base manifest root.

Ownership is rebuilt from the producer cell representation after canonical metadata and array equality is established; it is not literally rebuilt from newly decoded cell objects. Guards use the existing configured Windows file-custody model. They do not establish an adversarial host security boundary.

Thirty tests passed in one combined run: finalization nine, protocol eight at that revision, direct connection ten, source custody two and export one. A final targeted run passed the new pre-handoff failure test, post-ACK failure test and clean closure test. This covers 31 distinct tests overall; the final targeted run follows the last generic nonzero-exit fix. Existing regressions were not rerun broadly afterward.

The tiny fixed four-prism native fixture executes gmshToFoam, contractAudit and checkMesh. Its deliberate native-file write probe is denied between contractAudit and checkMesh; the guard was acquired before both readers. Synthetic producer proofs and protocol cold receipts are declared test preconditions, not scientific qualification. Protocol tests use real child processes and supervisor/source/resource handling with a fixed helper in place of full meshing. Independent retained-sphere inspection confirms array/metadata/order compatibility only. No candidate full-pipeline run, full-fixture comparison, cold-success campaign or speedup measurement was made.

Two review blockers were fixed: native files were initially guarded after their readers, and failed workers could retain legacy success before/after handoff. Focused fault tests cover both. Three final read-only reviews pass with exact source hashes and saved-evidence limits.

## Decision

Phase 1A is suitable as an isolated, locally reviewable audit-only implementation baseline. Proceed only to separately authorized Phase 1B implementation, with its distinct composed contract and review. No approval for production use, cold replay omission, general-shape qualification, clean-machine portability, CFD accuracy or performance claims follows from this milestone. Partial binding cost and net savings remain unmeasured; no artificial full replay was run to estimate them.
