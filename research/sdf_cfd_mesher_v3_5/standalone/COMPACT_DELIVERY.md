# Compact solver delivery

PERF19 is a separate storage and CLI candidate. V3 prisms still connect to the existing V4 direct transition/background path and joined acceptance/export. Meshing algorithms and scientific gates are unchanged.

Retention is independent of verification and source workers. `--retention full-evidence` is the default. Full evidence is needed for a later complete offline audit. `--retention compact-delivery` first runs the normal full workflow in fresh invocation-owned scratch. It authenticates the accepted full bundle and publishes a separate solver delivery. Optional `--immediate-audit` requires production mode and a separate passing audit before compact publication. Source workers remain explicit 1/2/4, with serial default.

## Existing accepted bundles

Run from this standalone folder on the configured Windows host. Use a trusted root from successful publication or retained independent review. A freshly computed hash of an untrusted bundle is not its own authority.

```powershell
python -B compact_delivery.py export --bundle C:\path\accepted-full --expected-root TRUSTED_VERIFICATION_SHA256 --output C:\path\new-compact
python -B compact_delivery.py check --bundle C:\path\new-compact --expected-root PUBLISHED_COMPACT_SHA256
```

To attach an existing separate audit, add `--audit-result C:\path\audit\AUDIT_RESULT.json --expected-audit-root TRUSTED_AUDIT_SHA256` to export. It must bind the same full root and closed verifier sources. Audit-mode full roots also need their original external source and supervisor closure records. Missing or changed records refuse. Historical export never deletes its source.

## New runs and cleanup

Add `--retention compact-delivery` to the normal complete mesh command. The fresh `--output` names the compact destination. Full staging is created beside it as `.v35-compact-scratch-<random>`. External `<output>.retention.json` records scientific acceptance, separate audit, publication and cleanup independently. An accepted full mesh remains accepted if a later audit or storage stage fails. Failed stages preserve scratch and diagnostics. Outer output checks count scratch and delivery bytes. Windows RSS checks sample the parent and child tree. Existing child limits also apply. Shared runtime log history and aggregate Linux RSS are outside this accounting.

Add `--cleanup-scratch` at invocation start to request deletion of only that new scratch after verified compact publication and any requested audit. Historical folders cannot be adopted. Before the first deletion, a fresh sibling `<output>.cleanup-authorization.json` records the verified producer root, any requested audit root/status, compact manifest root, live scratch identity, Windows process closure, actual native census and staged logical size. Schema `V35CompactCleanupAuthorization/1` is an immutable operational record. The code flushes and atomically commits it, checks its expected bytes/hash and holds it through deletion. Receipt failure or tampering prevents deletion. It requires no external reviewer acknowledgment and contains no full scientific proof archive. The existing retention sidecar records the actual cleanup result and receipt hash after the attempt. This preserves the finalized compact manifest. A live ownership capability binds the fresh marker, path, directory identity and PID birth. Cleanup keeps the delivery guarded, requires absent owned Windows processes, and always checks Linux root-bound processes and recorded groups, even with zero scope receipts. Success inspection never signals. Unknown or live processes refuse deletion. Files are removed through pinned Windows handles. Partial failure leaves the published delivery valid and reports actual failure.

Ordinary failure uses the frozen exact-folder backend cancellation and recorded birth/argv/PGID retirement, followed by an exact-root census. Hard death of the outer controller can still leave WSL work. Outputs must be outside the standalone runtime or under its excluded `results` folder. Custody exclusions are unchanged. Existing destinations, overlap, traversal, aliases and reparse points refuse. This is a configured local Windows filesystem workflow. Other filesystems and clean-machine portability are unqualified.

## Contents and publication

`V35CompactSolverDelivery/1` includes every authenticated recursive `polyMesh` file, including zones and sets, copied byte for byte. It includes CASE settings, inline Rhai geometry, units, source CLI, consumer inventory, source/native/manifest pins and check records. Records retain actual repair classification and assurance scope, compression, quality, warnings and separate audit status. They are not replay proofs. Solver fields, boundary conditions and solver dictionaries are not supplied. CFD qualification is not claimed.

A fresh destination is provisional until flushed `COMPACT.json` is committed by atomic same-handle, no-overwrite file rename. This is an atomic success marker, not an atomic directory rename. Source and payload guards stay held across commit. Generated metadata is checked against expected bytes after guards. Interrupted directories without the marker are incomplete and preserved. Parent handles prevent replacement; the root remains nonempty through a pinned owner file. Windows rejects conversion of a nonempty directory into a reparse point; see the [Microsoft filesystem specification](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-fsa/4aeefef8-92c3-4abc-af7a-a610caf8a165).

The checker validates exact population, sizes and hashes against the supplied compact root. It performs no scientific replay. `full_offline_replay_available=false`; the full offline entry rejects compact input. Keep full evidence if later replay is required. Full staging and the compact copy coexist before optional cleanup. Logical size measurements are not physical allocation or memory savings.

## Validation scope

Focused serialized/native-export, real Windows child/handle and injected-fault controls test storage and lifecycle boundaries. Synthetic proof premises are labelled. Exports of accepted sphere, blend and passage bundles check exact solver bytes and preserved full inventories. PERF19 contains no new meshing campaign. The new complete CLI lifecycle and cleanup after a live full mesh remain unqualified pending a separately authorized control. Earlier numerical qualification stays tied to its original runtime.
