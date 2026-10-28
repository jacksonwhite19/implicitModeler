# Local source-worker ownership contract, candidate 1

Status: independently reviewed and implemented as an unqualified candidate. The
52 distinct focused controls passed across preserved snapshots; the sample
benchmark stopped at the archived CASE acceptance-profile gate before evaluation.
No full mesh or performance qualification follows from this contract. This candidate
starts at `1c363474454c78b6d919af9305b7876ecd97415c`; the delivered PERF12
candidate and accepted bundles remain unchanged. This milestone permits local
implementation and focused controls, not full meshing qualification or promotion.

## Scope and explicit selection

`mesh.py --source-workers 1|2|4` selects source-segment execution only. Default is
1 and calls the original serial evaluator. Audit recomputes serially. A request
for 2 or 4 uses that many local Windows spawn processes only for an eligible
production handoff population with at least 1,024 segments and at least one
vertex per worker. Smaller inputs, non-Windows hosts, custom methods/callbacks,
nested query scopes or unsupported source owners use the original serial path
BEFORE any parallel computation. Record requested/effective workers and the
reason. Do not automatically choose a worker count from CPU availability.

The existing `source_handoff._context` eligibility is required, not replaced or
broadened. Parallel execution further requires the stock GeneralSdfAuthority,
ordinary boundary context, a null NumPy error callback, and its genuine existing
case/connection binding. The existing raw-proof validator supports depth at most 6 (64 leaves/127 boxes), so larger source depth budgets select serial before work; do not impose those limits after computing a parallel proof. Current sphere/blend/passage depth is 6. Missing capability means documented serial fallback.
A changed binding, bad source custody or a failed parallel operation is fatal;
it never triggers an unsafe serial fallback.

## Parent and worker custody

The handoff creates one private local pool lazily and reuses it for eligible
populations in that handoff. Do not spawn per segment. Capture the parent's
exact original source object/context and first height. Derive the case path and
hash from its verified connection, not a caller's replacement geometry packet.
Hold the case and current source/native runtime files read-only until pool
closure. Parent and workers must report the same complete runtime inventory.

The parent owns fresh pipes/process handles and a random session identifier.
Each worker waits for the parent to place it in a Windows kill-on-close Job
before it creates a canonical host. The parent retains that Job handle; worker
and canonical-host descendants cannot escape by becoming orphans. Close on
cancellation, fatal failure or parent exit. Confirm zero active contained
processes. Do not identify ownership by a bare reused PID.

Each child imports the current owned modules, holds its own source/case custody,
creates its own genuine canonical geometry/source through `cli.geometry`, and
uses the unchanged stock `qualify_segments`. It opens its normal query,
verified-operation and arithmetic scopes. It receives immutable coordinates,
not a pickled authority, Expression owner, lock, ContextVar or cache. Compare the
semantic context (geometry/expression/root/units, exact policies and budgets,
evaluator source bytes/paths, floating-point word and NumPy policy) with the
parent; process-local object IDs must remain local. Reject reconstruction or
context mismatches before evaluating a shard. No shared mutable cache exists.

This is an owned local execution boundary, similar to an owned native worker.
First-normal-height results are still computed by the original source method;
the current payload is not a new independently replayable height certificate.
Acceptance therefore requires custody of the actual spawned worker, its exact
code, bound request, execution and clean closure, plus the full original proof
payload. An arbitrary serialized `passed` boolean or historical proof file is
never an issuer. No distributed attestation or general remote proof service is
introduced.

## Deterministic population and result validation

Freeze the complete actual `(layers+1, vertices, 3)` binary64 positions in the
parent. Bind their full byte identity, shape, first height and population
sequence number. Predetermine contiguous unique original vertex ranges. Use a
barrier between layers; first-height predicates apply only to layer 1. Check
session, sequence, worker/range/layer and exact operand identities on every
request/reply. Check the complete unique coverage before issuing a result.

For each shard validate the original result's geometry, segment count,
start/end hashes, height role, complete ordered raw segment proofs, positive
endpoint/progress fields and height bound. Validate the current raw proof
structure/source-expression binding using the existing proof validator. Keep
its original encoding. Concatenate raw proofs in original order, use the
original full-row array hashes and unchanged maximum-height-error reduction.
No threshold, arithmetic method, rounding or numerical reassociation changes.
The merged layer dictionaries must retain the original serial schema/identity.

Only the private parent pool factory can issue a single-use in-process
population capability, after verifying all worker execution/context bindings.
It binds the same parent source/context, height and frozen positions. A
detached dict, a different owner, a changed population, a repeated capability
or an expired pool is refused. The capability supplies the complete proof
payload to the handoff, not an acceptance boolean.

## Refusal, fatal error and lifetime

Every worker resolves its assigned prefix in the current layer. On a known
scientific refusal, collect every sibling outcome in that layer before choosing
the lowest original vertex index. A fatal/authentication/resource/transport
error takes precedence over a scientific refusal. Preserve the original error
class/arguments and `SourceSegmentRefusal` classification. Translate only named
shard-local segment indices and start/end array-envelope hashes to the original
row domain; retain raw shard diagnostics for audit. Unknown diagnostic domains
are fatal rather than guessed. Parallel speculation changes attempted-call
counts; record that distinction instead of claiming serial abort populations.

A known scientific refusal may return to the existing collision proposal loop
only after every worker verifies a healthy source boundary and discards that
population. It issues no population capability. A later candidate is a new
population through the same still-valid pool, as the original serial loop
allows. Any fatal failure poisons the pool and aborts the run; no serial retry.

All waits and parent validation consume the caller's remaining whole-worker
deadline. Source-pool cancellation is cooperative at layer boundaries and
forcible through its owned Job on failure/deadline/close. For an explicit parallel selection on Windows, the outer supervisor must own
a second kill-on-close Job containing the complete mesh worker and descendants.
The mesh worker blocks at a start gate before importing evaluator/planner modules;
the supervisor assigns its retained process handle to that Job before release.
Before release, supervisor death or a bounded gate timeout exits without opening
a source host. After release, supervisor death closes its non-inherited Job
handle and kills the worker/pool descendants. An unconditional context-manager
finally terminates, waits and confirms zero contained processes on cancellation
(including KeyboardInterrupt/SystemExit), monitor failure or normal exit. The
existing serial launch is unchanged. Test outer-supervisor hard exit as well as
pool-parent exit and normal cancellation. This is a narrow launch/lifetime fix;
no unrelated supervisor rewrite is part of this contract. Output stays within existing run ceilings.
No shared metadata polling file is needed; pipe closure accepts EOF or Windows
BrokenPipe as transport failure. Operational metadata publication, if needed,
may use the bounded Windows sharing-race repair from PERF13, never predicate
retry.

## Existing handoff and consumer map

1. `mesh.execute` passes explicit worker/resource selection to `live_handoff`.
2. `_HandoffState.qualify` keeps its existing source/height/context guards. Its
   eligible dispatch uses the owned population capability; serial/default and
   unsupported-before-start paths retain `_segment_population` unchanged.
3. The existing complete-population validation, immutable payload retention,
   collision `accept` gate and single-use consumer checks remain mandatory.
4. `consume` rechecks the exact original parent context/positions. For retained
   parallel evidence it obtains clean worker/source closure before returning
   the proofs; failure is fatal. Changed-consumer fallback remains the original
   eligibility behavior and must not reuse the old parallel payload.
5. `prepare` records requested/effective execution and a closed bound worker
   execution record with the existing SOURCE_EVALUATION evidence. Retention,
   completion and offline validation bind that record to the full source proof
   identity, actual positions, height, geometry and producer runtime. Existing
   audit/producer obligations remain. Production profile identity changes and
   the new candidate has no inherited full-mesh qualification.
6. Exiting the handoff invalidates pending/accepted capabilities and closes all
   children, including when no consumer runs. No process or cache survives a
   run; nothing is reused across runs.

## Focused verification and stop rule

After contract approval, inspect implementation independently before numerical
claims. Test actual serial/4-process proof equality on fixed representative
passage, compressed blend and sphere operands, genuine handoff accept/single-use
consume and original prepare/retained-evidence validation. Test source/case/FP
and coordinate changes, incomplete/duplicate shards, crash/cancel, mixed
success/refusal, earliest failure ordering, expired evidence and clean process
closure. Use a small fixed sample benchmark with setup/transfer/assembly
included; reuse correctness evaluations as timings where possible.

Declare finite test budgets before execution. A scientific mismatch or failed
authentication/resource gate stops the affected campaign. Preserve failed
evidence; only routine interface plumbing may be repaired without changing
numerical inputs/gates. No full mesh, passage campaign, storage deletion,
packaging/promotion, push or automation is authorized. The final decision is
go/no-go for proposing a small complete control, not authorizing that control.
