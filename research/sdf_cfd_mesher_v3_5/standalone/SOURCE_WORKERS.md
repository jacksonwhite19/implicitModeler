# Optional local source workers

The experimental candidate adds `mesh.py --source-workers 1|2|4`. The default
is 1. Audit uses the original serial source recomputation. For production, an
explicit selection of 2 or 4 can evaluate source segments in local Windows
processes. This does not change the V3 prism construction or existing V4
direct outer/background, acceptance and export methods.

Only the stock eligible source/context can use workers. Each population needs
at least 1,024 segments and at least one vertex per worker. Custom evaluators,
active nested query scopes, a NumPy callback, source depth above 6 and smaller
inputs select the original serial evaluator before parallel work starts.
`SOURCE_EVALUATION.json` records requested/effective workers and the reason.
Selection is explicit; CPU availability does not choose a worker count.

Workers open genuine source owners from the parent's verified CASE binding.
The parent holds exact code/runtime/input custody, checks semantic and arithmetic
context, partitions original vertex order and uses a barrier between layers.
The stock predicates and complete ordered raw proofs remain unchanged. Failed
authentication, worker crash, resource exhaustion or transport failure aborts;
there is no serial retry after failed parallel execution. Known scientific
refusals retain the earliest original vertex and can return to the existing
collision proposal loop only after all current-layer workers resolve safely.
Speculative sibling work can change attempted-call counts.

The original pending → selected collision outcome → single-use consume gates
still apply. Workers and their source hosts must close before the consumer
receives retained proofs. First-height evidence comes from owned current local
execution of the stock method; a saved scalar or worker JSON is not a new
independent certificate. Producer completion and offline authentication bind
the execution record to actual positions, full proofs, CASE and producer runtime.
Offline replay still grants no fresh source authority.

The explicit parallel Windows launch contains the complete mesh worker in a
supervisor-owned kill-on-close Job. The worker waits before evaluator imports;
the supervisor releases it only after Job assignment. A nested Job contains the
source pool and its hosts. Cancellation and parent death retire owned processes.
Existing whole-run time, memory and output ceilings remain required.

The producer profile includes the new execution-owner contract. Earlier full
mesh qualifications belong to their recorded runtime/profile; they do not
qualify this candidate. Focused source controls support saving the experimental implementation. Sample
timings are unavailable. A complete control remains NO-GO until a fresh
current-profile CASE sample plan is reviewed; no full-run speed or release
qualification follows.
The implementation milestone report is in the task's `outputs/PERF14_REPORT.md`.

Contract and consumer map: [SOURCE_PARALLEL_CONTRACT.md](../SOURCE_PARALLEL_CONTRACT.md).
Retention remains unchanged; compact delivery is a separate design proposal.
