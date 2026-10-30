# PERF38 component contract investigation

The close-spheres failure is an overly restrictive surface-admission check. Two disjoint, nonnested obstacle bodies inside one connected farfield fit the direct-path mathematical boundary model. This is a design conclusion from source inspection and bounded component evidence, not a completed mesh qualification. The original four remaining global obligations were not replayed.

An isolated replacement contract now passes the saved local data and the synthetic controls. Independent review finds its conditional exterior-body contract sound and identifies no mathematical downstream blocker for this scope. It is ready for a narrow integration task, not a drop-in replacement or a qualified multi-body product. It remains outside production. No full-path retry ran in this dispatch. PERF37 remains two successes and one preserved failure.

## Actual call and inherited assumptions

`standalone/direct_connection.py:60` returns `finite_support.collar` as the construction helper. `finite_research/direct_evidence_modules/joined_audit.py:76` checks `wall`, `prefix_front`, then `farfield`, after compacting each population. In the saved NPZ, `cap` is that retained prefix front. `finite_support/collar.py:42-62` checks triangle validity/duplicates, closed consistently oriented edges, global reachability from vertex 0 and vertex links. The reachability failure at line 59 prevents the later vertex-link check.

The same finite-collar module also has `propose`, which requires a strict common mean kernel and produces a uniform dilation. Its single connected surface requirement is appropriate to that proposal. V3.5 never calls this proposal. The direct path receives the complete V3 prism cap and generates the outer tetrahedral background. No separate transition was generated or added here.

## Downstream assumption map

All paths below are relative to `standalone/` unless a copied V4 tools path is stated.

| Stage | Actual source | Component/region implication |
|---|---|---|
| Native import | `v4/tools/finite_research/direct_recipe155.py:28-46,54-69` | Preserves every cap/farfield triangle and cycle; imports the entire cap as surface 1, farfield as surface 2, and one volume with boundary `[2,-1]`. No cap-component count is imposed here. Actual multi-body qualification still needs the complete CLI. |
| Ownership and orientation | `v4/tools/finite_support/stored_wall_contract.py:13-85`; `v4/tools/local_termination_topology.py:82-129` | Face keys and owner cycles account for all declared wall faces and all cells. These checks do not require the wall to have one component. Stored wall cycles point outward from the fluid domain. |
| Joined enclosure/collision | `v4/tools/finite_research/direct_evidence_modules/joined_audit.py:75-108` | Enclosure uses all wall/cap vertices in actual farfield. Physical boundary collision includes all walls and farfield; all-face collision uses every owner face, including cross-component relations. No component filter is introduced. |
| Filling/degree | `producer_filling.py:2-8`; `v4/tools/prism_degree.py:19-75`; `v4/tools/cavity_volume_audit.py:12-59` | Producer filling obtains the actual outward cell boundary and uses the existing exact integer `serial` degree evaluator. Independent replay keeps the original rational oracle. Both sum oriented triangle contributions without a connected-boundary assumption. All-cell positive maps, every interior witness and total oriented-volume balance remain required, conditional on complete collision and incidence proofs. |
| Repair | `v4/tools/finite_research/direct_shared_apex.py:41-58,75-95,134-186` | All prefix, wall, cap, farfield and prism vertices are protected. Candidate movement uses full local stars, exact and actual-neighbour QA, and preserved connectivity. General repair assurance remains unqualified; this investigation performs no repair. |
| Export | `export_sources/run_farfield_complete.py:271-293`; `export_mesh.py:37-100` | Emits every wall/farfield triangle and one physical fluid region, then checks exact cell/face transfer and native mesh acceptance. Separate solid bodies do not require separate wall patches for this model. There is no general multi-region ownership or cavity-tree contract. |

The connected-component test in historical `capsule_core_control_audit.audit` is not an active direct-path gate. The direct path imports only its boundary-skewness function. It must not be used as evidence of a downstream connected-wall requirement or a new connectivity guarantee.

Two nonnested exterior bodies are compatible with the intended one-fluid-region model. A disconnected wall is not itself a disconnected fluid region. The saved bulk adjacency observation supports that distinction for this snapshot. Nested or hollow source geometry can instead create fluid pockets and mixed nesting roles; those require explicit region/nesting semantics and remain outside this correction's supported scope.

## Retained-data evidence

All inputs are from the preserved PERF37 failure. The diagnostic reads the same pinned NPZ bytes, checks the recorded parent array hashes and compares the joined prefix points, wall, cap, prism connectivity and prism IDs with the saved parent. This is consistency of an unaccepted retained snapshot, not a new live authenticated handoff.

| Population | Components | Vertices per component | Faces per component | Euler per component | Vertex links |
|---|---:|---|---|---|---|
| Wall | 2 | 1,303; 1,307 | 2,602; 2,610 | 2; 2 | PASS |
| Retained prism cap | 2 | 1,303; 1,307 | 2,602; 2,610 | 2; 2 | PASS |
| Farfield | 1 | 1,838 | 3,672 | 2 | PASS |

The original helper rejects the combined wall and combined cap with `DisconnectedCollarSurface`. It accepts the farfield. The candidate calls that same unchanged helper on each component and retains all its edge, orientation and vertex-link checks. Wall-to-cap column lineage is bijective and maps components `[0,1]` to `[0,1]`. Exact signed component volumes have the expected roles: wall cycles point into the solids, cap and farfield cycles point outward. The two cap AABBs have a strict separator. The saved 56,611 bulk tetrahedra have one shared-face adjacency component. This last observation is discrete connectivity only.

The original joined checks passed preservation, domain, incidence, winding, interface QA, joined QA and physical QA. Surfaces failed. Boundary collision, enclosure, all-face collision and filling remain unestablished. Final whole-mesh export and immediate independent audit did not run. No local contract result changes these facts.

## Isolated candidate

The candidate is in `component_contract.py`, outside `standalone/`. It uses the original surface predicate per connected component. It adds explicit roles and ordinary-prism lineage rather than changing the original finite-collar helper or any copied V4 module.

The parent wall/cap/prism arrays and every prefix point must match exactly. Bottom-to-top prism edges must form unambiguous, acyclic, complete, bijective wall-to-cap columns. Mapped wall triangles must match the cap cycles with the required reversal. Every wall component maps to exactly one complete cap component; missing, extra, merged or split component populations are rejected. The component count is inherited from the actual wall, not fixed to two or chosen by fixture name.

Each component must pass the original closed orientable manifold/link checks and have nonzero exact signed volume. Walls require the stored outward-from-fluid orientation; cap and farfield require outward orientation. Farfield remains connected. No genus-zero condition is added.

Exact degree samples check that each wall lies inside its corresponding cap, the cap is outside its wall, both are inside farfield, and separate cap components do not form an unsupported nesting tree. Strictly separated AABBs suffice for a nonnesting pair; otherwise degree is checked in both directions. Representative samples are only classification evidence conditional on the original complete nonintersection and positive-cell proofs. They do not prove embedding, separation, filling or continuous SDF containment on their own. Every original global proof remains mandatory.

The proposal targets exterior obstacle bodies. Nested material/cavity trees, isolated fluid pockets, mixed wall orientations and multiple fluid regions need explicit topology/region design. They are not admitted by treating each surface as independently valid. Open, pinched, nonmanifold or wrongly oriented boundaries remain invalid.

## Focused tests

The final suite has 18 passing controls: two positive local contracts and 16 expected rejections.

- Positive: two exterior bodies; one-body regression.
- Boundary failures: open components; duplicated inherited face population; original local duplicate-face rejection; a single reversed face; three uses of an edge; pinched vertex link; wholly inverted wall/cap roles; inverted farfield; disconnected farfield.
- Inherited correspondence failures: wrong cap cycle; missing inherited cap component; unintended extra wall; changed inherited point; cap geometry assigned to the wrong body.
- Containment failures: nested body tree; components outside farfield.

The saved close-spheres local contract also passes. It uses no new source query, mesh generation, conditioning, repair or global-proof replay. The baseline combined-surface failures are retained in the same receipt.

A separate arithmetic assertion inside the same bounded controls worker uses the original rational degree function on farfield plus two inward wall components. Four witnesses give degrees `[1,0,0,0]`: exterior fluid, the two solid interiors and outside farfield. The raw original function's `passed` flag is false because that API requires every witness to have degree one; this control intentionally expects three zero degrees. Its own expected-value assertion passes. This checks additive boundary semantics, not a complete filling or collision proof. The production integer evaluator was inspected in this dispatch; it was not executed by this small arithmetic control. Its earlier qualification is not replaced or expanded here.

Two initial test-harness failures remain visible: loading the Linux-packaged psutil before the Windows host package caused an import error; a negative control expected the later duplicate-face exception but the earlier inherited-population guard correctly rejected it. The harness import and expected exception were corrected. These are small synthetic test iterations, not retries of the scientific input. All child execution time is charged in `results/BUDGET.json`.

The final ledger has 11 closed child runs, including inspection, all synthetic iterations and retained-array diagnostics. Their aggregate measured execution time is 12.3494774005376 seconds out of 600 seconds. Execution receipts/logs total 36,092 bytes at execution close, before later report/review files, against a 536,870,912-byte output bound. No execution limit fired. Source inspection and independent review are separate. No further component execution is planned in this dispatch.

## Integration boundary and next validation

Do not replace `collar.surface` globally or relax `collar.propose`. A future integration should expose an explicit standalone direct-path surface interface with authenticated parent input and bound joined geometry. It must use the role-aware composite contract and preserve the existing joined receipt identity, custody, all twelve producer obligations, actual independent audit and successful-only publication. A call-order-based monkey patch is not sufficient.

The separate audit has the same issue: `standalone/direct_readback.py:20,53` supplies `_construction()` to copied `finite_research/direct_readback.py`, whose `semantics` calls the original joined auditor again at line 114. Both producer and independent replay must use the same reviewed local contract on their independently bound arrays. The reader must still reconstruct ownership, verify parent hashes and inherited digest, and rerun the original remaining obligations. Trusting a new producer receipt alone is insufficient. Prefer a standalone role-bound construction interface with exact input binding; leave copied V4 defaults and the original finite-collar machinery intact. The production policy/contract identity and its regression tests must explicitly account for the changed admission scope.

The candidate's parent equality checks are not authentication. The live parent lease and original stored-wall/ownership/preservation checks must remain the authority. The candidate does not certify the continuous Rhai/SDF fluid, generic repair correctness, portability or arbitrary multi-region export.

Before promotion, integration tests should include the unchanged single-body genus-one case and a valid nonnested pair with overlapping AABBs, to exercise the exact-degree relation branch. The retained pair exercises the strictly separated AABB branch. These are integration regression requirements, not new full-run authorization in this dispatch. Existing single-body behavior and independent replay must remain consistent.

If independent review confirms this narrow correction, the next full-path validation should be exactly one fresh run of the unchanged PERF37 close-spheres input and policy, after reviewed integration, with all producer and independent audit gates enabled. Keep the existing failure scratch. Stop on the first scientific failure. This dispatch does not execute or pre-authorize that retry.

The accepted production source and runtime are unchanged. No copied V4 module or original helper was edited. No remeshing, tuning, gate skipping, source promotion, failure cleanup, package, push, merge or automation occurred.
