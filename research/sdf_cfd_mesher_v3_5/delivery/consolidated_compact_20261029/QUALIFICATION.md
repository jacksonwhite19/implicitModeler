# Qualification and observed performance

| Evidence | Runtime identity / files | Supported result |
|---|---|---|
| Historical published fixed-case baseline | 8e428c2a... / earlier runtime | Accepted sphere/blend/passage comparison reference; preserved, not replaced |
| Optimized serial PERF10/11 | 3bdd14455b74a67e6fa1405efc945956f69bcb03925283ee1d6481b9815b82a8 / 1,885 | All three fixed cases, exact mesh/source comparisons and separate full offline audits |
| Explicit four workers PERF16/17/18 | 6d0840f9b58a1fe8a67554f985efb878ad5899ae0e8c7aa9c4f4a2890d329b0e / 1,889 | All three fixed cases, exact meshes and separate full offline audits; serial stays default |
| Saved-bundle compact exports PERF19 | 1680be25183adaf72e916b1084aa33303dbe30cf440f0dd7ab75e54b67ed5e98 / 1,894 | Sphere/blend/passage solver bytes exact; no new meshes or live cleanup qualification |
| Current live compact PERF22 | efb724a6a70273999f3e36e5873269984781de99b5eb7f306278f4001bf4d3ac / 1,895 | ONE sphere normal production/four-worker/immediate-audit/delivery/owned-cleanup pass; 42 focused tests |
| This extracted package | Same efb724a6... / 1,895 | CRC/hash/blob/extraction/help/verifier-identity checks only; no new mesh campaign |

Current source commit is 86a749af6aba1ed936d3e68213b473b985e6faa8. Meshing/scientific code remains unchanged from its predecessor; storage/lifecycle and path monitoring have their own evidence. This does not upgrade all earlier three-case results to current-runtime full qualification. No current live compact blend/passage, arbitrary-geometry, CFD-accuracy, clean-machine or release qualification is claimed.

Current sphere: 14,798 points, 12,780 prisms and 40,448 tetrahedra; seven recursive solver files including zones/sets are byte-identical to the authenticated accepted sphere. All twelve producer obligations, native MeshOK/export, finalizer/source/resource closure and all twelve immediate audit replays passed before cleanup. Four-worker records bind the exact current CASE/runtime and 6,400 source segments. Controller time was 537.256274 s including audit and storage. Offline audit adds runtime. No post-cleanup full physical-array comparison or replay of deleted proof files is claimed.

| Storage observation | Logical bytes |
|---|---:|
| Current sphere scratch before cleanup (397 files) | 317,584,127 |
| Current sphere compact delivery (14 files) | 5,742,092 |
| Current sphere scratch after actual cleanup | 0; exact owned root absent |
| Current small authorization/status receipts | 13,880 / 3,189 |
| Historical passage full bundle (preserved) | 3,843,729,069 |
| Historical passage compact export | 71,724,797 |

The passage 71.7 MB example is export-only evidence, not current live cleanup. Full staging is still needed. Current sampled Windows RSS peaked at 1,015,422,976 B; maximum sampled logical controller scope was 325,817,242 B. Logical removal is not physical reclaimed-space measurement or reduced peak-RAM evidence. Native/Windows closure and post-cleanup census passed; original 375-file baseline and prior 54-file failed sphere scratch remain exact.

Observed passage production times across separate candidates were 6,124.9154 s (102.1 min) -> 4,637.0778 s (77.3 min, optimized serial) -> 3,803.3566 s (63.4 min, explicit four workers). These are different single-run configurations, not a current live-compact passage benchmark, stable causal promise or guarantee. In the three-case parallel matrix, sphere was 2.105% slower, blend 2.808% shorter and passage 17.979% shorter than their serial observations. The parallel benefit depends on workload; use serial for small jobs unless measured otherwise.

Sphere has zero compression/changed plan vertices and no MeshOK warning. Blend retains 1,663 compressed vertices, local growth reversals, two committed edits/four inserted points and a 41-face severe-nonorthogonality warning (maximum 77.266 degrees). Passage retains two small apex moves, 181,034 protected coordinates unchanged, zero compression and no MeshOK warning. Mandatory minima pass while some interface/cap percentage targets remain nonblocking misses. Requested 200 um surface size and growth 1.3 are not guaranteed everywhere.

Repair/shared-apex assurance remains UNQUALIFIED; untouched cells carry source-float evidence, not a global exact-Q population or continuous whole-source/SDF-fluid proof. Native checkMesh MeshOK does not supply CFD accuracy or y-plus evidence. The current extra approved-dictionary quality check was NOT_RUN_NO_APPLICABLE_APPROVED_DICTIONARY. Keep these limits and warnings with solver output.

The path monitor fix addresses a reproduced Windows final-name race during ordinary checkpoint churn. Captured results included extended-prefix, renamed and NTFS deleted names. PERF21's exact unlogged triggering path remains unknown. Leaf size sampling uses no-follow metadata, rejects observed reparses/persistent errors and checks directory identity without locks that block cleanup. It is observational accounting, not protection against arbitrary transient ancestor swaps; strict held publication/deletion checks remain separate. The 42 focused controls include file churn, concurrent directory cleanup, reparse refusal, persistent errors, quota checks and existing storage/failure/ownership controls.

Small reports and review receipts are in qualification. Runtime inventory and exact package Git/blob bindings are integrity evidence, not replay operands. Original full bundles, failed runs and prior/frozen packages remain preserved outside this ZIP. Development stops at this consolidation: no new campaign, optimization, runtime edits, push, merge or frozen replacement is part of delivery.
