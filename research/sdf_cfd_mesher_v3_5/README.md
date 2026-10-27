# Mesher V3.5 Phase 1A candidate

The connected path is Rhai/SDF surface -> V3 complete-stack prisms -> existing V4 direct outer workflow -> joined acceptance/export/readback. The V4 workflow supplies the transition and background. There is no separate transition band.

This candidate adds guarded artifact finalization and a worker-to-supervisor guard handoff. Audit is the sole behavior: all twelve producer checks and full cold semantic replay remain mandatory. No production bypass or verification selector was added.

Focused validation passed: 30 tests in the combined run, then three targeted worker-exit/clean-closure checks after the last fix (31 distinct tests overall). Three independent read-only reviews cover source and saved test evidence. These controls do not qualify a full mesh on this changed candidate.

The last full-mesh qualified reference is frozen commit eff955ed3670e67b7d134b83e69c37f52b387906, runtime identity 5e38fe034bb6f90fd23f5fee854293960e3ead35c289450ebe8d4f5a935f6666 (1,866 files). It passed passage with explicit nodal construction and sphere/thick blend with explicit adaptive construction. The original checkout and frozen worktree remain separate.

See [PHASE1A_CONTRACT.md](PHASE1A_CONTRACT.md) for the consumer map, failure rules, schemas and limits. Phase 1B implementation, replay omission, new full-fixture runs and performance campaigns are outside this milestone. Component cost and net savings are unmeasured. The next decision is a separately authorized Phase 1B implementation review; this candidate is not promoted for production use.
