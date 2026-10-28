# Mesher V3.5 Phase 1B candidate

Rhai/SDF surface -> V3 complete-stack prisms -> existing V4 transition/background -> joined acceptance and native export. There is no separate transition band.

The isolated candidate implements explicit `--verification audit|production`, with audit as the default. Production supports complete direct workflow only. Carrier choice remains independent. Construction, all twelve producer joined checks, quality/source/collision/domain/filling predicates, native fidelity and source/resource closure remain mandatory. Production omits only the separate full saved-artifact semantic replay and records it as `not_run`.

Production uses a distinct composed profile, result, artifact manifest and final verification schema. Root/direct production results omit legacy `passed` and `full_mesh_complete` fields. Final acceptance exists only in supervisor-published VERIFICATION after closure. Offline audit requires a trusted expected verification root and explicit verifier runtime identity, retains original-run authority scope and writes only into a fresh separate output.

This is reviewed implementation with focused controls, not full-mesh production/audit equivalence or successful complete offline replay qualification. See [PHASE1B_CONTRACT.md](PHASE1B_CONTRACT.md) for schemas, consumers, commands, tests and the exact proposed next qualification sequence. No full mesh campaign or performance measurement ran in this milestone.

Last full-mesh qualified reference: frozen commit eff955ed3670e67b7d134b83e69c37f52b387906, runtime identity 5e38fe034bb6f90fd23f5fee854293960e3ead35c289450ebe8d4f5a935f6666 (1,866 files). Passage explicit nodal:546,464 cells/207,705 points; sphere explicit adaptive:53,228/14,798; thick blend explicit adaptive:119,441/38,868. These are reference results, not current-candidate qualification. The original checkout and frozen package remain separate and unchanged by this milestone. No push or replacement of the frozen package is authorized.
