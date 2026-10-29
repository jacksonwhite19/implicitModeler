# Mesher V3.5 consolidated compact candidate

V3 builds the complete prism stack. The existing V4 direct transition/background workflow fills the outer volume and performs joined acceptance and OpenFOAM export. There is no separate transition tool. This package carries the exact reviewed source and owned binary closure; it does not replace a frozen release.

Start with [installation](INSTALL.md), then [the quickstart](QUICKSTART.md). The practical command is explicit production verification, four source workers and compact retention. Add immediate audit and owned cleanup when you want verification before discarding temporary proof data. The current live compact/audit/cleanup evidence covers one fixed sphere on the configured host. See the [qualification matrix and engineering limits](QUALIFICATION.md).

| Control | Default | Explicit option |
|---|---|---|
| Verification | audit | production for complete direct workflow |
| Source workers | 1 (serial) | 2 or 4 for eligible production work; audit remains serial |
| Retention | full-evidence | compact-delivery |
| Scratch deletion | none | --cleanup-scratch in compact mode |
| Separate immediate audit | none added by this flag | --immediate-audit in compact production |

These controls are independent within their supported combinations. Keep serial for small jobs unless measurements justify workers. Production keeps all mandatory producer/export/closure checks and defers separate replay. An immediate or later audit adds runtime; it is not a free speed option. Full evidence is required for a future complete offline replay.

Compact output contains the complete recursive polyMesh, including zones and sets, geometry/settings/units, counts, quality warnings, checks performed and integrity/provenance records. Small external status and cleanup-authorization receipts bind the completed checks. Solver fields, boundary conditions and solver dictionaries are not supplied. CFD accuracy is not qualified.

Large evidence exists temporarily during a compact run. After successful producer checks, any requested audit and verified delivery, explicitly requested cleanup removes only the invocation's newly owned scratch. It never adopts or deletes historical input bundles. No permanent full proof archive or external acknowledgment is needed. Without --cleanup-scratch, staging remains. After cleanup, full offline replay is unavailable; reports are completed-check records, not replay proofs.

Failures preserve diagnostics. Scientific acceptance, audit, publication and cleanup have separate statuses; an accepted mesh can have a later storage failure. Read the retention sidecar and require the outcomes requested by your command. An interrupted delivery without COMPACT.json is incomplete. Do not delete failure folders or lower gates to force acceptance.

Runtime: commit 86a749af6aba1ed936d3e68213b473b985e6faa8, identity efb724a6a70273999f3e36e5873269984781de99b5eb7f306278f4001bf4d3ac, 1,895 files. RELEASE.json and PACKAGE_MANIFEST.json bind this local package. The source ZIP includes no raw full-run bundles, caches or secrets. [Qualification records](qualification/PERF22_REPORT.md) explain the measured result and limits.
