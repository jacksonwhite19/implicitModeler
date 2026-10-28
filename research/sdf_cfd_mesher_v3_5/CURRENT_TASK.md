# Mesher V3.5 verification candidate

This separate candidate connects V3 prisms to the existing V4 direct transition/background and joined acceptance/export. Start with [standalone instructions](standalone/README.md), [installation](standalone/INSTALL_VERIFICATION.md) and [tested commands/results](standalone/QUALIFICATION.md). No separate transition band is added.

The same reviewed runtime714b83dd/8e428 (1,873files) passed fixed sphere/adaptive, thick-blend/adaptive and passage/nodal audit/production/separate offline sequences. Audit is default; production explicitly defers separate replay and keeps all producer/export/closure gates. Retained warnings/repair/quality limits remain. See [compact evidence](qualification/verification_candidate_20261028/README.md) and the full accepted [report](qualification/passage_verification_closed_20261028/REPORT.md).

Packaging milestone dispatch2/5 only. No new mesh/scientific tests, runtime/gate changes, performance or robustness campaigns. Delivery is a local separate verification-candidate ZIP; no push, merge or frozen branch/package update. After independent package/docs review and callback, stop.
