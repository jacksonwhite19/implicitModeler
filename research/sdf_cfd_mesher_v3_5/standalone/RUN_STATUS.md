> Verification qualification (2026-10-28): fixed sphere/adaptive, thick-blend/adaptive and passage/nodal passed actual audit/production/separate offline on unchanged runtime714b83dd/8e428. Production audit remains not_run; separate replay passes. See [closed report](../qualification/passage_verification_closed_20261028/REPORT.md). Prior comparator failures are preserved. No automatic promotion or new campaign. Frozen eff955ed historical table below is unchanged.

# Mesher V3.5 frozen consolidation

Rhai/SDF surface -> V3 complete-stack prisms -> existing V4 direct outer workflow -> joined acceptance/export/readback. There is no separate transition band.

The exact 1,866-file runtime passed complete passage (explicit nodal), sphere and original thick blend (explicit adaptive). Sphere and blend final arrays match their earlier baselines byte-for-byte. See CONSOLIDATE1_REPORT.md and QUALIFICATION.md for counts, exact commands, timings, compression and limits.

Runtime identity: 5e38fe034bb6f90fd23f5fee854293960e3ead35c289450ebe8d4f5a935f6666. No runtime, gate or verification-policy change was made during consolidation. The small four-case robustness proposal is PREPARED, NOT RUN; no additional campaign is authorized. Performance and production/audit policy work are deferred.

Git excludes generated mesh outputs, caches and delivery archives. Compact qualification evidence is under qualification/consolidation_20261027. The source archive is delivered separately. Configured-host engineering qualification does not establish clean-machine portability, general-shape, continuous geometry or CFD accuracy.
