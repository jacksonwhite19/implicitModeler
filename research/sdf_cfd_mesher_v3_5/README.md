# SDF CFD Mesher V3.5 — frozen working candidate

The isolated standalone connects V3 complete-stack prisms to the existing V4 direct outer workflow. No separate transition band is required. The frozen build is the accepted NODAL5 runtime; current consolidation adds no meshing implementation changes.

See [standalone/README.md](standalone/README.md) for actual current qualification status, [standalone/QUALIFICATION.md](standalone/QUALIFICATION.md) for the three exact commands and declared carrier modes, [standalone/INSTALL_FROZEN.md](standalone/INSTALL_FROZEN.md) for the configured-host requirements, and [standalone/CONSOLIDATE1_REPORT.md](standalone/CONSOLIDATE1_REPORT.md) for the result and limitations. Compact source identity and evidence are in qualification/consolidation_20261027. CURRENT_TASK.md records the active milestone.

This is a frozen numerical engineering candidate, not CFD, continuous-geometry or general-shape qualification. Passage and sphere are complete on this build; the same-build original thick-blend comparison is currently pending. No result is silently inherited from an earlier build. Earlier local artifacts are preserved; generated multi-GB runs are not committed.
