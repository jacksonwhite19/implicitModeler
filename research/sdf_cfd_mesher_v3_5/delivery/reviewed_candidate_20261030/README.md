# Mesher V3.5 reviewed experimental candidate - 2026-10-30

Generic SDF/Rhai surface -> V3 prism stack -> existing V4 direct background -> joined verification/export. There is no separate transition generator in this path. This is a reviewed experimental candidate, not universal CFD production qualification.

Reviewed local snapshot: `b87196be`; scientific code: `9a3a63d6`. Runtime: `97d081d262f87756109dca1ae54bdf409382922d734f32910bf9a3d93cd4196f` (1,906 files). The public commit is a scoped snapshot on the previously published candidate, avoiding unrelated development history. Package RELEASE.json records its exact publication commit; PACKAGE_MANIFEST.json records payload hashes and Git origins.

Start with [installation and dependency limits](INSTALL.md), [copyable CLI usage](QUICKSTART.md) and [qualification/version matrix](QUALIFICATION.md). [Host dependency provenance](HOST_DEPENDENCIES.json) and license notices remain included.

Only the close-spheres full CLI and actual independent audit were run on this runtime. Ellipsoid and torus full runs belong to the earlier runtime. No new scientific run was done for publication. Solver outputs, transient experiments and failed scratch are not packaged. Canonical reports contain original-run paths as provenance; these are not paths that must exist after extraction.
