# Software Layout

This folder is the intended home for platform-owned or vendored software components.

Most implementation code has not been moved here yet. Existing CAD/SDF and exporter code still lives in the parent project and `dual_contouring` folders. Use `master_platform_manifest.md` to track current source-of-truth locations and migration status.

Target components:

- `sdf_generation_manual`: curated manual aircraft definitions and examples.
- `sdf_generation_auto`: parameterized aircraft-definition generation from optimizer variables.
- `exporters`: STL and future mesh/export adapters.
- `optimizer`: optimizer core, run database, candidate lifecycle, scoring, and orchestration.
- `dashboard`: standalone native local dashboard.
- `analysis_modules`: low/medium/high-fidelity analysis adapters.

Do not drop scratch files here without triage. Imported files need provenance, validation status, and a clear owner.
