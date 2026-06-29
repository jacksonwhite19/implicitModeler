# Optimizer Contracts

These contracts define the thin platform interface before implementation.

The exporter is only one downstream module. It should conform to these contracts rather than define them.

Contract files:

- `platform_interface_overview.md`
- `candidate_contract.md`
- `geometry_provider_contract.md`
- `evaluation_contract.md`
- `artifact_contract.md`
- `module_adapter_contract.md`
- `database_schema_contract.md`
- `event_log_contract.md`
- `failure_contract.md`
- `pre_export_gate_catalog.md`
- `sequential_gated_pipeline_contract.md`
- `post_cfd_stability_layout_contract.md`
- `mesh_export_contract.md`
- `mesh_comparison_acceptance_contract.md`
- `snappy_mesher_evidence_contract.md`

Use these files to keep optimizer, geometry generation, export, analysis, scoring, logging, dashboard, and future MDAO modules decoupled.
