# OML STL Exporter

Purpose: optimizer-facing adapter for producing CFD-ready OML STL artifacts from geometry-provider output.

Current validated implementation:

```text
dual_contouring\direct_sparse_sdf_mc_experiment
```

Default optimizer preset:

- `direct_sparse_oml_fast`
- `spacing_mm = 1.0`
- `tile_size_mm = 16`
- `sdf_workers = 8`

High-fidelity selected-design preset:

- `direct_sparse_oml_high_fidelity`
- `spacing_mm = 0.75`
- `tile_size_mm = 24`
- `sdf_workers = 8`

Contract:

- `oml_export_contract.md`
- `software/exporters/oml_stl/export_adapter_contract.md`

Status: adapter target defined, implementation not imported or called yet.

The exporter is a downstream platform module. Optimizer code should talk to the module adapter contract and should not call exporter scripts directly.
