from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

FEATURE_NAME = "aircraft_oml_native_mc"
DEFAULT_PRESET = "direct_sparse_oml_fast"


@dataclass(frozen=True)
class RealNoInletExportRequest:
    execution_enabled: bool
    preset: str
    feature_name: str
    geometry_source_path: Path
    exporter_script_path: Path
    exporter_working_directory: Path
    sdf_sidecar_path: Path
    result_json_path: Path
    quality_limits: dict[str, int]
    command: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "execution_enabled": self.execution_enabled,
            "preset": self.preset,
            "feature_name": self.feature_name,
            "geometry_source_path": str(self.geometry_source_path),
            "exporter_script_path": str(self.exporter_script_path),
            "exporter_working_directory": str(self.exporter_working_directory),
            "sdf_sidecar_path": str(self.sdf_sidecar_path),
            "result_json_path": str(self.result_json_path),
            "quality_limits": self.quality_limits,
            "command": self.command,
        }


def build_real_no_inlet_export_request(
    *,
    platform_root: Path,
    workspace: Path,
    execution_enabled: bool = False,
    preset: str = DEFAULT_PRESET,
    geometry_source_path: Path | None = None,
    result_dir_name: str | None = None,
    result_json_name: str = "no_inlet_oml_export_result.json",
    output_stem: str = "direct_sdf_oml",
    full_export: bool = False,
) -> RealNoInletExportRequest:
    platform_root = platform_root.resolve()
    workspace = workspace.resolve()
    repo_root = platform_root.parent
    exporter_root = repo_root / "dual_contouring" / "direct_sparse_sdf_mc_experiment"
    exporter_script = exporter_root / "scripts" / "optimizer_export_presets.py"
    geometry_source = geometry_source_path.resolve() if geometry_source_path else (
        platform_root
        / "software"
        / "sdf_generation_manual"
        / "curated_rhai"
        / "direct_sparse_oml_aircraft_no_inlet.rhai"
    )
    sidecar = repo_root / "dual_contouring" / "target" / "release" / "batch_sdf_distance_only.exe"
    result_dir = result_dir_name or (
        "real_no_inlet_export" if execution_enabled else "real_adapter_preflight"
    )
    result_json = workspace / result_dir / result_json_name
    quality_limits = {
        "max_boundary_edges": 0,
        "max_nonmanifold_edges": 0,
        "max_connected_components": 1,
        "max_duplicate_triangles": 0,
        "max_long_chord_sections": 0,
    }
    command = [
        "python",
        str(exporter_script),
        "--preset",
        preset,
        "--feature",
        FEATURE_NAME,
        "--sdf-script-source",
        str(geometry_source),
        "--sdf-sidecar",
        str(sidecar),
        "--sdf-workers",
        "8",
        "--output-stem",
        output_stem,
    ]
    if full_export:
        command += [
            "--full-export",
        ]
    else:
        command += [
            "--bbox-min=-64,0,-80",
            "--bbox-max=736,416,192",
            "--symmetry-half-mirror",
            "--symmetry-axis",
            "y",
            "--symmetry-plane-mm",
            "0",
        ]
    command += [
        "--result-json",
        str(result_json),
        "--max-boundary-edges",
        str(quality_limits["max_boundary_edges"]),
        "--max-nonmanifold-edges",
        str(quality_limits["max_nonmanifold_edges"]),
        "--max-connected-components",
        str(quality_limits["max_connected_components"]),
        "--max-duplicate-triangles",
        str(quality_limits["max_duplicate_triangles"]),
        "--max-long-chord-sections",
        str(quality_limits["max_long_chord_sections"]),
    ]
    if not execution_enabled:
        command.insert(4, "--dry-run")
    return RealNoInletExportRequest(
        execution_enabled=execution_enabled,
        preset=preset,
        feature_name=FEATURE_NAME,
        geometry_source_path=geometry_source,
        exporter_script_path=exporter_script,
        exporter_working_directory=repo_root / "dual_contouring",
        sdf_sidecar_path=sidecar,
        result_json_path=result_json,
        quality_limits=quality_limits,
        command=command,
    )


def preflight_real_no_inlet_export_boundary(
    *,
    platform_root: Path,
    workspace: Path,
) -> dict[str, Any]:
    request = build_real_no_inlet_export_request(
        platform_root=platform_root,
        workspace=workspace,
        execution_enabled=False,
    )
    required_paths = {
        "geometry_source_path": request.geometry_source_path,
        "exporter_script_path": request.exporter_script_path,
        "exporter_working_directory": request.exporter_working_directory,
        "sdf_sidecar_path": request.sdf_sidecar_path,
    }
    missing = [
        {"name": name, "path": str(path)}
        for name, path in required_paths.items()
        if not path.exists()
    ]
    tile_compatible = request.preset == DEFAULT_PRESET
    return {
        "available": not missing,
        "execution_enabled": request.execution_enabled,
        "execution_performed": False,
        "reason": "real adapter boundary preflight only; optimizer does not execute exporter by default",
        "missing_paths": missing,
        "tile_spacing_compatible": tile_compatible,
        "request": request.to_dict(),
    }
