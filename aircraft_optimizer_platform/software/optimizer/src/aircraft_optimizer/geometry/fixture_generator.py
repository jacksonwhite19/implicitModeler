from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from aircraft_optimizer.artifacts.registry import sha256_file
from aircraft_optimizer.jsonutil import dumps
from aircraft_optimizer.records import CandidateSeed
from aircraft_optimizer.schemas import VariableSchema
from geometry_generator_conditioning.metadata import (
    full_geometry_dirty_region,
    unavailable_conditioned_cache_metadata,
)


@dataclass(frozen=True)
class FixtureGeneratedGeometry:
    script_path: Path
    trace_path: Path
    feature_name: str
    coordinate_frame: str
    bbox_min_mm: list[float]
    bbox_max_mm: list[float]
    source_hash: str
    parameter_trace: dict[str, object]
    metadata: dict[str, object]


def generate_fixed_wing_fixture_geometry(
    *,
    platform_root: Path,
    workspace: Path,
    candidate_id: str,
    variable_schema_id: str,
    variable_schema: VariableSchema,
    candidate: CandidateSeed,
) -> FixtureGeneratedGeometry:
    variable_schema.validate_candidate(candidate)
    template_path = (
        platform_root
        / "software"
        / "sdf_generation_manual"
        / "curated_rhai"
        / "direct_sparse_oml_aircraft_no_inlet.rhai"
    )
    output_dir = workspace / "generated_geometry" / candidate_id
    output_dir.mkdir(parents=True, exist_ok=True)

    feature_name = "aircraft_oml_native_mc"
    script_path = output_dir / "fixed_wing_fixture_geometry.rhai"
    trace_path = output_dir / "fixed_wing_fixture_geometry.trace.json"

    parameter_trace: dict[str, object] = {
        "aircraft_family": candidate.aircraft_family,
        "family_schema_id": variable_schema.schema_id,
        "family_schema_version": variable_schema.schema_version,
        "variable_schema_id": variable_schema_id,
        "design_variables": candidate.design_variables_dict(),
        "normalized_design_vector": candidate.normalized_design_vector,
        "derived_parameters": {
            "template_source": str(template_path),
            "feature_name": feature_name,
        },
        "defaults_applied": {},
        "constraints_checked": {
            "schema_bounds": True,
            "normalized_policy": True,
            "topology_policy": variable_schema.raw.get("topology_policy"),
        },
    }

    template_text = template_path.read_text(encoding="utf-8")
    header = "\n".join(
        [
            "// Generated fixture geometry for aircraft_optimizer_platform.",
            "// This file is a traceable no-execution fixture and does not prove CAD/export validity.",
            f"// candidate_id: {candidate_id}",
            f"// schema_id: {variable_schema.schema_id}",
            "",
        ]
    )
    script_path.write_text(header + template_text, encoding="utf-8")
    trace_path.write_text(dumps(parameter_trace), encoding="utf-8")
    bbox_min_mm = [-128.0, -512.0, -128.0]
    bbox_max_mm = [1024.0, 512.0, 256.0]

    return FixtureGeneratedGeometry(
        script_path=script_path,
        trace_path=trace_path,
        feature_name=feature_name,
        coordinate_frame="native_aircraft_frame_x_length_y_span_z_vertical",
        bbox_min_mm=bbox_min_mm,
        bbox_max_mm=bbox_max_mm,
        source_hash=sha256_file(script_path),
        parameter_trace=parameter_trace,
        metadata={
            "generator_name": "fixed_wing_uav_fixture_generator",
            "generator_version": "0.1.0",
            "template_path": str(template_path),
            "template_hash": sha256_file(template_path),
            "trace_path": str(trace_path),
            "trace_hash": sha256_file(trace_path),
            "fixture_generation_only": True,
            "conditioned_geometry_cache": unavailable_conditioned_cache_metadata(),
            "dirty_regions": [
                full_geometry_dirty_region(
                    bbox_min_mm=bbox_min_mm,
                    bbox_max_mm=bbox_max_mm,
                    recommended_grid_spacing_mm=1.0,
                    recommended_halo_mm=3.0,
                    feature_ids=[feature_name],
                    notes=(
                        "Conservative full-candidate dirty region; fixture "
                        "generator does not run conditioning."
                    ),
                )
            ],
        },
    )
