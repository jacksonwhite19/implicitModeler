from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from aircraft_optimizer.artifacts.registry import sha256_file
from aircraft_optimizer.jsonutil import dumps
from aircraft_optimizer.records import CandidateSeed
from aircraft_optimizer.schemas import VariableSchema

FEATURE_NAME = "aircraft_oml_native_mc"
BASE_FUSELAGE_LENGTH_MM = 700.0
BASE_NOSE_BLUNTNESS = 1.80
BASE_TAIL_BLUNTNESS = 0.97
BASE_TAIL_LE_X_MM = 500.0
TAIL_END_CLEARANCE_MM = 25.0
HTAIL_AFT_OFFSET_MM = 98.0
VTAIL_AFT_OFFSET_MM = 160.0

AIRFOIL_SELECTOR_OPTIONS = {
    0: {"name": "thin_loiter", "naca": "2409"},
    1: {"name": "balanced_uav", "naca": "2412"},
    2: {"name": "high_lift_stable", "naca": "4412"},
}


@dataclass(frozen=True)
class RealNoInletGeneratedGeometry:
    script_path: Path
    trace_path: Path
    feature_name: str
    coordinate_frame: str
    bbox_min_mm: list[float]
    bbox_max_mm: list[float]
    source_hash: str
    parameter_trace: dict[str, object]
    metadata: dict[str, object]


def generate_real_no_inlet_candidate_geometry(
    *,
    platform_root: Path,
    workspace: Path,
    candidate_id: str,
    variable_schema_id: str,
    variable_schema: VariableSchema,
    candidate: CandidateSeed,
) -> RealNoInletGeneratedGeometry:
    variable_schema.validate_candidate(candidate)
    template_path = (
        platform_root
        / "software"
        / "sdf_generation_manual"
        / "curated_rhai"
        / "direct_sparse_oml_aircraft_no_inlet.rhai"
    )
    output_dir = workspace / "generated_real_geometry" / candidate_id
    output_dir.mkdir(parents=True, exist_ok=True)
    script_path = output_dir / "real_no_inlet_candidate_geometry.rhai"
    trace_path = output_dir / "real_no_inlet_candidate_geometry.trace.json"

    design = _script_design_values(candidate)
    template_text = template_path.read_text(encoding="utf-8")
    generated_text = _apply_design_values(template_text, design)
    header = "\n".join(
        [
            "// Generated real-export no-inlet geometry for aircraft_optimizer_platform.",
            "// Generated from fixed-topology candidate design variables.",
            f"// candidate_id: {candidate_id}",
            f"// schema_id: {variable_schema.schema_id}",
            f"// feature_name: {FEATURE_NAME}",
            "",
        ]
    )
    script_path.write_text(header + generated_text, encoding="utf-8")

    parameter_trace: dict[str, object] = {
        "aircraft_family": candidate.aircraft_family,
        "family_schema_id": variable_schema.schema_id,
        "family_schema_version": variable_schema.schema_version,
        "variable_schema_id": variable_schema_id,
        "design_variables": candidate.design_variables_dict(),
        "normalized_design_vector": candidate.normalized_design_vector,
        "derived_parameters": {
            "template_source": str(template_path),
            "feature_name": FEATURE_NAME,
            "script_design_values": design,
            "airfoil_selector_options": AIRFOIL_SELECTOR_OPTIONS,
            "shape_mapping": {
                "fuselage_length_mm": "700.0 + fuselage.length_delta_mm",
                "nose_bluntness": "0.9 + 1.8 * fuselage.nose_bluntness; missing value keeps 1.80",
                "tail_bluntness": "0.35 + 1.24 * fuselage.tail_bluntness; missing value keeps 0.97",
            },
        },
        "defaults_applied": design["defaults_applied"],
        "constraints_checked": {
            "schema_bounds": True,
            "normalized_policy": True,
            "topology_policy": variable_schema.raw.get("topology_policy"),
        },
    }
    trace_path.write_text(dumps(parameter_trace), encoding="utf-8")

    return RealNoInletGeneratedGeometry(
        script_path=script_path,
        trace_path=trace_path,
        feature_name=FEATURE_NAME,
        coordinate_frame="native_aircraft_frame_x_length_y_span_z_vertical",
        bbox_min_mm=[-128.0, -512.0, -128.0],
        bbox_max_mm=[1024.0, 512.0, 256.0],
        source_hash=sha256_file(script_path),
        parameter_trace=parameter_trace,
        metadata={
            "generator_name": "real_no_inlet_candidate_geometry_generator",
            "generator_version": "0.1.0",
            "template_path": str(template_path),
            "template_hash": sha256_file(template_path),
            "trace_path": str(trace_path),
            "trace_hash": sha256_file(trace_path),
            "real_export_geometry": True,
            "fixed_topology": True,
        },
    )


def _script_design_values(candidate: CandidateSeed) -> dict[str, object]:
    airfoil_index = int(_clamp(round(_variable(candidate, "wing.airfoil_selector", 1.0)), 0, 2))
    nose_raw = _optional_variable(candidate, "fuselage.nose_bluntness")
    tail_raw = _optional_variable(candidate, "fuselage.tail_bluntness")
    length_delta_mm = _variable(candidate, "fuselage.length_delta_mm", 0.0)
    defaults_applied: dict[str, object] = {}
    if "wing.airfoil_selector" not in candidate.design_variables:
        defaults_applied["wing.airfoil_selector"] = 1.0
    if "fuselage.length_delta_mm" not in candidate.design_variables:
        defaults_applied["fuselage.length_delta_mm"] = 0.0
    if nose_raw is None:
        defaults_applied["fuselage.nose_bluntness"] = "template_default"
    if tail_raw is None:
        defaults_applied["fuselage.tail_bluntness"] = "template_default"

    fuselage_length = round(BASE_FUSELAGE_LENGTH_MM + length_delta_mm, 6)
    tail_le_x = _tail_le_x_for_fuselage(fuselage_length)

    return {
        "fuselage_length": fuselage_length,
        "nose_bluntness": (
            BASE_NOSE_BLUNTNESS
            if nose_raw is None
            else round(0.9 + 1.8 * _clamp(nose_raw, 0.0, 1.0), 6)
        ),
        "tail_bluntness": (
            BASE_TAIL_BLUNTNESS
            if tail_raw is None
            else round(0.35 + 1.24 * _clamp(tail_raw, 0.0, 1.0), 6)
        ),
        "wing_span": round(_variable(candidate, "wing.span_mm", 700.0), 6),
        "wing_root_chord": round(_variable(candidate, "wing.root_chord_mm", 170.0), 6),
        "wing_tip_chord": round(_variable(candidate, "wing.tip_chord_mm", 65.0), 6),
        "wing_sweep_deg": round(_variable(candidate, "wing.sweep_deg", 15.0), 6),
        "htail_le_x": tail_le_x,
        "vtail_le_x": tail_le_x,
        "tail_constraints": {
            "tail_le_x_mm": tail_le_x,
            "tail_aft_x_mm": round(tail_le_x + max(HTAIL_AFT_OFFSET_MM, VTAIL_AFT_OFFSET_MM), 6),
            "fuselage_end_x_mm": fuselage_length,
            "minimum_tail_end_clearance_mm": TAIL_END_CLEARANCE_MM,
            "clearance_mm": round(
                fuselage_length - (tail_le_x + max(HTAIL_AFT_OFFSET_MM, VTAIL_AFT_OFFSET_MM)),
                6,
            ),
        },
        "main_wing_airfoil": AIRFOIL_SELECTOR_OPTIONS[airfoil_index],
        "defaults_applied": defaults_applied,
    }


def _apply_design_values(template_text: str, design: dict[str, object]) -> str:
    text = template_text
    for name in [
        "fuselage_length",
        "nose_bluntness",
        "tail_bluntness",
        "wing_span",
        "wing_root_chord",
        "wing_tip_chord",
        "wing_sweep_deg",
        "htail_le_x",
        "vtail_le_x",
    ]:
        text = _replace_numeric_let(text, name, float(design[name]))
    airfoil = design["main_wing_airfoil"]
    if not isinstance(airfoil, dict):
        raise TypeError("main_wing_airfoil design entry must be a dict")
    text = re.sub(
        r'(wing_with_airfoil_export_safe\(\s*)"[^"]+"',
        rf'\1"{airfoil["naca"]}"',
        text,
        count=1,
    )
    return _select_native_oml_mc_expression(text)


def _select_native_oml_mc_expression(text: str) -> str:
    old = "// Diagnostic feature-specific SDF for local patch sampling.\naircraft_oml_oblique15_y_slice_frame\n"
    new = (
        "// Native marching-cubes OML SDF for optimizer export.\n"
        "// This intentionally avoids legacy y-slice/oblique diagnostic frames.\n"
        "body_outer\n"
    )
    if new in text:
        return text
    if old not in text:
        raise ValueError("expected final oblique diagnostic SDF expression in no-inlet template")
    return text.replace(old, new)


def _replace_numeric_let(text: str, name: str, value: float) -> str:
    pattern = rf"let\s+{re.escape(name)}\s*=\s*[-+]?\d+(?:\.\d+)?\s*;"
    replacement = f"let {name} = {_format_float(value)};"
    updated, count = re.subn(pattern, replacement, text, count=1)
    if count != 1:
        raise ValueError(f"expected exactly one top-level Rhai binding for {name}")
    return updated


def _variable(candidate: CandidateSeed, variable_name: str, default: float) -> float:
    variable = candidate.design_variables.get(variable_name)
    if variable is None:
        return default
    return float(variable.value)


def _optional_variable(candidate: CandidateSeed, variable_name: str) -> float | None:
    variable = candidate.design_variables.get(variable_name)
    if variable is None:
        return None
    return float(variable.value)


def _clamp(value: float | int, minimum: float | int, maximum: float | int) -> float:
    return float(max(minimum, min(maximum, value)))


def _tail_le_x_for_fuselage(fuselage_length_mm: float) -> float:
    max_tail_aft_offset = max(HTAIL_AFT_OFFSET_MM, VTAIL_AFT_OFFSET_MM)
    max_allowed_le_x = fuselage_length_mm - TAIL_END_CLEARANCE_MM - max_tail_aft_offset
    return round(min(BASE_TAIL_LE_X_MM, max_allowed_le_x), 6)


def _format_float(value: float) -> str:
    if float(value).is_integer():
        return f"{int(value)}.0"
    return f"{value:.6f}".rstrip("0").rstrip(".")
