from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from aircraft_optimizer.records import FailureDraft, MetricValue


SUMMARY_METRICS: tuple[tuple[str, str, str | None], ...] = (
    ("boundary_edges", "export.boundary_edges", "count"),
    ("nonmanifold_edges", "export.nonmanifold_edges", "count"),
    ("connected_components", "export.connected_components", "count"),
    ("duplicate_triangles", "export.duplicate_triangles", "count"),
    ("long_chord_sections_ge_75mm", "export.long_chord_sections_ge_75mm", "count"),
    ("vertex_count", "export.vertices", "count"),
    ("triangle_count", "export.triangles", "count"),
    ("runtime_total_s", "export.runtime_seconds", "s"),
    ("sdf_sample_count", "export.sdf_sample_count", "count"),
    ("active_tile_count", "export.active_tile_count", "count"),
    ("aspect_p99", "export.aspect_p99", None),
    ("high_aspect_count", "export.high_aspect_count", "count"),
    ("max_edge_length", "export.max_edge_length", "mm"),
    ("p99_edge_length", "export.p99_edge_length", "mm"),
)

GATE_FAILURE_MAP: tuple[tuple[str, str], ...] = (
    ("boundary_edges", "meshing.boundary_edges_exceeded"),
    ("nonmanifold_edges", "meshing.nonmanifold_edges_exceeded"),
    ("connected_components", "meshing.connected_components_exceeded"),
    ("duplicate_triangles", "meshing.duplicate_triangles_exceeded"),
    ("long_chord_sections_ge_75mm", "meshing.long_chord_sections_exceeded"),
    ("runtime_total_s", "meshing.timeout"),
    ("high_aspect_count", "meshing.high_aspect_count_exceeded"),
    ("aspect_p99", "meshing.aspect_p99_exceeded"),
    ("p99_edge_length", "meshing.p99_edge_length_exceeded"),
    ("max_edge_length", "meshing.max_edge_length_exceeded"),
)


@dataclass(frozen=True)
class ExportArtifactRef:
    kind: str
    path: str

    def to_dict(self) -> dict[str, str]:
        return {"kind": self.kind, "path": self.path}


@dataclass(frozen=True)
class OmlStlExportParseResult:
    module_status: str
    passed: bool
    metrics: dict[str, MetricValue] = field(default_factory=dict)
    artifacts: list[ExportArtifactRef] = field(default_factory=list)
    failure: FailureDraft | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def metrics_dict(self) -> dict[str, dict[str, object]]:
        return {key: value.to_dict() for key, value in self.metrics.items()}

    def artifacts_dict(self) -> list[dict[str, str]]:
        return [artifact.to_dict() for artifact in self.artifacts]


def parse_oml_stl_export_result(path: Path) -> OmlStlExportParseResult:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("OML STL export result must be a JSON object")
    return parse_oml_stl_export_result_data(data)


def parse_oml_stl_export_result_data(data: dict[str, Any]) -> OmlStlExportParseResult:
    status = str(data.get("status", "unknown"))
    pass_gate = bool(data.get("pass_gate", False))
    summary = _dict(data.get("summary"))
    gate_failures = _strings(data.get("gate_failures"))

    metrics = _parse_metrics(summary)
    artifacts = _parse_artifacts(data, summary)
    failure = _parse_failure(status, pass_gate, gate_failures)

    return OmlStlExportParseResult(
        module_status=_module_status(status, pass_gate),
        passed=pass_gate and status == "passed",
        metrics=metrics,
        artifacts=artifacts,
        failure=failure,
        metadata={
            "source_status": status,
            "preset": data.get("preset"),
            "gate_failures": gate_failures,
            "command": data.get("command", []),
            "preset_definition": _dict(data.get("preset_definition")),
            "limits": _dict(data.get("limits")),
            "wrapper_runtime_s": data.get("wrapper_runtime_s"),
        },
    )


def _parse_metrics(summary: dict[str, Any]) -> dict[str, MetricValue]:
    metrics: dict[str, MetricValue] = {}
    for source_key, metric_key, unit in SUMMARY_METRICS:
        if source_key not in summary:
            continue
        metrics[metric_key] = MetricValue(
            value=summary[source_key],
            unit=unit,
            confidence=1.0,
            source="oml_stl_export",
        )
    return metrics


def _parse_artifacts(data: dict[str, Any], summary: dict[str, Any]) -> list[ExportArtifactRef]:
    artifacts: list[ExportArtifactRef] = []
    _append_artifact(artifacts, "export_result_json", data.get("result_json"))
    _append_artifact(artifacts, "oml_stl", summary.get("stl_path"))
    _append_artifact(artifacts, "stdout_log", data.get("stdout_log"))
    _append_artifact(artifacts, "stderr_log", data.get("stderr_log"))
    _append_artifact(artifacts, "section_audit_csv", summary.get("section_chord_audit_csv"))
    return artifacts


def _append_artifact(artifacts: list[ExportArtifactRef], kind: str, value: Any) -> None:
    if isinstance(value, str) and value:
        artifacts.append(ExportArtifactRef(kind=kind, path=value))


def _parse_failure(
    status: str,
    pass_gate: bool,
    gate_failures: list[str],
) -> FailureDraft | None:
    if status == "passed" and pass_gate:
        return None
    if status == "dry_run" and pass_gate:
        return None

    category, stage, retryable = _failure_classification(status, gate_failures)
    message = "; ".join(gate_failures) if gate_failures else f"OML STL export status: {status}"
    return FailureDraft(
        category=category,
        stage=stage,
        severity="recoverable" if retryable else "fatal",
        retryable=retryable,
        message=message,
        suggested_next_action=_suggested_next_action(status),
        metadata={
            "source_status": status,
            "failure_codes": [_failure_code(text) for text in gate_failures],
        },
    )


def _failure_classification(
    status: str,
    gate_failures: list[str],
) -> tuple[str, str, bool]:
    if status == "export_failed":
        return ("runtime.export_failed", "export", True)
    if status == "summary_missing":
        return ("artifact.missing_expected_artifact", "registration", True)
    if status == "invalid_preset":
        return ("runtime.environment_mismatch", "export_input", False)
    if status == "failed_quality_gate" or gate_failures:
        first_code = _failure_code(gate_failures[0]) if gate_failures else "meshing.quality_gate_failed"
        return (first_code, "quality_gate", False)
    return ("runtime.unknown_export_status", "export", True)


def _failure_code(text: str) -> str:
    for key, code in GATE_FAILURE_MAP:
        if text.startswith(key) or key in text:
            return code
    if "missing required file" in text:
        return "runtime.external_tool_missing"
    if "returned" in text:
        return "runtime.export_failed"
    return "meshing.quality_gate_failed"


def _module_status(status: str, pass_gate: bool) -> str:
    if status == "passed" and pass_gate:
        return "success"
    if status == "dry_run":
        return "skipped"
    if status in {"failed_quality_gate", "export_failed", "summary_missing", "invalid_preset"}:
        return "failed"
    return "failed" if not pass_gate else "skipped"


def _suggested_next_action(status: str) -> str | None:
    if status == "failed_quality_gate":
        return "inspect exporter result JSON and mesh quality artifacts"
    if status == "export_failed":
        return "inspect stdout and stderr logs"
    if status == "summary_missing":
        return "check exporter summary path and preset matching"
    if status == "invalid_preset":
        return "verify exporter paths, preset settings, and sidecar executable"
    return None


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _strings(value: Any) -> list[str]:
    return [str(item) for item in value] if isinstance(value, list) else []
