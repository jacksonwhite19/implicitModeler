from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from aircraft_optimizer.records import MetricValue

MODULE_NAME = "cfd_readiness_gate"
MODULE_VERSION = "0.1.0"

DEFAULT_CFD_READINESS_POLICY = {
    "max_boundary_edges": 0,
    "max_nonmanifold_edges": 0,
    "required_connected_components": 1,
    "max_duplicate_triangles": 0,
    "max_long_chord_sections_ge_75mm": 0,
    "max_triangles": 3500000,
    "max_vertices": 2000000,
    "max_aspect_p99": 75.0,
    "max_high_aspect_count": 25000,
    "max_p99_edge_length_mm": 3.0,
    "max_edge_length_mm": 8.0,
}


@dataclass(frozen=True)
class CfdReadinessResult:
    passed: bool
    metrics: dict[str, dict[str, object]]
    metadata: dict[str, Any]
    warnings: list[str]


def evaluate_cfd_readiness(
    export_metrics: dict[str, Any],
    *,
    policy: dict[str, Any] | None = None,
) -> CfdReadinessResult:
    active_policy = {**DEFAULT_CFD_READINESS_POLICY, **(policy or {})}
    values = {key: _metric_number(export_metrics, key) for key in _EXPORT_KEYS}
    checks = {
        "max_boundary_edges": _lte(values["export.boundary_edges"], active_policy["max_boundary_edges"]),
        "max_nonmanifold_edges": _lte(values["export.nonmanifold_edges"], active_policy["max_nonmanifold_edges"]),
        "required_connected_components": values["export.connected_components"] == active_policy["required_connected_components"],
        "max_duplicate_triangles": _lte(values["export.duplicate_triangles"], active_policy["max_duplicate_triangles"]),
        "max_long_chord_sections_ge_75mm": _lte(values["export.long_chord_sections_ge_75mm"], active_policy["max_long_chord_sections_ge_75mm"]),
        "max_triangles": _lte(values["export.triangles"], active_policy["max_triangles"]),
        "max_vertices": _lte(values["export.vertices"], active_policy["max_vertices"]),
        "max_aspect_p99": _optional_lte(values["export.aspect_p99"], active_policy["max_aspect_p99"]),
        "max_high_aspect_count": _optional_lte(values["export.high_aspect_count"], active_policy["max_high_aspect_count"]),
        "max_p99_edge_length_mm": _optional_lte(values["export.p99_edge_length"], active_policy["max_p99_edge_length_mm"]),
        "max_edge_length_mm": _optional_lte(values["export.max_edge_length"], active_policy["max_edge_length_mm"]),
    }
    failed_checks = [name for name, passed in checks.items() if not passed]
    metrics = {
        "cfd_readiness.failed_check_count": _metric(len(failed_checks), None),
        "cfd_readiness.ready": _metric(0 if failed_checks else 1, None),
    }
    return CfdReadinessResult(
        passed=not failed_checks,
        metrics=metrics,
        metadata={
            "module_name": MODULE_NAME,
            "module_version": MODULE_VERSION,
            "policy": active_policy,
            "checks": checks,
            "failed_checks": failed_checks,
            "export_metric_values": values,
        },
        warnings=[f"failed CFD-readiness check: {check}" for check in failed_checks],
    )


_EXPORT_KEYS = (
    "export.boundary_edges",
    "export.nonmanifold_edges",
    "export.connected_components",
    "export.duplicate_triangles",
    "export.long_chord_sections_ge_75mm",
    "export.triangles",
    "export.vertices",
    "export.aspect_p99",
    "export.high_aspect_count",
    "export.p99_edge_length",
    "export.max_edge_length",
)


def _metric_number(metrics: dict[str, Any], key: str) -> float | int | None:
    value = metrics.get(key)
    if isinstance(value, MetricValue):
        return value.value if isinstance(value.value, (int, float)) else None
    if isinstance(value, dict):
        raw = value.get("value")
        return raw if isinstance(raw, (int, float)) else None
    return None


def _lte(value: float | int | None, limit: Any) -> bool:
    return value is not None and float(value) <= float(limit)


def _optional_lte(value: float | int | None, limit: Any) -> bool:
    return value is None or float(value) <= float(limit)


def _metric(value: float | int, unit: str | None) -> dict[str, object]:
    return MetricValue(
        value=round(value, 6) if isinstance(value, float) else value,
        unit=unit,
        confidence=1.0,
        source=MODULE_NAME,
    ).to_dict()

