from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from aircraft_optimizer.modules.virtual_components import (
    estimate_virtual_mass_properties_for_static_margin,
)
from aircraft_optimizer.records import CandidateSeed, MetricValue

MODULE_NAME = "post_cfd_layout_adjustment"
MODULE_VERSION = "0.1.0"


@dataclass(frozen=True)
class StabilityLayoutAdjustment:
    passed: bool
    metrics: dict[str, dict[str, object]]
    metadata: dict[str, Any]
    warnings: list[str]


DEFAULT_STABILITY_LAYOUT_POLICY = {
    "desired_static_margin": 0.16,
    "min_static_margin": 0.08,
    "max_static_margin": 0.28,
    "max_static_margin_error": 0.035,
    "max_cg_error_mm": 45.0,
}


def adjust_layout_after_neutral_point(
    candidate: CandidateSeed,
    *,
    neutral_point_x_mm: float,
    policy: dict[str, Any] | None = None,
    virtual_component_assumptions: dict[str, Any] | None = None,
) -> StabilityLayoutAdjustment:
    active_policy = {**DEFAULT_STABILITY_LAYOUT_POLICY, **(policy or {})}
    desired_static_margin = float(active_policy["desired_static_margin"])
    mass_properties = estimate_virtual_mass_properties_for_static_margin(
        candidate,
        neutral_point_x_mm=float(neutral_point_x_mm),
        desired_static_margin=desired_static_margin,
        assumptions=virtual_component_assumptions,
    )
    if mass_properties.static_margin is None:
        raise ValueError("post-CFD layout adjustment requires static margin result")
    static_margin_error = mass_properties.static_margin - desired_static_margin
    checks = {
        "layout_feasible": mass_properties.layout_feasible,
        "min_static_margin": mass_properties.static_margin >= float(active_policy["min_static_margin"]),
        "max_static_margin": mass_properties.static_margin <= float(active_policy["max_static_margin"]),
        "max_static_margin_error": abs(static_margin_error) <= float(active_policy["max_static_margin_error"]),
        "max_cg_error_mm": abs(mass_properties.cg_error_mm) <= float(active_policy["max_cg_error_mm"]),
    }
    failed_checks = [name for name, passed in checks.items() if not passed]
    metrics = {
        **mass_properties.metrics(),
        "stability.desired_static_margin": _metric(desired_static_margin, None),
        "stability.static_margin_error": _metric(static_margin_error, None),
        "stability.adjustment_failed_check_count": _metric(len(failed_checks), None),
    }
    metadata = {
        "module_name": MODULE_NAME,
        "module_version": MODULE_VERSION,
        "source": "post_cfd_or_stability_neutral_point",
        "policy": active_policy,
        "checks": checks,
        "failed_checks": failed_checks,
        "layout": mass_properties.metadata()["layout"]
        | {
            "static_margin_available": True,
            "static_margin_source": "provided_neutral_point_x_mm",
            "edf_fixed": True,
        },
        "components": mass_properties.metadata()["components"],
    }
    return StabilityLayoutAdjustment(
        passed=not failed_checks,
        metrics=metrics,
        metadata=metadata,
        warnings=[f"failed post-CFD layout check: {check}" for check in failed_checks],
    )


def _metric(value: float | int, unit: str | None) -> dict[str, object]:
    return MetricValue(
        value=round(value, 6) if isinstance(value, float) else value,
        unit=unit,
        confidence=0.45,
        source=MODULE_NAME,
    ).to_dict()

