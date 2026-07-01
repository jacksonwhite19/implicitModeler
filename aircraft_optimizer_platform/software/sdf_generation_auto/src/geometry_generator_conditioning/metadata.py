from __future__ import annotations

from typing import Any


CONDITIONED_CACHE_CONTRACT_VERSION = "conditioned_geometry_cache.v1"


def full_geometry_dirty_region(
    *,
    bbox_min_mm: list[float],
    bbox_max_mm: list[float],
    source: str = "full_geometry",
    operation_type: str = "geometry_generation",
    recommended_grid_spacing_mm: float | None = None,
    recommended_halo_mm: float = 0.0,
    feature_ids: list[str] | None = None,
    component_ids: list[str] | None = None,
    topology_change_expected: bool = False,
    notes: str | None = None,
) -> dict[str, Any]:
    return {
        "dirty_region_id": "dirty_full_geometry",
        "source": source,
        "operation_type": operation_type,
        "feature_ids": feature_ids or [],
        "component_ids": component_ids or [],
        "bbox_min_mm": bbox_min_mm,
        "bbox_max_mm": bbox_max_mm,
        "support_radius_mm": None,
        "blend_width_mm": None,
        "offset_or_shell_width_mm": None,
        "recommended_grid_spacing_mm": recommended_grid_spacing_mm,
        "recommended_halo_mm": recommended_halo_mm,
        "topology_change_expected": topology_change_expected,
        "notes": notes,
    }


def unavailable_conditioned_cache_metadata(
    *,
    fallback_mode: str = "exporter_direct_sampling",
) -> dict[str, Any]:
    return {
        "cache_contract_version": CONDITIONED_CACHE_CONTRACT_VERSION,
        "canonical_graph_role": "source_of_truth",
        "cache_role": "derived_query_acceleration",
        "cache_state": "unavailable",
        "update_mode": "not_run",
        "fallback_available": True,
        "fallback_mode": fallback_mode,
        "client_awareness_required": False,
    }
