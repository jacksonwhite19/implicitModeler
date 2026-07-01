from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from aircraft_optimizer.artifacts.registry import sha256_file
from geometry_generator_conditioning.metadata import (
    full_geometry_dirty_region,
    unavailable_conditioned_cache_metadata,
)


@dataclass(frozen=True)
class ManualReferenceGeometry:
    script_path: Path
    feature_name: str
    coordinate_frame: str
    bbox_min_mm: list[float]
    bbox_max_mm: list[float]
    source_hash: str
    metadata: dict[str, object]


def default_no_inlet_reference(platform_root: Path) -> ManualReferenceGeometry:
    script_path = (
        platform_root
        / "software"
        / "sdf_generation_manual"
        / "curated_rhai"
        / "direct_sparse_oml_aircraft_no_inlet.rhai"
    )
    bbox_min_mm = [-128.0, -512.0, -128.0]
    bbox_max_mm = [1024.0, 512.0, 256.0]
    feature_name = "aircraft_oml_native_mc"
    return ManualReferenceGeometry(
        script_path=script_path,
        feature_name=feature_name,
        coordinate_frame="native_aircraft_frame_x_length_y_span_z_vertical",
        bbox_min_mm=bbox_min_mm,
        bbox_max_mm=bbox_max_mm,
        source_hash=sha256_file(script_path),
        metadata={
            "source_label": "curated_direct_sparse_no_inlet_oml",
            "role": "outer_mold_line",
            "provider_note": "Manual reference provider does not run CAD/SDF code.",
            "conditioned_geometry_cache": unavailable_conditioned_cache_metadata(),
            "dirty_regions": [
                full_geometry_dirty_region(
                    bbox_min_mm=bbox_min_mm,
                    bbox_max_mm=bbox_max_mm,
                    recommended_grid_spacing_mm=1.0,
                    recommended_halo_mm=3.0,
                    feature_ids=[feature_name],
                    notes=(
                        "Manual reference path exposes cache planning metadata "
                        "but does not run conditioning."
                    ),
                )
            ],
        },
    )
