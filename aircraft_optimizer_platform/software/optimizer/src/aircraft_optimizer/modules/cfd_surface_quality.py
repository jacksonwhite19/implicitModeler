from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.records import MetricValue

MODULE_NAME = "cfd_surface_quality_gate"
MODULE_VERSION = "0.1.0"

DEFAULT_SURFACE_QUALITY_POLICY = {
    "max_boundary_edges": 0,
    "max_nonmanifold_edges": 0,
    "max_duplicate_triangles": 0,
    "max_degenerate_triangles": 0,
    "min_triangle_quality": 1e-12,
    "min_edge_length_mm": 0.001,
    "max_triangles": 3500000,
    "max_vertices": 2000000,
}


@dataclass(frozen=True)
class CfdSurfaceQualityResult:
    passed: bool
    surface_result: dict[str, Any]
    metrics: dict[str, dict[str, object]]
    metadata: dict[str, Any]
    warnings: list[str]


def evaluate_cfd_surface_quality(
    stl_path: Path,
    *,
    policy: dict[str, Any] | None = None,
    vertex_precision_digits: int = 6,
) -> CfdSurfaceQualityResult:
    active_policy = {**DEFAULT_SURFACE_QUALITY_POLICY, **(policy or {})}
    triangles = read_stl_triangles(stl_path)
    surface_result = build_surface_result(
        stl_path,
        triangles,
        vertex_precision_digits=vertex_precision_digits,
    )
    failed_checks = evaluate_surface_checks(surface_result, active_policy)
    metrics = {
        "cfd_surface.vertices": _metric(surface_result["counts"]["vertices"], None),
        "cfd_surface.triangles": _metric(surface_result["counts"]["triangles"], None),
        "cfd_surface.boundary_edges": _metric(
            surface_result["topology"]["boundary_edges"], None
        ),
        "cfd_surface.nonmanifold_edges": _metric(
            surface_result["topology"]["nonmanifold_edges"], None
        ),
        "cfd_surface.duplicate_triangles": _metric(
            surface_result["topology"]["duplicate_triangles"], None
        ),
        "cfd_surface.degenerate_triangles": _metric(
            surface_result["quality"]["degenerate_triangles"], None
        ),
        "cfd_surface.min_triangle_quality": _metric(
            surface_result["quality"]["min_triangle_quality"], None
        ),
        "cfd_surface.min_edge_length_mm": _metric(
            surface_result["quality"]["min_edge_length_mm"], "mm"
        ),
        "cfd_surface.ready": _metric(0 if failed_checks else 1, None),
        "cfd_surface.failed_check_count": _metric(len(failed_checks), None),
    }
    return CfdSurfaceQualityResult(
        passed=not failed_checks,
        surface_result=surface_result,
        metrics=metrics,
        metadata={
            "module_name": MODULE_NAME,
            "module_version": MODULE_VERSION,
            "policy": active_policy,
            "failed_checks": failed_checks,
            "vertex_precision_digits": vertex_precision_digits,
        },
        warnings=[f"failed CFD-surface check: {check}" for check in failed_checks],
    )


def read_stl_triangles(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    data = path.read_bytes()
    if _looks_like_binary_stl(data):
        return _read_binary_stl(data)
    return _read_ascii_stl(data.decode("utf-8", errors="replace"))


def build_surface_result(
    stl_path: Path,
    triangles: list[tuple[tuple[float, float, float], ...]],
    *,
    vertex_precision_digits: int,
) -> dict[str, Any]:
    vertex_ids: dict[tuple[float, float, float], int] = {}
    edges: dict[tuple[int, int], int] = {}
    face_keys: dict[tuple[int, int, int], int] = {}
    edge_lengths: list[float] = []
    qualities: list[float] = []
    degenerate_count = 0

    for tri in triangles:
        ids = []
        for vertex in tri:
            key = tuple(round(component, vertex_precision_digits) for component in vertex)
            if key not in vertex_ids:
                vertex_ids[key] = len(vertex_ids)
            ids.append(vertex_ids[key])

        face_key = tuple(sorted(ids))
        face_keys[face_key] = face_keys.get(face_key, 0) + 1
        for a, b in ((ids[0], ids[1]), (ids[1], ids[2]), (ids[2], ids[0])):
            edge = tuple(sorted((a, b)))
            edges[edge] = edges.get(edge, 0) + 1

        lengths = [
            _distance(tri[0], tri[1]),
            _distance(tri[1], tri[2]),
            _distance(tri[2], tri[0]),
        ]
        edge_lengths.extend(lengths)
        quality = _triangle_quality(lengths, tri)
        qualities.append(quality)
        if quality <= 0.0 or min(lengths) <= 0.0:
            degenerate_count += 1

    boundary_edges = sum(1 for count in edges.values() if count == 1)
    nonmanifold_edges = sum(1 for count in edges.values() if count > 2)
    duplicate_triangles = sum(count - 1 for count in face_keys.values() if count > 1)
    min_quality = min(qualities) if qualities else 0.0
    min_edge = min(edge_lengths) if edge_lengths else 0.0
    aspect_p99 = _percentile(
        [_triangle_aspect_ratio(tri) for tri in triangles],
        0.99,
    )
    return {
        "surface_result_schema_version": "0.1.0",
        "stl_path": str(stl_path),
        "case_units": {"length": "mm"},
        "counts": {
            "vertices": len(vertex_ids),
            "triangles": len(triangles),
            "edges": len(edges),
        },
        "topology": {
            "boundary_edges": boundary_edges,
            "nonmanifold_edges": nonmanifold_edges,
            "duplicate_triangles": duplicate_triangles,
            "watertight_edge_manifold": boundary_edges == 0 and nonmanifold_edges == 0,
        },
        "quality": {
            "degenerate_triangles": degenerate_count,
            "min_triangle_quality": min_quality,
            "min_edge_length_mm": min_edge,
            "aspect_p99": aspect_p99,
        },
    }


def evaluate_surface_checks(
    surface_result: dict[str, Any],
    policy: dict[str, Any],
) -> list[str]:
    failed: list[str] = []
    counts = surface_result["counts"]
    topology = surface_result["topology"]
    quality = surface_result["quality"]
    if topology["boundary_edges"] > int(policy["max_boundary_edges"]):
        failed.append("boundary_edges")
    if topology["nonmanifold_edges"] > int(policy["max_nonmanifold_edges"]):
        failed.append("nonmanifold_edges")
    if topology["duplicate_triangles"] > int(policy["max_duplicate_triangles"]):
        failed.append("duplicate_triangles")
    if quality["degenerate_triangles"] > int(policy["max_degenerate_triangles"]):
        failed.append("degenerate_triangles")
    if quality["min_triangle_quality"] < float(policy["min_triangle_quality"]):
        failed.append("min_triangle_quality")
    if quality["min_edge_length_mm"] < float(policy["min_edge_length_mm"]):
        failed.append("min_edge_length_mm")
    if counts["triangles"] > int(policy["max_triangles"]):
        failed.append("max_triangles")
    if counts["vertices"] > int(policy["max_vertices"]):
        failed.append("max_vertices")
    return failed


def _looks_like_binary_stl(data: bytes) -> bool:
    if len(data) < 84:
        return False
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    return 84 + triangle_count * 50 == len(data)


def _read_binary_stl(data: bytes) -> list[tuple[tuple[float, float, float], ...]]:
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    triangles = []
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12fH", data, offset)
        triangles.append((values[3:6], values[6:9], values[9:12]))
        offset += 50
    return triangles


def _read_ascii_stl(text: str) -> list[tuple[tuple[float, float, float], ...]]:
    vertices = []
    triangles = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(vertices) == 3:
                triangles.append((vertices[0], vertices[1], vertices[2]))
                vertices = []
    return triangles


def _triangle_quality(
    lengths: list[float],
    tri: tuple[tuple[float, float, float], ...],
) -> float:
    denom = sum(length * length for length in lengths)
    if denom <= 0.0:
        return 0.0
    area = _triangle_area(tri)
    return (4.0 * math.sqrt(3.0) * area) / denom


def _triangle_aspect_ratio(tri: tuple[tuple[float, float, float], ...]) -> float:
    lengths = [
        _distance(tri[0], tri[1]),
        _distance(tri[1], tri[2]),
        _distance(tri[2], tri[0]),
    ]
    min_len = min(lengths)
    return float("inf") if min_len <= 0.0 else max(lengths) / min_len


def _triangle_area(tri: tuple[tuple[float, float, float], ...]) -> float:
    ax, ay, az = tri[0]
    bx, by, bz = tri[1]
    cx, cy, cz = tri[2]
    ab = (bx - ax, by - ay, bz - az)
    ac = (cx - ax, cy - ay, cz - az)
    cross = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    return 0.5 * math.sqrt(sum(component * component for component in cross))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    index = min(len(finite) - 1, max(0, round((len(finite) - 1) * q)))
    return finite[index]


def _metric(value: float | int | None, unit: str | None) -> dict[str, object]:
    safe_value: float | int = 0 if value is None else value
    return MetricValue(
        value=round(safe_value, 12) if isinstance(safe_value, float) else safe_value,
        unit=unit,
        confidence=1.0,
        source=MODULE_NAME,
    ).to_dict()
