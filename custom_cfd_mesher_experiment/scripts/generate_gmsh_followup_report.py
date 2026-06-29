from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from PIL import Image, ImageDraw


SELECTED_VARIANTS = {
    "fcv01_long_glider": Path("fcv01_long_glider"),
    "fcv02_short_swept": Path("fcv02_short_swept"),
    "fcv03_high_aspect_mild": Path("fcv03_high_aspect_mild_retry_5mm"),
    "fcv04_compact_wide_tail": Path("fcv04_compact_wide_tail"),
    "fcv05_aft_wing_fast": Path("fcv05_aft_wing_fast"),
}

FEATURE_CROPS = {
    "inlet_cap_body": (0.24, 0.74, 0.34, 0.66, 0.46, 1.02),
    "wing_leading_edge": (0.10, 0.58, 0.00, 1.00, 0.18, 0.92),
    "wing_trailing_edge": (0.38, 0.88, 0.00, 1.00, 0.18, 0.92),
    "wing_root_fuselage_blend": (0.22, 0.72, 0.32, 0.68, 0.12, 1.00),
    "wingtip_left": (0.18, 0.88, 0.00, 0.18, 0.12, 0.96),
    "wingtip_right": (0.18, 0.88, 0.82, 1.00, 0.12, 0.96),
    "tail_root_blend": (0.62, 1.02, 0.30, 0.70, 0.08, 1.02),
}

VIEW_ROTATIONS = {
    "iso": (-38.0, 23.0, 0.0),
    "top": (0.0, 90.0, 0.0),
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1200)
    args = parser.parse_args()

    started = time.perf_counter()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    reports = []
    for variant_id, rel in SELECTED_VARIANTS.items():
        variant_root = args.run_root / rel
        report_path = variant_root / "variant_report.json"
        report = json.loads(report_path.read_text(encoding="utf-8"))
        mesh_run_dir = Path(report["run_dir"])
        if not mesh_run_dir.is_absolute():
            mesh_run_dir = args.run_root.parent.parent / mesh_run_dir
        # Stored run_dir values are relative to custom_cfd_mesher_experiment.
        if not mesh_run_dir.exists():
            mesh_run_dir = args.run_root / rel / "gmsh_mesh"
        if not (mesh_run_dir / "gmsh" / "pipeline_report.json").exists() and (mesh_run_dir / "gmsh_mesh").exists():
            mesh_run_dir = mesh_run_dir / "gmsh_mesh"

        case_dir = mesh_run_dir / "gmsh" / "openfoam_case"
        prepared_stl = mesh_run_dir / "gmsh" / "aircraft_prepared_for_gmsh.stl"
        pipeline_report = mesh_run_dir / "gmsh" / "pipeline_report.json"
        pipeline = json.loads(pipeline_report.read_text(encoding="utf-8"))
        aircraft_bounds = np.asarray(
            pipeline["gmsh_report"]["farfield_domain"]["aircraft_bounds"],
            dtype=float,
        )

        out_dir = args.output_dir / variant_id
        out_dir.mkdir(parents=True, exist_ok=True)
        closeups = render_feature_closeups(
            prepared_stl,
            out_dir / "feature_closeups",
            width=args.width,
            height=args.height,
        )
        nonortho = run_and_localize_nonortho(
            case_dir=case_dir,
            aircraft_stl=prepared_stl,
            aircraft_bounds=aircraft_bounds,
            out_dir=out_dir / "nonortho",
            width=args.width,
            height=args.height,
        )

        variant_summary = {
            "variant_id": variant_id,
            "source_variant_report": str(report_path),
            "case_dir": str(case_dir),
            "aircraft_stl": str(prepared_stl),
            "cells": report.get("cells"),
            "points": report.get("points"),
            "aircraft_patch_faces": report.get("aircraft_patch_faces"),
            "max_skewness": report.get("max_skewness"),
            "max_non_orthogonality": report.get("max_non_orthogonality"),
            "severe_nonorth_faces_reported": report.get("severe_nonorth_faces"),
            "strict_checkmesh_verdict": report.get("strict_checkmesh_verdict"),
            "potential_foam": report.get("potential_foam", {}),
            "surface_deviation_metrics": report.get("surface_deviation_metrics", {}),
            "feature_closeups": closeups,
            "nonortho_localization": nonortho,
        }
        reports.append(variant_summary)
        (out_dir / "followup_variant_summary.json").write_text(
            json.dumps(variant_summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )

    summary = {
        "status": "ready",
        "run_root": str(args.run_root),
        "output_dir": str(args.output_dir),
        "runtime_s": time.perf_counter() - started,
        "variants": reports,
    }
    (args.output_dir / "gmsh_followup_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    (args.output_dir / "gmsh_followup_report.md").write_text(
        render_markdown(summary),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


def run_and_localize_nonortho(
    *,
    case_dir: Path,
    aircraft_stl: Path,
    aircraft_bounds: np.ndarray,
    out_dir: Path,
    width: int,
    height: int,
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "rm -rf postProcessing/checkMesh constant/polyMesh/sets/nonOrthoFaces && "
        "checkMesh -writeSurfaces -writeSets -nonOrthThreshold 70 > log.checkMesh.nonOrth70 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    (out_dir / "checkmesh_nonorth70_driver.log").write_text(result.stdout + result.stderr, encoding="utf-8")

    vtk_path = case_dir / "postProcessing" / "checkMesh" / "constant" / "nonOrthoFaces.vtk"
    copied_vtk = out_dir / "nonOrthoFaces.vtk"
    if vtk_path.exists():
        copied_vtk.write_text(vtk_path.read_text(encoding="utf-8", errors="replace"), encoding="utf-8")

    faces = read_vtk_faces(copied_vtk) if copied_vtk.exists() else {"points": np.empty((0, 3)), "polygons": []}
    centers = polygon_centers(faces["points"], faces["polygons"])
    labels = classify_center_labels(centers, aircraft_bounds)
    feature_counts = {label: int(labels.count(label)) for label in sorted(set(labels))}
    for expected in (
        "inlet_cap",
        "inlet_side_wall",
        "wing_leading_edge",
        "wing_trailing_edge",
        "wing_root_blend",
        "wingtip",
        "tail",
        "fuselage_nose",
        "fuselage_tail",
        "farfield_wake_region",
        "random_volume_cells",
    ):
        feature_counts.setdefault(expected, 0)
    feature_regions = summarize_feature_regions(centers, labels, aircraft_bounds)
    csv_path = out_dir / "nonOrthoFace_centroids.csv"
    write_centroid_csv(csv_path, centers, labels, aircraft_bounds)
    dominant = max(feature_counts.items(), key=lambda item: item[1])[0] if feature_counts else "none"
    likely_cause = likely_cause_for(dominant)
    matters_for_plumbing = False
    blocks_scoring = dominant != "farfield_transition" or len(centers) > 0

    overlay = out_dir / "nonortho_overlay_iso.png"
    if copied_vtk.exists():
        render_nonortho_overlay(
            aircraft_stl=aircraft_stl,
            vtk_faces=faces,
            output=overlay,
            width=width,
            height=height,
            view="iso",
        )
    top_overlay = out_dir / "nonortho_overlay_top.png"
    if copied_vtk.exists():
        render_nonortho_overlay(
            aircraft_stl=aircraft_stl,
            vtk_faces=faces,
            output=top_overlay,
            width=width,
            height=height,
            view="top",
        )

    summary = {
        "status": "ready" if result.returncode == 0 and copied_vtk.exists() else "failed",
        "command": ["wsl", "bash", "-lc", script],
        "returncode": result.returncode,
        "openfoam_log": str(case_dir / "log.checkMesh.nonOrth70"),
        "driver_log": str(out_dir / "checkmesh_nonorth70_driver.log"),
        "nonortho_vtk": str(copied_vtk) if copied_vtk.exists() else None,
        "nonortho_face_count": int(len(centers)),
        "nonortho_centroid_csv": str(csv_path) if csv_path.exists() else None,
        "feature_counts": feature_counts,
        "feature_regions": feature_regions,
        "dominant_location": dominant,
        "likely_cause": likely_cause,
        "matters_for_plumbing": matters_for_plumbing,
        "blocks_scoring_cfd": blocks_scoring,
        "overlay_iso": str(overlay) if overlay.exists() else None,
        "overlay_top": str(top_overlay) if top_overlay.exists() else None,
        "centroid_bounds_m": centers_bounds(centers),
        "classification_note": "Heuristic classification from nonOrthoFaces.vtk centroids against aircraft bounds.",
    }
    (out_dir / "nonortho_localization_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return summary


def render_feature_closeups(stl: Path, out_dir: Path, *, width: int, height: int) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(stl, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one mesh in {stl}")
    images: dict[str, str] = {}
    crop_manifest: dict[str, Any] = {}
    for name, crop in FEATURE_CROPS.items():
        cropped = crop_mesh(mesh, crop)
        output = out_dir / f"{name}.png"
        render_mesh(cropped, width=width, height=height, edges=True, view="iso").save(output)
        images[name] = str(output)
        crop_manifest[name] = {
            "normalized_crop_bounds": crop,
            "faces": int(len(cropped.faces)),
            "vertices": int(len(cropped.vertices)),
        }
    manifest = {
        "source_stl": str(stl),
        "images": images,
        "crops": crop_manifest,
    }
    (out_dir / "feature_closeup_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return manifest


def classify_center_labels(centers: np.ndarray, bounds: np.ndarray) -> list[str]:
    if len(centers) == 0:
        return []
    lo, hi = bounds
    span = np.maximum(hi - lo, 1e-12)
    norm = (centers - lo) / span
    labels: list[str] = []
    for n in norm:
        x, y, z = map(float, n)
        off_aircraft = x < -0.08 or x > 1.08 or y < -0.08 or y > 1.08 or z < -0.08 or z > 1.08
        if off_aircraft:
            labels.append("farfield_wake_region")
        elif 0.25 <= x <= 0.75 and 0.40 <= y <= 0.60 and z >= 0.52:
            labels.append("inlet_cap")
        elif 0.25 <= x <= 0.75 and 0.30 <= y <= 0.70 and z >= 0.45:
            labels.append("inlet_side_wall")
        elif x >= 0.78 and 0.30 <= y <= 0.70:
            labels.append("fuselage_tail")
        elif x >= 0.68 and 0.18 <= y <= 0.82:
            labels.append("tail")
        elif y <= 0.16 or y >= 0.84:
            labels.append("wingtip")
        elif 0.30 <= y <= 0.70 and 0.18 <= x <= 0.72:
            labels.append("wing_root_blend")
        elif x <= 0.38:
            labels.append("wing_leading_edge")
        elif x >= 0.58:
            labels.append("wing_trailing_edge")
        elif x <= 0.18 and 0.36 <= y <= 0.64:
            labels.append("fuselage_nose")
        else:
            labels.append("random_volume_cells")
    return labels


def summarize_feature_regions(
    centers: np.ndarray,
    labels: list[str],
    bounds: np.ndarray,
) -> dict[str, dict[str, object]]:
    if len(centers) == 0:
        return {}
    lo, hi = bounds
    span = np.maximum(hi - lo, 1e-12)
    norm = (centers - lo) / span
    regions: dict[str, dict[str, object]] = {}
    for label in sorted(set(labels)):
        idx = np.asarray([i for i, item in enumerate(labels) if item == label], dtype=int)
        local = centers[idx]
        local_norm = norm[idx]
        regions[label] = {
            "count": int(len(idx)),
            "centroid_mean_m": local.mean(axis=0).tolist(),
            "centroid_bounds_m": [local.min(axis=0).tolist(), local.max(axis=0).tolist()],
            "normalized_mean": local_norm.mean(axis=0).tolist(),
            "normalized_bounds": [local_norm.min(axis=0).tolist(), local_norm.max(axis=0).tolist()],
        }
    return regions


def write_centroid_csv(path: Path, centers: np.ndarray, labels: list[str], bounds: np.ndarray) -> None:
    if len(centers) == 0:
        path.write_text("x_m,y_m,z_m,x_norm,y_norm,z_norm,feature\n", encoding="utf-8")
        return
    lo, hi = bounds
    span = np.maximum(hi - lo, 1e-12)
    norm = (centers - lo) / span
    lines = ["x_m,y_m,z_m,x_norm,y_norm,z_norm,feature"]
    for center, normal, label in zip(centers, norm, labels):
        lines.append(
            ",".join(
                [
                    f"{center[0]:.12g}",
                    f"{center[1]:.12g}",
                    f"{center[2]:.12g}",
                    f"{normal[0]:.12g}",
                    f"{normal[1]:.12g}",
                    f"{normal[2]:.12g}",
                    label,
                ]
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def likely_cause_for(location: str) -> str:
    causes = {
        "inlet_cap": "local curvature and tight feature transitions around the faired inlet cap.",
        "inlet_side_wall": "local curvature and tight feature transitions along the faired inlet side wall.",
        "wing_root_blend": "surface curvature and tetrahedral transition near the wing/fuselage blend.",
        "wing_leading_edge": "thin leading-edge curvature plus isotropic tetrahedral transition.",
        "wing_trailing_edge": "thin trailing-edge wedge and local surface/volume size transition.",
        "wingtip": "tip curvature and local tetrahedral size transition.",
        "tail": "tail root and aft-body intersection curvature.",
        "fuselage_nose": "curved fuselage-nose tetrahedral cells away from named lifting-surface features.",
        "fuselage_tail": "curved fuselage-tail tetrahedral cells and aft-body transition.",
        "farfield_wake_region": "volume-size transition away from the aircraft.",
        "random_volume_cells": "isolated tetrahedral cells without a single dominant feature association.",
    }
    return causes.get(location, "No dominant cause inferred.")


def read_vtk_faces(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    points_match = re.search(r"POINTS\s+(\d+)\s+\w+\s+(.+?)(?:\nPOLYGONS|\nCELLS)", text, re.S)
    if not points_match:
        raise ValueError(f"Could not parse POINTS from {path}")
    point_count = int(points_match.group(1))
    points = np.fromstring(points_match.group(2), sep=" ", dtype=float).reshape((point_count, 3))
    poly_match = re.search(r"(?:POLYGONS|CELLS)\s+(\d+)\s+\d+\s+(.+?)(?:\nCELL_TYPES|\nPOINT_DATA|\nCELL_DATA|$)", text, re.S)
    if not poly_match:
        raise ValueError(f"Could not parse polygons/cells from {path}")
    face_count = int(poly_match.group(1))
    values = np.fromstring(poly_match.group(2), sep=" ", dtype=int)
    polygons: list[np.ndarray] = []
    cursor = 0
    for _ in range(face_count):
        n = int(values[cursor])
        cursor += 1
        polygons.append(values[cursor : cursor + n])
        cursor += n
    return {"points": points, "polygons": polygons}


def polygon_centers(points: np.ndarray, polygons: list[np.ndarray]) -> np.ndarray:
    if not polygons:
        return np.empty((0, 3))
    return np.asarray([points[poly].mean(axis=0) for poly in polygons], dtype=float)


def centers_bounds(centers: np.ndarray) -> list[list[float]] | None:
    if len(centers) == 0:
        return None
    return [centers.min(axis=0).tolist(), centers.max(axis=0).tolist()]


def crop_mesh(mesh: trimesh.Trimesh, normalized_bounds: tuple[float, float, float, float, float, float]) -> trimesh.Trimesh:
    bounds = mesh.bounds
    lo = bounds[0]
    hi = bounds[1]
    fractions = np.asarray(normalized_bounds, dtype=float)
    crop_lo = lo + (hi - lo) * fractions[[0, 2, 4]]
    crop_hi = lo + (hi - lo) * fractions[[1, 3, 5]]
    centers = mesh.triangles_center
    keep = np.all((centers >= crop_lo) & (centers <= crop_hi), axis=1)
    if not np.any(keep):
        return mesh.copy()
    return mesh.submesh([np.flatnonzero(keep)], append=True, repair=False)


def render_nonortho_overlay(
    *,
    aircraft_stl: Path,
    vtk_faces: dict[str, Any],
    output: Path,
    width: int,
    height: int,
    view: str,
) -> None:
    mesh = trimesh.load(aircraft_stl, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one mesh in {aircraft_stl}")
    image, transform = render_mesh(
        mesh,
        width=width,
        height=height,
        edges=False,
        view=view,
        return_transform=True,
    )
    draw = ImageDraw.Draw(image)
    points = vtk_faces["points"]
    polygons = vtk_faces["polygons"]
    if len(points) and polygons:
        screen = project_points(points, transform)
        for poly in polygons:
            pts = [tuple(screen[index]) for index in poly]
            draw.polygon(pts, outline=(220, 20, 20), fill=(230, 30, 30))
    image.save(output)


def render_mesh(
    mesh: trimesh.Trimesh,
    *,
    width: int,
    height: int,
    edges: bool,
    view: str,
    return_transform: bool = False,
) -> Image.Image | tuple[Image.Image, dict[str, Any]]:
    vertices = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.faces, dtype=np.int64)
    center = 0.5 * (mesh.bounds[0] + mesh.bounds[1])
    rotation = rotation_matrix(*VIEW_ROTATIONS[view])
    projected = (vertices - center) @ rotation.T
    xy = projected[:, :2]
    z = projected[:, 2]
    span = np.ptp(xy, axis=0)
    scale = 0.82 * min(width / max(span[0], 1e-12), height / max(span[1], 1e-12))
    transform = {"center": center, "rotation": rotation, "scale": scale, "width": width, "height": height}
    screen = project_points(vertices, transform)
    face_depth = z[faces].mean(axis=1)
    order = np.argsort(face_depth)
    face_normals = np.asarray(mesh.face_normals, dtype=float) @ rotation.T
    light = normalize(np.array([0.35, -0.45, 0.82]))
    image = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    for face_index in order:
        pts = [tuple(screen[index]) for index in faces[face_index]]
        shade = float(np.clip(np.dot(face_normals[face_index], light), -0.25, 1.0))
        base = 188 + int(52 * max(shade, 0.0))
        draw.polygon(pts, fill=(base, base + 2, base + 5))
        if edges:
            draw.line([pts[0], pts[1], pts[2], pts[0]], fill=(45, 45, 45), width=1)
    if return_transform:
        return image, transform
    return image


def project_points(points: np.ndarray, transform: dict[str, Any]) -> np.ndarray:
    projected = (points - transform["center"]) @ transform["rotation"].T
    xy = projected[:, :2]
    screen = np.empty_like(xy)
    screen[:, 0] = transform["width"] * 0.5 + xy[:, 0] * transform["scale"]
    screen[:, 1] = transform["height"] * 0.52 - xy[:, 1] * transform["scale"]
    return screen


def rotation_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    ry = np.array(
        [
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ]
    )
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ]
    )
    return rx @ ry @ rz


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector if norm == 0.0 else vector / norm


def windows_path_to_wsl(path: Path) -> str:
    text = str(path)
    drive, rest = text[0], text[2:]
    return f"/mnt/{drive.lower()}{rest.replace(chr(92), '/')}"


def render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# Gmsh Follow-Up Report",
        "",
        f"Run root: `{summary['run_root']}`",
        f"Output dir: `{summary['output_dir']}`",
        f"Runtime: `{summary['runtime_s']:.1f} s`",
        "",
        "## Severe Non-Orthogonal Localization",
        "",
        "| Variant | Severe faces | Dominant location | Blocks scoring CFD | Bad-face VTK | Overlay |",
        "|---|---:|---|---|---|---|",
    ]
    for report in summary["variants"]:
        non = report["nonortho_localization"]
        lines.append(
            "| {variant} | {count} | {loc} | {block} | `{vtk}` | `{overlay}` |".format(
                variant=report["variant_id"],
                count=non.get("nonortho_face_count"),
                loc=non.get("dominant_location"),
                block=non.get("blocks_scoring_cfd"),
                vtk=non.get("nonortho_vtk"),
                overlay=non.get("overlay_iso"),
            )
        )
    lines.extend(["", "## Feature Closeups", ""])
    for report in summary["variants"]:
        lines.append(f"### {report['variant_id']}")
        images = report["feature_closeups"]["images"]
        for name, path in images.items():
            lines.append(f"- `{name}`: `{path}`")
        lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    main()
