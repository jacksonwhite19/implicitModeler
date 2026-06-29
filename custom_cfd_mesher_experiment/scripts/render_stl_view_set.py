from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image, ImageDraw


VIEW_ROTATIONS = {
    "iso": (-38.0, 23.0, 0.0),
    "top": (0.0, 90.0, 0.0),
    "side": (0.0, 0.0, 0.0),
    "front": (90.0, 0.0, 0.0),
}

DEFAULT_CLOSEUPS = {
    "wing_root_blend": (0.28, 0.58, 0.0, 1.0, 0.32, 0.68),
    "leading_edge": (0.28, 0.72, 0.0, 1.0, 0.10, 0.45),
    "trailing_edge": (0.28, 0.78, 0.0, 1.0, 0.55, 0.90),
    "wingtip": (0.35, 0.82, 0.0, 1.0, 0.0, 0.20),
    "nose": (0.0, 0.22, 0.0, 1.0, 0.0, 1.0),
    "tail": (0.78, 1.0, 0.0, 1.0, 0.0, 1.0),
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1200)
    parser.add_argument("--edges", action="store_true")
    args = parser.parse_args()

    mesh = trimesh.load(args.input_stl, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("Expected a single triangle mesh")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, object] = {"input_stl": str(args.input_stl), "images": {}}
    for name in ("iso", "top", "side", "front"):
        output = args.output_dir / f"{name}.png"
        render_mesh(mesh, view=name, width=args.width, height=args.height, edges=args.edges).save(output)
        manifest["images"][name] = str(output)

    for name, bounds in DEFAULT_CLOSEUPS.items():
        cropped = crop_mesh(mesh, bounds)
        output = args.output_dir / f"closeup_{name}.png"
        render_mesh(cropped, view="iso", width=args.width, height=args.height, edges=args.edges).save(output)
        manifest["images"][f"closeup_{name}"] = str(output)
        manifest[f"closeup_{name}_normalized_bounds"] = bounds

    (args.output_dir / "view_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))


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


def render_mesh(mesh: trimesh.Trimesh, *, view: str, width: int, height: int, edges: bool) -> Image.Image:
    vertices = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.faces, dtype=np.int64)
    center = 0.5 * (mesh.bounds[0] + mesh.bounds[1])
    vertices = vertices - center

    projected = vertices @ rotation_matrix(*VIEW_ROTATIONS[view]).T
    xy = projected[:, :2]
    z = projected[:, 2]

    span = np.ptp(xy, axis=0)
    scale = 0.82 * min(width / max(span[0], 1e-12), height / max(span[1], 1e-12))
    screen = np.empty_like(xy)
    screen[:, 0] = width * 0.5 + xy[:, 0] * scale
    screen[:, 1] = height * 0.48 + xy[:, 1] * scale

    face_depth = z[faces].mean(axis=1)
    order = np.argsort(face_depth)
    face_normals = np.asarray(mesh.face_normals, dtype=float) @ rotation_matrix(*VIEW_ROTATIONS[view]).T
    light = normalize(np.array([0.35, -0.45, 0.82]))

    image = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    for face_index in order:
        pts = [tuple(screen[index]) for index in faces[face_index]]
        shade = float(np.clip(np.dot(face_normals[face_index], light), -0.25, 1.0))
        base = 188 + int(52 * max(shade, 0.0))
        color = (base, base + 2, base + 5)
        draw.polygon(pts, fill=color)
        if edges:
            draw.line([pts[0], pts[1], pts[2], pts[0]], fill=(45, 45, 45), width=1)

    return image


def rotation_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    yaw = np.deg2rad(yaw_deg)
    pitch = np.deg2rad(pitch_deg)
    roll = np.deg2rad(roll_deg)
    rz = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    ry = np.array(
        [
            [np.cos(pitch), 0.0, np.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-np.sin(pitch), 0.0, np.cos(pitch)],
        ]
    )
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll), np.cos(roll)],
        ]
    )
    return rx @ ry @ rz


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm == 0.0:
        return vector
    return vector / norm


if __name__ == "__main__":
    main()
