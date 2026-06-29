from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--patch", default="aircraft")
    parser.add_argument("--output-png", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1200)
    parser.add_argument("--edges", action="store_true")
    args = parser.parse_args()

    poly_dir = args.case_dir / "constant" / "polyMesh"
    points = np.asarray(parse_points(poly_dir / "points"), dtype=float)
    faces = parse_faces(poly_dir / "faces")
    patch = find_patch((poly_dir / "boundary").read_text(encoding="utf-8"), args.patch)
    patch_faces = faces[patch["start"] : patch["start"] + patch["count"]]
    image = render_patch(points, patch_faces, width=args.width, height=args.height, edges=args.edges)
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output_png)


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)


def parse_points(path: Path) -> list[tuple[float, float, float]]:
    text = strip_comments(path.read_text(encoding="utf-8"))
    match = re.search(r"\n\s*(\d+)\s*\(\s*(.*)\s*\)\s*$", text, flags=re.S)
    if not match:
        raise RuntimeError(f"Could not parse points file {path}")
    return [
        (float(x), float(y), float(z))
        for x, y, z in re.findall(r"\(([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\)", match.group(2))
    ]


def parse_faces(path: Path) -> list[list[int]]:
    text = strip_comments(path.read_text(encoding="utf-8"))
    match = re.search(r"\n\s*(\d+)\s*\(\s*(.*)\s*\)\s*$", text, flags=re.S)
    if not match:
        raise RuntimeError(f"Could not parse faces file {path}")
    faces: list[list[int]] = []
    for _, verts in re.findall(r"(\d+)\(([^)]*)\)", match.group(2)):
        faces.append([int(item) for item in verts.split()])
    return faces


def find_patch(boundary_text: str, patch_name: str) -> dict[str, int]:
    match = re.search(
        rf"(?ms)^\s*{re.escape(patch_name)}\s*\{{.*?nFaces\s+(\d+);\s*startFace\s+(\d+);.*?^\s*\}}",
        boundary_text,
    )
    if not match:
        raise RuntimeError(f"Could not find patch {patch_name!r}")
    return {"count": int(match.group(1)), "start": int(match.group(2))}


def render_patch(
    points: np.ndarray,
    faces: list[list[int]],
    *,
    width: int,
    height: int,
    edges: bool,
) -> Image.Image:
    used = np.unique(np.fromiter((index for face in faces for index in face), dtype=np.int64))
    center = 0.5 * (points[used].min(axis=0) + points[used].max(axis=0))
    shifted = points - center

    rotation = iso_rotation()
    projected = shifted @ rotation.T
    xy = projected[:, :2]
    z = projected[:, 2]
    patch_xy = xy[used]
    span = np.ptp(patch_xy, axis=0)
    scale = 0.78 * min(width / max(float(span[0]), 1e-12), height / max(float(span[1]), 1e-12))

    screen = np.empty_like(xy)
    screen[:, 0] = width * 0.5 + xy[:, 0] * scale
    screen[:, 1] = height * 0.47 + xy[:, 1] * scale

    face_depth = np.asarray([z[face].mean() for face in faces])
    order = np.argsort(face_depth)
    light = normalize(np.array([0.35, -0.45, 0.82]))
    image = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    for face_index in order:
        face = faces[int(face_index)]
        pts = [tuple(screen[index]) for index in face]
        normal = newell_normal(projected[face])
        normal = normalize(normal)
        shade = float(np.clip(np.dot(normal, light), -0.25, 1.0))
        base = 188 + int(52 * max(shade, 0.0))
        draw.polygon(pts, fill=(base, base + 2, base + 5))
        if edges:
            draw.line(pts + [pts[0]], fill=(45, 45, 45), width=1)
    return image


def iso_rotation() -> np.ndarray:
    yaw = math.radians(-38.0)
    pitch = math.radians(23.0)
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
    return ry @ rz


def newell_normal(vertices: np.ndarray) -> np.ndarray:
    normal = np.zeros(3)
    for a, b in zip(vertices, np.vstack([vertices[1:], vertices[:1]])):
        normal[0] += (a[1] - b[1]) * (a[2] + b[2])
        normal[1] += (a[2] - b[2]) * (a[0] + b[0])
        normal[2] += (a[0] - b[0]) * (a[1] + b[1])
    return normal


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-15:
        return vector
    return vector / norm


if __name__ == "__main__":
    main()
