from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from vtk import vtkPolyDataReader
from vtk.util.numpy_support import vtk_to_numpy


VIEW_ROTATIONS = {
    "iso": (-38.0, 23.0, 0.0),
    "top": (0.0, 90.0, 0.0),
    "side": (0.0, 0.0, 0.0),
    "front": (90.0, 0.0, 0.0),
}

# Bounds are normalized against the actual aircraft patch bounds. The exporter
# currently emits x=fuselage length, but the span/vertical axes have appeared as
# both y/z and z/y in sandbox artifacts, so closeup boxes are mapped through the
# inferred span and vertical axes at runtime.
FEATURE_BOXES_XSV = {
    # tuple order: x_min, x_max, span_min, span_max, vertical_min, vertical_max
    "nose_fuselage": (0.00, 0.24, 0.34, 0.66, 0.00, 1.00),
    "fuselage_midbody": (0.22, 0.68, 0.36, 0.64, 0.12, 0.88),
    "wing_root_blend": (0.26, 0.66, 0.32, 0.68, 0.16, 0.74),
    "left_wing_le": (0.18, 0.58, 0.03, 0.36, 0.14, 0.70),
    "right_wing_le": (0.18, 0.58, 0.64, 0.97, 0.14, 0.70),
    "left_wing_te": (0.34, 0.82, 0.03, 0.36, 0.16, 0.76),
    "right_wing_te": (0.34, 0.82, 0.64, 0.97, 0.16, 0.76),
    "left_wingtip": (0.28, 0.82, 0.00, 0.15, 0.10, 0.78),
    "right_wingtip": (0.28, 0.82, 0.85, 1.00, 0.10, 0.78),
    "tail_root_blend": (0.62, 0.96, 0.34, 0.66, 0.16, 0.92),
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--aircraft-vtk", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1400)
    parser.add_argument("--height", type=int, default=1000)
    parser.add_argument("--edges", action="store_true")
    args = parser.parse_args()

    vertices, faces = read_polydata(args.aircraft_vtk)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    axes = infer_axes(vertices)
    feature_boxes = {
        name: map_xsv_box_to_xyz(norm_box, axes) for name, norm_box in FEATURE_BOXES_XSV.items()
    }

    manifest: dict[str, object] = {
        "aircraft_vtk": str(args.aircraft_vtk),
        "point_count": int(len(vertices)),
        "face_count": int(len(faces)),
        "bounds": bounds_dict(vertices),
        "axis_mapping": axes,
        "images": {},
        "feature_boxes_x_span_vertical": FEATURE_BOXES_XSV,
        "feature_boxes_xyz": feature_boxes,
    }

    for view in ("iso", "top", "side", "front"):
        output = args.output_dir / f"{view}.png"
        render_mesh(vertices, faces, view=view, width=args.width, height=args.height, edges=args.edges).save(output)
        manifest["images"][view] = str(output)

    for name, norm_box in feature_boxes.items():
        cropped_vertices, cropped_faces = crop_mesh(vertices, faces, norm_box)
        output = args.output_dir / f"closeup_{name}.png"
        if len(cropped_faces) == 0:
            cropped_vertices, cropped_faces = vertices, faces
        render_mesh(cropped_vertices, cropped_faces, view="iso", width=args.width, height=args.height, edges=True).save(output)
        manifest["images"][f"closeup_{name}"] = str(output)
        manifest[f"{name}_face_count"] = int(len(cropped_faces))

    (args.output_dir / "feature_closeups_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))


def read_polydata(path: Path) -> tuple[np.ndarray, np.ndarray]:
    reader = vtkPolyDataReader()
    reader.SetFileName(str(path))
    reader.Update()
    polydata = reader.GetOutput()
    vertices = vtk_to_numpy(polydata.GetPoints().GetData()).astype(float)
    polys = vtk_to_numpy(polydata.GetPolys().GetData())
    faces: list[list[int]] = []
    cursor = 0
    while cursor < len(polys):
        n = int(polys[cursor])
        face = [int(value) for value in polys[cursor + 1 : cursor + 1 + n]]
        if n == 3:
            faces.append(face)
        elif n > 3:
            for index in range(1, n - 1):
                faces.append([face[0], face[index], face[index + 1]])
        cursor += n + 1
    return vertices, np.asarray(faces, dtype=np.int64)


def bounds_dict(vertices: np.ndarray) -> dict[str, list[float]]:
    lo = vertices.min(axis=0)
    hi = vertices.max(axis=0)
    return {"min": lo.tolist(), "max": hi.tolist(), "span": (hi - lo).tolist()}


def crop_mesh(
    vertices: np.ndarray,
    faces: np.ndarray,
    normalized_bounds: tuple[float, float, float, float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    lo = vertices.min(axis=0)
    hi = vertices.max(axis=0)
    fractions = np.asarray(normalized_bounds, dtype=float)
    crop_lo = lo + (hi - lo) * fractions[[0, 2, 4]]
    crop_hi = lo + (hi - lo) * fractions[[1, 3, 5]]
    centers = vertices[faces].mean(axis=1)
    keep = np.all((centers >= crop_lo) & (centers <= crop_hi), axis=1)
    kept_faces = faces[keep]
    if len(kept_faces) == 0:
        return vertices[:0], faces[:0]
    used = np.unique(kept_faces.reshape(-1))
    remap = {old: new for new, old in enumerate(used)}
    compact_faces = np.vectorize(remap.__getitem__)(kept_faces)
    return vertices[used], compact_faces.astype(np.int64)


def render_mesh(
    vertices: np.ndarray,
    faces: np.ndarray,
    *,
    view: str,
    width: int,
    height: int,
    edges: bool,
) -> Image.Image:
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=np.int64)
    center = 0.5 * (vertices.min(axis=0) + vertices.max(axis=0))
    local = vertices - center
    rotation = rotation_matrix(*VIEW_ROTATIONS[view])
    projected = local @ rotation.T
    xy = projected[:, :2]
    z = projected[:, 2]

    span = np.ptp(xy, axis=0)
    scale = 0.82 * min(width / max(span[0], 1e-12), height / max(span[1], 1e-12))
    screen = np.empty_like(xy)
    screen[:, 0] = width * 0.5 + xy[:, 0] * scale
    screen[:, 1] = height * 0.48 + xy[:, 1] * scale

    face_vertices = vertices[faces]
    normals = np.cross(face_vertices[:, 1] - face_vertices[:, 0], face_vertices[:, 2] - face_vertices[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-15
    normals[valid] /= lengths[valid, None]
    normals = normals @ rotation.T
    face_depth = z[faces].mean(axis=1)
    order = np.argsort(face_depth)
    light = normalize(np.array([0.35, -0.45, 0.82]))

    image = Image.new("RGB", (width, height), (92, 97, 119))
    draw = ImageDraw.Draw(image)
    for face_index in order:
        pts = [tuple(screen[index]) for index in faces[face_index]]
        shade = float(np.clip(np.dot(normals[face_index], light), -0.25, 1.0))
        base = 170 + int(62 * max(shade, 0.0))
        color = (base, base + 2, base + 5)
        draw.polygon(pts, fill=color)
        if edges:
            draw.line([pts[0], pts[1], pts[2], pts[0]], fill=(25, 25, 25), width=1)
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


def infer_axes(vertices: np.ndarray) -> dict[str, int]:
    span = np.ptp(vertices, axis=0)
    # X is the contract for fuselage length in current exporter outputs.
    remaining = [1, 2]
    span_axis = max(remaining, key=lambda axis: span[axis])
    vertical_axis = min(remaining, key=lambda axis: span[axis])
    return {"length": 0, "span": int(span_axis), "vertical": int(vertical_axis)}


def map_xsv_box_to_xyz(
    box: tuple[float, float, float, float, float, float],
    axes: dict[str, int],
) -> tuple[float, float, float, float, float, float]:
    output = [0.0] * 6
    ranges = {
        axes["length"]: (box[0], box[1]),
        axes["span"]: (box[2], box[3]),
        axes["vertical"]: (box[4], box[5]),
    }
    for axis, values in ranges.items():
        output[2 * axis] = values[0]
        output[2 * axis + 1] = values[1]
    return tuple(output)


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm == 0.0:
        return vector
    return vector / norm


if __name__ == "__main__":
    main()
