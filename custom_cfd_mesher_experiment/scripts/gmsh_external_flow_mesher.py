from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import gmsh
import numpy as np
import trimesh
import trimesh.repair


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--target-faces", type=int, default=0)
    parser.add_argument("--padding", default="0.7,0.5,0.5")
    parser.add_argument(
        "--farfield-policy",
        choices=["fixed-padding", "dynamic"],
        default="fixed-padding",
    )
    parser.add_argument("--upstream-lengths", type=float, default=1.5)
    parser.add_argument("--downstream-lengths", type=float, default=3.0)
    parser.add_argument("--side-y-spans", type=float, default=2.0)
    parser.add_argument("--side-z-spans", type=float, default=2.0)
    parser.add_argument("--min-farfield-padding", type=float, default=0.15)
    parser.add_argument("--surface-size", type=float, default=0.025)
    parser.add_argument("--feature-size", type=float, default=0.0)
    parser.add_argument("--feature-distance-min", type=float, default=0.0)
    parser.add_argument("--feature-distance-max", type=float, default=0.0)
    parser.add_argument("--refinement-boxes", default="")
    parser.add_argument("--farfield-size", type=float, default=0.18)
    parser.add_argument(
        "--farfield-size-policy",
        choices=["fixed", "dynamic"],
        default="fixed",
    )
    parser.add_argument("--farfield-size-fraction", type=float, default=0.15)
    parser.add_argument("--min-farfield-size", type=float, default=0.04)
    parser.add_argument("--max-farfield-size", type=float, default=0.35)
    parser.add_argument(
        "--farfield-patches",
        choices=["single", "split"],
        default="single",
        help="Use one farfield physical group or split box faces into inlet/outlet/side patches.",
    )
    parser.add_argument("--angle-deg", type=float, default=40.0)
    parser.add_argument("--curve-angle-deg", type=float, default=180.0)
    parser.add_argument("--algorithm3d", type=int, default=10)
    parser.add_argument("--optimize", default="default,Netgen")
    parser.add_argument(
        "--geometry-mode",
        choices=["create-geometry", "create-topology", "none"],
        default="create-geometry",
    )
    args = parser.parse_args()

    args.run_dir.mkdir(parents=True, exist_ok=True)
    prepared_stl = args.run_dir / "aircraft_prepared_for_gmsh.stl"
    surface_report = prepare_surface(
        args.input_stl,
        prepared_stl,
        scale=args.scale,
        target_faces=args.target_faces,
    )

    report = run_gmsh(
        prepared_stl,
        args.run_dir,
        farfield_policy=args.farfield_policy,
        fixed_padding=parse_vector3(args.padding),
        upstream_lengths=args.upstream_lengths,
        downstream_lengths=args.downstream_lengths,
        side_y_spans=args.side_y_spans,
        side_z_spans=args.side_z_spans,
        min_farfield_padding=args.min_farfield_padding,
        surface_size=args.surface_size,
        feature_size=args.feature_size,
        feature_distance_min=args.feature_distance_min,
        feature_distance_max=args.feature_distance_max,
        refinement_boxes=parse_refinement_boxes(args.refinement_boxes),
        farfield_size=args.farfield_size,
        farfield_size_policy=args.farfield_size_policy,
        farfield_size_fraction=args.farfield_size_fraction,
        min_farfield_size=args.min_farfield_size,
        max_farfield_size=args.max_farfield_size,
        farfield_patches=args.farfield_patches,
        angle_deg=args.angle_deg,
        curve_angle_deg=args.curve_angle_deg,
        algorithm3d=args.algorithm3d,
        geometry_mode=args.geometry_mode,
        optimizers=parse_optimizers(args.optimize),
        surface_report=surface_report,
    )
    (args.run_dir / "gmsh_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def prepare_surface(input_stl: Path, output_stl: Path, *, scale: float, target_faces: int) -> dict[str, object]:
    mesh = trimesh.load(input_stl, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("Expected a single triangle mesh")
    mesh = mesh.copy()
    mesh.vertices *= scale
    mesh.merge_vertices(digits_vertex=9)
    trimesh.repair.fix_normals(mesh, multibody=True)
    if mesh.volume < 0:
        mesh.invert()
    if target_faces and len(mesh.faces) > target_faces:
        mesh = mesh.simplify_quadric_decimation(face_count=target_faces, aggression=3)
        mesh.merge_vertices(digits_vertex=9)
        trimesh.repair.fix_normals(mesh, multibody=True)
        if mesh.volume < 0:
            mesh.invert()
    output_stl.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(output_stl)
    return {
        "input_stl": str(input_stl),
        "prepared_stl": str(output_stl),
        "scale": scale,
        "target_faces": target_faces or None,
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "bounds": mesh.bounds.tolist(),
    }


def run_gmsh(
    prepared_stl: Path,
    run_dir: Path,
    *,
    farfield_policy: str,
    fixed_padding: list[float],
    upstream_lengths: float,
    downstream_lengths: float,
    side_y_spans: float,
    side_z_spans: float,
    min_farfield_padding: float,
    surface_size: float,
    feature_size: float,
    feature_distance_min: float,
    feature_distance_max: float,
    refinement_boxes: list[dict[str, object]],
    farfield_size: float,
    farfield_size_policy: str,
    farfield_size_fraction: float,
    min_farfield_size: float,
    max_farfield_size: float,
    farfield_patches: str,
    angle_deg: float,
    curve_angle_deg: float,
    algorithm3d: int,
    geometry_mode: str,
    optimizers: list[str],
    surface_report: dict[str, object],
) -> dict[str, object]:
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 1)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
    gmsh.option.setNumber("Mesh.Algorithm3D", algorithm3d)
    bounds = np.asarray(surface_report["bounds"], dtype=float)
    farfield_domain = compute_farfield_domain(
        bounds,
        policy=farfield_policy,
        fixed_padding=fixed_padding,
        upstream_lengths=upstream_lengths,
        downstream_lengths=downstream_lengths,
        side_y_spans=side_y_spans,
        side_z_spans=side_z_spans,
        min_padding=min_farfield_padding,
    )
    effective_farfield_size = compute_farfield_mesh_size(
        bounds,
        fixed_size=farfield_size,
        policy=farfield_size_policy,
        fraction=farfield_size_fraction,
        min_size=min_farfield_size,
        max_size=max_farfield_size,
        surface_size=surface_size,
    )

    size_min_candidates = [surface_size, effective_farfield_size]
    if feature_size > 0.0:
        size_min_candidates.append(feature_size)
    for refinement_box in refinement_boxes:
        size_min_candidates.append(float(refinement_box["size"]))
    gmsh.option.setNumber("Mesh.MeshSizeMin", min(size_min_candidates))
    gmsh.option.setNumber("Mesh.MeshSizeMax", max(surface_size, effective_farfield_size))
    gmsh.option.setNumber("Mesh.SaveAll", 0)
    try:
        gmsh.model.add("aircraft_external_flow")
        gmsh.merge(str(prepared_stl))
        gmsh.model.mesh.classifySurfaces(
            angle_deg * math.pi / 180.0,
            True,
            True,
            curve_angle_deg * math.pi / 180.0,
        )
        if geometry_mode == "create-geometry":
            gmsh.model.mesh.createGeometry()
        elif geometry_mode == "create-topology":
            gmsh.model.mesh.createTopology(True, True)
        aircraft_surfaces = [tag for dim, tag in gmsh.model.getEntities(2)]
        aircraft_curves = [tag for dim, tag in gmsh.model.getEntities(1)]
        aircraft_points = gmsh.model.getEntities(0)
        gmsh.model.mesh.setSize(aircraft_points, surface_size)

        farfield_box = add_farfield_box(farfield_domain["bounds"], effective_farfield_size)
        box_surfaces = list(farfield_box["surfaces"].values())
        gmsh.model.geo.synchronize()

        outer_loop = gmsh.model.geo.addSurfaceLoop(box_surfaces)
        inner_loop = gmsh.model.geo.addSurfaceLoop(aircraft_surfaces)
        volume = gmsh.model.geo.addVolume([outer_loop, inner_loop])
        gmsh.model.geo.synchronize()

        aircraft_group = gmsh.model.addPhysicalGroup(2, aircraft_surfaces)
        gmsh.model.setPhysicalName(2, aircraft_group, "aircraft")
        farfield_groups = add_farfield_physical_groups(farfield_box["surfaces"], farfield_patches)
        fluid_group = gmsh.model.addPhysicalGroup(3, [volume])
        gmsh.model.setPhysicalName(3, fluid_group, "fluid")
        background_fields = []
        feature_field = add_feature_size_field(
            aircraft_curves,
            surface_size=surface_size,
            feature_size=feature_size,
            dist_min=feature_distance_min,
            dist_max=feature_distance_max,
        )
        if feature_field is not None:
            background_fields.append(int(feature_field["field"]))
        refinement_field = add_refinement_box_fields(
            bounds,
            refinement_boxes,
            surface_size=surface_size,
        )
        if refinement_field is not None:
            background_fields.append(int(refinement_field["field"]))
        if len(background_fields) > 1:
            combined = gmsh.model.mesh.field.add("Min")
            gmsh.model.mesh.field.setNumbers(combined, "FieldsList", background_fields)
            gmsh.model.mesh.field.setAsBackgroundMesh(combined)
            background_field = {"type": "min", "field": combined, "children": background_fields}
        elif len(background_fields) == 1:
            gmsh.model.mesh.field.setAsBackgroundMesh(background_fields[0])
            background_field = {"type": "single", "field": background_fields[0]}
        else:
            background_field = None

        gmsh.model.mesh.generate(3)
        for optimizer in optimizers:
            method = "" if optimizer == "default" else optimizer
            gmsh.model.mesh.optimize(method, True, 3)
        msh_path = run_dir / "mesh.msh"
        su2_path = run_dir / "mesh.su2"
        gmsh.write(str(msh_path))
        openfoam_case = write_openfoam_case_skeleton(run_dir / "openfoam_case")
        try:
            gmsh.write(str(su2_path))
            su2_written = True
        except Exception as exc:
            su2_written = False
            (run_dir / "mesh_su2_export_error.txt").write_text(str(exc), encoding="utf-8")

        elem_types, elem_tags, _ = gmsh.model.mesh.getElements(3)
        node_tags, _, _ = gmsh.model.mesh.getNodes()
        return {
            "strategy": "gmsh_external_flow_from_stl",
            "prepared_surface": surface_report,
            "farfield_policy": farfield_policy,
            "fixed_padding": fixed_padding,
            "farfield_domain": farfield_domain,
            "surface_size": surface_size,
            "feature_size": feature_size or None,
            "feature_distance_min": feature_distance_min or None,
            "feature_distance_max": feature_distance_max or None,
            "feature_field": feature_field,
            "refinement_boxes": refinement_boxes,
            "refinement_field": refinement_field,
            "background_field": background_field,
            "farfield_size": effective_farfield_size,
            "farfield_size_policy": farfield_size_policy,
            "requested_farfield_size": farfield_size,
            "farfield_size_fraction": farfield_size_fraction,
            "min_farfield_size": min_farfield_size,
            "max_farfield_size": max_farfield_size,
            "farfield_patches": farfield_patches,
            "farfield_groups": farfield_groups,
            "angle_deg": angle_deg,
            "curve_angle_deg": curve_angle_deg,
            "algorithm3d": algorithm3d,
            "geometry_mode": geometry_mode,
            "optimizers": optimizers,
            "aircraft_surface_count": len(aircraft_surfaces),
            "aircraft_curve_count": len(aircraft_curves),
            "box_surface_count": len(box_surfaces),
            "nodes": int(len(node_tags)),
            "volume_element_types": [int(value) for value in elem_types],
            "volume_element_count": int(sum(len(tags) for tags in elem_tags)),
            "msh": str(msh_path),
            "openfoam_case": str(openfoam_case),
            "su2": str(su2_path) if su2_written else None,
            "su2_written": su2_written,
        }
    finally:
        gmsh.finalize()


def add_feature_size_field(
    aircraft_curves: list[int],
    *,
    surface_size: float,
    feature_size: float,
    dist_min: float,
    dist_max: float,
) -> dict[str, object] | None:
    if feature_size <= 0.0:
        return None
    if not aircraft_curves:
        return None
    if feature_size >= surface_size:
        return None
    if dist_min < 0.0 or dist_max <= dist_min:
        raise ValueError("Feature sizing requires 0 <= feature_distance_min < feature_distance_max")

    distance = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(distance, "CurvesList", aircraft_curves)
    gmsh.model.mesh.field.setNumber(distance, "Sampling", 80)

    threshold = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(threshold, "InField", distance)
    gmsh.model.mesh.field.setNumber(threshold, "SizeMin", feature_size)
    gmsh.model.mesh.field.setNumber(threshold, "SizeMax", surface_size)
    gmsh.model.mesh.field.setNumber(threshold, "DistMin", dist_min)
    gmsh.model.mesh.field.setNumber(threshold, "DistMax", dist_max)
    return {
        "type": "curve_distance_threshold",
        "curve_count": len(aircraft_curves),
        "distance_field": distance,
        "threshold_field": threshold,
        "field": threshold,
    }


def add_refinement_box_fields(
    bounds: np.ndarray,
    refinement_boxes: list[dict[str, object]],
    *,
    surface_size: float,
) -> dict[str, object] | None:
    if not refinement_boxes:
        return None
    fields = []
    lo, hi = bounds
    span = hi - lo
    for box in refinement_boxes:
        fractions = np.asarray(box["bounds"], dtype=float)
        box_lo = lo + span * fractions[[0, 2, 4]]
        box_hi = lo + span * fractions[[1, 3, 5]]
        field = gmsh.model.mesh.field.add("Box")
        gmsh.model.mesh.field.setNumber(field, "VIn", float(box["size"]))
        gmsh.model.mesh.field.setNumber(field, "VOut", surface_size)
        gmsh.model.mesh.field.setNumber(field, "XMin", float(box_lo[0]))
        gmsh.model.mesh.field.setNumber(field, "XMax", float(box_hi[0]))
        gmsh.model.mesh.field.setNumber(field, "YMin", float(box_lo[1]))
        gmsh.model.mesh.field.setNumber(field, "YMax", float(box_hi[1]))
        gmsh.model.mesh.field.setNumber(field, "ZMin", float(box_lo[2]))
        gmsh.model.mesh.field.setNumber(field, "ZMax", float(box_hi[2]))
        gmsh.model.mesh.field.setNumber(field, "Thickness", float(box.get("thickness", 0.01)))
        fields.append(field)
        box["absolute_bounds"] = [box_lo.tolist(), box_hi.tolist()]
    if len(fields) == 1:
        return {"type": "box", "field": fields[0], "children": fields}
    combined = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(combined, "FieldsList", fields)
    return {"type": "box_min", "field": combined, "children": fields}


def add_farfield_physical_groups(box_surfaces: dict[str, int], mode: str) -> dict[str, list[int]]:
    if mode == "single":
        surfaces = list(box_surfaces.values())
        group = gmsh.model.addPhysicalGroup(2, surfaces)
        gmsh.model.setPhysicalName(2, group, "farfield")
        return {"farfield": surfaces}
    if mode != "split":
        raise ValueError(f"Unknown farfield patch mode: {mode}")

    groups = {
        "inlet": [box_surfaces["xmin"]],
        "outlet": [box_surfaces["xmax"]],
        "side_ymin": [box_surfaces["ymin"]],
        "side_ymax": [box_surfaces["ymax"]],
        "side_zmin": [box_surfaces["zmin"]],
        "side_zmax": [box_surfaces["zmax"]],
    }
    for name, surfaces in groups.items():
        group = gmsh.model.addPhysicalGroup(2, surfaces)
        gmsh.model.setPhysicalName(2, group, name)
    return groups


def compute_farfield_domain(
    bounds: np.ndarray,
    *,
    policy: str,
    fixed_padding: list[float],
    upstream_lengths: float,
    downstream_lengths: float,
    side_y_spans: float,
    side_z_spans: float,
    min_padding: float,
) -> dict[str, object]:
    span = bounds[1] - bounds[0]
    if np.any(span <= 0.0):
        raise ValueError(f"Invalid aircraft bounds for farfield sizing: {bounds.tolist()}")

    if policy == "fixed-padding":
        lo_pad = np.asarray(fixed_padding, dtype=float)
        hi_pad = np.asarray(fixed_padding, dtype=float)
    elif policy == "dynamic":
        if min(upstream_lengths, downstream_lengths, side_y_spans, side_z_spans, min_padding) < 0.0:
            raise ValueError("Dynamic farfield sizing values must be non-negative")
        lo_pad = np.asarray(
            [
                max(min_padding, span[0] * upstream_lengths),
                max(min_padding, span[1] * side_y_spans),
                max(min_padding, span[2] * side_z_spans),
            ],
            dtype=float,
        )
        hi_pad = np.asarray(
            [
                max(min_padding, span[0] * downstream_lengths),
                max(min_padding, span[1] * side_y_spans),
                max(min_padding, span[2] * side_z_spans),
            ],
            dtype=float,
        )
    else:
        raise ValueError(f"Unknown farfield policy: {policy}")

    lo = bounds[0] - lo_pad
    hi = bounds[1] + hi_pad
    return {
        "policy": policy,
        "aircraft_bounds": bounds.tolist(),
        "aircraft_span": span.tolist(),
        "padding_lo": lo_pad.tolist(),
        "padding_hi": hi_pad.tolist(),
        "bounds": {
            "lo": lo.tolist(),
            "hi": hi.tolist(),
        },
        "parameters": {
            "upstream_lengths": upstream_lengths,
            "downstream_lengths": downstream_lengths,
            "side_y_spans": side_y_spans,
            "side_z_spans": side_z_spans,
            "min_padding": min_padding,
            "fixed_padding": fixed_padding,
        },
    }


def compute_farfield_mesh_size(
    bounds: np.ndarray,
    *,
    fixed_size: float,
    policy: str,
    fraction: float,
    min_size: float,
    max_size: float,
    surface_size: float,
) -> float:
    if fixed_size <= 0.0 or fraction <= 0.0 or min_size <= 0.0 or max_size <= 0.0 or surface_size <= 0.0:
        raise ValueError("Mesh sizes and dynamic farfield fraction must be positive")
    if min_size > max_size:
        raise ValueError("min_farfield_size must be <= max_farfield_size")
    if policy == "fixed":
        return fixed_size
    if policy != "dynamic":
        raise ValueError(f"Unknown farfield size policy: {policy}")
    characteristic = float(np.max(bounds[1] - bounds[0]))
    dynamic_size = characteristic * fraction
    return max(surface_size, min(max(dynamic_size, min_size), max_size))


def add_farfield_box(bounds: dict[str, list[float]], lc: float) -> dict[str, object]:
    lo = np.asarray(bounds["lo"], dtype=float)
    hi = np.asarray(bounds["hi"], dtype=float)
    p = [
        gmsh.model.geo.addPoint(lo[0], lo[1], lo[2], lc),
        gmsh.model.geo.addPoint(hi[0], lo[1], lo[2], lc),
        gmsh.model.geo.addPoint(hi[0], hi[1], lo[2], lc),
        gmsh.model.geo.addPoint(lo[0], hi[1], lo[2], lc),
        gmsh.model.geo.addPoint(lo[0], lo[1], hi[2], lc),
        gmsh.model.geo.addPoint(hi[0], lo[1], hi[2], lc),
        gmsh.model.geo.addPoint(hi[0], hi[1], hi[2], lc),
        gmsh.model.geo.addPoint(lo[0], hi[1], hi[2], lc),
    ]
    lines = [
        gmsh.model.geo.addLine(p[0], p[1]),
        gmsh.model.geo.addLine(p[1], p[2]),
        gmsh.model.geo.addLine(p[2], p[3]),
        gmsh.model.geo.addLine(p[3], p[0]),
        gmsh.model.geo.addLine(p[4], p[5]),
        gmsh.model.geo.addLine(p[5], p[6]),
        gmsh.model.geo.addLine(p[6], p[7]),
        gmsh.model.geo.addLine(p[7], p[4]),
        gmsh.model.geo.addLine(p[0], p[4]),
        gmsh.model.geo.addLine(p[1], p[5]),
        gmsh.model.geo.addLine(p[2], p[6]),
        gmsh.model.geo.addLine(p[3], p[7]),
    ]
    loops = {
        "zmin": gmsh.model.geo.addCurveLoop([lines[0], lines[1], lines[2], lines[3]]),
        "zmax": gmsh.model.geo.addCurveLoop([lines[4], lines[5], lines[6], lines[7]]),
        "ymin": gmsh.model.geo.addCurveLoop([lines[0], lines[9], -lines[4], -lines[8]]),
        "xmax": gmsh.model.geo.addCurveLoop([lines[1], lines[10], -lines[5], -lines[9]]),
        "ymax": gmsh.model.geo.addCurveLoop([lines[2], lines[11], -lines[6], -lines[10]]),
        "xmin": gmsh.model.geo.addCurveLoop([lines[3], lines[8], -lines[7], -lines[11]]),
    }
    surfaces = {name: gmsh.model.geo.addPlaneSurface([loop]) for name, loop in loops.items()}
    return {
        "bounds": bounds,
        "surfaces": surfaces,
    }


def write_openfoam_case_skeleton(case_dir: Path) -> Path:
    system_dir = case_dir / "system"
    system_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "constant").mkdir(exist_ok=True)
    control_dict = system_dir / "controlDict"
    control_dict.write_text(
        """/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM
   \\\\    /   O peration     |
    \\\\  /    A nd           |
     \\\\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{
    format      ascii;
    class       dictionary;
    object      controlDict;
}

application     simpleFoam;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         1;
deltaT          1;
writeControl    timeStep;
writeInterval   1;
purgeWrite      0;
writeFormat     ascii;
writePrecision  8;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable true;

// ************************************************************************* //
""",
        encoding="utf-8",
    )
    return case_dir


def parse_vector3(value: str) -> list[float]:
    parts = [float(part) for part in value.split(",") if part.strip()]
    if len(parts) != 3 or any(part <= 0.0 for part in parts):
        raise ValueError("Expected three positive comma-separated values")
    return parts


def parse_optimizers(value: str) -> list[str]:
    values = [part.strip() for part in value.split(",") if part.strip()]
    if any(part.lower() in {"none", "off", "false"} for part in values):
        return []
    return values


def parse_refinement_boxes(value: str) -> list[dict[str, object]]:
    if not value.strip():
        return []
    boxes = []
    for raw_box in value.split(";"):
        if not raw_box.strip():
            continue
        if ":" not in raw_box:
            raise ValueError("Refinement boxes must use name:x0,x1,y0,y1,z0,z1,size[,thickness]")
        name, raw_numbers = raw_box.split(":", 1)
        numbers = [float(part) for part in raw_numbers.split(",") if part.strip()]
        if len(numbers) not in (7, 8):
            raise ValueError("Refinement boxes require 7 or 8 numeric values")
        bounds = numbers[:6]
        if any(value < 0.0 or value > 1.0 for value in bounds):
            raise ValueError("Refinement box normalized bounds must be in [0, 1]")
        if bounds[0] >= bounds[1] or bounds[2] >= bounds[3] or bounds[4] >= bounds[5]:
            raise ValueError("Refinement box min values must be below max values")
        if numbers[6] <= 0.0:
            raise ValueError("Refinement box size must be positive")
        boxes.append(
            {
                "name": name.strip(),
                "bounds": bounds,
                "size": numbers[6],
                "thickness": numbers[7] if len(numbers) == 8 else 0.01,
            }
        )
    return boxes


if __name__ == "__main__":
    main()
