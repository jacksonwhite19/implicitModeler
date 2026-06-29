from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import gmsh
import numpy as np

from gmsh_external_flow_mesher import (
    add_farfield_box,
    parse_vector3,
    prepare_surface,
    write_openfoam_case_skeleton,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--target-faces", type=int, default=2000)
    parser.add_argument("--padding", default="0.7,0.5,0.5")
    parser.add_argument("--surface-size", type=float, default=0.01)
    parser.add_argument("--farfield-size", type=float, default=0.12)
    parser.add_argument("--layer-heights", default="0.0005,0.0015")
    parser.add_argument("--layer-elements", default="1,1")
    parser.add_argument(
        "--thickness-suppression-boxes",
        default="",
        help="Semicolon-separated xmin,xmax,ymin,ymax,zmin,zmax,multiplier boxes for local BL thickness.",
    )
    parser.add_argument(
        "--skip-layer-surface-boxes",
        default="",
        help="Semicolon-separated xmin,xmax,ymin,ymax,zmin,zmax boxes. Classified surface patches with centers inside these boxes will not receive BL extrusion.",
    )
    parser.add_argument(
        "--skip-layer-min-nodes",
        type=int,
        default=0,
        help="Do not extrude boundary layers from classified aircraft surface patches with fewer mesh nodes than this threshold.",
    )
    parser.add_argument("--no-recombine", action="store_true")
    parser.add_argument("--angle-deg", type=float, default=40.0)
    parser.add_argument("--curve-angle-deg", type=float, default=180.0)
    parser.add_argument("--algorithm3d", type=int, default=10)
    parser.add_argument(
        "--geometry-mode",
        choices=["create-geometry", "create-topology"],
        default="create-topology",
    )
    parser.add_argument(
        "--auto-orient-discrete-surfaces",
        action="store_true",
        help="Reverse classified STL surface patches whose average normal points toward the aircraft centroid.",
    )
    parser.add_argument("--optimize", default="default,Netgen,Relocate3D")
    args = parser.parse_args()

    args.run_dir.mkdir(parents=True, exist_ok=True)
    prepared_stl = args.run_dir / "aircraft_prepared_for_gmsh.stl"
    surface_report = prepare_surface(
        args.input_stl,
        prepared_stl,
        scale=args.scale,
        target_faces=args.target_faces,
    )
    report = run_boundary_layer_gmsh(
        prepared_stl,
        args.run_dir,
        surface_report=surface_report,
        padding=parse_vector3(args.padding),
        surface_size=args.surface_size,
        farfield_size=args.farfield_size,
        layer_heights=parse_float_list(args.layer_heights),
        layer_elements=parse_int_list(args.layer_elements),
        thickness_suppression_boxes=parse_suppression_boxes(args.thickness_suppression_boxes),
        skip_layer_surface_boxes=parse_plain_boxes(args.skip_layer_surface_boxes),
        skip_layer_min_nodes=args.skip_layer_min_nodes,
        recombine=not args.no_recombine,
        angle_deg=args.angle_deg,
        curve_angle_deg=args.curve_angle_deg,
        algorithm3d=args.algorithm3d,
        geometry_mode=args.geometry_mode,
        auto_orient_discrete_surfaces=args.auto_orient_discrete_surfaces,
        optimizers=parse_optimizers(args.optimize),
    )
    (args.run_dir / "gmsh_boundary_layer_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def run_boundary_layer_gmsh(
    prepared_stl: Path,
    run_dir: Path,
    *,
    surface_report: dict[str, object],
    padding: list[float],
    surface_size: float,
    farfield_size: float,
    layer_heights: list[float],
    layer_elements: list[int],
    thickness_suppression_boxes: list[dict[str, float]],
    skip_layer_surface_boxes: list[dict[str, float]],
    skip_layer_min_nodes: int,
    recombine: bool,
    angle_deg: float,
    curve_angle_deg: float,
    algorithm3d: int,
    geometry_mode: str,
    auto_orient_discrete_surfaces: bool,
    optimizers: list[str],
) -> dict[str, object]:
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 1)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
    gmsh.option.setNumber("Mesh.Algorithm3D", algorithm3d)
    gmsh.option.setNumber("Mesh.MeshSizeMin", min([surface_size, farfield_size, *layer_heights]))
    gmsh.option.setNumber("Mesh.MeshSizeMax", max(surface_size, farfield_size))
    gmsh.option.setNumber("Mesh.SaveAll", 0)
    try:
        gmsh.model.add("aircraft_external_flow_boundary_layer")
        gmsh.merge(str(prepared_stl))
        gmsh.model.mesh.classifySurfaces(
            angle_deg * math.pi / 180.0,
            True,
            True,
            curve_angle_deg * math.pi / 180.0,
        )
        if geometry_mode == "create-geometry":
            gmsh.model.mesh.createGeometry()
        else:
            gmsh.model.mesh.createTopology(True, True)
        aircraft_dimtags = gmsh.model.getEntities(2)
        orientation_report = (
            orient_discrete_surfaces_outward(aircraft_dimtags)
            if auto_orient_discrete_surfaces
            else None
        )
        aircraft_surfaces = [tag for _, tag in aircraft_dimtags]
        gmsh.model.mesh.setSize(gmsh.model.getEntities(0), surface_size)
        layer_selection = select_layer_surfaces(
            aircraft_dimtags,
            skip_layer_surface_boxes,
            min_nodes=skip_layer_min_nodes,
        )
        layer_dimtags = [(2, tag) for tag in layer_selection["layer_surface_tags"]]
        unlayered_dimtags = [(2, tag) for tag in layer_selection["skipped_surface_tags"]]

        thickness_view = add_thickness_view(layer_heights[-1], thickness_suppression_boxes)

        bl_out = gmsh.model.geo.extrudeBoundaryLayer(
            layer_dimtags,
            layer_elements,
            layer_heights,
            recombine,
            False,
            thickness_view if thickness_view is not None else -1,
        )
        gmsh.model.geo.synchronize()
        bl_volumes = [tag for dim, tag in bl_out if dim == 3]
        outer_skin = find_outer_skin_surfaces(bl_volumes, set(aircraft_dimtags))

        bounds = np.asarray(surface_report["bounds"], dtype=float)
        farfield_bounds = {
            "lo": (bounds[0] - np.asarray(padding, dtype=float)).tolist(),
            "hi": (bounds[1] + np.asarray(padding, dtype=float)).tolist(),
        }
        farfield_box = add_farfield_box(farfield_bounds, farfield_size)
        box_surfaces = list(farfield_box["surfaces"].values())
        gmsh.model.geo.synchronize()

        outer_loop = gmsh.model.geo.addSurfaceLoop(box_surfaces)
        inner_loop = gmsh.model.geo.addSurfaceLoop([tag for _, tag in outer_skin] + [tag for _, tag in unlayered_dimtags])
        farfield_volume = gmsh.model.geo.addVolume([outer_loop, inner_loop])
        gmsh.model.geo.synchronize()

        aircraft_group = gmsh.model.addPhysicalGroup(2, aircraft_surfaces)
        gmsh.model.setPhysicalName(2, aircraft_group, "aircraft")
        farfield_group = gmsh.model.addPhysicalGroup(2, box_surfaces)
        gmsh.model.setPhysicalName(2, farfield_group, "farfield")
        fluid_volumes = [farfield_volume, *bl_volumes]
        fluid_group = gmsh.model.addPhysicalGroup(3, fluid_volumes)
        gmsh.model.setPhysicalName(3, fluid_group, "fluid")

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
            "strategy": "gmsh_external_flow_with_topological_boundary_layer",
            "prepared_surface": surface_report,
            "padding": padding,
            "surface_size": surface_size,
            "farfield_size": farfield_size,
            "layer_heights": layer_heights,
            "layer_elements": layer_elements,
            "thickness_suppression_boxes": thickness_suppression_boxes,
            "skip_layer_surface_boxes": skip_layer_surface_boxes,
            "skip_layer_min_nodes": skip_layer_min_nodes,
            "layer_selection": layer_selection,
            "thickness_view": thickness_view,
            "recombine": recombine,
            "angle_deg": angle_deg,
            "curve_angle_deg": curve_angle_deg,
            "algorithm3d": algorithm3d,
            "geometry_mode": geometry_mode,
            "auto_orient_discrete_surfaces": auto_orient_discrete_surfaces,
            "orientation_report": orientation_report,
            "optimizers": optimizers,
            "aircraft_surface_count": len(aircraft_surfaces),
            "boundary_layer_volume_count": len(bl_volumes),
            "outer_skin_surface_count": len(outer_skin),
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


def orient_discrete_surfaces_outward(aircraft_dimtags: list[tuple[int, int]]) -> dict[str, object]:
    node_tags, coords, _ = gmsh.model.mesh.getNodes()
    coord_by_tag = {
        int(tag): np.asarray(coords[index * 3 : index * 3 + 3], dtype=float)
        for index, tag in enumerate(node_tags)
    }
    if not coord_by_tag:
        return {"status": "skipped_no_nodes", "reversed_surfaces": []}

    aircraft_center = np.mean(np.asarray(list(coord_by_tag.values()), dtype=float), axis=0)
    reversed_surfaces: list[dict[str, object]] = []
    surface_reports: list[dict[str, object]] = []

    for dim, tag in aircraft_dimtags:
        elem_types, _, elem_node_tags = gmsh.model.mesh.getElements(dim, tag)
        weighted_normal = np.zeros(3, dtype=float)
        weighted_center = np.zeros(3, dtype=float)
        total_area = 0.0
        triangle_count = 0

        for elem_type, flat_nodes in zip(elem_types, elem_node_tags):
            if int(elem_type) != 2:
                continue
            nodes = [int(value) for value in flat_nodes]
            for index in range(0, len(nodes), 3):
                try:
                    p0 = coord_by_tag[nodes[index]]
                    p1 = coord_by_tag[nodes[index + 1]]
                    p2 = coord_by_tag[nodes[index + 2]]
                except KeyError:
                    continue
                normal = np.cross(p1 - p0, p2 - p0)
                area2 = float(np.linalg.norm(normal))
                if area2 <= 0.0:
                    continue
                center = (p0 + p1 + p2) / 3.0
                weighted_normal += normal
                weighted_center += center * area2
                total_area += area2
                triangle_count += 1

        if total_area <= 0.0 or triangle_count == 0:
            surface_reports.append({"surface": tag, "status": "skipped_no_triangles"})
            continue

        surface_center = weighted_center / total_area
        dot = float(np.dot(weighted_normal, surface_center - aircraft_center))
        reverse = dot < 0.0
        if reverse:
            gmsh.model.mesh.reverse([(dim, tag)])
            gmsh.model.mesh.setReverse(dim, tag, True)
            reversed_surfaces.append(
                {
                    "surface": tag,
                    "dot": dot,
                    "triangles": triangle_count,
                    "center": surface_center.tolist(),
                }
            )
        surface_reports.append(
            {
                "surface": tag,
                "triangles": triangle_count,
                "dot": dot,
                "reversed": reverse,
                "center": surface_center.tolist(),
            }
        )

    return {
        "status": "ready",
        "aircraft_center": aircraft_center.tolist(),
        "surface_count": len(aircraft_dimtags),
        "reversed_count": len(reversed_surfaces),
        "reversed_surfaces": reversed_surfaces,
        "surfaces": surface_reports,
    }


def select_layer_surfaces(
    aircraft_dimtags: list[tuple[int, int]],
    skip_boxes: list[dict[str, float]],
    *,
    min_nodes: int = 0,
) -> dict[str, object]:
    reports = []
    layer_tags = []
    skipped_tags = []
    for dim, tag in aircraft_dimtags:
        center_report = surface_center(dim, tag)
        center = center_report.get("center")
        matched_box = None
        if center is not None:
            x, y, z = center
            for box in skip_boxes:
                if box["xmin"] <= x <= box["xmax"] and box["ymin"] <= y <= box["ymax"] and box["zmin"] <= z <= box["zmax"]:
                    matched_box = box
                    break
        node_count = int(center_report.get("node_count") or 0)
        matched_min_nodes = min_nodes > 0 and node_count < min_nodes
        skipped = matched_box is not None or matched_min_nodes
        if skipped:
            skipped_tags.append(tag)
        else:
            layer_tags.append(tag)
        reports.append(
            {
                "surface": tag,
                **center_report,
                "layered": not skipped,
                "matched_skip_box": matched_box,
                "matched_min_nodes": matched_min_nodes,
            }
        )
    if not layer_tags:
        raise ValueError("All aircraft surfaces were excluded from boundary-layer extrusion")
    return {
        "surface_count": len(aircraft_dimtags),
        "layer_surface_count": len(layer_tags),
        "skipped_surface_count": len(skipped_tags),
        "layer_surface_tags": layer_tags,
        "skipped_surface_tags": skipped_tags,
        "surfaces": reports,
    }


def surface_center(dim: int, tag: int) -> dict[str, object]:
    node_tags, coords, _ = gmsh.model.mesh.getNodes(dim, tag, includeBoundary=True)
    if len(node_tags) == 0:
        return {"status": "skipped_no_nodes", "center": None}
    points = np.asarray(coords, dtype=float).reshape((-1, 3))
    return {
        "status": "ready",
        "center": points.mean(axis=0).tolist(),
        "node_count": int(len(node_tags)),
    }


def find_outer_skin_surfaces(bl_volumes: list[int], aircraft_dimtags: set[tuple[int, int]]) -> list[tuple[int, int]]:
    boundary = gmsh.model.getBoundary(
        [(3, tag) for tag in bl_volumes],
        combined=False,
        oriented=False,
        recursive=False,
    )
    counts: dict[tuple[int, int], int] = {}
    for dimtag in boundary:
        counts[dimtag] = counts.get(dimtag, 0) + 1
    return [dimtag for dimtag, count in counts.items() if count == 1 and dimtag not in aircraft_dimtags]


def add_thickness_view(base_thickness: float, boxes: list[dict[str, float]]) -> int | None:
    if not boxes:
        return None
    node_tags, coords, _ = gmsh.model.mesh.getNodes()
    values: list[list[float]] = []
    for index in range(len(node_tags)):
        x = coords[index * 3]
        y = coords[index * 3 + 1]
        z = coords[index * 3 + 2]
        multiplier = 1.0
        for box in boxes:
            if (
                box["xmin"] <= x <= box["xmax"]
                and box["ymin"] <= y <= box["ymax"]
                and box["zmin"] <= z <= box["zmax"]
            ):
                multiplier = min(multiplier, box["multiplier"])
        values.append([base_thickness * multiplier])
    view = gmsh.view.add("boundary_layer_thickness")
    gmsh.view.addModelData(view, 0, gmsh.model.getCurrent(), "NodeData", node_tags.tolist(), values, 0.0, 1)
    return int(gmsh.view.getIndex(view))


def parse_float_list(value: str) -> list[float]:
    values = [float(part) for part in value.split(",") if part.strip()]
    if not values or any(item <= 0 for item in values):
        raise ValueError("Expected positive comma-separated floats")
    if values != sorted(values):
        raise ValueError("Boundary-layer heights must be cumulative and increasing")
    return values


def parse_int_list(value: str) -> list[int]:
    values = [int(part) for part in value.split(",") if part.strip()]
    if not values or any(item <= 0 for item in values):
        raise ValueError("Expected positive comma-separated integers")
    return values


def parse_optimizers(value: str) -> list[str]:
    values = [part.strip() for part in value.split(",") if part.strip()]
    if any(part.lower() in {"none", "off", "false"} for part in values):
        return []
    return values


def parse_plain_boxes(value: str) -> list[dict[str, float]]:
    boxes: list[dict[str, float]] = []
    for raw_box in value.split(";"):
        if not raw_box.strip():
            continue
        parts = [float(part) for part in raw_box.split(",")]
        if len(parts) != 6:
            raise ValueError("Layer surface skip boxes must be xmin,xmax,ymin,ymax,zmin,zmax")
        xmin, xmax, ymin, ymax, zmin, zmax = parts
        if xmin > xmax or ymin > ymax or zmin > zmax:
            raise ValueError("Skip box min values must be <= max values")
        boxes.append(
            {
                "xmin": xmin,
                "xmax": xmax,
                "ymin": ymin,
                "ymax": ymax,
                "zmin": zmin,
                "zmax": zmax,
            }
        )
    return boxes


def parse_suppression_boxes(value: str) -> list[dict[str, float]]:
    boxes: list[dict[str, float]] = []
    for raw_box in value.split(";"):
        if not raw_box.strip():
            continue
        parts = [float(part) for part in raw_box.split(",")]
        if len(parts) != 7:
            raise ValueError("Suppression boxes must be xmin,xmax,ymin,ymax,zmin,zmax,multiplier")
        xmin, xmax, ymin, ymax, zmin, zmax, multiplier = parts
        if xmin > xmax or ymin > ymax or zmin > zmax:
            raise ValueError("Suppression box min values must be <= max values")
        if multiplier <= 0.0 or multiplier > 1.0:
            raise ValueError("Suppression multiplier must be in (0, 1]")
        boxes.append(
            {
                "xmin": xmin,
                "xmax": xmax,
                "ymin": ymin,
                "ymax": ymax,
                "zmin": zmin,
                "zmax": zmax,
                "multiplier": multiplier,
            }
        )
    return boxes


if __name__ == "__main__":
    main()
