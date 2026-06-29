from __future__ import annotations

import argparse
import json
from pathlib import Path

import trimesh


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--pad", type=float, default=0.6)
    parser.add_argument("--base-cells", default="56,36,56")
    parser.add_argument("--surface-feature-angle", type=float, default=150.0)
    parser.add_argument("--feature-level", type=int, default=2)
    parser.add_argument("--surface-min-level", type=int, default=2)
    parser.add_argument("--surface-max-level", type=int, default=3)
    parser.add_argument("--n-cells-between-levels", type=int, default=3)
    parser.add_argument("--snap-tolerance", type=float, default=1.0)
    parser.add_argument("--n-smooth-patch", type=int, default=5)
    parser.add_argument("--max-local-cells", type=int, default=1_500_000)
    parser.add_argument("--max-global-cells", type=int, default=4_000_000)
    parser.add_argument("--add-layers", action="store_true")
    parser.add_argument("--n-surface-layers", type=int, default=3)
    parser.add_argument(
        "--layer-relative-sizes",
        choices=["true", "false"],
        default="true",
        help="Use snappyHexMesh relative layer thicknesses when true, absolute case-unit thicknesses when false.",
    )
    parser.add_argument("--layer-expansion-ratio", type=float, default=1.2)
    parser.add_argument("--final-layer-thickness", type=float, default=0.25)
    parser.add_argument("--min-layer-thickness", type=float, default=0.05)
    parser.add_argument("--layer-feature-angle", type=float, default=60.0)
    parser.add_argument("--n-layer-iter", type=int, default=50)
    parser.add_argument("--n-relaxed-iter", type=int, default=20)
    parser.add_argument("--max-non-ortho", type=float, default=70.0)
    parser.add_argument("--max-boundary-skewness", type=float, default=20.0)
    parser.add_argument("--max-internal-skewness", type=float, default=4.0)
    parser.add_argument(
        "--symmetry-half-domain",
        action="store_true",
        help="Clamp the background box to a symmetry plane for half-domain snappy/mirror experiments.",
    )
    parser.add_argument("--symmetry-axis", choices=["x", "y", "z"], default="y")
    parser.add_argument("--symmetry-plane", type=float, default=0.0)
    parser.add_argument(
        "--split-centerline-cap-region",
        action="store_true",
        help="Treat STL solids named aircraft and centerline_cap as separate snappy regions and avoid layers on the cap.",
    )
    parser.add_argument("--aircraft-region-name", default="aircraft")
    parser.add_argument("--centerline-cap-region-name", default="centerline_cap")
    parser.add_argument(
        "--refinement-boxes",
        default="",
        help="Semicolon-separated xmin,xmax,ymin,ymax,zmin,zmax,level boxes in case units.",
    )
    parser.add_argument("--parallel-procs", type=int, default=0)
    args = parser.parse_args()

    base_cells = parse_base_cells(args.base_cells)
    case_dir = args.case_dir
    geometry_dir = case_dir / "constant" / "geometry"
    system_dir = case_dir / "system"
    geometry_dir.mkdir(parents=True, exist_ok=True)
    system_dir.mkdir(parents=True, exist_ok=True)

    if args.split_centerline_cap_region:
        mesh = trimesh.load(args.input_stl, process=True, force="mesh")
    else:
        mesh = trimesh.load(args.input_stl, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("Expected a single triangle mesh")
    mesh.vertices *= args.scale
    mesh.merge_vertices(digits_vertex=9)
    scaled_stl = geometry_dir / "aircraft.stl"
    if args.split_centerline_cap_region:
        scale_ascii_stl_vertices(args.input_stl, scaled_stl, args.scale)
    else:
        mesh.export(scaled_stl)

    mins = mesh.bounds[0]
    maxs = mesh.bounds[1]
    lo = [float(mins[0] - args.pad), float(mins[1] - args.pad), float(mins[2] - args.pad)]
    hi = [float(maxs[0] + args.pad), float(maxs[1] + args.pad), float(maxs[2] + args.pad)]
    symmetry_surface = None
    if args.symmetry_half_domain:
        axis_index = {"x": 0, "y": 1, "z": 2}[args.symmetry_axis]
        if mins[axis_index] < args.symmetry_plane - 1e-9:
            raise ValueError(
                f"Half-domain requires the STL to lie on the positive side of "
                f"{args.symmetry_axis}={args.symmetry_plane}; bounds are {mesh.bounds.tolist()}"
            )
        lo[axis_index] = args.symmetry_plane
        symmetry_surface = f"{args.symmetry_axis}min"
    bounds = (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    inside_point = (bounds[0] + 0.05, max(bounds[2] + 0.05, args.symmetry_plane + 0.05), 0.0)

    write_text(system_dir / "blockMeshDict", render_block_mesh(bounds, base_cells, symmetry_surface=symmetry_surface))
    write_text(system_dir / "surfaceFeaturesDict", render_surface_features(args.surface_feature_angle))
    write_text(
        system_dir / "snappyHexMeshDict",
        render_snappy(
            inside_point,
            feature_level=args.feature_level,
            surface_min_level=args.surface_min_level,
            surface_max_level=args.surface_max_level,
            n_cells_between_levels=args.n_cells_between_levels,
            snap_tolerance=args.snap_tolerance,
            n_smooth_patch=args.n_smooth_patch,
            max_local_cells=args.max_local_cells,
            max_global_cells=args.max_global_cells,
            add_layers=args.add_layers,
            n_surface_layers=args.n_surface_layers,
            layer_relative_sizes=args.layer_relative_sizes == "true",
            layer_expansion_ratio=args.layer_expansion_ratio,
            final_layer_thickness=args.final_layer_thickness,
            min_layer_thickness=args.min_layer_thickness,
            layer_feature_angle=args.layer_feature_angle,
            n_layer_iter=args.n_layer_iter,
            n_relaxed_iter=args.n_relaxed_iter,
            max_non_ortho=args.max_non_ortho,
            max_boundary_skewness=args.max_boundary_skewness,
            max_internal_skewness=args.max_internal_skewness,
            refinement_boxes=parse_refinement_boxes(args.refinement_boxes),
            split_centerline_cap_region=args.split_centerline_cap_region,
            aircraft_region_name=args.aircraft_region_name,
            centerline_cap_region_name=args.centerline_cap_region_name,
        ),
    )
    write_text(system_dir / "controlDict", render_control())
    if args.parallel_procs > 1:
        write_text(system_dir / "decomposeParDict", render_decompose_par(args.parallel_procs))
    if args.symmetry_half_domain:
        write_text(system_dir / "mirrorMeshDict", render_mirror_mesh(args.symmetry_axis, args.symmetry_plane))

    report = {
        "input_stl": str(args.input_stl),
        "case_dir": str(case_dir),
        "scaled_stl": str(scaled_stl),
        "scale": args.scale,
        "pad": args.pad,
        "bounds_m": {"min": [bounds[0], bounds[2], bounds[4]], "max": [bounds[1], bounds[3], bounds[5]]},
        "inside_point": list(inside_point),
        "symmetry_half_domain": args.symmetry_half_domain,
        "symmetry_axis": args.symmetry_axis if args.symmetry_half_domain else None,
        "symmetry_plane": args.symmetry_plane if args.symmetry_half_domain else None,
        "symmetry_surface": symmetry_surface,
        "split_centerline_cap_region": args.split_centerline_cap_region,
        "aircraft_region_name": args.aircraft_region_name if args.split_centerline_cap_region else None,
        "centerline_cap_region_name": args.centerline_cap_region_name if args.split_centerline_cap_region else None,
        "parallel_procs": args.parallel_procs if args.parallel_procs > 1 else None,
        "base_cells": list(base_cells),
        "surface_feature_angle": args.surface_feature_angle,
        "feature_level": args.feature_level,
        "surface_min_level": args.surface_min_level,
        "surface_max_level": args.surface_max_level,
        "n_cells_between_levels": args.n_cells_between_levels,
        "snap_tolerance": args.snap_tolerance,
        "n_smooth_patch": args.n_smooth_patch,
        "surface_faces": int(len(mesh.faces)),
        "surface_vertices": int(len(mesh.vertices)),
        "watertight": bool(mesh.is_watertight),
        "add_layers": args.add_layers,
        "n_surface_layers": args.n_surface_layers,
        "layer_relative_sizes": args.layer_relative_sizes == "true",
        "layer_expansion_ratio": args.layer_expansion_ratio,
        "final_layer_thickness": args.final_layer_thickness,
        "min_layer_thickness": args.min_layer_thickness,
        "layer_feature_angle": args.layer_feature_angle,
        "n_layer_iter": args.n_layer_iter,
        "n_relaxed_iter": args.n_relaxed_iter,
        "max_non_ortho": args.max_non_ortho,
        "max_boundary_skewness": args.max_boundary_skewness,
        "max_internal_skewness": args.max_internal_skewness,
        "refinement_boxes": parse_refinement_boxes(args.refinement_boxes),
    }
    write_text(case_dir / "case_report.json", json.dumps(report, indent=2, sort_keys=True))
    print(json.dumps(report, indent=2, sort_keys=True))


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def scale_ascii_stl_vertices(input_stl: Path, output_stl: Path, scale: float) -> None:
    output_stl.parent.mkdir(parents=True, exist_ok=True)
    with input_stl.open("r", encoding="ascii", errors="strict") as src, output_stl.open(
        "w",
        encoding="ascii",
        newline="\n",
    ) as dst:
        for line in src:
            stripped = line.strip()
            if stripped.startswith("vertex "):
                parts = stripped.split()
                if len(parts) == 4:
                    x, y, z = (float(parts[1]) * scale, float(parts[2]) * scale, float(parts[3]) * scale)
                    indent = line[: len(line) - len(line.lstrip())]
                    dst.write(f"{indent}vertex {x:.9e} {y:.9e} {z:.9e}\n")
                    continue
            dst.write(line)


def parse_base_cells(value: str) -> tuple[int, int, int]:
    parts = [int(part) for part in value.split(",")]
    if len(parts) != 3 or any(part <= 0 for part in parts):
        raise argparse.ArgumentTypeError("--base-cells must be three positive integers")
    return (parts[0], parts[1], parts[2])


def parse_refinement_boxes(value: str) -> list[dict[str, float | int]]:
    boxes: list[dict[str, float | int]] = []
    if not value.strip():
        return boxes
    for index, item in enumerate(value.split(";")):
        if not item.strip():
            continue
        parts = [float(part) for part in item.split(",") if part.strip()]
        if len(parts) != 7:
            raise argparse.ArgumentTypeError(
                "--refinement-boxes entries must be xmin,xmax,ymin,ymax,zmin,zmax,level"
            )
        xmin, xmax, ymin, ymax, zmin, zmax, level = parts
        if not (xmin < xmax and ymin < ymax and zmin < zmax and level >= 0):
            raise argparse.ArgumentTypeError(f"Invalid refinement box at index {index}")
        boxes.append(
            {
                "name": f"refinementBox{index}",
                "xmin": xmin,
                "xmax": xmax,
                "ymin": ymin,
                "ymax": ymax,
                "zmin": zmin,
                "zmax": zmax,
                "level": int(level),
            }
        )
    return boxes


def foam_header(class_name: str, object_name: str) -> str:
    return f"""/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM
   \\\\    /   O peration     |
    \\\\  /    A nd           |
     \\\\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       {class_name};
    object      {object_name};
}}

"""


def render_block_mesh(
    bounds: tuple[float, float, float, float, float, float],
    cells: tuple[int, int, int],
    *,
    symmetry_surface: str | None = None,
) -> str:
    x0, x1, y0, y1, z0, z1 = bounds
    nx, ny, nz = cells
    patch_faces = {
        "ymax": "(3 7 6 2)",
        "xmin": "(0 4 7 3)",
        "xmax": "(2 6 5 1)",
        "ymin": "(1 5 4 0)",
        "zmin": "(0 3 2 1)",
        "zmax": "(4 5 6 7)",
    }
    farfield_faces = "\n".join(
        f"            {face}" for name, face in patch_faces.items() if name != symmetry_surface
    )
    symmetry_patch = ""
    if symmetry_surface is not None:
        symmetry_patch = f"""
    symmetry
    {{
        type symmetryPlane;
        faces
        (
            {patch_faces[symmetry_surface]}
        );
    }}
"""
    return (
        foam_header("dictionary", "blockMeshDict")
        + f"""scale 1;

vertices
(
    ({x0} {y0} {z0})
    ({x1} {y0} {z0})
    ({x1} {y1} {z0})
    ({x0} {y1} {z0})
    ({x0} {y0} {z1})
    ({x1} {y0} {z1})
    ({x1} {y1} {z1})
    ({x0} {y1} {z1})
);

blocks
(
    hex (0 1 2 3 4 5 6 7) ({nx} {ny} {nz}) simpleGrading (1 1 1)
);

boundary
(
    farfield
    {{
        type patch;
        faces
        (
{farfield_faces}
        );
    }}
{symmetry_patch}
);

// ************************************************************************* //
"""
    )


def render_mirror_mesh(axis: str, plane: float) -> str:
    normal = {"x": "(1 0 0)", "y": "(0 1 0)", "z": "(0 0 1)"}[axis]
    if axis == "x":
        point = f"({plane} 0 0)"
    elif axis == "y":
        point = f"(0 {plane} 0)"
    else:
        point = f"(0 0 {plane})"
    return (
        foam_header("dictionary", "mirrorMeshDict")
        + f"""planeType pointAndNormal;

pointAndNormalDict
{{
    point  {point};
    normal {normal};
}}

planeTolerance 1e-6;

// ************************************************************************* //
"""
    )


def render_decompose_par(subdomains: int) -> str:
    if subdomains <= 1:
        raise ValueError("decomposeParDict requires more than one subdomain")
    return (
        foam_header("dictionary", "decomposeParDict")
        + f"""numberOfSubdomains {subdomains};

method scotch;

// ************************************************************************* //
"""
    )


def render_surface_features(included_angle: float) -> str:
    return (
        foam_header("dictionary", "surfaceFeaturesDict")
        + f"""surfaces ("aircraft.stl");
includedAngle {included_angle};
subsetFeatures
{{
    nonManifoldEdges yes;
    openEdges yes;
}}

// ************************************************************************* //
"""
    )


def render_snappy(
    inside_point: tuple[float, float, float],
    *,
    feature_level: int,
    surface_min_level: int,
    surface_max_level: int,
    n_cells_between_levels: int,
    snap_tolerance: float,
    n_smooth_patch: int,
    max_local_cells: int,
    max_global_cells: int,
    add_layers: bool,
    n_surface_layers: int,
    layer_relative_sizes: bool,
    layer_expansion_ratio: float,
    final_layer_thickness: float,
    min_layer_thickness: float,
    layer_feature_angle: float,
    n_layer_iter: int,
    n_relaxed_iter: int,
    max_non_ortho: float,
    max_boundary_skewness: float,
    max_internal_skewness: float,
    refinement_boxes: list[dict[str, float | int]],
    split_centerline_cap_region: bool,
    aircraft_region_name: str,
    centerline_cap_region_name: str,
) -> str:
    px, py, pz = inside_point
    add_layers_token = "true" if add_layers else "false"
    relative_sizes_token = "true" if layer_relative_sizes else "false"
    layer_patch_controls = (
        f"""    layers
    {{
        {aircraft_region_name}
        {{
            nSurfaceLayers {n_surface_layers};
        }}
    }}
"""
        if add_layers
        else "    layers {}\n"
    )
    geometry_regions = ""
    refinement_surface_regions = ""
    if split_centerline_cap_region:
        geometry_regions = f"""
        regions
        {{
            {aircraft_region_name}
            {{
                name {aircraft_region_name};
            }}
            {centerline_cap_region_name}
            {{
                name {centerline_cap_region_name};
            }}
        }}
"""
        refinement_surface_regions = f"""
            regions
            {{
                {aircraft_region_name}
                {{
                    level ({surface_min_level} {surface_max_level});
                    patchInfo
                    {{
                        type wall;
                    }}
                }}
                {centerline_cap_region_name}
                {{
                    level (0 0);
                    patchInfo
                    {{
                        type wall;
                    }}
                }}
            }}
"""
    refinement_geometry = "\n".join(
        f"""    {box["name"]}
    {{
        type searchableBox;
        min ({box["xmin"]} {box["ymin"]} {box["zmin"]});
        max ({box["xmax"]} {box["ymax"]} {box["zmax"]});
    }}"""
        for box in refinement_boxes
    )
    refinement_regions = (
        "\n".join(
            f"""        {box["name"]}
        {{
            mode inside;
            levels ((1e15 {box["level"]}));
        }}"""
            for box in refinement_boxes
        )
        if refinement_boxes
        else ""
    )
    return (
        foam_header("dictionary", "snappyHexMeshDict")
        + f"""castellatedMesh true;
snap            true;
addLayers       {add_layers_token};

geometry
{{
    aircraft
    {{
        type triSurface;
        file "aircraft.stl";
{geometry_regions}
    }}
{refinement_geometry}
}};

castellatedMeshControls
{{
    maxLocalCells {max_local_cells};
    maxGlobalCells {max_global_cells};
    minRefinementCells 0;
    nCellsBetweenLevels {n_cells_between_levels};

    features
    (
        {{
            file "aircraft.eMesh";
            level {feature_level};
        }}
    );

    refinementSurfaces
    {{
        aircraft
        {{
            level ({surface_min_level} {surface_max_level});
            patchInfo
            {{
                type wall;
            }}
{refinement_surface_regions}
        }}
    }}

    resolveFeatureAngle 30;
    refinementRegions
    {{
{refinement_regions}
    }}
    insidePoint ({px} {py} {pz});
    allowFreeStandingZoneFaces true;
}}

snapControls
{{
    nSmoothPatch {n_smooth_patch};
    tolerance {snap_tolerance};
    nSolveIter 120;
    nRelaxIter 8;
    nFeatureSnapIter 20;
    implicitFeatureSnap true;
    explicitFeatureSnap true;
    multiRegionFeatureSnap true;
}}

addLayersControls
{{
    relativeSizes {relative_sizes_token};
{layer_patch_controls}
    expansionRatio {layer_expansion_ratio};
    finalLayerThickness {final_layer_thickness};
    minThickness {min_layer_thickness};
    nGrow 0;
    featureAngle {layer_feature_angle};
    nRelaxIter 5;
    nSmoothSurfaceNormals 1;
    nSmoothNormals 3;
    nSmoothThickness 10;
    maxFaceThicknessRatio 0.5;
    maxThicknessToMedialRatio 0.3;
    minMedialAxisAngle 90;
    nBufferCellsNoExtrude 0;
    nLayerIter {n_layer_iter};
    nRelaxedIter {n_relaxed_iter};
}}

meshQualityControls
{{
    maxNonOrtho {max_non_ortho};
    maxBoundarySkewness {max_boundary_skewness};
    maxInternalSkewness {max_internal_skewness};
    maxConcave 80;
    minVol 1e-16;
    minTetQuality 1e-30;
    minArea -1;
    minTwist 0.02;
    minDeterminant 0.001;
    minFaceWeight 0.02;
    minVolRatio 0.01;
    minTriangleTwist -1;
    nSmoothScale 4;
    errorReduction 0.75;

    relaxed
    {{
        maxNonOrtho {max(max_non_ortho, 75.0)};
        maxBoundarySkewness {max(max_boundary_skewness, 30.0)};
        maxInternalSkewness {max(max_internal_skewness, 6.0)};
        minVol 1e-16;
    }}
}}

debug 0;
mergeTolerance 1e-6;

// ************************************************************************* //
"""
    )


def render_control() -> str:
    return (
        foam_header("dictionary", "controlDict")
        + """application     simpleFoam;
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
"""
    )


if __name__ == "__main__":
    main()
