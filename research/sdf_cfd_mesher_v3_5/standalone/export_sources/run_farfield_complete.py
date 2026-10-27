#!/usr/bin/env python3
"""Assemble one frozen prism shell with a new farfield core and run Foundation 13."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import pathlib
import re
import resource
import shlex
import subprocess
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
V3 = HERE.parent
PRIOR_COMPLETE = V3 / "held_out_geometry_cohort/run_complete_stage.py"
PRIOR_MATH = V3 / "refined_gmsh_layer_realization/run_gmsh_layer.py"
FOUNDATION = V3 / "hxt_complete_mesh_preserved_core/bin/foundationCompleteAudit"


def load_module(name: str, path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


old = load_module("v3_farfield_old_complete", PRIOR_COMPLETE)
prism_math = load_module("v3_farfield_prism_math", PRIOR_MATH)


def sha(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: pathlib.Path, value: dict) -> str:
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n", encoding="ascii")
    return sha(path)


def cyclic(a, b) -> bool:
    a = tuple(map(int, a)); b = tuple(map(int, b))
    return any(a == b[i:] + b[:i] for i in range(len(b)))


def opposed(a, b) -> bool:
    return cyclic(a, tuple(reversed(tuple(b))))


def verify_core_authority(
    fixture: str, replay: str, layer_dir: pathlib.Path, core_dir: pathlib.Path, policy_path: pathlib.Path
) -> dict:
    policy_sha = sha(policy_path)
    dresult_path = core_dir / "delaunay_validation.json"
    nresult_path = core_dir / "netgen_validation.json"
    dresult = json.loads(dresult_path.read_text(encoding="utf-8"))
    nresult = json.loads(nresult_path.read_text(encoding="utf-8"))
    for result in (dresult, nresult):
        if not result["pass"] or result["fixture"] != fixture or result["replay"] != replay:
            raise RuntimeError("CoreValidationAuthorityMismatch")
        if result["policy_sha256"] != policy_sha:
            raise RuntimeError("CorePolicyIdentityMismatch")
        if not result["validation"]["oriented_boundary_contract"]["pass"]:
            raise RuntimeError("CoreOrientedBoundaryContractFailed")
        if not result["validation"]["persistent_curve_contract"]["pass"]:
            raise RuntimeError("CorePersistentCurveContractFailed")
    for relative, expected in nresult["frozen_inputs"]["files"].items():
        path = V3 / relative
        if sha(path) != expected:
            raise RuntimeError(f"FrozenNearBodyIdentityMismatch:{relative}")
    if layer_dir.resolve() != (V3 / next(
        relative for relative in nresult["frozen_inputs"]["files"] if relative.endswith("actual_shadow.npy")
    )).resolve().parent:
        raise RuntimeError("FrozenLayerDirectoryMismatch")
    for key, expected in nresult["hashes"].items():
        if key == "canonical_combined":
            combined = hashlib.sha256(
                bytes.fromhex(nresult["hashes"]["canonical_coordinates"])
                + bytes.fromhex(nresult["hashes"]["canonical_connectivity"])
            ).hexdigest()
            if combined != expected:
                raise RuntimeError("NetgenCanonicalCombinedIdentityMismatch")
            continue
        filename = f"netgen_{key}.npy"
        if sha(core_dir / filename) != expected:
            raise RuntimeError(f"NetgenCoreArrayIdentityMismatch:{filename}")
    if sha(core_dir / "farfield_points.npy") != dresult["farfield"]["points_sha256"]:
        raise RuntimeError("FarfieldCoordinateAuthorityMismatch")
    if sha(core_dir / "farfield_faces.npy") != dresult["farfield"]["faces_sha256"]:
        raise RuntimeError("FarfieldConnectivityAuthorityMismatch")
    return {
        "policy_sha256": policy_sha,
        "delaunay_validation_sha256": sha(dresult_path),
        "netgen_validation_sha256": sha(nresult_path),
        "frozen_inputs": nresult["frozen_inputs"],
        "farfield_identity_sha256": dresult["farfield"]["identity_sha256"],
    }


def mixed_targeted_incidence(prisms, tets, interface, farfield) -> dict:
    interface_lookup = {tuple(sorted(map(int, row))): i for i, row in enumerate(interface)}
    farfield_lookup = {tuple(sorted(map(int, row))): i for i, row in enumerate(farfield)}
    prism_count = np.zeros(len(interface), dtype=np.int8)
    prism_winding = np.zeros(len(interface), dtype=np.int8)
    tet_interface_count = np.zeros(len(interface), dtype=np.int8)
    tet_interface_winding = np.zeros(len(interface), dtype=np.int8)
    tet_far_count = np.zeros(len(farfield), dtype=np.int8)
    tet_far_winding = np.zeros(len(farfield), dtype=np.int8)
    for cell in prisms:
        for pattern in ((0, 2, 1), (3, 4, 5)):
            row = tuple(int(cell[i]) for i in pattern)
            face_id = interface_lookup.get(tuple(sorted(row)))
            if face_id is not None:
                prism_count[face_id] += 1
                prism_winding[face_id] += int(cyclic(row, interface[face_id]))
    for cell in tets:
        for pattern in ((0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)):
            row = tuple(int(cell[i]) for i in pattern)
            key = tuple(sorted(row))
            face_id = interface_lookup.get(key)
            if face_id is not None:
                tet_interface_count[face_id] += 1
                tet_interface_winding[face_id] += int(opposed(row, interface[face_id]))
            face_id = farfield_lookup.get(key)
            if face_id is not None:
                tet_far_count[face_id] += 1
                tet_far_winding[face_id] += int(cyclic(row, farfield[face_id]))
    result = {
        "interface_prism_owner_errors": int(np.count_nonzero(prism_count != 1)),
        "interface_prism_winding_errors": int(np.count_nonzero(prism_winding != 1)),
        "interface_tet_owner_errors": int(np.count_nonzero(tet_interface_count != 1)),
        "interface_tet_winding_errors": int(np.count_nonzero(tet_interface_winding != 1)),
        "farfield_tet_owner_errors": int(np.count_nonzero(tet_far_count != 1)),
        "farfield_tet_winding_errors": int(np.count_nonzero(tet_far_winding != 1)),
    }
    result["pass"] = all(value == 0 for key, value in result.items() if key != "pass")
    return result


def patch_range(boundary: pathlib.Path, name: str) -> tuple[int, int]:
    text = boundary.read_text(encoding="utf-8")
    block = re.search(rf"(?ms)^\s*{re.escape(name)}\s*\{{(.*?)^\s*\}}", text)
    if block is None:
        raise RuntimeError(f"MissingOpenFoamPatch:{name}")
    count = int(re.search(r"nFaces\s+(\d+)\s*;", block.group(1)).group(1))
    start = int(re.search(r"startFace\s+(\d+)\s*;", block.group(1)).group(1))
    return start, count


def canonical_rows(rows: np.ndarray) -> np.ndarray:
    rows = np.sort(np.asarray(rows, dtype=np.int64), axis=1)
    if not len(rows):
        return rows
    order = np.lexsort(tuple(rows[:, i] for i in reversed(range(rows.shape[1]))))
    return rows[order]


def conversion_fidelity(audit_dir, case, coordinates, prisms, tets, wall, farfield, interface) -> dict:
    foam_points = np.fromfile(audit_dir / "points.f64", dtype=np.float64).reshape(-1, 3)
    mixed_order = np.lexsort((coordinates[:, 2], coordinates[:, 1], coordinates[:, 0]))
    foam_order = np.lexsort((foam_points[:, 2], foam_points[:, 1], foam_points[:, 0]))
    points_exact = len(foam_points) == len(coordinates) and np.array_equal(
        foam_points[foam_order], coordinates[mixed_order]
    )
    if not points_exact:
        raise RuntimeError("OpenFoamPointConversionMismatch")
    foam_to_mixed = np.empty(len(foam_points), dtype=np.int64)
    foam_to_mixed[foam_order] = mixed_order

    offsets = np.fromfile(audit_dir / "cell_offsets.u64", dtype=np.uint64)
    cell_points = np.fromfile(audit_dir / "cell_points.i32", dtype=np.int32)
    sizes = np.diff(offsets)
    expected_groups = (
        len(sizes) == len(prisms) + len(tets)
        and np.all(sizes[: len(prisms)] == 6)
        and np.all(sizes[len(prisms) :] == 4)
    )
    cells_exact = False
    if expected_groups:
        foam_prisms = foam_to_mixed[cell_points[: 6 * len(prisms)].reshape(-1, 6)]
        foam_tets = foam_to_mixed[cell_points[6 * len(prisms) :].reshape(-1, 4)]
        cells_exact = np.array_equal(canonical_rows(foam_prisms), canonical_rows(prisms)) and np.array_equal(
            canonical_rows(foam_tets), canonical_rows(tets)
        )

    face_offsets = np.fromfile(audit_dir / "face_offsets.u64", dtype=np.uint64)
    face_points = np.fromfile(audit_dir / "face_points.i32", dtype=np.int32)
    owner = np.fromfile(audit_dir / "owner.i32", dtype=np.int32)
    neighbour = np.fromfile(audit_dir / "neighbour.i32", dtype=np.int32)
    face_sizes = np.diff(face_offsets)

    def patch_faces(name: str, expected: np.ndarray) -> tuple[bool, int, int]:
        start, count = patch_range(case / "constant/polyMesh/boundary", name)
        if count != len(expected) or not np.all(face_sizes[start : start + count] == 3):
            return False, start, count
        starts = face_offsets[start : start + count].astype(np.int64)
        rows = np.column_stack((face_points[starts], face_points[starts + 1], face_points[starts + 2]))
        rows = foam_to_mixed[rows]
        exact = np.array_equal(canonical_rows(rows), canonical_rows(expected))
        tri = foam_points[np.column_stack((face_points[starts], face_points[starts + 1], face_points[starts + 2]))]
        centroids = np.fromfile(audit_dir / "cell_centres.f64", dtype=np.float64).reshape(-1, 3)
        normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        outward = np.einsum("ij,ij->i", normal, tri.mean(axis=1) - centroids[owner[start : start + count]]) > 0
        return exact and bool(np.all(outward)), start, count

    wall_exact, wall_start, wall_count = patch_faces("wall", wall)
    far_exact, far_start, far_count = patch_faces("farfield", farfield)
    internal_ids = np.flatnonzero(face_sizes[: len(neighbour)] == 3)
    starts = face_offsets[internal_ids].astype(np.int64)
    internal_triangles = foam_to_mixed[
        np.column_stack((face_points[starts], face_points[starts + 1], face_points[starts + 2]))
    ]
    internal_keys = canonical_rows(internal_triangles)
    interface_keys = canonical_rows(interface)
    dtype = np.dtype((np.void, internal_keys.dtype.itemsize * 3))
    common = np.intersect1d(
        np.ascontiguousarray(internal_keys).view(dtype).reshape(-1),
        np.ascontiguousarray(interface_keys).view(dtype).reshape(-1),
    )
    interface_internal_exact = len(common) == len(interface)
    volumes = np.fromfile(audit_dir / "cell_volumes.f64", dtype=np.float64)
    result = {
        "pass": bool(points_exact and cells_exact and wall_exact and far_exact and interface_internal_exact),
        "points_binary64_exact": bool(points_exact),
        "canonical_cells_exact": bool(cells_exact),
        "cell_group_order_and_arity_exact": bool(expected_groups),
        "wall_patch_exact_and_outward": bool(wall_exact),
        "farfield_patch_exact_and_outward": bool(far_exact),
        "interface_faces_internal_exact": bool(interface_internal_exact),
        "wall_patch": {"start": wall_start, "count": wall_count},
        "farfield_patch": {"start": far_start, "count": far_count},
        "internal_faces": int(len(neighbour)),
        "cells": int(len(sizes)),
        "points": int(len(foam_points)),
        "positive_cell_failures": int(np.count_nonzero(volumes <= 0)),
        "complete_volume_m3": float(np.sum(volumes)),
    }
    return result


def timed(command: str, cwd: pathlib.Path, name: str, timeout: int = 1200) -> dict:
    stdout = cwd / f"{name}.stdout"
    stderr = cwd / f"{name}.stderr"
    resource_file = cwd / f"{name}.resource.txt"
    started = time.perf_counter()
    with stdout.open("wb") as out_stream, stderr.open("wb") as err_stream:
        process = subprocess.run(
            ["/usr/bin/time", "-v", "-o", str(resource_file.resolve()), "bash", "-lc", command],
            cwd=cwd,
            stdout=out_stream,
            stderr=err_stream,
            timeout=timeout,
        )
    text = resource_file.read_text(errors="replace") if resource_file.exists() else ""
    match = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)", text)
    return {
        "exit_code": process.returncode,
        "runtime_seconds": time.perf_counter() - started,
        "peak_rss_bytes": int(match.group(1)) * 1024 if match else None,
        "stdout_sha256": sha(stdout),
        "stderr_sha256": sha(stderr),
    }


def write_msh(path: pathlib.Path, coordinates, wall, farfield, prisms, tets) -> dict:
    with path.open("w", encoding="ascii", buffering=4 << 20, newline="\n") as stream:
        stream.write('$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$PhysicalNames\n3\n2 2 "wall"\n2 3 "farfield"\n3 1 "fluid"\n$EndPhysicalNames\n')
        stream.write(f"$Nodes\n{len(coordinates)}\n")
        for i, (x, y, z) in enumerate(coordinates, 1):
            stream.write(f"{i} {x:.17g} {y:.17g} {z:.17g}\n")
        stream.write("$EndNodes\n")
        total = len(wall) + len(farfield) + len(prisms) + len(tets)
        stream.write(f"$Elements\n{total}\n")
        tag = 1
        for row in wall:
            stream.write(f"{tag} 2 2 2 1 " + " ".join(str(int(v) + 1) for v in row) + "\n")
            tag += 1
        for row in farfield:
            stream.write(f"{tag} 2 2 3 2 " + " ".join(str(int(v) + 1) for v in row) + "\n")
            tag += 1
        for row in prisms:
            stream.write(f"{tag} 6 2 1 1 " + " ".join(str(int(v) + 1) for v in row) + "\n")
            tag += 1
        for row in tets:
            stream.write(f"{tag} 4 2 1 1 " + " ".join(str(int(v) + 1) for v in row) + "\n")
            tag += 1
        stream.write("$EndElements\n")
    return {"sha256": sha(path), "bytes": path.stat().st_size, "nodes": len(coordinates), "elements": total}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--replay", default="A")
    parser.add_argument("--layer-dir", required=True)
    parser.add_argument("--core-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--policy", required=True)
    args = parser.parse_args()

    layer_dir = pathlib.Path(args.layer_dir)
    core_dir = pathlib.Path(args.core_dir)
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    policy_path = pathlib.Path(args.policy)
    policy = json.loads(policy_path.read_text())
    started = time.perf_counter()
    core_authority = verify_core_authority(
        args.fixture, args.replay, layer_dir, core_dir, policy_path
    )

    shadow = np.load(layer_dir / "actual_shadow.npy")
    wall = np.load(layer_dir / "wall_faces.npy")
    prisms = np.load(layer_dir / "canonical_prisms.npy")
    face_count = len(wall)
    wall_vertex_count = shadow.shape[1]
    node_tags = np.load(core_dir / "netgen_node_tags.npy")
    core_coordinates = np.load(core_dir / "netgen_coordinates.npy")
    core_tet_tags = np.load(core_dir / "netgen_connectivity_tags.npy")
    interface_tags = np.load(core_dir / "netgen_interface_tags.npy")
    farfield_tags = np.load(core_dir / "netgen_farfield_tags.npy")
    farfield_faces_local = np.load(core_dir / "farfield_faces.npy")

    tag_position = {int(tag): i for i, tag in enumerate(node_tags)}
    core_to_mixed = np.full(len(node_tags), -1, dtype=np.int64)
    for source, tag in enumerate(interface_tags):
        core_to_mixed[tag_position[int(tag)]] = 15 * wall_vertex_count + source
    mixed_coordinates = list(shadow.reshape(-1, 3))
    next_index = len(mixed_coordinates)
    for i, point in enumerate(core_coordinates):
        if core_to_mixed[i] < 0:
            core_to_mixed[i] = next_index
            mixed_coordinates.append(point)
            next_index += 1
    mixed_coordinates = np.asarray(mixed_coordinates, dtype=np.float64)
    tet_positions = np.asarray([[tag_position[int(v)] for v in row] for row in core_tet_tags], dtype=np.int64)
    mixed_tets = core_to_mixed[tet_positions]
    farfield = np.asarray(
        [[core_to_mixed[tag_position[int(farfield_tags[int(v)])]] for v in row] for row in farfield_faces_local],
        dtype=np.int64,
    )
    interface = 15 * wall_vertex_count + wall
    np.save(output / "mixed_coordinates.npy", mixed_coordinates, allow_pickle=False)
    np.save(output / "mixed_prisms.npy", prisms, allow_pickle=False)
    np.save(output / "mixed_tets.npy", mixed_tets, allow_pickle=False)
    np.save(output / "farfield_faces.npy", farfield, allow_pickle=False)
    incidence = mixed_targeted_incidence(prisms, mixed_tets, interface, farfield)
    angles = prism_math.surface_minimum_angles(shadow[0], wall)
    prism_evaluation = prism_math.evaluate_prisms(shadow, wall, angles)
    tet_volumes = old.tet_volume(mixed_coordinates[mixed_tets])
    prism_volume = float(np.sum(prism_evaluation["volume"]))
    core_volume = float(np.sum(tet_volumes))
    total_volume = prism_volume + core_volume
    cfg = policy["fixtures"][args.fixture]
    scale = float(cfg["characteristic_scale_m"])
    half = float(cfg["farfield_half_extent_m"])
    if half != 2.5 * scale:
        raise RuntimeError("PolicyFarfieldExtentMismatch")
    expected = (2 * half) ** 3 - old.signed_surface_volume(shadow[0], wall)
    tolerance = max(4096 * np.finfo(float).eps * max(expected, float(np.sum(np.abs(prism_evaluation["volume"]))) + float(np.sum(np.abs(tet_volumes)))), 1e-9)
    if not incidence["pass"] or np.any(tet_volumes <= 0) or prism_evaluation["summary"]["positive_volume_failures"] or abs(total_volume - expected) > tolerance:
        raise RuntimeError("MixedAssemblyHardGateFailed")
    assembly = {
        "schema": "V3FarfieldPolicyMixedAssembly/v1",
        "fixture": args.fixture,
        "replay": args.replay,
        "pass": True,
        "policy_sha256": sha(policy_path),
        "core_authority": core_authority,
        "counts": {
            "points": len(mixed_coordinates),
            "prisms": len(prisms),
            "tetrahedra": len(mixed_tets),
            "cells": len(prisms) + len(mixed_tets),
            "wall_faces": len(wall),
            "interface_faces": face_count,
            "farfield_faces": len(farfield),
        },
        "topology": {
            "targeted_incidence": incidence,
            "interface_one_prism_one_tet": incidence["interface_prism_owner_errors"] == 0 and incidence["interface_tet_owner_errors"] == 0,
            "interface_opposed_winding": incidence["interface_prism_winding_errors"] == 0 and incidence["interface_tet_winding_errors"] == 0,
            "farfield_one_tet_owner_outward": incidence["farfield_tet_owner_errors"] == 0 and incidence["farfield_tet_winding_errors"] == 0,
            "remaining_interface_boundary": 0,
            "external_patches": ["wall", "farfield"],
        },
        "geometry": {
            "prism_volume_m3": prism_volume,
            "core_volume_m3": core_volume,
            "complete_volume_m3": total_volume,
            "expected_volume_m3": expected,
            "volume_error_m3": abs(total_volume - expected),
            "volume_tolerance_m3": tolerance,
            "nonpositive_prisms": prism_evaluation["summary"]["positive_volume_failures"],
            "nonpositive_tets": int(np.sum(tet_volumes <= 0)),
        },
        "hashes": {
            "coordinates": sha(output / "mixed_coordinates.npy"),
            "prisms": sha(output / "mixed_prisms.npy"),
            "tets": sha(output / "mixed_tets.npy"),
            "farfield_faces": sha(output / "farfield_faces.npy"),
        },
        "prism_determinant": prism_evaluation["summary"]["determinant"],
    }
    write_json(output / "assembly.json", assembly)

    transfer = write_msh(output / "complete_transfer.msh", mixed_coordinates, wall, farfield, prisms, mixed_tets)
    case = output / "openfoam_case"
    old.create_case(case)
    convert = timed(
        f"source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; gmshToFoam {shlex.quote(str((output / 'complete_transfer.msh').resolve()))} -case {shlex.quote(str(case.resolve()))}",
        output,
        "gmshToFoam",
    )
    if convert["exit_code"] != 0:
        raise RuntimeError("gmshToFoamFailed")
    audit_dir = output / "foundation_audit"
    audit_dir.mkdir()
    audit = timed(
        f"source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; {shlex.quote(str(FOUNDATION.resolve()))} -case {shlex.quote(str(case.resolve()))} -outputDir {shlex.quote(str(audit_dir.resolve()))}",
        output,
        "foundationAudit",
    )
    check = timed(
        f"source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; checkMesh -constant -allTopology -allGeometry -case {shlex.quote(str(case.resolve()))}",
        output,
        "checkMesh",
    )
    if audit["exit_code"] != 0 or not (audit_dir / "foundation_audit_summary.json").exists():
        raise RuntimeError("FoundationAuditFailed")
    conversion = conversion_fidelity(
        audit_dir, case, mixed_coordinates, prisms, mixed_tets, wall, farfield, interface
    )
    if not conversion["pass"]:
        raise RuntimeError("OpenFoamConversionMismatch")
    check_text = (output / "checkMesh.stdout").read_text(errors="replace")
    summary = json.loads((audit_dir / "foundation_audit_summary.json").read_text())
    expected_cells = len(prisms) + len(mixed_tets)
    audit_counts = summary.get("counts", summary)
    count_gate = (
        audit_counts.get("cells") == expected_cells
        and audit_counts.get("points") == len(mixed_coordinates)
        and conversion["positive_cell_failures"] == 0
    )
    mesh_ok = "Mesh OK." in check_text and check["exit_code"] == 0 and count_gate
    failed = [line.strip() for line in check_text.splitlines() if "Failed" in line or line.strip().startswith("***")]
    nonorth = re.search(r"Mesh non-orthogonality Max:\s*([0-9.eE+-]+).*average:\s*([0-9.eE+-]+)", check_text)
    skew = re.search(r"Max skewness =\s*([0-9.eE+-]+)", check_text)
    poly = case / "constant/polyMesh"
    files = {path.name: sha(path) for path in poly.iterdir() if path.is_file()}
    poly_hash = hashlib.sha256("".join(f"{key}:{files[key]}\n" for key in sorted(files)).encode()).hexdigest()
    result = {
        "schema": "V3FarfieldPolicyFoundation/v1",
        "fixture": args.fixture,
        "replay": args.replay,
        "policy_sha256": sha(policy_path),
        "literal_mesh_ok": mesh_ok,
        "checkMesh_exit_code": check["exit_code"],
        "failed_lines": failed,
        "counts": audit_counts,
        "hard_gate_count_match": count_gate,
        "conversion_fidelity": conversion,
        "maximum_nonorthogonality": float(nonorth.group(1)) if nonorth else None,
        "average_nonorthogonality": float(nonorth.group(2)) if nonorth else None,
        "maximum_skewness_checkMesh": float(skew.group(1)) if skew else None,
        "polyMesh_sha256": poly_hash,
        "polyMesh_files": files,
        "transfer": transfer,
        "gmshToFoam": convert,
        "foundation_audit": audit,
        "checkMesh": check,
        "runtime_seconds": time.perf_counter() - started,
        "peak_rss_bytes": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024),
    }
    write_json(output / "foundation_result.json", result)
    print(json.dumps({"assembly": assembly, "foundation": result}, sort_keys=True))
    return 0 if mesh_ok else 4


if __name__ == "__main__":
    raise SystemExit(main())
