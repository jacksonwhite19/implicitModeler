#!/usr/bin/env python3
"""Foundation 13-equivalent prism centres and internal side-face weights.

This module mirrors face::areaAndCentre, primitiveMesh::makeCellCentresAndVols,
and meshCheck::faceWeights from the installed OpenFOAM Foundation 13 source.
It is geometry-only.  It creates no mesh topology.
"""

from __future__ import annotations

import numpy as np


def face_area_centre_tri(p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    area = 0.5 * np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    centre = (p[:, 0] + p[:, 1] + p[:, 2]) / 3.0
    return area, centre


def face_area_centre_quad(p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized Foundation face::areaAndCentre for four-point faces."""
    pavg = np.mean(p, axis=1)
    pnext = np.roll(p, -1, axis=1)
    a = np.cross(pnext - p, pavg[:, None, :] - p)
    suma = np.sum(a, axis=1)
    mag = np.linalg.norm(suma, axis=1)
    ahat = suma / np.maximum(mag[:, None], np.finfo(np.float64).tiny)
    an = np.einsum("nij,nj->ni", a, ahat)
    suman = np.sum(an, axis=1)
    sumanc = np.sum(an[:, :, None] * (p + pnext + pavg[:, None, :]), axis=1)
    centre = np.where(
        (suman > np.finfo(np.float64).tiny)[:, None],
        sumanc / (3.0 * np.maximum(suman[:, None], np.finfo(np.float64).tiny)),
        pavg,
    )
    return 0.5 * suma, centre


def internal_face_skewness(
    points: np.ndarray,
    face_centres: np.ndarray,
    face_areas: np.ndarray,
    owner_centres: np.ndarray,
    neighbour_centres: np.ndarray,
) -> np.ndarray:
    """Foundation 13 ``meshCheck::faceSkewness`` for internal faces.

    ``points`` is an ``N x P x 3`` array holding each face's ordered points.
    The implementation mirrors the installed Foundation 13 source.  The
    ``rootVSmall`` terms are represented by the binary64 value used by the
    OpenFOAM scalar constants; they only protect exactly degenerate input.
    """
    root_v_small = np.float64(1.0e-150)
    p = np.asarray(points, dtype=np.float64)
    fc = np.asarray(face_centres, dtype=np.float64)
    fa = np.asarray(face_areas, dtype=np.float64)
    own = np.asarray(owner_centres, dtype=np.float64)
    nei = np.asarray(neighbour_centres, dtype=np.float64)
    cpf = fc - own
    d = nei - own
    scale = np.einsum("ij,ij->i", fa, cpf) / (
        np.einsum("ij,ij->i", fa, d) + root_v_small
    )
    sv = cpf - scale[:, None] * d
    sv_mag = np.linalg.norm(sv, axis=1)
    sv_hat = sv / (sv_mag[:, None] + root_v_small)
    fd = 0.2 * np.linalg.norm(d, axis=1) + root_v_small
    projected_extent = np.max(
        np.abs(np.einsum("nij,nj->ni", p - fc[:, None, :], sv_hat)),
        axis=1,
    )
    fd = np.maximum(fd, projected_extent)
    return sv_mag / fd


def prism_cell_centres(prisms: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return Foundation-equivalent centres and volumes for type-6 wedges.

    ``prisms`` uses [lower0, lower1, lower2, upper0, upper1, upper2].
    Face area vectors are oriented outward before the Foundation pyramid sum.
    """
    p = np.asarray(prisms, dtype=np.float64)
    if p.ndim != 3 or p.shape[1:] != (6, 3):
        raise ValueError("ExpectedPrismArrayNx6x3")
    faces = (
        p[:, [0, 2, 1]],
        p[:, [3, 4, 5]],
        p[:, [0, 1, 4, 3]],
        p[:, [1, 2, 5, 4]],
        p[:, [2, 0, 3, 5]],
    )
    areas: list[np.ndarray] = []
    centres: list[np.ndarray] = []
    for index, face in enumerate(faces):
        if index < 2:
            area, centre = face_area_centre_tri(face)
        else:
            area, centre = face_area_centre_quad(face)
        areas.append(area)
        centres.append(centre)
    fa = np.stack(areas, axis=1)
    fc = np.stack(centres, axis=1)
    vertex_est = np.mean(p, axis=1)
    outward = np.einsum("nij,nij->ni", fa, fc - vertex_est[:, None, :])
    fa = np.where((outward < 0.0)[:, :, None], -fa, fa)
    cest = np.mean(fc, axis=1)
    pyr3 = np.einsum("nij,nij->ni", fa, fc - cest[:, None, :])
    pc = 0.75 * fc + 0.25 * cest[:, None, :]
    volume3 = np.sum(pyr3, axis=1)
    centre = np.sum(pyr3[:, :, None] * pc, axis=1) / volume3[:, None]
    return centre, volume3 / 3.0


def unique_manifold_edges(faces: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return sorted edge vertices and its two incident oriented face ids."""
    tri = np.asarray(faces, dtype=np.int64)
    raw_edges = np.concatenate((tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]))
    raw_faces = np.tile(np.arange(len(tri), dtype=np.int64), 3)
    keys = np.sort(raw_edges, axis=1)
    order = np.lexsort((keys[:, 1], keys[:, 0]))
    keys = keys[order]
    raw_faces = raw_faces[order]
    starts = np.flatnonzero(np.r_[True, np.any(keys[1:] != keys[:-1], axis=1)])
    stops = np.r_[starts[1:], len(keys)]
    counts = stops - starts
    if np.any(counts != 2):
        raise RuntimeError("WallNotClosedTwoManifold")
    incident = np.column_stack((raw_faces[starts], raw_faces[starts + 1]))
    return keys[starts], incident


def layer_geometry(
    wall: np.ndarray,
    faces: np.ndarray,
    trajectories: np.ndarray,
    lower_height: np.ndarray,
    upper_height: np.ndarray,
    edges: np.ndarray,
    edge_faces: np.ndarray,
) -> dict[str, np.ndarray]:
    lower_points = wall + lower_height[:, None] * trajectories
    upper_points = wall + upper_height[:, None] * trajectories
    lower = lower_points[faces]
    upper = upper_points[faces]
    prisms = np.concatenate((lower, upper), axis=1)
    centres, volumes = prism_cell_centres(prisms)
    side = np.stack(
        (
            lower_points[edges[:, 0]],
            lower_points[edges[:, 1]],
            upper_points[edges[:, 1]],
            upper_points[edges[:, 0]],
        ),
        axis=1,
    )
    area, face_centre = face_area_centre_quad(side)
    cell_a = centres[edge_faces[:, 0]]
    cell_b = centres[edge_faces[:, 1]]
    d_a = np.abs(np.einsum("ij,ij->i", area, face_centre - cell_a))
    d_b = np.abs(np.einsum("ij,ij->i", area, cell_b - face_centre))
    weight = np.minimum(d_a, d_b) / np.maximum(
        d_a + d_b, np.finfo(np.float64).tiny
    )
    ratio = np.maximum(d_a, d_b) / np.maximum(
        np.minimum(d_a, d_b), np.finfo(np.float64).tiny
    )
    skewness = internal_face_skewness(
        side, face_centre, area, cell_a, cell_b
    )
    return {
        "centres": centres,
        "volumes": volumes,
        "side_face_centres": face_centre,
        "side_face_areas": area,
        "d_a_area_scaled": d_a,
        "d_b_area_scaled": d_b,
        "side_weights": weight,
        "side_symmetric_ratio": ratio,
        "side_skewness": skewness,
    }


def layer_subset_geometry(
    wall: np.ndarray,
    faces: np.ndarray,
    trajectories: np.ndarray,
    lower_height: np.ndarray,
    upper_height: np.ndarray,
    selected_faces: np.ndarray,
    selected_edges: np.ndarray,
    edges: np.ndarray,
    edge_faces: np.ndarray,
) -> dict[str, np.ndarray]:
    """Foundation geometry for a face/edge subset used by bounded planners."""
    face_ids = np.asarray(selected_faces, dtype=np.int64)
    edge_ids = np.asarray(selected_edges, dtype=np.int64)
    lower_points = wall + lower_height[:, None] * trajectories
    upper_points = wall + upper_height[:, None] * trajectories
    incidence = faces[face_ids]
    prisms = np.concatenate((lower_points[incidence], upper_points[incidence]), axis=1)
    centres, volumes = prism_cell_centres(prisms)
    local = np.full(len(faces), -1, dtype=np.int64)
    local[face_ids] = np.arange(len(face_ids), dtype=np.int64)
    pair = local[edge_faces[edge_ids]]
    if np.any(pair < 0):
        raise RuntimeError("SelectedEdgesRequireBothIncidentFaces")
    edge = edges[edge_ids]
    side = np.stack(
        (
            lower_points[edge[:, 0]], lower_points[edge[:, 1]],
            upper_points[edge[:, 1]], upper_points[edge[:, 0]],
        ), axis=1,
    )
    area, face_centre = face_area_centre_quad(side)
    d_a = np.abs(np.einsum(
        "ij,ij->i", area, face_centre - centres[pair[:, 0]]
    ))
    d_b = np.abs(np.einsum(
        "ij,ij->i", area, centres[pair[:, 1]] - face_centre
    ))
    weight = np.minimum(d_a, d_b) / np.maximum(
        d_a + d_b, np.finfo(np.float64).tiny
    )
    skewness = internal_face_skewness(
        side, face_centre, area, centres[pair[:, 0]], centres[pair[:, 1]]
    )
    return {
        "centres": centres, "volumes": volumes,
        "side_weights": weight,
        "side_skewness": skewness,
        "side_symmetric_ratio": np.maximum(d_a, d_b) / np.maximum(
            np.minimum(d_a, d_b), np.finfo(np.float64).tiny
        ),
    }
