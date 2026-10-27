"""Generic metric proposals. Actual mesh acceptance remains with V4 owners."""
import math
import numpy as np


def _geometry(points, triangles, centres, volumes):
    p = np.asarray(points, dtype=np.float64)
    raw = np.asarray(triangles)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('InvalidCapPoints')
    if raw.ndim != 2 or raw.shape[1] != 3 or not len(raw) or raw.dtype.kind not in 'iu':
        raise ValueError('InvalidCapTriangles')
    f = raw.astype(np.int64, copy=False)
    if np.any(f < 0) or np.any(f >= len(p)) or any(len(set(row)) != 3 for row in f.tolist()):
        raise ValueError('InvalidCapTriangleIndices')
    if len({tuple(sorted(row)) for row in f.tolist()}) != len(f):
        raise ValueError('DuplicateCapTriangles')
    c = np.asarray(centres, dtype=np.float64)
    v = np.asarray(volumes, dtype=np.float64)
    if c.shape != (len(f), 3) or v.shape != (len(f),) or not np.isfinite(c).all() or not np.isfinite(v).all() or np.any(v <= 0):
        raise ValueError('InvalidActualOwnerGeometry')
    return p, f, c, v


def cap_geometry_metric(points, triangles, owner_centres, owner_volumes):
    """Return nodal log-SPD using actual outward cap geometry and owner distance.

    Faces point away from the prism owners. Volumes authenticate the supplied
    positive-owner geometry domain; they do not enter this sizing objective.
    Tangential scales are longest edge L and its transverse altitude H. The
    normal scale sqrt(24)*d is the established V3 proposal, not an admission gate.
    """
    p, f, centres, volumes = _geometry(points, triangles, owner_centres, owner_volumes)
    tri = p[f]
    surface = .5 * np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(surface, axis=1)
    if not np.isfinite(area).all() or np.any(area <= 0):
        raise ValueError('DegenerateCapTriangles')
    normals = surface / area[:, None]
    distance = np.einsum('fi,fi->f', normals, tri.mean(axis=1) - centres)
    if not np.isfinite(distance).all() or np.any(distance <= 0):
        raise ValueError('CapOrientationOrOwnerDistance')
    edges = np.stack((tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 1], tri[:, 0] - tri[:, 2]), axis=1)
    length = np.linalg.norm(edges, axis=2)
    index = np.arange(len(f)); longest = length.argmax(axis=1)
    L = length[index, longest]; H = 2 * area / L
    tangent = edges[index, longest] / L[:, None]
    transverse = np.cross(normals, tangent)
    transverse /= np.linalg.norm(transverse, axis=1)[:, None]
    scales = np.stack((L, H, np.sqrt(24.) * distance), axis=1)
    # Construct the log directly in its orthonormal frame. This avoids squaring
    # very small geometric scales before taking the logarithm.
    frame = np.stack((tangent, transverse, normals), axis=2)
    log_faces = np.einsum('nij,nj,nkj->nik', frame, -2 * np.log(scales), frame)
    nodal = np.zeros((len(p), 3, 3)); count = np.zeros(len(p), dtype=np.int64)
    for role in range(3):
        np.add.at(nodal, f[:, role], log_faces)
        np.add.at(count, f[:, role], 1)
    if np.any(count == 0):
        raise ValueError('UnownedCapVertex')
    nodal /= count[:, None, None]
    nodal = .5 * (nodal + nodal.transpose(0, 2, 1))
    if not np.isfinite(nodal).all():
        raise ValueError('InvalidCapMetric')
    return nodal, dict(nodal_cap='LogEuclideanIncidentFaceMean', inner_scales='L,H,sqrt(24)*actual_owner_distance',
                      minimum_owner_distance=float(distance.min()), minimum_owner_volume=float(volumes.min()),
                      minimum_scale=float(scales.min()), maximum_scale=float(scales.max()))


def copied_seed_metric(points, triangles, owner_centres, owner_volumes, *, rows, shape_progress, size_growth, shape_power=1.):
    """Metric in level-major copied-seed order, with independent shape and size.

    shape_progress=1 makes the final metric isotropic; zero preserves the
    initial unit-determinant shape. size_growth=1 preserves geometric-mean
    length at every level. These controls propose geometry, not acceptance.
    """
    if type(rows) is not int or rows <= 0:
        raise ValueError('InvalidSeedRows')
    if type(shape_progress) not in (float, int) or not math.isfinite(shape_progress) or not 0 <= shape_progress <= 1:
        raise ValueError('InvalidShapeProgress')
    if type(size_growth) not in (float, int) or not math.isfinite(size_growth) or size_growth <= 0:
        raise ValueError('InvalidSizeGrowth')
    if type(shape_power) not in (float,int) or not math.isfinite(shape_power) or shape_power<=0:
        raise ValueError('InvalidShapePower')
    log_cap, report = cap_geometry_metric(points, triangles, owner_centres, owner_volumes)
    eigen, frame = np.linalg.eigh(log_cap)
    mean = eigen.mean(axis=1); shape = eigen - mean[:, None]
    result = []
    with np.errstate(over='raise', under='ignore', invalid='raise'):
        try:
            for level in range(rows + 1):
                u = level / rows
                logs = mean[:, None] + (1 - shape_progress * u**shape_power) * shape - 2 * u * math.log(size_growth)
                result.append(np.einsum('nij,nj,nkj->nik', frame, np.exp(logs), frame))
        except FloatingPointError as exc:
            raise ValueError('UnrepresentableSeedMetric') from exc
    metric = np.concatenate(result)
    metric = .5 * (metric + metric.transpose(0, 2, 1))
    values = np.linalg.eigvalsh(metric)
    if not np.isfinite(metric).all() or np.any(values <= 0):
        raise ValueError('UnrepresentableSeedMetric')
    sizes = 1 / np.sqrt(values)
    return metric, dict(report, shape='UnitDeterminantLogSPDRelaxation/1', shape_progress=float(shape_progress),
                        size_growth=float(size_growth), shape_power=float(shape_power), rows=rows, hmin=float(sizes.min()), hmax=float(sizes.max()),
                        initial_anisotropy_max=float(np.exp((eigen.max(axis=1) - eigen.min(axis=1)) / 2).max()),
                        scientific_authority=False)
