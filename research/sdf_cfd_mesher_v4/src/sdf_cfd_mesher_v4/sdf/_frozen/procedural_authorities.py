"""Live procedural authorities used by the geometry-neutral V3MMG1 provider.

This module contains geometry definitions.  It owns no mesh topology operation.
All persistent identity is supplied by the input ledgers and integer registries.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

POSITION_LIMIT = 51.480559638e-6
NORMAL_LIMIT_RAD = math.radians(5.729078314)
HMAX = 0.04


def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if not n > 0.0:
        raise ValueError("zero vector")
    return v / n


def tangent_metric(u: np.ndarray, hu: float, v: np.ndarray, hv: float,
                   n: np.ndarray, hn: float | None = None) -> np.ndarray:
    if hn is None:
        hn = min(hu, hv)
    m = np.zeros((3, 3), dtype=np.float64)
    for d, h in ((unit(u), hu), (unit(v), hv), (unit(n), hn)):
        m += np.outer(d, d) / max(h, 1.0e-9) ** 2
    return m


def curvature_size(kappa: float, multiplier: float) -> float:
    if not kappa > 0.0:
        return HMAX
    radius = 1.0 / kappa
    delta = min(POSITION_LIMIT, radius)
    h_chord = 2.0 * math.sqrt(max(0.0, 2.0 * radius * delta - delta * delta))
    h_normal = 2.0 * math.sin(0.5 * NORMAL_LIMIT_RAD) / kappa
    return max(1.0e-7, min(HMAX, multiplier * h_chord, multiplier * h_normal))


@dataclass(frozen=True)
class SeedVertex:
    point: np.ndarray
    sheet_ref: int
    chart: int
    corner_ref: int = 0


@dataclass(frozen=True)
class SeedFace:
    vertices: tuple[int, int, int]
    sheet_ref: int


@dataclass(frozen=True)
class SeedEdge:
    vertices: tuple[int, int]
    curve_ref: int
    ridge: bool


class Authority:
    """Geometry plug-in contract consumed by the generic provider."""

    name: str
    sheet_names: dict[int, str]
    curve_names: dict[int, str]
    corner_names: dict[int, str] = {}

    def seed(self, vertices_path: str, faces_path: str,
             features_path: str | None) -> tuple[list[SeedVertex], list[SeedFace], list[SeedEdge]]:
        raise NotImplementedError

    def metric(self, vertex: SeedVertex, curve_refs: set[int], multiplier: float) -> np.ndarray:
        raise NotImplementedError

    def project_sheet(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def project_curve(self, curve_ref: int, point: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def exact_normal(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def implicit_value(self, sheet_ref: int, point: np.ndarray) -> float:
        """A length-valued branch residual. Zero is the procedural sheet."""
        return float(np.linalg.norm(point - self.project_sheet(sheet_ref, point)))

    def curvature_upper(self, sheet_ref: int, points: np.ndarray) -> float:
        raise NotImplementedError

    def feature_residual(self, curve_ref: int, point: np.ndarray) -> float:
        return float(np.linalg.norm(point - self.project_curve(curve_ref, point)))

    def branch_name(self, sheet_ref: int) -> str:
        return self.sheet_names[sheet_ref]

    def competing_component_sheet_refs(self, sheet_ref: int) -> tuple[int, ...]:
        return ()


def read_tsv(path: str) -> list[dict[str, str]]:
    with open(path, "r", encoding="utf-8") as stream:
        header = stream.readline().rstrip("\n").split("\t")
        return [dict(zip(header, line.rstrip("\n").split("\t")))
                for line in stream if line.strip()]


class Naca0012Authority(Authority):
    name = "naca0012_extrusion"
    t = 0.12
    half_span = 0.25
    trailing_half = 0.00126
    sheet_names = {1: "airfoil.skin", 2: "airfoil.trailing_face",
                   3: "airfoil.cap_negative", 4: "airfoil.cap_positive"}
    curve_names = {
        101: "airfoil.curve.skin_cap.negative",
        102: "airfoil.curve.skin_cap.positive",
        103: "airfoil.curve.trailing.lower.span",
        104: "airfoil.curve.trailing.upper.span",
        105: "airfoil.curve.trailing_cap.negative",
        106: "airfoil.curve.trailing_cap.positive",
    }

    @classmethod
    def yt(cls, x: float) -> float:
        x = min(1.0, max(0.0, x))
        return 5.0 * cls.t * (0.2969 * math.sqrt(x) - 0.1260 * x
                              - 0.3516 * x * x + 0.2843 * x ** 3
                              - 0.1015 * x ** 4)

    @classmethod
    def dyt(cls, x: float) -> float:
        x = max(1.0e-15, x)
        return 5.0 * cls.t * (0.2969 / (2.0 * math.sqrt(x)) - 0.1260
                              - 2.0 * 0.3516 * x + 3.0 * 0.2843 * x * x
                              - 4.0 * 0.1015 * x ** 3)

    @classmethod
    def d2yt(cls, x: float) -> float:
        x = max(1.0e-15, x)
        return 5.0 * cls.t * (-0.2969 / (4.0 * x * math.sqrt(x))
                              - 2.0 * 0.3516 + 6.0 * 0.2843 * x
                              - 12.0 * 0.1015 * x * x)

    @classmethod
    def curvature(cls, x: float) -> float:
        if x <= 1.0e-12:
            return 1.0 / (1.1019 * cls.t * cls.t)
        yp = cls.dyt(x)
        return abs(cls.d2yt(x)) / (1.0 + yp * yp) ** 1.5

    @classmethod
    def closest_x(cls, sign: float, p: np.ndarray) -> float:
        from scipy.optimize import minimize_scalar
        f = lambda x: (x - p[0]) ** 2 + (sign * cls.yt(x) - p[1]) ** 2
        result = minimize_scalar(f, bounds=(0.0, 1.0), method="bounded",
                                 options={"xatol": 1.0e-15, "maxiter": 160})
        candidates = [0.0, 1.0, float(result.x)]
        return min(candidates, key=f)

    def seed(self, vertices_path: str, faces_path: str,
             features_path: str | None) -> tuple[list[SeedVertex], list[SeedFace], list[SeedEdge]]:
        vr = read_tsv(vertices_path)
        vertices = []
        for row in vr:
            branch = int(row["procedural_branch"])
            ref = 1 if branch <= 1 else branch
            corner = 10001 + int(row["vertex"]) if int(row["in_dimension"]) == 0 else 0
            vertices.append(SeedVertex(np.array([float(row["x"]), float(row["y"]),
                                                 float(row["z"])], dtype=np.float64),
                                       ref, branch, corner))
        faces = []
        for row in read_tsv(faces_path):
            branch = int(row.get("validation_procedural_branch", row.get("procedural_branch", "0")))
            ref = 1 if branch <= 1 else branch
            faces.append(SeedFace((int(row["v0"]), int(row["v1"]), int(row["v2"])), ref))
        edges: list[SeedEdge] = []
        if features_path:
            by_name = {name: ref for ref, name in self.curve_names.items()}
            for row in read_tsv(features_path):
                edges.append(SeedEdge((int(row["v0"]), int(row["v1"])),
                                      by_name[row["curve"]], True))
        return vertices, faces, edges

    def metric(self, vertex: SeedVertex, curve_refs: set[int], multiplier: float) -> np.ndarray:
        p = vertex.point
        if vertex.sheet_ref == 1 or curve_refs.intersection({101, 102}):
            sign = -1.0 if vertex.chart == 1 else 1.0
            x = min(1.0, max(0.0, float(p[0])))
            u = unit(np.array([0.0, sign, 0.0]) if x <= 1e-10 else
                     np.array([1.0, sign * self.dyt(x), 0.0]))
            v = np.array([0.0, 0.0, 1.0])
            n = unit(np.cross(u, v))
            hu = curvature_size(self.curvature(x), multiplier)
            return tangent_metric(u, hu, v, HMAX, n, hu)
        if vertex.sheet_ref in (3, 4):
            return tangent_metric(np.array([1., 0., 0.]), HMAX,
                                  np.array([0., 1., 0.]), HMAX,
                                  np.array([0., 0., 1.]), HMAX)
        return tangent_metric(np.array([0., 1., 0.]), HMAX,
                              np.array([0., 0., 1.]), HMAX,
                              np.array([1., 0., 0.]), HMAX)

    def _project_profile(self, sign: float, p: np.ndarray) -> np.ndarray:
        x = self.closest_x(sign, p)
        return np.array([x, sign * self.yt(x), min(self.half_span, max(-self.half_span, p[2]))])

    def project_sheet(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        p = np.asarray(point, dtype=float)
        if sheet_ref == 1:
            pu, pl = self._project_profile(1.0, p), self._project_profile(-1.0, p)
            return pu if np.dot(p - pu, p - pu) <= np.dot(p - pl, p - pl) else pl
        if sheet_ref == 2:
            return np.array([1.0, p[1], p[2]])
        if sheet_ref == 3:
            return np.array([p[0], p[1], -self.half_span])
        if sheet_ref == 4:
            return np.array([p[0], p[1], self.half_span])
        raise KeyError(sheet_ref)

    def project_curve(self, curve_ref: int, point: np.ndarray) -> np.ndarray:
        p = np.asarray(point, dtype=float)
        if curve_ref in (101, 102):
            sign = -1.0 if p[1] < 0.0 else 1.0
            q = self._project_profile(sign, p)
            q[2] = -self.half_span if curve_ref == 101 else self.half_span
            return q
        if curve_ref == 103:
            return np.array([1.0, -self.trailing_half, np.clip(p[2], -self.half_span, self.half_span)])
        if curve_ref == 104:
            return np.array([1.0, self.trailing_half, np.clip(p[2], -self.half_span, self.half_span)])
        if curve_ref == 105:
            return np.array([1.0, np.clip(p[1], -self.trailing_half, self.trailing_half), -self.half_span])
        if curve_ref == 106:
            return np.array([1.0, np.clip(p[1], -self.trailing_half, self.trailing_half), self.half_span])
        raise KeyError(curve_ref)

    def exact_normal(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        if sheet_ref == 1:
            if point[0] <= 1.0e-12:
                return np.array([-1., 0., 0.])
            sign = -1.0 if point[1] < 0.0 else 1.0
            return unit(np.array([-self.dyt(float(point[0])), sign, 0.0]))
        return {2: np.array([1., 0., 0.]), 3: np.array([0., 0., -1.]),
                4: np.array([0., 0., 1.])}[sheet_ref]

    def curvature_upper(self, sheet_ref: int, points: np.ndarray) -> float:
        if sheet_ref != 1:
            return 0.0
        lo, hi = max(0.0, float(points[:, 0].min())), min(1.0, float(points[:, 0].max()))
        xs = np.linspace(lo, hi, 257)
        return float(max(self.curvature(float(x)) for x in xs)) * (1.0 + 1.0e-12)

    def branch_name(self, sheet_ref: int) -> str:
        return {1: "airfoil.skin", 2: "airfoil.trailing_face.branch.planar",
                3: "airfoil.cap_negative.branch.planar",
                4: "airfoil.cap_positive.branch.planar"}[sheet_ref]


class CohortAuthority(Authority):
    def __init__(self, fixture: str):
        self.fixture = fixture
        self.name = fixture
        if fixture == "capped_cylinder":
            self.sheet_names = {1: "cylinder.side", 2: "cap.negative", 3: "cap.positive"}
            self.curve_names = {101: "circle.negative", 102: "circle.positive"}
        elif fixture == "narrow_gap":
            self.sheet_names = {1: "component.negative", 2: "component.positive"}
            self.curve_names = {}
        elif fixture == "sharp_feature_closed_body":
            self.sheet_names = {1: "bicone.negative", 2: "bicone.positive"}
            self.curve_names = {101: "bicone.equator"}
            self.corner_names = {1001: "bicone.tip.negative", 1002: "bicone.tip.positive"}
        else:
            self.sheet_names = {1: fixture + ".surface"}
            self.curve_names = {}
            self.corner_names = {}

    def seed(self, vertices_path: str, faces_path: str,
             features_path: str | None) -> tuple[list[SeedVertex], list[SeedFace], list[SeedEdge]]:
        rows = read_tsv(vertices_path)
        vertices = [SeedVertex(np.array([float(r["x"]), float(r["y"]), float(r["z"])]),
                               1, 0, int(r.get("corner", "0"))) for r in rows]
        faces = []
        for r in read_tsv(faces_path):
            raw_sheet = int(r["sheet"])
            ref = raw_sheet + 1
            faces.append(SeedFace((int(r["v0"]), int(r["v1"]), int(r["v2"])), ref))
        incident: list[set[int]] = [set() for _ in vertices]
        for f in faces:
            for v in f.vertices:
                incident[v].add(f.sheet_ref)
        vertices = [SeedVertex(v.point, min(incident[i]) if incident[i] else 1, 0, v.corner_ref)
                    for i, v in enumerate(vertices)]
        edges = []
        if features_path:
            by_name = {name: ref for ref, name in self.curve_names.items()}
            for r in read_tsv(features_path):
                if r["curve"] in by_name:
                    edges.append(SeedEdge((int(r["v0"]), int(r["v1"])),
                                          by_name[r["curve"]], True))
        return vertices, faces, edges

    def _sphere(self, ref: int) -> tuple[np.ndarray, float]:
        if self.fixture == "narrow_gap":
            return (np.array([-0.36, 0., 0.]) if ref == 1 else np.array([0.36, 0., 0.]), 0.3)
        return np.zeros(3), 0.5

    def _ellipsoid_axes(self) -> np.ndarray | None:
        axes = {
            "thin_smooth_lens": (.50, .12, .25),
            "varying_curvature_body": (.50, .30, .15),
            "rotating_curvature_blend": (.50, .28, .10),
            "long_anisotropic_smooth_extrusion": (.70, .12, .12),
            "poorly_oriented_curvature": (.50, .22, .07),
        }.get(self.fixture)
        return None if axes is None else np.asarray(axes, dtype=float)

    def _torus_parameters(self) -> tuple[float, float] | None:
        parameters = {
            "saddle_surface": (.42, .12),
            "one_direction_fillet": (.65, .07),
            "mixed_curvature_sliver": (.45, .09),
        }.get(self.fixture)
        return parameters

    def _blend_parameters(self) -> tuple[np.ndarray, np.ndarray, float] | None:
        if self.fixture == "wing_body_smooth_junction":
            return np.array([.72, .18, .055]), np.array([.18, .18, .58]), .045
        if self.fixture == "wing_body_simplified_blend":
            return np.array([.58, .20, .09]), np.array([.20, .20, .46]), .060
        return None

    def _normal_map_principal_data(self, sheet_ref: int, point: np.ndarray):
        """Deterministic local shape operator from the live normal authority.

        This is used when the procedural source has no analytic Hessian. The
        2.50 reserve is applied to curvature magnitudes after the symmetric
        tangent shape operator is formed.
        """
        p = np.asarray(point, dtype=float)
        n = self.exact_normal(sheet_ref, p)
        u = unit(np.cross(n, np.array([0., 0., 1.])) if abs(n[2]) < .9
                 else np.cross(n, np.array([0., 1., 0.])))
        v = unit(np.cross(n, u)); eps = 1.0e-5
        du = (self.exact_normal(sheet_ref, self.project_sheet(sheet_ref, p + eps * u)) -
              self.exact_normal(sheet_ref, self.project_sheet(sheet_ref, p - eps * u))) / (2.0 * eps)
        dv = (self.exact_normal(sheet_ref, self.project_sheet(sheet_ref, p + eps * v)) -
              self.exact_normal(sheet_ref, self.project_sheet(sheet_ref, p - eps * v))) / (2.0 * eps)
        shape = np.array([[np.dot(u, du), np.dot(u, dv)],
                          [np.dot(v, du), np.dot(v, dv)]])
        values, vectors = np.linalg.eigh(0.5 * (shape + shape.T))
        d0 = unit(vectors[0, 0] * u + vectors[1, 0] * v)
        d1 = unit(vectors[0, 1] * u + vectors[1, 1] * v)
        return d0, d1, 2.50 * abs(float(values[0])), 2.50 * abs(float(values[1])), n

    def metric(self, vertex: SeedVertex, curve_refs: set[int], multiplier: float) -> np.ndarray:
        p = vertex.point
        if self.fixture in ("sphere", "already_qualified_smooth_surface", "narrow_gap"):
            c, r = self._sphere(vertex.sheet_ref)
            n = unit(p - c)
            u = unit(np.cross(n, np.array([0., 0., 1.])) if abs(n[2]) < .9 else np.cross(n, np.array([0., 1., 0.])))
            v = unit(np.cross(n, u))
            h = curvature_size(1.0 / r, multiplier)
            return tangent_metric(u, h, v, h, n, h)
        if self.fixture == "capped_cylinder" and vertex.sheet_ref == 1:
            n = unit(np.array([p[0], p[1], 0.]))
            u = unit(np.array([-p[1], p[0], 0.]))
            h = curvature_size(1.0 / .35, multiplier)
            return tangent_metric(u, h, np.array([0., 0., 1.]), HMAX, n, h)
        torus = self._torus_parameters()
        if self.fixture == "concave_pocket" or torus is not None:
            major, minor = (.32, .16) if torus is None else torus
            rho = math.hypot(p[0], p[1])
            c = np.array([major * p[0] / max(rho, 1e-15), major * p[1] / max(rho, 1e-15), 0.])
            n = unit(p - c)
            u = unit(np.array([-p[1], p[0], 0.]))
            v = unit(np.cross(n, u))
            h1 = curvature_size(1.0 / minor, multiplier)
            h2 = curvature_size(1.0 / max(minor, major - minor), multiplier)
            return tangent_metric(u, h2, v, h1, n, min(h1, h2))
        if self._blend_parameters() is not None:
            d0, d1, k0, k1, n = self._normal_map_principal_data(vertex.sheet_ref, p)
            return tangent_metric(d0, curvature_size(k0, multiplier),
                                  d1, curvature_size(k1, multiplier), n,
                                  min(curvature_size(k0, multiplier),
                                      curvature_size(k1, multiplier)))
        if self.fixture == "sharp_feature_closed_body":
            rho = math.hypot(p[0], p[1])
            n = self.exact_normal(vertex.sheet_ref, p)
            if rho <= 1.0e-15:
                # The required apex is a zero-measure singular corner.  Its
                # incident smooth sheet sectors, not a fictitious apex normal,
                # control the neighboring metric.
                return tangent_metric(np.array([1.0, 0.0, 0.0]), HMAX,
                                      np.array([0.0, 1.0, 0.0]), HMAX, n, HMAX)
            circumferential = unit(np.array([-p[1], p[0], 0.0]))
            meridional = unit(np.cross(n, circumferential))
            # On rho + 0.5*sign(z)*z = 0.35 the circumferential
            # principal curvature is n_r/rho and the meridional curvature is
            # zero.  This is the required corner-local anisotropic authority.
            k_circumferential = 2.50 * (1.0 / math.sqrt(1.25)) / rho
            h_circumferential = curvature_size(k_circumferential, multiplier)
            return tangent_metric(circumferential, h_circumferential,
                                  meridional, HMAX, n, h_circumferential)
        axes = self._ellipsoid_axes()
        if axes is not None:
            n = self.exact_normal(vertex.sheet_ref, p)
            u = unit(np.cross(n, np.array([0., 0., 1.])) if abs(n[2]) < .9
                     else np.cross(n, np.array([0., 1., 0.])))
            v = unit(np.cross(n, u))
            k = {
                "thin_smooth_lens": 35.0,
                "varying_curvature_body": 23.0,
                "rotating_curvature_blend": 40.0,
                "long_anisotropic_smooth_extrusion": 58.333333333333336,
                "poorly_oriented_curvature": 102.04081632653062,
            }[self.fixture]
            h = curvature_size(k, multiplier)
            return tangent_metric(u, h, v, h, n, h)
        # Rounded-box transitions are deliberately conservative but remain tangent-only.
        n = self.exact_normal(vertex.sheet_ref, p)
        u = unit(np.cross(n, np.array([0., 0., 1.])) if abs(n[2]) < .9 else np.cross(n, np.array([0., 1., 0.])))
        v = unit(np.cross(n, u))
        k = 10.0 if self.fixture == "rounded_box" else 0.0
        h = curvature_size(k, multiplier)
        return tangent_metric(u, h, v, h, n, h)

    def project_sheet(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        p = np.asarray(point, dtype=float)
        if self.fixture in ("sphere", "already_qualified_smooth_surface", "narrow_gap"):
            c, r = self._sphere(sheet_ref)
            return c + r * unit(p - c)
        if self.fixture == "capped_cylinder":
            if sheet_ref == 1:
                q = p.copy(); q[:2] = .35 * unit(p[:2]); q[2] = np.clip(q[2], -.45, .45); return q
            q = p.copy(); q[2] = -.45 if sheet_ref == 2 else .45; return q
        torus = self._torus_parameters()
        if self.fixture == "concave_pocket" or torus is not None:
            major, minor = (.32, .16) if torus is None else torus
            rho = math.hypot(p[0], p[1]); radial = np.array([p[0], p[1], 0.]) / max(rho, 1e-15)
            c = major * radial
            return c + minor * unit(p - c)
        if self.fixture == "rounded_box":
            half = np.array([.4, .3, .25]); rr = .1
            core = half - rr
            q = np.clip(p, -core, core)
            d = p - q
            if np.linalg.norm(d) > 1e-15:
                return q + rr * unit(d)
            # Interior numerical fallback goes to the nearest core face plus radius.
            margins = core - np.abs(p)
            axis = int(np.argmin(margins)); d = np.zeros(3); d[axis] = 1. if p[axis] >= 0 else -1.
            return q + rr * d
        axes = self._ellipsoid_axes()
        if axes is not None:
            q = math.sqrt(float(np.sum((p / axes) ** 2)))
            if not q > 0.0:
                raise ValueError("ellipsoid centre has no radial projector")
            return p / q
        if self.fixture == "sharp_feature_closed_body":
            # Persistent sheet identity selects one finite cone sector.  Project
            # in the (rho,z) meridian and clamp to that sector's two analytic
            # corners.  Using the union abs(z) field here can carry an MMGS point
            # through the singular apex and onto the opposite sheet.
            rho = math.hypot(p[0], p[1])
            radial = (np.array([p[0], p[1], 0.0]) / rho
                      if rho > 1.0e-15 else np.array([1.0, 0.0, 0.0]))
            apex_z = -0.7 if sheet_ref == 1 else 0.7
            a = np.array([0.0, apex_z])
            b = np.array([0.35, 0.0])
            meridian = np.array([rho, p[2]])
            direction = b - a
            t = float(np.clip(np.dot(meridian - a, direction) /
                              np.dot(direction, direction), 0.0, 1.0))
            projected_meridian = a + t * direction
            if t <= 1.0e-14:
                return np.array([0.0, 0.0, apex_z])
            return np.array([radial[0] * projected_meridian[0],
                             radial[1] * projected_meridian[0],
                             projected_meridian[1]])
        if self._blend_parameters() is not None:
            q = p.copy()
            for _ in range(48):
                value = self.implicit_value(sheet_ref, q)
                if abs(value) <= 1.0e-14:
                    break
                grad = self._wing_body_gradient(q)
                q = q - value * grad / float(np.dot(grad, grad))
            return q
        raise KeyError(self.fixture)

    def project_curve(self, curve_ref: int, point: np.ndarray) -> np.ndarray:
        if self.fixture == "capped_cylinder":
            q = np.asarray(point, dtype=float).copy(); q[:2] = .35 * unit(q[:2]); q[2] = -.45 if curve_ref == 101 else .45; return q
        if self.fixture == "sharp_feature_closed_body" and curve_ref == 101:
            q = np.asarray(point, dtype=float).copy()
            q[:2] = .35 * unit(q[:2]); q[2] = 0.0
            return q
        raise KeyError(curve_ref)

    @staticmethod
    def _ellipsoid_value(point: np.ndarray, axes: np.ndarray) -> float:
        return (math.sqrt(float(np.sum((point / axes) ** 2))) - 1.0) * float(np.min(axes))

    @staticmethod
    def _ellipsoid_gradient(point: np.ndarray, axes: np.ndarray) -> np.ndarray:
        scaled_norm = math.sqrt(float(np.sum((point / axes) ** 2)))
        if not scaled_norm > 0.0:
            raise ValueError("ellipsoid centre has no gradient")
        return float(np.min(axes)) * point / (axes * axes * scaled_norm)

    @staticmethod
    def _ellipsoid_hessian(point: np.ndarray, axes: np.ndarray) -> np.ndarray:
        """Analytic Hessian of the length-valued ellipsoid function."""
        p = np.asarray(point, dtype=float)
        inv_axes2 = 1.0 / (axes * axes)
        bp = inv_axes2 * p
        q = math.sqrt(float(np.dot(p, bp)))
        if not q > 0.0:
            raise ValueError("ellipsoid centre has no Hessian")
        scale = float(np.min(axes))
        return scale * (np.diag(inv_axes2) / q - np.outer(bp, bp) / (q ** 3))

    @staticmethod
    def _ellipsoid_hessian_norm_bound(box_lo: np.ndarray, box_hi: np.ndarray,
                                      axes: np.ndarray) -> float:
        """Conservative Hessian 2-norm bound over an axis-aligned box."""
        min_abs = np.where((box_lo <= 0.0) & (box_hi >= 0.0), 0.0,
                           np.minimum(np.abs(box_lo), np.abs(box_hi)))
        max_abs = np.maximum(np.abs(box_lo), np.abs(box_hi))
        inv_axes2 = 1.0 / (axes * axes)
        q_min = math.sqrt(float(np.sum(min_abs * min_abs * inv_axes2)))
        if not q_min > 0.0:
            return math.inf
        bp_max = math.sqrt(float(np.sum((max_abs * inv_axes2) ** 2)))
        return float(np.min(axes)) * (float(np.max(inv_axes2)) / q_min +
                                      bp_max * bp_max / (q_min ** 3))

    @staticmethod
    def _smooth_min(a: float, b: float, k: float) -> float:
        h = min(1.0, max(0.0, 0.5 + 0.5 * (b - a) / k))
        return (1.0 - h) * b + h * a - k * h * (1.0 - h)

    def _wing_body_gradient(self, point: np.ndarray) -> np.ndarray:
        parameters = self._blend_parameters()
        if parameters is None:
            raise ValueError("fixture has no smooth blend")
        wing, body, blend_width = parameters
        a = self._ellipsoid_value(point, wing); b = self._ellipsoid_value(point, body)
        h = min(1.0, max(0.0, 0.5 + 0.5 * (b - a) / blend_width))
        return (h * self._ellipsoid_gradient(point, wing) +
                (1.0 - h) * self._ellipsoid_gradient(point, body))

    def _wing_body_hessian(self, point: np.ndarray) -> np.ndarray:
        p = np.asarray(point, dtype=float)
        parameters = self._blend_parameters()
        if parameters is None:
            raise ValueError("fixture has no smooth blend")
        wing, body, blend_width = parameters
        a = self._ellipsoid_value(p, wing); b = self._ellipsoid_value(p, body)
        ga = self._ellipsoid_gradient(p, wing); gb = self._ellipsoid_gradient(p, body)
        ha = self._ellipsoid_hessian(p, wing); hb = self._ellipsoid_hessian(p, body)
        h = min(1.0, max(0.0, 0.5 + 0.5 * (b - a) / blend_width))
        if h <= 0.0:
            return hb
        if h >= 1.0:
            return ha
        difference = ga - gb
        return h * ha + (1.0 - h) * hb - np.outer(difference, difference) / (2.0 * blend_width)

    def gradient_hessian(self, sheet_ref: int,
                         point: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Exact differential authority for the smooth procedural SDFs.

        The returned Hessian follows the convention used by ``implicit_value``.
        It is intentionally unavailable for non-smooth rounded-box and cone
        authorities; callers must fail closed rather than invent a derivative
        through a singular feature.
        """
        p = np.asarray(point, dtype=float)
        if self.fixture in ("sphere", "already_qualified_smooth_surface",
                            "narrow_gap"):
            centre, _ = self._sphere(sheet_ref)
            q = p - centre
            radius = float(np.linalg.norm(q))
            if not radius > 0.0:
                raise ValueError("sphere centre has no SDF differential")
            normal = q / radius
            return normal, (np.eye(3) - np.outer(normal, normal)) / radius
        if self.fixture == "capped_cylinder":
            if sheet_ref != 1:
                normal = np.array([0.0, 0.0, -1.0 if sheet_ref == 2 else 1.0])
                return normal, np.zeros((3, 3))
            rho = math.hypot(p[0], p[1])
            if not rho > 0.0:
                raise ValueError("cylinder axis has no radial SDF differential")
            radial = np.array([p[0] / rho, p[1] / rho])
            hessian = np.zeros((3, 3))
            hessian[:2, :2] = (np.eye(2) - np.outer(radial, radial)) / rho
            return np.array([radial[0], radial[1], 0.0]), hessian
        torus = self._torus_parameters()
        if self.fixture == "concave_pocket" or torus is not None:
            major, _ = (.32, .16) if torus is None else torus
            rho = math.hypot(p[0], p[1])
            q = rho - major
            distance = math.hypot(q, p[2])
            if not rho > 0.0 or not distance > 0.0:
                raise ValueError("torus medial set has no SDF differential")
            radial = np.array([p[0] / rho, p[1] / rho])
            f_rho = q / distance
            f_z = p[2] / distance
            f_rr = p[2] * p[2] / (distance ** 3)
            f_zz = q * q / (distance ** 3)
            f_rz = -q * p[2] / (distance ** 3)
            hessian = np.zeros((3, 3))
            hessian[:2, :2] = (f_rr * np.outer(radial, radial) +
                                (f_rho / rho) *
                                (np.eye(2) - np.outer(radial, radial)))
            hessian[:2, 2] = f_rz * radial
            hessian[2, :2] = f_rz * radial
            hessian[2, 2] = f_zz
            return np.array([f_rho * radial[0], f_rho * radial[1], f_z]), hessian
        axes = self._ellipsoid_axes()
        if axes is not None:
            return (self._ellipsoid_gradient(p, axes),
                    self._ellipsoid_hessian(p, axes))
        if self._blend_parameters() is not None:
            return self._wing_body_gradient(p), self._wing_body_hessian(p)
        raise ValueError(f"{self.fixture} has no smooth differential authority")

    def wing_body_hessian_norm_bound(self, points: np.ndarray) -> float:
        """Conservative smooth-min Hessian norm over a triangle support box."""
        p = np.asarray(points, dtype=float)
        lo, hi = p.min(axis=0), p.max(axis=0)
        parameters = self._blend_parameters()
        if parameters is None:
            raise ValueError("fixture has no smooth blend")
        wing, body, blend_width = parameters
        ha = self._ellipsoid_hessian_norm_bound(lo, hi, wing)
        hb = self._ellipsoid_hessian_norm_bound(lo, hi, body)

        def value_interval(axes: np.ndarray) -> tuple[float, float]:
            """Outward conservative interval for the length-valued ellipsoid."""
            inv_axes2 = 1.0 / (axes * axes)
            min_abs = np.where((lo <= 0.0) & (hi >= 0.0), 0.0,
                               np.minimum(np.abs(lo), np.abs(hi)))
            max_abs = np.maximum(np.abs(lo), np.abs(hi))
            q_lo = math.sqrt(float(np.dot(min_abs * min_abs, inv_axes2)))
            q_hi = math.sqrt(float(np.dot(max_abs * max_abs, inv_axes2)))
            scale = float(np.min(axes))
            return (math.nextafter(scale * (q_lo - 1.0), -math.inf),
                    math.nextafter(scale * (q_hi - 1.0), math.inf))

        a_lo, a_hi = value_interval(wing)
        b_lo, b_hi = value_interval(body)
        h_lo = 0.5 + 0.5 * (b_lo - a_hi) / blend_width
        h_hi = 0.5 + 0.5 * (b_hi - a_lo) / blend_width
        # Outside the polynomial blend band the smooth minimum is exactly one
        # ellipsoid.  Excluding the inactive branch here is both tighter and
        # more faithful than the former unconditional max(ha,hb) bound.
        if h_hi <= 0.0:
            return hb
        if h_lo >= 1.0:
            return ha

        def gradient_intervals(axes: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            inv_axes2 = 1.0 / (axes * axes)
            min_abs = np.where((lo <= 0.0) & (hi >= 0.0), 0.0,
                               np.minimum(np.abs(lo), np.abs(hi)))
            max_abs = np.maximum(np.abs(lo), np.abs(hi))
            q_min = math.sqrt(float(np.sum(min_abs * min_abs * inv_axes2)))
            q_max = math.sqrt(float(np.sum(max_abs * max_abs * inv_axes2)))
            if not q_min > 0.0:
                return np.full(3, -1.0), np.full(3, 1.0)
            scale = float(np.min(axes)) * inv_axes2
            lower = np.empty(3); upper = np.empty(3)
            for i in range(3):
                if lo[i] >= 0.0:
                    lower[i] = scale[i] * lo[i] / q_max
                    upper[i] = scale[i] * hi[i] / q_min
                elif hi[i] <= 0.0:
                    lower[i] = scale[i] * lo[i] / q_min
                    upper[i] = scale[i] * hi[i] / q_max
                else:
                    lower[i] = scale[i] * lo[i] / q_min
                    upper[i] = scale[i] * hi[i] / q_min
            return lower, upper

        ga_lo, ga_hi = gradient_intervals(wing)
        gb_lo, gb_hi = gradient_intervals(body)
        difference_lo = ga_lo - gb_hi
        difference_hi = ga_hi - gb_lo
        difference_abs = np.maximum(np.abs(difference_lo), np.abs(difference_hi))
        # In the active polynomial smooth-min band,
        # H = h*Ha + (1-h)*Hb - (ga-gb)(ga-gb)^T/(2k).
        # The interval gradient enclosure makes the rank-one term local rather
        # than applying the former global |ga-gb|<=2 bound.
        return math.nextafter(
            max(ha, hb) + float(np.dot(difference_abs, difference_abs)) /
            (2.0 * blend_width), math.inf)

    def exact_normal(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        if self.fixture in ("sphere", "already_qualified_smooth_surface", "narrow_gap"):
            c, _ = self._sphere(sheet_ref); return unit(point - c)
        if self.fixture == "capped_cylinder":
            return unit(np.array([point[0], point[1], 0.])) if sheet_ref == 1 else np.array([0., 0., -1. if sheet_ref == 2 else 1.])
        torus = self._torus_parameters()
        if self.fixture == "concave_pocket" or torus is not None:
            major = .32 if torus is None else torus[0]
            rho = math.hypot(point[0], point[1]); c = np.array([major * point[0] / max(rho,1e-15), major * point[1] / max(rho,1e-15), 0.]); return unit(point - c)
        axes = self._ellipsoid_axes()
        if axes is not None:
            return unit(np.asarray(point, dtype=float) / (axes * axes))
        if self.fixture == "sharp_feature_closed_body":
            p = np.asarray(point, dtype=float); rho = math.hypot(p[0], p[1])
            if not rho > 1.0e-15:
                return np.array([0., 0., -1. if p[2] < 0.0 else 1.])
            return unit(np.array([p[0] / rho, p[1] / rho, -.5 if sheet_ref == 1 else .5]))
        if self._blend_parameters() is not None:
            return unit(self._wing_body_gradient(np.asarray(point, dtype=float)))
        core = np.array([.3, .2, .15])
        d = np.maximum(np.abs(point) - core, 0.0)
        if np.linalg.norm(d) > 1e-15:
            return unit(np.sign(point) * d)
        q = np.abs(point) - core
        axis = int(np.argmax(q)); n = np.zeros(3); n[axis] = 1.0 if point[axis] >= 0 else -1.0
        return n

    def implicit_value(self, sheet_ref: int, point: np.ndarray) -> float:
        p = np.asarray(point, dtype=float)
        if self.fixture in ("sphere", "already_qualified_smooth_surface", "narrow_gap"):
            c, r = self._sphere(sheet_ref); return float(np.linalg.norm(p-c)-r)
        if self.fixture == "capped_cylinder":
            if sheet_ref == 1: return math.hypot(p[0],p[1])-.35
            return p[2]+.45 if sheet_ref == 2 else p[2]-.45
        torus = self._torus_parameters()
        if self.fixture == "concave_pocket" or torus is not None:
            major, minor = (.32, .16) if torus is None else torus
            rho = math.hypot(p[0],p[1]); return math.hypot(rho-major,p[2])-minor
        if self.fixture == "rounded_box":
            q = np.abs(p)-np.array([.3,.2,.15])
            return float(np.linalg.norm(np.maximum(q,0.0))+min(float(np.max(q)),0.0)-.1)
        axes = self._ellipsoid_axes()
        if axes is not None:
            return self._ellipsoid_value(p, axes)
        if self.fixture == "sharp_feature_closed_body":
            sign = -1.0 if sheet_ref == 1 else 1.0
            return math.hypot(p[0], p[1]) + .5 * sign * float(p[2]) - .35
        parameters = self._blend_parameters()
        if parameters is not None:
            wing, body, blend_width = parameters
            a = self._ellipsoid_value(p, wing)
            b = self._ellipsoid_value(p, body)
            return self._smooth_min(a, b, blend_width)
        return super().implicit_value(sheet_ref, p)

    def curvature_upper(self, sheet_ref: int, points: np.ndarray) -> float:
        if self.fixture in ("sphere", "already_qualified_smooth_surface"): return 2.0
        if self.fixture == "narrow_gap": return 1.0 / .3
        if self.fixture == "capped_cylinder": return 1.0 / .35 if sheet_ref == 1 else 0.0
        if self.fixture == "rounded_box": return 10.0
        if self.fixture == "concave_pocket": return 1.0 / .16
        torus = self._torus_parameters()
        if torus is not None: return 1.0 / torus[1]
        kappa = {
            "thin_smooth_lens": 35.0,
            "varying_curvature_body": 23.0,
            "rotating_curvature_blend": 40.0,
            "long_anisotropic_smooth_extrusion": 58.333333333333336,
            "poorly_oriented_curvature": 102.04081632653062,
        }.get(self.fixture)
        if kappa is not None: return kappa
        if self.fixture == "sharp_feature_closed_body": return 8.0
        if self._blend_parameters() is not None:
            p = np.asarray(points, dtype=float)
            centre = p.mean(axis=0)
            radius = max(float(np.linalg.norm(point - centre)) for point in p)
            hessian = self.wing_body_hessian_norm_bound(p)
            gradient_lower = (float(np.linalg.norm(self._wing_body_gradient(centre))) -
                              hessian * radius)
            if not gradient_lower > 0.0 or not math.isfinite(hessian):
                return math.inf
            # The normal derivative is the tangential part of H/|grad(phi)|.
            # The established 2.50 differential-geometry reserve is retained.
            return math.nextafter(2.50 * hessian / gradient_lower, math.inf)
        return 0.0

    def competing_component_sheet_refs(self, sheet_ref: int) -> tuple[int, ...]:
        if self.fixture == "narrow_gap":
            return (2,) if sheet_ref == 1 else (1,)
        return ()


class Valence3Authority(Authority):
    name = "valence3_multisheet_junction"
    sheet_names = {1: "fixture.sheet.lune.0", 2: "fixture.sheet.lune.1",
                   3: "fixture.sheet.lune.2"}
    curve_names = {101: "fixture.curve.meridian.0", 102: "fixture.curve.meridian.1",
                   103: "fixture.curve.meridian.2"}
    corner_names = {1001: "fixture.corner.north", 1002: "fixture.corner.south"}
    _curve_ref = {name: ref for ref, name in curve_names.items()}

    def seed(self, vertices_path: str, faces_path: str,
             features_path: str | None) -> tuple[list[SeedVertex], list[SeedFace], list[SeedEdge]]:
        raw = read_tsv(vertices_path)
        corner_map = {"100": 1001, "101": 1002}
        vertices = [SeedVertex(np.array([float(r["x"]), float(r["y"]), float(r["z"])]),
                               1, 0, corner_map.get(r.get("corner", "-1"), 0)) for r in raw]
        faces = [SeedFace((int(r["v0"]), int(r["v1"]), int(r["v2"])),
                          int(r["sheet"]) + 1) for r in read_tsv(faces_path)]
        incident = [set() for _ in vertices]
        for face in faces:
            for vertex in face.vertices:
                incident[vertex].add(face.sheet_ref)
        vertices = [SeedVertex(v.point, min(incident[i]), 0, v.corner_ref)
                    for i, v in enumerate(vertices)]
        edges = []
        if features_path:
            for r in read_tsv(features_path):
                edges.append(SeedEdge((int(r["from_vertex"]), int(r["to_vertex"])),
                                      self._curve_ref[r["persistent_curve_id"]], False))
        return vertices, faces, edges

    def metric(self, vertex: SeedVertex, curve_refs: set[int], multiplier: float) -> np.ndarray:
        n = unit(vertex.point)
        u = unit(np.cross(n, np.array([0., 0., 1.])) if abs(n[2]) < .9
                 else np.cross(n, np.array([0., 1., 0.])))
        v = unit(np.cross(n, u)); h = curvature_size(1.0, multiplier)
        return tangent_metric(u, h, v, h, n, h)

    def project_sheet(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        return unit(np.asarray(point, dtype=float))

    def project_curve(self, curve_ref: int, point: np.ndarray) -> np.ndarray:
        longitude = {101: 0.0, 102: 2.0 * math.pi / 3.0, 103: 4.0 * math.pi / 3.0}[curve_ref]
        radial = np.array([math.cos(longitude), math.sin(longitude), 0.0])
        p = np.asarray(point, dtype=float)
        return unit(float(np.dot(p, radial)) * radial + np.array([0., 0., p[2]]))

    def exact_normal(self, sheet_ref: int, point: np.ndarray) -> np.ndarray:
        return unit(np.asarray(point, dtype=float))

    def implicit_value(self, sheet_ref: int, point: np.ndarray) -> float:
        return float(np.linalg.norm(point) - 1.0)

    def curvature_upper(self, sheet_ref: int, points: np.ndarray) -> float:
        return 1.0

    def branch_name(self, sheet_ref: int) -> str:
        return f"fixture.branch.{sheet_ref - 1}"


def make_authority(spec: str) -> Authority:
    if spec == "naca0012":
        return Naca0012Authority()
    if spec.startswith("cohort:"):
        return CohortAuthority(spec.split(":", 1)[1])
    if spec == "valence3_multisheet_junction":
        return Valence3Authority()
    raise ValueError(f"unknown authority {spec}")
