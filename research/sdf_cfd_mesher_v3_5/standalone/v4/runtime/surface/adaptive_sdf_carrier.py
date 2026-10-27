"""Direct-SDF predictor and conformal temporary metric-carrier refinement.

The predictor is deliberately not a geometry certificate.  It samples the
live procedural authority at vertices, edge midpoints, and the face centroid
and asks whether the existing three nodal tensors can represent the stronger
interior pointwise requirement within the unused normal-angle margin.
"""

from __future__ import annotations

from dataclasses import dataclass,asdict
import math
import time
from carrier_split_checks import refinement_edges,validate_split
from collections import defaultdict
from typing import Callable

import numpy as np

from procedural_authorities import (NORMAL_LIMIT_RAD, SeedEdge, SeedFace,
                                    SeedVertex, unit)


@dataclass(frozen=True)
class CarrierOraclePolicy:
    normal_qualification_fraction: float = 0.90
    mmgs_response_reserve: float = 5.5
    # One bisection resolves a factor two in physical sampling scale.  The
    # frozen 10% normal qualification reserve therefore requires
    # ceil(log2(1/(1-0.90))) = 4 bounded levels.
    maximum_depth: int = 4
    minimum_edge_m: float = 1.0e-7
    normal_limit_rad: float = NORMAL_LIMIT_RAD

    @property
    def allowed_metric_edge_ratio(self) -> float:
        return (math.tan(self.normal_limit_rad) /
                math.tan(self.normal_qualification_fraction * self.normal_limit_rad))

    @property
    def normal_interpolation_budget_rad(self) -> float:
        return ((1.0 - self.normal_qualification_fraction) *
                self.normal_limit_rad)


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return math.acos(float(np.clip(np.dot(unit(a), unit(b)), -1.0, 1.0)))


def _spd_edge_ratio(required: np.ndarray, represented: np.ndarray,
                    normal: np.ndarray) -> float:
    """Largest required/represented metric-edge ratio in the surface tangent."""
    first, second = _tangent_basis(unit(normal))
    basis = np.column_stack((first, second))
    required = basis.T @ required @ basis
    represented = basis.T @ represented @ basis
    values, vectors = np.linalg.eigh(0.5 * (represented + represented.T))
    if not np.all(values > 0.0):
        return math.inf
    inverse_root = (vectors * (1.0 / np.sqrt(values))) @ vectors.T
    relative = inverse_root @ required @ inverse_root
    lam = float(np.max(np.linalg.eigvalsh(0.5 * (relative + relative.T))))
    return math.sqrt(max(0.0, lam))


def _tangent_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    axis = np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < .9 else np.array([0.0, 1.0, 0.0])
    first = unit(np.cross(normal, axis))
    return first, unit(np.cross(normal, first))


def _directional_normal_derivatives(authority, sheet_ref: int, point: np.ndarray,
                                    physical_step: float) -> tuple[float, float]:
    normal = authority.exact_normal(sheet_ref, point)
    first, second = _tangent_basis(normal)
    try:
        gradient, hessian = authority.gradient_hessian(sheet_ref, point)
        magnitude = float(np.linalg.norm(gradient))
        projector = np.eye(3) - np.outer(normal, normal)
        derivative = projector @ hessian / magnitude
        return (float(np.linalg.norm(derivative @ first)),
                float(np.linalg.norm(derivative @ second)))
    except (AttributeError, ValueError):
        step = max(1.0e-8, min(.1 * physical_step, 1.0e-4))
        result = []
        for direction in (first, second):
            plus = authority.project_sheet(sheet_ref, point + step * direction)
            minus = authority.project_sheet(sheet_ref, point - step * direction)
            np_ = authority.exact_normal(sheet_ref, plus)
            nm_ = authority.exact_normal(sheet_ref, minus)
            result.append(float(np.linalg.norm(np_ - nm_)) / (2.0 * step))
        return result[0], result[1]


def assess_carrier_face(authority, vertices: list[SeedVertex], face: SeedFace,
                        metric_at: Callable[[np.ndarray, int], np.ndarray],
                        policy: CarrierOraclePolicy, detailed: bool = True,
                        *, derivative_diagnostics: bool = True) -> dict:
    if type(derivative_diagnostics) is not bool:raise ValueError('ExplicitCarrierDiagnosticChoice')
    ids = face.vertices
    points = np.asarray([vertices[i].point for i in ids], dtype=float)
    sheet = int(face.sheet_ref)
    vertex_normals = [authority.exact_normal(sheet, p) for p in points]
    vertex_metrics = [metric_at(p, sheet) for p in points]
    barycentric = (
        (.5, .5, 0.0), (0.0, .5, .5), (.5, 0.0, .5),
        (1.0/3.0, 1.0/3.0, 1.0/3.0),
    )
    maximum_metric_ratio = 1.0
    maximum_normal_defect = 0.0
    maximum_normal_spread = 0.0
    derivative_samples = []
    branch_failures = 0
    competing_clearance = math.inf
    maximum_branch_residual = 0.0
    minimum_edge = min(float(np.linalg.norm(points[(i+1)%3] - points[i])) for i in range(3))
    probes = []
    for weights in barycentric:
        w = np.asarray(weights, dtype=float)
        raw = np.einsum("i,ij->j", w, points)
        point = authority.project_sheet(sheet, raw)
        normal = authority.exact_normal(sheet, point)
        interpolated_normal = unit(np.einsum("i,ij->j", w, np.asarray(vertex_normals)))
        represented = sum(float(wi) * metric for wi, metric in zip(w, vertex_metrics))
        required = metric_at(point, sheet)
        ratio = _spd_edge_ratio(required, represented, normal)
        defect = _angle(normal, interpolated_normal)
        spread = max(_angle(normal, value) for value in vertex_normals)
        derivatives = (_directional_normal_derivatives(authority, sheet, point, minimum_edge)
                       if derivative_diagnostics else None)
        branch_residual = abs(float(authority.implicit_value(sheet, point)))
        maximum_branch_residual = max(maximum_branch_residual, branch_residual)
        branch_failures += int(branch_residual > 1.0e-10)
        for competitor in authority.competing_component_sheet_refs(sheet):
            competing_clearance = min(competing_clearance,
                                      abs(float(authority.implicit_value(competitor, point))))
        maximum_metric_ratio = max(maximum_metric_ratio, ratio)
        maximum_normal_defect = max(maximum_normal_defect, defect)
        maximum_normal_spread = max(maximum_normal_spread, spread)
        if derivatives is not None:derivative_samples.append(derivatives)
        if detailed:
            probes.append({"weights": list(weights), "point": point.tolist(),
                           "metric_edge_ratio": ratio,
                           "normal_interpolation_defect_deg": math.degrees(defect),
                           "normal_spread_deg": math.degrees(spread),
                           "branch_residual_m": branch_residual,
                           "branch_residual_limit_m": 1.0e-10,
                           "directional_normal_derivative": None if derivatives is None else list(derivatives)})
    derivative_values = np.asarray(derivative_samples, dtype=float)
    derivative_ratio = (float(np.max(derivative_values) /
                       max(float(np.min(derivative_values)), 1.0e-12))
                       if derivative_diagnostics else None)
    metric_inadequate = maximum_metric_ratio > policy.allowed_metric_edge_ratio
    normal_inadequate = maximum_normal_defect > policy.normal_interpolation_budget_rad
    inadequate = bool(metric_inadequate or normal_inadequate or branch_failures)
    return {
        "inadequate": inadequate,
        "metric_inadequate": bool(metric_inadequate),
        "normal_inadequate": bool(normal_inadequate),
        "maximum_metric_edge_ratio": maximum_metric_ratio,
        "allowed_metric_edge_ratio": policy.allowed_metric_edge_ratio,
        "maximum_normal_interpolation_defect_deg": math.degrees(maximum_normal_defect),
        "normal_interpolation_budget_deg": math.degrees(policy.normal_interpolation_budget_rad),
        "maximum_sampled_normal_spread_deg": math.degrees(maximum_normal_spread),
        "directional_derivative_ratio": derivative_ratio,
        "branch_failures": branch_failures,
        "maximum_branch_residual_m": maximum_branch_residual,
        "branch_residual_limit_m": 1.0e-10,
        "minimum_competing_branch_clearance_m": (None if math.isinf(competing_clearance)
                                                   else competing_clearance),
        "minimum_edge_m": minimum_edge,
        "probes": probes,
    }


def promote_face_probe_metrics(authority, vertices: list[SeedVertex],
                               faces: list[SeedFace],
                               metric_at: Callable[[np.ndarray, int], np.ndarray],
                               policy: CarrierOraclePolicy
                               ) -> tuple[list[np.ndarray], dict]:
    """Conservatively transport stronger fixed probe metrics to shared nodes.

    This is an interpolation representation rule, not a new physical target.
    If a probe tensor is stronger than ordinary nodal interpolation permits,
    each incident nodal tensor is intersected with M_probe/q^2, where q is the
    frozen admissible metric-edge ratio.  The operation can only strengthen a
    hard requirement and is deterministic in canonical face order.
    """
    base = [np.asarray(metric_at(v.point, v.sheet_ref), dtype=float).copy()
            for v in vertices]
    promoted = [m.copy() for m in base]
    active_faces: set[int] = set()
    requested_vertices: set[int] = set()
    weights_set = (
        (.5, .5, 0.0), (0.0, .5, .5), (.5, 0.0, .5),
        (1.0/3.0, 1.0/3.0, 1.0/3.0),
    )
    scale = policy.allowed_metric_edge_ratio ** 2
    for fi, face in enumerate(faces):
        ids = face.vertices
        points = np.asarray([vertices[i].point for i in ids], dtype=float)
        source = [base[i] for i in ids]
        for raw_weights in weights_set:
            weights = np.asarray(raw_weights, dtype=float)
            point = authority.project_sheet(face.sheet_ref, weights @ points)
            normal = authority.exact_normal(face.sheet_ref, point)
            required = np.asarray(metric_at(point, face.sheet_ref), dtype=float)
            represented = sum(float(w) * m for w, m in zip(weights, source))
            if _spd_edge_ratio(required, represented, normal) <= policy.allowed_metric_edge_ratio:
                continue
            transported = required / scale
            active_faces.add(fi)
            for vi in ids:
                # Local import avoids a dependency cycle with the provider.
                a = promoted[vi]
                values, vectors = np.linalg.eigh(a)
                root = (vectors * np.sqrt(values)) @ vectors.T
                inverse = (vectors * (1.0 / np.sqrt(values))) @ vectors.T
                relative = inverse @ transported @ inverse
                rv, rq = np.linalg.eigh(.5 * (relative + relative.T))
                cap = (rq * np.maximum(rv, 1.0)) @ rq.T
                promoted[vi] = .5 * (root @ cap @ root + (root @ cap @ root).T)
                requested_vertices.add(vi)
    changed = [i for i, (a, b) in enumerate(zip(base, promoted))
               if not np.array_equal(a, b)]
    return promoted, {
        "enabled": True,
        "role": "conservative interpolation-compatible transport; physical target unchanged",
        "active_faces": len(active_faces),
        "requested_vertices": len(requested_vertices),
        "changed_vertices": len(changed),
        "metric_edge_ratio_limit": policy.allowed_metric_edge_ratio,
        "weakening_allowed": False,
    }


def _edge_key(a: int, b: int) -> tuple[int, int]:
    return (a, b) if a < b else (b, a)


def _split_face(face: SeedFace, mids: dict[tuple[int, int], int]) -> list[SeedFace]:
    a, b, c = face.vertices
    m0 = mids.get(_edge_key(a, b)); m1 = mids.get(_edge_key(b, c)); m2 = mids.get(_edge_key(c, a))
    mask = (int(m0 is not None) | (int(m1 is not None) << 1) | (int(m2 is not None) << 2))
    r = face.sheet_ref
    templates = {
        0: [(a, b, c)],
        1: [(a, m0, c), (m0, b, c)],
        2: [(a, b, m1), (a, m1, c)],
        4: [(a, b, m2), (b, c, m2)],
        3: [(a, m0, c), (m0, m1, c), (m0, b, m1)],
        6: [(b, m1, a), (m1, m2, a), (m1, c, m2)],
        5: [(c, m2, b), (m2, m0, b), (m2, a, m0)],
        7: [(a, m0, m2), (m0, b, m1), (m2, m1, c), (m0, m1, m2)],
    }
    return [SeedFace(tuple(int(x) for x in tri), r) for tri in templates[mask]]


def refine_metric_carriers(authority, vertices: list[SeedVertex], faces: list[SeedFace],
                           edges: list[SeedEdge],
                           metric_at: Callable[[np.ndarray, int], np.ndarray],
                           policy: CarrierOraclePolicy,
                           progress: Callable[[dict], None] | None = None,
                           interpolation_compatible_transition: bool = False,
                           *, derivative_diagnostics: bool = True, evidence=None, terminal_capture=None,
                           ) -> tuple[list[SeedVertex], list[SeedFace], list[SeedEdge], dict]:
    if type(derivative_diagnostics) is not bool:raise ValueError('ExplicitCarrierDiagnosticChoice')
    vertices = list(vertices); faces = list(faces); edges = list(edges)
    metric_cache: dict[tuple[int, bytes], np.ndarray] = {}
    raw_metric_at = metric_at

    def metric_at(point: np.ndarray, sheet_ref: int) -> np.ndarray:
        key = (int(sheet_ref), np.asarray(point, dtype="<f8").tobytes())
        if key not in metric_cache:
            metric_cache[key] = raw_metric_at(point, sheet_ref)
        return metric_cache[key]
    original_faces = len(faces)
    original_vertices = len(vertices)
    root_ids = list(range(len(faces)))
    depths = [0] * len(faces)
    feature_map = {_edge_key(*edge.vertices): edge for edge in edges}
    root_areas = []
    for face in faces:
        p = np.asarray([vertices[i].point for i in face.vertices])
        root_areas.append(.5 * float(np.linalg.norm(np.cross(p[1]-p[0], p[2]-p[0]))))
    risk_roots: set[int] = set(); closure_roots: set[int] = set()
    level_rows = []
    stopped_inadequate = []
    final_assessments = []
    decisions=[];validation=[];construction_refusal=None
    assessment_cache={};assessment_calls=0;assessment_hits=0;assessment_seconds=0.
    # Green closure can create a child whose risk depth is lower than the
    # current global iteration.  Allow enough deterministic iterations for
    # every such child to consume its own bounded risk depth.
    for level in range(3 * policy.maximum_depth + 2):
        assessment_metric = metric_at
        transition = {"enabled": False, "active_faces": 0, "changed_vertices": 0}
        if interpolation_compatible_transition:
            nodal, transition = promote_face_probe_metrics(
                authority, vertices, faces, metric_at, policy)
            lookup = {(int(v.sheet_ref), np.asarray(v.point, dtype="<f8").tobytes()): nodal[i]
                      for i, v in enumerate(vertices)}
            def assessment_metric(point: np.ndarray, sheet_ref: int) -> np.ndarray:
                return lookup.get((int(sheet_ref), np.asarray(point, dtype="<f8").tobytes()),
                                  metric_at(point, sheet_ref))
        assessments=[];t=time.perf_counter()
        for face in faces:
            key=(face.sheet_ref,tuple(face.vertices))
            # A row is reusable only if face geometry and nodal metrics are unchanged.
            if not interpolation_compatible_transition and key in assessment_cache:
                row=assessment_cache[key];assessment_hits+=1
            else:
                row=assess_carrier_face(authority,vertices,face,assessment_metric,policy,False,
                                        derivative_diagnostics=derivative_diagnostics)
                assessment_calls+=1
                if not interpolation_compatible_transition:assessment_cache[key]=row
            assessments.append(row)
        assessment_seconds+=time.perf_counter()-t
        final_assessments = assessments
        marked = []
        blocked = []
        for i, row in enumerate(assessments):
            if not row["inadequate"]:
                continue
            selected=refinement_edges(vertices,faces[i],policy)
            reason=('MaximumDepth' if depths[i]>=policy.maximum_depth else
                    'NoEligibleEdge' if not selected else 'OrdinaryRedSplit' if len(selected)==3 else 'ProtectedShortEdgeLongestSplit')
            decisions.append(dict(level=level,face_id=i,root_face_id=root_ids[i],depth=depths[i],
                sheet_ref=int(faces[i].sheet_ref),vertex_ids=list(faces[i].vertices),
                coordinates_m=[vertices[v].point.tolist() for v in faces[i].vertices],
                predicates=row,minimum_edge_guard_m=policy.minimum_edge_m,maximum_depth=policy.maximum_depth,
                selected_edges=[list(e) for e in selected],refinement_eligible=reason not in ('MaximumDepth','NoEligibleEdge'),reason=reason))
            if depths[i] < policy.maximum_depth and selected:
                marked.append(i); risk_roots.add(root_ids[i])
            else:
                blocked.append(i)
        level_rows.append({"level": level, "faces": len(faces),
                           "inadequate_faces": sum(x["inadequate"] for x in assessments),
                           "refined_risk_faces": len(marked),
                           "maximum_metric_edge_ratio": max(x["maximum_metric_edge_ratio"] for x in assessments),
                           "maximum_normal_interpolation_defect_deg": max(x["maximum_normal_interpolation_defect_deg"] for x in assessments),
                           "transition_promoted_faces": transition["active_faces"],
                           "transition_promoted_vertices": transition["changed_vertices"],
                           "bounded_unresolved_faces": len(blocked)})
        if progress is not None:
            progress(level_rows[-1])
        if evidence is not None:
            evidence(dict(schema='CarrierFaceDecisions/1',face_decisions=decisions,levels=level_rows,
                          construction_validation=validation))
        if blocked or not marked:
            break
        if level == 3 * policy.maximum_depth + 1:
            # Keep terminal geometry and terminal assessments in one generation.
            # Never commit a last subdivision that cannot be assessed here.
            construction_refusal=dict(code='CarrierIterationLimit',level=level)
            break
        split_edges = set()
        for index in marked:
            split_edges.update(refinement_edges(vertices,faces[index],policy))
        edge_faces = defaultdict(list)
        for i, face in enumerate(faces):
            a,b,c = face.vertices
            for key in (_edge_key(a,b), _edge_key(b,c), _edge_key(c,a)):
                edge_faces[key].append(i)
        old_vertices=list(vertices)
        midpoint = {}
        for key in sorted(split_edges):
            incident = edge_faces[key]
            sheets = sorted({faces[i].sheet_ref for i in incident})
            feature = feature_map.get(key)
            raw = .5 * (vertices[key[0]].point + vertices[key[1]].point)
            if feature is not None:
                point = authority.project_curve(feature.curve_ref, raw)
                sheet_ref = min(sheets)
            elif len(sheets) == 1:
                sheet_ref = sheets[0]
                point = authority.project_sheet(sheet_ref, raw)
            else:
                raise RuntimeError(f"semantic boundary edge {key} lacks persistent curve authority")
            midpoint[key] = len(vertices)
            vertices.append(SeedVertex(np.asarray(point), int(sheet_ref), 0, 0))
        new_faces=[]; new_roots=[]; new_depths=[];changed=[]
        marked_set=set(marked)
        for i, face in enumerate(faces):
            children=_split_face(face, midpoint)
            if len(children)>1:changed.extend(range(len(new_faces),len(new_faces)+len(children)))
            if len(children)>1 and i not in marked_set:
                closure_roots.add(root_ids[i])
            new_faces.extend(children)
            new_roots.extend([root_ids[i]]*len(children))
            new_depths.extend([depths[i]+int(i in marked_set)]*len(children))
        new_features=[]
        for key, edge in sorted(feature_map.items()):
            if key in midpoint:
                m=midpoint[key]
                new_features.extend((SeedEdge((key[0],m),edge.curve_ref,edge.ridge),
                                     SeedEdge((m,key[1]),edge.curve_ref,edge.ridge)))
            else:
                new_features.append(edge)
        try:
            validation.append(validate_split(authority,old_vertices,faces,vertices,new_faces,changed))
        except ValueError as error:
            construction_refusal=dict(code=str(error.args[0]),details=repr(error.args[1:]),level=level,
                proposed_vertices=len(vertices),proposed_faces=len(new_faces))
            vertices=old_vertices
            break
        faces, root_ids, depths, edges = new_faces, new_roots, new_depths, new_features
        feature_map = {_edge_key(*edge.vertices): edge for edge in edges}
    stopped_inadequate = [(i, row, root_ids[i], depths[i])
                          for i, row in enumerate(final_assessments)
                          if row["inadequate"]]
    total_area=sum(root_areas)
    receipt = {
        "enabled": True,
        "oracle_role": "adaptive sampled SDF proposal; not final acceptance authority",
        "normal_limit_deg": math.degrees(policy.normal_limit_rad),
        "normal_qualification_fraction": policy.normal_qualification_fraction,
        "metric_edge_ratio_limit": policy.allowed_metric_edge_ratio,
        "normal_interpolation_budget_deg": math.degrees(policy.normal_interpolation_budget_rad),
        "maximum_depth": policy.maximum_depth,
        "minimum_edge_m": policy.minimum_edge_m,
        "original_vertices": original_vertices, "original_faces": original_faces,
        "refined_vertices": len(vertices), "refined_faces": len(faces),
        "added_vertices": len(vertices)-original_vertices,
        "added_faces": len(faces)-original_faces,
        "risk_root_faces": len(risk_roots), "closure_root_faces": len(closure_roots-risk_roots),
        "risk_area_fraction": sum(root_areas[i] for i in risk_roots)/max(total_area,1e-300),
        "closure_area_fraction": sum(root_areas[i] for i in closure_roots-risk_roots)/max(total_area,1e-300),
        "maximum_reached_depth": max(depths) if depths else 0,
        "depth_distribution": {str(d): depths.count(d) for d in sorted(set(depths))},
        "unresolved_faces": len(stopped_inadequate),
        "levels": level_rows,
        "materialization_authorized": len(stopped_inadequate)==0 and construction_refusal is None,
        "face_decisions": decisions,
        "construction_validation": validation,
        "construction_refusal": construction_refusal,
        "assessment_instrumentation":dict(evaluated_faces=assessment_calls,unchanged_face_cache_hits=assessment_hits,seconds=assessment_seconds),
        "temporary_carriers_required": True,
        "interpolation_compatible_transition": interpolation_compatible_transition,
        "unique_point_metric_queries": len(metric_cache),
        **({} if derivative_diagnostics else {"derivative_diagnostics":dict(enabled=False,
            role='Optional descriptive derivative ratio; metric/normal/branch adequacy unchanged',
            omitted_face_probes=4*sum(row['faces'] for row in level_rows))}),
    }
    if terminal_capture is not None:
        # Capture the values actually used, without reevaluating the field.
        # Every returned generation has been assessed, including rollback.
        keys=[(int(v.sheet_ref),np.asarray(v.point,dtype='<f8').tobytes()) for v in vertices]
        if any(key not in metric_cache for key in keys):raise ValueError('CarrierTerminalMetricNotCaptured')
        base=np.asarray([metric_cache[key] for key in keys],dtype='<f8')
        represented=np.asarray(nodal if interpolation_compatible_transition else base,dtype='<f8')
        payload=dict(arrays=dict(coordinates=np.asarray([v.point for v in vertices],dtype='<f8'),
            triangles=np.asarray([f.vertices for f in faces],dtype='<i8'),
            vertex_metadata=np.asarray([(v.sheet_ref,v.chart,v.corner_ref) for v in vertices],dtype='<i8'),
            face_refs=np.asarray([f.sheet_ref for f in faces],dtype='<i8'),
            feature_edges=np.asarray([(*e.vertices,e.curve_ref,int(e.ridge)) for e in edges],dtype='<i8').reshape(-1,4),
            roots=np.asarray(root_ids,dtype='<i8'),depths=np.asarray(depths,dtype='<i8'),
            base_nodal_metrics=base,represented_nodal_metrics=represented),effective_policy=asdict(policy),
            representation=dict(kind='probe_promoted_nodal_tensors' if interpolation_compatible_transition else 'pointwise_nodal_tensors',
                carrier_interpolant='Arithmetic barycentric tensors at source-projected probes',
                native_consumer_equivalence_proved=False,continuous_majorant_proved=False,
                cache_dependencies='Held source and policy; immutable coordinates/sheet references; fixed nodal field. Any changed promoted node invalidates all incident faces.'),
            carrier_status=dict(materialization_authorized=receipt['materialization_authorized'],unresolved_faces=len(stopped_inadequate),
                                construction_refusal=construction_refusal,assessment_level=level_rows[-1]['level']))
        receipt['terminal_snapshot']=terminal_capture(payload)
    return vertices,faces,edges,receipt
