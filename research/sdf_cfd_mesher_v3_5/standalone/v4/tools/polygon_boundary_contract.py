"""Stored-source qualification premises; existing predicates/certifier own geometry.

Convex tangential halfspaces test the original submitted vertices. An incident
finite-curve witness can separately bound an outside vertex; membership is never
relabelled. No submitted coordinates, projection or meshing are changed.
"""
from dataclasses import dataclass
from fractions import Fraction as F
import copy
import hashlib
import json
import math
import re
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from canonical_polygon_authority import PolygonAuthority, validate_topology, sha, V4
from polygon_conversion import derive_conversion, enclosure, direction_bound
from shape_operator_support_envelope import _sqrt, _div, _add, _const
from sdf_cfd_mesher_v4.sdf.canonical import GeometryFailure
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel

exact = kernel.module('rank_geometry')


def scalar_batch_limit():
    """Use the native declaration, not a second numeric policy constant.

    V4CanonicalKernel/1 does not advertise limits. This source-backed path is
    for the repository-owned, identity-checked host; its native source is part
    of SourceBoundaryContract dependencies. The low-level guard is unchanged.
    """
    source=V4/'canonical_kernel/src/lib.rs'
    values=re.findall(r'^pub const MAX_BATCH: usize = ([0-9_]+);$',source.read_text(),re.M)
    if len(values)!=1 or int(values[0].replace('_',''))<=0:
        raise GeometryFailure('ScalarBatchLimitUnavailable','scalar_diagnostics',str(source))
    return int(values[0].replace('_',''))


def scalar_diagnostics(geometry,points):
    """All-or-error ordered aggregation; never expose a partial successful result."""
    limit=scalar_batch_limit();values=[];sizes=[]
    with geometry.owner.lock:
        actual=geometry.owner.request(op='describe',handle=geometry.handle)
        if actual!=geometry.description:
            raise GeometryFailure('GeometryDescriptionMismatch','scalar_diagnostics','native descriptor differs',geometry.identity)
        # Even empty input queries the live handle under the existing contract.
        for start in range(0,max(1,len(points)),limit):
            chunk=points[start:start+limit]
            if hasattr(chunk,'tolist'):chunk=chunk.tolist()
            result=geometry.query(chunk)
            if (not isinstance(result,list) or len(result)!=len(chunk) or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in result)):
                raise GeometryFailure('InvalidScalarDiagnosticResponse','scalar_diagnostics','count/type/nonfinite result',geometry.identity)
            values.extend(result);sizes.append(len(chunk))
    return values,sizes


def identity(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def rational(p):
    return tuple(F(float(x)) for x in p)


def geometry_state(a):
    return dict(scale=a.scale,points=a.unit.points.tolist(),point_names=a.unit.point_names,
        loops=a.unit.brep.loops,planes=[(n.tolist(),h) for n,h in a.unit.planes],
        references={n:getattr(a,n) for n in ('sheet_names','curve_names','corner_names')},
        curve_vertices=a.unit.curve_vertices)


def construct_record(a):
    if not isinstance(a,PolygonAuthority):
        raise GeometryFailure('UnsupportedSourceBoundaryRepresentation','source_boundary','admitted stored polygon Authority required')
    packet=a.geometry.stored_topology()
    if packet!=a.packet:
        raise GeometryFailure('SourceBoundaryIdentityMismatch','source_boundary','live topology differs',a.geometry_identity)
    v=validate_topology(packet,a.geometry.identity)
    # Reconstruct ONLY through the existing source-loop Authority. Detect changes
    # to cached coordinates, references or planes, not just an unchanged hash label.
    fresh=PolygonAuthority(a.geometry,packet)
    if geometry_state(fresh)!=geometry_state(a):
        raise GeometryFailure('SourceBoundaryIdentityMismatch','source_boundary','cached Authority differs from stored source',a.geometry_identity)
    conversion,branches=derive_conversion(a,v,packet['metres_per_model_unit'])
    conversion['accepted_for_admission']=True  # conversion evidence only, never a surface pass
    refs={n:copy.deepcopy(getattr(a,n)) for n in ('sheet_names','curve_names','corner_names')}
    deps={str(p):sha(p) for p in (Path(__file__),V4/'tools/polygon_conversion.py',
        V4/'tools/canonical_polygon_authority.py',V4/'tools/audit_adaptive_surface.py',
        V4/'canonical_kernel/src/lib.rs',V4/'src/sdf_cfd_mesher_v4/sdf/canonical.py',
        V4.parent/'sdf_cfd_mesher_v3/adaptive_surface_generic_qualification/continuous_geometry_certifier.py',
        V4.parent/'sdf_cfd_mesher_v3/adaptive_surface_generic_qualification/shape_operator_support_envelope.py')}
    deps.update(a.provenance)
    return dict(schema='StoredPolygonSourceBoundary/1',geometry_identity=a.geometry_identity,
        topology_identity=packet['topology_identity'],kernel_source_sha256=packet['kernel_source_sha256'],
        description=a.geometry.owner.request(op='describe',handle=a.geometry.handle),
        units='m',physical_unit_rational=str(F(str(packet['metres_per_model_unit']))),
        unit_convention='exact rational of caller scale decimal round-trip; 0.001 means exactly 1/1000',
        premises=a.checks,source_dependencies=deps,exact_predicates_identity=kernel.VERSION,
        source_state_identity=identity(geometry_state(a)),references=refs,
        domains=dict(sheet_loops=a.unit.brep.loops,curve_vertices=a.unit.curve_vertices,
            corner_stars={k:val['ordered_curves'] for k,val in a.unit.brep.corners.items()}),
        conversion=conversion,branch_bounds=branches,
        target='stored mathematical polygon boundary, not quantized evaluator zero set',
        live_sdf_samples_not_continuous_authority=True,
        numerical_scope='positive-aligned face/branch directions with exact sin^2 < 1/4; finite-segment endpoint enclosures through the unchanged rational-lifted projector; other cases unresolved',
        finite_panel_scope='strict orthogonal footprints or exact incident finite-curve witnesses; convex interpolation of per-vertex witnesses; no roundoff attribution or distance cutoff',
        production_surface_qualified=False)


@dataclass
class SourceBoundaryContract:
    record: dict
    accuracy_limits: dict | None = None

    @classmethod
    def automatic(cls,a,accuracy_limits=None):
        result=cls(construct_record(a),copy.deepcopy(accuracy_limits))
        result.check_limits()
        return result

    @property
    def sheet_names(self):return self.record['references']['sheet_names']
    @property
    def curve_names(self):return self.record['references']['curve_names']
    @property
    def corner_names(self):return self.record['references']['corner_names']
    @property
    def expected_components(self):return 1
    @property
    def expected_euler(self):return 2
    @property
    def expected_boundary_loops(self):return 0
    @property
    def projection_tolerance_m(self):return 2e-12
    @property
    def position_limit_m(self):return None if self.accuracy_limits is None else self.accuracy_limits['position_m']
    @property
    def normal_limit_rad(self):return None if self.accuracy_limits is None else self.accuracy_limits['normal_rad']
    @property
    def premise(self):return self.record['target']+'; finite convex domains and incident branches required'
    @property
    def source(self):return 'automatic stored topology: '+self.record['topology_identity']

    def check_limits(self):
        if self.accuracy_limits is not None:
            if (set(self.accuracy_limits)!={'position_m','normal_rad'} or
                any(type(x) not in (int,float) or not math.isfinite(x) or x<=0 for x in self.accuracy_limits.values())):
                raise GeometryFailure('InvalidAccuracyRequirements','source_boundary','positive finite position_m and normal_rad required')

    def validate(self):
        self.check_limits()
        if self.accuracy_limits is None:
            raise GeometryFailure('AccuracyRequirementsMissing','source_boundary','acceptance not evaluated')

    def verify(self,a):
        self.check_limits()
        if self.record!=construct_record(a):
            raise GeometryFailure('SourceBoundaryEvidenceMismatch','source_boundary','stale, altered or mismatched derivation',a.geometry_identity)


def curve_endpoint_enclosure(a,view,ref,p,ends):
    """Enclose the EXISTING finite projector, never move qualification input.

    The unchanged CompleteAuthority method accepts rational arrays. Executing it
    on the materialized SI source segment provides an exact clamped witness.
    Its domain is checked by the existing exact predicate. Uniform similarity
    leaves this mathematical segment projection unchanged; rounding does not.
    """
    P=rational(p)
    witness=tuple(a.unit.project_curve.__func__(view,ref,np.array(P,dtype=object)))
    if not exact.onsegment(witness,*ends):
        raise GeometryFailure('FiniteCurveWitnessFailure','source_boundary_qualification',ref,a.geometry_identity)
    difference=exact.sub(P,witness);d2=exact.dot(difference,difference)
    witness_record=[str(q) for q in witness]
    if d2==0:
        return dict(exact_member=True,materialized_distance_upper_m=0.,computed_residual_m=None,
                    enclosure_correction_m=None,endpoint_bound_m=0.,distance_squared_exact='0',witness_exact_m=witness_record)
    try:upper=float(_sqrt(enclosure(d2))[1])
    except (ArithmeticError,OverflowError):return None
    if not math.isfinite(upper) or F(upper)**2<d2:
        raise GeometryFailure('NumericalEnclosureFailure','source_boundary_qualification',ref,a.geometry_identity)
    measured=float(a.feature_residual(ref,p))
    if not math.isfinite(measured):
        return None
    correction=max(F(0),F(upper)-F(measured))
    delta=0. if correction==0 else float(enclosure(correction)[1])
    bound=float(_add(_const(measured),_const(delta))[1])
    return dict(exact_member=False,materialized_distance_upper_m=upper,computed_residual_m=measured,
                enclosure_correction_m=delta,endpoint_bound_m=bound,distance_squared_exact=str(d2),witness_exact_m=witness_record)


def panel_curve_witness(a,view,ref,p,ends,domain):
    """Connect an existing finite-segment enclosure to an exact finite panel.

    Source incidence is required separately by the caller. The materialized
    segment endpoints and witness must lie in this SAME exact panel, not merely
    in the rounded Authority plane. No proximity or magnitude rule is used.
    """
    N,origin,hs=domain
    def member(q):return exact.dot(N,exact.sub(q,origin))==0 and bool(exact.clip_polygon([q],hs))
    if not all(member(q) for q in ends):
        raise GeometryFailure('FinitePanelWitnessFailure','source_boundary_qualification','source segment not in materialized panel',a.geometry_identity)
    row=curve_endpoint_enclosure(a,view,ref,p,ends)
    if row is None:return None
    witness=tuple(map(F,row['witness_exact_m']))
    if not exact.onsegment(witness,*ends) or not member(witness):
        raise GeometryFailure('FinitePanelWitnessFailure','source_boundary_qualification','invalid finite witness',a.geometry_identity)
    delta=exact.sub(rational(p),witness)
    if F(row['endpoint_bound_m'])**2<exact.dot(delta,delta):
        raise GeometryFailure('NumericalEnclosureFailure','source_boundary_qualification','witness displacement not enclosed',a.geometry_identity)
    return row


def enforce_and_compose(a,contract,x,faces,face_refs,curve_edges,corner_refs):
    """Called by the actual V4 qualifier for every submitted surface, not seed-only."""
    from historical_capsule_surface import provider
    from continuous_geometry_certifier import certify_face
    contract.verify(a)
    x=np.asarray(x,float);faces=np.asarray(faces);face_refs=np.asarray(face_refs)
    ce=np.asarray(curve_edges);cr=np.asarray(corner_refs)
    def fail(code,detail):raise GeometryFailure(code,'source_boundary_qualification',detail,a.geometry_identity)
    if (x.ndim!=2 or x.shape[1]!=3 or not np.isfinite(x).all() or faces.ndim!=2 or faces.shape[1]!=3
        or faces.dtype.kind not in 'iu' or np.any(faces<0) or np.any(faces>=len(x))
        or face_refs.shape!=(len(faces),) or face_refs.dtype.kind not in 'iu' or ce.ndim!=2 or ce.shape[1]!=3 or ce.dtype.kind not in 'iu'
        or np.any(ce[:,:2]<0) or np.any(ce[:,:2]>=len(x)) or cr.shape!=(len(x),) or cr.dtype.kind not in 'iu'):
        fail('InvalidSurfaceQueryBuffers','coordinates or reference dimensions/indices')
    if (set(face_refs)!=set(a.sheet_names) or set(ce[:,2])!=set(a.curve_names)
        or set(cr)-{0}!=set(a.corner_names)):
        fail('QualificationReferenceCoverageFailure','sheet/curve/corner coverage differs')
    points=a.unit.points*a.scale
    Q=[rational(p) for p in points];queries=[rational(p) for p in x]
    lookup={k:i for i,k in enumerate(a.unit.point_names)}
    domains={}
    for ref,name in a.unit.sheet_names.items():
        loop=[Q[lookup[k]] for k in a.unit.brep.loops[name]]
        N=exact.cross(exact.sub(loop[1],loop[0]),exact.sub(loop[2],loop[0]))
        hs=[(exact.cross(N,exact.sub(loop[(i+1)%len(loop)],p)),p) for i,p in enumerate(loop)]
        # Source loop/segment coordinates use one materialization. Verify exact
        # planarity and convexity rather than assuming rounded planes coincide.
        if (not exact.nonzero(N) or any(exact.dot(N,exact.sub(p,loop[0]))!=0 or
            not exact.clip_polygon([p],hs) for p in loop)):
            fail('FinitePanelPremiseFailure',dict(sheet=ref,reason='exact materialized planarity/convexity'))
        domains[ref]=(N,loop[0],hs)
    incidence={};incident_curves=[set() for _ in x];curve_uses={}
    for f,ref in zip(faces,face_refs):
        for p,q in zip(f,np.roll(f,-1)):incidence.setdefault(tuple(sorted((int(p),int(q)))),set()).add(int(ref))
    sheet_ids={name:ref for ref,name in a.unit.sheet_names.items()}
    for p,q,ref in ce:
        p,q,ref=int(p),int(q),int(ref)
        incident_curves[p].add(ref);incident_curves[q].add(ref)
        curve_uses.setdefault(ref,[]).append((p,q))
    rational_view=SimpleNamespace(points=np.array(Q,dtype=object),curve_vertices=a.unit.curve_vertices)
    failures=[];bounds=[];counts={};residual_max=0.;unresolved=[]
    E=contract.record['conversion']['source_to_materialized_affine_L1_bound_m']['upper_binary64']
    branch={r['ref']:r['normal_angle_upper_rad'] for r in contract.record['branch_bounds']}
    for i,(f,ref) in enumerate(zip(faces,face_refs)):
        ref=int(ref);N,origin,hs=domains[ref]
        # Halfspaces are tangent to the finite panel. Testing unmodified points
        # establishes membership of their mathematical orthogonal footprints.
        # No exact coplanarity/slack is demanded; plane distance is handled below.
        bad=[int(j) for j in f if not exact.clip_polygon([queries[int(j)]],hs)]
        witnesses=[];unsupported=[];unowned=[]
        for j in bad:
            # Corners keep complete-star/exact-restoration semantics. Unlabelled
            # sheet vertices have no general nearest-panel fallback.
            if cr[j] or len(incident_curves[j])!=1:unowned.append(j);continue
            curve=next(iter(incident_curves[j]));indices=a.unit.curve_vertices[curve]
            keys=tuple(sorted(a.unit.point_names[k] for k in indices))
            expected={sheet_ids[u[0]] for u in a.unit.brep.edges[keys]}
            if ref not in expected or any(incidence.get(tuple(sorted((p,q))))!=expected
                for p,q in curve_uses[curve] if j in (p,q)):
                fail('SourceCurveIncidenceFailure',dict(face=i,vertex=j,curve=curve,sheet=ref))
            row=panel_curve_witness(a,rational_view,curve,x[j],[Q[k] for k in indices],domains[ref])
            if row is None:
                unsupported.append(j)
            else:witnesses.append(dict(vertex=j,curve=curve,route='incident_finite_curve_witness',**row))
        if unowned:failures.append(dict(face=i,sheet=ref,vertices=unowned));continue
        if unsupported:
            unresolved.append(dict(face=i,operation='finite_panel_witness',vertices=unsupported,reason='existing finite-segment enclosure unavailable'))
            continue
        tri=x[f];status,pos,angle,method=certify_face(a,ref,tri)
        counts[status]=counts.get(status,0)+1
        values=[abs(a.implicit_value(ref,p)) for p in tri];residual_max=max(residual_max,*values)
        # Exact residual of the real materialized finite patch plane. The affine
        # maximum occurs at a triangle vertex, as in the existing K=0 certificate.
        r=max(abs(exact.dot(N,exact.sub(queries[int(j)],origin))) for j in f)
        distance=0. if r==0 else float(_div(enclosure(r),_sqrt(enclosure(exact.dot(N,N))))[1])
        if F(distance)**2*exact.dot(N,N)<r*r:fail('NumericalEnclosureFailure',i)
        # Strict vertices use orthogonal footprints; outside curve vertices use
        # the verified witnesses above. Convex combinations stay in one finite
        # convex panel. This is a max, not a sum of perpendicular/lateral errors.
        distance=max([distance]+[row['endpoint_bound_m'] for row in witnesses])
        correction=max(F(0),F(distance)-F(float(pos))) if math.isfinite(pos) else None
        if correction is None:unresolved.append(dict(face=i,operation='position',reason=status));continue
        correction_upper=0. if correction==0 else float(enclosure(correction)[1])
        position=float(_add(_add(_const(pos),_const(correction_upper)),_const(E))[1])
        T=exact.cross(exact.sub(queries[int(f[1])],queries[int(f[0])]),exact.sub(queries[int(f[2])],queries[int(f[0])]))
        # Reuse the existing rational cross/dot direction enclosure on the actual
        # triangle, not only on exactly parallel planes. Exact edge subtraction
        # and cross products bypass floating normalization/acos roundoff. The
        # correction is an a posteriori enclosure gap, not a libm error estimate.
        try:ne=direction_bound(rational(a.unit.planes[ref-1][0]),T)
        except (ArithmeticError,OverflowError):ne=None
        delta_upper=None
        if ne is None or not math.isfinite(angle):
            normal=None;unresolved.append(dict(face=i,operation='normal_evaluation',reason='direction enclosure requires nonzero, positive-aligned vectors, exact sin^2 < 1/4, finite interval operations and computed angle'))
        else:
            delta=max(F(0),F(ne)-F(float(angle)))
            delta_upper=0. if delta==0 else float(enclosure(delta)[1])
            normal=float(_add(_add(_const(angle),_const(delta_upper)),_const(branch[ref]))[1])
        bounds.append(dict(face=i,sheet=ref,method=method,legacy_status=status,owning_residual_m=max(values),
            strict_footprint_outside_vertices=bad,
            exact_panel_member_vertices=[int(j) for j in f if j not in bad and exact.dot(N,exact.sub(queries[int(j)],origin))==0],
            position_proof_route='convex_vertex_witnesses' if witnesses else 'strict_orthogonal_footprints',
            panel_witnesses=witnesses,materialized_panel_distance_upper_m=distance,
            computed_position_bound_m=pos,position_numeric_correction_m=correction_upper,
            affine_remainder_m=0.,source_conversion_m=E,source_position_bound_m=position,
            computed_normal_bound_rad=angle,normal_numeric_correction_rad=delta_upper,
            exact_triangle_to_branch_enclosure_rad=ne,
            branch_to_source_normal_rad=branch[ref],branch_variation_rad=0.,source_normal_bound_rad=normal))
    if failures:fail('FiniteSheetDomainFailure',dict(faces_checked=len(faces),failures=failures))
    # Existing curve graph/topology checks, plus source ancestry incidence binding.
    topology=provider.topology_report(x,faces,face_refs,ce,None,a)
    if not topology['topology_pass']:fail('SurfaceTopologyFailure',topology)
    curve_bounds=[];curve_evidence=[]
    for p,q,ref in ce:
        p,q,ref=int(p),int(q),int(ref)
        indices=a.unit.curve_vertices[ref];keys=tuple(sorted(a.unit.point_names[j] for j in indices))
        expected={sheet_ids[u[0]] for u in a.unit.brep.edges[keys]}
        if incidence.get(tuple(sorted((p,q))))!=expected:fail('SourceCurveIncidenceFailure',dict(edge=[p,q],curve=ref))
        incident_curves[p].add(ref);incident_curves[q].add(ref)
        ends=[Q[j] for j in indices]
        endpoint_rows=[curve_endpoint_enclosure(a,rational_view,ref,x[j],ends) for j in (p,q)]
        if all(r is not None for r in endpoint_rows):
            distance=max(r['endpoint_bound_m'] for r in endpoint_rows)
            bound=E if distance==0 else float(_add(_const(distance),_const(E))[1])
            curve_bounds.append(bound)
            # Distance to a convex finite segment is convex. Corresponding
            # clamped endpoint witnesses interpolate within that same segment.
            curve_evidence.append(dict(curve=ref,edge=[p,q],endpoints=endpoint_rows,source_bound_m=bound))
        else:
            unresolved.append(dict(curve=ref,operation='finite_curve',reason='finite interval enclosure or existing feature residual unavailable'))
    restored=x.copy()
    try:names=a.restore_corners(restored,incident_curves)
    except ValueError as error:fail('SourceCornerIncidenceFailure',str(error))
    for i,ref in enumerate(cr):
        if ref:
            key=a.unit.corner_names[int(ref)]
            if len(incident_curves[i])<2 or not np.array_equal(x[i],restored[i]) or not np.array_equal(x[i],points[lookup[key]]):
                fail('SourceCornerIdentityFailure',dict(vertex=i,corner=int(ref)))
        elif len(incident_curves[i])>1:fail('SourceCornerIdentityFailure',dict(vertex=i,reason='corner reference absent'))
    supplied=contract.accuracy_limits is not None
    limits_ok=(supplied and not unresolved and all(b['source_position_bound_m']<=contract.position_limit_m
        and b['source_normal_bound_rad']<=contract.normal_limit_rad and b['legacy_status']=='Certified' for b in bounds)
        and all(b<=contract.position_limit_m for b in curve_bounds))
    sample=np.concatenate([x,x[faces].mean(1)])
    values,chunk_sizes=scalar_diagnostics(a.geometry,sample)
    return dict(geometry_identity=a.geometry_identity,topology_identity=a.packet['topology_identity'],
        contract_identity=identity(contract.record),triangles_checked=len(faces),finite_domain_failures=0,
        topology=topology,curve_count=len(ce),corner_count=len(names),bounds=bounds,
        position_bound_max_m=max((b['source_position_bound_m'] for b in bounds),default=None),
        normal_bound_max_rad=max((b['source_normal_bound_rad'] for b in bounds if b['source_normal_bound_rad'] is not None),default=None),
        curve_source_bound_max_m=max(curve_bounds,default=None),curve_evidence=curve_evidence,unresolved=unresolved,
        residual_gate=dict(quantity='abs(owning Authority implicit_value)',limit_m=2e-12,max_m=residual_max,passed=residual_max<=2e-12),
        accuracy_limits=contract.accuracy_limits,acceptance='not_evaluated' if not supplied else ('accepted' if limits_ok and residual_max<=2e-12 else 'rejected'),
        geometry_requirements_passed=bool(limits_ok and residual_max<=2e-12),
        intersections='not evaluated by this geometry substage',passed=False,
        scalar_diagnostics=dict(locations_m=sample.tolist(),values_m=values,count=len(values),chunk_sizes=chunk_sizes,maximum_abs_m=max(map(abs,values)),
            live_sdf_samples_not_continuous_authority=True,quantized_callback=True))
