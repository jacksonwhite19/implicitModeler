"""Representation binding to retained V3 source-loop operations; no wing formulas."""
from fractions import Fraction
import struct
import math
import ast
import hashlib
import importlib.util
import json
import sys
import types
import subprocess
import time
import shutil
from pathlib import Path
from collections import Counter
import numpy as np
from sdf_cfd_mesher_v4.sdf.canonical import GeometryFailure

V4=Path(__file__).resolve().parents[1]
V3=V4.parent/'sdf_cfd_mesher_v3'
COMPLETE=V3/'complete_short_wing_qualification_2026-09-06/complete_authority.py'
CORNER=V3/'finite_wing_trailing_tip_qualification_2026-09-06/corner_authority.py'
EXPECTED={COMPLETE:'7a948f98da2daff9e6d347fe019b5a4e22828ecab244e666422547fb09d4a212',
          CORNER:'4925f98ba660448681d6cbf982aa0ee6a7b9ec9f021afc5dbfea535ee877e680'}


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def legacy_class(source, output=None):
    """Load the unchanged class without executing study.py or old main programs.

    Only dependency and I/O globals are injected. The class AST, including its
    unit-chord assertion and all operations, is compiled without rewriting.
    A fresh namespace per binding prevents shared mutable geometry/output state.
    """
    from historical_capsule_surface import Authority, SeedVertex, SeedFace
    from procedural_authorities import SeedEdge, tangent_metric, HMAX
    for p,h in EXPECTED.items():
        if sha(p)!=h: raise GeometryFailure('LegacySourceChanged','polygon_binding',str(p))
    spec=importlib.util.spec_from_file_location('_v4_polygon_corner',CORNER)
    corner=importlib.util.module_from_spec(spec);spec.loader.exec_module(corner)
    node=next(n for n in ast.parse(COMPLETE.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='CompleteAuthority')
    def write(path,value):
        Path(path).write_text(json.dumps(value,indent=2,default=lambda x:x.tolist())+'\n')
    def wsl(path):
        p=str(Path(path).resolve());return '/mnt/'+p[0].lower()+p[2:].replace('\\','/')
    seed_start=time.monotonic()
    def linux(command,tag,seconds):
        if output is None: raise GeometryFailure('SeedOutputRequired','labelled_chart_seed','explicit task output')
        used=sum(p.stat().st_size for p in Path(output).rglob('*') if p.is_file())
        if used>8*1024*1024 or shutil.disk_usage(output).free<8*1024*1024:
            raise GeometryFailure('StorageBudgetExceeded','labelled_chart_seed',used)
        remaining=300-(time.monotonic()-seed_start)
        if remaining<=0:raise GeometryFailure('SeedResourceDeadline','labelled_chart_seed','300 seconds')
        return subprocess.run(['wsl','-e',*command],capture_output=True,text=True,timeout=min(seconds,remaining,30))
    module=types.ModuleType('_v4_existing_complete_authority');module.__file__=str(COMPLETE)
    ns=module.__dict__;ns.update(np=np,Counter=Counter,Authority=Authority,SeedVertex=SeedVertex,
        SeedFace=SeedFace,SeedEdge=SeedEdge,tangent_metric=tangent_metric,HMAX=HMAX,corner=corner,
        HERE=Path(output) if output else Path('.'),V3=V3,read=lambda path:{'authority':source},
        write=write,wsl=wsl,linux=linux)
    # Registration is only for inspect/provenance. Instances retain their own globals.
    sys.modules[module.__name__]=module
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(COMPLETE),'exec'),ns)
    return ns['CompleteAuthority'],corner


def f32(bits):
    if type(bits) is not int or not 0 <= bits <= 0xffffffff:
        raise ValueError('InvalidBinary32Bits')
    return struct.unpack('<f', struct.pack('<I', bits))[0]


def validate_topology(packet, geometry_identity):
    """Exact checks on the stored polygon, not feature discovery or certification."""
    def reject(code, detail):
        raise GeometryFailure(code, 'polygon_representation', detail, geometry_identity)
    if packet.get('geometry_identity') != geometry_identity:
        reject('GeometryIdentityMismatch', 'stored topology differs from handle')
    data=packet.get('data',{})
    if data.get('status') != 'available':
        reject('UnsupportedStoredRepresentation', data.get('reason'))
    if data.get('representation') != 'StoredPolygonSectionLoft/1':
        reject('UnsupportedStoredRepresentation', data.get('representation'))
    sections=data.get('sections',[])
    if len(sections)<2: reject('UniformExtrusionPremiseFailed','fewer than two sections')
    try:
        if any(len(s['position_bits'])!=3 or any(len(p)!=2 for p in s['point_bits']) for s in sections):
            raise ValueError('StoredTupleDimensions')
        decoded=[dict(point_bits=s['point_bits'], chord=f32(s['chord_bits']),
            position=[f32(b) for b in s['position_bits']], twist=f32(s['twist_bits'])) for s in sections]
        points=[tuple(f32(b) for b in p) for p in decoded[0]['point_bits']]
    except (ValueError, KeyError, TypeError, struct.error) as e:
        reject('InvalidStoredTopology',str(e))
    if any(not math.isfinite(x) for s in decoded for x in [s['chord'],s['twist'],*s['position']]) or any(not math.isfinite(x) for p in points for x in p):
        reject('NonfiniteStoredTopology','source values')
    first=decoded[0];chord=first['chord'];scale=packet.get('metres_per_model_unit')
    if not isinstance(scale,(int,float)) or not math.isfinite(scale) or scale<=0 or chord<=0 or not math.isfinite(chord*scale):
        reject('InvalidUnitScale','positive finite stored chord and model-unit scale required')
    for i,s in enumerate(decoded):
        if s['point_bits']!=first['point_bits'] or s['chord']!=chord or s['twist']!=0 or s['position'][0]!=0 or s['position'][2]!=0:
            reject('UniformExtrusionPremiseFailed',dict(section=i,required='identical points/chords, zero twist and x/z placement'))
        if sections[i]['source_section_index']!=i or (i and s['position'][1]<=decoded[i-1]['position'][1]):
            reject('UniformExtrusionPremiseFailed',dict(section=i,required='strictly ordered finite y positions'))
    if len(points)<4 or points[0]!=points[-1]:
        reject('InvalidSectionClosure','explicit repeated endpoint required')
    p=points[:-1];n=len(p)
    if len(set(p))!=n: reject('DegenerateSection','repeated non-closing source vertex')
    if n*n>1_000_000:
        reject('TopologyCheckResourceLimit','at most 1000000 exact half-plane tests per binding; no approximation fallback')
    # Rational signs of the exact stored binary floats. No geometric tolerance.
    q=[tuple(Fraction(x) for x in v) for v in p]
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    area=sum(q[i][0]*q[(i+1)%n][1]-q[(i+1)%n][0]*q[i][1] for i in range(n))
    if not area: reject('DegenerateSection','zero exact signed area')
    sign=1 if area>0 else -1
    failures=[];collinear=0
    for i in range(n):
        if q[i]==q[(i+1)%n]: reject('DegenerateSection',dict(edge=i))
        for j in range(n):
            value=sign*cross(q[i],q[(i+1)%n],q[j])
            if value<0:
                failures.append(dict(edge=i,vertex=j,oriented_cross=float(value),exact_cross=str(value)))
            elif value==0 and j not in (i,(i+1)%n): collinear+=1
    if failures:
        reject('ConvexOrientationPremiseFailed',dict(failures=failures,checks=n*n,
            existing_implementation='corner_authority.orient_convex',geometry_changed=False))
    return dict(sections=decoded,points=points,segments=n,length_scale_m=chord*scale,
        checks=dict(section_count=len(decoded),stored_points=len(points),segments=n,
            explicit_closure=True,nondegenerate=True,convex_exact_halfplane_checks=n*n,
            convex_failures=0,collinear_nonendpoint_checks=collinear,signed_area_normalized=float(area/2),
            uniform_extrusion=True,frame='identity, span Y',
            expected_panels=n,expected_caps=2,expected_curves=3*n,expected_corners=2*n,
            span_model=decoded[-1]['position'][1]-decoded[0]['position'][1]))


class PolygonAuthority:
    """SI similarity wrapper around the existing unit-chord affine Authority.

    These are exact-branch operations of the promoted stored polygon, NOT an
    assertion that canonical f32 distance evaluates the same mathematical field.
    No default sizing or continuous canonical-consistency premise is supplied.
    """
    def __init__(self, geometry, packet):
        v=validate_topology(packet,geometry.identity)
        self.geometry=geometry;self.geometry_identity=geometry.identity
        self.packet=packet;self.scale=v['length_scale_m'];self.checks=v['checks']
        chord=v['sections'][0]['chord']
        self.unit_source={'sections':[dict(point_bits=s['point_bits'],chord=1.,twist=0.,
            position=[0.,s['position'][1]/chord,0.]) for s in v['sections']]}
        cls,self.corner=legacy_class(self.unit_source)
        try:self.unit=cls()
        except (AssertionError,ValueError) as e:
            raise GeometryFailure('SourceLoopPremiseFailed','polygon_binding',str(e),self.geometry_identity) from e
        prefix='stored-polygon/'+packet['topology_identity']
        for name in ('sheet_names','curve_names','corner_names'):
            setattr(self,name,{k:prefix+'/'+v for k,v in getattr(self.unit,name).items()})
        b=self.unit.brep;n=v['segments']
        # Independent source-ancestry coverage; no mesh or proximity classification.
        expected={frozenset(f'live.section/node/{i}/{side}' for i in range(n)) for side in ('negative','positive')}
        expected.update(frozenset(f'live.section/node/{j}/{side}' for j in (i,(i+1)%n)
            for side in ('negative','positive')) for i in range(n))
        actual=[frozenset(loop) for loop in b.loops.values()]
        covered=set(actual)==expected and len(actual)==len(expected)
        self.checks.update(sheets=len(b.loops),curves=len(b.edges),corners=len(b.corners),
            complete_source_loops=covered,opposite_shared_edge_uses=all(len(u)==2 and u[0][1:]==u[1][1:][::-1] for u in b.edges.values()),
            complete_corner_stars=all(set(c['ordered_sheets'])==set(b.incident[k]) for k,c in b.corners.items()),
            euler=len(b.points)-len(b.edges)+len(b.loops),root_wall=False,internal_section_guides=len(v['sections'])-2)
        if not covered or self.checks['euler']!=2 or not self.checks['opposite_shared_edge_uses'] or not self.checks['complete_corner_stars']:
            raise GeometryFailure('SourceCoverageFailure','polygon_binding',self.checks,self.geometry_identity)
        self.provenance={str(p):h for p,h in EXPECTED.items()}
        self.canonical_consistency='promoted stored-source uniform extrusion; canonical f32 field relationship unqualified'

    def scalar_value(self, points):return np.asarray(self.geometry.query(np.asarray(points).tolist()))
    def numerical_project(self,points):return self.geometry.numerical_project(points)
    def numerical_normal(self,points):return self.geometry.numerical_normal(points)
    def feature_inventory(self):return self.geometry.feature_inventory()
    def feature_evaluate(self,requests):return self.geometry.feature_evaluate(requests)
    def feature_query(self,points,options=None):return self.geometry.feature_query(points,options)
    def project_sheet(self,ref,p):return self.unit.project_sheet(ref,np.asarray(p)/self.scale)*self.scale
    def project_curve(self,ref,p):return self.unit.project_curve(ref,np.asarray(p)/self.scale)*self.scale
    def exact_normal(self,ref,p):return self.unit.exact_normal(ref,np.asarray(p)/self.scale)
    def implicit_value(self,ref,p):return self.unit.implicit_value(ref,np.asarray(p)/self.scale)*self.scale
    def feature_residual(self,ref,p):return self.unit.feature_residual(ref,np.asarray(p)/self.scale)*self.scale
    def gradient_hessian(self,ref,p):
        g,h=self.unit.gradient_hessian(ref,np.asarray(p)/self.scale);return g,h/self.scale
    def curvature_upper(self,ref,p):return self.unit.curvature_upper(ref,np.asarray(p)/self.scale)/self.scale
    def _normal_map_principal_data(self,ref,p):
        u,v,a,b,n=self.unit._normal_map_principal_data(ref,np.asarray(p)/self.scale)
        return u,v,a/self.scale,b/self.scale,n
    def competing_component_sheet_refs(self,ref):return self.unit.competing_component_sheet_refs(ref)

    def restore_corners(self,points,incident_curves):
        # The legacy helper identifies corners by complete source curve stars.
        q=np.asarray(points).copy()/self.scale
        restored=self.corner.restore_persistent_points(self.unit,q,incident_curves)
        for i,refs in enumerate(incident_curves):
            if len(refs)>=2:points[i]=q[i]*self.scale
        return restored

    def labelled_seed(self,authority,policy,output):
        """Existing boundary-only CDT, not implicit extraction or adaptive meshing."""
        from procedural_authorities import SeedVertex
        if authority is not self:raise GeometryFailure('GeometryIdentityMismatch','labelled_chart_seed','wrong authority',self.geometry_identity)
        if self.geometry.stored_topology()!=self.packet:
            raise GeometryFailure('GeometryIdentityMismatch','labelled_chart_seed','live topology changed',self.geometry_identity)
        output=Path(output)/'labelled_seed';output.mkdir(parents=True,exist_ok=False)
        # Bounded resource guard, not a scientific length or quality threshold.
        budget=8*1024*1024
        if shutil.disk_usage(output).free<budget:raise GeometryFailure('StorageBudgetExceeded','labelled_chart_seed',budget,self.geometry_identity)
        start=time.perf_counter();cls,_=legacy_class(self.unit_source,output);a=cls()
        vertices,faces,edges=a.seed(None,None,None)
        vertices=[SeedVertex(v.point*self.scale,v.sheet_ref,v.chart,v.corner_ref) for v in vertices]
        used=sum(p.stat().st_size for p in output.rglob('*') if p.is_file())
        if used>budget:raise GeometryFailure('StorageBudgetExceeded','labelled_chart_seed',used,self.geometry_identity)
        return dict(coordinates=np.array([v.point for v in vertices]),triangles=np.array([f.vertices for f in faces]),
            vertices=vertices,faces=faces,edges=edges,timing={'seconds':time.perf_counter()-start},seed_size_m=None,
            seed_route='ExistingCGALConstrainedSourceLoopCharts',controls='source loop vertices and boundary constraints only; no size parameter',
            output_bytes=used,geometry_identity=self.geometry_identity,topology_identity=self.packet['topology_identity'])
