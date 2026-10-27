"""Explicit production surface path: live SDF -> CGAL seed -> V3 carrier -> MMGS.

The preserved generic algorithms execute from the owned V4 runtime. Fixture
walls are never fresh generation inputs. This module is a compatibility owner;
mesh.py is the public case-driven entry.
The reference sizing policy retains V3 development-equivalence limits. It makes
no CFD accuracy claim. Surface generation never constructs a prism/front/core.
"""
from __future__ import annotations
import dataclasses, hashlib, json, math, os, re, shutil, subprocess, sys, time
from pathlib import Path
import numpy as np
import psutil

sys.dont_write_bytecode = True
REPO = Path(__file__).resolve().parents[3]
V4 = REPO/'research/sdf_cfd_mesher_v4'
V3 = REPO/'research/sdf_cfd_mesher_v3'
from owned_runtime import SURFACE as REFERENCE
sys.path.insert(0, str(REFERENCE))
from procedural_authorities import Authority, SeedVertex, SeedFace, CohortAuthority
from shape_aware_metric import ShapeMetricPolicy, apply_shape_metric, generic_geometry_metric, generic_geometry_metric_from_differential
from adaptive_sdf_carrier import CarrierOraclePolicy, refine_metric_carriers
import generic_v3mmg1_provider as provider
from sdf_cfd_mesher_v4.common.authority import identity, array_hash
from geometry_family_authority import Source, primitive

VERSION = 'V4ProductionAdaptiveSdfSurface/1'
OUT = V4/'results/production_adaptive_capsule_surface_foundation'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1<<20), b''):h.update(b)
    return h.hexdigest()

def save(p, value):
    Path(p).parent.mkdir(parents=True, exist_ok=True)
    Path(p).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False,
        default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a.item() if isinstance(a,np.generic) else str(a))+'\n')

def wsl(p):
    p=str(Path(p).resolve()); return '/mnt/'+p[0].lower()+p[2:].replace('\\','/')

def run_native(command, stem, timeout=120):
    """Linux timeout owns the native child, including termination on timeout."""
    stem=Path(stem); start=time.perf_counter()
    free=shutil.disk_usage(stem.parent).free
    measured_input=sum(p.stat().st_size for p in stem.parent.glob('*') if p.is_file())
    if free < measured_input: raise RuntimeError('InsufficientMeasuredScratchSpace')
    timed=['/usr/bin/time','-v','-o',wsl(stem.with_suffix('.resources.txt')),
           'timeout','--signal=TERM','--kill-after=5',str(timeout)]+command
    result=subprocess.run(['wsl','-e']+timed,capture_output=True,text=True,timeout=timeout+15)
    stem.with_suffix('.stdout').write_text(result.stdout)
    stem.with_suffix('.stderr').write_text(result.stderr)
    resources=stem.with_suffix('.resources.txt').read_text()
    m=re.search(r'Maximum resident set size \(kbytes\): (\d+)',resources)
    row=dict(command=command,seconds=time.perf_counter()-start,exit_code=result.returncode,
        peak_rss_bytes=int(m[1])*1024 if m else None,free_before_bytes=free,
        measured_input_bytes=measured_input,timeout_seconds=timeout)
    save(stem.with_suffix('.json'),row)
    if result.returncode:raise RuntimeError('NativeSurfaceStageFailed:'+str(row)+result.stderr[-2000:])
    return row,result.stdout

@dataclasses.dataclass(frozen=True)
class SurfaceSizing:
    minimum_size_m: float = 1e-7
    maximum_size_m: float = .04
    position_limit_m: float = 51.480559638e-6
    normal_limit_deg: float = 5.729078314
    metric_multiplier: float = 1.
    shape_response_reserve: float = 5.5
    normal_qualification_fraction: float = .90
    carrier_maximum_depth: int = 4
    grading: float = 1.3
    policy_source: str = 'V3GenericDevelopmentEquivalence/1'

    def validate(self):
        if not 0 < self.minimum_size_m <= self.maximum_size_m:raise ValueError('InvalidSurfaceSizeBounds')
        if not all(math.isfinite(v) for v in dataclasses.asdict(self).values() if isinstance(v,(int,float))):
            raise ValueError('NonfiniteSurfacePolicy')
        # Reuse the recovered implementation exactly. New tolerances require a
        # separately qualified policy, not implicit mutation of V3 globals.
        fixed=SurfaceSizing()
        for k in ('position_limit_m','normal_limit_deg','metric_multiplier','shape_response_reserve',
                  'normal_qualification_fraction','carrier_maximum_depth','grading','policy_source'):
            if getattr(self,k)!=getattr(fixed,k):raise ValueError('UnqualifiedSurfacePolicy:'+k)

def canonical(x, f, *, return_permutations=False):
    x=np.asarray(x,dtype='<f8').copy(); x[x==0]=0.
    f=np.asarray(f,dtype='<i8')
    if not np.isfinite(x).all():raise ValueError('NonfiniteSurface')
    order=np.lexsort((x[:,2],x[:,1],x[:,0])); inverse=np.empty(len(order),int);inverse[order]=np.arange(len(order))
    x=x[order];f=inverse[f]
    if len(np.unique(x,axis=0))!=len(x):raise ValueError('DuplicateSurfaceVertices')
    f=np.asarray([np.roll(row,-int(np.argmin(row))) for row in f],dtype='<i8')
    face_order=np.lexsort((f[:,2],f[:,1],f[:,0]));f=f[face_order]
    if len(np.unique(np.sort(f,axis=1),axis=0))!=len(f):raise ValueError('DuplicateSurfaceTriangles')
    return (x,f,order,inverse,face_order) if return_permutations else (x,f)

def read_off(p):
    rows=Path(p).read_text().splitlines(); nv,nf,_=map(int,rows[1].split())
    return np.array([[float(v) for v in r.split()] for r in rows[2:2+nv]]),np.array([[int(v) for v in r.split()[1:]] for r in rows[2+nv:2+nv+nf]],int)

def write_off(p,x,f):
    with Path(p).open('w') as out:
        out.write(f'OFF\n{len(x)} {len(f)} 0\n')
        for row in x:out.write(' '.join(format(v,'.17g') for v in row)+'\n')
        for row in f:out.write('3 '+' '.join(map(str,row))+'\n')

class CapsuleGeometry(Authority):
    """Geometry plug-in only. Connectivity is extracted, never direction-indexed."""
    name='analytic_capsule_sdf'
    sheet_names={1:'capsule.continuous_surface'}
    curve_names={}; corner_names={}
    _normal_map_principal_data=CohortAuthority._normal_map_principal_data

    def __init__(self, definition):
        if definition['primitive']!='capsule':raise ValueError('CapsuleGeometryRequired')
        self.definition={k:definition[k] for k in ('primitive','radius','half_length')}
        self.geometry_identity=identity(dict(implementation_hash=sha(V4/'tools/geometry_family_authority.py'),definition=self.definition))
        self.projection=dict(operations=0,failures=0,zero_gradient=0,nonfinite=0,maximum_iterations=1)
        self.projection_seconds=0.

    def implicit_value(self,ref,p):
        self.check_ref(ref);return float(primitive(np.asarray(p)[None,:],self.definition)[0][0])

    def check_ref(self,ref):
        if ref!=1:raise ValueError('UnknownSourceBranch')

    def project_sheet(self,ref,p):
        self.check_ref(ref);t=time.perf_counter();p=np.asarray(p,float)
        self.projection['operations']+=1
        if not np.isfinite(p).all():
            self.projection['failures']+=1;self.projection['nonfinite']+=1;raise ValueError('NonfiniteProjection')
        center=np.array([0.,0.,np.clip(p[2],-self.definition['half_length'],self.definition['half_length'])])
        direction=p-center; length=np.linalg.norm(direction)
        if not length>0:
            self.projection['failures']+=1;self.projection['zero_gradient']+=1;raise ValueError('ProjectionZeroGradient')
        q=center+self.definition['radius']*(direction/length)
        self.projection_seconds+=time.perf_counter()-t
        return q

    def exact_normal(self,ref,p):
        self.check_ref(ref);n=primitive(np.asarray(p)[None,:],self.definition)[1][0]
        if not np.linalg.norm(n)>0:raise ValueError('UndefinedSourceNormal')
        return n

    def gradient_hessian(self,ref,p):
        n=self.exact_normal(ref,p);p=np.asarray(p)
        center=np.array([0.,0.,np.clip(p[2],-self.definition['half_length'],self.definition['half_length'])])
        jac=np.eye(3)
        if abs(p[2])<self.definition['half_length']:jac[2,2]=0.
        return n,(jac-np.outer(n,n))/np.linalg.norm(p-center)

    def curvature_upper(self,ref,p):
        self.check_ref(ref);return 1./self.definition['radius']

    def surface_diagnostics(self,x,s,dist):
        # Unchanged capsule descriptive calculations, moved out of the generic audit.
        kmax=np.full(len(x),self.curvature_upper(1,None));kmin=np.where(abs(x[:,2])<self.definition['half_length'],0.,kmax)
        return dict(minimum_principal=dist(kmin),maximum_principal=dist(kmax),
            edge_times_max_curvature=dist(s['edge_length']*kmax[0]),triangle_circumradius_times_max_curvature=dist(s['circumradius']*kmax[0]),
            maximum_edge_ratio_location=((x[s['edges'][s['edge_length'].argmax()]]).mean(0)).tolist(),
            feature_size_gate='V3 local-feature-size gate: Not implemented.',source_medial_distance_m=self.definition['radius'])

def metric(authority, point, policy, differential=False, *, sheet_ref=1):
    vertex=SeedVertex(np.asarray(point),sheet_ref,0,0)
    fn=generic_geometry_metric_from_differential if differential else generic_geometry_metric
    m,principal=fn(authority,vertex,policy.metric_multiplier)
    m,_=apply_shape_metric(m,principal,ShapeMetricPolicy(policy.normal_qualification_fraction,policy.shape_response_reserve))
    vals,vecs=np.linalg.eigh(m)
    vals=np.clip(vals,1/policy.maximum_size_m**2,1/policy.minimum_size_m**2)
    return (vecs*vals)@vecs.T

def seed_size(curvature_bound, maximum_size):
    # V3 held_out_geometry_cohort::Fixture::physical_edge_target. Use its
    # geometry-derived bound to avoid silently retaining metre-scale seed
    # resolution on smaller domains. Final MMGS sizing is unchanged.
    if not math.isfinite(curvature_bound) or curvature_bound<0:raise ValueError('InvalidCurvatureBound')
    return min(maximum_size,.20/curvature_bound) if curvature_bound else maximum_size

def require_production_surface(record, *, test_only_fixture_override=False):
    if record.get('generator_type')!='AdaptiveProductionSurface':
        if test_only_fixture_override:return dict(test_only=True,production_authorized=False)
        raise ValueError('FixtureOrUnknownSurfaceNotProductionAuthority')
    for key in ('implementation_identity','geometry_identity','configuration_identity','sizing_field_identity',
                'adaptation_identity','surface_identity','qualification'):
        if not record.get(key):raise ValueError('MissingProductionSurfaceBinding:'+key)
    q=record['qualification']
    if not all(q.get(k) is True for k in ('geometry','topology','orientation','self_intersection','sizing_audit','determinism')):
        raise ValueError('SurfaceQualificationIncomplete')
    return record

class ProductionAdaptiveSdfSurfaceMesher:
    """Explicit historical caller; delegates to the single production executor."""
    def __init__(self, policy=None):
        self.policy=policy or SurfaceSizing();self.policy.validate()

    def generate(self, authority, output):
        from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher as Entry
        return Entry().generate(capsule_binding(authority,self.policy),output)


def capsule_seed(authority, policy, output):
    """Case-owned extraction inputs; same executable and arithmetic as the sealed run."""
    d=authority.definition
    physical_seed_size=seed_size(authority.curvature_upper(1,None),policy.maximum_size_m)
    from owned_runtime import BIN,verify
    verify()
    timing,_=run_native([wsl(BIN/'capsule_seed'),repr(d['radius']),repr(d['half_length']),
        wsl(output/'initial_raw.off'),repr(physical_seed_size)],output/'extraction')
    raw,faces=read_off(output/'initial_raw.off')
    x=np.array([authority.project_sheet(1,p) for p in raw]);x,faces=canonical(x,faces)
    return dict(coordinates=x,triangles=faces,vertices=[SeedVertex(p,1,0,0) for p in x],
        faces=[SeedFace(tuple(f),1) for f in faces],edges=[],timing=timing,seed_size_m=physical_seed_size)


def capsule_binding(authority, policy=None):
    from surface_binding import SurfaceBinding,QualificationContract
    from audit_adaptive_surface import audit_directory
    from procedural_authorities import POSITION_LIMIT,NORMAL_LIMIT_RAD
    if not isinstance(authority,CapsuleGeometry):raise ValueError('HistoricalCapsuleAuthorityRequired')
    refs=dict(sheet_names=authority.sheet_names,curve_names=authority.curve_names,corner_names=authority.corner_names)
    contract=QualificationContract(1,2,0,**refs,position_limit_m=POSITION_LIMIT,
        normal_limit_rad=NORMAL_LIMIT_RAD,projection_tolerance_m=2e-12,
        premise='Retained binary64 analytic capsule: continuous sheet, exact projection and curvature bound',
        source='production_adaptive_capsule_surface_foundation/fresh/SURFACE_AUTHORITY.json')
    return SurfaceBinding(authority,authority.geometry_identity,'m','binary64 analytic Authority',refs,
        seed_provider=capsule_seed,sizing_policy=policy or SurfaceSizing(),qualification=contract,
        qualifier=audit_directory)

def main():
    label=sys.argv[1]
    # Read only analytic geometry parameters. No preserved surface array is opened.
    path=V4/'results/production_scale_resume_fast/capsule_completion/GEOMETRY_AUTHORITY.json'
    old=json.loads(path.read_text());spec=old['spec']
    if Source(spec).source_identity!=old['authority']['sdf']:raise ValueError('AnalyticSourceIdentityMismatch')
    authority=CapsuleGeometry(spec['components'][0])
    save(OUT/'GEOMETRY.json',dict(definition=authority.definition,geometry_identity=authority.geometry_identity,
        original_source_identity=Source(spec).source_identity,source_hash=sha(V4/'tools/geometry_family_authority.py'),
        definition_record_hash=sha(path),old_mesh_reads=0))
    ProductionAdaptiveSdfSurfaceMesher().generate(authority,OUT/label)

if __name__=='__main__':main()
