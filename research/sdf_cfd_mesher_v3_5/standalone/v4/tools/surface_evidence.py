"""Shared surface evidence operations, moved unchanged from qualified drivers.

No historical geometry, policy, test modules or reference run is loaded.
"""
import collections,contextlib,json,math
from pathlib import Path
from unittest.mock import patch
import numpy as np
import audit_adaptive_surface as audit
from historical_capsule_surface import save,sha,identity,array_hash
from sdf_cfd_mesher_v4.sdf.canonical import GeometryFailure

def read(path): return json.loads(Path(path).read_text(encoding='utf-8'))
NONSCIENTIFIC={'seconds','timing','peak_rss_bytes','retained_rss_bytes','topology_seconds'}
SEED=Path('labelled_spline_seed/surface/live/seed')

def stop(code,stage,**evidence):
    raise GeometryFailure(code,stage,evidence)


@contextlib.contextmanager
def observe_qualification(out,binding):
    """Stop on existing predicates; no replacement certificate or tolerance."""
    original_arrays=audit.audit_arrays;original_face=audit.certify_surface_face
    original_feature=audit.feature_bound;original_native=audit.run_native
    rows=[];features=[];counts=collections.Counter();methods=collections.Counter()
    def finite_or_text(value):
        return value if value is None or math.isfinite(value) else str(value)
    def persist():
        save(out/'QUALIFICATION_PROGRESS.json',dict(face_counts=dict(counts),methods=dict(methods),
            surface_assurance=getattr(binding.qualification,'assurance','qualification'),
            faces_completed=len(rows),features_completed=len(features),
            maximum_position_bound=finite_or_text(max((r[0] for r in rows),default=None)),
            maximum_normal_bound=finite_or_text(max((r[1] for r in rows),default=None))))
        np.savez_compressed(out/'PARTIAL_CERTIFICATES.npz',face_bounds=np.array(rows),feature_bounds=np.array(features))
    def arrays(*args,**kwargs):
        result,data=original_arrays(*args,**kwargs);save(out/'ARRAY_AUDIT.json',result)
        if not result['topology']['passed']:stop('SurfaceTopologyFailed','array_audit',topology=result['topology'])
        if result['normal']['inverted_triangles']:
            x,f,a=args[:3];fr=kwargs['face_refs']
            bad=[i for i,(ref,p,n) in enumerate(zip(fr,data['centre'],data['normal'])) if n@a.exact_normal(ref,p)<=0]
            save(out/'INVERTED_FACES.json',dict(indices=bad,triangles=np.asarray(f)[bad],coordinates=np.asarray(x)[np.asarray(f)[bad]]))
            stop('SourceRelativeInversion','array_audit',count=len(bad),entities='INVERTED_FACES.json')
        if result['sdf_residual']['max']>binding.qualification.projection_tolerance_m:
            stop('OwningResidualGateFailed','array_audit',maximum=result['sdf_residual']['max'],limit=binding.qualification.projection_tolerance_m)
        return result,data
    def face(a,ref,triangle,contract,**kwargs):
        value=original_face(a,ref,triangle,contract,**kwargs);status,position,normal,method=value
        rows.append((position,normal));counts[status]+=1;methods[method]+=1
        if status!='Certified' or position>binding.qualification.position_limit_m or normal>binding.qualification.normal_limit_rad:
            safe=[status,finite_or_text(position),finite_or_text(normal),method]
            persist();save(out/'FIRST_FACE_FAILURE.json',dict(index=len(rows)-1,ref=ref,triangle=triangle,result=safe))
            stop('ContinuousFaceRequirementFailed','continuous_geometry',face=len(rows)-1,result=safe)
        if len(rows)%2000==0:persist()
        return value
    def feature(a,p,q,ref):
        value=original_feature(a,p,q,ref);features.append(value)
        if not math.isfinite(value) or value>binding.qualification.position_limit_m:
            persist();save(out/'FIRST_FEATURE_FAILURE.json',dict(index=len(features)-1,ref=ref,points=[p,q],bound=str(value)))
            stop('ContinuousFeatureRequirementFailed','continuous_features',edge=len(features)-1,bound=str(value))
        return value
    def native(command,stem,*args,**kwargs):
        value=original_native(command,stem,*args,**kwargs)
        if Path(stem).name=='intersections':
            r=json.loads(value[1]);save(out/'INTERSECTIONS.json',r)
            if r['forbidden'] or r['unresolved']:stop('SurfaceIntersectionRequirementFailed','intersections',result=r)
        return value
    with patch.object(audit,'audit_arrays',arrays),patch.object(audit,'certify_surface_face',face),patch.object(audit,'feature_bound',feature),patch.object(audit,'run_native',native):
        try:yield
        finally:persist()


def compare(first, second, *, retained_capsule=False):
    old, new = read(first/'GENERATION.json'),read(second/'GENERATION.json')
    keys=('coordinate_hash','topology_hash','sizing_field_identity','residual_hash',
          'geometry_identity','configuration_identity','adaptation_identity','surface_identity',
          'carrier','mmgs_operation_rows','projections','initial_vertices','initial_triangles',
          'final_vertices','final_triangles')
    fields={k:old[k]==new[k] for k in keys}
    arrays={}
    for name in ('INITIAL.npz','SURFACE.npz','QUALITY.npz','GEOMETRY_CERTIFICATE.npz','REFERENCES.npz'):
        # Original capsule replay compares the 23 arrays in four sealed NPZs.
        # REFERENCES was introduced by the shared executor after that seal.
        if retained_capsule and name=='REFERENCES.npz':continue
        if not (first/name).exists() and not (second/name).exists():continue
        with np.load(first/name) as a, np.load(second/name) as b:
            arrays[name]={k:k in b.files and np.array_equal(a[k],b[k]) for k in a.files}
            arrays[name]['same_keys']=set(a.files)==set(b.files)
    a,b=read(first/'AUDIT.json'),read(second/'AUDIT.json')
    source={}
    if 'source_boundary' in a:
        for k in ('bounds','curve_evidence','unresolved','position_bound_max_m',
                  'normal_bound_max_rad','contract_identity'):
            source[k]=a['source_boundary'][k]==b['source_boundary'][k]
    return dict(fields=fields,arrays=arrays,source_boundary=source,
        independent_audits_passed=a['passed'] and b['passed'],
        passed=all(fields.values()) and all(all(v.values()) for v in arrays.values())
            and all(source.values()) and a['passed'] and b['passed'])


def scientific(value):
    """Drop only named timing/process metadata; retain every scientific field."""
    if isinstance(value,dict):return {k:scientific(v) for k,v in value.items() if k not in NONSCIENTIFIC}
    if isinstance(value,list):return [scientific(v) for v in value]
    return value


def compare_arrays(a,b):
    with np.load(a) as x,np.load(b) as y:
        out={'same_keys':set(x.files)==set(y.files),'arrays':{}}
        for key in sorted(set(x.files)|set(y.files)):
            if key not in x.files or key not in y.files:out['arrays'][key]={'equal':False};continue
            p,q=x[key],y[key];same=p.shape==q.shape and p.dtype==q.dtype
            equal=same and np.array_equal(p,q)
            bits=same and p.tobytes()==q.tobytes()
            row=dict(shape=list(p.shape),dtype=str(p.dtype),equal=bool(equal),stored_bits_equal=bool(bits),
                reference_hash=array_hash(p),replay_hash=array_hash(q))
            if same:
                row['different_elements']=int(np.count_nonzero(p!=q))
                if np.issubdtype(p.dtype,np.number):row['maximum_absolute_difference']=0. if equal else float(np.max(np.abs(p.astype(float)-q.astype(float))))
            out['arrays'][key]=row
        out['passed']=out['same_keys'] and all(r['equal'] and r.get('stored_bits_equal',False) for r in out['arrays'].values())
        return out


def scientific_comparison(reference,replay,*,seed_route='ExistingCGALSplineParameterCharts'):
    a,b=reference/'fresh',replay/'fresh'
    result=dict(existing_comparison=compare(a,b),arrays={},files={},records={})
    names=['INITIAL.npz','SURFACE.npz','REFERENCES.npz','QUALITY.npz','GEOMETRY_CERTIFICATE.npz']
    if seed_route=='ExistingCGALSplineParameterCharts':names.append(str(SEED/'SEED.npz'))
    elif seed_route not in ('CGALImplicitExtraction','CGALImplicitSurfaceExtraction/1','CanonicalMarchingCubesSeed/1'):raise ValueError('UnsupportedSeedEvidenceRoute')
    for name in names:result['arrays'][name]=compare_arrays(a/name,b/name)
    result['arrays']['partial_face_and_feature_bounds']=compare_arrays(reference/'PARTIAL_CERTIFICATES.npz',replay/'PARTIAL_CERTIFICATES.npz')
    seed_files=([str(SEED/f'{i}.{kind}') for i in range(1,5) for kind in ('input','output')]
        if seed_route=='ExistingCGALSplineParameterCharts' else
        ['seed.off'] if seed_route in ('CGALImplicitSurfaceExtraction/1','CanonicalMarchingCubesSeed/1') else ['initial_raw.off'])
    for name in ['metric.bin','raw.mesh','surface.off']+seed_files:
        h,k=sha(a/name),sha(b/name);result['files'][name]=dict(reference=h,replay=k,equal=h==k)
    records=['CARRIER.json','AUDIT.json']
    if seed_route=='ExistingCGALSplineParameterCharts':records.append(str(SEED/'SEED_AUDIT.json'))
    for name in records:
        p,q=scientific(read(a/name)),scientific(read(b/name))
        result['records'][name]=dict(equal=p==q,reference_identity=identity(p),replay_identity=identity(q))
    result['excluded_metadata']=sorted(NONSCIENTIFIC)
    result['reference_payloads']={name:sha(a/name) for name in names}
    result['replay_payloads']={name:sha(b/name) for name in names}
    result['reference_audit_sha256']=sha(a/'AUDIT.json');result['replay_audit_sha256']=sha(b/'AUDIT.json')
    result['passed']=result['existing_comparison']['passed'] and all(x['passed'] for x in result['arrays'].values()) and all(x['equal'] for x in result['files'].values()) and all(x['equal'] for x in result['records'].values())
    return result


from fractions import Fraction as Q
import ast,struct
from shape_aware_metric import apply_shape_metric
from historical_capsule_surface import SeedVertex
from procedural_authorities import SeedFace,SeedEdge
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel

def contract_reader(authority):
    from owned_runtime import SOURCE
    source=SOURCE/'verify_written_metric.py'
    tree=ast.parse(source.read_text(encoding='utf-8'))
    outer=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='verify')
    fn=next(n for n in outer.body if isinstance(n,ast.FunctionDef) and n.name=='unpack')
    ns=dict(struct=struct,np=np,SeedVertex=SeedVertex,SeedFace=SeedFace,SeedEdge=SeedEdge,
        a=getattr(authority,'unit',authority),vp=struct.Struct('<ddd i BB dddddd'),fp=struct.Struct('<iiii'),ep=struct.Struct('<iiiBB'))
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),ns)
    return ns['unpack']

def exact_majorant(actual, requirement):
    # Independent all-direction check of *computed* binary64 tensors, not a
    # certificate of the mathematical geometry or eigensolver. Principal minors
    # characterize PSD for symmetric 3x3; use the existing locked exact signs.
    exact=kernel.module('rank_geometry')
    m=[[Q(float(actual[i,j]))-Q(float(requirement[i,j])) for j in range(3)] for i in range(3)]
    if any(m[i][j]!=m[j][i] for i in range(3) for j in range(3)):return False
    minors=[m[i][i] for i in range(3)]
    minors += [m[i][i]*m[j][j]-m[i][j]*m[j][i] for i,j in ((0,1),(0,2),(1,2))]
    minors += [exact.dot(m[0],exact.cross(m[1],m[2]))]
    return all(exact.sign(v)>=0 for v in minors)

def verify_written(binding,directory,vertices,faces,edges,*,reference=None):
    a=binding.authority;c=binding.metric_connection;unpack=contract_reader(a)
    _,_,_,actual=unpack(directory/'metric.bin')
    old=reference
    hv,hf,he,hm=unpack(old) if old is not None else (None,None,None,None)
    curves=[set() for _ in vertices]
    for edge in edges:
        for i in edge.vertices:curves[i].add(edge.curve_ref)
    checks=collections.Counter();maximum_difference=0.
    for i,v in enumerate(vertices):
        m,row=c.evaluate(v.point,v.sheet_ref,curves=curves[i],trace=True)
        difference=float(np.max(abs(m[np.triu_indices(3)]-actual[i][np.triu_indices(3)])))
        maximum_difference=max(maximum_difference,difference)
        if difference!=0:stop('WrittenMetricMismatch','tensor_serialization',vertex=i,difference=difference)
        # Existing coupled finish_shape also applies family scaling. This
        # boundary requires an unscaled term and applies the scale once below.
        shape=getattr(c,'finish_unscaled_shape',apply_shape_metric)
        row['shape']=(row['prism'] if c.shape is None else shape(row['prism'],row['principal'],c.shape)[0])
        for name in ('geometry','feature','geometry_feature','prism','shape'):
            term=row[name]
            if term is None:continue
            term=np.triu(term)+np.triu(term,1).T
            if not exact_majorant(actual[i],term):stop('MetricContributionLost','tensor_serialization',vertex=i,term=name)
            checks[name]+=1
        if not np.array_equal(m,row['shape']/(c.length_scale*c.length_scale)):
            stop('FamilyScalingMismatch','tensor_serialization',vertex=i)
    same_points=old is not None and np.array_equal([v.point for v in vertices],[v.point for v in hv])
    same_faces=old is not None and np.array_equal([f.vertices for f in faces],[f.vertices for f in hf])
    same_labels=old is not None and ([f.sheet_ref for f in faces]==[f.sheet_ref for f in hf] and
        [(e.vertices,e.curve_ref) for e in edges]==[(e.vertices,e.curve_ref) for e in he])
    record=dict(vertices=len(vertices),triangles=len(faces),feature_edges=len(edges),
        exact_directional_checks=dict(checks),maximum_written_difference=maximum_difference,
        family_scale_applied_once=True,metric_override=False,clipping=False,
        historical_points_equal=same_points,historical_faces_equal=same_faces,historical_labels_equal=same_labels,
        historical_tensors_equal=np.array_equal(actual,hm) if same_points and same_labels else None,
        written_sha256=sha(directory/'metric.bin'),historical_sha256=sha(old) if old is not None else None,
        complete_contract_bytes_equal=(directory/'metric.bin').read_bytes()==old.read_bytes() if old is not None else None,
        eig_min=float(np.linalg.eigvalsh(actual).min()),eig_max=float(np.linalg.eigvalsh(actual).max()))
    save(directory.parent/'WRITTEN_METRIC_VERIFICATION.json',record)
    if same_points and same_faces and same_labels and not record['historical_tensors_equal']:
        stop('MatchedInputHistoricalMetricMismatch','tensor_serialization',record=record)
    return record
