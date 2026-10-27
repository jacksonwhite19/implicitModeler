"""Generic case connection to the existing surface, prism and core owners.

This sufficient profile requires a proved C1 star-shaped closed source. Native
CGAL extraction proposes triangles; the existing audit accepts the real target.
"""
from contextlib import ExitStack,contextmanager
from contextvars import ContextVar
from dataclasses import asdict
from fractions import Fraction
from pathlib import Path
import math,json,threading,time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import file_hash,identity
from sdf_cfd_mesher_v4.sdf.regional import upper_float
from sdf_cfd_mesher_v4.sdf.regional_owner import canonical_region_scope
from general_sdf_authority import GeneralSdfAuthority,ManualRequirements
from general_source_contract import GeneralSourceQualification
from historical_capsule_surface import SurfaceSizing,SeedVertex,SeedFace,read_off,wsl,save
from prism_host_metric import PrismMetricPolicy
from surface_binding import SurfaceBinding
from audit_adaptive_surface import audit_directory

_case_queries=ContextVar('general_surface_case_queries',default=None)

def validate_general_policy(case):
    g=case['general_sdf']
    if set(g)!=set(('profile','manual','proof','seed')) or g['profile'] not in ('smooth_star_shaped_real_source/1','explicit_numerical_surface_requirements/1'):
        raise ValueError('ExplicitGeneralSourceProfileRequired')
    if 'rhai' not in case['geometry'] and 'rhai_file' not in case['geometry']:raise ValueError('CanonicalRhaiSourceRequired')
    ManualRequirements(**g['manual']).validate()
    p=g['proof'];fields=('root_ref','maximum_nodes','maximum_depth','maximum_patches','precision_bits',
        'interior_m','topology_depth','topology_boxes','coverage_depth','coverage_boxes')
    if set(p)!=set(fields):raise ValueError('ExplicitGeneralProofBudgetsRequired')
    for name in fields:
        if name=='interior_m':continue
        if type(p[name]) is not int or p[name]<1:raise ValueError('InvalidGeneralProofBudget',name)
    if p['precision_bits']>256:raise ValueError('InvalidGeneralProofPrecision')
    c=np.asarray(p['interior_m'],dtype=float)
    if c.shape!=(3,) or not np.isfinite(c).all():raise ValueError('ExplicitInteriorWitnessRequired')
    s=g['seed']
    stock=s.get('method')=='CanonicalMarchingCubesSeed/1'
    if stock:
        from seed_controls import validate_settings
        validate_settings(s)
    else:
        validate_cgal_seed(s)
    surface=case['surface'];r=g['manual']
    from procedural_authorities import POSITION_LIMIT,NORMAL_LIMIT_RAD
    from surface_requirements195 import PROFILE,validate_accuracy
    if g['profile']==PROFILE:
        validate_accuracy(surface['position_limit_m'],surface['normal_limit_rad'],surface['normal_qualification_fraction'])
    elif surface['position_limit_m']!=POSITION_LIMIT or surface['normal_limit_rad']!=NORMAL_LIMIT_RAD:
        raise ValueError('UnqualifiedGeneralAccuracyPolicy')
    if (surface['minimum_size_m']!=r['minimum_size_m'] or surface['maximum_size_m']!=r['maximum_size_m']
            or surface['length_scale']!=1 or surface['metric_multiplier']!=1):
        raise ValueError('ManualSIRequirementsConflict')


def validate_cgal_seed(s):
    if set(s)!=set(('relative_error','criteria','seed','timeout_seconds','maximum_queries')):
        raise ValueError('ExplicitGeneralSeedControlsRequired')
    if (type(s['seed']) is not int or s['seed']<0 or type(s['maximum_queries']) is not int or s['maximum_queries']<1
            or type(s['timeout_seconds']) not in (float,int) or not math.isfinite(s['timeout_seconds']) or s['timeout_seconds']<=0):
        raise ValueError('InvalidGeneralSeedResourceControls')
    from sdf_cfd_mesher_v4.sdf.cgal import CgalInvocation
    from sdf_cfd_mesher_v4.sdf.canonical import EnclosingRegion
    CgalInvocation(('proposal',),'--extract-surface','proposal.off',s['relative_error'],
        extraction_criteria=tuple(s['criteria']),seed=s['seed']).command(EnclosingRegion((0,0,0),1,'validation only'))


class ManualMetricConnection:
    def __setattr__(self,name,value):
        if getattr(self,'_sealed',False):raise ValueError('ProtectedGeneralMetricConnection')
        object.__setattr__(self,name,value)
    def __init__(self,binding,sizing,case_pin,path,case_identity):
        self.binding=binding;self.authority=binding.authority;self.sizing=sizing
        self.prism=self.authority._prism_policy;self.shape=self.authority.shape_controls();self.length_scale=1
        self.source_identity=case_identity;self.path=path;self.pin=case_pin
        self._qualification=binding.qualification
        self._general_identity=identity(binding.general_policy)
        self._sealed=True
    def verify(self,b):
        self.sizing.validate();self.authority.verify()
        if (b is not self.binding or b.authority is not self.authority or b.metric_connection is not self
                or b.sizing_policy!=self.sizing or type(b.qualification) is not GeneralSourceQualification
                or b.qualification is not self._qualification or identity(b.general_policy)!=self._general_identity
                or b.qualifier is not audit_directory or b.seed_provider is not labelled_seed
                or b.seed_route!=seed_route(b.general_policy)
                or not self._case_matches()
                or self.shape!=self.authority.shape_controls()
                or self.shape.normal_qualification_fraction!=self.sizing.normal_qualification_fraction
                or self.shape.mmgs_response_reserve!=self.sizing.shape_response_reserve
                or self.shape.normal_limit_rad!=self.authority.manual_controls().normal_limit_rad
                or b.qualification.normal_limit_rad!=self.authority.manual_controls().normal_limit_rad
                or b.qualification.position_limit_m!=self.authority.manual_controls().position_limit_m
                or self.sizing.position_limit_m!=b.qualification.position_limit_m
                or self.sizing.normal_limit_deg!=math.degrees(b.qualification.normal_limit_rad)):raise ValueError('DetachedGeneralMeshConnection')
        b.qualification.verify(self.authority)

    def _case_matches(self):
        held=_case_queries.get()
        if held is not None and held[0] is self:
            held[1]();return True
        return file_hash(self.path)==self.pin

    @contextmanager
    def query_scope(self,output):
        """Authenticate case once while the existing guard denies all writes."""
        from immutable_state_access import guard
        from path_preserving_deduplication import durable_json
        if _case_queries.get() is not None:raise ValueError('NestedGeneralCaseScope')
        path=Path(self.path).resolve();pin=self.pin;thread=threading.get_ident()
        active=True;checks=0;hashes=0;stats=None
        def live():
            nonlocal checks
            if not active:raise ValueError('ExpiredGeneralCaseScope')
            if threading.get_ident()!=thread:raise ValueError('GeneralCaseThreadMismatch')
            if Path(self.path).resolve()!=path or self.pin!=pin:raise ValueError('ChangedGeneralCaseBinding')
            checks+=1
        def snapshot():
            live()
            return dict(source_snapshot(),case_scope_checks=checks,case_authentication_hashes=hashes)
        with guard(path):
            if file_hash(path)!=pin:raise ValueError('ChangedGeneralCaseInput')
            hashes+=1
            with self.authority.query_scope() as source_snapshot:
                token=_case_queries.set((self,live,snapshot))
                try:
                    self.verify(self.binding)
                    yield
                    self.verify(self.binding)
                finally:
                    try:stats=snapshot()
                    finally:active=False;_case_queries.reset(token)
                    if file_hash(path)!=pin:raise ValueError('ChangedProtectedGeneralCaseInput')
                    hashes+=1;stats['case_authentication_hashes']=hashes
                    if Path(output).is_dir():durable_json(Path(output)/'SOURCE_QUERY_REUSE.json',stats)

    def query_statistics(self):
        held=_case_queries.get()
        return held[2]() if held is not None and held[0] is self else None

    @contextmanager
    def construction_scope(self):
        held=_case_queries.get()
        if held is None or held[0] is not self:raise ValueError('ProtectedCaseScopeRequired')
        self.verify(self.binding)
        with self.authority.construction_scope():yield
        self.verify(self.binding)

    def carrier_options(self):
        self.verify(self.binding)
        # Manual sizing requires no differential diagnostic. Adequacy stays on.
        from standalone_tool import carrier_derivative_diagnostics_enabled
        return dict(derivative_diagnostics=carrier_derivative_diagnostics_enabled())
    def carrier_policy(self):
        from adaptive_sdf_carrier import CarrierOraclePolicy
        self.verify(self.binding)
        requirements=self.authority.manual_controls()
        if not requirements.explicit_numerical:return CarrierOraclePolicy()
        # Only the declared angle changes; retain all historical carrier controls.
        return CarrierOraclePolicy(normal_limit_rad=requirements.normal_limit_rad)
    def carrier_assessment_enabled(self):
        self.verify(self.binding)
        from standalone_tool import carrier_assessment_enabled
        return carrier_assessment_enabled()
    def evaluate(self,p,ref,*,curves=(),differential=False,trace=False):
        self.verify(self.binding)
        if curves or int(ref) not in self.binding.references['sheet_names']:raise ValueError('GeneralSourceReferenceMismatch')
        a=self.authority;m=a.composed_metric(p,ref)
        if trace:
            base=a.metric(SeedVertex(np.asarray(p),int(ref),0,0),set(),1)
            return m,dict(geometry=base,feature=None,geometry_feature=base,
                          prism=a.prism_metric(p,ref),principal=a.principal_data(p,ref))
        return m
    def finish_unscaled_shape(self,matrix,principal,policy):
        self.verify(self.binding)
        return self.authority.finish_shape(matrix,principal,policy)
    def carrier(self,p,ref):return self.evaluate(p,ref,differential=True)
    def evaluate_batch(self,points,refs,*,curves):
        self.verify(self.binding)
        captured=tuple(frozenset(c) for c in curves)
        if len(curves)!=len(points) or any(captured):raise ValueError('GeneralSourceReferenceMismatch')
        result=self.authority.composed_metrics_batch(points,refs)
        if tuple(frozenset(c) for c in curves)!=captured:raise ValueError('GeneralSourceReferenceMismatch')
        self.verify(self.binding)
        return result
    def write_contract(self,path,vertices,faces,edges):
        self.verify(self.binding)
        result=self.authority.write_contract(path,vertices,faces,edges)
        self.verify(self.binding)
        return dict(result,coupled_connection=dict(source_identity=self.source_identity,length_scale=1,
            metric_override_used=False,composition='Explicit SI size cap + projected normal-map metric -> existing prism tangent intersection -> retained generic shape conditioning',shape_policy=asdict(self.shape)))


def seed_route(g):
    return 'CanonicalMarchingCubesSeed/1' if g['seed'].get('method')=='CanonicalMarchingCubesSeed/1' else 'CGALImplicitSurfaceExtraction/1'


def canonical_seed_points(x,f):
    """Exact point quotient of a stock proposal, before source projection.

    No distance/area tolerance is used. Only triangles whose distinct native
    indices become the SAME point index are removed. The quotient must preserve
    closed oriented topology and manifold links at every affected vertex.
    This is construction, not surface acceptance or source-completeness proof.
    """
    x=np.asarray(x);f=np.asarray(f)
    if (x.dtype!=np.dtype('<f8') or x.ndim!=2 or x.shape[1]!=3 or not len(x)
            or not np.isfinite(x).all() or f.dtype.kind not in 'iu'
            or f.ndim!=2 or f.shape[1]!=3 or not len(f)
            or np.any(f<0) or np.any(f>=len(x))):raise ValueError('InvalidSeedPointIdentityInput')
    _,first,inverse=np.unique(x,axis=0,return_index=True,return_inverse=True)
    if len(first)==len(x):
        return x,f,dict(schema='V4ExactSeedPointIdentity/1',changed=False,
            merged_vertices=0,removed_index_collapsed_faces=0,
            role='Exact construction point identities only; final acceptance unchanged.')
    # Keep first occurrence and surviving face order. Unique seeds are untouched.
    order=np.argsort(first);representatives=first[order]
    index=np.empty(len(first),dtype=np.int64);index[order]=np.arange(len(first))
    mapping=index[inverse];points=x[representatives].copy();mapped=mapping[f]
    collapsed=(mapped[:,0]==mapped[:,1])|(mapped[:,1]==mapped[:,2])|(mapped[:,2]==mapped[:,0])
    if not collapsed.any():raise ValueError('SeedPointQuotientWithoutCollapsedEdge')
    faces=mapped[~collapsed].copy()
    if not len(faces):raise ValueError('EmptySeedPointQuotient')
    from audit_adaptive_surface import topology
    before=topology(x,f,expected_components=None,expected_euler=None)
    # Duplicate coordinates are the only permitted original topology failure.
    if any(before[k] for k in ('unused_vertices','boundary_edges','nonmanifold_edges',
            'duplicate_triangles','orientation_edge_failures')):raise ValueError('InvalidSeedPointQuotientParent')
    after=topology(points,faces,expected_components=before['connected_components'],expected_euler=before['euler'])
    if not after['passed']:raise ValueError('SeedPointQuotientTopologyFailure')
    parents=list(range(len(x)))
    def root(v):
        while parents[v]!=v:parents[v]=parents[parents[v]];v=parents[v]
        return v
    for row in f:
        a,b,c=map(int,row);parents[root(a)]=root(b);parents[root(b)]=root(c)
    labels=np.asarray([root(v) for v in range(len(x))],dtype=np.int64)
    if not np.array_equal(labels,labels[representatives][mapping]):raise ValueError('SeedPointQuotientComponentMerge')
    def component_eulers(rows,component_labels):
        vertices={};edges={};faces_count={}
        for v,c in enumerate(component_labels):vertices.setdefault(int(c),set()).add(v)
        for row in rows:
            a,b,c=map(int,row);owner=int(component_labels[a])
            faces_count[owner]=faces_count.get(owner,0)+1
            edges.setdefault(owner,set()).update((tuple(sorted((a,b))),tuple(sorted((b,c))),tuple(sorted((c,a)))))
        return {c:len(v)-len(edges.get(c,()))+faces_count.get(c,0) for c,v in vertices.items()}
    component_euler=component_eulers(f,labels)
    if component_eulers(faces,labels[representatives])!=component_euler:raise ValueError('SeedPointQuotientComponentEulerFailure')
    counts=np.bincount(mapping,minlength=len(points));affected=set(map(int,np.flatnonzero(counts>1)))
    original=set(map(int,np.flatnonzero(np.isin(mapping,list(affected)))))
    # Removing/remapping incident faces also changes unmerged neighbor stars.
    # Require valid links throughout the full affected face neighborhood.
    incident=np.isin(f,list(original)).any(axis=1)
    original=set(map(int,np.unique(f[incident])))
    affected=set(map(int,mapping[list(original)]))
    def require_links(rows,vertices):
        # Same cyclic-link requirement as the existing local termination owner.
        # Apply it to all changed stars; never accept a point-touching bow tie.
        links={v:[] for v in vertices}
        for row in rows:
            a,b,c=map(int,row)
            for v,left,right in ((a,b,c),(b,c,a),(c,a,b)):
                if v in links:links[v].append((left,right))
        for edges in links.values():
            if not edges or len({tuple(sorted(e)) for e in edges})!=len(edges):raise ValueError('SeedPointQuotientLinkFailure')
            adjacency={}
            for a,b in edges:
                adjacency.setdefault(a,set()).add(b);adjacency.setdefault(b,set()).add(a)
            seen=set();todo=[next(iter(adjacency))]
            while todo:
                v=todo.pop()
                if v not in seen:seen.add(v);todo.extend(adjacency[v]-seen)
            if len(seen)!=len(adjacency) or any(len(a)!=2 for a in adjacency.values()):raise ValueError('SeedPointQuotientLinkFailure')
    require_links(f,original);require_links(faces,affected)
    # No component may be joined to another merely because its points coincide.
    # Exact surviving triangles describe the same geometric simplices.
    from sdf_cfd_mesher_v4.common.authority import array_hash
    return points,faces,dict(schema='V4ExactSeedPointIdentity/1',changed=True,
        merged_vertices=len(x)-len(points),removed_index_collapsed_faces=int(collapsed.sum()),
        removed_face_indices=np.flatnonzero(collapsed).tolist(),mapping_sha256=array_hash(mapping),
        before=before,after=after,component_eulers=sorted(component_euler.values()),affected_links_checked=len(original)+len(affected),
        role='Exact construction point identities only; final acceptance unchanged.')


def stock_seed(a,settings,output):
    """Use native metadata bounds only for a proposal, never as source proof."""
    from historical_capsule_surface import write_off
    scale=case_scale(a._connection_binding)
    bounds=a.geometry.description['stored_metadata']['bounds_model']
    if bounds is None:raise ValueError('StockSeedBoundsUnavailable')
    box=np.asarray(bounds,dtype=float)*scale
    if box.shape!=(2,3) or not np.isfinite(box).all():raise ValueError('InvalidStockSeedBounds')
    box[0]-=settings['padding_m'];box[1]+=settings['padding_m']
    from seed_controls import query_controls,validate_plan,validate_extraction
    plan_packet=a.geometry.surface_seed_plan(**query_controls(settings))
    plan=validate_plan(settings,plan_packet['data'],a.geometry.description)
    save(Path(output)/'SEED_PREFLIGHT.json',plan_packet)
    if not plan['admitted']:raise ValueError('SeedPreflightRefused',plan['reason'],plan)
    packet=a.geometry.surface_seed(**query_controls(settings))
    data=packet['data']
    validate_extraction(settings,data,plan)
    x=np.asarray(data['coordinates_m'],dtype='<f8');f=np.asarray(data['triangles'],dtype=np.int64)
    if (x.ndim!=2 or x.shape[1]!=3 or len(x)==0 or not np.isfinite(x).all()
            or f.ndim!=2 or f.shape[1]!=3 or len(f)==0 or np.any(f<0) or np.any(f>=len(x))):
        raise ValueError('InvalidStockSeedOutput')
    write_off(Path(output)/'seed.off',x,f)
    native_vertices,native_triangles=len(x),len(f)
    started=time.perf_counter();x,f,point_identity=canonical_seed_points(x,f)
    point_identity['seconds']=time.perf_counter()-started
    save(Path(output)/'SEED_POINT_IDENTITY.json',point_identity)
    receipt={k:v for k,v in data.items() if k not in ('coordinates_m','triangles')}
    timing=dict(native=receipt,callback_queries=0,native_scalar_evaluations=data['scalar_evaluations'],
        seed_vertices=len(x),seed_triangles=len(f),geometry_identity=packet['geometry_identity'],
        native_seed_vertices=native_vertices,native_seed_triangles=native_triangles,
        point_identity=point_identity,
        seed_source='Native metadata box; proposal only. Existing final real-source coverage remains required.')
    save(Path(output)/'SEED_EXTRACTION.json',timing)
    return x,f,timing,plan['spacing_m_effective']


def labelled_seed(a,policy,output):
    a.verify();b=a._connection_binding;b.metric_connection.verify(b)
    settings=b.general_policy['seed']
    if seed_route(b.general_policy)=='CanonicalMarchingCubesSeed/1':
        x,f,timing,size=stock_seed(a,settings,output)
    else:
        x,f,timing,size=cgal_seed(a,settings,output)
    ref=a._root_ref
    # Both native extractors propose. The same real target owns projection,
    # source labels, orientation, metric refinement and final qualification.
    active=getattr(a,'construction_active',lambda:False)()
    project=getattr(a,'project_sheet_batch',None)
    t=time.perf_counter()
    x=(project(ref,x) if active and callable(project) else
       np.asarray([a.project_sheet(ref,v) for v in x],dtype='<f8'))
    timing['seed_projection_seconds']=time.perf_counter()-t;t=time.perf_counter()
    batch=getattr(a,'construction_normals_batch',None)
    if active and callable(batch):
        # Each original row uses mean(0), before any face orientation flip.
        centres=np.asarray([x[row].mean(0) for row in f],dtype='<f8')
        proposals=batch(centres,np.full(len(f),ref,dtype=np.int64))
    else:proposals=None
    for i,row in enumerate(f):
        normal=proposals[i] if proposals is not None else a.exact_normal(ref,x[row].mean(0))
        if np.cross(x[row[1]]-x[row[0]],x[row[2]]-x[row[0]])@normal<0:
            row[1],row[2]=row[2],row[1]
    timing['seed_orientation_seconds']=time.perf_counter()-t
    a.verify()
    return dict(coordinates=x,triangles=f,vertices=[SeedVertex(v,ref,0,0) for v in x],
        faces=[SeedFace(tuple(map(int,row)),ref) for row in f],edges=[],timing=timing,seed_size_m=size)


def cgal_seed(a,settings,output):
    a.verify();b=a._connection_binding
    b.metric_connection.verify(b)
    from owned_runtime import BIN,verify
    from sdf_cfd_mesher_v4.sdf.cgal import run_cgal,CgalInvocation
    from sdf_cfd_mesher_v4.sdf.canonical import EnclosingRegion
    verify();p=Path(output)/'seed.off'
    radius=upper_float(Fraction(b.qualification.source_topology['enclosure_radius_m']))
    invocation=CgalInvocation(('wsl.exe','-e',wsl(BIN/'canonical_cgal_seed')),'--extract-surface',wsl(p),
        settings['relative_error'],extraction_criteria=tuple(settings['criteria']),seed=settings['seed'])
    def progress(row):
        # Compact cumulative checkpoints survive a failed native seed. They are
        # passive diagnostics and never become acceptance or sizing inputs.
        save(Path(output)/'SEED_QUERY_DIAGNOSTIC.json',row)
        with (Path(output)/'SEED_QUERY_PROGRESS.jsonl').open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(row,sort_keys=True)+'\n')
    from standalone_tool import seed_query_diagnostics_enabled
    enabled=seed_query_diagnostics_enabled()
    timing=run_cgal(a.geometry,invocation,EnclosingRegion((0.,0.,0.),radius,'Proved real-source enclosure; native output remains a proposal'),
        timeout_seconds=settings['timeout_seconds'],maximum_queries=settings['maximum_queries'],
        progress=progress if enabled else None,diagnostic_unit_scale=case_scale(b) if enabled else None)
    save(Path(output)/'SEED_EXTRACTION.json',timing)
    native=timing['native']
    if (native.get('mode')!='unqualified_surface_seed' or native.get('route')!='CGALImplicitSurfaceExtraction/1'
            or native.get('volume_refinement') is not False or native.get('inactive_legacy_criteria_indices')!=[0,4,5]):
        raise ValueError('SurfaceOnlySeedReceiptRequired')
    x,f=read_off(p)
    return x,f,timing,settings['criteria'][2]


def case_scale(b):
    # The target already pins this exact binary64 SI coefficient. Decode the
    # captured mathematical coefficient without loading a second geometry.
    return float(Fraction(b.qualification.target['unit_scale_exact']))


class HeldGeometry:
    def __setattr__(self,name,value):
        if getattr(self,'_sealed',False):raise ValueError('ProtectedGeneralGeometryLifetime')
        object.__setattr__(self,name,value)
    def __init__(self,host,held):
        self.host=host;self.held=held;self._sealed=True
    def close(self):
        try:self.held.close()
        finally:self.host.close()


def connect(host,geometry,path):
    from owned_runtime import verify,BIN
    from standalone_case import load
    case,_,pin=load(path);g=case['general_sdf'];p=g['proof'];m=verify()
    executable=next(r['sha256'] for r in m['files'] if r['path']=='runtime/bin/canonical_kernel_host.exe')
    source=m['native_host_installation']['kernel_source_sha256']
    held=ExitStack()
    try:
        scope=held.enter_context(canonical_region_scope(geometry,authenticated_executable_sha256=executable,
            authenticated_source_sha256=source,**{k:p[k] for k in ('root_ref','maximum_nodes','maximum_depth','maximum_patches','precision_bits')}))
        from shape_aware_metric import ShapeMetricPolicy
        a=GeneralSdfAuthority(scope,ManualRequirements(**g['manual'],
            position_limit_m=case['surface']['position_limit_m'],normal_limit_rad=case['surface']['normal_limit_rad'],
            explicit_numerical=g['profile']=='explicit_numerical_surface_requirements/1'),
            PrismMetricPolicy(**case['surface']['prism_host']),
            ShapeMetricPolicy(case['surface']['normal_qualification_fraction'],case['surface']['shape_response_reserve'],
                normal_limit_rad=case['surface']['normal_limit_rad']))
        from standalone_tool import surface_assurance
        assurance=surface_assurance()
        if assurance=='numerical' and seed_route(g)!='CanonicalMarchingCubesSeed/1':
            raise ValueError('NumericalAcceptanceRequiresExistingStockSeedBounds')
        q=GeneralSourceQualification(a,p['interior_m'],assurance=assurance,**{k:p[k] for k in ('topology_depth','topology_boxes','coverage_depth','coverage_boxes')})
        s=case['surface'];sizing=SurfaceSizing(**{k:s[k] for k in ('minimum_size_m','maximum_size_m','position_limit_m','metric_multiplier',
            'shape_response_reserve','normal_qualification_fraction','carrier_maximum_depth','grading')},normal_limit_deg=math.degrees(s['normal_limit_rad']))
        if g['profile']=='explicit_numerical_surface_requirements/1':
            from surface_requirements195 import numerical_sizing
            sizing=numerical_sizing(s)
        sizing.validate()
        b=SurfaceBinding(a,geometry.identity,'m','Authenticated real expression; binary64 proposals; '+assurance+' acceptance',
            {k:getattr(a,k) for k in ('sheet_names','curve_names','corner_names')},canonical=geometry,
            query_discovery=geometry.discover(),seed_provider=labelled_seed,sizing_policy=sizing,
            qualification=q,qualifier=audit_directory,seed_route=seed_route(g),
            limitations=[dict(profile=g['profile'],assurance=assurance,target=q.target,
                unsupported=('Source proof failures, sharp features, multiple components, handles and unexported operators fail admission.'
                    if assurance=='qualification' else 'No whole-source coverage, global C1/topology or continuous geometry proof. Sharp feature identity support is not established; sampled acceptance can miss unsampled defects.'))])
        b.general_policy=g;b.metric_connection=ManualMetricConnection(b,sizing,pin,path,identity(case))
        # Private immutable authority captures the exact selected seed binding.
        # Establish once before final sealing; consumers verify the same owner.
        object.__setattr__(a,'_connection_binding',b)
        b.metric_connection.verify(b)
        return HeldGeometry(host,held),b
    except BaseException:held.close();raise
