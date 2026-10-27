"""Bounded experimental capsule sizing controls. Never selects a winner by default."""
import ast
import collections
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
from generic_core_sizing import CoreSizingField, GmshSizingAdapter, add_analytic_box

V4=Path(__file__).resolve().parents[1]
OUT=V4/'results/capsule_sizing_controls'
HANDOFF=V4/'results/capsule_user_domain_handoff'
SIZING=V4/'results/generic_core_sizing'
V3=V4.parent/'sdf_cfd_mesher_v3'
DRIVER=V3/'airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py'


def write(path,value):
    path=Path(path);path.parent.mkdir(exist_ok=True,parents=True)
    path.write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n')


import stage_observations as obs

@obs.observed('source_digest','io_hash')
def digest(path):return hashlib.sha256(obs.byte_value(Path(path).read_bytes())).hexdigest()


def reopen_control_manifest(path,expected_sha256):
    """Reopen experimental evidence only; this cannot authorize production."""
    path=Path(path)
    if digest(path)!=expected_sha256:raise ValueError('ControlManifestCorruption')
    r=json.loads(path.read_text())
    if r['scope']!='NON_PRODUCTION_EXPERIMENTAL_PARAMETER' or r['production_selected']:
        raise ValueError('UnexpectedProductionAuthority')
    if r['sizing_architecture']!='77c1c794f90eb913c351f40c09ba68cd3aae79600080195dc64f02c9e64472c7':
        raise ValueError('WrongSizingArchitecture')
    for name,value in r['files'].items():
        p=(path.parent/name).resolve()
        if not p.is_relative_to(path.parent.resolve()):raise ValueError('UnsafeControlPath')
        if digest(p)!=value:raise ValueError('ControlComponentCorruption:'+name)
    return r


def production_gate(candidates):
    """No numerical score or coarsest/finest fallback can select a pair."""
    if not candidates or any(any(c['failures'].values()) for c in candidates):
        raise ValueError('FailedCandidateCannotBeProduction')
    if any(not c.get('complete_core_certificate',False) for c in candidates):
        raise ValueError('ReducedScreensAreNotCompleteCoreQualification')
    raise ValueError('ExplicitEvidenceBackedSelectionRequired')


def distribution(x):
    x=np.asarray(x)
    return dict(count=int(x.size),minimum=float(x.min()),p05=float(np.quantile(x,.05)),
                median=float(np.median(x)),p95=float(np.quantile(x,.95)),maximum=float(x.max()))


def candidates(seeds,separation):
    """A 2x2 diagnostic family, not physical convergence authority.

    g_ref is the local slope of historical V3 growth=1.3. Half that slope
    tests slower grading. Caps correspond to the median seed grown under g_ref
    over half and all of the minimum available radial clearance. These are
    experimental bracketing choices, not newly asserted scientific rules.
    """
    reference=math.log(1.3)
    low=float(np.median(seeds))+reference*separation/2
    high=float(np.median(seeds))+reference*separation
    return [dict(id=f'C{i}',scope='NON_PRODUCTION_EXPERIMENTAL_PARAMETER',
                 g=g,h_max=cap,g_hex=g.hex(),h_max_hex=cap.hex(),regional_caps=[])
            for i,(g,cap) in enumerate(((reference/2,low),(reference,low),
                                       (reference/2,high),(reference,high)))]


def field_inputs():
    x=np.load(HANDOFF/'FRONT.npy');tri=np.load(HANDOFF/'TRIANGLES.npy');s=np.load(SIZING/'SEEDS.npy')
    return x,tri,s


def prepare():
    x,tri,s=field_inputs();domain=json.loads((HANDOFF/'DOMAIN.json').read_text());bounds=np.array(domain['bounds'])
    separation=min(float((x[:,k]-bounds[k,0]).min()) for k in range(3))
    separation=min(separation,min(float((bounds[k,1]-x[:,k]).min()) for k in range(3)))
    family=candidates(s,separation)
    write(OUT/'CANDIDATES.json',dict(candidates=family,reference_slope=math.log(1.3),
        derivation='g={ln(1.3)/2,ln(1.3)}; h_max={median_seed+ln(1.3)*minimum_patch_clearance/2,median_seed+ln(1.3)*minimum_patch_clearance}',
        purpose='2x2 controlled contrast of grading and cap; no quality threshold or production choice',
        median_seed=float(np.median(s)),minimum_patch_clearance=separation,
        inputs={str(p.relative_to(V4.parent)):digest(p) for p in [HANDOFF/'FRONT.npy',HANDOFF/'TRIANGLES.npy',HANDOFF/'DOMAIN.json',SIZING/'SEEDS.npy',DRIVER]}))
    centers=x[tri].mean(1);normal=np.cross(x[tri[:,1]]-x[tri[:,0]],x[tri[:,2]]-x[tri[:,0]])
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    ids=[int(np.argmin(s)),int(np.argsort(s,kind='stable')[len(s)//2]),int(np.argmax(s))]
    paths={label:(centers[i],normal[i]) for label,i in zip(('fine_normal','median_normal','coarse_normal'),ids)}
    for k,axis in enumerate('xyz'):
        for sign in (-1,1):
            i=int(np.argmax(sign*x[:,k]));d=np.zeros(3);d[k]=sign
            paths[f'{axis}_{"minus" if sign<0 else "plus"}']=(x[i],d)
    grid=np.array(np.meshgrid(*[np.linspace(*b,5) for b in bounds],indexing='ij')).reshape(3,-1).T
    grid=grid[~np.all((grid>=x.min(0))&(grid<=x.max(0)),axis=1)]
    for candidate in family:
        start=time.perf_counter();field=CoreSizingField(x[tri],s,g=candidate['g'],h_max=candidate['h_max'])
        profiles={}
        for label,(startpoint,direction) in paths.items():
            end=min((bounds[k,int(direction[k]>0)]-startpoint[k])/direction[k] for k in range(3) if direction[k]!=0)
            distances=np.unique(np.r_[0,.001,.01,np.linspace(0,end,9)])
            sizes=[field.evaluate(startpoint+d*direction) for d in distances]
            activation=None
            for j in range(1,len(sizes)):
                if sizes[j]==field.h_max:
                    lo,hi=float(distances[j-1]),float(distances[j])
                    for _ in range(32):
                        mid=(lo+hi)/2
                        if field.evaluate(startpoint+mid*direction)==field.h_max:hi=mid
                        else:lo=mid
                    activation=[lo,hi];break
            profiles[label]=dict(start=startpoint.tolist(),direction=direction.tolist(),distances_m=distances.tolist(),
                sizes_m=sizes,first_observed_cap_activation_bracket_m=activation,
                sampled_slopes=np.diff(sizes).astype(float).__truediv__(np.diff(distances)).tolist())
        patches={}
        for k,axis in enumerate('xyz'):
            for side in (0,1):
                p=np.zeros(3);p[k]=bounds[k,side]
                patches[f'{axis}_{"min" if side==0 else "max"}']=field.evaluate(p)
        write(OUT/(candidate['id']+'_FIELD.json'),dict(candidate=candidate,profiles=profiles,
            outer_patch_center_targets_m=patches,volume_sample=distribution([field.evaluate(p) for p in grid]),
            volume_sample_definition='5^3 Cartesian grid excluding L11 bounding box; all retained points outside L11',
            seconds=time.perf_counter()-start))
    return family


def _historical_setup(gmsh):
    # Execute the exact recovered pure V3 setup, with no module-level V3 I/O.
    tree=ast.parse(DRIVER.read_text());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='setup_gmsh')
    namespace={};exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(DRIVER),'exec'),namespace)
    namespace['setup_gmsh'](gmsh,1)


def setup(gmsh):
    from gmsh_core_setup import setup_gmsh
    setup_gmsh(gmsh,1)


class MeasuredCallback:
    def __init__(self,field):
        self.field=field;self.calls=0;self.hits=0;self.seconds=0.;self.cpu=0.;self.triangles=collections.Counter();self.cache=collections.OrderedDict()
    def __call__(self,dim,tag,x,y,z,lc):
        start=time.perf_counter();cpu=time.process_time();self.calls+=1;key=(x,y,z)
        if key in self.cache:
            value=self.cache.pop(key);self.cache[key]=value;self.hits+=1
        else:
            value,work=self.field.evaluate_with_work(key);self.triangles[work['triangles']]+=1
            self.cache[key]=value
            if len(self.cache)>65536:self.cache.popitem(last=False)
        self.seconds+=time.perf_counter()-start;self.cpu+=time.process_time()-cpu
        return value
    def report(self):
        return dict(evaluations=self.calls,exact_coordinate_cache_hits=self.hits,unique_evaluations=self.calls-self.hits,
                    seconds=self.seconds,cpu=self.cpu,queries_per_second=self.calls/self.seconds if self.seconds else 0,
                    triangles_tested_histogram=dict(self.triangles),cache_capacity=65536,
                    cache_note='Bounded exact-coordinate memoization only; no quantization or policy change')


def surface_arrays(gmsh):
    tags,coords,_=gmsh.model.mesh.getNodes();tags=np.array(tags,dtype=np.int64);coords=np.array(coords).reshape(-1,3)
    order=np.argsort(tags);tags=tags[order];coords=coords[order]
    faces=[]
    for kind,_,nodes in zip(*gmsh.model.mesh.getElements(2)):
        if kind!=2 and len(nodes):raise ValueError('NontriangularSurface')
        if kind==2:faces.extend(np.searchsorted(tags,np.asarray(nodes).reshape(-1,3)))
    return coords,np.array(faces,dtype=np.int64)


def boundary_audit(points,tets,interface,triangles,*,_return_call_data=False):
    """Exact interface identity and oriented one-core-owner incidence."""
    lookup={tuple(p):i for i,p in enumerate(points)}
    if len(lookup)!=len(points):raise ValueError('DuplicateCoordinates')
    try:ids=np.array([lookup[tuple(p)] for p in interface])
    except KeyError:raise ValueError('MovedOrMissingInterfaceVertex')
    if not np.array_equal(points[ids],interface):raise ValueError('ChangedInterfaceVertex')
    patterns=np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    ordered=tets[:,patterns].reshape(-1,3);keys=np.sort(ordered,axis=1)
    unique,inverse,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
    wanted={tuple(sorted(f)):j for j,f in enumerate(ids[triangles])}
    matches={};boundary=np.flatnonzero(counts[inverse]==1)
    for index in boundary:
        key=tuple(keys[index])
        if key in wanted:
            j=wanted[key];face=ordered[index];original=ids[triangles[j]]
            # Oriented tetra outward face must oppose the prism-owner face.
            cycles=[tuple(np.roll(original[::-1],k)) for k in range(3)]
            if tuple(face) not in cycles:raise ValueError('CorePrismWindingMismatch')
            matches[j]=index//4
    if len(matches)!=len(triangles):raise ValueError(f'MissingOrSplitInterfaceFaces:{len(matches)}/{len(triangles)}')
    result=(dict(vertices_unchanged=len(ids),faces_unchanged=len(matches),interface_steiner_points=0,
                moved_vertices=0,split_faces=0,opposite_winding=True,nonmanifold_faces=int(sum(counts>2))),np.array([matches[i] for i in range(len(triangles))]),unique[counts==1])
    if _return_call_data:
        # Products of this exact traversal, not persisted evidence or an input
        # that may override the audit. Keep the default capsule tuple unchanged.
          return (*result,dict(lookup=lookup,interface_ids=ids,
                              boundary_uses=ordered[boundary].copy(),
                              ordered=ordered,unique=unique,inverse=inverse))
    return result


def _historical_run(candidate_id):
    """Retained pre-generalization comparison owner; not production dispatch."""
    import gmsh,resource,traceback
    candidate=next(c for c in json.loads((OUT/'CANDIDATES.json').read_text())['candidates'] if c['id']==candidate_id)
    folder=OUT/candidate_id;folder.mkdir(exist_ok=False)
    started=time.perf_counter();cpu=time.process_time();x,tri,seeds=field_inputs()
    field=CoreSizingField(x[tri],seeds,g=candidate['g'],h_max=candidate['h_max']);callback=MeasuredCallback(field)
    result=dict(candidate=candidate,status='running',production_selected=False,phase={})
    def snapshot(stage):
        result.update(stage=stage,wall=time.perf_counter()-started,cpu=time.process_time()-cpu,
                      peak_rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,callback=callback.report())
        write(folder/'RESULT.json',result)
    try:
        assert gmsh.__version__=='4.15.2'
        result['phase']['field_preparation']=time.perf_counter()-started
        gmsh.initialize([],False);setup(gmsh);gmsh.model.add('experimental_outer')
        add_analytic_box(gmsh,[[-.75,.75],[-.75,.75],[-1.25,1.25]]);gmsh.model.occ.synchronize()
        GmshSizingAdapter(field).install(gmsh);gmsh.model.mesh.setSizeCallback(callback)
        t=time.perf_counter();gmsh.model.mesh.generate(2);far,faces=surface_arrays(gmsh)
        # Orient each exact planar box triangle outward using its centroid.
        q=far[faces];flip=np.einsum('ij,ij->i',np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0]),q.mean(1))<0
        faces[flip]=faces[flip][:,[0,2,1]]
        result['phase']['outer_discretization']=time.perf_counter()-t
        result['outer']=dict(vertices=len(far),faces=len(faces))
        np.savez(folder/'OUTER.npz',points=far,triangles=faces)
        gmsh.finalize();snapshot('outer_complete')
        gmsh.initialize([],False);setup(gmsh);gmsh.model.add('experimental_core')
        # V3 transfer mechanism with dynamically disjoint namespaces; no case IDs.
        gmsh.model.addDiscreteEntity(2,1);gmsh.model.mesh.addNodes(2,1,np.arange(1,len(x)+1),x.ravel())
        gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(tri)+1),(tri+1).ravel())
        gmsh.model.addDiscreteEntity(2,2);offset=len(x)+1
        gmsh.model.mesh.addNodes(2,2,np.arange(offset,offset+len(far)),far.ravel())
        gmsh.model.mesh.addElementsByType(2,2,np.arange(len(tri)+1,len(tri)+1+len(faces)),(faces+offset).ravel())
        gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True)
        gmsh.model.addDiscreteEntity(3,1,[2,-1]);gmsh.model.mesh.createGeometry([(3,1)])
        GmshSizingAdapter(field).install(gmsh);gmsh.model.mesh.setSizeCallback(callback)
        snapshot('core_generation');t=time.perf_counter();gmsh.model.mesh.generate(3)
        result['phase']['point_insertion_and_tetrahedralization']=time.perf_counter()-t
        tags,raw,_=gmsh.model.mesh.getNodes();tags=np.array(tags);points=np.array(raw).reshape(-1,3)
        order=np.argsort(tags);tags=tags[order];points=points[order]
        types,_,conn=gmsh.model.mesh.getElements(3)
        if list(types)!=[4]:raise ValueError('NotLinearTetrahedra')
        tets=np.searchsorted(tags,np.asarray(conn[0]).reshape(-1,4))
        np.savez(folder/'CORE.npz',points=points,tets=tets)
        result['core']=dict(vertices=len(points),tetrahedra=len(tets),interior_vertices=len(points)-len(x)-len(far))
        t=time.perf_counter();audit,owners,boundary=boundary_audit(points,tets,x,tri)
        result['interface']=audit;np.save(folder/'RAW_CORE_OWNERS.npy',owners)
        result['phase']['interface_audit']=time.perf_counter()-t
        result['status']='generated_interface_passed';snapshot('core_saved')
    except Exception as exc:
        result['status']='blocked';result['error']=str(exc);result['traceback']=traceback.format_exc();snapshot('failed')
    finally:
        if gmsh.isInitialized():gmsh.finalize()
    return result


def run(candidate_id):
    """Compatibility case binding to the shared qualified core sequence."""
    from core_backend import generate
    candidate=next(c for c in json.loads((OUT/'CANDIDATES.json').read_text())['candidates'] if c['id']==candidate_id)
    x,tri,seeds=field_inputs()
    field=CoreSizingField(x[tri],seeds,g=candidate['g'],h_max=candidate['h_max'])
    return generate(x,tri,field,[[-.75,.75],[-.75,.75],[-1.25,1.25]],OUT/candidate_id,candidate=candidate)


if __name__=='__main__':
    import sys
    print(json.dumps(prepare() if sys.argv[1]=='prepare' else run(sys.argv[1]),indent=2))
