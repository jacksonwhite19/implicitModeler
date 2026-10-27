"""Isolated constrained cavity proposer, reusing V4's discrete Gmsh recipe.

Generated meshes are private search states. Only the parent can admit a whole
cavity, with the unchanged V3.5 shape, neighbour and geometry predicates.
"""
from pathlib import Path
import json,sys,time,traceback
ROOT=Path(__file__).resolve().parent
from direct_compat import install
install()
sys.path[:0]=[str(ROOT/'v4/runtime/lib/finite_native_packages'),str(ROOT/'v4/tools'),str(ROOT/'v4/src'),str(ROOT/'v4/runtime/lib/gmsh')]
import owned_runtime
owned_runtime.activate()
import numpy as np

def save(p,r):
    Path(p).write_text(json.dumps(r,indent=2)+'\n')

def intermediate(x,t,boundary_points,boundary_faces):
    """Exact combinatorial/volume guard before passing a mesh to Netgen.

    Does not enforce final shape or confer acceptance. Boundary coordinates are
    mapped by bytes; no approximate welding or duplicate-cell removal.
    """
    from background_materialization import build
    from background_topology_repair import orient,boundary
    import local_termination_topology as topology
    from fractions import Fraction
    if not len(t) or not np.isfinite(x).all() or len({p.tobytes() for p in x})!=len(x):raise ValueError('InvalidNativePointPopulation')
    if len({tuple(sorted(row)) for row in t})!=len(t):raise ValueError('DuplicateNativeTetrahedra')
    lookup={p.tobytes():i for i,p in enumerate(x)}
    try:mapping=np.asarray([lookup[p.tobytes()] for p in boundary_points],dtype='<i8')
    except KeyError:raise ValueError('ChangedNativeBoundaryCoordinate') from None
    rows=np.asarray(orient(x,t),dtype='<i8');cells=build(rows,0)
    expected={tuple(sorted(map(int,f))):(-1 if sum(f[i]>f[j] for i in range(3) for j in range(i+1,3))%2 else 1) for f in mapping[boundary_faces]}
    if boundary(cells)[0]!=expected:raise ValueError('ChangedNativeOrientedBoundary')
    volume=sum(Fraction(topology.exact_cell(x,c)['signed_volume']) for c in cells)
    return rows,mapping,volume

def run(folder,pin):
    from sdf_cfd_mesher_v4.common.authority import file_hash
    from direct_runtime_identity import RuntimeIdentity
    import gmsh
    from constrained_netgen_adapter import load_volume_classified
    from background_materialization import build
    import local_termination_topology as topology
    from fractions import Fraction
    request=folder/'INPUT.json'
    if file_hash(request)!=pin:raise ValueError('ChangedCavityRequest')
    req=json.loads(request.read_text())
    if file_hash(folder/'INPUT.npz')!=req['input_sha256']:raise ValueError('ChangedCavityInput')
    owned_runtime.verify();runtime=RuntimeIdentity(req['source_files']);save(folder/'RUNTIME_PRE.json',runtime.verify())
    import core_backend
    if core_backend.locks()!=req['locks']:raise ValueError('ChangedConstrainedBackendLocks')
    with np.load(folder/'INPUT.npz',allow_pickle=False) as z:d={k:z[k].copy() for k in z.files}
    x,t,f=d['points'],d['tetrahedra'],d['boundary'];bvs=np.unique(f);interior=np.setdiff1d(np.unique(t),bvs)
    expected_volume=sum(Fraction(topology.exact_cell(x,c)['signed_volume']) for c in build(t,0))
    result=dict(transport_passed=False,request_sha256=pin,stages=[],scientific_acceptance=False)
    start=time.monotonic()
    def event(stage):
        with (folder/'EVENTS.jsonl').open('a') as fp:fp.write(json.dumps(dict(stage=stage,seconds=time.monotonic()-start))+'\n')
        print(stage,flush=True)
    def extract(stage):
        tags,coords,_=gmsh.model.mesh.getNodes();coords=np.asarray(coords,dtype='<f8').reshape(-1,3);tags=np.asarray(tags);order=np.argsort(tags);tags=tags[order];coords=coords[order]
        types,_,nodes=gmsh.model.mesh.getElements(3);rows=[]
        for typ,ns in zip(types,nodes):
            if typ!=4:raise ValueError('NonTetCavityOutput')
            ns=np.asarray(ns).reshape(-1,4);indices=np.searchsorted(tags,ns)
            if np.any(indices>=len(tags)) or not np.array_equal(tags[indices],ns):raise ValueError('MissingNativeNode')
            rows.extend(indices)
        rows=np.asarray(rows,dtype='<i8').reshape(-1,4)
        if len(rows)>req['max_tets'] or len(coords)>len(x)+req['max_new_points']:raise ValueError('NativePopulationBudget')
        # Raw data survives every subsequent predicate/optimizer refusal.
        np.savez_compressed(folder/(stage+'.npz'),points=coords,tetrahedra=rows,tags=tags)
        row=dict(stage=stage,sha256=file_hash(folder/(stage+'.npz')),points=len(coords),cells=len(rows),intermediate_passed=False);result['stages'].append(row)
        try:
            oriented,mapping,volume=intermediate(coords,rows,x[bvs],np.searchsorted(bvs,f))
            if volume!=expected_volume:raise ValueError('NativeExactVolumeMismatch')
            row['intermediate_passed']=True;row['exact_volume']=str(volume)
            return coords,oriented,mapping[np.searchsorted(bvs,f)]
        except ValueError as e:row['refusal']=str(e);return None
        finally:save(folder/'RESULT.json',result)
    gmsh.initialize([],False)
    try:
        # Same deterministic constrained discrete-boundary recipe used by V4.
        options={'General.Terminal':1,'General.Verbosity':3,'General.NumThreads':1,'Mesh.MaxNumThreads1D':1,'Mesh.MaxNumThreads2D':1,'Mesh.MaxNumThreads3D':1,'Mesh.Algorithm3D':1,'Mesh.ElementOrder':1,'Mesh.MeshSizeFromPoints':0,'Mesh.MeshSizeFromCurvature':0,'Mesh.MeshSizeExtendFromBoundary':0,'Mesh.Optimize':0,'Mesh.OptimizeNetgen':0,'Mesh.Reproducible':1,'Mesh.RandomSeed':1,'Mesh.RandomFactor':1e-12,'Mesh.RandomFactor3D':1e-12,'Mesh.Smoothing':0}
        for k,v in options.items():gmsh.option.setNumber(k,v)
        gmsh.model.add('bounded_constrained_cavity');gmsh.model.addDiscreteEntity(2,1)
        gmsh.model.mesh.addNodes(2,1,(bvs+1).tolist(),x[bvs].ravel().tolist());gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(f)+1).tolist(),(f+1).ravel().tolist())
        gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True);gmsh.model.addDiscreteEntity(3,1,[1]);gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
        sites=[(int(i)+1,x[i]) for i in interior];rule=req['point_rule']
        if rule=='seed':additional=[x[d['seed_tet']].mean(0)]
        elif rule=='spread':
            q=x[t];v=abs(np.linalg.det(q[:,1:]-q[:,0,None]));additional=[q[i].mean(0) for i in np.argsort(-v,kind='stable')[:3]]
        elif rule=='existing':additional=[]
        else:raise ValueError('UnknownCavityPointRule')
        known={p.tobytes() for p in x};added=[]
        for p in additional:
            if p.tobytes() not in known:known.add(p.tobytes());added.append(p)
        sites +=[(len(x)+i+1,p) for i,p in enumerate(added)]
        for i,(node,p) in enumerate(sites):
            tag=i+2;gmsh.model.addDiscreteEntity(0,tag);gmsh.model.mesh.addNodes(0,tag,[node],p.tolist());gmsh.model.mesh.addElementsByType(tag,15,[len(f)+i+1],[node]);gmsh.model.mesh.embed(0,[tag],3,1)
        h=float(np.median(np.linalg.norm(x[f]-x[np.roll(f,1,axis=1)],axis=2)))
        gmsh.option.setNumber('Mesh.MeshSizeMin',h);gmsh.option.setNumber('Mesh.MeshSizeMax',h)
        save(folder/'OPTIONS.json',dict(options=options,size=h,point_rule=rule,embedded_points=len(sites)))
        event('generate_start');gmsh.model.mesh.generate(3);event('generate_end');valid=extract('generated')
        if valid is not None:
            xx,tt,ff=valid
            save(folder/'CLASSIFICATION.json',load_volume_classified(gmsh,xx,tt,ff))
            event('netgen_start');gmsh.model.mesh.optimize('Netgen',force=True,niter=1,dimTags=[(3,1)]);event('netgen_end');extract('optimized')
    except (TypeError,IndexError,KeyError,AttributeError):raise
    except Exception as e:
        result.update(proposal_error=dict(type=type(e).__name__,message=str(e),traceback=traceback.format_exc()))
    finally:gmsh.finalize()
    save(folder/'RUNTIME_POST.json',runtime.verify())
    result.update(transport_passed=True,seconds=time.monotonic()-start,
        files={p.name:file_hash(p) for p in folder.iterdir() if p.name in {'RUNTIME_PRE.json','RUNTIME_POST.json','OPTIONS.json','CLASSIFICATION.json','generated.npz','optimized.npz','EVENTS.jsonl'}})
    save(folder/'RESULT.json',result)

if __name__=='__main__':run(Path(sys.argv[1]),sys.argv[2])
