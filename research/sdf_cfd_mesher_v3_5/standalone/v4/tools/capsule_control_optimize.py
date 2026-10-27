"""Recovered one-Gmsh/one-Netgen sequence on an isolated experimental core."""
import time
import resource
import numpy as np
import gmsh
from capsule_sizing_control import OUT,field_inputs,setup,write,boundary_audit,digest


def arrays():
    tags,raw,_=gmsh.model.mesh.getNodes();tags=np.array(tags);p=np.array(raw).reshape(-1,3)
    order=np.argsort(tags);tags=tags[order];p=p[order]
    kinds,_,conn=gmsh.model.mesh.getElements(3)
    assert list(kinds)==[4]
    t=np.searchsorted(tags,np.asarray(conn[0]).reshape(-1,4))
    return p,t


def _historical_run(candidate):
    """Retained comparison implementation, not production dispatch."""
    folder=OUT/candidate;data=np.load(folder/'CORE.npz');p=data['points'];t=data['tets']
    x,tri,_=field_inputs();outer=np.load(folder/'OUTER.npz')
    lookup={tuple(v):i+1 for i,v in enumerate(p)}
    inner=np.array([lookup[tuple(v)] for v in x])[tri]
    far=np.array([lookup[tuple(v)] for v in outer['points']])[outer['triangles']]
    result=dict(input=digest(folder/'CORE.npz'),stages=[]);started=time.perf_counter()
    gmsh.initialize([],False)
    try:
        setup(gmsh);gmsh.model.add('isolated_saved_core')
        gmsh.model.addDiscreteEntity(2,1);gmsh.model.addDiscreteEntity(2,2)
        gmsh.model.addDiscreteEntity(3,1,[2,-1])
        gmsh.model.mesh.addNodes(3,1,np.arange(1,len(p)+1),p.ravel())
        gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(inner)+1),inner.ravel())
        gmsh.model.mesh.addElementsByType(2,2,np.arange(len(inner)+1,len(inner)+1+len(far)),far.ravel())
        gmsh.model.mesh.addElementsByType(1,4,np.arange(len(inner)+len(far)+1,len(inner)+len(far)+1+len(t)),(t+1).ravel())
        gmsh.model.mesh.reclassifyNodes()
        before=arrays();assert np.array_equal(before[0],p) and np.array_equal(before[1],t)
        for name in ('Gmsh','Netgen'):
            tick=time.perf_counter();cpu=time.process_time()
            if name=='Netgen':
                # Isolated saved-core reload, followed by volume-only geometry.
                gmsh.write(str(folder/'PRECONDITIONED.msh'))
                gmsh.finalize();gmsh.initialize([],False);setup(gmsh);gmsh.open(str(folder/'PRECONDITIONED.msh'))
                a=arrays();gmsh.model.mesh.createGeometry([(3,1)]);b=arrays()
                assert all(np.array_equal(u,v) for u,v in zip(a,b))
            gmsh.model.mesh.optimize(name,force=True)
            p,t=arrays();audit,interface_owners,_=boundary_audit(p,t,x,tri)
            # Outer coordinates and face identity are also immutable.
            lookup={tuple(v):i for i,v in enumerate(p)}
            far_ids=np.array([lookup[tuple(v)] for v in outer['points']])
            assert np.array_equal(p[far_ids],outer['points'])
            np.savez(folder/(name.upper()+'.npz'),points=p,tets=t)
            if name=='Netgen':np.save(folder/'FINAL_CORE_OWNERS.npy',interface_owners)
            result['stages'].append(dict(name=name,seconds=time.perf_counter()-tick,cpu=time.process_time()-cpu,
                core_vertices=len(p),tetrahedra=len(t),interface=audit))
        result['passed_boundary']=True
    except Exception as exc:
        result['error']=str(exc);result['passed_boundary']=False
        import traceback
        result['traceback']=traceback.format_exc()
    finally:
        if gmsh.isInitialized():gmsh.finalize()
    result.update(seconds=time.perf_counter()-started,peak_rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
    write(folder/'OPTIMIZATION.json',result)
    return result


def run(candidate):
    from core_backend import optimize
    x,tri,_=field_inputs()
    return optimize(OUT/candidate,x,tri)


if __name__=='__main__':
    import sys,json
    print(json.dumps(run(sys.argv[1]),indent=2))
