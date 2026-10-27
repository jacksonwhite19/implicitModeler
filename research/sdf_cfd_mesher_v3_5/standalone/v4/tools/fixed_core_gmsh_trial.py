"""One bounded standard Gmsh operation, with an exact frozen surface contract."""
import sys,time,json,resource,traceback
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
import gmsh
from scipy.spatial import cKDTree
from fixed_core_study import Interface,evaluate,save,load_core,quantiles,local_sizes
from broader_geometry_core import extract_all_volumes,components
from repair_bulk_core import V3,load

def run(src,interface,dest,method):
    dest.mkdir(parents=True,exist_ok=False);start=time.perf_counter();iface=Interface(interface);result=dict(method=method)
    old=load('fixed_gmsh_reference',V3/'airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py')
    gmsh.initialize([],False)
    try:
        old.setup_gmsh(gmsh,1);gmsh.option.setNumber('General.Verbosity',3)
        if method=='hxt_sized':
            gmsh.option.setNumber('Mesh.Algorithm3D',10);gmsh.model.add('fixed_core')
            groups=components(iface.f)
            for i,g in enumerate(groups,1):
                gmsh.model.addDiscreteEntity(2,i);ids=np.unique(iface.f[g])
                gmsh.model.mesh.addNodes(2,i,(ids+1).tolist(),iface.x[ids].ravel().tolist())
                gmsh.model.mesh.addElementsByType(i,2,(g+1).tolist(),(iface.f[g]+1).ravel().tolist())
            gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True)
            for i in range(1,len(groups)+1):gmsh.model.addDiscreteEntity(3,i,[-i])
            gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
            h,face_h=local_sizes(iface);tree=cKDTree(iface.x)
            calls=[0]
            def sizing(dim,tag,a,b,c,lc):
                d,k=tree.query((a,b,c));calls[0]+=1
                return min(float(h.max()),float(h[k])+float(d))
            gmsh.model.mesh.setSizeCallback(sizing)
            save(dest/'SIZING.json',dict(rule='min(equilateral area edge,minimum altitude); nearest boundary vertex scale plus distance, capped by maximum boundary scale',vertex_scale=quantiles(h),face_scale=quantiles(face_h)))
            tick=time.perf_counter();gmsh.model.mesh.generate(3);result['operation_seconds']=time.perf_counter()-tick;result['size_queries']=calls[0]
        else:
            if (src/'core.msh').exists():gmsh.open(str(src/'core.msh'))
            else:
                xx,tt,cc,_=load_core(src,interface);tags=np.arange(1,len(xx)+1)
                gmsh.model.add('fixed_array_core');gmsh.model.addDiscreteEntity(2,1)
                gmsh.model.mesh.addNodes(2,1,tags[cc].tolist(),xx[cc].ravel().tolist())
                gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(iface.f)+1).tolist(),tags[cc[iface.f]].ravel().tolist())
                gmsh.model.addDiscreteEntity(3,1,[-1]);inside=np.setdiff1d(np.arange(len(xx)),cc)
                gmsh.model.mesh.addNodes(3,1,tags[inside].tolist(),xx[inside].ravel().tolist())
                gmsh.model.mesh.addElementsByType(1,4,np.arange(len(iface.f)+1,len(iface.f)+1+len(tt)).tolist(),tags[tt].ravel().tolist())
            if method=='Netgen':gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
            tick=time.perf_counter();gmsh.model.mesh.optimize(method,force=True,niter=1,dimTags=gmsh.model.getEntities(3));result['operation_seconds']=time.perf_counter()-tick
        arrays=extract_all_volumes(gmsh,old)
        lookup=old.exact_coordinate_map(arrays['node_tags'],arrays['coordinates'])
        matches=[lookup.get(np.asarray(p,dtype='<f8').tobytes(),[]) for p in iface.x]
        if not all(len(z)==1 for z in matches):raise ValueError('FrozenCoordinateMappingFailed')
        it=np.array([z[0] for z in matches]);ci=np.searchsorted(arrays['node_tags'],it)
        summary,metrics=evaluate(arrays['coordinates'],arrays['tets'],ci,iface)
        arrays.update(interface_tags=it,interface_owner=metrics['interface_owner'],farfield_tags=np.empty(0,dtype='i8'))
        for k,v in arrays.items():np.save(dest/(k+'.npy'),v)
        np.savez_compressed(dest/'METRICS.npz',**metrics)
        gmsh.write(str(dest/'core.msh'));save(dest/'QUALITY.json',summary);result['quality']=summary
        before_x=np.load(src/'coordinates.npy');before_t=np.load(src/'tets.npy')
        before_set={p.tobytes() for p in before_x};after_set={p.tobytes() for p in arrays['coordinates']}
        result.update(new_coordinate_rows=len(after_set-before_set),removed_coordinate_rows=len(before_set-after_set),connectivity_array_equal=np.array_equal(before_t,arrays['tets']))
    except Exception as e:result.update(error=str(e),traceback=traceback.format_exc())
    finally:gmsh.finalize()
    result.update(seconds=time.perf_counter()-start,peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
    save(dest/'TRIAL.json',result);print(json.dumps(result),flush=True)
if __name__=='__main__':run(*map(Path,sys.argv[1:4]),sys.argv[4])
