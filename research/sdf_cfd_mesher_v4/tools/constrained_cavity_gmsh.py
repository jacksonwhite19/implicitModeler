"""Isolated standard constrained Gmsh worker, no shell state or globals copied.

Never run an optimizer on malformed generated topology. Persist raw output and
stage events before every native operation. The parent enforces 120 seconds.
"""
import sys,time,json,resource,ctypes,traceback
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
import gmsh
from constrained_cavity import *
from constrained_netgen_adapter import load_volume_classified

def run(input_file,out,method,points):
    ctypes.CDLL(None).prctl(4,0,0,0,0) # no new unbounded kernel crash dump; compact inputs remain
    out.mkdir(parents=True,exist_ok=False);start=time.perf_counter();data=dict(np.load(input_file));x,t,f=data['x'],data['t'],data['f'];bvs=np.unique(f)
    result=dict(method=method,point_rule=points,input=str(input_file),stages=[])
    def event(stage,**kw):
        row=dict(stage=stage,elapsed=time.perf_counter()-start,peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,**kw)
        with (out/'EVENTS.jsonl').open('a') as fp:fp.write(json.dumps(row)+'\n')
        print(json.dumps(row),flush=True)
    added=[]
    if points=='provided':added=list(np.asarray(data['steiner_points']).reshape(-1,3))
    elif points=='one':added=[x[t[np.argmax(volume(x[t]))]].mean(0)]
    elif points=='multi':
        # One interior point per face-connected bad seed region is handled by
        # the input; two largest existing tets provide separated inner sites.
        ids=np.argsort(-volume(x[t]),kind='stable')[:min(3,len(t))];added=[x[t[i]].mean(0) for i in ids]
    elif points=='kernel':
        p,k=kernel_point(x,f)
        if not k['strict_kernel']:raise ValueError('NoStrictVisibilityKernel')
        added=[p]
    if len({p.tobytes() for p in added})!=len(added):raise ValueError('DuplicateSteinerPoint')
    if any(any(np.array_equal(p,v) for v in x[bvs]) for p in added):raise ValueError('SteinerOnBoundaryVertex')
    if not all(interior_point_valid(p,x,t) for p in added):raise ValueError('SteinerNotStrictlyInside')
    np.save(out/'STEINER.npy',np.asarray(added).reshape(-1,3))
    gmsh.initialize([],False)
    try:
        options={'General.Terminal':1,'General.Verbosity':4,'General.NumThreads':1,'Mesh.MaxNumThreads1D':1,'Mesh.MaxNumThreads2D':1,'Mesh.MaxNumThreads3D':1,'Mesh.Algorithm3D':10 if method=='hxt' else 1,'Mesh.ElementOrder':1,'Mesh.MeshSizeFromPoints':0,'Mesh.MeshSizeFromCurvature':0,'Mesh.MeshSizeExtendFromBoundary':0,'Mesh.Optimize':0,'Mesh.OptimizeNetgen':0,'Mesh.Reproducible':1,'Mesh.RandomSeed':1,'Mesh.RandomFactor':1e-12,'Mesh.RandomFactor3D':1e-12,'Mesh.Smoothing':0}
        for k,v in options.items():gmsh.option.setNumber(k,v)
        gmsh.model.add('constrained_cavity');gmsh.model.addDiscreteEntity(2,1)
        gmsh.model.mesh.addNodes(2,1,(bvs+1).tolist(),x[bvs].ravel().tolist());gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(f)+1).tolist(),(f+1).ravel().tolist())
        gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True);gmsh.model.addDiscreteEntity(3,1,[1]);gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
        for i,p in enumerate(added):
            tag=i+2;gmsh.model.addDiscreteEntity(0,tag);gmsh.model.mesh.addNodes(0,tag,[len(x)+i+1],p.tolist());gmsh.model.mesh.addElementsByType(tag,15,[len(f)+i+1],[len(x)+i+1]);gmsh.model.mesh.embed(0,[tag],3,1)
        h=float(np.median(np.linalg.norm(x[f]-x[np.roll(f,1,axis=1)],axis=2)))
        # Constant size options avoid a Python callback lifetime in native code.
        gmsh.option.setNumber('Mesh.MeshSizeMin',h);gmsh.option.setNumber('Mesh.MeshSizeMax',h)
        save(out/'OPTIONS.json',dict(options=options,h=h,embedded_points=added))
        def extract(stage):
            tags,coords,_=gmsh.model.mesh.getNodes();tags=np.asarray(tags);coords=np.asarray(coords).reshape(-1,3);order=np.argsort(tags);tags=tags[order];coords=coords[order]
            et,_,nodes=gmsh.model.mesh.getElements(3);tt=[]
            for typ,ns in zip(et,nodes):
                if typ!=4:raise ValueError('NonTetOutput')
                tt.extend(np.searchsorted(tags,np.asarray(ns).reshape(-1,4)))
            tt=np.asarray(tt,dtype='i8').reshape(-1,4);np.savez_compressed(out/(stage+'_RAW.npz'),x=coords,t=tt,tags=tags)
            lookup={p.tobytes():i for i,p in enumerate(x)};xx=list(x);mapping=[]
            for p in coords:
                if p.tobytes() not in lookup:lookup[p.tobytes()]=len(xx);xx.append(p)
                mapping.append(lookup[p.tobytes()])
            xx=np.array(xx);mapped=np.array(mapping,dtype='i8')[tt];mapped=orient(xx,mapped)
            iv=invariants(xx,t,mapped);row=dict(stage=stage,invariants=iv,output_tets=len(tt),new_points=len(xx)-len(x))
            if iv['passed']:
                q,_=quality(data,xx,mapped);row['quality']=q
                np.savez_compressed(out/(stage+'_CANDIDATE.npz'),x=xx,t=mapped)
            result['stages'].append(row);save(out/'RESULT.json',result);event(stage+'_validated',invariants=iv['passed'],tets=len(tt),failures=row.get('quality',{}).get('failures'))
            return iv['passed']
        event('generate_start');tick=time.perf_counter();gmsh.model.mesh.generate(3);result['generate_seconds']=time.perf_counter()-tick;event('generate_end')
        valid=extract('generated')
        if method=='netgen_safe' and valid:
            # Gmsh 4.15.2 buildNetgenStructure imports boundary vertices and
            # region mesh_vertices, but does not import vertices classified on
            # embedded 0D entities. Rebuild only the valid compact cavity mesh
            # with every interior vertex classified on the volume before the
            # standard Netgen optimizer. Coordinates/connectivity are unchanged.
            ar=np.load(out/'generated_CANDIDATE.npz');xx,tt=ar['x'],ar['t'];classification=load_volume_classified(gmsh,xx,tt,f)
            event('interior_classification_repaired',**classification)
        if method in ('netgen','netgen_safe') and valid:
            event('netgen_start');tick=time.perf_counter();gmsh.model.mesh.optimize('Netgen',force=True,niter=1,dimTags=[(3,1)]);result['netgen_seconds']=time.perf_counter()-tick;event('netgen_end');extract('optimized')
        elif method in ('netgen','netgen_safe'):result['netgen_skipped']='Generated cavity invariant failed'
    except Exception as e:result.update(error=str(e),traceback=traceback.format_exc());event('error',message=str(e))
    finally:
        event('finalize_start');gmsh.finalize();event('finalize_end')
    result.update(seconds=time.perf_counter()-start,peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
    save(out/'RESULT.json',result)

if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]),sys.argv[3],sys.argv[4])
