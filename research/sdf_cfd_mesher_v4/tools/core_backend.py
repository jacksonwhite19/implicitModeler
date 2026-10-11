"""Case boundary for the existing C1 Gmsh/Netgen sequence. No new mesher.

The capsule compatibility runner and standalone tool share this implementation.
The default retains C1 sizing and optimization. The explicit owned HXT method
uses four 3D threads and scales the generation target to 0.95*h(x) in 3D only.
Surface transfer, boundary audits, optimizer calls and final acceptance are retained.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from owned_runtime import V4
sys.path.insert(0,str(V4/'runtime/lib/gmsh'))
from capsule_sizing_control import (setup, surface_arrays, boundary_audit,
                                    MeasuredCallback, DRIVER, digest as file_hash,
                                    write as write_record)
from generic_core_sizing import GmshSizingAdapter, add_analytic_box
from optimized_core_sizing import ExactCachedSizingField

# Existing portable C1 report writer. These are diagnostic/result records,
# never authenticated checkpoint commits or restart authorities.
durable_json = write_record


def locks(method=None):
    import gmsh
    if gmsh.__version__ != '4.15.2':
        raise ValueError('RequiredQualifiedGmshVersion')
    paths = {'setup': V4/'tools/gmsh_core_setup.py', 'backend': Path(__file__),
             'core_field': V4/'tools/generic_core_sizing.py',
             'optimized_field': V4/'tools/optimized_core_sizing.py',
             'capsule_helpers': V4/'tools/capsule_sizing_control.py',
             'gmsh_python': Path(gmsh.__file__), 'gmsh_library': Path(gmsh.lib._name)}
    manifest=json.loads((V4/'runtime/MANIFEST.json').read_text(encoding='utf-8'))
    approved={r['path']:r['sha256'] for r in manifest['files']}
    for name in ('gmsh_python','gmsh_library'):
        expected=V4/'runtime/lib/gmsh'/('gmsh.py' if name=='gmsh_python' else 'libgmsh.so.4.15')
        if paths[name].resolve()!=expected.resolve() or file_hash(expected)!=approved.get(expected.relative_to(V4).as_posix()):
            raise ValueError('UnreviewedCoreDependency',name)
    from background_sizing import selection_record
    sizing=selection_record()
    result=dict(version=gmsh.__version__, files={k: file_hash(p) for k,p in paths.items()},sizing=sizing)
    if method is not None:
        from background_backend_method import selection_record
        result['backend_method']=selection_record(method)
    return result


def generate(x, tri, field, bounds, folder, *, candidate=None,execution=None):
    """Same C1 generation; explicit box and immutable input triangles."""
    import gmsh
    if gmsh.__version__ != '4.15.2':raise ValueError('RequiredQualifiedGmshVersion')
    folder=Path(folder);folder.mkdir(exist_ok=False)
    started=time.perf_counter();callback=MeasuredCallback(field) if execution is None else execution.callback
    configure=setup if execution is None else execution.setup
    install=gmsh.model.mesh.setSizeCallback if execution is None else execution.install
    generate_mesh=gmsh.model.mesh.generate if execution is None else execution.generate
    finalize=gmsh.finalize if execution is None else execution.finalize
    result=dict(candidate=candidate,status='running',production_selected=False,phase={})
    def snapshot(stage):
        result.update(stage=stage,wall=time.perf_counter()-started,callback=callback.report())
        durable_json(folder/'RESULT.json',result)
    try:
        gmsh.initialize([],False);configure(gmsh);gmsh.model.add('experimental_outer')
        add_analytic_box(gmsh,bounds);gmsh.model.occ.synchronize()
        GmshSizingAdapter(field).install(gmsh);install(callback)
        t=time.perf_counter();generate_mesh(2);far,faces=surface_arrays(gmsh)
        # Same orientation formula, translated to the explicitly supplied box.
        # Centroid direction is valid for its six convex planar patches.
        centre=np.asarray(bounds,dtype=float).mean(axis=1)
        q=far[faces];flip=np.einsum('ij,ij->i',np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0]),q.mean(1)-centre)<0
        faces[flip]=faces[flip][:,[0,2,1]]
        result['phase']['outer_discretization']=time.perf_counter()-t
        result['outer']=dict(vertices=len(far),faces=len(faces))
        np.savez(folder/'OUTER.npz',points=far,triangles=faces)
        finalize();snapshot('outer_complete')
        gmsh.initialize([],False);configure(gmsh);gmsh.model.add('experimental_core')
        gmsh.model.addDiscreteEntity(2,1);gmsh.model.mesh.addNodes(2,1,np.arange(1,len(x)+1),x.ravel())
        gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(tri)+1),(tri+1).ravel())
        gmsh.model.addDiscreteEntity(2,2);offset=len(x)+1
        gmsh.model.mesh.addNodes(2,2,np.arange(offset,offset+len(far)),far.ravel())
        gmsh.model.mesh.addElementsByType(2,2,np.arange(len(tri)+1,len(tri)+1+len(faces)),(faces+offset).ravel())
        gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True)
        gmsh.model.addDiscreteEntity(3,1,[2,-1]);gmsh.model.mesh.createGeometry([(3,1)])
        GmshSizingAdapter(field).install(gmsh);install(callback)
        snapshot('core_generation');t=time.perf_counter();generate_mesh(3)
        result['phase']['point_insertion_and_tetrahedralization']=time.perf_counter()-t
        tags,raw,_=gmsh.model.mesh.getNodes();tags=np.array(tags);points=np.array(raw).reshape(-1,3)
        order=np.argsort(tags);tags=tags[order];points=points[order]
        types,_,conn=gmsh.model.mesh.getElements(3)
        if list(types)!=[4]:raise ValueError('NotLinearTetrahedra')
        tets=np.searchsorted(tags,np.asarray(conn[0]).reshape(-1,4))
        np.savez(folder/'CORE.npz',points=points,tets=tets)
        result['core']=dict(vertices=len(points),tetrahedra=len(tets),interior_vertices=len(points)-len(x)-len(far))
        audit,owners,_=boundary_audit(points,tets,x,tri)
        if audit['nonmanifold_faces']:raise ValueError('NonmanifoldCore')
        result['interface']=audit;np.save(folder/'RAW_CORE_OWNERS.npy',owners)
        result['status']='generated_interface_passed';snapshot('core_saved')
    except BaseException as error:
        import traceback
        result.update(status='blocked',error=str(error),traceback=traceback.format_exc());snapshot('failed')
        raise
    finally:
        if gmsh.isInitialized():finalize()
    return result


def arrays(gmsh):
    # Original optimizer extraction order and tag mapping.
    tags,raw,_=gmsh.model.mesh.getNodes();tags=np.array(tags);p=np.array(raw).reshape(-1,3)
    order=np.argsort(tags);tags=tags[order];p=p[order]
    kinds,_,conn=gmsh.model.mesh.getElements(3)
    if list(kinds)!=[4]:raise ValueError('NotLinearTetrahedra')
    return p,np.searchsorted(tags,np.asarray(conn[0]).reshape(-1,4))


def optimize(folder,x,tri,*,execution=None):
    """Unchanged C1 one Gmsh pass, isolated reload, one Netgen pass."""
    import gmsh
    configure=setup if execution is None else execution.setup
    folder=Path(folder);data=np.load(folder/'CORE.npz');p,t=data['points'],data['tets']
    outer=np.load(folder/'OUTER.npz');lookup={tuple(v):i+1 for i,v in enumerate(p)}
    inner=np.array([lookup[tuple(v)] for v in x])[tri]
    far=np.array([lookup[tuple(v)] for v in outer['points']])[outer['triangles']]
    result=dict(input=file_hash(folder/'CORE.npz'),stages=[]);started=time.perf_counter()
    gmsh.initialize([],False)
    try:
        configure(gmsh);gmsh.model.add('isolated_saved_core')
        gmsh.model.addDiscreteEntity(2,1);gmsh.model.addDiscreteEntity(2,2)
        gmsh.model.addDiscreteEntity(3,1,[2,-1])
        gmsh.model.mesh.addNodes(3,1,np.arange(1,len(p)+1),p.ravel())
        gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(inner)+1),inner.ravel())
        gmsh.model.mesh.addElementsByType(2,2,np.arange(len(inner)+1,len(inner)+1+len(far)),far.ravel())
        gmsh.model.mesh.addElementsByType(1,4,np.arange(len(inner)+len(far)+1,len(inner)+len(far)+1+len(t)),(t+1).ravel())
        gmsh.model.mesh.reclassifyNodes()
        before=arrays(gmsh)
        if not np.array_equal(before[0],p) or not np.array_equal(before[1],t):raise ValueError('ReloadedCoreChanged')
        for name in ('Gmsh','Netgen'):
            tick=time.perf_counter()
            if name=='Netgen':
                gmsh.write(str(folder/'PRECONDITIONED.msh'))
                gmsh.finalize();gmsh.initialize([],False);configure(gmsh);gmsh.open(str(folder/'PRECONDITIONED.msh'))
                a=arrays(gmsh);gmsh.model.mesh.createGeometry([(3,1)]);b=arrays(gmsh)
                if any(not np.array_equal(u,v) for u,v in zip(a,b)):raise ValueError('VolumeGeometryReloadChanged')
            gmsh.model.mesh.optimize(name,force=True)
            p,t=arrays(gmsh);audit,interface_owners,_=boundary_audit(p,t,x,tri)
            if audit['nonmanifold_faces']:raise ValueError('NonmanifoldCore')
            lookup={tuple(v):i for i,v in enumerate(p)}
            far_ids=np.array([lookup[tuple(v)] for v in outer['points']])
            if not np.array_equal(p[far_ids],outer['points']):raise ValueError('ChangedFarfieldCoordinates')
            np.savez(folder/(name.upper()+'.npz'),points=p,tets=t)
            if name=='Netgen':np.save(folder/'FINAL_CORE_OWNERS.npy',interface_owners)
            result['stages'].append(dict(name=name,seconds=time.perf_counter()-tick,
                core_vertices=len(p),tetrahedra=len(t),interface=audit))
        result['passed_boundary']=True
    except BaseException as error:
        import traceback
        result.update(error=str(error),passed_boundary=False,traceback=traceback.format_exc())
        raise
    finally:
        if gmsh.isInitialized():gmsh.finalize()
        result['seconds']=time.perf_counter()-started
        durable_json(folder/'OPTIMIZATION.json',result)
    return result


def worker(folder):
    import resource,threading,shutil,os
    folder=Path(folder);request=json.loads((folder/'INPUT.json').read_text(encoding='utf-8'))
    from background_backend_method import request_method,LEGACY,DirectExecution
    method=request_method(request);expected=locks(method)
    if request['locks']!=expected:raise ValueError('CoreBackendIdentityChanged')
    if method is not None and request.get('backend_method_identity')!=expected['backend_method']:
        raise ValueError('ChangedBackgroundMethodRequest')
    for name,pin in request['files'].items():
        if file_hash(folder/name)!=pin:raise ValueError('ChangedCoreBackendInput',name)
    data=np.load(folder/'INPUT.npz')
    x,tri,seeds=data['points'],data['triangles'],data['seeds']
    from background_sizing import Field
    execution=None
    if method is None or method==LEGACY:
        field=Field(x[tri],seeds,g=request['growth_per_m'],h_max=request['maximum_size_m'])
    else:
        import gmsh
        execution=DirectExecution(gmsh,x[tri],seeds,g=request['growth_per_m'],h_max=request['maximum_size_m'],
            selection=expected['backend_method'])
        field=execution.field
    start=time.perf_counter()
    stopped=threading.Event()
    def resources():
        # Same operational reserve as the Windows supervisor. Gmsh runs in
        # Linux; terminating only its Windows transport is not sufficient.
        while not stopped.wait(1):
            try:
                free=shutil.disk_usage(folder).free
                if free<request['resources']['minimum_free_bytes']:
                    raise ValueError('OperationalResourceLimit',free,request['resources']['minimum_free_bytes'])
            except BaseException as error:
                try:
                    durable_json(folder/'BACKEND_RESOURCE_STOP.json',dict(error=str(error),
                        seconds=time.perf_counter()-start))
                finally:
                    try:print('Core resource monitor failed; worker stopped: '+str(error),file=sys.stderr,flush=True)
                    finally:os._exit(2)
    watcher=threading.Thread(target=resources,daemon=True);watcher.start()
    try:
        options={} if execution is None else dict(execution=execution)
        result=generate(x,tri,field,request['bounds_m'],folder/'generated',**options)
        optimize(folder/'generated',x,tri,**options)
        report=None
        if execution is not None:report=execution.finish()
        elif method is not None:report=dict(passed=True,selection=expected['backend_method'],execution='existing_worker')
        if locks(method)!=expected:raise ValueError('ChangedBackgroundDependenciesAfterWorker')
        names=('NETGEN.npz','OUTER.npz','CORE.npz','GMSH.npz','RAW_CORE_OWNERS.npy','FINAL_CORE_OWNERS.npy')
        payload=dict(passed=True,locks=expected,generation=result,
            input_sha256=file_hash(folder/'INPUT.json'),
            files={name:file_hash(folder/'generated'/name) for name in names},
            optimization=json.loads((folder/'generated/OPTIMIZATION.json').read_text(encoding='utf-8')))
        if report is not None:payload['backend_method']=report
        durable_json(folder/'BACKEND.json',payload)
    finally:
        stopped.set();watcher.join()
        try:
            if execution is not None:execution.close()
            else:field.close()
        finally:
            durable_json(folder/'BACKEND_RESOURCES.json',dict(seconds=time.perf_counter()-start,
                process_peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
                platform='Linux; actual Gmsh worker, not the Windows WSL launcher'))


if __name__=='__main__':
    sys.dont_write_bytecode=True
    parser=argparse.ArgumentParser();parser.add_argument('--probe',action='store_true');parser.add_argument('--input',type=Path)
    parser.add_argument('--method')
    args=parser.parse_args()
    if args.probe:print(json.dumps(locks(args.method),sort_keys=True))
    else:worker(args.input)
