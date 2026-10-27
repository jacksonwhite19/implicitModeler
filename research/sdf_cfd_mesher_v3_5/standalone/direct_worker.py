"""Local transport for the existing V4 fixed-boundary Delaunay/Netgen recipe.

No historical campaign authorization or accepted mesh is imported. The V3.5
supervisor owns this process and holds its complete local source closure.
"""
from pathlib import Path
import json,sys,time,traceback,shutil
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
from direct_compat import install
install()
sys.path[:0]=[str(ROOT/'v4/runtime/lib/finite_native_packages'),str(ROOT/'v4/tools'),str(ROOT/'v4/src'),str(ROOT/'v4/runtime/lib/gmsh')]
import owned_runtime
owned_runtime.activate()
import numpy as np
from sdf_cfd_mesher_v4.common.authority import file_hash
from finite_research.direct_evidence_modules.variant import write as save


def execute(folder,expected):
    request_path=folder/'INPUT.json'
    if file_hash(request_path)!=expected:raise ValueError('ChangedDirectWorkerRequest')
    request=json.loads(request_path.read_text())
    for name,pin in request['files'].items():
        if file_hash(folder/name)!=pin:raise ValueError('ChangedDirectWorkerInput',name)
    owned_runtime.verify()
    import core_backend as core
    import gmsh
    from finite_research import direct_recipe155 as recipe
    from finite_research.direct_evidence_modules import variant
    from finite_research.direct_contract import validate_policy
    policy=validate_policy(request['policy'])
    locks=core.locks()
    if locks!=request['locks']:raise ValueError('ChangedDirectWorkerDependencies')
    from direct_runtime_identity import RuntimeIdentity
    runtime=RuntimeIdentity(request['source_files'])
    save(folder/'RUNTIME_PRE.json',runtime.verify())
    with np.load(folder/'CAP.npz',allow_pickle=False) as data:cap={k:data[k] for k in data.files}
    mode=request['mode'];profile=None;original_optimize=gmsh.model.mesh.optimize
    result=dict(passed=False,request_sha256=expected,mode=mode,locks=locks)
    started=time.perf_counter()
    try:
        if mode=='probe':
            result.update(generation_calls=0,optimizer_calls=0,meshing_invoked=False)
        elif mode=='farfield':
            from generic_core_sizing import add_analytic_box,GmshSizingAdapter
            from optimized_core_sizing import ExactCachedSizingField
            from finite_research.direct_evidence_modules.farfield_callback import CheckedCallback
            field=ExactCachedSizingField(cap['points'][cap['triangles']],cap['seeds'],g=policy['growth_per_m'],h_max=policy['maximum_size_m'])
            callback=CheckedCallback(core.MeasuredCallback(field))
            gmsh.initialize([],False);core.setup(gmsh);gmsh.model.add('v35_v4_farfield')
            add_analytic_box(gmsh,np.asarray(policy['bounds_m']));gmsh.model.occ.synchronize()
            GmshSizingAdapter(field).install(gmsh);gmsh.model.mesh.setSizeCallback(callback)
            gmsh.model.mesh.generate(2)
            callback_record=callback.require();points,faces=core.surface_arrays(gmsh)
            centre=np.asarray(policy['bounds_m']).mean(axis=1);triangles=points[faces]
            flip=np.einsum('ij,ij->i',np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),triangles.mean(1)-centre)<0
            faces[flip]=faces[flip][:,[0,2,1]]
            from standalone_background import check_box
            check_box(points,faces,np.asarray(policy['bounds_m']))
            np.savez(folder/'OUTER.npz',points=points,triangles=faces)
            gmsh.model.mesh.removeSizeCallback();gmsh.finalize()
            result.update(generation_calls=1,dimension=2,callback=callback_record)
        elif mode=='candidate':
            with np.load(folder/'OUTER.npz',allow_pickle=False) as data:outer={k:data[k] for k in data.files}
            generated=folder/'generated';generated.mkdir()
            shutil.copyfile(folder/'OUTER.npz',generated/'OUTER.npz')
            calls=[]
            profile=recipe.Profile(gmsh,cap,policy['growth_per_m'],policy['maximum_size_m'],lambda r:save(folder/'GENERATION_ENTERED.json',r))
            # Preserve the qualified two-session callback lifecycle, including
            # the exact farfield reuse session with no 2D generation.
            gmsh.initialize([],False);profile.setup(gmsh);gmsh.model.add('retained_farfield_session')
            gmsh.model.addDiscreteEntity(2,2)
            gmsh.model.mesh.addNodes(2,2,np.arange(1,len(outer['points'])+1),outer['points'].ravel())
            gmsh.model.mesh.addElementsByType(2,2,np.arange(1,len(outer['triangles'])+1),(outer['triangles']+1).ravel())
            profile.install()
            recipe.surface_check(gmsh,dict(points=np.empty((0,3),dtype='<f8'),triangles=np.empty((0,3),dtype='<i8')),outer)
            profile.finalize()
            gmsh.initialize([],False);profile.setup(gmsh);recipe.import_boundary(gmsh,cap,outer);profile.install()
            before=recipe.boundary_snapshot(gmsh);save(folder/'BOUNDARY_BEFORE.json',before)
            if not recipe.boundary_difference(before,before)['passed']:raise ValueError('InvalidDirectBoundarySnapshot')
            callback_before=profile.callback.report();algorithm_before=gmsh.option.getNumber('Mesh.Algorithm3D')
            gmsh.logger.start()
            generation_error=None
            try:profile.generate()
            except BaseException as error:generation_error=error
            finally:
                messages=gmsh.logger.get();gmsh.logger.stop()
                trace=dict(messages=messages,callback_before=callback_before,callback_after=profile.callback.report(),algorithm_before=algorithm_before,algorithm_after=gmsh.option.getNumber('Mesh.Algorithm3D'))
                save(folder/'ALGORITHM_TRACE.json',trace)
            captures={}
            def capture_raw():
                points,tets=core.arrays(gmsh);captures.update(points=points,tets=tets)
                np.savez(generated/'CORE.npz',points=points,tets=tets)
            def capture_boundary():
                after=recipe.boundary_snapshot(gmsh);difference=recipe.boundary_difference(before,after)
                captures['difference']=difference;save(folder/'BOUNDARY_AFTER.json',after);save(folder/'BOUNDARY_DIFFERENCE.json',difference)
            captured=recipe.terminal_checks([('raw',capture_raw),('boundary',capture_boundary)])
            save(folder/'GENERATION_CAPTURE.json',dict(captured,accepted_at_capture=False,error=None if generation_error is None else repr(generation_error)))
            if generation_error is not None:raise generation_error
            if not captured['passed']:raise ValueError('DirectGenerationCaptureRefusal')
            points,tets,difference=captures['points'],captures['tets'],captures['difference']
            execution=recipe.execution_evidence(trace);save(folder/'ALGORITHM_EXECUTION.json',execution)
            if not execution['passed'] or not difference['passed']:raise ValueError('DirectGenerationBoundaryOrAlgorithmRefusal')
            shape_policy=dict(tet_transition=dict(quality_targets=policy['quality_targets']))
            stages=dict(CORE=recipe.stage_shape(core,points,tets,cap,outer,shape_policy))
            profile.finalize()
            namespace=dict(Path=Path,np=np,time=time,setup=core.setup,arrays=core.arrays,file_hash=core.file_hash,durable_json=core.durable_json,boundary_audit=core.boundary_audit,exact_reload=variant.exact_reload)
            optimize=variant.optimizer_function(ROOT/'v4/tools/core_backend.py',namespace)
            def optimizer_boundary(name):
                p,t=core.arrays(gmsh)
                save(folder/('BOUNDARY_AFTER_'+name.upper()+'.json'),recipe.stage_shape(core,p,t,cap,outer,shape_policy))
            gmsh.model.mesh.optimize=variant.guarded_optimizer(gmsh,original_optimize,calls,on_complete=optimizer_boundary)
            optimize(generated,cap['points'],cap['triangles'],execution=SimpleNamespace(setup=profile.setup))
            if len(calls)!=2 or not all(r['completed'] for r in calls):raise ValueError('DirectOptimizerInvocationMismatch')
            for name in ('GMSH','NETGEN'):
                with np.load(generated/(name+'.npz'),allow_pickle=False) as data:stages[name]=recipe.stage_shape(core,data['points'],data['tets'],cap,outer,shape_policy)
            save(generated/'STAGE_SHAPE.json',stages)
            method=profile.finish(True);save(generated/'METHOD.json',method)
            result.update(generation_calls=1,optimizer_calls=2,calls=calls,method=method)
        else:raise ValueError('UnknownDirectWorkerMode')
        for name,pin in request['files'].items():
            if file_hash(folder/name)!=pin:raise ValueError('ChangedDirectWorkerInput',name)
        if file_hash(request_path)!=expected or core.locks()!=locks:raise ValueError('ChangedDirectWorkerClosure')
        result['passed']=True
    except BaseException as error:
        result.update(error=repr(error),traceback=traceback.format_exc())
    finally:
        gmsh.model.mesh.optimize=original_optimize
        try:
            if profile is not None:profile.close()
            elif gmsh.isInitialized():
                gmsh.model.mesh.removeSizeCallback();gmsh.finalize()
        except BaseException as error:result.update(passed=False,cleanup_error=repr(error))
        try:save(folder/'RUNTIME_POST.json',runtime.verify())
        except BaseException as error:result.update(passed=False,runtime_exit_error=repr(error))
        result.update(seconds=time.perf_counter()-started)
        result['files']={p.relative_to(folder).as_posix():file_hash(p) for p in folder.rglob('*') if p.is_file() and p.suffix in ('.npz','.msh','.json') and p.name not in ('RESULT.json',)}
        save(folder/'RESULT.json',result)
    return 0 if result['passed'] else 1


if __name__=='__main__':
    if not __debug__ or sys.flags.optimize:raise RuntimeError('AssertionsRequired')
    raise SystemExit(execute(Path(sys.argv[1]).resolve(),sys.argv[2]))
