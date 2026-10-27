"""Private single-session finite cavity; existing Gmsh/native/optimizer owners."""
from pathlib import Path
import sys, json, time, traceback, hashlib
HERE=Path(__file__).resolve().parent
V4=HERE.parents[2]
sys.dont_write_bytecode=True
sys.path[:0]=[str(V4/'tools'),str(V4/'src'),str(V4/'runtime/lib/gmsh')]
# Selected diagnostic bootstrap changes no scientific verifier or import order.
if __name__=='__main__' and len(sys.argv)>2:
    if len(sys.argv)!=4 or sys.argv[2]!='--stage-observations-request' or len(sys.argv[3])!=64 or any(c not in '0123456789abcdef' for c in sys.argv[3]):
        raise ValueError('InvalidCollarStageObservationArguments')
    bootstrap=Path(sys.argv[1])/'REQUEST.json'
    bootstrap_bytes=bootstrap.read_bytes()
    if hashlib.sha256(bootstrap_bytes).hexdigest()!=sys.argv[3]:raise ValueError('ChangedCollarStageObservationRequest')
    import stage_observations as _stage_obs
    if Path(_stage_obs.__file__).resolve()!=V4/'tools/stage_observations.py':raise ValueError('WrongStageObservationOrigin')
    _stage_obs.activate_child(json.loads(bootstrap_bytes)['stage_observations'],Path(sys.argv[1])/'STAGE_TIMING.json','finite_backend',sys.argv[3],finite=True)
import native_runtime
native_runtime.verify()
import numpy as np
import gmsh
import core_backend as core
from background_backend_method import DirectExecution,HXT,selection_record
from standalone_background import compare_boundary
from sdf_cfd_mesher_v4.common.authority import array_hash


def validate_receipt(report):
    c=report['callback']
    sessions=[dict(dimension=3,before=1.,factor=.95,passed=True)]
    options={'Mesh.Algorithm3D':10,'General.NumThreads':4,'Mesh.MaxNumThreads1D':1,
        'Mesh.MaxNumThreads2D':1,'Mesh.MaxNumThreads3D':4,'Mesh.OptimizeThreshold':.3,
        'Mesh.Reproducible':1,'Mesh.RandomSeed':1}
    if (report.get('schema')!='PrivateFiniteCollarHXT/1' or report.get('closed') is not True
            or report.get('generation_sessions')!=sessions or report.get('settings')!=[options]*3
            or selection_record(HXT)!=report.get('selection')
            or c.get('registrations')!=1 or c.get('removals')!=1
            or c.get('quarantined') is not False or c.get('python_failure') is not False
            or c.get('native_failure')!=0 or c.get('cache_capacity')!=0
            or c.get('exact_coordinate_cache_hits')!=0
            or type(c.get('evaluations')) is not int or c['evaluations']<=0
            or c['evaluations']!=c.get('native_calls',-1)+c.get('fallback_calls',-1)):
        raise ValueError('IncompletePrivateCollarLifecycle')


def receipt(execution):
    execution.callback.require_passed()
    if execution.closed or execution.callback.registered:raise ValueError('ChangedCollarLifecycle')
    c=execution.callback.report()
    result=dict(schema='PrivateFiniteCollarHXT/1',passed=True,selection=execution.selection,
        generation_sessions=execution.sessions,settings=execution.settings,callback=c)
    execution.close()
    result['closed']=execution.closed
    validate_receipt(result)
    return result


def sizing_options(field):
    # Same GmshSizingAdapter options, without its additional Python callback.
    values={'Mesh.MeshSizeFromPoints':0,'Mesh.MeshSizeFromCurvature':0,
        'Mesh.MeshSizeExtendFromBoundary':0,'Mesh.MeshSizeFactor':1,
        'Mesh.MeshSizeMin':0,'Mesh.MeshSizeMax':field.h_max}
    for k,v in values.items():
        gmsh.option.setNumber(k,v)
        if gmsh.option.getNumber(k)!=v:raise ValueError('CollarSizingReadback',k)
    return values


def stage_audit(p,t,x,tri,outer,folder,name):
    # Save rejected geometry before its acceptance guard can raise.
    np.savez(folder/(name+'_OBSERVED.npz'),points=p,tets=t)
    a,inner,far=compare_boundary(p,t,x,tri,outer,tri)
    from capsule_sizing_control import boundary_audit
    _,owners,_=boundary_audit(p,t,x,tri)
    _,outer_owners,_=boundary_audit(p,t,outer,tri[:,::-1])
    record=dict(passed=True,stage=name,points_sha256=array_hash(p),tets_sha256=array_hash(t),
        inner_ids=inner.tolist(),outer_ids=far.tolist(),inner_owners=owners.tolist(),
        outer_owners=outer_owners.tolist(),audit=a)
    core.durable_json(folder/(name+'_BOUNDARY.json'),record)
    return record


import stage_observations as obs

@obs.observed('finite_backend.run','backend_operation')
def run(folder):
    folder=Path(folder);started=time.perf_counter();times={};execution=None
    mark=started
    def measured(name):
        nonlocal mark
        now=time.perf_counter();times[name]=now-mark;mark=now
    request=json.loads((folder/'REQUEST.json').read_text())
    result=dict(passed=False,stage='input',schema='PrivateFiniteCollarBackend/1')
    from finite_support.flexible_native_contract import version
    from finite_support import retained_native_contract as retained
    diagnostic=request.get('schema')==retained.REQUEST
    if diagnostic:retained.validate_request(request)
    flexible=False if diagnostic else version(request)==2
    request_sha256=core.file_hash(folder/'REQUEST.json')
    if flexible:
        from finite_support.flexible_native_contract import validate_request
        from finite_research.flexible_controls import identities
        validate_request(request)
        result.update(schema='PrivateFiniteCollarBackend/2',request_sha256=core.file_hash(folder/'REQUEST.json'),
            input_sha256=request['input_sha256'],geometry_scope='Q0_INITIALIZER_PROVENANCE_ONLY',
            stage_binding_identity=request['stage_binding_identity'],source_release_sha256=request['source_release_sha256'],
            **identities(request['controls']))
    if diagnostic:result.update(retained.receipt_fields(request,request_sha256))
    def checked_sources():
        if (flexible or diagnostic) and core.file_hash(folder/'REQUEST.json')!=request_sha256:
            raise ValueError('ChangedFlexibleNativeRequest')
        if (flexible or diagnostic) and core.file_hash(V4/'runtime/RELEASE.json')!=request['source_release_sha256']:
            raise ValueError('ChangedFlexibleNativeRelease')
        for raw,h in request['source_pins'].items():
            path=Path('/mnt/'+raw[0].lower()+'/'+raw[3:].replace('\\','/')) if raw[1:3]==':\\' else Path(raw)
            if core.file_hash(path)!=h:raise ValueError('ChangedPrivateBackendSource',raw)
        if core.file_hash(folder/'INPUT.npz')!=request['input_sha256']:
            raise ValueError('ChangedPrivateBackendInput')
    try:
        checked_sources();locks=core.locks(HXT)
        measured('initial_source_and_locks_seconds')
        with np.load(folder/'INPUT.npz',allow_pickle=False) as z:
            x,tri,outer,seeds=(z[k] for k in ('cap','faces','outer','seeds'))
            if diagnostic:retained.verify_inputs(request,{name:z[name] for name in z.files},folder,V4)
        if diagnostic:retained.consume(folder,request)
        execution=DirectExecution(gmsh,x[tri],seeds,g=request['growth_per_m'],
            h_max=request['maximum_size_m'],selection=locks['backend_method'])
        measured('input_and_field_construction_seconds')
        samples={}
        for name,points in [('inner',x),('outer',outer)]:
            a=np.array([execution.field.evaluate(p) for p in points])
            samples[name]=dict(minimum=float(a.min()),maximum=float(a.max()),
                quantiles=np.quantile(a,[0,.01,.5,.99,1]).tolist(),count=len(a))
        measured('diagnostic_field_sampling_seconds')
        np.savez(folder/'OUTER.npz',points=outer,triangles=tri)
        result.update(stage='generation',locks=locks,field_samples=samples)
        core.durable_json(folder/'PROGRESS.json',result)
        measured('initial_output_seconds')
        gmsh.initialize([],False);execution.setup(gmsh);gmsh.model.add('private_finite_collar')
        gmsh.model.addDiscreteEntity(2,1)
        gmsh.model.mesh.addNodes(2,1,np.arange(1,len(x)+1),x.ravel())
        gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(tri)+1),(tri+1).ravel())
        gmsh.model.addDiscreteEntity(2,2);offset=len(x)+1
        gmsh.model.mesh.addNodes(2,2,np.arange(offset,offset+len(outer)),outer.ravel())
        gmsh.model.mesh.addElementsByType(2,2,np.arange(len(tri)+1,2*len(tri)+1),(tri+offset).ravel())
        gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True)
        gmsh.model.addDiscreteEntity(3,1,[2,-1]);gmsh.model.mesh.createGeometry([(3,1)])
        result['sizing_options']=sizing_options(execution.field);execution.install(execution.callback)
        measured('discrete_setup_seconds')
        tick=time.perf_counter();execution.generate(3);times['generate_seconds']=time.perf_counter()-tick
        mark=time.perf_counter()
        p,t=core.arrays(gmsh);np.savez(folder/'CORE.npz',points=p,tets=t)
        execution.finalize()
        measured('extraction_serialization_finalize_seconds')
        tick=time.perf_counter();raw=stage_audit(p,t,x,tri,outer,folder,'RAW');times['raw_boundary_seconds']=time.perf_counter()-tick
        mark=time.perf_counter()
        stages=[];original=core.boundary_audit
        def checked(p,t,x,tri):
            name=('GMSH','NETGEN')[len(stages)]
            tick=time.perf_counter();stages.append(stage_audit(p,t,x,tri,outer,folder,name))
            times[name.lower()+'_boundary_seconds']=time.perf_counter()-tick
            return original(p,t,x,tri)
        core.boundary_audit=checked
        try:
            result['stage']='optimization';optimization=core.optimize(folder,x,tri,execution=execution)
        finally:core.boundary_audit=original
        measured('optimization_total_seconds')
        lifecycle=receipt(execution);checked_sources()
        if request.get('control'):
            import copy
            checked=[]
            for kind in ('session','setup','registrations','quarantined','native_failure','closed','selection'):
                bad=copy.deepcopy(lifecycle)
                if kind=='session':bad['generation_sessions'].append(dict(dimension=2,before=1.,factor=1.,passed=True))
                elif kind=='setup':bad['settings'].append(bad['settings'][0])
                elif kind=='closed':bad['closed']=False
                elif kind=='selection':bad['selection']['algorithm3d']=1
                elif kind=='quarantined':bad['callback'][kind]=True
                elif kind=='registrations':bad['callback'][kind]=2
                else:bad['callback'][kind]=1
                try:validate_receipt(bad)
                except ValueError:checked.append(kind)
                else:raise ValueError('MissedLifecycleNegative',kind)
            result['lifecycle_controls']=checked
        native_runtime.verify()
        if core.locks(HXT)!=locks:raise ValueError('ChangedCollarBackendLocks')
        measured('final_source_native_locks_seconds')
        result.update(passed=True,stage='complete',lifecycle=lifecycle,optimization=optimization,
            boundary_stages=[raw,*stages],files={n:core.file_hash(folder/n) for n in
                ('CORE.npz','OUTER.npz','GMSH.npz','NETGEN.npz')})
        loaded={k:str(Path(m.__file__).resolve()) for k,m in list(sys.modules.items())
            if getattr(m,'__file__',None) and Path(m.__file__).suffix=='.py' and V4 in Path(m.__file__).resolve().parents}
        pinned={str(Path('/mnt/'+k[0].lower()+'/'+k[3:].replace('\\','/')).resolve())
            if k[1:3]==':\\' else str(Path(k).resolve()) for k in request['source_pins']}
        missing=sorted(set(loaded.values())-pinned)
        result['loaded_source_closure']=dict(modules=loaded,unpinned=missing)
        if missing:raise ValueError('UnpinnedNativeCollarSource',missing)
        measured('output_hashes_and_loaded_source_closure_seconds')
    except BaseException as error:
        result.update(passed=False,error=repr(error),traceback=traceback.format_exc())
    finally:
        cleanup_errors=[]
        try:
            if gmsh.isInitialized():
                if execution is not None:execution.finalize()
                else:gmsh.finalize()
        except BaseException as error:cleanup_errors.append(repr(error))
        finally:
            try:
                if execution is not None:
                    execution.close()
                    result['cleanup_closed']=execution.closed
            except BaseException as error:cleanup_errors.append(repr(error))
        if cleanup_errors:result.update(passed=False,cleanup_errors=cleanup_errors)
        measured('cleanup_seconds')
        result.update(backend_seconds=time.perf_counter()-started,exclusive_times=times,
            timing_nesting={'gmsh_boundary_seconds':'optimization_total_seconds','netgen_boundary_seconds':'optimization_total_seconds',
                'optimization.seconds':'optimization_total_seconds'},
            timing_scope='run() only; initial native-runtime imports/checks and final BACKEND.json write are outside this total; boundary timers are nested in optimization.')
        core.durable_json(folder/'BACKEND.json',result)
    return 0 if result['passed'] else 1

if __name__=='__main__':
    _stage_code=None
    try:_stage_code=run(sys.argv[1])
    finally:
        if '_stage_obs' in globals():_stage_obs.finish('returned:'+str(_stage_code))
    raise SystemExit(_stage_code)
