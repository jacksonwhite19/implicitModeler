"""Bounded retained-front enrollment/dispatch checks. Never runs stage executors."""
import argparse, json, sys, time, shutil, traceback
from pathlib import Path
from unittest.mock import patch
import numpy as np
import psutil
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sealed_front_connection import (read,load_case,inspect_front,enroll_native,build_access_indices,
    indexed_session,bind_forecast,ScalarOffsetGeometry,IncidentNormalFirstLayerGeometry,install_qualified_stack)
from path_preserving_deduplication import file_hash,durable_json
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from composable_state import CommittedState
OUT=V4/'results/sealed_front_connection'


def save(name,value):
    durable_json(OUT/name,value)


def expect_error(operation,message):
    try:operation()
    except (ValueError,KeyError) as error:
        if message not in str(error):raise
        return str(error)
    raise AssertionError('ExpectedRejection:'+message)


def preflight():
    if OUT.exists():raise ValueError('PreserveExistingStudy')
    wing=V4/'results/live_sf2_first_layer';a=read(wing/'AUTHORITY.json')
    if identity(a['scientific'])!=a['first_layer_identity'] or not a['first_layer_qualified']:
        raise ValueError('WingQualificationIdentityMismatch')
    if a['first_layer_identity']!='e8cdfcad633d1bb4ab0af8ae680319348581ba8c675d4b73e812247ddfdbe6ec':raise ValueError('WrongSealedWing')
    documents=[wing/n for n in ('AUTHORITY.json','IMPLEMENTATION.json','REPLAY.json','REQUEST.json','A/RESULT.json','A/COVERAGE_RECHECK.json')]
    if file_hash(wing/'IMPLEMENTATION.json')!=a['implementation_sha256'] or file_hash(wing/'REPLAY.json')!=a['replay_sha256']:
        raise ValueError('WingEvidenceMismatch')
    for p,h in read(wing/'IMPLEMENTATION.json').items():
        if file_hash(ROOT/p)!=h:raise ValueError('WingSourceMismatch:'+p)
    capsule_surface=V4/'results/production_adaptive_capsule_surface_foundation/fresh'
    bootstrap=V4/'results/persistent_publication_optimization'
    parent=read(bootstrap/'PARENT_INCIDENCE_AUTHORITY.json')
    capsule_request=V4/'results/qualification_prism_progression_convention/REQUEST.json'
    capsule_authority=read(capsule_surface/'SURFACE_AUTHORITY.json')
    wing_surface=V4/'results/live_sf2_surface/replay_20260929/authority'
    cases={
        'wing':dict(units='m',surface=str(wing_surface),request=str(wing/'REQUEST.json'),
            request_sha256=a['request_sha256'],checkpoint=str(wing/'A/checkpoints'),front_qualification=str(wing/'AUTHORITY.json'),
            committed=a['committed_front_identity'],geometry_identity=a['geometry'],surface_identity=a['scientific']['surface'],
            layer=1,front_coordinates=a['scientific']['front_coordinate_hash'],
            sources={str(p.resolve()):file_hash(p) for p in documents}),
        'capsule':dict(units='m',surface=str(capsule_surface),request=str(capsule_request),request_sha256=file_hash(capsule_request),
            reference=parent['source'],geometry_identity=capsule_authority['geometry_identity'],surface_identity=capsule_authority['surface_identity'],
            sources={str(p.resolve()):file_hash(p) for p in (capsule_surface/'SURFACE_AUTHORITY.json',capsule_surface.parent/'GEOMETRY.json',capsule_request)},
            geometry_definition=str(capsule_surface.parent/'GEOMETRY.json'),spec=dict(components=[read(capsule_surface.parent/'GEOMETRY.json')['definition']]))}
    for case in cases.values():
        wall,tri,surface,request=load_case(case)
        paths=[Path(case['request']),Path(case['surface'])/'SURFACE_AUTHORITY.json']
        paths += [Path(case['surface'])/n for n in surface['files']]
        paths += [(Path(case['request']).parent/r['path']).resolve() for r in request['declaration']['source_documents']]
        case['sources'].update({str(p.resolve()):file_hash(p) for p in paths})
    from rolling_layer_retention import checkpoint_closure
    closure,envelope=checkpoint_closure(Path(cases['wing']['checkpoint']))
    # Metadata only for storage estimation. No scientific payload duplication.
    payload_bytes=sum(Path(p).stat().st_size for p in closure)
    counts={k:envelope['components'][k].get('count') for k in ('vertices','cells','faces','certificate','quality')}
    oldhot=V4/'results/clean_sheet_incremental_l4_to_l5/L4_access/objects'
    measured_hot=sum(p.stat().st_size for p in oldhot.iterdir() if p.is_file())
    n=53153;cells=106302;faces=372057;pairs=envelope['components']['certificate']['count']
    # Known array encodings: pair postings 2*count*u32; keys/maps/offsets per face;
    # numeric keys/owners/winding 112 B/face, cells 72 B/cell. Existing 2x reserve.
    estimate=2*pairs*4+faces*(64+8+8+112)+cells*72+measured_hot*(2*n)/(5*16895)
    budget=int(np.ceil(2*estimate))
    if shutil.disk_usage(ROOT).free<budget:raise ValueError('OperationalFreeReserve')
    OUT.mkdir()
    save('CASES.json',cases)
    save('PREFLIGHT.json',dict(source_checkpoint_files=len(closure),source_checkpoint_bytes=payload_bytes,
        native_counts=counts,original_closure=closure,wing_authority=a['first_layer_identity'],
        measured_comparable_hot_bytes=measured_hot,estimated_index_and_hot_bytes=estimate,output_limit_bytes=budget,
        minimum_free_bytes=budget,free_bytes=shutil.disk_usage(ROOT).free,proof_payload_copy=False,
        controls_scope='Operational 2x reserve over known index encodings plus measured hot metadata; no scientific threshold',
        no_layer_advance=True))


def capsule():
    from geometry_family_authority import Source,sdf_record
    from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
    from production_forecast_admission import BoundProductionForecastCache
    cases=read(OUT/'CASES.json');case=cases['capsule']
    case['geometry_definition']=str(Path(case['surface']).parent/'GEOMETRY.json')
    boot=V4/'results/persistent_publication_optimization'
    p=read(boot/'PARENT_INCIDENCE_AUTHORITY.json');v=read(boot/'PARENT_VERSION_AUDIT.json');l=read(boot/'PARENT_LOOKUP_AUTHORITY.json')
    pair=read(V4/'results/prism_assembly_optimization/INCIDENCE_ACCESS.json')
    indices=read(OUT/'CAPSULE_INDICES.json') if (OUT/'CAPSULE_INDICES.json').exists() else dict(directory=str(boot/'parent_incidence'),root=p['root'],version_path=str(boot/'PARENT_VERSION_CLOSURE.json'),
        version_root=v['root'],lookup=l['reference'],pair_path=pair['path'],pair_root=pair['root'])
    if (OUT/'CAPSULE_REFERENCE.json').exists():case['reference']=read(OUT/'CAPSULE_REFERENCE.json')
    with CommittedState(case['reference']) as base:
        case.update(committed=base.expected['committed'],layer=base.layer,front_coordinates=base.expected['front_coordinates'])
        wall,tri,surface,request=load_case(case)
        direct=base.initialize(wall,tri,surface,case['request'],Source(case['spec']))
        owner=ScalarOffsetGeometry(case['spec'],case['geometry_identity']);bound=bind_forecast(base,case,owner)
        if direct.binding!=bound.binding:raise ValueError('CapsuleBindingChanged')
        c1=BoundProductionForecastCache(direct);c2=BoundProductionForecastCache(bound)
        if c1.scientific_identity()!=c2.scientific_identity():raise ValueError('CapsuleForecastKeyChanged')
        # Existing retained layer coordinates; target is the SAME retained level,
        # never the next level. These are geometry calls, not a layer forecast.
        ids=np.unique(np.linspace(0,len(wall)-1,64,dtype=int));points=bound.front.coordinates[ids]
        target=bound.schedule.targets[base.layer-1]
        samples1=direct.sdf.evaluate(points);samples2=bound.sdf.evaluate(points)
        query_equal={k:np.asarray(v).tobytes()==np.asarray(vars(samples2)[k]).tobytes() for k,v in vars(samples1).items()}
        direct_prediction=predict_next_front(points,target,Source(case['spec']),iteration_cap=256)
        prediction=owner.predict(points,target,iteration_cap=256)
        prediction_equal={k:np.asarray(v).tobytes()==np.asarray(vars(prediction)[k]).tobytes() for k,v in vars(direct_prediction).items() if k!='projection_seconds'}
        if not all(query_equal.values()) or not all(prediction_equal.values()):raise ValueError('CapsuleOperationMismatch')
        record=sdf_record(case['spec'],points,target)
        if record!=owner.validate_target(points,target):raise ValueError('CapsuleTargetCheckChanged')
        inspected=inspect_front(base,wall,tri,surface,request,base.expected)
        with indexed_session(base,inspected,indices) as (numeric,pairs):
            face=base['prism_front']['0'];source_face=base['faces'][face]
            affected=pairs.affected({'face:'+face:'unchanged_probe'})
            index_result=dict(faces=len(numeric),source_face=source_face,pairs=affected)
        base.check()
        save('CAPSULE.json',dict(reference=base.reference,content=base.content_root,binding_equal=True,cache_identity_equal=True,
            samples=len(ids),query_fields=query_equal,prediction_fields=prediction_equal,target_check_equal=True,
            target_scope='Existing L10 target only, no L11 prediction',index_access=index_result,indices=indices,
            no_forecast=True,no_generation=True))
    cases['capsule']=case;save('CASES.json',cases)


def capsule_recover():
    """Existing explicit deep recovery; private metadata, unchanged science."""
    import recursive_epoch_recovery as recovery
    from contextlib import ExitStack
    import immutable_state_access as access
    prior=V4/'results/prism_assembly_optimization/epoch_access'
    pointer=read(prior/'CURRENT.json');record=read(prior/pointer['file'])
    if file_hash(prior/pointer['file'])!=pointer['sha256'] or identity(record)!=pointer['root']:
        raise ValueError('RecoveryReferenceCorrupt')
    result=recovery.recover(record['source'],record['content'],record['scientific'],OUT/'capsule_epoch',progress=lambda x:print(x,flush=True))
    original=read(OUT/'CASES.json')['capsule']['reference']
    # Match by content, not source path: retained controls had an earlier epoch.
    boot=V4/'results/persistent_publication_optimization';p=read(boot/'PARENT_INCIDENCE_AUTHORITY.json')
    ref=next(n['reference'] for n in result['nodes'] if n['content']==p['parent_content'])
    save('CAPSULE_REFERENCE.json',ref)
    destination=OUT/'capsule_indices';destination.mkdir(exist_ok=False)
    with ExitStack() as held,CommittedState(ref) as base:
        def renew(path,expected):
            held.enter_context(access.guard(path));body=read(path)
            if identity(body)!=expected:raise ValueError('WrongSealedIndexRoot')
            if body['parent_content']!=base.content_root:raise ValueError('WrongSealedIndexParent')
            old={r['path']:{'sha256':r['sha256'],'token':r['token']} for r in body['files'].values()}
            fresh=recovery.refresh_records(old,held,{})
            body['files']={name:dict(path=r['path'],**fresh[r['path']]) for name,r in body['files'].items()}
            body['epoch']=access.epoch()
            return body
        numeric=renew(boot/'parent_incidence/MANIFEST.json',p['root'])
        nd=destination/'numeric';nd.mkdir();durable_json(nd/'MANIFEST.json',numeric);nr=identity(numeric)
        from persistent_numeric_incidence import PackedSegment
        from prism_source_lookup import write_lookup
        # Reuse lookup writer and its exact key rules rather than rewriting tokens.
        with PackedSegment(nd,nr,base.content_root) as packed:
            lookup=write_lookup(destination/'lookup',packed.ordered_items(),16895,nr,base.content_root)
        from audit_parent_version_closure import bind_for_state
        vp=destination/'VERSION.json';vr=bind_for_state(base,nr,vp)
        old=read(V4/'results/prism_assembly_optimization/INCIDENCE_ACCESS.json')
        path=Path(old['path']);held.enter_context(access.guard(path));body=read(path)
        if identity(body)!=old['root'] or body['parent_content']!=base.content_root:raise ValueError('WrongSealedPairIndex')
        body=dict(body,files=recovery.refresh_records(body['files'],held,{}),epoch=access.epoch())
        pp=destination/'PAIRS.json';durable_json(pp,body)
        indices=dict(directory=str(nd),root=nr,version_path=str(vp),version_root=vr,lookup=lookup,pair_path=str(pp),pair_root=identity(body))
        from prism_persistent_hotpath import bind_bootstrap
        from incremental_pair_storage import PairIncidence
        bind_bootstrap(base,nd,nr,vp,vr,lookup)
        with PairIncidence(base,pp,indices['pair_root']):pass
        save('CAPSULE_INDICES.json',indices)


def wing_enroll():
    cases=read(OUT/'CASES.json');case=cases['wing']
    case['front_qualification']=str(Path(case['checkpoint']).parents[1]/'AUTHORITY.json')
    save('CASES.json',cases)
    ref=enroll_native(case,OUT/'wing_access')
    save('WING_REFERENCE.json',ref)


def wing_indices():
    case=read(OUT/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
    with CommittedState(read(OUT/'WING_REFERENCE.json')) as base:
        inspected=inspect_front(base,wall,tri,surface,request,base.expected)
        indices=build_access_indices(base,inspected,OUT/'wing_indices')
        save('WING_INDICES.json',indices)


def wing_open(report='WING.json'):
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from production_forecast_admission import BoundProductionForecastCache
    case=read(OUT/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
    owner=IncidentNormalFirstLayerGeometry(case['geometry_identity']);results=[]
    def denied(*args,**kwargs):raise AssertionError('NoRestartForecastOrGenerationDuringGuardedOpen')
    for attempt in range(2):
        with patch.object(NativeCheckpointStore,'restart',denied),patch.object(BoundProductionForecastCache,'forecast',denied),CommittedState(read(OUT/'WING_REFERENCE.json')) as base:
            inspected=inspect_front(base,wall,tri,surface,request,base.expected)
            if len(inspected.remaining_increments)!=0:raise ValueError('InventedRemainingSchedule')
            with indexed_session(base,inspected,read(OUT/'WING_INDICES.json')) as (numeric,pairs):
                face=base['prism_front']['0'];source_face=base['faces'][face]
                result=dict(scientific=base.scientific,content=base.content_root,faces=len(numeric),
                    source_face=source_face,affected=pairs.affected({'face:'+face:'unchanged_probe'}),
                    remaining_increments=0,forecast=expect_error(lambda:bind_forecast(base,case,owner),'NoPendingIncrement'),
                    continuation=expect_error(lambda:owner.predict(None,None),'IncidentNormalContinuationNotQualified'))
            base.check();results.append(result)
    if results[0]!=results[1]:raise ValueError('GuardedReopenDifference')
    save(report,dict(opens=results,reopen_equal=True,native_restart_during_reopen=False,no_layer_advance=True))


def wing_open_optimized():
    from sealed_front_connection import qualified_dispatch
    with qualified_dispatch():
        wing_open('WING_OPTIMIZED.json')


def wing_operations():
    """Existing live branch normals and compatibility; no front construction."""
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy,verify_kernel_sources
    from live_spline_first_layer import existing_compatibility
    case=read(OUT/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
    source,_=verify_kernel_sources()
    with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz') as data:
        refs=data['face_refs'];corners=np.flatnonzero(data['corner_refs']);edges=data['curve_edges'];directions=data['direction']
    ids=np.unique(np.r_[np.linspace(0,len(wall)-1,64,dtype=int),corners,edges[:8,:2].ravel().astype(int)])
    incident={int(v):set() for v in ids}
    for f,r in zip(tri,refs):
        for v in f:
            if int(v) in incident:incident[int(v)].add(int(r))
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
        obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
        binding=bind_sf2_policy(bind_canonical(obj));a=binding.authority
        if obj.identity!=case['geometry_identity']:raise ValueError('LiveGeometryIdentityMismatch')
        owner=IncidentNormalFirstLayerGeometry(obj.identity);direct=existing_compatibility();rows=[]
        for v in ids:
            normals=np.array([a.exact_normal(r,wall[v]) for r in sorted(incident[int(v)])])
            left=direct(normals);right=owner.compatibility(normals)
            if identity(left)!=identity(right):raise ValueError('CompatibilityOperationMismatch')
            vector=np.asarray(right['displacement_per_height'])
            if vector.tobytes()!=directions[v].tobytes():raise ValueError('RetainedFeatureDirectionMismatch')
            rows.append(dict(vertex=int(v),sheets=sorted(incident[int(v)]),rank=right['rank'],residual=right['max_residual'],norm=float(np.linalg.norm(vector))))
        save('WING_OPERATIONS.json',dict(geometry=obj.identity,samples=len(ids),source=source,all_direct_equal=True,
            all_retained_directions_bit_equal=True,rows=rows,normalization=False,no_front_generated=True,
            later_vectors='Not declared for this sealed case; V3 AcceptedFrontState.positions exists but is not silently populated.'))


def dispatch():
    from sealed_front_connection import qualified_dispatch
    import composable_publication,critical_transaction,critical_prism_transition
    import persistent_prism_transition
    with qualified_dispatch() as executors:
        if composable_publication.Store is not critical_transaction.Store:raise ValueError('WrongStore')
        if persistent_prism_transition.produce is not critical_prism_transition.produce:raise ValueError('WrongCriticalTransition')
        import inspect
        if inspect.getclosurevars(executors['generation']).nonlocals['produce'] is not critical_prism_transition.produce:
            raise ValueError('GenerationWrapperBypassesCriticalImplementation')
        save('DISPATCH.json',dict(executors={k:v.__module__+'.'+v.__qualname__ for k,v in executors.items()},
            generation_closure_is_critical_produce=True,critical_transition_bound=True,critical_store_bound=True,called=[],no_layer_advance=True))


def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['preflight','capsule','capsule_recover','wing_enroll','wing_indices','wing_open','wing_open_optimized','wing_operations','dispatch'])
    parser.add_argument('--output',type=Path,default=OUT)
    args=parser.parse_args();OUT=args.output.resolve()
    start=time.perf_counter();before=shutil.disk_usage(ROOT).free
    try:
        if args.mode!='preflight':
            controls=read(OUT/'PREFLIGHT.json')
            if before<controls['minimum_free_bytes']:raise ValueError('OperationalFreeReserve')
        if args.mode=='capsule':
            from sealed_front_connection import qualified_dispatch
            with qualified_dispatch():capsule()
        else:globals()[args.mode]()
        controls=read(OUT/'PREFLIGHT.json')
        retained=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
        if retained>controls['output_limit_bytes'] or shutil.disk_usage(ROOT).free<controls['minimum_free_bytes']:
            raise ValueError('OperationalStorageLimit')
    except BaseException as error:
        if OUT.exists():save(args.mode+'_FAILURE.json',dict(error=type(error).__name__+':'+str(error),traceback=traceback.format_exc(),no_layer_advance=True))
        raise
    finally:
        if OUT.exists():
            size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
            save(args.mode+'_RESOURCE.json',dict(seconds=time.perf_counter()-start,peak_working_set=psutil.Process().memory_info().peak_wset,
                initial_free=before,final_free=shutil.disk_usage(ROOT).free,retained_logical_bytes=size))
    print(args.mode,'PASSED',flush=True)


if __name__=='__main__':main()
