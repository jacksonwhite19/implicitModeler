"""One retained capsule L10->L11 control through the common production entry.

Configure existing access recovery, indices, executors and comparison only.
No geometry, cell, proof, storage or publication implementation lives here.
"""
import argparse
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import time

V4=Path(__file__).resolve().parents[1]; ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from sealed_front_connection import read
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity

OUT=V4/'results/capsule_shared_production'; RUN=OUT/'run'; MONITOR=OUT/'monitor'
SOURCE_CONTRACT=None;SOURCE_REVIEW=None
SOURCE=V4/'results/sealed_front_connection'
REFERENCE=V4/'results/remaining_prism_critical_path/normal_candidate_clean_prism'


def configure(output=None,source_contract=None,source_review=None):
    global OUT,RUN,MONITOR,SOURCE_CONTRACT,SOURCE_REVIEW
    if (source_contract is None)!=(source_review is None):raise ValueError('IncompleteSourceCompatibilityInputs')
    if output is not None:
        path=Path(output).resolve()
        if not path.is_relative_to(V4/'results') or path==V4/'results':raise ValueError('ControlOutputOutsideResults')
        OUT=path;RUN=path/'run';MONITOR=path/'monitor'
    SOURCE_CONTRACT=Path(source_contract).resolve() if source_contract is not None else None
    SOURCE_REVIEW=Path(source_review).resolve() if source_review is not None else None


def compatibility_options():
    if SOURCE_CONTRACT is None:return None
    return dict(record_path=str(SOURCE_CONTRACT),record_identity=identity(read(SOURCE_CONTRACT)),
        review_path=str(SOURCE_REVIEW),review_sha256=file_hash(SOURCE_REVIEW))


def enter_compatibility(held,options):
    if options is not None:
        from historical_implementation_access import qualified_source_access
        return held.enter_context(qualified_source_access(**options))


def ensure_output_directory():
    # Evidence writers require a parent. Configure already bounds CLI outputs.
    OUT.mkdir(parents=True,exist_ok=True)


def require_monitored_worker():
    """Direct worker CLI must not bypass the existing live resource monitor."""
    import os,psutil
    pid=int(os.environ.get('V4_SHARED_CAPSULE_MONITOR_PID','0'))
    if pid<=0 or os.getppid()!=pid or os.environ.get('V4_SHARED_CAPSULE_MONITOR_OUTPUT')!=str(MONITOR.resolve()):
        raise ValueError('LiveCapsuleResourceMonitorRequired')
    args=psutil.Process(pid).cmdline()
    if 'monitor' not in args or str(Path(__file__).resolve()) not in [str(Path(a).resolve()) for a in args]:
        raise ValueError('WrongCapsuleResourceMonitor')
    if not (MONITOR/'RESOURCE_PREFLIGHT.json').exists():raise ValueError('MissingLiveResourcePreflight')


def ancestry(reference):
    """Read authenticated roots only; legacy full-digest recovery runs later."""
    import recursive_epoch_recovery as recovery
    nodes=[];seen=set()
    with ExitStack() as held:
        while True:
            if reference['root'] in seen:raise ValueError('AncestorCycle')
            seen.add(reference['root'])
            body=recovery.sealed_body(reference,held)
            directory=Path(reference['directory']);pointer=directory/'CURRENT.json'
            root=directory/read(pointer)['file']
            nodes.append(dict(reference=reference,root=str(root),sha256=file_hash(root),epoch=body['epoch']))
            if reference['kind']=='native':
                for p,h in body['content']['implementation'].items():
                    from historical_implementation_access import check_pinned
                    try:check_pinned(p,h,body['content_root'],'IncompatibleRecoveryImplementation')
                    except ValueError as error:raise ValueError(*error.args,p) from error
                return nodes
            reference=body['plan']['parent']


def preflight():
    from run_successor_benchmark import execution_sources,execution_binaries
    from sealed_front_connection import load_case
    from record_receipt_resume import disk
    if (OUT/'REVIEW_REQUEST.json').exists() or RUN.exists():raise ValueError('PreserveCapsuleControl')
    case=read(SOURCE/'CASES.json')['capsule']
    wall,tri,surface,request=load_case(case)
    reference=read(SOURCE/'CAPSULE_REFERENCE.json')
    source_access=compatibility_options()
    with ExitStack() as held:
        enter_compatibility(held,source_access)
        nodes=ancestry(reference)
    parent=read(nodes[0]['root'])
    # The preserved L5 bootstrap has no production acceptance. Do not weaken
    # component recovery to accept it. Reuse the capsule's original full-digest
    # recovery and its already verified L11 scientific pin, then select L10.
    recovery_directory=SOURCE/'capsule_epoch';pointer=read(recovery_directory/'CURRENT.json')
    recovery_path=recovery_directory/pointer['file'];renewal=read(recovery_path)
    from recursive_epoch_recovery import verified_record,RECOVERY
    if (file_hash(recovery_path)!=pointer['sha256'] or identity(renewal)!=pointer['root']
        or renewal.get('schema')!=RECOVERY or not verified_record(renewal)
        or not any(n['reference']==reference and n['content']==parent['plan']['scientific'] for n in renewal['nodes'])):
        raise ValueError('UnboundLegacyCapsuleRecovery')
    expected=read(REFERENCE/'CANDIDATE_PLAN.json')
    if (case['layer']!=10 or parent['plan']['scientific']!=expected['content']['parent_content']
        or case['committed']!=parent['plan']['expected']['committed']
        or identity(expected['content'])!=expected['scientific']
        or expected['scientific']!='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870'
        or surface['surface_identity']!='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'):
        raise ValueError('WrongRetainedCapsuleControl')
    authority_path=REFERENCE.parent/'AUTHORITY.json'; authority=read(authority_path)
    evidence={}
    for name in ('CANDIDATE_PLAN.json','NORMAL.json','GENERATION.json','ASSEMBLY.json','FRESH_CERTIFICATE.json','CANDIDATE_ACCEPTANCE.json'):
        p=REFERENCE/name;want=authority['evidence']['normal_candidate_clean_prism/'+name]
        if file_hash(p)!=want:raise ValueError('ChangedQualifiedCapsuleReference',name)
        evidence[str(p.resolve())]=want
    resources=deepcopy(read(V4/'results/assembly_binding_production/run/START.json')['resources'])
    if resources['output_limit_bytes'] is not None or resources['memory_limit'] is not None:
        raise ValueError('UnexpectedCurrentResourcePolicy')
    if disk()['C:\\']['available']<resources['minimum_free_bytes']:raise ValueError('OperationalResourceLimit')
    paths=[SOURCE/n for n in ('CASES.json','CAPSULE_REFERENCE.json','CAPSULE_INDICES.json')]
    paths += [Path(n['root']) for n in nodes]+[authority_path,recovery_directory/'CURRENT.json',recovery_path]
    indices=read(SOURCE/'CAPSULE_INDICES.json')
    paths += [Path(indices['directory'])/'MANIFEST.json',Path(indices['lookup']['directory'])/'MANIFEST.json',
              Path(indices['pair_path']),Path(indices['version_path'])]
    inputs={str(p.resolve()):file_hash(p) for p in paths}
    inputs.update(case['sources']);inputs.update(evidence)
    if source_access is not None:
        for name in ('record_path','review_path'):
            p=source_access[name];inputs[p]=file_hash(p)
        for row in read(source_access['record_path'])['rows']:
            inputs[row['historical']]=row['historical_sha256']
    control=dict(case=case,reference=reference,parent_content=parent['plan']['scientific'],indices=indices,
        expected_scientific=expected['scientific'],reference_plan=str((REFERENCE/'CANDIDATE_PLAN.json').resolve()),
        resources=resources,scope='One private normal capsule L10->L11 control; no canonical advancement, fresh deep mesh audit or replay campaign',
        ancestry=nodes,recovery=dict(source=renewal['source'],content=renewal['content'],scientific=renewal['scientific']),
        vertices=len(wall),new_cells=len(tri),source_access=source_access)
    ensure_output_directory()
    durable_json(OUT/'CONTROL.json',control)
    RUN.mkdir();durable_json(RUN/'START.json',dict(resources=resources,scope=control['scope']))
    seeds=[Path(__file__),V4/'tools/capsule_reuse_dispatch.py',V4/'tools/run_critical_reuse_control.py']
    q=dict(runtime=execution_sources(seeds),binary=execution_binaries(),inputs=inputs,
           control_sha256=file_hash(OUT/'CONTROL.json'),start_sha256=file_hash(RUN/'START.json'))
    durable_json(OUT/'REVIEW_REQUEST.json',q)
    print('Preflight ready; independent execution review required',len(q['runtime']),flush=True)


def verified(held):
    import immutable_state_access as access
    for name in ('REVIEW_REQUEST.json','REVIEW.json','CONTROL.json'):
        held.enter_context(access.guard(OUT/name))
    q=read(OUT/'REVIEW_REQUEST.json');r=read(OUT/'REVIEW.json')
    if (r.get('passed') is not True or not r.get('reviewer') or r.get('runtime')!=q['runtime']
        or r.get('request_sha256')!=file_hash(OUT/'REVIEW_REQUEST.json')
        or q['control_sha256']!=file_hash(OUT/'CONTROL.json') or q['start_sha256']!=file_hash(RUN/'START.json')):
        raise ValueError('IndependentCapsuleControlReviewRequired')
    held.enter_context(access.guard(RUN/'START.json'))
    for group in ('runtime','binary','inputs'):
        for p,h in q[group].items():
            held.enter_context(access.guard(p))
            if file_hash(p)!=h:raise ValueError('ChangedCapsuleControlDependency',p)
    return read(OUT/'CONTROL.json')


def renew_indices(base,original,vertex_stride):
    """Existing capsule recovery pattern; immutable payloads remain referenced."""
    import immutable_state_access as access
    from recursive_epoch_recovery import refresh_records
    from persistent_numeric_incidence import PackedSegment
    from prism_source_lookup import write_lookup
    from audit_parent_version_closure import bind_for_state
    folder=OUT/'indices';folder.mkdir(exist_ok=False)
    with ExitStack() as held:
        def renew(path,expected):
            held.enter_context(access.guard(path));old=read(path)
            if identity(old)!=expected or old['parent_content']!=base.content_root:
                raise ValueError('WrongSealedIndexParent')
            # Same transformation as check_sealed_front_connection.capsule_recover.
            if isinstance(old['files'],dict) and all('path' in r for r in old['files'].values()):
                refs={r['path']:{'sha256':r['sha256'],'token':r['token']} for r in old['files'].values()}
                fresh=refresh_records(refs,held,{})
                files={name:dict(path=r['path'],**fresh[r['path']]) for name,r in old['files'].items()}
            else:
                files=refresh_records(old['files'],held,{})
            return dict(old,files=files,epoch=access.epoch())
        numeric=renew(Path(original['directory'])/'MANIFEST.json',original['root'])
        nd=folder/'numeric';nd.mkdir();durable_json(nd/'MANIFEST.json',numeric);nr=identity(numeric)
        # Use original lookup construction, not another face/ownership algorithm.
        with PackedSegment(nd,nr,base.content_root) as packed:
            lookup=write_lookup(folder/'lookup',packed.ordered_items(),vertex_stride,nr,base.content_root)
        vp=folder/'VERSION.json';vr=bind_for_state(base,nr,vp)
        pairs=renew(Path(original['pair_path']),original['pair_root']);pp=folder/'PAIRS.json'
        durable_json(pp,pairs)
    indices=dict(directory=str(nd),root=nr,lookup=lookup,version_path=str(vp),version_root=vr,
                 pair_path=str(pp),pair_root=identity(pairs))
    durable_json(OUT/'INDICES.json',indices);return indices


def compare_enrollment_binding(base,case,owner,adapter,source_access):
    """Exact old/current enrollment arithmetic on the real authenticated parent."""
    import importlib.util
    import numpy as np
    from dataclasses import fields
    from production_forecast_admission import BoundProductionForecastCache
    from sealed_front_connection import load_case
    if source_access is None:return dict(scope='No historical source resolution requested')
    record=read(source_access['record_path'])
    row=next(r for r in record['rows'] if r['relation']['role']=='forecast_cache_scope')
    spec=importlib.util.spec_from_file_location('qualified_original_forecast',row['historical'])
    original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
    wall,tri,surface,request=load_case(case)
    old=original.ProductionForecastAdapter.__new__(original.ProductionForecastAdapter)
    old._bind(base,wall,tri,surface,case['request'],owner,base.expected,base.expected['committed'])
    arrays={}
    def equal(name,left,right):
        arrays[name]=left.dtype==right.dtype and left.shape==right.shape and left.tobytes()==right.tobytes()
    for name in ('committed_prefix','remaining_increments'):equal(name,getattr(old,name),getattr(adapter,name))
    for name in ('front','topology','schedule'):
        left=getattr(old,name);right=getattr(adapter,name)
        for item in fields(left):
            a=getattr(left,item.name);b=getattr(right,item.name)
            if isinstance(a,np.ndarray):equal(name+'/'+item.name,a,b)
            elif hasattr(a,'offsets'):
                equal(name+'/'+item.name+'/offsets',a.offsets,b.offsets)
                equal(name+'/'+item.name+'/values',a.values,b.values)
            elif a!=b:raise ValueError('EnrollmentMetadataMismatch',name,item.name)
    passed=(all(arrays.values()) and old.binding==adapter.binding and old.identity==adapter.identity
        and old.sdf.source_identity==adapter.sdf.source_identity
        and original.BoundProductionForecastCache(old).scientific_identity()==BoundProductionForecastCache(adapter).scientific_identity())
    result=dict(passed=passed,vertices=len(adapter.front.coordinates),arrays=arrays,
        default_cache_identity_equal=passed,binding=adapter.identity,
        scope='Historical/current enrollment and default-cache inputs exact; no lookahead executed and no mesh/proof qualification substituted')
    durable_json(RUN/'SOURCE_BINDING_COMPARISON.json',result)
    if not passed:raise ValueError('EnrollmentBindingScientificMismatch')
    return result


def worker():
    import traceback,psutil
    import immutable_state_access as access
    from capsule_reuse_dispatch import qualified_dispatch,PRODUCTION_CONTROLS
    from sealed_front_connection import load_case,ScalarOffsetGeometry,bind_forecast,indexed_session,inspect_front
    from composable_state import CommittedState
    from recursive_epoch_recovery import recover
    from profile_remaining_prism import memory_sample
    from run_critical_reuse_control import compare_plan
    from run_authorized_continuation import close_preparation_jobs
    from unittest.mock import patch
    import composable_publication
    import capsule_user_domain_handoff as h
    start=time.perf_counter();stage='verification';phases={};measurements={}
    def timed(name,call):
        nonlocal stage
        require_monitored_worker()
        stage=name;durable_json(RUN/'STAGE.json',dict(stage=name,seconds=time.perf_counter()-start))
        print(name,flush=True);tick=h.measured()
        with memory_sample(RUN,name):result=call()
        measurements[name]=h.metrics(*tick);phases[name]=measurements[name]['seconds']
        durable_json(RUN/'PHASES.json',phases);durable_json(RUN/'MEASURED.json',measurements)
        return result
    if (RUN/'RESULT.json').exists() or (RUN/'STOP.json').exists():raise ValueError('PreserveCapsuleControlRun')
    try:
        require_monitored_worker()
        with ExitStack() as held:
            c=verified(held)
            if compatibility_options()!=c.get('source_access'):raise ValueError('WrongControlSourceAccess')
            enter_compatibility(held,c.get('source_access'))
            r=c['recovery']
            renewed=timed('access_recovery',lambda:recover(r['source'],r['content'],r['scientific'],OUT/'access',progress=lambda s:print(s,flush=True)))
            ops=held.enter_context(qualified_dispatch())
            parent_ref=next(n['reference'] for n in renewed['nodes'] if n['content']==c['parent_content'])
            base=timed('parent_open',lambda:held.enter_context(CommittedState(parent_ref)))
            if base.layer!=10 or base.content_root!=c['parent_content']:raise ValueError('WrongCapsuleParent')
            indices=timed('indices',lambda:renew_indices(base,c['indices'],c['vertices']))
            case=dict(c['case'],reference=base.reference,committed=base.expected['committed'])
            wall,tri,surface,request=load_case(case)
            owner=ScalarOffsetGeometry(case['spec'],case['geometry_identity'])
            adapter=timed('binding',lambda:bind_forecast(base,case,owner))
            timed('source_binding_comparison',lambda:compare_enrollment_binding(base,case,owner,adapter,c.get('source_access')))
            inspected=inspect_front(base,wall,tri,surface,request,base.expected)
            geometry,pairs=held.enter_context(indexed_session(base,adapter,indices))
            held.callback(close_preparation_jobs,RUN)
            from accepted_production_contract import plan_transition
            plan=plan_transition(base,adapter)
            if plan['target_layer']!=11 or plan['increment']!=0.0008700835643990311:
                raise ValueError('WrongCapsuleIncrement')
            durable_json(RUN/'BINDING.json',dict(parent=base.scientific,geometry=owner.geometry_identity,
                source=owner.source_identity,plan=plan,controls=dict(PRODUCTION_CONTROLS),indices=indices))
            timed('admission',lambda:ops['admission'](base,adapter,case['spec'],RUN))
            timed('generation',lambda:ops['generation'](base,adapter,case['spec'],RUN))
            timed('assembly',lambda:ops['assembly'](base,adapter,RUN,pair_index=pairs))
            original=composable_publication.prepare;compared=[]
            def checked(parent,authority,out):
                if parent is not base or authority is not adapter or out!=RUN or compared:
                    raise ValueError('WrongCapsuleComparisonBoundary')
                actual=original(parent,authority,out);expected=read(c['reference_plan'])
                result=compare_plan(actual,expected)
                durable_json(RUN/'REFERENCE_COMPARISON.json',result)
                if not result['passed']:raise ValueError('CapsuleScientificComponentMismatch')
                compared.append(actual['scientific']);return actual
            with patch.object(composable_publication,'prepare',checked):
                reference=timed('publication',lambda:ops['publication'](base,adapter,RUN,pairs))
            if len(compared)!=1:raise ValueError('MissingCapsuleScientificComparison')
            import assembly_quality_handoff as handoff
            durable_json(RUN/'REUSE.json',dict(handoff._active.get()['stats']))
            base.check()
        durable_json(RUN/'RESULT.json',dict(passed=True,layer=11,scientific=compared[0],reference=reference,
            phases=phases,admission_to_publication=sum(phases[k] for k in ('admission','generation','assembly','publication')),
            seconds=time.perf_counter()-start,peak_main_working_set=psutil.Process().memory_info().peak_wset,
            no_fresh_replay=True,no_deep_qualification_campaign=True,canonical_layers_unchanged=True))
    except BaseException as error:
        durable_json(RUN/'STOP.json',dict(code=str(error),exception=type(error).__name__,stage=stage,
            traceback=traceback.format_exc(),phases=phases,seconds=time.perf_counter()-start,
            peak_main_working_set=psutil.Process().memory_info().peak_wset))
        raise


def reopen():
    from composable_state import CommittedState
    from profile_remaining_prism import memory_sample
    start=time.perf_counter()
    with ExitStack() as held:
        c=verified(held);result=read(RUN/'RESULT.json')
        if compatibility_options()!=c.get('source_access'):raise ValueError('WrongControlSourceAccess')
        enter_compatibility(held,c.get('source_access'))
        if result.get('passed') is not True or (RUN/'STOP.json').exists():raise ValueError('IncompleteCapsuleControl')
        with memory_sample(RUN,'reopen'),CommittedState(result['reference']) as state:
            state.check()
            if state.layer!=11 or state.scientific!=c['expected_scientific']:
                raise ValueError('CapsuleControlReopenMismatch')
            durable_json(RUN/'REOPEN.json',dict(passed=True,layer=state.layer,scientific=state.scientific,
                reference=state.reference,cells=len(state['cells']),proofs=len(state['certificate']),
                quality=len(state['quality']),seconds=time.perf_counter()-start,stats=dict(state.stats)))


def monitor():
    import finish_retained_l5 as existing
    import os
    from unittest.mock import patch
    with ExitStack() as held:verified(held)
    existing.CAMPAIGN=RUN;existing.OUT=MONITOR;MONITOR.mkdir(exist_ok=False)
    args=[sys.executable,'-B',__file__,'worker','--output',str(OUT)]
    if SOURCE_CONTRACT is not None:args.extend(['--source-contract',str(SOURCE_CONTRACT),'--source-review',str(SOURCE_REVIEW)])
    with patch.dict(os.environ,V4_SHARED_CAPSULE_MONITOR_PID=str(os.getpid()),V4_SHARED_CAPSULE_MONITOR_OUTPUT=str(MONITOR.resolve())):
        return existing.monitor(worker_command=args,extra_storage_roots=(OUT/'access',OUT/'indices'),enforce_output_cap=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('preflight','worker','monitor','reopen'))
    p.add_argument('--output');p.add_argument('--source-contract');p.add_argument('--source-review')
    args=p.parse_args();mode=args.mode
    configure(args.output,args.source_contract,args.source_review)
    start=time.perf_counter()
    try:result=globals()[mode]()
    except BaseException as error:
        if mode=='preflight':
            import traceback,psutil
            failure=OUT/'PREFLIGHT_FAILURE.json'
            if failure.exists():raise ValueError('PreserveCapsulePreflightFailure') from error
            ensure_output_directory()
            durable_json(failure,dict(passed=False,stage='preflight',exception=type(error).__name__,
                arguments=list(error.args),traceback=traceback.format_exc(),seconds=time.perf_counter()-start,
                peak_main_working_set=psutil.Process().memory_info().peak_wset,
                production_started=False,recovery_started=False,layer_advanced=False))
        raise
    if mode=='monitor':sys.exit(result)
