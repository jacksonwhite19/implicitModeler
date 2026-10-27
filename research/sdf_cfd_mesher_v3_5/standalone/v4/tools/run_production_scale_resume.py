"""Bounded observer/supervisor around the unchanged native scientific path."""
import sys
sys.dont_write_bytecode=True
import argparse,json,os,subprocess,time,traceback
from pathlib import Path
from collections import Counter
import psutil
import production_scale_authority as authority
import run_geometry_family as native
from production_phase_profile import Profile
from production_compute_control import ProgressLedger,BranchStopped,stop_reason,termination,checkpoint_stop_record,prior_case_progress
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,ReceiptTable

OUT=native.ROOT/'results/production_scale_geometry_resume'


def read(path):return json.loads(path.read_text())


def setup():
    OUT.mkdir(parents=True,exist_ok=True)
    prior=native.ROOT/'results/first_layer_determinant_applicability'
    expected={'report.md':'25a73ce1fde14d2829f1d00503c4dda7e95782bd1fe1b3ded85875e917a4f6dc',
              'AUTHORITY.json':'ebe5475f4423c9d87d37547bc57f7a99b551c28a931184c4cddf09942070aed3'}
    assert all(file_hash(prior/k)==v for k,v in expected.items())
    path=OUT/'TASK_MANIFEST.json'
    if not path.exists():
        status=subprocess.check_output(['git','status','--short'],text=True)
        native.g.save(path,dict(parent='9924097c13acbc99dea8f6bf2e8fd1988ec858d9',prior=expected,
            cases=authority.cohort(),hard_total_seconds=14400,default_case_seconds=2700,
            hard_case_seconds=4500,stagnant_stage_seconds=900,rejected_without_progress=10,
            semantics='Operational task controls, not scientific gates',
            unrelated=[r for r in status.splitlines() if 'research/sdf_cfd_mesher_v4' not in r]))


def summary(state):
    cells=Counter(c['kind'] for c in state['cells'].values())
    layers=Counter(c['layer'] for c in state['cells'].values())
    values={k:[] for k in ('determinant','weight','volume','symmetric_volume_ratio','skewness','aspect')}
    # Receipt rows are streamed. Only scalar extrema are retained below.
    extrema={k:None for k in values}
    for _,receipt in state['quality'].items():
        row=receipt.get('value',{})
        for k in extrema:
            if k in row:
                v=row[k];old=extrema[k]
                extrema[k]=v if old is None else (max(old,v) if k in ('skewness','aspect') else min(old,v))
    return dict(layer=max(layers,default=0),cells=dict(cells),layer_population=dict(layers),
        columns=dict(Counter(c['status'] for c in state['columns'].values())),
        remesh_events=len(state['events']),relations=len(state['certificate']),extrema=extrema,
        proofs_disk_backed=isinstance(state['certificate'],ReceiptTable),
        version_summary=dict(entities=len(state['versions']),geometry_max=max(v['geometry'] for v in state['versions'].values()),topology_max=max(v['topology'] for v in state['versions'].values())),atlas_regions=len(state['atlas']),forecast_entries=len(state['forecast']))


def worker(name,label,resume=False):
    spec=next(s for s in authority.cohort() if s['name']==name)
    folder=OUT/name/label;folder.mkdir(parents=True,exist_ok=True)
    ledger=ProgressLedger(folder/'PROGRESS.jsonl');profile=Profile(folder/'MEMORY.jsonl')
    original=[];proc=psutil.Process()
    def wrap(obj,key,phase):
        old=getattr(obj,key);original.append((obj,key,old));measured=profile.wrap(old,phase)
        def completed(*args,**kwargs):
            result=measured(*args,**kwargs)
            from production_compute_control import record_validated_stage
            record_validated_stage(ledger,key,result)
            return result
        setattr(obj,key,completed)
    def factory(spec,folder):
        with profile.phase('wall_load'):provider=native.AnalyticProvider(spec,folder)
        provider.compute_observer=ledger.trial
        for key in ('initialize','propose','evidence','discover','remesh','numerical_validation',
                    'certify_validation','rebuild','bind','pack','geometry','candidate_check','certify_candidate'):
            wrap(provider,key,key)
        return provider
    def observe(event,state,provider):
        data=summary(state)
        from retained_state_storage import retained_storage
        data['retained_storage']=retained_storage(dict(native_state=state,provider_records=provider.records,previous_state=provider.previous_certificate_state,mesh_cache=provider.meshes,quality_cache=provider.checked,numeric_cache=provider.numerical))
        ledger.progress(event,**data,rss=proc.memory_info().rss,
            provider_cache_entries={k:len(getattr(provider,k)) for k in ('meshes','checked','numerical')})
        native.g.save(folder/'LATEST_ACCEPTED.json',dict(**data,current=read(folder/'checkpoints/CURRENT.json')))
    qualified_numeric=set()
    old_validation=native.g.validation
    def validation(*args,**kwargs):
        with profile.phase('hard_quality_and_certificate'):
            result=old_validation(*args,**kwargs)
        points,cells,outer,checked,times=result
        if checked['quality']['passed']:
            layer=max(c['layer'] for c in cells);key=(layer,len(cells))
            if key not in qualified_numeric:
                qualified_numeric.add(key)
                ledger.progress('numerical_candidate_passed',layer=layer,
                    cells=len(cells),geometry=native.g.q.array_hash(points),scope='Numerical candidate only; not a committed or complete certificate receipt',times=times)
        return result
    original.append((native.g,'validation',old_validation));native.g.validation=validation
    import reuse_mixed_certificate as reuse
    old_certificate=reuse.certify;certified=set()
    def certificate(*args,**kwargs):
        tick=time.perf_counter()
        with profile.phase('exact_certificate_reuse_and_new_relations'):
            result=old_certificate(*args,**kwargs)
        key=result['proof_hash'];fresh=key not in certified;certified.add(key)
        record=ledger.progress if result['passed'] and fresh else ledger.record
        record('certificate_complete',passed=result['passed'],required=result['required_pairs'],
            reused=result.get('reused',0),new=result.get('new',result['required_pairs']),
            certificate_seconds=time.perf_counter()-tick,proof_hash=key)
        return result
    original.append((reuse,'certify',old_certificate));reuse.certify=certificate
    old_publish=NativeCheckpointStore.publish
    def publish(store,state,**kwargs):
        result=old_publish(store,state,**kwargs)
        ledger.progress('checkpoint_published',layer=max(c['layer'] for c in state['cells'].values()),
            cells=len(state['cells']),remesh_events=len(state['events']),scientific=result,rss=proc.memory_info().rss)
        return result
    original.append((NativeCheckpointStore,'publish',old_publish));NativeCheckpointStore.publish=publish
    import general_local_correction as search
    tangent=search.tangent_search;compression=search.search_compression
    def bounded_tangent(*args,**kwargs):
        ledger.start_branch('TangentialCorrection')
        try:return tangent(*args,**kwargs)
        except BranchStopped as exc:return None,[dict(compute_stop=str(exc),scientific_classification=None)]
    def bounded_compression(*args,**kwargs):
        ledger.start_branch('CompressionCorrection')
        try:return compression(*args,**kwargs)
        except BranchStopped as exc:return None,[dict(compute_stop=str(exc),scientific_classification=None)],{},None
    original.extend([(search,'tangent_search',tangent),(search,'search_compression',compression)])
    search.tangent_search=bounded_tangent;search.search_compression=bounded_compression
    from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
    for key in ('publish','initialize','restart','transact'):wrap(MixedFrontJournal,key,'checkpoint_'+key)
    try:
        native.run(name,label,resume=resume,spec=spec,output=OUT,provider_factory=factory,observer=observe)
        result=read(folder/'SHELL_RESULT.json')
        native.g.save(folder/'TERMINATION.json',dict(reason='ShellQualified' if result['all_active_schedules_complete'] else 'ImplementationBlocked'))
    except BaseException as exc:
        native.g.save(folder/'TERMINATION.json',dict(reason='ImplementationBlocked',error=type(exc).__name__+':'+str(exc),traceback=traceback.format_exc()))
        raise
    finally:
        profile.close()
        for obj,key,value in reversed(original):setattr(obj,key,value)


def supervise(name,label,resume=False):
    setup();folder=OUT/name/label;folder.mkdir(parents=True,exist_ok=True)
    if (folder/'SESSION.json').exists():raise ValueError('FreshSessionLabelRequired')
    records=[read(p) for p in OUT.glob('*/*/SESSION.json')]
    audits=[read(p) for p in OUT.glob('*/*/COMMITTED_RESULT.json')]
    native.g.save(folder/'SOURCE_SNAPSHOT.json',{str(p.relative_to(native.ROOT)):file_hash(p) for root in ('src','tools') for p in sorted((native.ROOT/root).rglob('*.py'))})
    task_used=sum(r['seconds'] for r in records);case_used=sum(r['seconds'] for r in records if r['case']==name)
    task_used+=sum(r['replay_seconds'] for r in audits)
    case_used+=sum(r['replay_seconds'] for r in audits if r['case']==name)
    seed=folder/'SEED_AUTHORITY.json';seed_seconds=read(seed)['seconds'] if seed.exists() else 0
    task_used+=seed_seconds;case_used+=seed_seconds
    task_started=(OUT/'TASK_MANIFEST.json').stat().st_mtime
    inherited_progress=prior_case_progress(OUT,name)
    if stop_reason(0,0,inherited_progress,case_used,task_used):raise ValueError('ComputeBudgetExhausted')
    args=[sys.executable,'-B',str(Path(__file__)),'worker',name,'--label',label]+(['--resume'] if resume else [])
    start=time.monotonic();last=start;post2=inherited_progress;candidate_progress=False;peak=0;seen=0;reason=None;at_budget_checkpoint=False
    with (folder/'CONSOLE.log').open('w') as log:
        child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        while child.poll() is None:
            elapsed=time.monotonic()-start
            progress=folder/'PROGRESS.jsonl'
            if progress.exists():
                lines=progress.read_text().splitlines()
                for line in lines[seen:]:
                    try:row=json.loads(line)
                    except json.JSONDecodeError:break
                    seen+=1
                    if row.get('progress'):last=start+row['seconds']
                    if row.get('event')=='certificate_complete' and row.get('passed') and row.get('new',0)>0:candidate_progress=True
                    if row.get('event')=='checkpoint_published' and row.get('layer',0)>2:post2=True
                    if row.get('event')=='retained_after_commit' and row.get('layer',0)>2 and case_used+elapsed>=2700:
                        at_budget_checkpoint=True
            try:peak=max(peak,psutil.Process(child.pid).memory_info().rss)
            except psutil.NoSuchProcess:pass
            reason=stop_reason(elapsed,time.monotonic()-last,post2 or candidate_progress,case_used,max(task_used,time.time()-task_started-elapsed))
            if not reason and case_used+elapsed>=4200:reason='CaseBudgetReserveForCheckpointAudit'
            if not reason and time.time()-task_started>=13800:reason='TaskBudgetReserveForCheckpointAudit'
            if not reason and at_budget_checkpoint:reason='DefaultCaseBudgetAfterDurableProgress'
            if reason:
                child.terminate();child.wait()
                native.g.save(folder/'TERMINATION.json',checkpoint_stop_record(folder,reason,post2 or candidate_progress))
                break
            time.sleep(2)
    seconds=time.monotonic()-start
    record=dict(case=name,label=label,seconds=seconds+seed_seconds,worker_wall_seconds=seconds,seed_seconds=seed_seconds,cold_from_layer0=not resume,exit_code=child.returncode,
                peak_rss=peak,task_accumulated_seconds=task_used+seconds,case_accumulated_seconds=case_used+seconds)
    native.g.save(folder/'SESSION.json',record)
    native.g.save(OUT/'COMPUTE_LEDGER.json',dict(sessions=records+[record],active_case_execution_seconds=task_used+seconds,
        hard_cap_seconds=14400,cap_reached=task_used+seconds>=14400))
    print(json.dumps(record),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['worker','run','setup']);ap.add_argument('case',nargs='?');ap.add_argument('--label',default='A');ap.add_argument('--resume',action='store_true');a=ap.parse_args()
    if a.mode=='setup':setup()
    elif a.mode=='worker':worker(a.case,a.label,a.resume)
    else:supervise(a.case,a.label,a.resume)
