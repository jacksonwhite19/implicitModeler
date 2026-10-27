"""Resume retained assembly/publication only. No generation executor is called."""
import argparse
from contextlib import ExitStack, contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
OUT = V4/'results/retained_l5_finish'
CAMPAIGN = V4/'results/wing_receipt_resume/A'
FOLDER = CAMPAIGN/'L5'
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key
from record_receipt_resume import disk, status

def read(p): return json.loads(Path(p).read_text(encoding='utf-8'))
def save(n, v): durable_json(OUT/n, v)


def save_monitor_snapshot(name,value,retry_seconds):
    """Retry only a telemetry rename blocked by a short Windows reader.

    The sample was checked before this call. Never use this for scientific
    metadata. The pending fsynced bytes are reused, not regenerated or ignored.
    Bound the retry to the existing sampling interval; persistent errors fail.
    """
    if name not in ('MONITOR.json','RETENTION_MONITOR.json'):
        raise ValueError('UnsupportedTelemetryRetryPath')
    if type(retry_seconds) not in (int,float) or retry_seconds<0:
        raise ValueError('InvalidTelemetryRetryInterval')
    pending=OUT/(name+'.pending')
    # Preserve durable_json's bytes and sync ordering. Only the following
    # rename may be retried: a write/flush/fsync error is not a saved sample.
    with pending.open('w',encoding='utf-8',newline='\n') as stream:
        json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    from path_preserving_deduplication import atomic_replace
    deadline=time.perf_counter()+retry_seconds
    try:atomic_replace(pending,OUT/name);return
    except PermissionError as error:
        if getattr(error,'winerror',None) not in (5,32,33):raise
        original=error
    while time.perf_counter()<deadline:
        time.sleep(min(.05,max(0,deadline-time.perf_counter())))
        try:atomic_replace(pending,OUT/name);return
        except PermissionError as error:
            if getattr(error,'winerror',None) not in (5,32,33):raise
    raise original

def allocation(paths):
    records=[metadata(p) for root in paths for p in root.rglob('*') if p.is_file()]
    unique={physical_key(r):r for r in records}
    return dict(files=len(records), logical=sum(r['bytes'] for r in records),
        allocated=sum(r['cluster_bytes'] for r in unique.values()),
        shared_allocation=sum(r['cluster_bytes'] for r in unique.values() if r['links']>1))

def live_allocation(paths):
    """Observe allocation without denying concurrent writers or replacements."""
    import ctypes
    from ctypes import wintypes as W
    from path_preserving_deduplication import api,standard
    k=api();seen=set();allocated=0;missed=0
    for p in paths:
        handle=k.CreateFileW(str(p.resolve()),0,7,None,3,0,None)
        if handle==W.HANDLE(-1).value:
            missed+=1;continue
        try:
            r=standard(handle)
            try:st=p.stat()
            except FileNotFoundError:
                missed+=1;continue
            key=(st.st_dev,st.st_ino)
            if key not in seen:allocated+=r['allocated'];seen.add(key)
        finally:k.CloseHandle(handle)
    return dict(allocated=allocated,files=len(seen),concurrent_misses=missed,
        scope='GetFileInformationByHandleEx allocation; concurrent snapshot, not authentication')

def baseline():
    if OUT.exists(): raise ValueError('PreservePriorFinishAttempt')
    prior=V4/'results/wing_storage_connection'
    evidence=read(prior/'EVIDENCE.json')
    if file_hash(prior/'EVIDENCE.json')!='f1d893eada1b049bf4bbfa58ddbd83d6a2884b6810b888961e09f12a3fc4af5a':
        raise ValueError('ChangedStorageEvidence')
    old=read(prior/'BASELINE.json')
    current=status()
    if any(s not in current for s in old['unrelated_status']): raise ValueError('ChangedUnrelatedStatus')
    head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    if head!='9fd493a9cf1668bf29a91443c8b63c4fe3a4ff13': raise ValueError('UnexpectedStartingHead')
    paths={r['path'] for r in old['files']}
    files=[]
    changed='research/sdf_cfd_mesher_v4/tools/critical_index_pipeline.py'
    for p in sorted(paths|{changed}):
        source=ROOT/p
        # This one intentional source edit was made after initial inspection.
        data=subprocess.check_output(['git','show','HEAD:'+p]) if p==changed else source.read_bytes()
        files.append(dict(path=p,bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
    dependencies=read(prior/'DEPENDENCIES.json')
    verified=[]
    for r in dependencies['active']+dependencies['failed_l5']:
        p=Path(r['path'])
        if p.stat().st_size!=r['bytes'] or file_hash(p)!=r['sha256']:
            raise ValueError('ChangedRetainedDependency:'+str(p))
        verified.append(dict(path=str(p),bytes=r['bytes'],sha256=r['sha256']))
    OUT.mkdir()
    save('BASELINE.json',dict(head=head,branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        utc=datetime.now(timezone.utc).isoformat(),unrelated=old['unrelated_status'],files=files,
        dependencies=verified,storage=allocation([CAMPAIGN]),disk=disk(),
        source_change=changed,storage_evidence=file_hash(prior/'EVIDENCE.json')))
    print('Baseline verified',len(verified),'retained dependencies',flush=True)

@contextmanager
def preserve_existing_exports(folder, records):
    """Compare the existing writer's encoding; never overwrite retained exports."""
    from unittest.mock import patch
    import composable_assembly, critical_prepare
    writer=durable_json
    def write(path,value):
        path=Path(path)
        if path.exists() and path.name in {'NEW_RECEIPT_INPUTS.json','NEW_ATLAS.json',
                'NEW_FORECAST_DEPENDENCIES.json','BINDING_PATCH.json','PRIOR_DETERMINANT_COMPARISON.json'}:
            h=hashlib.sha256();size=0
            for chunk in json.JSONEncoder(indent=2,sort_keys=True,allow_nan=False).iterencode(value):
                data=chunk.encode('utf-8');h.update(data);size+=len(data)
            h.update(b'\n');size+=1
            if path.stat().st_size!=size or file_hash(path)!=h.hexdigest():
                raise ValueError('RetainedExportMismatch:'+str(path))
            records.append(dict(path=str(path),sha256=h.hexdigest(),bytes=size))
        else:writer(path,value)
    with patch.object(composable_assembly,'durable_json',write),patch.object(critical_prepare,'durable_json',write):
        yield

def worker():
    import traceback
    import numpy as np
    import psutil
    import immutable_state_access as access
    from sdf_cfd_mesher_v4.common.authority import identity
    from check_sf2_continuation import existing_state_class, exact_vertex_map, STUDY, PRIOR, access_controls
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from accepted_production_contract import bind_production, plan_transition, validate_authorization
    from sealed_front_connection import load_case, qualified_dispatch
    from composable_state import CommittedState
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy,verify_kernel_sources
    start=time.perf_counter();phases={};stage='controls';exports=[]
    def timed(name,call):
        nonlocal stage
        stage=name;save('STAGE.json',dict(stage=stage,elapsed=time.perf_counter()-start))
        print(stage,flush=True);tick=time.perf_counter();result=call()
        phases[name]=time.perf_counter()-tick;save('PHASES.json',phases);return result
    try:
        controls=OUT/'controls';controls.mkdir()
        timed('capsule_and_access_controls',lambda:access_controls(controls))
        case=read(PRIOR/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
        diagnostic=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
        auth=read(V4/'results/wing_continuation_layers2to5/AUTHORIZATION.json')
        auth_id='bd2e3209707fd7f11d3082a069f2490a6903421e73ca16727f375683b4d96772'
        state,selected=existing_state_class().load(STUDY/'planner/selected_state')
        mapping=exact_vertex_map(state.wall,wall)
        with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz',allow_pickle=False) as z:
            first,refs=z['front'],z['face_refs']
        kernel,_=verify_kernel_sources()
        reference=read(CAMPAIGN/'L4/REOPEN.json')['reference']
        prior=read(OUT/'BASELINE.json'); rows={r['path']:r for r in prior['dependencies']}
        with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
            obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
            authority=bind_sf2_policy(bind_canonical(obj)).authority
            original=AcceptedTrajectoryOwner(authority,diagnostic,identity(diagnostic),wall,first,tri,refs,mapping,selected,parent=case['committed'])
            validate_authorization(auth,auth_id,original)
            with qualified_dispatch(),CommittedState(reference) as base,ExitStack() as sessions:
                if base.layer!=4 or base.scientific!='a68a11559a9301442a19293474b0715c50540bbed0cc4f72105c398fb7c5e374':
                    raise ValueError('WrongRetainedParent')
                adapter,inspected=bind_production(base,case,original,auth,auth_id)
                from prism_persistent_hotpath import geometry_session
                from incremental_pair_storage import PairIncidence,write_proof_index
                sessions.enter_context(geometry_session(base,adapter))
                pairs=sessions.enter_context(PairIncidence(base,base.plan['pair_incidence_manifest'],base.plan['pair_incidence_root']))
                plan=plan_transition(base,adapter)
                save('BINDING.json',dict(plan=plan,geometry=obj.identity,kernel=kernel,parent=base.scientific,
                    authorization=auth_id,owner=original.source_identity,schedule=original.schedule.sha256,
                    increment=adapter.schedule.row_record(5),scope='Retained L5 assembly; no forecast or generation calls'))
                replay=FOLDER/'replay'
                # Hold every retained input through both preparations; only
                # explicitly derived output files are left writable.
                writable={'NEW_RECEIPT_INPUTS.json','NEW_ATLAS.json','NEW_FORECAST_DEPENDENCIES.json',
                    'BINDING_PATCH.json','PRIOR_DETERMINANT_COMPARISON.json'}
                for root in (FOLDER,replay):
                    for p in root.iterdir():
                        if p.is_file() and p.name not in writable:
                            sessions.enter_context(access.guard(p))
                            r=rows[str(p)]
                            if file_hash(p)!=r['sha256']:raise ValueError('ChangedRetainedInput:'+str(p))
                    if read(root/'GENERATION.json')['transition_plan']!=plan or read(root/'ADMISSION.json')['evidence']['binding']!=adapter.binding:
                        raise ValueError('WrongRetainedTransition')
                import composable_assembly,composable_publication
                from critical_index_pipeline import IndexJob,JOBS
                from prism_receipt_witness import _WITNESSES
                from recursive_production_publication import publish
                def index_sha(root):return rows[str(root/'objects/new-proof-key-index.sqlite')]['sha256']
                with preserve_existing_exports(FOLDER,exports):
                    timed('replay_assembly',lambda:composable_assembly.assemble(base,adapter,replay,pair_index=pairs,retained_index_sha256=index_sha(replay)))
                    JOBS[str(FOLDER)]=IndexJob(base,FOLDER,write_proof_index,retained_sha256=index_sha(FOLDER))
                    a=timed('primary_prepare',lambda:composable_publication.prepare(base,adapter,FOLDER))
                    b=timed('replay_prepare',lambda:composable_publication.prepare(base,adapter,replay))
                    comparisons={k:b['components'].get(k)==v for k,v in a['components'].items()}
                    passed=all(comparisons.values()) and a['scientific']==b['scientific'] and set(a['components'])==set(b['components'])
                    durable_json(FOLDER/'REPLAY.json',dict(passed=passed,comparisons=comparisons,content=a['scientific'],replay_content=b['scientific']))
                    if not passed:raise ValueError('DeterministicScientificComponentMismatch')
                    reference=timed('recursive_publication',lambda:publish(base,adapter,FOLDER,pairs))
                for root in (FOLDER,replay):
                    job=JOBS.pop(str(root),None)
                    if job:job.close()
                    witness=_WITNESSES.pop(str(root),None)
                    if witness:witness['held'].close()
                base.check()
            def reopen():
                with CommittedState(reference) as opened:
                    opened.check()
                    if opened.layer!=5 or opened.scientific!=a['scientific']:raise ValueError('PublishedReopenMismatch')
                    result=dict(passed=True,reference=reference,scientific=opened.scientific)
                    durable_json(FOLDER/'REOPEN.json',result)
                    return result
            reopened=timed('guarded_reopen',reopen)
        from recursive_replay_retention import retain
        retention=timed('successful_replay_retention',lambda:retain(FOLDER))
        # Only redundant derived restart indices, after success and byte hashes
        # are preserved. Never remove their canonical retained index.
        removed=[]
        for root in (FOLDER,replay):
            record=read(root/'INDEX_RESTART.json');p=Path(record['reconstruction'])
            canonical=FOLDER/'objects/new-proof-key-index.sqlite'
            if p.parent!=root/'objects' or p.name!='restart-proof-key-index.sqlite' or file_hash(p)!=record['sha256'] or file_hash(canonical)!=record['sha256']:
                raise ValueError('ChangedRestartIndexBeforeRetention')
            m=metadata(p);removed.append(dict(**m,sha256=record['sha256'],recover_from=str(canonical)))
        save('RESTART_INDEX_RETENTION.json',dict(successful_reopen=reopened,files=removed))
        for r in removed:Path(r['path']).unlink()
        save('RESULT.json',dict(passed=True,last_qualified_layer=5,reopen=reopened,comparisons=comparisons,
            phases=phases,seconds=time.perf_counter()-start,peak_main_working_set=psutil.Process().memory_info().peak_wset,
            retention=retention,restart_index_allocation_reclaimed=sum(r['cluster_bytes'] for r in removed),
            no_generation=True,no_proof_recomputation=True,no_layer6=True))
    except BaseException as e:
        save('STOP.json',dict(code=str(e),exception=type(e).__name__,stage=stage,traceback=traceback.format_exc(),
            phases=phases,seconds=time.perf_counter()-start,peak_main_working_set=psutil.Process().memory_info().peak_wset))
        raise
    finally:save('PRESERVED_EXPORTS.json',exports)

def monitored_processes(proc,child):
    """An exited worker may disappear during the preceding storage sample.

    Ignore only a confirmed child exit. An unexplained missing LIVE worker is
    still an error. The caller must still check the collected resource sample.
    """
    import psutil
    try:return [proc]+proc.children(recursive=True)
    except psutil.NoSuchProcess:
        if child.poll() is None:raise
        return []


def monitor(retention_only=False, *, worker_command=None, extra_storage_roots=(), enforce_output_cap=True,
            resources=None, storage_roots=None):
    """Observe total storage; an explicit policy may disable the output cap.

    Legacy callers retain their original control. Monitoring-only mode does not
    disable the existing free-space safeguard or alter worker/scientific results.
    """
    if type(enforce_output_cap) is not bool:raise ValueError('InvalidOutputCapMode')
    import psutil
    resources=dict(resources) if resources is not None else read(CAMPAIGN/'START.json')['resources']
    peaks=dict(working_set=0,tree_working_set=0,logical=0,allocated=0)
    if not enforce_output_cap:
        resources=dict(resources,inherited_output_limit_bytes=resources.get('inherited_output_limit_bytes',resources['output_limit_bytes']),
            output_limit_bytes=None,total_storage_policy='monitor_without_output_cap')
    roots=[Path(p).resolve() for p in (storage_roots if storage_roots is not None else (CAMPAIGN,*extra_storage_roots,OUT))]
    if not roots or (storage_roots is not None and not any(OUT.resolve()==r or OUT.resolve().is_relative_to(r) for r in roots)):
        raise ValueError('MissingMonitoredOutputRoot')
    if len(set(roots))!=len(roots) or any(a!=b and (a in b.parents or b in a.parents) for a in roots for b in roots):
        raise ValueError('OverlappingMonitoredStorageRoots')
    env=dict(os.environ, PYTHONUTF8='1',PYTHONDONTWRITEBYTECODE='1')
    for name in ('CRITICAL','NUMERIC','PERSISTENT','LOOKUP','FACE_KEYS','DEGREE_OVERLAP','DEGREE_PROCESS','INDEX_OVERLAP','INDEX_PROCESS','QUALITY_BATCH'):
        env['V4_'+name]='1'
    env.update(V4_DEGREE_WORKERS='2',V4_QUALITY_CODEC='4',V4_PROFILE='0')
    start=time.perf_counter();samples=[];resource_stopped=False
    prefix='RETENTION_' if retention_only else ''
    def storage_sample():
        paths=[p for root in roots for p in root.rglob('*') if p.is_file()]
        logical=0
        for p in paths:
            try:logical+=p.stat().st_size
            except FileNotFoundError:pass
        return dict(logical=logical,allocation=live_allocation(paths),free=disk()['C:\\']['available'])
    def resource_failure(row):
        return ((enforce_output_cap and (row['logical']>resources['output_limit_bytes'] or
                row['allocation']['allocated']>resources['output_limit_bytes'])) or
                row['free']<resources['minimum_free_bytes'])
    initial=storage_sample()
    save(prefix+'RESOURCE_PREFLIGHT.json',dict(resources=resources,measurement=initial,
        storage_roots=[str(p) for p in roots]))
    if resource_failure(initial):
        save(prefix+'RESOURCE_STOP.json',dict(resources=resources,measurement=initial,stage='before_worker_launch'))
        return 1
    command=worker_command if worker_command is not None else ([sys.executable,'-B',str(Path(__file__).with_name('finish_retained_l5_retention.py'))] if retention_only else [sys.executable,'-B',__file__,'worker'])
    with (OUT/(prefix+'WORKER.log')).open('x',encoding='utf-8') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env)
        proc=psutil.Process(child.pid)
        while child.poll() is None:
            row=storage_sample();logical=row['logical'];tree=0;main=0
            for p in monitored_processes(proc,child):
                try:
                    m=p.memory_info();tree+=m.rss
                    if p.pid==proc.pid:main=m.peak_wset
                except psutil.NoSuchProcess:pass
            peaks['working_set']=max(peaks['working_set'],main);peaks['tree_working_set']=max(peaks['tree_working_set'],tree)
            peaks['logical']=max(peaks['logical'],logical)
            row.update(seconds=time.perf_counter()-start,main_peak=main,tree_working_set=tree)
            peaks['allocated']=max(peaks['allocated'],row['allocation']['allocated'])
            samples.append(row)
            if resource_failure(row):
                resource_stopped=True
                save(prefix+'RESOURCE_STOP.json',dict(resources=resources,measurement=row,peaks=peaks))
                for p in monitored_processes(proc,child):
                    try:p.terminate()
                    except psutil.NoSuchProcess:
                        if p.pid==proc.pid and child.poll() is None:raise
                child.wait();break
            try:
                save_monitor_snapshot(prefix+'MONITOR.json',dict(resources=resources,peaks=peaks,latest=row,pid=child.pid),resources['monitor_seconds'])
            except BaseException:
                # A persistently broken monitor must not leave an unmonitored
                # worker running. Do not convert the monitor failure to success.
                for p in monitored_processes(proc,child):
                    try:p.terminate()
                    except psutil.NoSuchProcess:
                        if p.pid==proc.pid and child.poll() is None:raise
                child.wait()
                raise
            time.sleep(resources['monitor_seconds'])
    worker_exit=child.wait()
    # A completed worker cannot turn a concurrently observed resource breach
    # into success. Preserve its REAL exit code separately from monitor outcome.
    exit_code=1 if resource_stopped and worker_exit==0 else worker_exit
    save(prefix+'MONITOR_FINAL.json',dict(exit_code=exit_code,worker_exit_code=worker_exit,resource_stopped=resource_stopped,
        peaks=peaks,seconds=time.perf_counter()-start,samples=samples,disk=disk(),storage_roots=[str(p) for p in roots],storage=allocation(roots)))
    print('Worker exit',worker_exit,'monitor exit',exit_code,flush=True)
    return exit_code

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','worker','monitor','retention-monitor']);a=p.parse_args()
    if a.mode=='baseline':baseline()
    elif a.mode=='worker':worker()
    else:sys.exit(monitor(a.mode=='retention-monitor'))
