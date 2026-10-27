"""Compact preservation/resource record for the authorized retained resume."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
from fractions import Fraction
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[3]
V4 = ROOT / 'research/sdf_cfd_mesher_v4'
OUT = V4 / 'results/wing_receipt_resume'

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def disk():
    result = {}
    for drive in ('C:\\', 'Z:\\'):
        available = ctypes.c_ulonglong()
        total = ctypes.c_ulonglong()
        free = ctypes.c_ulonglong()
        if not ctypes.windll.kernel32.GetDiskFreeSpaceExW(drive, ctypes.byref(available), ctypes.byref(total), ctypes.byref(free)):
            raise ctypes.WinError()
        result[drive] = dict(available=available.value, total=total.value, free=free.value)
    return result

def status():
    return subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode('utf-8').splitlines()

def record(final=False):
    old = json.loads((V4 / 'results/first_layer_receipt_connection/BASELINE.json').read_text(encoding='utf-8'))
    paths = {r['path'] for r in old['files']}
    paths.update('research/sdf_cfd_mesher_v4/tools/' + name for name in (
        'run_authorized_continuation.py','first_layer_receipt_connection.py','retained_transition_input.py',
        'critical_assembly.py','critical_prepare.py','accepted_production_contract.py'))
    files = [dict(path=p, bytes=(ROOT/p).stat().st_size, sha256=sha(ROOT/p)) for p in sorted(paths)]
    current = status()
    missing = [s for s in old['status'] if s not in current]
    if missing:
        raise ValueError('BaselineStatusChanged', missing)
    body = dict(utc=datetime.now(timezone.utc).isoformat(),
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        status=current, unrelated_status=old['status'], files=files, disk=disk())
    OUT.mkdir(exist_ok=True)
    name = 'FINAL.json' if final else 'BASELINE.json'
    target = OUT/name
    if target.exists():
        raise ValueError('PreserveExistingRecord',str(target))
    if final:
        before=json.loads((OUT/'BASELINE.json').read_text(encoding='utf-8'))
        baseline={r['path']:r for r in before['files']}
        body['changed_protected_files']=[r['path'] for r in files if r!=baseline[r['path']]]
    target.write_text(json.dumps(body,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps(dict(record=str(target),files=len(files),unrelated=len(old['status']),disk=body['disk'])))

def summary():
    """Reduce completed existing evidence; do not evaluate or replace any gate."""
    read=lambda p:json.loads(p.read_text(encoding='utf-8'))
    run=OUT/'A'
    result={n:read(run/n) for n in ('RESULT.json','STOP.json','LAST_QUALIFIED.json','RESOURCES.json','PHASES.json') if (run/n).exists()}
    result['layers']={}
    for layer in range(2,6):
        folder=run/f'L{layer}'
        if not folder.exists():continue
        item={}
        for name in ('CONNECTION','TARGET','ADMISSION','GENERATION','ASSEMBLY','REPLAY','REOPEN','FIRST_LAYER_RECEIPT_CONNECTION','PRIOR_DETERMINANT_COMPARISON'):
            p=folder/(name+'.json')
            if p.exists():item[name]=read(p)
        p=folder/'NEW_QUALITY.json'
        if p.exists():
            q=read(p)
            item['quality']=dict(passed=q['passed'],failures=q['failures'],cells=len(q['cells']),faces=len(q['faces']),
                minimum_volume=min(r['volume'] for r in q['cells']),
                minimum_exact_volume=str(min(Fraction(r['signed_volume']) for r in q['exact'])),
                minimum_exact_jacobian=str(min(Fraction(r['jacobian']) for r in q['exact'])),
                nonpositive_volume=sum(r['volume_sign']<=0 for r in q['exact']),
                nonpositive_jacobian=sum(r['jacobian_sign']<=0 for r in q['exact']),
                minimum_determinant=min(r['determinant'] for r in q['cells']),
                maximum_aspect=max(r['aspect'] for r in q['cells']),
                minimum_weight=min(r['weight'] for r in q['faces']),
                maximum_skewness=max(r['skewness'] for r in q['faces']),
                minimum_symmetric_volume_ratio=min(r['symmetric_volume_ratio'] for r in q['faces']))
            del q
        p=folder/'AFFECTED_INCIDENCE.json'
        if p.exists():
            q=read(p)
            item['incidence']=dict(passed=q['passed'],failures=q['failures'],cells=len(q['cells']),
                minimum_determinant=min(r['determinant'] for r in q['cells']),scope=q['scope'])
            del q
        p=folder/'FRESH_CERTIFICATE.json'
        if p.exists():
            q=read(p)
            item['continuous']={k:q[k] for k in ('passed','failures','required_pairs','combined_pairs','prior_population','prior_payloads_read','prior_payloads_recomputed','coverage','methods','proof_hash')}
        p=folder/'VOLUME_DELTA.json'
        if p.exists():
            q=read(p);degree=q['global_degree']
            item['volume']=dict(exact_balance=q['exact_balance'],new_signed_volume=q['new_signed_volume'],
                degree={k:v for k,v in degree.items() if k not in ('cells','values')},
                degree_value_count=len(degree.get('values',[])),
                degree_min=min(r['degree'] for r in degree['values']),
                degree_max=max(r['degree'] for r in degree['values']))
            del q
        item['files']=[dict(path=str(p.relative_to(ROOT)).replace('\\','/'),bytes=p.stat().st_size,sha256=sha(p))
            for p in sorted(folder.glob('*')) if p.is_file() and p.suffix in ('.json','.npz')]
        result['layers'][str(layer)]=item
    paths=[p for p in OUT.rglob('*') if p.is_file() and p.name!='SUMMARY.json']
    physical=0;unique=set();logical=sum(p.stat().st_size for p in paths)
    fn=ctypes.windll.kernel32.GetCompressedFileSizeW;fn.restype=ctypes.c_ulong
    for p in paths:
        token=(p.stat().st_dev,p.stat().st_ino)
        if token in unique:continue
        unique.add(token);high=ctypes.c_ulong();low=fn(str(p),ctypes.byref(high))
        if low==0xffffffff and ctypes.get_last_error():raise ctypes.WinError()
        physical+=(high.value<<32)|low
    result['storage']=dict(logical_bytes=logical,compressed_size_api_bytes=physical,files=len(paths),
        note='Compressed-size API is not cluster allocation. Includes links to pre-existing immutable payloads. See EVIDENCE for NTFS cluster allocation. C and Z capacities are not added.')
    result['disk']=disk()
    target=OUT/'SUMMARY.json'
    if target.exists():raise ValueError('PreserveExistingSummary')
    target.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps(dict(path=str(target),sha256=sha(target),storage=result['storage'])))

def monitor(pid):
    """Enforce the runner's existing allowance during long stages as well."""
    import psutil
    run=OUT/'A'
    start=json.loads((run/'START.json').read_text(encoding='utf-8'))
    resources=start['resources']
    size=sum(p.stat().st_size for p in run.rglob('*') if p.is_file())
    free=disk()['C:\\']['available']
    if size<=resources['output_limit_bytes'] and free>=resources['minimum_free_bytes']:
        print(json.dumps(dict(output_bytes=size,available_c=free,breach=False)))
        return
    process=psutil.Process(pid)
    command=process.cmdline()
    if not any('run_authorized_continuation.py' in arg for arg in command) or not any(Path(arg).resolve()==run.resolve() for arg in command if 'wing_receipt_resume' in arg):
        raise ValueError('RefuseToStopUnrelatedProcess')
    children=process.children(recursive=True)
    body=dict(code='OperationalResourceLimit',utc=datetime.now(timezone.utc).isoformat(),
        output_bytes=size,available_c=free,resources=resources,
        process=dict(pid=pid,command=command,memory=process.memory_info()._asdict()),
        children=[dict(pid=p.pid,command=p.cmdline(),memory=p.memory_info()._asdict()) for p in children],
        last_qualified=json.loads((run/'LAST_QUALIFIED.json').read_text(encoding='utf-8')),
        scope='External monitoring of unchanged runner limits; preserve partial stage, no publication or further advancement')
    target=OUT/'MONITOR_STOP.json'
    if target.exists():raise ValueError('PreservePriorMonitorStop')
    target.write_text(json.dumps(body,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    process.terminate()
    for child in children:
        try:child.terminate()
        except psutil.NoSuchProcess:pass
    _,alive=psutil.wait_procs([process]+children,timeout=10)
    if alive:raise ValueError('ResourceStopProcessesStillAlive',[p.pid for p in alive])
    print(json.dumps(body))

def evidence():
    from path_preserving_deduplication import metadata, physical_key
    read=lambda p:json.loads(p.read_text(encoding='utf-8'))
    source=V4/'results/authorized_continuation/encoding_resume/L2'
    manifest=V4/'results/authorized_continuation/EVIDENCE.json'
    expected='e66f3222eafdee15418525230c972501c1e37d31115a48493c85089150946993'
    if sha(manifest)!=expected:raise ValueError('OriginalManifestChanged')
    prefix=str(source.relative_to(ROOT)).replace('\\','/')+'/'
    original=[r for r in read(manifest)['files'] if r['path'].replace('\\','/').startswith(prefix)]
    for row in original:
        path=ROOT/row['path']
        if path.stat().st_size!=row['bytes'] or sha(path)!=row['sha256']:raise ValueError('OriginalCandidateChanged',str(path))
    seen=set();allocated=0;shared=0;logical=0;count=0
    for path in OUT.rglob('*'):
        if not path.is_file():continue
        r=metadata(path);logical+=r['bytes'];count+=1
        key=physical_key(r)
        if key in seen:continue
        seen.add(key);allocated+=r['cluster_bytes']
        if r['links']>1:shared+=r['cluster_bytes']
    stop=read(OUT/'MONITOR_STOP.json');start=read(OUT/'A/START.json')
    seconds=(datetime.fromisoformat(stop['utc'])-datetime.fromisoformat(start['utc'])).total_seconds()
    compact=[]
    for path in sorted(OUT.rglob('*')):
        if path.is_file() and path.suffix in ('.json','.txt','.log') and path.stat().st_size<200000:
            compact.append(dict(path=str(path.relative_to(ROOT)).replace('\\','/'),bytes=path.stat().st_size,sha256=sha(path)))
    comparison={}
    layer=OUT/'A/L5'
    for name in ('GENERATED_NEXT.npz','NEW_QUALITY.json','AFFECTED_INCIDENCE.json','GEOMETRY_DELTA.json','VOLUME_DELTA.json'):
        a=sha(layer/name);b=sha(layer/'replay'/name)
        comparison[name]=dict(candidate=a,replay=b,equal=a==b)
    body=dict(classification='V4WingL4CommittedL5ReplayResourceStopped',last_qualified_layer=4,
        worker_elapsed_to_stop_s=seconds,stop=stop,
        original_candidate_manifest=expected,original_candidate_files_verified=original,
        partial_L5_file_comparison=comparison,
        partial_comparison_scope='Read-only post-stop comparison only; required prepared-component replay incomplete.',
        physical=dict(files=count,logical_bytes=logical,ntfs_allocated_cluster_bytes=allocated,
            shared_preexisting_cluster_bytes=shared,new_unique_cluster_bytes=allocated-shared),
        summary_storage_correction='GetCompressedFileSizeW in SUMMARY is a compressed-size measurement, not NTFS cluster allocation; use physical here.',
        disk=disk(),compact_files=compact,
        preservation=read(OUT/'FINAL.json'),cleanup='None; all candidates, replay data, active ancestry and stopped output retained.')
    target=OUT/'EVIDENCE.json'
    if target.exists():raise ValueError('PreserveExistingEvidence')
    target.write_text(json.dumps(body,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps(dict(path=str(target),sha256=sha(target),physical=body['physical'],elapsed_s=seconds,partial=comparison,disk=body['disk'])))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--final',action='store_true');p.add_argument('--summary',action='store_true');p.add_argument('--monitor-pid',type=int);p.add_argument('--evidence',action='store_true')
    args=p.parse_args()
    if args.monitor_pid:monitor(args.monitor_pid)
    elif args.summary:summary()
    elif args.evidence:evidence()
    else:record(args.final)
