"""Compact preservation and resource record for the receipt-boundary task."""
import json, subprocess, shutil, time, sys
from pathlib import Path
V4=Path(__file__).resolve().parents[1]; ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.common.authority import file_hash
from path_preserving_deduplication import durable_json
OUT=V4/'results/first_layer_receipt_connection'
def baseline():
    old=json.loads((V4/'results/authorized_continuation/BASELINE.json').read_text(encoding='utf-8'))
    records=[dict(path=r['path'],sha256=file_hash(ROOT/r['path']),bytes=(ROOT/r['path']).stat().st_size) for r in old['files']]
    status=subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode().splitlines()
    task=('research/sdf_cfd_mesher_v4/tools/first_layer_receipt_connection.py','research/sdf_cfd_mesher_v4/tools/record_receipt_task.py','research/sdf_cfd_mesher_v4/tools/critical_assembly.py')
    unchanged=[s for s in status if s[3:] not in task]
    if unchanged!=old['status']:raise ValueError('BaselineUnrelatedStatusMismatch')
    OUT.mkdir(exist_ok=False)
    durable_json(OUT/'BASELINE.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
      status=unchanged,files=records,initial_C=150808977408,initial_Z=847650447360,start_epoch=time.time(),
      note='Critical assembly is task-owned and its pre-edit bytes are committed in starting HEAD.'))
def finish():
    import ctypes
    import hashlib
    read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
    base=read(OUT/'BASELINE.json');run=OUT/'resume'
    task=['tools/'+n for n in ('first_layer_receipt_connection.py','retained_transition_input.py','record_receipt_task.py','diagnose_first_layer_receipts.py',
         'critical_assembly.py','critical_prepare.py','run_authorized_continuation.py')]
    task+=['tests/test_first_layer_receipt_connection.py','tests/test_retained_transition_input.py',
           'FIRST_LAYER_RECEIPT_CONNECTION_REPORT.md']
    allowed={'research/sdf_cfd_mesher_v4/'+p for p in task}
    status=subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode().splitlines()
    unrelated=[s for s in status if s[3:] not in allowed and not s[3:].startswith('research/sdf_cfd_mesher_v4/results/first_layer_receipt_connection')]
    if unrelated!=base['status']:raise ValueError('UnrelatedStatusMismatch')
    changed=[r['path'] for r in base['files'] if r['path'] not in allowed and file_hash(ROOT/r['path'])!=r['sha256']]
    if changed:raise ValueError('ProtectedContentChanged:'+str(changed))
    prior=read(V4/'results/authorized_continuation/EVIDENCE.json')
    retained=[r for r in prior['files'] if '/encoding_resume/L2/' in r['path']]
    if any(file_hash(ROOT/r['path'])!=r['sha256'] for r in retained):raise ValueError('RetainedCandidateChanged')
    layers={}
    for folder in sorted(run.glob('L[2-5]')):
        row={}
        for name in ('ADMISSION','GENERATION','ASSEMBLY','FRESH_CERTIFICATE','VOLUME_DELTA','REPLAY','REOPEN',
                     'FIRST_LAYER_RECEIPT_CONNECTION','PHASES','PRIOR_DETERMINANT_COMPARISON'):
            p=folder/(name+'.json')
            if p.exists():
                val=read(p)
                row[name]=val if name in ('ASSEMBLY','REPLAY','REOPEN','FIRST_LAYER_RECEIPT_CONNECTION') else dict(sha256=file_hash(p))
                if name=='GENERATION':row['counts']=val['counts'];row['phases']=val['phases'];row['hard_failures']=val['hard_failures'];row['topology']=val['topology_audit']
                if name=='FRESH_CERTIFICATE':row['proofs']={k:v for k,v in val.items() if k in ('passed','failures','required_pairs','combined_pairs','codes')}
                if name=='VOLUME_DELTA':
                    degree=val['global_degree']
                    row['volume_degree']={k:v for k,v in val.items() if k!='global_degree'}
                    row['volume_degree']['global_degree']=dict(passed=degree['passed'],unresolved=degree['unresolved'],
                        values_count=len(degree['values']),nonunit_count=sum(r['degree']!=1 for r in degree['values']),
                        full_record_sha256=file_hash(p))
        p=folder/'NEW_QUALITY.json'
        if p.exists():
            q=read(p);row['quality']=dict(passed=q['passed'],failures=q['failures'],cells=len(q['cells']),faces=len(q['faces']),
              extrema={k:float(fun(r[k] for r in q[group])) for k,group,fun in (
                ('volume','cells',min),('determinant','cells',min),('aspect','cells',max),('weight','faces',min),
                ('skewness','faces',max),('symmetric_volume_ratio','faces',min))},
              exact_cells=len(q['exact']),exact_nonpositive=sum(r['volume_sign']<=0 or r['jacobian_sign']<=0 for r in q['exact']))
        layers[folder.name]=row
    summary=dict(layers=layers,stop=read(run/'STOP.json') if (run/'STOP.json').exists() else None,
        result=read(run/'RESULT.json') if (run/'RESULT.json').exists() else None,resources=read(run/'RESOURCES.json'))
    old_summary=OUT/'SUMMARY.json'
    if old_summary.exists() and old_summary.stat().st_size>1000000:
        durable_json(OUT/'SUMMARY_REDUCTION.json',dict(old_sha256=file_hash(old_summary),old_bytes=old_summary.stat().st_size,
          reason='Remove duplicated degree rows from task-owned summary; counts and source hash retained. Original full VOLUME_DELTA and every other scientific input remain unchanged.',
          original_source='research/sdf_cfd_mesher_v4/results/authorized_continuation/encoding_resume/L2/VOLUME_DELTA.json'))
    durable_json(old_summary,summary)
    originals={(ROOT/r['path']).stat().st_ino for r in retained}
    seen=set();files=[];allocated=0;logical=0;shared=0
    get=ctypes.windll.kernel32.GetCompressedFileSizeW;get.argtypes=[ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_ulong)];get.restype=ctypes.c_ulong
    for p in sorted([p for p in OUT.rglob('*') if p.is_file() and p.name!='EVIDENCE.json']+[V4/n for n in task if (V4/n).exists()]):
        s=p.stat();logical+=s.st_size
        hi=ctypes.c_ulong();lo=get(str(p),ctypes.byref(hi));size=lo+(hi.value<<32)
        reused=s.st_ino in originals
        if reused:shared+=s.st_size
        elif s.st_ino not in seen:allocated+=size
        seen.add(s.st_ino)
        files.append(dict(path=str(p.relative_to(ROOT)).replace('\\','/'),bytes=s.st_size,allocated=size,shared_retained=reused,sha256=file_hash(p)))
    def capacity(path):
        available=ctypes.c_ulonglong();total=ctypes.c_ulonglong();free=ctypes.c_ulonglong()
        if not ctypes.windll.kernel32.GetDiskFreeSpaceExW(str(path),ctypes.byref(available),ctypes.byref(total),ctypes.byref(free)):raise OSError('DiskMeasurementFailed')
        return dict(available_to_caller=available.value,total=total.value,total_free=free.value)
    disks={drive:capacity(drive+':/') for drive in ('C','Z')}
    source_changes=[]
    for name in ('critical_assembly.py','critical_prepare.py','run_authorized_continuation.py'):
        path='research/sdf_cfd_mesher_v4/tools/'+name
        original=subprocess.check_output(['git','show',base['head']+':'+path],cwd=ROOT)
        source_changes.append(dict(path=path,before_git_blob_sha256=hashlib.sha256(original).hexdigest(),after_worktree_sha256=file_hash(ROOT/path)))
    durable_json(OUT/'EVIDENCE.json',dict(starting_head=base['head'],branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        initial_C=base['initial_C'],initial_Z=base['initial_Z'],final_C=disks['C']['available_to_caller'],final_Z=disks['Z']['available_to_caller'],disks=disks,
        elapsed_from_baseline_s=time.time()-base['start_epoch'],retained_logical=logical,new_allocated=allocated,shared_existing_logical=shared,
        preservation=dict(status_entries=len(unrelated),protected_files=len(base['files'])-sum(r['path'] in allowed for r in base['files']),
          changed_protected=changed,retained_L2_files=len(retained),all_retained_hashes_equal=True,scope='Selected protected sources/evidence, exact case-sensitive status; no whole-repository inventory'),
        task_files=task,source_changes=source_changes,files=files,cleanup='Only duplicate rows removed from task-owned SUMMARY; original full record, prior summary hash and reduction reason retained. No original candidate, proof or ancestry deletion.'))
if __name__=='__main__':
    if '--finish' in sys.argv:finish()
    else:baseline()
