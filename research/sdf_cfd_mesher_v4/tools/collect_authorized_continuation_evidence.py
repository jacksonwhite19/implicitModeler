"""Hash task outputs and compare the bounded pre-edit preservation record."""
import json
import shutil
import subprocess
import sys
import ctypes
from datetime import datetime,timezone
from pathlib import Path
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.common.authority import file_hash
from path_preserving_deduplication import durable_json,metadata
OUT=V4/'results/authorized_continuation'
PREFIX='research/sdf_cfd_mesher_v4/'
EDITED=['composable_admission.py','production_readonly_admission.py','critical_prism_transition.py',
        'critical_assembly.py','composable_assembly.py','critical_prepare.py','composable_publication.py']
NEW=['accepted_production_contract.py','run_authorized_continuation.py','summarize_authorized_continuation.py','collect_authorized_continuation_evidence.py']
TASK={PREFIX+'tools/'+n for n in EDITED+NEW}|{PREFIX+'tests/test_accepted_production_contract.py',PREFIX+'AUTHORIZED_CONTINUATION_REPORT.md'}
def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def owned(p):return p in TASK or p.startswith(PREFIX+'results/authorized_continuation/')
def available(drive):
    caller=ctypes.c_ulonglong();total=ctypes.c_ulonglong();free=ctypes.c_ulonglong()
    if not ctypes.windll.kernel32.GetDiskFreeSpaceExW(str(drive),ctypes.byref(caller),ctypes.byref(total),ctypes.byref(free)):
        raise ctypes.WinError()
    return dict(available_to_caller=caller.value,total_free=free.value,total=total.value)
def main():
    baseline=read(OUT/'BASELINE.json');same=[];changed=[]
    for r in baseline['files']:
        p=ROOT/r['path'];now=file_hash(p)
        if now==r['sha256'] and p.stat().st_size==r['bytes']:same.append(r['path'])
        else:
            if r['path'] not in {PREFIX+'tools/'+n for n in EDITED}:raise ValueError('ProtectedFileChanged:'+r['path'])
            changed.append(dict(path=r['path'],before=r['sha256'],after=now))
    status=subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode('utf-8').splitlines()
    unrelated=[s for s in status if not owned(s[3:])]
    if unrelated!=baseline['status']:raise ValueError('UnrelatedCaseSensitiveStatusChanged')
    preservation=dict(unrelated_entries=len(unrelated),exact_status_match=True,protected_baseline_files=len(baseline['files']),
        unchanged_count=len(same),unchanged_paths=same,intentional_changes=changed,
        coverage='Selected source/evidence file bytes plus exact porcelain entries and guarded opened authorities; not a full repository or full proof-payload inventory',
        no_deletion=True,no_v3_or_kernel_edits=True)
    durable_json(OUT/'PRESERVATION.json',preservation)
    files=[]
    paths=sorted({ROOT/p for p in TASK if (ROOT/p).exists()}|{p for p in OUT.rglob('*') if p.is_file() and p.name!='EVIDENCE.json'})
    for p in paths:
        m=metadata(p)
        files.append(dict(path=p.relative_to(ROOT).as_posix(),bytes=m['bytes'],allocated=m['allocated'],cluster_bytes=m['cluster_bytes'],sha256=file_hash(p)))
    branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip()
    evidence=dict(classification='V4AcceptedFrontProductionAdmissionConnectedL2AssemblyBlocked',
        starting_head=baseline['head'],head_at_measurement=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),branch=branch,
        initial_C=baseline['initial_C'],final_C=available('C:/')['available_to_caller'],initial_Z=baseline['initial_Z'],final_Z=available('Z:/')['available_to_caller'],
        backing_volume_details=dict(C=available('C:/'),Z=available('Z:/')),
        aliases='C:/mnt/c and Z:/WSL are not added',
        measured_utc=datetime.now(timezone.utc).isoformat(),
        task_elapsed_seconds=(datetime.now(timezone.utc)-datetime.fromisoformat(baseline['utc'])).total_seconds(),
        retained_logical=sum(r['bytes'] for r in files),retained_allocated=sum(r['allocated'] for r in files),retained_clusters=sum(r['cluster_bytes'] for r in files),
        allocation_scope='Task result directory plus changed/new task source and report; manifest itself excluded. Existing authorities/ancestry are referenced, not duplicated.',
        cleanup='None; the encoding stop, complete failed assembly candidate and derivative diagnosis remain retained.',
        preservation=preservation,files=files,
        original_request=read(V4/'results/wing_continuation_layers2to5/AUTHORIZATION.json'),
        original_diagnostic_generation_authorized=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')['generation_authorized'],
        implementation_authority=dict(commit='28fc3aca116e8e7dd2ca7729e911c1930fd8d4f4',identity='7b86ab69db9d90f5a1ed073208577643e5db238853acb62f51935a082b1604f2'))
    durable_json(OUT/'EVIDENCE.json',evidence)
    print(json.dumps({k:evidence[k] for k in ('final_C','final_Z','retained_logical','retained_allocated','retained_clusters','task_elapsed_seconds')},indent=2))
    print('preserved',len(same),'intentional',len(changed),'status',len(unrelated),'files',len(files))
if __name__=='__main__':main()
