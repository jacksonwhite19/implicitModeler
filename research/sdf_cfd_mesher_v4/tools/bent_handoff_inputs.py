"""Authority and storage checks for a read-only shell/handoff investigation."""
import os,json,time,shutil,subprocess
from pathlib import Path
from constrained_cavity_audit import ROOT,V4,digest
from determinant_component_audit import verify
from fixed_core_study import save

OUT=V4/'results/bent_prism_shell_core_handoff'
SHELL=V4/'results/terminal_capacity_aware_remesh_acceptance'
INTERFACE=SHELL/'interface'
CORE=V4/'results/constrained_core_cavity_steiner/accepted_local'
FALLBACK=V4/'results/bent_passage_large_case_fallback'

def audit():
    OUT.mkdir(exist_ok=False);tick=time.perf_counter();free=shutil.disk_usage(ROOT).free
    (OUT/'START_STATUS.bin').write_bytes(subprocess.check_output(['git','status','--porcelain=v1','-z'],cwd=ROOT))
    sizes=[];errors=[]
    for d,ds,fs in os.walk(V4/'results',followlinks=False):
        for f in fs:
            p=Path(d)/f
            try:sizes.append((p.stat().st_size,str(p)))
            except OSError as e:errors.append(str(e))
    authorities=[verify(SHELL),verify(CORE.parent),verify(V4/'results/bent_core_determinant_components'),verify(FALLBACK)]
    assert not any(a['mismatches'] for a in authorities)
    relevant=sorted([(p.stat().st_size,str(p)) for root in (SHELL,FALLBACK) for p in root.rglob('*') if p.is_file()],reverse=True)[:12]
    r=dict(starting_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),starting_free_bytes=free,v4_result_bytes=sum(n for n,p in sizes),largest_relevant_shell=relevant,largest_v4=sorted(sizes,reverse=True)[:20],read_errors=errors,authority_checks=authorities,precleanup_bytes=0,task_scratch_before=[],scratch_requirement='Not measured',crash_dumps='Untouched; no pressure requires action',seconds=time.perf_counter()-tick)
    save(OUT/'INPUT_AUDIT.json',r);print(json.dumps(r),flush=True)

if __name__=='__main__':audit()
