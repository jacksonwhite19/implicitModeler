"""Read-only authority and storage inventory for determinant-component work."""
import os, json, shutil, subprocess, time
from pathlib import Path
from constrained_cavity_audit import ROOT,V4,digest,save

OUT=V4/'results/bent_core_determinant_components'
PRIOR=V4/'results/constrained_core_cavity_steiner'
SOURCE=PRIOR/'accepted_local'
INTERFACE=V4/'results/terminal_capacity_aware_remesh_acceptance/interface'

def verify(base):
    a=json.loads((base/'AUTHORITY.json').read_text()); bad=[]
    for r in a['artifacts']:
        p=base/r['path']
        if not p.exists() or p.stat().st_size!=r['bytes'] or digest(p)!=r['sha256']:bad.append(r['path'])
    return dict(root=str(base),checked=len(a['artifacts']),mismatches=bad,report_sha256=digest(base/'report.md'),authority_sha256=digest(base/'AUTHORITY.json'))

def run():
    OUT.mkdir(exist_ok=False);tick=time.perf_counter();free=shutil.disk_usage(ROOT).free
    (OUT/'START_STATUS.bin').write_bytes(subprocess.check_output(['git','status','--porcelain=v1','-z'],cwd=ROOT))
    total=results=0;top=[];errors=[];excluded=[]
    for d,ds,fs in os.walk(ROOT,followlinks=False):
        for name in list(ds):
            p=Path(d)/name
            if p.is_symlink() or p.is_junction():ds.remove(name);excluded.append(str(p))
        for name in fs:
            p=Path(d)/name
            try:n=p.stat().st_size
            except OSError as e:errors.append(str(e));continue
            total+=n
            if p.is_relative_to(V4/'results'):results+=n;top.append((n,str(p)))
    checks=[verify(PRIOR),verify(INTERFACE.parent)]
    dumps=[]
    for r in json.loads((PRIOR/'STORAGE_INPUT_AUDIT.json').read_text())['crash_dumps']:
        p=Path(r['path']);dumps.append(dict(path=str(p),bytes=p.stat().st_size,sha256=digest(p),retained=True,reason='Unique native heap evidence was not extracted; compact reproducer and signal notes do not replace it'))
    relevant=sorted([(p.stat().st_size,str(p)) for p in PRIOR.rglob('*') if p.is_file()],reverse=True)[:12]
    r=dict(starting_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),starting_free_bytes=free,repository_logical_bytes=total,result_tree_logical_bytes=results,largest_20_v4=sorted(top,reverse=True)[:20],largest_relevant_core=relevant,read_errors=errors,excluded=excluded,authorities=checks,crash_dumps=dumps,crash_dump_bytes=sum(r['bytes'] for r in dumps),precleanup_bytes=0,scratch_estimate='Not measured',seconds=time.perf_counter()-tick)
    save(OUT/'AUDIT.json',r);save(OUT/'CRASH_DISPOSITION.json',dict(dumps=dumps,deleted_bytes=0,compact_evidence=str(PRIOR/'CRASH_NOTES.json'),reproducer=str(PRIOR/'reproducer'),adapter=str(V4/'tools/constrained_netgen_adapter.py')))
    print(json.dumps({**{k:v for k,v in r.items() if k!='read_errors'},'read_error_count':len(errors)}),flush=True)
    assert not any(z['mismatches'] for z in checks)

if __name__=='__main__':run()
