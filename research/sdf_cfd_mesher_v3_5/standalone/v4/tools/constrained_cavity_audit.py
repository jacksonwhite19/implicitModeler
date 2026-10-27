"""Read-only input/disk inventory for the constrained cavity study."""
import os, json, hashlib, shutil, subprocess, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
V4=ROOT/'research/sdf_cfd_mesher_v4'
OUT=V4/'results/constrained_core_cavity_steiner'

def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()

def save(p,x):p.write_text(json.dumps(x,indent=2)+'\n')

def run():
    OUT.mkdir(exist_ok=False)
    start=time.perf_counter(); total=0; results=0; top=[]; errors=[]; excluded=[]
    free=shutil.disk_usage(ROOT).free
    for d,ds,fs in os.walk(ROOT,followlinks=False):
        for name in list(ds):
            p=Path(d)/name
            if p.is_symlink() or p.is_junction():ds.remove(name);excluded.append(str(p))
        for name in fs:
            p=Path(d)/name
            try:n=p.stat().st_size
            except OSError as e:errors.append(str(e));continue
            total+=n
            if p.is_relative_to(V4/'results'):results+=n;top.append((n,str(p.relative_to(ROOT))))
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'],cwd=ROOT)
    (OUT/'START_STATUS.bin').write_bytes(status)
    audits=[]
    for rel in ('terminal_capacity_aware_remesh_acceptance','bent_terminal_shell_fixed_interface_core'):
        base=V4/'results'/rel;a=json.loads((base/'AUTHORITY.json').read_text());bad=[]
        for r in a['artifacts']:
            p=base/r['path']
            if not p.exists() or digest(p)!=r['sha256'] or p.stat().st_size!=r['bytes']:bad.append(r['path'])
        audits.append(dict(root=rel,authority_sha256=digest(base/'AUTHORITY.json'),report_sha256=digest(base/'report.md'),checked=len(a['artifacts']),mismatches=bad))
    dumps=[]
    prior=json.loads((V4/'results/bent_terminal_shell_fixed_interface_core/NATIVE_FAILURES.json').read_text())
    for r in prior['crash_dumps']:
        p=Path(r['path']);dumps.append(dict(path=str(p),exists=p.exists(),bytes=p.stat().st_size if p.exists() else None,sha256=digest(p) if p.exists() else None,previous_sha256=r['sha256']))
    result=dict(starting_head=head,starting_free_bytes=free,repository_logical_bytes=total,result_tree_logical_bytes=results,largest_20_v4_artifacts=sorted(top,reverse=True)[:20],excluded_reparse_paths=excluded,read_errors=errors,authorities=audits,crash_dumps=dumps,precleanup_bytes=0,task_scratch_before=[],seconds=time.perf_counter()-start)
    save(OUT/'STORAGE_INPUT_AUDIT.json',result)
    print(json.dumps({**{k:v for k,v in result.items() if k!='read_errors'},'read_error_count':len(errors)}),flush=True)
    assert not any(a['mismatches'] for a in audits)

if __name__=='__main__':run()
