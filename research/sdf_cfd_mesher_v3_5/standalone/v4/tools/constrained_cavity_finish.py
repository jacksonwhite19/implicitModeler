"""Seal evidence, remove byte-identical task scratch only, and record resource use."""
import json,time,shutil,subprocess
from pathlib import Path
from constrained_cavity_audit import OUT,V4,ROOT,digest
from fixed_core_study import save
from fixed_core_cleanup import remove_duplicate

def run():
    files=[p for p in OUT.rglob('*') if p.is_file()];inventory=[dict(path=str(p.relative_to(OUT)).replace('\\','/'),bytes=p.stat().st_size,sha256=digest(p)) for p in files]
    save(OUT/'EVIDENCE_BEFORE_CLEANUP.json',dict(artifacts=inventory,policy='Retain all unique evidence and input authorities; delete only byte-identical task raw-mesher exports after a replacement is hashed'))
    groups={}
    for r in inventory:groups.setdefault(r['sha256'],[]).append(OUT/r['path'])
    deleted=[]
    for hs,paths in groups.items():
        if len(paths)<2:continue
        # Keep at least one complete identical file; never prune scientific
        # inputs, accepted cores, replay arrays, or prior sealed authorities.
        keep=sorted(paths,key=lambda p:('_RAW' in p.stem,str(p)))[0]
        for p in paths:
            if p==keep or not p.stem.endswith('_RAW'):continue
            row=remove_duplicate(OUT,p,keep,OUT/'CLEANUP.json');deleted.append(row)
    if not deleted:save(OUT/'CLEANUP.json',[])
    processes=[dict(path=str(p.relative_to(OUT)),**json.loads(p.read_text())) for p in OUT.rglob('*.process.json')]
    native=[];position=[]
    for p in OUT.rglob('RESULT.json'):
        r=json.loads(p.read_text())
        if 'generate_seconds' in r:native.append(dict(path=str(p.relative_to(OUT)),**{k:r[k] for k in ('seconds','generate_seconds','netgen_seconds','peak_rss_bytes') if k in r}))
        if 'evaluations' in r:position.append(dict(path=str(p.relative_to(OUT)),**{k:r[k] for k in ('seconds','evaluations','variables','iterations','peak_rss_bytes')}))
    current=json.loads((OUT/'accepted_local/RESULT.json').read_text())
    performance=dict(processes=processes,native=native,point_solvers=position,supervised_process_wall_seconds=sum(r['seconds'] for r in processes),max_supervised_process_seconds=max(r['seconds'] for r in processes),process_failures=sum(r['exit_code']!=0 for r in processes),process_timeouts=sum(r['exit_code']==124 for r in processes),max_native_operation_seconds=max(r.get(k,0) for r in native for k in ('generate_seconds','netgen_seconds')),point_solver_seconds=sum(r['seconds'] for r in position),max_point_solver_seconds=max(r['seconds'] for r in position),measured_peak_rss_bytes=max([current['peak_rss_bytes']]+[r.get('peak_rss_bytes',0) for r in native+position]),crashed_native_peak_rss='Not measured',all_inclusive_active_scientific_seconds='Not measured')
    save(OUT/'PERFORMANCE.json',performance)
    retained=[(p.stat().st_size,str(p.relative_to(OUT))) for p in OUT.rglob('*') if p.is_file()]
    storage=dict(ending_free_bytes=shutil.disk_usage(ROOT).free,retained_task_bytes=sum(n for n,_ in retained),task_created_bytes_gross='Not measured',physical_peak_disk_bytes='Not measured',cleanup_bytes_deleted=sum(r['bytes'] for r in deleted),cleanup_file_count=len(deleted),crash_dump_bytes_deleted=0,retained_prior_crash_dump_bytes=1323872256,largest_retained=sorted(retained,reverse=True)[:20])
    save(OUT/'STORAGE_END.json',storage);print(json.dumps(dict(performance={k:v for k,v in performance.items() if k not in ('processes','native','point_solvers')},storage=storage)),flush=True)

if __name__=='__main__':run()
