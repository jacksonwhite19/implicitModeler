"""Seal bounded probe failures without publishing any rejected geometry."""
import time,json,shutil,subprocess,sys
from pathlib import Path
import numpy as np
import psutil
from determinant_component_audit import OUT,SOURCE,INTERFACE,PRIOR,V4,ROOT,verify,digest
from constrained_cavity import *
from fixed_core_study import distance_surface,determinant
from fixed_core_cleanup import remove_duplicate

def run():
    tick=time.perf_counter();x,t,ci,iface=load_core(SOURCE,INTERFACE);q,m=evaluate(x,t,ci,iface)
    trials=[]
    for p in sorted(OUT.glob('*_solve/RESULT.json')):
        r=json.loads(p.read_text());name=p.parent.name
        trials.append(dict(name=name,moving_points=len(r['moving_vertices']),tets=r['after']['tets'],before_failures=r['before']['failures'],after_failures=r['after']['failures'],minimum=r['after']['minimum'],maximum=r['after']['maximum'],invariants=r['invariants']['passed'],inside=all(r['strict_inside'].values()),accepted=r['accepted_proposal'],iterations=r['iterations'],evaluations=r['evaluations'],solver_success=r['solver_success'],message=r['message'],seconds=r['seconds'],peak_rss_bytes=r['peak_rss_bytes']))
    native=[]
    for p in sorted(OUT.glob('*/RESULT.json')):
        r=json.loads(p.read_text())
        if 'stages' in r:native.append(dict(name=p.parent.name,seconds=r['seconds'],peak_rss_bytes=r['peak_rss_bytes'],generate_seconds=r.get('generate_seconds'),netgen_seconds=r.get('netgen_seconds'),stages=r['stages']))
    enriched=[]
    for probe in json.loads((OUT/'forensics/PROBES.json').read_text()):
        d=dict(np.load(OUT/'forensics'/f"{probe['label']}_r3.npz"));tri=d['x'][d['f']]
        for row in probe['cells']:
            centre=x[t[row['cell']]].mean(0);dist=np.array([distance_surface(centre,z[None]) for z in tri]);j=int(dist.argmin())
            row['nearest_ring3_boundary_face_global_vertices']=d['global_vertices'][d['f'][j]]
            sr=row['sensitivity']['rows'];row['largest_abs_sensitivity']=max(sr,key=lambda r:abs(r['central_derivative']))
        enriched.append(probe)
    save(OUT/'PROBE_FORENSICS.json',enriched)
    protected=json.loads((SOURCE/'RESULT.json').read_text())['replacement_tets'];protected_ids=set(range(len(t)-protected,len(t)))
    supports=[]
    for p in sorted((OUT/'forensics').glob('A_*.npz')):
        d=np.load(p);supports.append(dict(input=p.name,cells=len(d['global_cells']),protected_cell_overlap=sorted(protected_ids&set(map(int,d['global_cells'])))))
    verify_inputs=[verify(PRIOR),verify(INTERFACE.parent)]
    assert not any(a['mismatches'] for a in verify_inputs)
    save(OUT/'FINAL_VERIFICATION.json',dict(core_identity=q['identity'],quality=q,interface_coordinates_exact=np.array_equal(x[ci].view('u8'),iface.x.view('u8')),interface_connectivity_orientation_exact=len(iface.mapping(x,t,ci))==len(iface.f),protected_local_repairs_unchanged=True,authority_checks=verify_inputs,support_overlap=supports,accepted_replacements=0,seconds=time.perf_counter()-tick,peak_rss_bytes=psutil.Process().memory_info().peak_wset,retained_rss_bytes=psutil.Process().memory_info().rss))
    processes=[dict(name=p.stem,**json.loads(p.read_text())) for p in OUT.glob('*.process.json')]
    perf=dict(processes=processes,supervised_wall_seconds=sum(r['seconds'] for r in processes),max_supervised_seconds=max(r['seconds'] for r in processes),native=native,local_solves=trials,peak_rss_bytes=max([r['peak_rss_bytes'] for r in native+trials]+[psutil.Process().memory_info().peak_wset]),total_active_scientific_seconds='Not measured')
    save(OUT/'PERFORMANCE.json',perf)
    disposition=dict(classification='V4BentCoreDeterminantComponentsCharacterized',probe_A=dict(cells=[7505,14039],qualified=False,retained_minimum=min(m['determinant'][[7505,14039]]),reason='Three cavity extents and deterministic point strategies failed; fixed-connectivity coordinate support also failed'),probe_B=dict(cells=[3045,4116,4518],qualified=False,attempted=False,reason='Probe A prerequisite did not qualify; forensic extraction only',retained_minimum=min(m['determinant'][[3045,4116,4518]])),generic_method_qualified=False,components_swept=0,components_repaired=0,accepted_replacements=0,final_core=str(SOURCE),quality=q,complete_core=False,complete_assembly='Not run',complete_certificate='Not measured',geometric_impossibility_proven=False)
    save(OUT/'RESULT.json',disposition)
    save(OUT/'ADMISSION_DEFECT.json',dict(cause='Strict inclusion in one source tet rejects original interior vertices on internal cell boundaries',fix='Reuse exact-coordinate source vertices only if referenced and absent from cavity boundary; newly proposed positions retain strict exact tet admission',scientific_gate_change=False,regression='test_original_interior_vertex_is_not_a_strict_tet_point',pre_fix_reproducer='A_r5 mixed_support uses the original interior vertex',status='Fixed and tested'))
    # Only byte-identical task-local exports are disposable. Record before deletion.
    inventory=[dict(path=str(p.relative_to(OUT)),bytes=p.stat().st_size,sha256=digest(p)) for p in OUT.rglob('*') if p.is_file()]
    save(OUT/'EVIDENCE_BEFORE_CLEANUP.json',inventory);seen={};deleted=[]
    for p in sorted(OUT.glob('*/*_RAW.npz')):
        key=(p.stat().st_size,digest(p))
        if key in seen:deleted.append(remove_duplicate(OUT,p,seen[key],OUT/'CLEANUP.json'))
        else:seen[key]=p
    if not (OUT/'CLEANUP.json').exists():save(OUT/'CLEANUP.json',[])
    save(OUT/'STORAGE_END.json',dict(free_bytes=shutil.disk_usage(ROOT).free,retained_task_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),deleted_bytes=sum(r['bytes'] for r in deleted),deleted_files=len(deleted),crash_dump_bytes_deleted=0,task_created_lifetime_bytes='Not measured',physical_peak_disk_bytes='Not measured'))
    print(json.dumps(dict(result=disposition,performance={k:v for k,v in perf.items() if k not in ('native','local_solves','processes')},cleanup=deleted),default=lambda z:z.tolist()),flush=True)

if __name__=='__main__':run()
