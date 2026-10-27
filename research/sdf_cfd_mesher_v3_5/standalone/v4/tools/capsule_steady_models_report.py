"""Direct, inspectable evidence tables. Does not decide admission or convergence."""
import json
import numpy as np
import capsule_steady_models as s

def table(lines,headers,rows):
    lines.extend(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|'])
    for row in rows:lines.append('|'+'|'.join(format(x,'.12g') if isinstance(x,float) else str(x) for x in row)+'|')
    lines.append('')

def report():
    root=s.ROOT;read=s.h.read;lines=['# Capsule steady mesh and turbulence-model sensitivity','']
    decision=root/'DECISION.json'
    if decision.exists():
        d=read(decision);s.h.verify_record(d);lines+=['```json',json.dumps(d,indent=2),'```','']
    else:lines+=['DRAFT: The staged steady study is in progress. No final mesh selection or model admission is implied by this report.','']
    lines+=['Starting HEAD: '+s.HEAD,'','Prisms and L11 remain exact. No transient, URANS, SAS, DES or LES run is authorized. All new CFD runs are non-qualifying.','']
    family=read(root/'FAMILY_CONTRACT.json');lines+=['## Exact sizing recovery','```json',json.dumps(family,indent=2),'```','']
    requested=read(root/'REQUESTED_SIZING.json')
    table(lines,['Requested field','min','p01','p05','median','p95','p99','max'],[[n]+[v['statistics'][k] for k in ('min','p01','p05','median','p95','p99','max')] for n,v in requested['variants'].items()])
    names=[p.name for p in root.iterdir() if p.is_dir() and (p/'CANDIDATE.json').exists()]
    resolution={n:read(root/'resolution'/f'{n}.json') for n in names}
    for metric in ('global_h_V','global_cell_edges'):
        lines+=['## '+metric,''];table(lines,['Mesh','n','min','p01','p05','median','p95','p99','max'],[[n]+[r[metric][k] for k in ('n','min','p01','p05','median','p95','p99','max')] for n,r in resolution.items()])
    lines+=['## Identical regional definitions','']
    table(lines,['Mesh','Region','Tets','hV median','hV p95','hV max','Edge median','Edge p95','Edge max'],[[n,k,v['tetrahedra']]+[v[m].get(q,'unavailable') for m in ('h_V','edges') for q in ('median','p95','max')] for n,r in resolution.items() for k,v in r['regions'].items()])
    lines+=['## Farfield',''];table(lines,['Mesh','Side','Tets','hV median','hV p95','hV max','Edge median','Edge p95','Edge max'],[[n,k,v['tetrahedra']]+[v[m][q] for m in ('h_V','edges') for q in ('median','p95','max')] for n,r in resolution.items() for k,v in r['farfield'].items()])
    lines+=['## Cells across D',''];table(lines,['Mesh','s/D','X','Y'],[[n,k,v['X'],v['Y']] for n,r in resolution.items() for k,v in r['cells_across_D'].items()])
    lines+=['## Neighbor size ratios',''];table(lines,['Mesh','Region','median','p95','p99','max','Worst pair','Centers'],[[n,k]+[v['ratios'][q] for q in ('median','p95','p99','max')]+[v['worst']['cells'],v['worst']['centers']] for n,r in resolution.items() for k,v in r['grading'].items()])
    for n,r in resolution.items():
        counts=[r['sectors'][str(i)]['wake']['tetrahedra'] for i in range(8)]
        asym=dict(wake_counts=counts,count_CV=float(np.std(counts)/np.mean(counts)),sectors=r['sectors'],edge_tensor=r['edge_direction_tensor'],eigenvalues=np.linalg.eigvalsh(r['edge_direction_tensor']).tolist(),face_normal_tensor=r['face_normal_tensor'],rotational_samples=r['rotational_samples'])
        lines+=['## '+n+' geometry evidence','```json',json.dumps(asym,indent=2),'```']
        for file in ('CANDIDATE.json','CONTINUOUS.json','FOUNDATION.json','CFD_CONTRACT.json'):
            lines +=[file,'```json',json.dumps(read(root/n/file),indent=2),'```']
        for file in ('generation.resources','pilot.resources'):
            if (root/n/file).exists():lines +=[file,'```',(root/n/file).read_text(),'```']
    comparison='FINE_COMPARISON.json' if (root/'FINE_COMPARISON.json').exists() else 'COMPARISON.json'
    if (root/comparison).exists():
        c=read(root/comparison);s.h.verify_record(c)
        for window in ('201-500','301-500','401-500','451-500'):
            lines+=['## Window '+window,'']
            table(lines,['Mesh','Signal','Mean','Median','Std','Range','Slope','Detrended range'],[[n,k]+[v[q] for q in ('mean','median','std','range','slope','detrended_range')] for n,r in c['meshes'].items() for k,v in r['windows'][window]['signals'].items()])
            table(lines,['Mesh','Vector','Mean magnitude','Range','Slope','First azimuth','Last azimuth','Angle slope'],[[n,k,v['magnitude']['mean'],v['magnitude']['range'],v['magnitude']['slope'],v['first_angle'],v['last_angle'],v['angle']['slope']] for n,r in c['meshes'].items() for k,v in r['windows'][window]['lateral'].items()])
        for n in names:
            if n not in c['meshes']:continue
            lines+=['## '+n+' health and terminal fields','```json',json.dumps({k:v for k,v in c['meshes'][n].items() if k!='windows'},indent=2),'```']
    wake='FINE_DENSE_WAKE.json' if (root/'FINE_DENSE_WAKE.json').exists() else 'DENSE_WAKE.json'
    if (root/wake).exists():
        d=read(root/wake);lines+=['## Common dense wake sampling','',d['method'],'']
        table(lines,['Mesh','s/D','Deficit integral','Centroid','RMS width','Distance median','Distance p95','Distance max'],[[n,k,v['deficit_integral'],v['centroid'],v['width_rms']]+[v['sampling_distance'][q] for q in ('median','p95','max')] for n,r in d['variants'].items() for k,v in r['stations'].items()])
    for file in ('STAGE1_DECISION.json','FINE_FAMILY_CONTRACT.json','FINE_REQUESTED_SIZING.json','FINE_REQUESTED_REGIONS.json','FINE_REALIZED_PREFLIGHT.json','MESH_SELECTION.json','MODEL_SOURCE_RECOVERY.json','MODEL_REGISTRY.json','WALL_GUIDANCE.json','PRELIMINARY_WALL_ADMISSION.json','MODEL_ADMISSION.json','SA_PROSPECTIVE_CONTROLS.json','SA_ADMISSION.json','OPTIONAL_REFINEMENT_ESTIMATE.json','INTERIM_EXTREME_GC2F-S2_50.json','INTERIM_SPEED_GC2F-S2_60.json','INTERIM_OMEGA_LOCALIZATION.json','STORAGE_before_cleanup.json','STORAGE_after_cleanup.json','CLEANUP.json'):
        if (root/file).exists():lines+=['## '+file,'```json',json.dumps(read(root/file),indent=2),'```']
    for file in ('SSTLM_ADMISSION.json','SA_FREESTREAM_CONTEXT.json','MODEL_RESOURCE_PREFLIGHT.json','LOAD_DECOMPOSITION.json','MECHANICAL_REPAIRS.json','PERFORMANCE.json','WALL_STRESS_COMPARISON.json'):
        if (root/file).exists():lines+=['## '+file,'```json',json.dumps(read(root/file),indent=2),'```']
    if (root/'MODEL_COMPARISON.json').exists():
        models=read(root/'MODEL_COMPARISON.json');s.h.verify_record(models)
        for window in ('201-500','301-500','401-500','451-500'):
            lines+=['## Steady model window '+window,'']
            table(lines,['Run','Signal','Mean','Median','Std','Range','Slope','Detrended range'],[[n,k]+[v[q] for q in ('mean','median','std','range','slope','detrended_range')] for n,r in models['runs'].items() for k,v in r['windows'][window]['signals'].items()])
            table(lines,['Run','Vector','Mean magnitude','Range','Slope','First azimuth','Last azimuth'],[[n,k,v['magnitude']['mean'],v['magnitude']['range'],v['magnitude']['slope'],v['first_angle'],v['last_angle']] for n,r in models['runs'].items() for k,v in r['windows'][window]['lateral'].items()])
        for n,r in models['runs'].items():
            lines+=['## '+n+' model evidence','```json',json.dumps({k:v for k,v in r.items() if k not in ('windows','loads')},indent=2),'```']
        lines+=['## Model pair comparison','```json',json.dumps(models['pairs'],indent=2),'```']
    for folder in sorted((root/'model_runs').glob('SA-*')):
        for file in ('CFD_CONTRACT.json','INITIAL_AUDIT.json','pilot.resources'):
            if (folder/file).exists():lines+=['## '+folder.name+'/'+file,'```',(folder/file).read_text(),'```']
    for file in sorted(root.glob('TESTS_*.txt')):lines+=[file.name,'```',file.read_text(),'```']
    for n in names:
        if (root/n/'WALL_YPLUS.json').exists():lines+=['## '+n+' solved y+','```json',json.dumps(read(root/n/'WALL_YPLUS.json'),indent=2),'```']
    for n in ('GW1-S1','GW1-S2'):
        p=root/'wall_reference'/n/'WALL_YPLUS.json'
        if p.exists():lines+=['## '+n+' solved y+','```json',json.dumps(read(p),indent=2),'```']
    lines+=['Prism changes = 0. Run A = NO. Run A2 = NO. Production authority replaced = NO. No CFD solution qualification.','']
    (root/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8',newline='\n')

if __name__=='__main__':report()
