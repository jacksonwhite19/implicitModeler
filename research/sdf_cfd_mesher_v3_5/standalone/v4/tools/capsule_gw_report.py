"""Deterministic eight-core comparison; no solver launch or control mutation."""
import json
import subprocess
from pathlib import Path
import numpy as np
import capsule_gw_pair as g
f=g.f
f.ROOT=g.ROOT
import capsule_pressure_diagnostic as p
h=f.h
NAMES=('Baseline','W1','R1','C2','C2-R','G1','GW1-S1','GW1-S2')


def comparison(write=True):
    result={}
    for name in NAMES:
        if name=='Baseline':
            rows=h.read(p.c.OUT/'HISTORY.json')['rows']
            probes=h.read(f.OLD/'BASELINE.json')['wake_probes']
            checkpoints=None
        else:
            folder={'W1':f.OLD,'R1':f.OLD/'R1','C2':g.PREVIOUS/'C2','C2-R':g.PREVIOUS/'C2-R','G1':g.PREVIOUS/'G1'}.get(name,f.ROOT/name)
            m=h.read(folder/'MEASUREMENTS.json');h.verify_record(m)
            assert h.file_hash(folder/'HISTORY.json')==m['history']
            rows=h.read(folder/'HISTORY.json')['rows'];probes=m['wake_probes'];checkpoints=m['checkpoints']
        assert [r['iteration'] for r in rows]==list(range(1,501))
        probes=[dict(r,distance=r['distance'] if 'distance' in r else r['sample_distance']) for r in probes]
        windows=p.windows(rows)
        center=[dict(point=r['point'],station=r['s_D'],distance=r['distance'],U=r['U'],p=r['p'],k=r['k'],omega=r['omega'],nut=r['nut'],speed=float(np.linalg.norm(r['U'])),deficit=33.528-r['U'][2]) for r in probes if r['r_D']==0]
        result[name]=dict(windows=windows,wake=f.prior.sampled_wake(probes),centerline=center,sampling_distance=f.statistics([r['distance'] for r in probes]),last=rows[-1],linear_totals={eq:sum(r['residuals'][eq]['iterations'] for r in rows) for eq in rows[0]['residuals']},bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']],maxiter=[r['iteration'] for r in rows if any(v['iterations']>=5000 for v in r['residuals'].values())],flux_max=max(r['flux']['normalized'] for r in rows),flux_final50=p.c.stats([r['flux']['normalized'] for r in rows[-50:]]),wall_max=max(r['flux']['wall_sumMag'] for r in rows),checkpoints=checkpoints)
    r=h.seal(dict(schema='CapsuleG1PlusW1Comparison/1',family_contract=h.read(f.ROOT/'FAMILY_CONTRACT.json')['identity'],meshes=result,comparison_scope='All 500 outer iterations; initial residuals. Baseline has an authorized restart at200; W1/R1 and new cases are continuous fresh starts. The runs are non-qualifying and may remain unsteady in iteration space. Terminal wake samples are not time means.',sampling='Same physical points. Nearest cell centroid, no interpolated-order claim. Positive axial-deficit disk quadrature radiusD; sparse25 points per station. Distances are retained. Wake width/centroid are diagnostic approximations, not mesh-independent integrals.',scales=dict(q=688.5276551999999,force=3.460917477391561,moment=.5537467963826498),URANS_runs=0,Run_A=False,Run_A2=False))
    if write:h.durable_json(f.ROOT/'COMPARISON.json',r)
    return r


def table(lines,headers,rows):
    lines.extend(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|'])
    for row in rows:lines.append('|'+'|'.join(format(x,'.10g') if isinstance(x,float) else str(x) for x in row)+'|')
    lines.append('')


def report():
    c=h.read(f.ROOT/'COMPARISON.json');h.verify_record(c)
    lines=['# Frozen-prism exact G1+W1 pair diagnostic','',c['comparison_scope'],'',c['sampling'],'','## Sizing contracts','',json.dumps(h.read(f.ROOT/'FAMILY_CONTRACT.json'),sort_keys=True),'']
    decision_path=f.ROOT/'DECISION.json'
    if decision_path.exists():
        decision=h.read(decision_path);h.verify_record(decision)
        lines[2:2]=[decision['classification'],'']+decision['findings']+['','Recommended next task: '+decision['recommended_next_task']['name']+'. '+decision['recommended_next_task']['scope'],'']
    for name in g.NAMES:
        lines.extend([name+': `'+json.dumps(h.read(f.ROOT/name/'CONTRACT.json'),sort_keys=True)+'`',''])
    requested=h.read(f.ROOT/'REQUESTED_SIZING.json')
    lines.extend(['## Requested sizing samples','',requested['sample_definition'],''])
    table(lines,['Mesh','g','h_max','min','median','p95','max'],[[n,v['g'],v['h_max']]+[v['statistics'][k] for k in ('min','median','p95','max')] for n,v in requested['variants'].items()])
    res={n:h.read(f.ROOT/'resolution'/f'{n}.json') for n in NAMES}
    for metric in ('global_h_V','global_cell_edges'):
        lines+=['## '+metric,'']
        table(lines,['Mesh','Samples','min','p01','p05','median','p95','p99','max'],[[n]+[res[n][metric][k] for k in ('n','min','p01','p05','median','p95','p99','max')] for n in NAMES])
    lines+=['## All regional realized sizes','','Edges are counted six per tetra; shared edges have incident-cell weighting. Regions overlap. See exact region definitions in resolution records.','']
    table(lines,['Mesh','Region','Tets','hV median','hV p95','hV max','Edge median','Edge p95','Edge max'],[[n,k,v['tetrahedra']]+[v[m].get(q,'N/A: no cell centroids in region') for m in ('h_V','edges') for q in ('median','p95','max')] for n in NAMES for k,v in res[n]['regions'].items()])
    lines+=['## Exact farfield-adjacent cells','']
    table(lines,['Mesh','Side','Tets','hV median','hV p95','hV max','Edge median','Edge p95','Edge max'],[[n,k,v['tetrahedra']]+[v[m][q] for m in ('h_V','edges') for q in ('median','p95','max')] for n in ('Baseline','W1','G1','GW1-S1','GW1-S2') for k,v in res[n]['farfield'].items()])
    lines+=['## Line/tetra intersections across one diameter','']
    table(lines,['Mesh','s/D','X','Y'],[[n,k,v['X'],v['Y']] for n in NAMES for k,v in res[n]['cells_across_D'].items()])
    lines+=['## Neighbor linear-size grading','']
    table(lines,['Mesh','Region','median','p95','p99','max','Worst cell pair','Worst centers'],[[n,k]+[v['ratios'][q] for q in ('median','p95','p99','max')]+[v['worst']['cells'],v['worst']['centers']] for n in NAMES for k,v in res[n]['grading'].items()])
    lines+=['## Geometry-only asymmetry','']
    for n in NAMES:
        r=res[n];lines.extend([n+': `'+json.dumps(dict(sector_counts=[r['sectors'][str(i)]['all']['tetrahedra'] for i in range(8)],wake_sector_counts=[r['sectors'][str(i)]['wake']['tetrahedra'] for i in range(8)],edge_direction_tensor=r['edge_direction_tensor'],wake_edge_direction_tensor=r['wake_edge_direction_tensor'],face_normal_tensor=r['face_normal_tensor'],rotational_samples=r['rotational_samples']),sort_keys=True)+'`',''])
    lines+=['## Complete mesh gates and cost','']
    for n in g.NAMES:
        d=f.ROOT/n;candidate=h.read(d/'CANDIDATE.json');check=h.read(d/'FOUNDATION.json');cert=h.read(d/'CONTINUOUS.json')
        lines.extend([n+' candidate: '+candidate['identity'],'','Quality: `'+json.dumps(check['summary'],sort_keys=True)+'`','','Fidelity: `'+json.dumps(check['fidelity'],sort_keys=True)+'`','','Continuous: `'+json.dumps(cert,sort_keys=True)+'`','','Literal: Mesh OK.','','Generation resources:','```',(d/'generation.resources').read_text(),'```','','CFD resources:','```',(d/'pilot.resources').read_text(),'```',''])
    lines+=['## Numerical controls','','Both new cases use the original PCG/DIC pressure solve (tolerance1e-13, relTol0, maxIter5000); smoothSolver/GaussSeidel U/k/omega (1e-8,0.1,1sweep,5000); SIMPLE nonOrth0, consistent yes, momentumPredictor yes; U/k/omega relaxation0.9/0.5/0.5; bounded linearUpwindV U and bounded upwind k/omega. Physics, turbulence, wall/farfield conditions and decomposition are unchanged. Each fresh case uses original uniform fields, six ranks, fixed500 horizon. No terminal-field mapping.','']
    for n in g.NAMES:
        lines.extend([n+' exact controls: `'+json.dumps(h.read(f.ROOT/n/'CFD_CONTRACT.json'),sort_keys=True)+'`',''])
    lines+=['## Numerical health','']
    for n,v in c['meshes'].items():
        lines.extend([n+': `'+json.dumps({k:v[k] for k in ('linear_totals','bounds','maxiter','flux_max','flux_final50','wall_max','last')},sort_keys=True)+'`',''])
    for window in ('201-500','301-500','401-500','451-500'):
        lines+=['## Late window '+window,'']
        table(lines,['Mesh','Signal','Mean','Median','Std','Range','Slope','Detrended range','Normalized range'],[[n,k]+[s[q] for q in ('mean','median','std','range','slope','detrended_range')]+[s.get('normalized_range','N/A')] for n,v in c['meshes'].items() for k,s in v['windows'][window]['signals'].items()])
        table(lines,['Mesh','Lateral','Mean magnitude','Range','Magnitude slope','First angle deg','Last angle deg','Angle slope deg/iteration'],[[n,k,s['magnitude']['mean'],s['magnitude']['range'],s['magnitude']['slope'],s['first_angle'],s['last_angle'],s['angle']['slope']] for n,v in c['meshes'].items() for k,s in v['windows'][window]['lateral'].items()])
    lines+=['## Terminal wake sampling','','Velocity is m/s; OpenFOAM p is kinematic pressure in m^2/s^2; k is m^2/s^2; omega is 1/s; nut is m^2/s. Force and moment windows are solver-iteration diagnostics, not physical-time statistics. Azimuth is unwrapped separately in each observation window.','']
    table(lines,['Mesh','s/D','Centerline U','p','k','omega','nut','Sampling distance m'],[[n,r['station'],r['U'],r['p'],r['k'],r['omega'],r['nut'],r['distance']] for n,v in c['meshes'].items() for r in v['centerline']])
    table(lines,['Mesh','s/D','Deficit centroid m','RMS width m','Sampled deficit integral','Uz min','Uz median','Lateral velocity median'],[[n,k,w['center'],w['width_rms'],w['sampled_deficit_integral'],w['Uz']['min'],w['Uz']['median'],w['lateral_velocity']['median']] for n,v in c['meshes'].items() for k,w in v['wake'].items()])
    dense=h.read(f.ROOT/'DENSE_WAKE.json');h.verify_record(dense)
    lines.extend(['## Dense common wake quadrature','',dense['method'],''])
    table(lines,['Mesh','s/D','Deficit centroid m','RMS width m','Deficit integral m3/s','Sampling median m','Sampling p95 m','Sampling max m'],[[n,k,w['centroid'],w['width_rms'],w['deficit_integral']]+[w['sampling_distance'][q] for q in ('median','p95','max')] for n,v in dense['variants'].items() for k,w in v['stations'].items()])
    lines+=['## Terminal fields and checkpoint health','']
    for n in g.NAMES:
        lines.extend([n+': `'+json.dumps(c['meshes'][n]['checkpoints'],sort_keys=True)+'`',''])
    lines.extend(['URANS runs = 0; prism changes = 0; Run A = NO; Run A2 = NO. No CFD numerical qualification or spatial convergence claim.',''])
    for file in ('STORAGE_before_cleanup.json','STORAGE_after_cleanup.json','CLEANUP.json'):
        if (f.ROOT/file).exists():lines.extend([file+': `'+json.dumps(h.read(f.ROOT/file),sort_keys=True)+'`',''])
    lines+=['## Verification and resource accounting','']
    for file in ('TESTS_PRIOR.txt','TESTS_CORE.txt','TESTS_FINAL.txt'):
        if (f.ROOT/file).exists():lines.extend([file,'```',(f.ROOT/file).read_text().strip(),'```',''])
    rows=[json.loads(line) for line in (f.ROOT/'RESOURCE_SAMPLES.jsonl').read_text().splitlines()]
    for n in g.NAMES:
        summary={k:max(row['cases'][n][k] for row in rows) for k in ('rss_bytes','rank_hwm_sum_bytes','cpu_seconds','ranks')}
        lines.extend([n+' sampled six-rank resources: `'+json.dumps(summary,sort_keys=True)+'`',''])
    lines.extend(['GNU time CPU includes the MPI job; maximum RSS there is not the sum of rank RSS. The separate observer records the aggregate rank RSS. Both six-rank jobs ran concurrently; wall times are shared-machine costs, not isolated benchmarks.','', 'The one deselected prior test rewrites an old sealed comparison result. Its deterministic comparison mechanism is exercised by the new task-owned replay test instead. V3 was not edited. The 155 unrelated starting Git entries are preserved.',''])
    (f.ROOT/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8',newline='\n')


if __name__=='__main__':
    comparison();report()
