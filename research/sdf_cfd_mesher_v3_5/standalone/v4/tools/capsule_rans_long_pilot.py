"""Read-only long-pilot trajectories, checkpoint localization and stopping study."""
import hashlib,json,re,shutil,subprocess
import numpy as np
import capsule_rans_upwind_pilot as u
h=u.h
OUT=u.OUT.parent/'capsule_rans_long_pilot'
CHECKPOINTS=(20,50,100,150,200)

def labels(path):
    data=path.read_bytes();m=re.search(rb'\n(\d+)\s*\(',data)
    if not m:raise ValueError('MissingLabelList')
    n=int(m[1]);end=m.end()+4*n
    if data[end:end+1]!=b')':raise ValueError('WrongInt32Encoding')
    return np.frombuffer(data[m.end():end],dtype='<i4')

def window(rows,start,end,key,scale):
    a=np.asarray([r[key] for r in rows if start<=r['iteration']<=end])
    if len(a)!=end-start+1:raise ValueError('IncompleteWindow')
    slope=np.polyfit(np.arange(len(a)),a,1)[0]
    return dict(range=np.ptp(a,axis=0).tolist(),normalized_range=(np.ptp(a,axis=0)/scale).tolist(),slope_per_iteration=slope.tolist(),normalized_slope=(slope/scale).tolist())

def analyze():
    contract=h.read(OUT/'CONTRACT.json');h.verify_record(contract)
    for name,d in contract['controls'].items():assert h.file_hash(OUT/'pilot'/name)==d
    text=(OUT/'pilot.log').read_text();rows=u.e.parse_history(text)
    assert len(rows)==200 and [r['iteration'] for r in rows]==list(range(1,201))
    assert u.e.warning_audit(text)
    h.durable_json(OUT/'HISTORY.json',dict(rows=rows))
    ranks=[]
    for i in range(6):
        p=OUT/'pilot'/f'processor{i}';pm=p/'constant/polyMesh'
        boundary=(pm/'boundary').read_text();b=re.search(r'\bwall\s*\{([^}]+)\}',boundary)[1]
        start=int(re.search(r'startFace\s+(\d+)',b)[1]);count=int(re.search(r'nFaces\s+(\d+)',b)[1])
        owners=labels(pm/'owner');wallcells=set(map(int,owners[start:start+count]))
        ranks.append((p,labels(pm/'cellProcAddressing'),wallcells,u.e.read_internal(p/'0/C',3)))
    checkpoints={}
    for step in range(10,201,10):
        fields={};arrays={}
        for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
            parts=[u.e.read_internal(p/str(step)/name,dim) for p,_,_,_ in ranks]
            a=np.concatenate(parts);assert len(a)==562033
            arrays[name]=parts
            v=np.linalg.norm(a,axis=1) if dim==3 else a[:,0]
            fields[name]=dict(min=float(v.min()),max=float(v.max()),nonfinite=int((~np.isfinite(a)).sum()),negative=int((v<0).sum()),sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest())
        proc=int(np.argmax([x.max() for x in arrays['omega']]));local=int(np.argmax(arrays['omega'][proc][:,0]))
        _,globalids,wallcells,centres=ranks[proc]
        location=dict(processor=proc,local_cell=local,global_cell=int(globalids[local]),center=centres[local].tolist(),wall_adjacent=local in wallcells,
            k=float(arrays['k'][proc][local,0]),nut=float(arrays['nut'][proc][local,0]),U_magnitude=float(np.linalg.norm(arrays['U'][proc][local])))
        # Quantify location of the largest fixed population without creating an acceptance gate.
        top=[]
        for rank,(p,ids,walls,centers) in enumerate(ranks):
            omega=arrays['omega'][rank][:,0]
            ix=np.argsort(omega,kind='stable')[-100:]
            top.extend((float(omega[j]),int(ids[j]),int(j) in walls) for j in ix)
        top=sorted(top,reverse=True)[:100]
        checkpoints[str(step)]=dict(fields=fields,omega_max_location=location,top100_omega_wall_adjacent=sum(x[2] for x in top),wall_distance='Not computed; exact owner incidence establishes adjacency')
    q=.5*u.e.p.prior.RHO*u.e.p.prior.U**2;fs=q*u.e.p.prior.AREA;ms=fs*.16
    windows={str((lo,hi)):dict(force=window(rows,lo,hi,'force',fs),moment=window(rows,lo,hi,'moment',ms)) for lo,hi in [(176,200),(151,200),(101,150),(101,200)]}
    residuals={}
    for eq in ('p','Ux','Uy','Uz','k','omega'):
        a=np.asarray([r['residuals'][eq]['initial'] for r in rows])
        residuals[eq]=dict(samples={str(i):float(a[i-1]) for i in (1,20,50,100,150,200)},minimum=float(a.min()),maximum=float(a.max()),final50_min=float(a[-50:].min()),final50_max=float(a[-50:].max()),final50_slope=float(np.polyfit(np.arange(50),a[-50:],1)[0]))
    bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']]
    flux=np.array([r['flux']['normalized'] for r in rows]);force_pass=all(max(windows[str(w)][key]['normalized_range'])<=1e-5 for w in [(101,150),(151,200)] for key in ['force','moment'])
    study=dict(residual_type='Initial residual per p,Ux,Uy,Uz,k,omega; one solve each per iteration; final linear residual does not measure outer convergence.',
        prior_residual_1e5_all_last100=all(max(r['residuals'][eq]['initial'] for eq in residuals)<=1e-5 for r in rows[-100:]),
        prior_force_moment_1e5_two_windows=force_pass,prior_flux_1e10=bool(flux.max()<=1e-10),prior_wall_1e12=all(r['flux']['wall_sumMag']<=1e-12 for r in rows),
        prior_maximum500='Not tested beyond 200; no conclusion on sufficiency.',
        freeze='Not automatically promoted; inspect trajectory and justify prospectively before any qualification run.')
    result=dict(schema='V4LongPilotMeasurements/1',contract=contract['identity'],checkpoints=checkpoints,residuals=residuals,windows=windows,bounds=bounds,
        q=q,q_hex=q.hex(),force_scale=fs,force_scale_hex=fs.hex(),moment_scale=ms,moment_scale_hex=ms.hex(),
        max_flux=float(flux.max()),final_flux=rows[-1]['flux'],final50_flux_quantiles=np.quantile(flux[-50:],[0,.5,1]).tolist(),max_wall_flux=max(r['flux']['wall_sumMag'] for r in rows),
        linear_totals={eq:sum(r['residuals'][eq]['iterations'] for r in rows) for eq in residuals},maxIter_hits=sum(v['iterations']>=5000 for r in rows for v in r['residuals'].values()),
        first=rows[0],last=rows[-1],stopping_study=study,log_hash=h.file_hash(OUT/'pilot.log'),history_hash=h.file_hash(OUT/'HISTORY.json'))
    h.durable_json(OUT/'MEASUREMENTS.json',h.seal(result))
    print(json.dumps(result,indent=2))

def seal_result():
    m=h.read(OUT/'MEASUREMENTS.json');h.verify_record(m)
    assert m['maxIter_hits']==0
    assert all(f['nonfinite']==0 for cp in m['checkpoints'].values() for f in cp['fields'].values())
    assert len(m['bounds'])==1 and m['bounds'][0]['iteration']==66
    old=h.read(u.OUT/'MEASUREMENTS.json')
    assert all(m['checkpoints']['20']['fields'][n]['sha256']==f['sha256'] for n,f in old['fields'].items())
    contract=h.read(OUT/'CONTRACT.json');h.verify_record(contract)
    mesh=h.read(u.e.p.prior.m.OUT/'AUTHORITY.json')
    for name,digest in mesh['openfoam']['members'].items():
        assert h.file_hash(u.e.p.prior.m.CORE/'foundation/openfoam_case/constant/polyMesh'/name)==digest
        assert h.file_hash(OUT/'pilot/constant/polyMesh'/name)==h.file_hash(u.OUT/'pilot/constant/polyMesh'/name)
    pre=h.read(OUT/'PREFLIGHT.json')
    status=[s for s in subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines() if 'capsule_rans_long_pilot' not in s]
    assert status==pre['status']
    size=lambda d:sum(f.stat().st_size for f in d.rglob('*') if f.is_file())
    r=h.seal(dict(schema='V4CapsuleLongPilotResult/1',classification='V4CapsuleRANSLongPilotHealthyStoppingContractNotYetEstablished',
        starting_head='253108ea1782eeda3d7dd452471f97f7bb8b74a6',mesh=mesh['identity'],healthy_pilot='721ee0793b4191097893f346ad8e8c8c63d82c28967b58345ad257953c479f2e',
        pilot_contract=contract['identity'],measurements=m['identity'],status='NON_QUALIFYING_LONG_STABILIZATION_PILOT',
        controls=contract['controls'],numerical_controls_unchanged=True,mesh_unchanged=True,epoch_renewal=False,
        command='mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot',iterations=200,exit_status=int((OUT/'pilot.exit').read_text()),
        wall_seconds=160.38,user_cpu_seconds=917.84,system_cpu_seconds=21.88,reported_peak_rss_KiB=268756,aggregate_peak_rss='not measured',
        checkMesh='Mesh OK.',warning='AcceptedFoundation13WallGroupNameCollisionWarning',
        healthy_scope='No repeated positivity pathology, no nonfinite written values at all 20 output times, no linear maxIter hits. One isolated negative-k correction at 66 retained as a concern. No convergence claim.',
        omega_classification='A: consistently wall-localized maximum at all written times. Exact boundary-owner incidence, OpenFOAM cell centers and cellProcAddressing used; no approximate nearest-wall classification.',
        stopping_contract=None,
        criterion_assessment=dict(residual='1e-5 initial residual is not achieved except omega; mixed trends remain. No settled residual floor demonstrated.',
            windows='Two disjoint 50-step windows fail force/moment proposal; 25/50/100 views show drift and reversal. 50-step adequacy is not established.',
            force_moment='1e-5 ranges are not met. Do not relax thresholds to admit this trajectory; no plateau/noise floor from which to justify replacements.',
            conservation='Existing 1e-10 normalized phi and 1e-12 wall proposal satisfied throughout. These checks do not establish convergence.',
            maximum_iterations='500 remains unvalidated: this study stops at 200 as required.',
            bounding='A future policy must distinguish isolated bounded correction from recurring pathology; no new magnitude threshold invented.'),
        additional_evidence='A longer bounded unchanged-control trajectory or focused diagnosis of persistent force drift is needed to establish a plateau, residual floor, window duration and sufficient run horizon.',
        checkpoint20_exact_replay=True,deterministic_parsing=True,run_a=0,run_a2=0,
        retained_logical_bytes=size(OUT),pilot_logical_bytes=size(OUT/'pilot'),peak_allocation='not sampled',deleted_bytes=0,
        starting_free_bytes=pre['free_bytes'],ending_free_bytes=shutil.disk_usage(OUT).free,
        tests='61 passed',unrelated_dirty_entries=155,v3_modified=False,
        next_task='Run one bounded unchanged-control continuation study of force/residual stabilization to establish whether a steady plateau exists before selecting stopping thresholds.'))
    h.durable_json(OUT/'RESULT.json',r)
    lines=['# Capsule 200-iteration stabilization pilot',r['classification'],'','No stopping contract issued. No Run A/A2. All checkpoint fields retained; no cleanup deleted evidence.','', '```json',json.dumps(r,indent=2),'```','', '## Full trajectory diagnostics','```json',json.dumps(m,indent=2),'```']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print(json.dumps(dict(identity=r['identity'],result_hash=h.file_hash(OUT/'RESULT.json'),report_hash=h.file_hash(OUT/'REPORT.md'),storage=r['retained_logical_bytes'],free=r['ending_free_bytes'],scales=[m['q'],m['force_scale'],m['moment_scale']],hex=[m['q_hex'],m['force_scale_hex'],m['moment_scale_hex']]),indent=2))

if __name__=='__main__':
    import sys
    (seal_result if len(sys.argv)>1 and sys.argv[1]=='seal' else analyze)()
