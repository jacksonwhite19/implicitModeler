"""Immutable-input continuation of the diagnostic pilot, 201..500 only."""
import hashlib,json,re,shutil,subprocess,sys
import numpy as np
import capsule_rans_long_pilot as l
h=l.h
OUT=l.OUT.parent/'capsule_rans_500_pilot'
HEAD='5911dcfe05a732a85d4a2cf030adea65789a9e53'

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    old=h.read(l.OUT/'RESULT.json');h.verify_record(old);assert old['identity']=='fe44476991c8c32e3806ed2fa0e0413f794fcbe6046a7cf2bb480e3ef626677a'
    healthy=h.read(l.u.OUT/'RESULT.json');h.verify_record(healthy);assert healthy['identity']=='721ee0793b4191097893f346ad8e8c8c63d82c28967b58345ad257953c479f2e'
    m=l.u.e.p.prior.m;a=h.read(m.OUT/'AUTHORITY.json');m.validate(a,h.read(m.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    access=h.read(m.OUT/'ACCESS.json')
    with m.Store(access['directory']).open(access['root']):pass
    measurements=h.read(l.OUT/'MEASUREMENTS.json');h.verify_record(measurements)
    verified={}
    for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
        arrays=np.concatenate([l.u.e.read_internal(l.OUT/'pilot'/f'processor{i}'/'200'/name,dim) for i in range(6)])
        digest=hashlib.sha256(arrays.astype('<f8').tobytes()).hexdigest()
        assert digest==measurements['checkpoints']['200']['fields'][name]['sha256'];verified[name]=digest
    status=[s for s in subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines() if 'capsule_rans_500_pilot' not in s];assert len(status)==155
    OUT.mkdir(exist_ok=True)
    h.durable_json(OUT/'PREFLIGHT.json',dict(status=status,free_bytes=shutil.disk_usage(OUT).free,epoch_renewal=False,verified_200=verified))
    l.u.e.p.prior.write(OUT/'.gitignore','pilot/\n')
    case=OUT/'pilot';assert not case.exists()
    shutil.copytree(l.OUT/'controls',case)
    shutil.copytree(l.OUT/'pilot/constant/polyMesh',case/'constant/polyMesh')
    for i in range(6):
        for n in ('constant','0','200'):shutil.copytree(l.OUT/'pilot'/f'processor{i}'/n,case/f'processor{i}'/n)
    raw={str(f.relative_to(case)):h.file_hash(f) for f in case.glob('processor*/200/**/*') if f.is_file()}
    for n,d in raw.items():assert h.file_hash(l.OUT/'pilot'/n)==d
    control=case/'system/controlDict';text=control.read_text();text=text.replace('startTime 0;','startTime 200;').replace('endTime 200;','endTime 500;');l.u.e.p.prior.write(control,text)
    oldcontrols=h.read(l.OUT/'CONTRACT.json')['controls']
    for n,d in oldcontrols.items():
        if n!='system/controlDict':assert h.file_hash(case/n)==d
        l.u.e.p.prior.write(OUT/'controls'/n,(case/n).read_text())
    record=h.seal(dict(schema='V4Capsule200to500Continuation/1',status='NON_QUALIFYING',start=200,end=500,additional_iterations=300,
        mesh=a['identity'],parent_result=old['identity'],verified_200=verified,restart_file_hashes=raw,
        controls={n:h.file_hash(case/n) for n in oldcontrols},only_changes=['startTime 0 -> 200','endTime 200 -> 500'],
        continuity='All six processor iteration-200 files including phi and uniform metadata copied byte-exact; fixed decomposition copied; restart from startTime 200, not iteration 0.',hardlinks=0))
    h.durable_json(OUT/'CONTRACT.json',record)
    linux='/mnt/c/'+str(OUT.resolve())[3:].replace('\\','/')
    script=f'''source /opt/openfoam13/etc/bashrc
set -e
cd {linux}
sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun > EXECUTABLE.txt
set +e
/usr/bin/time -v -o continuation.resources mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot > continuation.log 2>&1
code=$?
echo "$code" > continuation.exit
exit "$code"
'''
    l.u.e.p.prior.write(OUT/'run.sh',script)
    print(record['identity'])

def stats(a):
    a=np.asarray(a);return dict(min=np.min(a,axis=0).tolist(),max=np.max(a,axis=0).tolist(),mean=np.mean(a,axis=0).tolist(),median=np.median(a,axis=0).tolist(),range=np.ptp(a,axis=0).tolist(),slope=np.polyfit(np.arange(len(a)),a,1)[0].tolist())

def analyze():
    contract=h.read(OUT/'CONTRACT.json');h.verify_record(contract)
    for n,d in contract['controls'].items():assert h.file_hash(OUT/'pilot'/n)==d
    for n,d in contract['restart_file_hashes'].items():assert h.file_hash(OUT/'pilot'/n)==d
    oldrows=h.read(l.OUT/'HISTORY.json')['rows'];text=(OUT/'continuation.log').read_text();newrows=l.u.e.parse_history(text)
    assert [r['iteration'] for r in newrows]==list(range(201,501));assert l.u.e.warning_audit(text)
    rows=oldrows+newrows;assert [r['iteration'] for r in rows]==list(range(1,501))
    h.durable_json(OUT/'HISTORY.json',dict(rows=rows))
    ranks=[]
    for i in range(6):
        p=OUT/'pilot'/f'processor{i}';pm=p/'constant/polyMesh';b=re.search(r'\bwall\s*\{([^}]+)\}',(pm/'boundary').read_text())[1]
        start=int(re.search(r'startFace\s+(\d+)',b)[1]);count=int(re.search(r'nFaces\s+(\d+)',b)[1]);owners=l.labels(pm/'owner')
        ranks.append((p,l.labels(pm/'cellProcAddressing'),set(map(int,owners[start:start+count])),l.u.e.read_internal(p/'0/C',3)))
    checkpoints={}
    for step in range(210,501,10):
        arrays={};fields={}
        for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
            parts=[l.u.e.read_internal(p/str(step)/name,dim) for p,_,_,_ in ranks];a=np.concatenate(parts);assert len(a)==562033
            arrays[name]=parts;v=np.linalg.norm(a,axis=1) if dim==3 else a[:,0]
            fields[name]=dict(min=float(v.min()),max=float(v.max()),negative=int((v<0).sum()),nonfinite=int((~np.isfinite(a)).sum()),sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest())
        proc=int(np.argmax([a.max() for a in arrays['omega']]));j=int(np.argmax(arrays['omega'][proc]));_,ids,wall,c=ranks[proc]
        top=[]
        for rank,(_,gid,w,_) in enumerate(ranks):
            omega=arrays['omega'][rank][:,0];ix=np.argsort(omega,kind='stable')[-100:];top.extend((float(omega[k]),int(gid[k]),int(k) in w) for k in ix)
        checkpoints[str(step)]=dict(fields=fields,omega_location=dict(global_cell=int(ids[j]),processor=proc,local_cell=j,center=c[j].tolist(),wall_adjacent=j in wall,k=float(arrays['k'][proc][j,0]),nut=float(arrays['nut'][proc][j,0]),U=float(np.linalg.norm(arrays['U'][proc][j]))),top100_wall_adjacent=sum(v[2] for v in sorted(top,reverse=True)[:100]))
    residuals={eq:dict(samples={str(i):rows[i-1]['residuals'][eq]['initial'] for i in (200,250,300,350,400,450,500)},windows={f'{lo}-{hi}':stats([r['residuals'][eq]['initial'] for r in rows[lo-1:hi]]) for lo,hi in [(451,500),(401,450),(351,400),(401,500),(351,500)]}) for eq in rows[0]['residuals']}
    windows={f'{lo}-{hi}':{key:dict(dimensional=stats([r[key] for r in rows[lo-1:hi]]),normalized=stats(np.asarray([r[key] for r in rows[lo-1:hi]])/scale)) for key,scale in [('force',3.460917477391561),('moment',.5537467963826498)]} for lo,hi in [(451,500),(401,450),(351,400),(401,500),(301,500)]}
    result=dict(schema='V4ContinuationMeasurements/1',contract=contract['identity'],checkpoints=checkpoints,residuals=residuals,windows=windows,
        bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']],
        max_flux_all=max(r['flux']['normalized'] for r in rows),max_flux_new=max(r['flux']['normalized'] for r in newrows),final50_flux=stats([r['flux']['normalized'] for r in rows[-50:]]),max_wall_flux=max(r['flux']['wall_sumMag'] for r in rows),last=rows[-1],
        force_samples={str(i):dict(force=rows[i-1]['force'],moment=rows[i-1]['moment']) for i in (200,250,300,350,400,450,500)},
        linear_totals={eq:sum(r['residuals'][eq]['iterations'] for r in newrows) for eq in rows[0]['residuals']},maxIter_hits=sum(v['iterations']>=5000 for r in newrows for v in r['residuals'].values()),
        log_hash=h.file_hash(OUT/'continuation.log'),history_hash=h.file_hash(OUT/'HISTORY.json'))
    h.durable_json(OUT/'MEASUREMENTS.json',h.seal(result));print(json.dumps(result,indent=2))

def finalize():
    m=h.read(OUT/'MEASUREMENTS.json');h.verify_record(m)
    assert m['maxIter_hits']==0 and all(b['iteration']<=200 for b in m['bounds'])
    assert all(f['nonfinite']==0 for cp in m['checkpoints'].values() for f in cp['fields'].values())
    assert all(cp['omega_location']['wall_adjacent'] for cp in m['checkpoints'].values())
    contract=h.read(OUT/'CONTRACT.json');h.verify_record(contract)
    mesh=h.read(l.u.e.p.prior.m.OUT/'AUTHORITY.json')
    for n,d in mesh['openfoam']['members'].items():
        assert h.file_hash(l.u.e.p.prior.m.CORE/'foundation/openfoam_case/constant/polyMesh'/n)==d
        assert h.file_hash(OUT/'pilot/constant/polyMesh'/n)==h.file_hash(l.OUT/'pilot/constant/polyMesh'/n)
    pre=h.read(OUT/'PREFLIGHT.json')
    status=[s for s in subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines() if 'capsule_rans_500_pilot' not in s];assert status==pre['status']
    size=lambda d:sum(f.stat().st_size for f in d.rglob('*') if f.is_file())
    r=h.seal(dict(schema='V4Capsule500PilotResult/1',classification='V4CapsuleRANS500IterationPilotHealthyNoSteadyPlateau',
        starting_head=HEAD,mesh=mesh['identity'],parent_result=contract['parent_result'],contract=contract['identity'],measurements=m['identity'],
        status='NON_QUALIFYING',continued_from_200=True,restart_hashes_exact=True,controls_unchanged=True,mesh_unchanged=True,
        command='mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot',start=200,end=500,additional_iterations=300,total_history=500,
        exit_status=int((OUT/'continuation.exit').read_text()),wall_seconds=222.24,user_cpu_seconds=1267.42,system_cpu_seconds=32.79,reported_peak_rss_KiB=258440,aggregate_peak_rss='not measured',
        foundation_build='13-cde978a97c93',executable_sha256='f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e',
        reason='Lateral force and bending-moment window means drift and reverse; final-50 residual slopes are positive for every equation. Conservation and localized bounded omega do not establish a steady plateau.',
        physical_interpretation='No zero-side-force gate imposed. Evidence does not distinguish SIMPLE iteration dynamics, numerical-control limitations or physically unsteady RANS behavior. No mesh defect established.',
        stopping_contract=None,new_thresholds=0,run_a=0,run_a2=0,extension_beyond500=False,
        residual_semantics='Initial residual for each p,Ux,Uy,Uz,k,omega solve; exactly one pressure solve per outer iteration.',
        reference_scales=dict(q=688.5276551999999,force=3.460917477391561,moment=.5537467963826498),
        terminal_field_hashes={n:f['sha256'] for n,f in m['checkpoints']['500']['fields'].items()},
        terminal_file_hashes={str(f.relative_to(OUT/'pilot')):h.file_hash(f) for f in sorted((OUT/'pilot').glob('processor*/500/*')) if f.is_file()},
        tests='68 passed',epoch_renewal=False,unrelated_dirty_entries=155,v3_modified=False,
        retained_logical_bytes=size(OUT),continuation_case_bytes=size(OUT/'pilot'),starting_free_bytes=pre['free_bytes'],ending_free_bytes=shutil.disk_usage(OUT).free,deleted_bytes=0,peak_allocation='not sampled',
        next_task='Run a bounded steady-SIMPLE iteration-dynamics diagnostic of the lateral-force and bending-moment drift, with one controlled comparison to separate numerical convergence behavior from evidence of physical unsteadiness.'))
    h.durable_json(OUT/'RESULT.json',r)
    (OUT/'REPORT.md').write_text('# Capsule continuation to 500\n\n'+r['classification']+'\n\nNo stopping contract, no A/A, no extension or tuning.\n\n```json\n'+json.dumps(r,indent=2)+'\n```\n\n## Complete measured windows and checkpoints\n```json\n'+json.dumps(m,indent=2)+'\n```\n',encoding='utf8')
    print(json.dumps(dict(identity=r['identity'],result_hash=h.file_hash(OUT/'RESULT.json'),report_hash=h.file_hash(OUT/'REPORT.md'),storage=r['retained_logical_bytes'],case=r['continuation_case_bytes'],free_before=r['starting_free_bytes'],free_after=r['ending_free_bytes']),indent=2))

if __name__=='__main__':{'prepare':prepare,'analyze':analyze,'finalize':finalize}[sys.argv[1]]()
