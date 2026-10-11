"""One frozen pressure-only diagnostic; never promotes a CFD solution."""
import difflib,hashlib,json,re,shutil,subprocess,sys
import numpy as np
import capsule_motorbike_diagnostic as d
c=d.c;l=c.l;h=d.h
OUT=c.OUT.parent/'capsule_pressure_diagnostic'
HEAD='ec33ba7974f4b3ad56c336386196345cc3fac977'

def pressure(text):
    match=re.search(r'\bp\s*\{[^{}]*\}',text)
    if not match:raise ValueError('MissingPressureBlock')
    return match

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    r=h.read(d.OUT/'RESULT.json');h.verify_record(r);assert r['identity']=='fe735e7d14cc6e083fd54ceda7002be39aeaabf0f1663d978bf256ab8f0b39cb'
    baseline=h.read(c.OUT/'RESULT.json');h.verify_record(baseline)
    m=h.read(c.OUT/'MEASUREMENTS.json');h.verify_record(m);assert baseline['measurements']==m['identity']
    assert h.file_hash(c.OUT/'HISTORY.json')==m['history_hash']
    mesh=l.u.e.p.prior.m;mesh.validate(h.read(mesh.OUT/'AUTHORITY.json'),h.read(mesh.OUT/'CFD_HANDOFF.json'),r['mesh'])
    access=h.read(mesh.OUT/'ACCESS.json')
    with mesh.Store(access['directory']).open(access['root']):pass
    OUT.mkdir(exist_ok=True);case=OUT/'pilot';assert not case.exists()
    free=shutil.disk_usage(OUT).free;assert free>10*1024**3
    controls=h.read(l.OUT/'CONTRACT.json');h.verify_record(controls)
    for n,sha in controls['controls'].items():assert h.file_hash(l.OUT/'controls'/n)==sha
    source=subprocess.check_output(['wsl','cat',d.SOURCE+'/system/fvSolution'])
    (OUT/'motorBike_fvSolution').write_bytes(source)
    original=(l.OUT/'controls/system/fvSolution').read_text();block=pressure(source.decode()).group()
    new=pressure(original);updated=original[:new.start()]+block+original[new.end():]
    assert pressure(updated).group()==block
    assert pressure(updated).re.sub('PRESSURE',updated)==pressure(original).re.sub('PRESSURE',original)
    shutil.copytree(l.OUT/'controls',case);shutil.copytree(l.OUT/'pilot/constant/polyMesh',case/'constant/polyMesh')
    for i in range(6):
        for part in ('constant','0'):shutil.copytree(l.OUT/'pilot'/f'processor{i}'/part,case/f'processor{i}'/part)
    (case/'system/fvSolution').write_text(updated,newline='\n')
    ctl=case/'system/controlDict';ctl.write_text(ctl.read_text().replace('endTime 200;','endTime 500;'),newline='\n')
    for n,sha in controls['controls'].items():
        if n not in ('system/controlDict','system/fvSolution'):assert h.file_hash(case/n)==sha
    for n in ('points','faces','owner','neighbour','boundary'):assert h.file_hash(case/'constant/polyMesh'/n)==h.file_hash(l.OUT/'pilot/constant/polyMesh'/n)
    for n in controls['controls']:
        dest=OUT/'controls'/n;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(case/n,dest)
    inherited={str(p.relative_to(case)).replace('\\','/'):h.file_hash(p) for pattern in ('processor*/0/**/*','processor*/constant/**/*','constant/polyMesh/*') for p in case.glob(pattern) if p.is_file()}
    for n,sha in inherited.items():assert h.file_hash(l.OUT/'pilot'/n)==sha
    record=h.seal(dict(schema='V4PressureDiagnostic/1',status='NON_QUALIFYING_PRESSURE_SOLVER_DIAGNOSTIC',mesh=r['mesh'],baseline=baseline['identity'],previous_diagnostic=r['identity'],horizon=500,start=0,source_path=d.SOURCE+'/system/fvSolution',source_sha256=h.file_hash(OUT/'motorBike_fvSolution'),source_lines=[source.decode()[:pressure(source.decode()).start()].count('\n')+1,source.decode()[:pressure(source.decode()).end()].count('\n')+1],old_pressure=new.group(),new_pressure=block,numerical_mechanism_changes=1,mechanism='pressure linear solver only',administrative_change='endTime 200 -> 500 on original-state template; matches full baseline horizon',controls={n:h.file_hash(case/n) for n in controls['controls']},inherited=inherited,initial={n:controls['controls']['0/'+n] for n in ('U','p','k','omega','nut')},diff=list(difflib.unified_diff(original.splitlines(),updated.splitlines(),fromfile='baseline/fvSolution',tofile='diagnostic/fvSolution',lineterm='')),epoch_renewal=False))
    h.durable_json(OUT/'CONTRACT.json',record)
    h.durable_json(OUT/'PREFLIGHT.json',dict(free_bytes=free,status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines(),baseline_files={str(p.relative_to(c.OUT)):h.file_hash(p) for p in (c.OUT/'HISTORY.json',c.OUT/'RESULT.json',c.OUT/'MEASUREMENTS.json')}))
    (OUT/'.gitignore').write_text('pilot/\n')
    linux='/mnt/c/'+str(OUT.resolve())[3:].replace('\\','/')
    (OUT/'run.sh').write_text(f'''source /opt/openfoam13/etc/bashrc
cd {linux}
sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun > EXECUTABLE.txt
/usr/bin/time -v -o pilot.resources mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot > pilot.log 2>&1
echo $? > pilot.exit
''',newline='\n')
    print(record['identity'])

def windows(rows):
    signals={eq:np.array([r['residuals'][eq]['initial'] for r in rows]) for eq in ('p','Ux','Uy','Uz','k','omega')}
    for key,prefix in [('force','F'),('moment','M')]:
        for j,axis in enumerate('xyz'):signals[prefix+axis]=np.array([r[key][j] for r in rows])
    result={}
    for lo in (201,301,401,451):
        s={k:v[lo-1:] for k,v in signals.items()};out=dict(signals={k:d.signal(v) for k,v in s.items()},cross={a+'/'+b:d.cross(s[a],s[b]) for a,b in [('Fx','My'),('Fy','Mx'),('Fx','Fy'),('Ux','Uy'),('Fx','Ux'),('Fy','Uy')]},lateral={})
        for prefix in ('F','M'):
            angle=np.unwrap(np.arctan2(s[prefix+'y'],s[prefix+'x']))*180/np.pi
            out['lateral'][prefix]=dict(magnitude=d.signal(np.hypot(s[prefix+'x'],s[prefix+'y'])),angle=d.signal(angle),first_angle=float(angle[0]),last_angle=float(angle[-1]))
            for axis in 'xyz':out['signals'][prefix+axis]['normalized_range']=out['signals'][prefix+axis]['range']/(3.460917477391561 if prefix=='F' else .5537467963826498)
        result[f'{lo}-500']=out
    return result

def health(rows,fields):
    if any(f['nonfinite'] for cp in fields.values() for f in cp['fields'].values()):return 'NonfiniteFailure'
    bounds=[(r['iteration'],b) for r in rows for b in r['bounds']]
    if any(b['field']=='omega' for _,b in bounds):return 'OmegaBoundingRequiresReview'
    if len([1 for _,b in bounds if b['field']=='k'])>1:return 'RepeatedKBoundingRequiresReview'
    if any(v['iterations']>=(1000 if eq=='p' else 5000) for r in rows for eq,v in r['residuals'].items()):return 'LinearMaxIterFailure'
    return 'NoRepeatedPositivityOrLinearFailure'

def analyze():
    con=h.read(OUT/'CONTRACT.json');h.verify_record(con)
    for n,sha in {**con['controls'],**con['inherited']}.items():assert h.file_hash(OUT/'pilot'/n)==sha
    text=(OUT/'pilot.log').read_text();rows=l.u.e.parse_history(text);assert [r['iteration'] for r in rows]==list(range(1,501));assert l.u.e.warning_audit(text)
    h.durable_json(OUT/'HISTORY.json',dict(rows=rows))
    ranks=[]
    for i in range(6):
        p=OUT/'pilot'/f'processor{i}';pm=p/'constant/polyMesh';b=re.search(r'\bwall\s*\{([^}]+)\}',(pm/'boundary').read_text())[1]
        start=int(re.search(r'startFace\s+(\d+)',b)[1]);count=int(re.search(r'nFaces\s+(\d+)',b)[1])
        ranks.append((p,l.labels(pm/'cellProcAddressing'),set(map(int,l.labels(pm/'owner')[start:start+count])),l.u.e.read_internal(p/'0/C',3)))
    checkpoints={}
    for step in range(10,501,10):
        arrays={};fields={}
        for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
            parts=[l.u.e.read_internal(p/str(step)/name,dim) for p,_,_,_ in ranks];a=np.concatenate(parts);assert len(a)==562033;arrays[name]=parts
            v=np.linalg.norm(a,axis=1) if dim==3 else a[:,0]
            fields[name]=dict(min=float(v.min()),max=float(v.max()),negative=int((v<0).sum()),nonfinite=int((~np.isfinite(a)).sum()),sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest())
        proc=int(np.argmax([a.max() for a in arrays['omega']]));j=int(np.argmax(arrays['omega'][proc]));_,ids,wall,centers=ranks[proc]
        checkpoints[str(step)]=dict(fields=fields,omega_location=dict(global_cell=int(ids[j]),center=centers[j].tolist(),wall_adjacent=j in wall,k=float(arrays['k'][proc][j,0]),nut=float(arrays['nut'][proc][j,0]),U=float(np.linalg.norm(arrays['U'][proc][j]))))
    old=h.read(c.OUT/'HISTORY.json')['rows']
    def summary(rr):
        p=np.array([r['residuals']['p']['iterations'] for r in rr]);return dict(pressure_iterations=dict(total=int(p.sum()),mean=float(p.mean()),median=float(np.median(p)),max=int(p.max())),flux_max=max(r['flux']['normalized'] for r in rr),flux_final=rr[-1]['flux'],flux_final50=c.stats([r['flux']['normalized'] for r in rr[-50:]]),wall_max=max(r['flux']['wall_sumMag'] for r in rr),bounds=[dict(iteration=r['iteration'],**b) for r in rr for b in r['bounds']],linear_totals={eq:sum(r['residuals'][eq]['iterations'] for r in rr) for eq in rr[0]['residuals']},last=rr[-1])
    result=h.seal(dict(schema='V4PressureComparison/1',contract=con['identity'],baseline=dict(summary=summary(old),windows=windows(old)),diagnostic=dict(summary=summary(rows),windows=windows(rows)),checkpoints=checkpoints,health=health(rows,checkpoints),history_hash=h.file_hash(OUT/'HISTORY.json'),log_hash=h.file_hash(OUT/'pilot.log')))
    h.durable_json(OUT/'MEASUREMENTS.json',result);print(result['identity'],result['health'])

def report():
    m=h.read(OUT/'MEASUREMENTS.json');con=h.read(OUT/'CONTRACT.json')
    lines=['# Capsule pressure-only diagnostic','', 'NON_QUALIFYING. No stopping contract, Run A, Run A2 or CFD numerical authority.', '', '## Frozen difference','', '```diff', '\n'.join(con['diff']),'```', '', 'Source: '+con['source_path']+' lines '+str(con['source_lines'])+'; SHA-256 '+con['source_sha256'], '', 'All non-pressure fvSolution text is exact. fvSchemes, physical properties, boundary fields, initialization, decomposition and mesh are byte-identical. Only administrative endTime changes from the 200-step original template to the authorized 500-step horizon. The sealed baseline was run as 200 plus 300 steps with a restart; this fresh diagnostic is continuous. The runtime comparison includes this small scope difference.', '', '## Numerical health and pressure effort','']
    for key in ('baseline','diagnostic'):
        s=m[key]['summary'];lines += [key+': `'+json.dumps({n:s[n] for n in ('pressure_iterations','linear_totals','flux_max','flux_final50','wall_max','bounds')},sort_keys=True)+'`','']
    lines+=['## Checkpoint fields','','|Iteration|Field|min|max|Negative|Nonfinite|','|---|---|---:|---:|---:|---:|']
    for step in (20,50,100,200,300,400,500):
        cp=m['checkpoints'][str(step)]
        for name,f in cp['fields'].items():lines.append(f"|{step}|{name}|{f['min']:.16g}|{f['max']:.16g}|{f['negative']}|{f['nonfinite']}|")
        lines+=['','Omega location: `'+json.dumps(cp['omega_location'])+'`','']
    for w in m['baseline']['windows']:
        lines+=['', '## '+w,'','|Control|Signal|Mean|Median|Std|Range|Slope|Detrended range|Normalized range|','|---|---|---:|---:|---:|---:|---:|---:|---:|']
        for key in ('baseline','diagnostic'):
            for name,s in m[key]['windows'][w]['signals'].items():lines.append('|'+key+'|'+name+'|'+'|'.join(f'{s[n]:.10g}' for n in ('mean','median','std','range','slope','detrended_range'))+'|'+str(s.get('normalized_range','N/A'))+'|')
            lines+=['','Lateral vectors '+key+': `'+json.dumps(m[key]['windows'][w]['lateral'],sort_keys=True)+'`','','Cross-correlations '+key+': `'+json.dumps(m[key]['windows'][w]['cross'],sort_keys=True)+'`','']
    lines+=['','## Resources','','```',(OUT/'pilot.resources').read_text(),'```','','Full spectra, ACF and cross-correlations are retained in MEASUREMENTS.json. Periods are solver iterations, not physical time. No new convergence threshold was introduced.']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')

def finalize():
    con=h.read(OUT/'CONTRACT.json');h.verify_record(con)
    m=h.read(OUT/'MEASUREMENTS.json');h.verify_record(m);assert m['contract']==con['identity']
    assert h.file_hash(OUT/'HISTORY.json')==m['history_hash'] and h.file_hash(OUT/'pilot.log')==m['log_hash']
    assert int((OUT/'pilot.exit').read_text())==0
    for n,sha in {**con['controls'],**con['inherited']}.items():assert h.file_hash(OUT/'pilot'/n)==sha
    assert hashlib.sha256(subprocess.check_output(['wsl','cat',con['source_path']])).hexdigest()==con['source_sha256']
    pre=h.read(OUT/'PREFLIGHT.json')
    for n,sha in pre['baseline_files'].items():assert h.file_hash(c.OUT/n)==sha
    prior=h.read(d.OUT/'RESULT.json');h.verify_record(prior)
    for n,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
        a=np.concatenate([l.u.e.read_internal(c.OUT/'pilot'/f'processor{i}'/'500'/n,dim) for i in range(6)])
        assert hashlib.sha256(a.astype('<f8').tobytes()).hexdigest()==prior['terminal_verified'][n]
    mesh=l.u.e.p.prior.m;mesh.validate(h.read(mesh.OUT/'AUTHORITY.json'),h.read(mesh.OUT/'CFD_HANDOFF.json'),con['mesh'])
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=lambda ss:[s for s in ss if 'capsule_pressure_diagnostic' not in s]
    assert unrelated(status)==unrelated(pre['status']) and len(unrelated(status))==155
    conclusion='Decision B. The stock pressure mechanism reduces runtime, residuals and short-period load variation, but does not remove a biased lateral load or its angular drift. All six final-50 residual slopes remain positive. Final lateral force mean magnitude is 0.4379819427184668 N and its angle changes by -5.10285206416588 degrees in that window. Pressure solution contributes to the iteration dynamics but does not resolve the main behavior. Conservation is materially less accurate under the looser solve; no conservation acceptance limit is changed or newly qualified.'
    recommendation='Stop steady-SIMPLE control experiments. Run one bounded transient/URANS diagnostic on the same sealed mesh, physical case and SST model, with a prospectively defined physical-time and time-step contract; do not infer a physical frequency from the steady iteration spectrum.'
    text='\n## Decision\n\n'+conclusion+'\n\nOmega maximum is wall-adjacent at every requested checkpoint (20,50,100,200,300,400,500) and every saved checkpoint from 20 onward. The extra iteration-10 check is an exception: maximum 268837.71867794264 at cell 61818, center (0.0302140915363064,0.021772205296656674,0.055181023332793164), not directly wall-adjacent. No direct wall-adjacency claim is made for that startup maximum.\n\nExactly one recommended next task: '+recommendation+'\n\nNo production control was promoted. Run A = NO; Run A2 = NO; stopping contract = NO; CFD numerical authority = NO.\n\nCleanup of task-created intermediate fields was rejected by execution policy (blocked by policy). No deletion occurred. All fields remain retained.\n'
    report();f=OUT/'REPORT.md';f.write_text(f.read_text()+text,encoding='utf-8',newline='\n')
    r=h.seal(dict(schema='V4PressureDiagnosticResult/1',classification='V4CapsuleRANSMotorBikePressureSolverDoesNotResolvePersistentOscillation',decision='B',conclusion=conclusion,next_task=recommendation,starting_head=HEAD,mesh=con['mesh'],baseline=con['baseline'],previous_diagnostic=con['previous_diagnostic'],contract=con['identity'],measurements=m['identity'],iterations=500,exit_status=0,wall_seconds=210.88,user_cpu_seconds=1176.03,system_cpu_seconds=46.29,peak_process_RSS_KiB=273512,aggregate_RSS='not measured',baseline_wall_seconds=382.62,baseline_timing_scope='200-step run plus 300-step continuation, including restart overhead',command='mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot',epoch_renewal=False,changed_numerical_mechanisms=1,mesh_changes=0,Run_A=False,Run_A2=False,stopping_contract=None,CFD_numerical_authority=None,promoted=False,unrelated_dirty_entries=155,V3_modified=False,terminal={n:f['sha256'] for n,f in m['checkpoints']['500']['fields'].items()},storage=dict(free_start=pre['free_bytes'],free_end=shutil.disk_usage(OUT).free,retained_logical_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),reclaimed_bytes=0,cleanup='Deletion rejected by execution policy; no deletion performed.'),implementation_sha256=h.file_hash(__file__),evidence={n:h.file_hash(OUT/n) for n in ('REPORT.md','MEASUREMENTS.json','HISTORY.json','CONTRACT.json','pilot.log','pilot.resources','TESTS.xml','EXECUTABLE.txt')}))
    h.durable_json(OUT/'RESULT.json',r);print(json.dumps(r,indent=2))

if __name__=='__main__':globals()[sys.argv[1]]()
