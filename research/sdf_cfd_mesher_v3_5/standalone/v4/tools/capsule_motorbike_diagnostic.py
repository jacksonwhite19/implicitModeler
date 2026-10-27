"""Read-only capsule signal analysis and untouched Foundation reference provenance."""
import hashlib,json,re,shutil,subprocess,sys
import numpy as np
import capsule_rans_500_pilot as c
h=c.h
OUT=c.OUT.parent/'capsule_motorbike_diagnostic'
SOURCE='/opt/openfoam13/tutorials/incompressibleFluid/motorBikeSteady'

def digest_tree(path):
    return {str(p.relative_to(path)).replace('\\','/'):h.file_hash(p) for p in sorted(path.rglob('*')) if p.is_file()}

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()=='43844965648ad8c2b134775794f583366df68d25'
    r=h.read(c.OUT/'RESULT.json');h.verify_record(r)
    assert r['identity']=='e6657f2659ce6f11133c5adfc431c4f7e6cce74ea31ed80bde498e3a08b84f6a'
    m=h.read(c.OUT/'MEASUREMENTS.json');h.verify_record(m)
    assert r['measurements']==m['identity']
    assert h.file_hash(c.OUT/'HISTORY.json')==m['history_hash']
    healthy=h.read(c.l.u.OUT/'RESULT.json');h.verify_record(healthy)
    assert healthy['identity']=='721ee0793b4191097893f346ad8e8c8c63d82c28967b58345ad257953c479f2e'
    authority=c.l.u.e.p.prior.m
    authority.validate(h.read(authority.OUT/'AUTHORITY.json'),h.read(authority.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    access=h.read(authority.OUT/'ACCESS.json')
    with authority.Store(access['directory']).open(access['root']):pass
    terminal={}
    for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
        a=np.concatenate([c.l.u.e.read_internal(c.OUT/'pilot'/f'processor{i}'/'500'/name,dim) for i in range(6)])
        terminal[name]=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest()
        assert terminal[name]==m['checkpoints']['500']['fields'][name]['sha256']
    OUT.mkdir(exist_ok=True)
    linux='/mnt/c/'+str(OUT.resolve())[3:].replace('\\','/')
    subprocess.run(['wsl','bash','-lc',f'cp -r {SOURCE} {linux}/motorBikeSteady'],check=True)
    controls=digest_tree(OUT/'motorBikeSteady')
    shutil.copytree(OUT/'motorBikeSteady',OUT/'stock_controls')
    capsule={str(p.relative_to(c.OUT)):h.file_hash(p) for p in [c.OUT/'HISTORY.json',c.OUT/'RESULT.json',c.OUT/'MEASUREMENTS.json',*list((c.OUT/'controls').rglob('*'))] if p.is_file()}
    h.durable_json(OUT/'PREFLIGHT.json',dict(free_bytes=shutil.disk_usage(OUT).free,controls=controls,capsule=capsule,terminal=terminal,epoch_renewal=False,status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines(),mesh='307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e',pilot=r['identity']))
    (OUT/'.gitignore').write_text('motorBikeSteady/\n')
    (OUT/'run.sh').write_text(f'''source /opt/openfoam13/etc/bashrc
cd {linux}
sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/{{foamRun,snappyHexMesh,potentialFoam,blockMesh,decomposePar,reconstructPar}} > EXECUTABLES.txt
/usr/bin/time -v -o workflow.resources bash motorBikeSteady/Allrun > workflow.log 2>&1
echo $? > workflow.exit
''',newline='\n')

def detrend(a):
    a=np.asarray(a,dtype=float);x=np.arange(len(a));return a-np.polyval(np.polyfit(x,a,1),x)

def signal(a):
    a=np.asarray(a,dtype=float);d=detrend(a);n=len(a)
    power=np.abs(np.fft.rfft(d*np.hanning(n)))**2;power[0]=0;j=int(np.argmax(power))
    ac=np.correlate(d,d,'full')[n-1:];ac=ac/ac[0] if ac[0] else ac
    peaks=[i for i in range(2,n//2) if ac[i]>ac[i-1] and ac[i]>=ac[i+1] and ac[i]>0]
    spectral_peaks=[i for i in range(1,len(power)-1) if power[i]>power[i-1] and power[i]>=power[i+1]]
    return dict(**c.stats(a),std=float(a.std()),detrended_range=float(np.ptp(d)),spectral_period=n/j if j else None,spectral_cycles=j,power_fraction=float(power[j]/power.sum()) if power.sum() else 0,secondary_peaks=[dict(period=n/i,cycles=i,fraction=float(power[i]/power.sum())) for i in sorted(spectral_peaks,key=lambda i:power[i],reverse=True)[:5]],acf_peaks=[dict(lag=i,value=float(ac[i])) for i in peaks],spectral_note='Linear detrend and Hann window. Frequencies are per iteration, not physical time. Fewer than three cycles is weak evidence. ACF uses fixed zero-lag energy denominator.')

def cross(a,b):
    a=detrend(a);b=detrend(b);n=len(a);values=[]
    for lag in range(-n//4,n//4+1):
        x,y=(a[:n-lag],b[lag:]) if lag>=0 else (a[-lag:],b[:n+lag])
        values.append((lag,float(np.corrcoef(x,y)[0,1])))
    best=max(values,key=lambda p:abs(p[1]))
    return dict(zero=dict(values)[0],best_lag=best[0],best_correlation=best[1],meaning='Positive lag means second signal follows first; detrended overlap Pearson correlation; exploratory lag search.')

def analyze():
    rows=h.read(c.OUT/'HISTORY.json')['rows'];assert [r['iteration'] for r in rows]==list(range(1,501))
    signals={eq:np.array([r['residuals'][eq]['initial'] for r in rows]) for eq in rows[0]['residuals']}
    for key,prefix in [('force','F'),('moment','M')]:
        for j,axis in enumerate('xyz'):signals[prefix+axis]=np.array([r[key][j] for r in rows])
    windows={}
    for lo in (201,301,401,451):
        subset={k:v[lo-1:] for k,v in signals.items()}
        windows[str(lo)+'-500']=dict(signals={k:signal(v) for k,v in subset.items()},cross={a+'/'+b:cross(subset[a],subset[b]) for a,b in [('Fx','My'),('Fy','Mx'),('Fx','Fy'),('Ux','Uy'),('Fx','Ux'),('Fy','Uy')]},lateral={})
        for prefix in ('F','M'):
            x,y=subset[prefix+'x'],subset[prefix+'y'];angle=np.unwrap(np.arctan2(y,x))*180/np.pi
            windows[str(lo)+'-500']['lateral'][prefix]=dict(magnitude=signal(np.hypot(x,y)),angle_degrees=signal(angle),first_angle=float(angle[0]),last_angle=float(angle[-1]),sign_changes_x=int(np.count_nonzero(np.diff(np.sign(x)))),sign_changes_y=int(np.count_nonzero(np.diff(np.sign(y)))))
    maxima={}
    for path in (c.l.OUT,c.OUT):
        measured=h.read(path/'MEASUREMENTS.json');h.verify_record(measured)
        assert h.file_hash(path/'HISTORY.json')==measured['history_hash']
        for step,cp in measured['checkpoints'].items():maxima[step]={n:cp['fields'][n]['max'] for n in ('k','omega','nut')}
    h.durable_json(OUT/'CAPSULE_ANALYSIS.json',h.seal(dict(windows=windows,checkpoint_maxima=maxima,flux_force_cross=cross([r['flux']['normalized'] for r in rows[200:]],[np.hypot(*r['force'][:2]) for r in rows[200:]]),flux_max=max(r['flux']['normalized'] for r in rows),wall_max=max(r['flux']['wall_sumMag'] for r in rows),terminal=rows[-1])))

def parse_motorbike(text):
    rows=[]
    for match in re.finditer(r'^Time = (\d+)s?\s*$(.*?)(?=^Time = |\Z)',text,re.M|re.S):
        block=match[2];res={}
        for x in re.finditer(r'Solving for (\w+), Initial residual = ([^,]+), Final residual = ([^,]+), No Iterations (\d+)',block):
            res.setdefault(x[1],[]).append(dict(initial=float(x[2]),final=float(x[3]),iterations=int(x[4])))
        bounds=[dict(field=x[1],minimum=float(x[2]),maximum=float(x[3]),average=float(x[4])) for x in re.finditer(r'bounding (\w+), min: (\S+) max: (\S+) average: (\S+)',block)]
        rows.append(dict(iteration=int(match[1]),residuals=res,bounds=bounds))
    return rows

def reference():
    case=OUT/'motorBikeSteady';text=(case/'log.foamRun').read_text();rows=parse_motorbike(text)
    assert len(rows)==500 and rows[-1]['iteration']==500
    pre=h.read(OUT/'PREFLIGHT.json')
    for name,d in pre['controls'].items():assert h.file_hash(case/name)==d,name
    for name,d in pre['capsule'].items():assert h.file_hash(c.OUT/name)==d,name
    equations={eq:dict(samples={str(i):rows[i-1]['residuals'][eq][0]['initial'] for i in (1,100,250,400,500)},whole=c.stats([r['residuals'][eq][0]['initial'] for r in rows]),last50=signal([r['residuals'][eq][0]['initial'] for r in rows[-50:]]),last100=signal([r['residuals'][eq][0]['initial'] for r in rows[-100:]]),linear_iterations=sum(v['iterations'] for r in rows for v in r['residuals'][eq])) for eq in ('Ux','Uy','Uz','p','k','omega')}
    logs={}
    for f in case.glob('log.*'):
        t=f.read_text();logs[f.name]=dict(sha256=h.file_hash(f),ended=bool(re.search(r'^End\s*$',t,re.M)),fatal='FOAM FATAL' in t,warnings=re.findall(r'^.*(?:Warning|warning).*$',t,re.M),execution=re.findall(r'ExecutionTime = ([\d.]+) s\s+ClockTime = ([\d.]+) s',t)[-1:])
        shutil.copyfile(f,OUT/f.name)
    assert all(v['ended'] and not v['fatal'] for v in logs.values())
    coeff_files=list(case.glob('postProcessing/**/forceCoeffs.dat'))
    coeff={}
    for i,f in enumerate(coeff_files):
        a=np.loadtxt(f);shutil.copyfile(f,OUT/f'forceCoeffs_{i}.dat');coeff[str(f.relative_to(case))]=dict(header='\n'.join(x for x in f.read_text().splitlines() if x.startswith('#')),rows=len(a),terminal=a[-1].tolist(),last50={str(j):signal(a[-50:,j]) for j in range(1,a.shape[1])},last300={str(j):signal(a[-300:,j]) for j in range(1,a.shape[1])})
    h.durable_json(OUT/'MOTORBIKE_HISTORY.json',dict(rows=rows))
    h.durable_json(OUT/'MOTORBIKE_ANALYSIS.json',h.seal(dict(equations=equations,logs=logs,coefficients=coeff,bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']],warnings=re.findall(r'^.*(?:Warning|warning|FOAM FATAL|sigFpe|Floating point).*$' ,text,re.M),reported_linear_1000_hits=sum(v['iterations']>=1000 for r in rows for vs in r['residuals'].values() for v in vs),control_hashes_unchanged=True)))

def report():
    a=h.read(OUT/'CAPSULE_ANALYSIS.json');b=h.read(OUT/'MOTORBIKE_ANALYSIS.json')
    lines=['# Stock Foundation and capsule diagnostic comparison','',
        'Only the stock motorBike tutorial was run. Capsule new iterations = 0; control changes = 0; mesh changes = 0; Run A = NO; Run A2 = NO.',
        'Residuals are first initial residuals per equation and outer iteration. Stock log precision is the shipped six-digit output; capsule logs retain more digits. Spectra use linear detrending and a Hann window. ACF uses zero-lag energy normalization. These are iteration periods, not physical frequencies.',
        '', '## MotorBike initial residuals', '', '|Equation|1|100|250|400|500|Run min|max|Last50 mean|median|range|slope/iteration|Linear iterations|','|---|'+'---:|'*12]
    for eq,v in b['equations'].items():
        values=[*v['samples'].values(),v['whole']['min'],v['whole']['max'],v['last50']['mean'],v['last50']['median'],v['last50']['range'],v['last50']['slope'],v['linear_iterations']]
        lines.append('|'+eq+'|'+'|'.join(f'{x:.9g}' for x in values)+'|')
    for w,v in a['windows'].items():
        lines+=['',f'## Capsule {w}', '', '|Signal|Mean|Std|Range|Slope/iteration|Detrended range|','|---|---:|---:|---:|---:|---:|']
        for name,s in v['signals'].items():lines.append('|'+name+'|'+'|'.join(f'{s[k]:.9g}' for k in ('mean','std','range','slope','detrended_range'))+'|')
        lines+=['','Cross-correlations: `'+json.dumps(v['cross'],sort_keys=True)+'`','', 'Lateral vectors: `'+json.dumps(v['lateral'],sort_keys=True)+'`']
    lines+=['','## Capsule spectra and autocorrelation (201–500)','']
    for k,v in a['windows']['201-500']['signals'].items():
        if k[0] in ('F','M'):lines.append(k+': `'+json.dumps({n:v[n] for n in ('secondary_peaks','acf_peaks')})+'`')
    lines+=['','## Native stock coefficients','',json.dumps(b['coefficients'],indent=2),'','## Resource measurement','', '```', (OUT/'workflow.resources').read_text(),'```','', '## Full deterministic evidence','', 'CAPSULE_ANALYSIS.json and MOTORBIKE_ANALYSIS.json retain all statistics, including four windows, cross-correlations, spectra, checkpoint maxima and native coefficient statistics. MOTORBIKE_HISTORY.json retains all parsed solves; raw logs retain source output. Stock files are retained in stock_controls and their hashes in PREFLIGHT.json.']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')

def finalize():
    pre=h.read(OUT/'PREFLIGHT.json')
    source=json.loads(subprocess.check_output(['wsl','python3','-c',f"import pathlib,hashlib,json; p=pathlib.Path('{SOURCE}'); print(json.dumps({{str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest() for f in p.rglob('*') if f.is_file()}}))"],text=True))
    assert source==pre['controls']
    for name,sha in pre['capsule'].items():assert h.file_hash(c.OUT/name)==sha
    for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
        a=np.concatenate([c.l.u.e.read_internal(c.OUT/'pilot'/f'processor{i}'/'500'/name,dim) for i in range(6)])
        assert hashlib.sha256(a.astype('<f8').tobytes()).hexdigest()==pre['terminal'][name]
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=lambda rows:[x for x in rows if 'capsule_motorbike_diagnostic' not in x]
    assert unrelated(status)==unrelated(pre['status']) and len(unrelated(status))==155
    analysis=h.read(OUT/'CAPSULE_ANALYSIS.json');reference=h.read(OUT/'MOTORBIKE_ANALYSIS.json')
    summary='The capsule has a coherent approximately 17-iteration load component plus material slow lateral drift. Stock motorBike also has residual floors and coefficient oscillations, including broader approximately 43-iteration spectral content. Nonzero transverse U residuals alone are normal in this reference. The comparison does not prove physical unsteadiness or a numerical cause. No evidence of abnormal Foundation execution was found. Diagnostic choice B is closest, with limited causal confidence; existing SIMPLE and relaxation controls already match stock, so changing those to the same values would not be a useful experiment.'
    next_task='Run one bounded, non-qualifying capsule pressure-solver comparison using the stock motorBike GAMG/GaussSeidel pressure settings as the only changed mechanism; hold mesh, physics, SIMPLE, relaxation and all other controls fixed. This tests a documented remaining numerical difference, not a proven cause.'
    result=h.seal(dict(schema='V4StockFoundationBehaviorDiagnostic/1',classification='V4CapsuleRANSBehaviorShowsMaterialPersistentOscillationBeyondFoundationReference',diagnostic_choice='B with limited causal attribution',conclusion=summary,next_task=next_task,starting_head='43844965648ad8c2b134775794f583366df68d25',mesh=pre['mesh'],pilot=pre['pilot'],capsule_analysis=analysis['identity'],motorbike_analysis=reference['identity'],stock_source_unchanged=True,stock_controls_unchanged=True,stock_files=len(source),capsule_new_iterations=0,capsule_control_changes=0,capsule_mesh_changes=0,capsule_A=False,capsule_A2=False,unrelated_dirty_entries=155,V3_modified=False,terminal_verified=pre['terminal'],free_start=pre['free_bytes'],free_end=shutil.disk_usage(OUT).free,retained_logical_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),reclaimed_bytes=0,cleanup='No deletion. The modest stock reference allocation is retained with terminal fields for reproducibility.',evidence={n:h.file_hash(OUT/n) for n in ['PREFLIGHT.json','CAPSULE_ANALYSIS.json','MOTORBIKE_ANALYSIS.json','MOTORBIKE_HISTORY.json','REPORT.md','TESTS.xml','workflow.resources','EXECUTABLES.txt']}))
    result.pop('identity');result['implementation_sha256']=h.file_hash(__file__);result=h.seal(result)
    h.durable_json(OUT/'RESULT.json',result)
    (OUT/'CONCLUSION.md').write_text(summary+'\n\nExactly one recommended next task: '+next_task+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))

if __name__=='__main__':globals()[sys.argv[1]]()
