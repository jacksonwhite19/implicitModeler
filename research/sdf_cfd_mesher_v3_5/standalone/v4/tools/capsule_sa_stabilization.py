"""Exact-state, non-qualifying SA continuation. No scientific tuning."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import re
import numpy as np
import capsule_steady_models_sa as sa
import capsule_steady_models_history as history
import capsule_pressure_diagnostic as p
s=sa.s; h=s.h
ROOT=s.f.prior.control.V4/'results/capsule_sa_stabilization'
NAMES=('SA-GW1-S1','SA-GW1-S2')
HEAD='6e12c56fb33d0f2a168b674e75226a666ed49907'

def global_arrays(case,step):
    ids=[sa.binary_labels(case/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
    n=sum(map(len,ids));assert np.array_equal(np.sort(np.concatenate(ids)),np.arange(n))
    result={}
    for field,dim in [('U',3),('p',1),('nuTilda',1),('nut',1)]:
        a=np.empty((n,dim))
        for i,index in enumerate(ids):a[index]=p.l.u.e.read_internal(case/f'processor{i}/{step}/{field}',dim)
        result[field]=a
    return result

def canonical(case,step):
    result={}
    for field,a in global_arrays(case,step).items():
        v=np.linalg.norm(a,axis=1) if field=='U' else a[:,0]
        result[field]=dict(min=float(v.min()),max=float(v.max()),negative=int((v<0).sum()),nonfinite=int((~np.isfinite(a)).sum()),sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest())
    return result

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    prior=h.read(s.ROOT/'RESULT.json');h.verify_record(prior)
    assert prior['identity']=='1223f19abe165d46ab2e48a5367ae1423aa4d8b9a14f5db5c86010d459df486c'
    for rel,d in prior['evidence'].items():assert h.file_hash(s.ROOT/rel)==d,rel
    for rel,d in prior['implementation'].items():assert h.file_hash(s.f.prior.control.V4/rel)==d,rel
    retention=h.read(s.ROOT/'RETENTION_AUTHORITY.json');h.verify_record(retention)
    ROOT.mkdir(exist_ok=False)
    status=subprocess.check_output(['git','status','--porcelain'],text=True)
    (ROOT/'STARTING_STATUS.txt').write_text(status)
    pre=dict(starting_head=HEAD,prior=prior['identity'],free_bytes=shutil.disk_usage(ROOT).free,frozen=s.f.prior.frozen(),cases={})
    for name in NAMES:
        old=s.ROOT/'model_runs'/name;case=old/'pilot'
        con=h.read(old/'CFD_CONTRACT.json');h.verify_record(con)
        measured=h.read(old/'MEASUREMENTS.json');h.verify_record(measured)
        source=sa.mesh_source(con['mesh']);candidate=h.read(source/'CANDIDATE.json');h.verify_record(candidate)
        assert candidate['identity']==con['candidate'] and candidate['frozen']==pre['frozen']
        for rel,d in con['controls'].items():assert h.file_hash(case/rel)==d,rel
        for rel,d in con['mesh_files'].items():assert h.file_hash(case/'constant/polyMesh'/rel)==d,rel
        files={}
        for rank in range(6):
            for f in (case/f'processor{rank}/500').rglob('*'):
                if f.is_file():
                    digest=h.file_hash(f);assert digest==retention['retained_files'][str(f.relative_to(s.ROOT))]
                    files[str(f.relative_to(case))]=digest
            for f in (case/f'processor{rank}/constant').rglob('*'):
                if f.is_file():
                    rel=f.relative_to(case);digest=h.file_hash(f)
                    assert digest==h.file_hash(source/'pilot'/rel),str(rel)
                    files[str(rel)]=digest
        fields=canonical(case,500)
        assert fields==measured['checkpoints']['500']
        target=ROOT/name/'pilot';target.mkdir(parents=True)
        # Ordinary byte copies preserve the old sealed cases and exact decomposition.
        for directory in ('constant','system','0'):
            shutil.copytree(case/directory,target/directory)
        for rank in range(6):
            for directory in ('constant','500'):
                shutil.copytree(case/f'processor{rank}'/directory,target/f'processor{rank}'/directory)
        for rel,d in files.items():assert h.file_hash(target/rel)==d and (target/rel).stat().st_nlink==1
        control=target/'system/controlDict';before=control.read_bytes();after=before
        for a,b in [(b'startTime 0;',b'startTime 500;'),(b'endTime 500;',b'endTime 1000;')]:
            assert after.count(a)==1;after=after.replace(a,b)
        control.write_bytes(after)
        controls={rel:h.file_hash(target/rel) for rel in con['controls']}
        assert [rel for rel in controls if controls[rel]!=con['controls'][rel]]==['system/controlDict']
        contract=h.seal(dict(schema='FrozenSAContinuation/1',prior=measured['identity'],prior_contract=con['identity'],candidate=candidate['identity'],frozen=pre['frozen'],start=500,end=1000,ranks=6,scientific_changes=0,repartition=False,restart_from_zero=False,qualifying=False,controls=controls,mesh_files=con['mesh_files'],continuation_files=files,canonical500=fields,diff=list(difflib.unified_diff(before.decode().splitlines(),after.decode().splitlines(),fromfile='sealed500',tofile='continuation1000',lineterm=''))))
        h.durable_json(ROOT/name/'CONTRACT.json',contract)
        (ROOT/name/'.gitignore').write_text('pilot/\n')
        linux='/mnt/c/'+str((ROOT/name).resolve())[3:].replace('\\','/')
        (ROOT/name/'run.sh').write_text(f'''source /opt/openfoam13/etc/bashrc
set -e
cd {linux}
sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun > EXECUTABLE.txt
set +e
/usr/bin/time -v -o continuation.resources mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot > continuation.log 2>&1
echo $? > continuation.exit
''',newline='\n')
        pre['cases'][name]=contract['identity']
    h.durable_json(ROOT/'PREFLIGHT.json',h.seal(pre))

WINDOWS=((401,500),(451,500),(501,750),(751,1000),(801,900),(901,1000),(901,950),(951,1000))

def stopping_contract(assessments,proposal=None):
    """Fail closed: measured full-vector evidence is required for both seeds.

    This function does not estimate a threshold from a terminal value. A future
    proposal must carry its own numerical-floor evidence and number provenance.
    """
    if set(assessments)!=set(NAMES):raise ValueError('IncompletePairAssessment')
    if not all(v['full_vector_fixed_point'] for v in assessments.values()):
        if proposal is not None:raise ValueError('NoContractWithoutBothFixedPoints')
        return None
    if proposal is None:return None
    if not proposal.get('numeric_provenance') or not proposal.get('full_vector_conditions'):
        raise ValueError('UnsupportedStoppingNumbers')
    return h.seal(proposal)

def combined(old,new,complete=True):
    first=re.search(r'^Time = ',new,re.M)
    new=new[first.start():] if first else ''
    times=list(map(int,re.findall(r'^Time = (\d+)s?\s*$',new,re.M)))
    if not complete and times:
        new=new[:list(re.finditer(r'^Time = ',new,re.M))[-1].start()];times=times[:-1]
    if times!=list(range(501,501+len(times))):raise ValueError('InvalidContinuationSequence')
    if complete and times!=list(range(501,1001)):raise ValueError('IncompleteContinuation')
    rows=history.parse(old+'\n'+new,'SpalartAllmaras')
    if len(rows)!=500+len(times):raise ValueError('CombinedHistoryGap')
    return rows

def statistics(values):
    v=np.asarray(values,dtype=float)
    if len(v)<2 or not np.isfinite(v).all():raise ValueError('InvalidSignal')
    x=np.arange(len(v),dtype=float);x-=x.mean()
    slope=float(np.dot(x,v-v.mean())/np.dot(x,x))
    return dict(mean=float(v.mean()),median=float(np.median(v)),std=float(v.std()),min=float(v.min()),max=float(v.max()),range=float(np.ptp(v)),slope=slope,detrended_range=float(np.ptp(v-slope*x)),first=float(v[0]),last=float(v[-1]))

def signals(rows):
    out={eq:np.array([r['residuals'][eq]['initial'] for r in rows]) for eq in ('p','Ux','Uy','Uz','nuTilda')}
    for key,prefix in [('force','F'),('moment','M')]:
        a=np.array([r[key] for r in rows])
        for j,axis in enumerate('xyz'):out[prefix+axis]=a[:,j]
        out[prefix+'_lateral']=np.hypot(a[:,0],a[:,1])
        principal=np.degrees(np.arctan2(a[:,1],a[:,0]))
        angle=np.degrees(np.unwrap(np.radians(principal)))
        anchor=min(499,len(rows)-1)
        out[prefix+'_angle_deg']=angle+principal[anchor]-angle[anchor]
    for key in ('pressure_force','viscous_force'):out[key+'_z']=np.array([r[key][2] for r in rows])
    return out

def audit(name):
    folder=ROOT/name;case=folder/'pilot';con=h.read(folder/'CONTRACT.json');h.verify_record(con)
    for rel,d in con['controls'].items():assert h.file_hash(case/rel)==d,rel
    for rel,d in con['mesh_files'].items():assert h.file_hash(case/'constant/polyMesh'/rel)==d,rel
    for rel,d in con['continuation_files'].items():assert h.file_hash(case/rel)==d,rel
    assert s.f.prior.frozen()==con['frozen']
    assert canonical(case,500)==con['canonical500']
    return con

def health():
    result={}
    for name in NAMES:
        folder=ROOT/name;new=(folder/'continuation.log').read_text()
        finished=(folder/'continuation.exit').exists()
        rows=combined((s.ROOT/'model_runs'/name/'pilot.log').read_text(),new,finished)
        continuation=rows[500:]
        assert p.l.u.e.warning_audit(new)
        assert not re.search(r'FOAM FATAL|Floating point exception|Segmentation fault',new)
        bounds=[dict(iteration=r['iteration'],**b) for r in continuation for b in r['bounds']]
        maxiter=[r['iteration'] for r in continuation if any(v['iterations']>=5000 for v in r['residuals'].values())]
        result[name]=dict(completed=rows[-1]['iteration'],bounds=bounds,maxiter=maxiter,last=rows[-1],exit=(folder/'continuation.exit').read_text().strip() if finished else None)
        assert not maxiter,'LinearMaxIterPathology'
        assert len(bounds)<2,'RepeatedBoundednessRequiresStop'
        checkpoint_file=folder/'CHECKPOINT_HEALTH.json'
        checked=h.read(checkpoint_file) if checkpoint_file.exists() else {}
        for step in (600,700,800,900,1000):
            if str(step) in checked or rows[-1]['iteration']<step or (not finished and rows[-1]['iteration']==step):continue
            values=canonical(folder/'pilot',step)
            assert all(v['nonfinite']==0 for v in values.values()),'NonfiniteField'
            assert all(values[f]['negative']==0 for f in ('nuTilda','nut')),'NegativeTurbulenceField'
            checked[str(step)]=values
        h.durable_json(checkpoint_file,checked)
    h.durable_json(ROOT/'HEALTH.json',result)
    print(json.dumps({n:dict(completed=v['completed'],bounds=len(v['bounds']),maxiter=v['maxiter'],exit=v['exit'],drag=v['last']['force'][2],angle_deg=float(np.degrees(np.arctan2(v['last']['force'][1],v['last']['force'][0]))),flux=v['last']['flux']['normalized']) for n,v in result.items()}))

def analyze():
    output={}
    for name in NAMES:
        con=audit(name);folder=ROOT/name;case=folder/'pilot'
        assert (folder/'continuation.exit').read_text().strip()=='0'
        text=(folder/'continuation.log').read_text();assert p.l.u.e.warning_audit(text)
        rows=combined((s.ROOT/'model_runs'/name/'pilot.log').read_text(),text)
        h.durable_json(folder/'HISTORY.json',dict(model='SpalartAllmaras',rows=rows))
        h.durable_json(folder/'LINEAR_STATUS.json',dict(maxIter=5000,rows=[dict(iteration=r['iteration'],hit={eq:v['iterations']>=5000 for eq,v in r['residuals'].items()}) for r in rows]))
        series=signals(rows)
        windows={f'{lo}-{hi}':{key:statistics(v[lo-1:hi]) for key,v in series.items()} for lo,hi in WINDOWS}
        for window in windows.values():
            for key in ('Fx','Fy','Fz','F_lateral','Mx','My','Mz','M_lateral'):
                scale=3.460917477391561 if key.startswith('F') else .5537467963826498
                window[key]['normalized_range']=window[key]['range']/scale
                window[key]['normalized_slope']=window[key]['slope']/scale
        checkpoints={str(t):canonical(case,t) for t in (500,600,700,800,900,1000)}
        assert all(v['nonfinite']==0 for c in checkpoints.values() for v in c.values())
        assert all(c[f]['negative']==0 for c in checkpoints.values() for f in ('nuTilda','nut'))
        flux=np.array([r['flux']['normalized'] for r in rows]);wall=np.array([r['flux']['wall_sumMag'] for r in rows])
        result=h.seal(dict(contract=con['identity'],qualifying=False,iterations=list(range(1,1001)),continuation_iterations=500,checkpoints=checkpoints,windows=windows,
            residual_checkpoints={str(t):rows[t-1]['residuals'] for t in (500,600,700,800,900,1000)},
            linear_totals={period:{eq:sum(r['residuals'][eq]['iterations'] for r in rr) for eq in rows[0]['residuals']} for period,rr in [('1-1000',rows),('501-1000',rows[500:])]},
            bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']],maxiter=[r['iteration'] for r in rows if any(v['iterations']>=5000 for v in r['residuals'].values())],
            conservation=dict(max_all=float(flux.max()),max_continuation=float(flux[500:].max()),final=float(flux[-1]),final100=statistics(flux[-100:]),wall_max=float(wall.max()),wall_final=float(wall[-1]),last=rows[-1]['flux']),
            last=rows[-1],log=h.file_hash(folder/'continuation.log'),history=h.file_hash(folder/'HISTORY.json')))
        h.durable_json(folder/'MEASUREMENTS.json',result);output[name]=result['identity']
        # Identical retained nearest-cell sampling: unchanged mesh, same indices.
        import capsule_gw_wake as wake
        previous=np.load(s.ROOT/'model_runs'/name/'WAKE.npz')
        arrays=global_arrays(case,1000)
        np.savez_compressed(folder/'CANONICAL_1000.npz',**arrays)
        sampled={key:value[previous['cell_ids']] for key,value in arrays.items()}
        np.savez_compressed(folder/'WAKE_1000.npz',points=previous['points'],cell_ids=previous['cell_ids'],distances=previous['distances'],**sampled)
        stations={}
        for j,station in enumerate((.5,1,2,4,6,8,10)):
            sl=slice(j*3209,(j+1)*3209);q=previous['points'][sl];u=sampled['U'][sl]
            metrics=wake.disk_metrics(q,u)
            metrics['centerline_U']=u[np.all(q[:,:2]==0,axis=1)][0].tolist()
            metrics['sampling_distance_max']=float(previous['distances'][sl].max())
            stations[str(station)]=metrics
        h.durable_json(folder/'WAKE_COMPARISON.json',h.seal(dict(method='Identical retained 22463-point nearest-cell indices, distances and positive-deficit disk metrics',previous=h.read(s.ROOT/'model_runs'/name/'MEASUREMENTS.json')['wake']['stations'],terminal=stations,archive=h.file_hash(folder/'WAKE_1000.npz'),canonical_archive=h.file_hash(folder/'CANONICAL_1000.npz'))))
    h.durable_json(ROOT/'ANALYSIS.json',h.seal(output))

if __name__=='__main__':
    dict(prepare=prepare,health=health,analyze=analyze)[sys.argv[1] if len(sys.argv)>1 else 'prepare']()
