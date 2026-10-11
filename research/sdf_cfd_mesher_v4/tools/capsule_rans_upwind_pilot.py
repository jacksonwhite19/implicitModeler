"""Single authorized turbulence-convection correction pilot; no A/A or tuning."""
import hashlib,json,re,shutil,subprocess,sys
import numpy as np
import capsule_rans_pilot_evidence as e
from capsule_rans_checks import registry_audit
h=e.p.h
BASE=e.p.OUT
OUT=e.p.prior.m.RESULTS/'capsule_rans_upwind_pilot'
EXPECTED_HEAD='4e3cf09df683d6ff9fe12e40fe56b7a049eb4c3c'

def change(text):
    for field in ('k','omega'):
        old=f'div(phi,{field}) bounded Gauss linearUpwind grad({field});'
        if text.count(old)!=1:raise ValueError('UnexpectedBaselineScheme')
        text=text.replace(old,f'div(phi,{field}) bounded Gauss upwind;')
    return text

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==EXPECTED_HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    old=h.read(BASE/'RESULT.json');h.verify_record(old);assert old['identity']=='6d5dd21a3779e8be8e04e9fb40c0b85c60b36c40e21f72ccfa65f5b1a0629498'
    contract=h.read(BASE/'PILOT_CONTRACT.json');h.verify_record(contract);assert contract['identity']=='112b62a99260f2b546fbf7e22969c986ee5d3d4a766dcc0abfd444416061604b'
    m=e.p.prior.m;a=h.read(m.OUT/'AUTHORITY.json');m.validate(a,h.read(m.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    access=h.read(m.OUT/'ACCESS.json')
    with m.Store(access['directory']).open(access['root']):pass
    status=[s for s in subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines() if 'capsule_rans_upwind_pilot' not in s];assert len(status)==155
    OUT.mkdir(exist_ok=True)
    h.durable_json(OUT/'PREFLIGHT.json',dict(status=status,starting_head=EXPECTED_HEAD,free_bytes=shutil.disk_usage(OUT).free,epoch_renewal=False))
    e.p.prior.write(OUT/'.gitignore','pilot/\n')
    for name in ('system/fvSchemes','system/fvSolution','system/controlDict','system/forceCoeffs','0/k','0/omega','0/nut'):
        source='/opt/openfoam13/tutorials/incompressibleFluid/motorBikeSteady/'+name
        data=subprocess.check_output(['wsl','cat',source]);target=OUT/'provenance'/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    ref=(OUT/'provenance/system/fvSchemes').read_text()
    assert re.search(r'div\(phi,k\)\s+bounded Gauss upwind;',ref) and re.search(r'div\(phi,omega\)\s+bounded Gauss upwind;',ref)
    case=OUT/'pilot'
    if case.exists():raise ValueError('NoSecondPilot')
    shutil.copytree(BASE/'controls',case)
    for name,digest in contract['controls'].items():assert h.file_hash(case/name)==digest
    scheme=case/'system/fvSchemes';e.p.prior.write(scheme,change(scheme.read_text()))
    shutil.copytree(BASE/'pilot/constant/polyMesh',case/'constant/polyMesh')
    for name in a['openfoam']['members']:assert h.file_hash(case/'constant/polyMesh'/name)==h.file_hash(BASE/'pilot/constant/polyMesh'/name)
    for name in contract['controls']:e.p.prior.write(OUT/'controls'/name,(case/name).read_text())
    new=h.seal(dict(schema='V4CapsuleUpwindCorrectionPilot/1',classification='NON_QUALIFYING_CORRECTION_PILOT',iterations=20,
        baseline=old['identity'],mesh=a['identity'],changed_entries=['div(phi,k)','div(phi,omega)'],
        controls={n:h.file_hash(case/n) for n in contract['controls']},
        provenance={str(f.relative_to(OUT/'provenance')):h.file_hash(f) for f in (OUT/'provenance').rglob('*') if f.is_file()},
        no_other_control_change=all(h.file_hash(case/n)==v for n,v in contract['controls'].items() if n!='system/fvSchemes')))
    h.durable_json(OUT/'CONTRACT.json',new)
    script=(BASE/'run_pilot.sh').read_text().replace(str(BASE.name),str(OUT.name))
    e.p.prior.write(OUT/'run_pilot.sh',script)
    print(new['identity'])

def analyze():
    c=h.read(OUT/'CONTRACT.json');h.verify_record(c)
    for n,d in c['controls'].items():assert h.file_hash(OUT/'pilot'/n)==d
    text=(OUT/'pilot.log').read_text();rows=e.parse_history(text)
    assert len(rows)==20 and e.warning_audit(text)
    assert registry_audit(['forces','surfaceFieldValue'],(OUT/'FUNCTION_REGISTRY.txt').read_text())['passed']
    assert 'Mesh OK.' in (OUT/'checkMesh.log').read_text()
    fields={}
    for name,components in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
        a=np.concatenate([e.read_internal(OUT/'pilot'/f'processor{i}'/'20'/name,components) for i in range(6)])
        assert len(a)==562033
        vals=np.linalg.norm(a,axis=1) if components==3 else a[:,0]
        fields[name]=dict(minimum=float(vals.min()),maximum=float(vals.max()),nonfinite=int((~np.isfinite(a)).sum()),negative=int((vals<0).sum()),sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest())
    bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']]
    h.durable_json(OUT/'HISTORY.json',dict(rows=rows,bounds=bounds))
    old=h.read(BASE/'RESULT.json')
    summary=dict(first_negative_k=next((b['iteration'] for b in bounds if b['field']=='k' and b['minimum']<0),None),negative_k_iterations=sum(b['field']=='k' and b['minimum']<0 for b in bounds),worst_prebound_k=min((b['minimum'] for b in bounds if b['field']=='k'),default=None),maximum_normalized_flux=max(r['flux']['normalized'] for r in rows),maximum_wall_flux=max(r['flux']['wall_sumMag'] for r in rows),linear_maxIter_hits=sum(v['iterations']>=5000 for r in rows for v in r['residuals'].values()))
    result=dict(contract=c['identity'],fields=fields,summary=summary,bounds=bounds,first=rows[0],last=rows[-1],
        old_comparison=dict(negative_k_iterations=17,worst_prebound_k=-.82618513301446228,fields=old['terminal_internal_fields'],runtime=old['wall_seconds'],flux=old['maximum_normalized_flux']),
        controls=c['controls'],linear_iterations={eq:sum(r['residuals'][eq]['iterations'] for r in rows) for eq in rows[0]['residuals']},
        terminal_file_hashes={str(f.relative_to(OUT/'pilot')):h.file_hash(f) for f in sorted((OUT/'pilot').glob('processor*/20/*')) if f.is_file()},
        force_range=np.ptp([r['force'] for r in rows],axis=0).tolist(),moment_range=np.ptp([r['moment'] for r in rows],axis=0).tolist(),
        history_sha256=h.file_hash(OUT/'HISTORY.json'),log_sha256=h.file_hash(OUT/'pilot.log'))
    h.durable_json(OUT/'MEASUREMENTS.json',h.seal(result));print(json.dumps(result,indent=2))

def finalize():
    measured=h.read(OUT/'MEASUREMENTS.json');h.verify_record(measured)
    assert not measured['bounds'] and measured['summary']['linear_maxIter_hits']==0
    assert all(f['nonfinite']==0 for f in measured['fields'].values())
    assert measured['fields']['k']['negative']==measured['fields']['omega']['negative']==0
    assert int((OUT/'pilot.exit').read_text())==0
    mesh=h.read(e.p.prior.m.OUT/'AUTHORITY.json')
    for name,digest in mesh['openfoam']['members'].items():
        assert h.file_hash(e.p.prior.m.CORE/'foundation/openfoam_case/constant/polyMesh'/name)==digest
        assert h.file_hash(OUT/'pilot/constant/polyMesh'/name)==h.file_hash(BASE/'pilot/constant/polyMesh'/name)
    pre=h.read(OUT/'PREFLIGHT.json')
    remaining=[s for s in subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines() if 'capsule_rans_upwind_pilot' not in s]
    assert remaining==pre['status']
    size=lambda d:sum(f.stat().st_size for f in d.rglob('*') if f.is_file())
    result=h.seal(dict(schema='V4CapsuleUpwindPilotQualification/1',classification='V4CapsuleRANSMotorBikeStyleTurbulencePilotHealthy',
        scope='NON_QUALIFYING_CORRECTION_PILOT; short numerical-health evidence only; no convergence or A/A qualification',
        starting_head=EXPECTED_HEAD,mesh=mesh['identity'],mesh_unchanged=True,measurements=measured,
        contract=h.read(OUT/'CONTRACT.json')['identity'],exit_status=0,iterations=20,
        command='mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot',
        foundation_build='13-cde978a97c93',executable_sha256='f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e',
        wall_seconds=16.85,user_cpu_seconds=90.66,system_cpu_seconds=2.21,time_peak_rss_KiB=262832,aggregate_peak_rss='not measured',
        checkMesh='Mesh OK.',warning='AcceptedFoundation13WallGroupNameCollisionWarning',
        source_line_evidence={'system/fvSchemes':'18 steadyState; 30 velocity; 31-32 turbulence upwind; 38 corrected Laplacian',
            'system/fvSolution':'40-56 turbulence solvers; 59-63 SIMPLE; 70-77 relaxation',
            '0/k':'39-49 kqRWallFunction differs from preserved capsule kLowReWallFunction',
            '0/omega':'38-48 omegaWallFunction',
            '0/nut':'44-66 nutkWallFunction differs from preserved capsule nutUSpaldingWallFunction',
            'system/forceCoeffs':'9-29 forceCoeffs library, patch and reference settings; capsule dimensional forces retained'},
        conclusion='Changing only turbulence convection removed all logged k/omega bounding and the extreme omega state in this 20-step comparison. This supports the correction for a longer stability study, not a general root-cause or convergence claim.',
        prebound_minimum_note='No bounding message means pre-bounding minima are unreported, not numerically zero.',
        tests='55 passed',run_a=0,run_a2=0,controls_changed_after_launch=False,epoch_renewal=False,
        starting_free_bytes=pre['free_bytes'],ending_free_bytes=shutil.disk_usage(OUT).free,
        pilot_logical_bytes=size(OUT/'pilot'),task_logical_bytes=size(OUT),deleted_bytes=0,previous_pilot_deleted=False,
        unrelated_dirty_entries=155,v3_modified=False,
        next_task='Run a bounded longer non-qualifying stabilization and stopping-contract study with these same Foundation-style controls; do not launch A/A until its prospective contract is justified.'))
    h.durable_json(OUT/'RESULT.json',result)
    lines=['# Foundation-style capsule turbulence correction pilot','',result['classification'],'',result['scope'],'',
        'Only div(phi,k) and div(phi,omega) changed from bounded Gauss linearUpwind grad(field) to bounded Gauss upwind.',
        'The reference motorBike wall models and pressure solver differ; none of those differences were imported.',
        'Every initial/control file except fvSchemes remains byte-identical to the failed pilot. Mesh member hashes remain exact.',
        '', '```json',json.dumps(result,indent=2),'```','', '## Installed Foundation reference snapshots']
    for f in sorted((OUT/'provenance').rglob('*')):
        if f.is_file():lines+=['### '+str(f.relative_to(OUT/'provenance')),'```',*['%4d %s'%(i,s) for i,s in enumerate(f.read_text().splitlines(),1)],'```']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print(json.dumps(dict(identity=result['identity'],result_sha256=h.file_hash(OUT/'RESULT.json'),report_sha256=h.file_hash(OUT/'REPORT.md'),storage=result['task_logical_bytes'],free=result['ending_free_bytes'],last=measured['last'],range=measured['force_range']),indent=2))

if __name__=='__main__':{'prepare':prepare,'analyze':analyze,'finalize':finalize}[sys.argv[1]]()
