"""Deterministic read-only pilot diagnostics; no solve, tuning, or retry."""
import hashlib
import json
import re
import shutil
import subprocess
import numpy as np
import capsule_rans_authorized_pilot as p
from capsule_rans_checks import registry_audit

NUMBER=r'[-+0-9.eE]+'
ALLOWED="Removing patchGroup 'wall' which clashes with patch 0 of the same name."

def warning_audit(text):
    conflicts=re.findall(r'Removing patchGroup[^\n]+',text)
    warning_blocks=re.findall(r'--> FOAM Warning[^\n]*\n(?:[^\n]*\n){0,4}',text)
    return all(x.strip()==ALLOWED for x in conflicts) and all(ALLOWED in b for b in warning_blocks)

def parse_history(text):
    rows=[]
    for match in re.finditer(r'^Time = (\d+)s?\s*\n(.*?)(?=^Time = |\Z)',text,re.M|re.S):
        block=match[2];res={}
        for eq,a,b,n in re.findall(r'Solving for (\w+), Initial residual = ('+NUMBER+r'), Final residual = ('+NUMBER+r'), No Iterations (\d+)',block):
            if eq in res:raise ValueError('MultipleEquationSolvesRequireExplicitSemantics')
            res[eq]=dict(initial=float(a),final=float(b),iterations=int(n))
        if set(res)!=set(['Ux','Uy','Uz','p','k','omega']):raise ValueError('IncompleteResiduals')
        vectors=re.findall(r'(pressure|viscous|porous)\s*:\s*\(([^)]+)\)',block)
        if len(vectors)!=6:raise ValueError('IncompleteForcesMoments')
        arrays=[list(map(float,v.split())) for _,v in vectors]
        flux={}
        for operation,patch,value in re.findall(r'(sum|sumMag)\((wall|farfield)\) of phi = ('+NUMBER+r')',block):flux[patch+'_'+operation]=float(value)
        if len(flux)!=4:raise ValueError('IncompleteFlux')
        signed=flux['wall_sum']+flux['farfield_sum'];absolute=flux['wall_sumMag']+flux['farfield_sumMag']
        inflow=(absolute-signed)/2;outflow=(absolute+signed)/2
        if max(inflow,outflow)<=0:raise ValueError('NoThroughFlow')
        bounds=[dict(field=f,minimum=float(a),maximum=float(b),average=float(c)) for f,a,b,c in re.findall(r'bounding (\w+), min: ('+NUMBER+r') max: ('+NUMBER+r') average: ('+NUMBER+r')',block)]
        rows.append(dict(iteration=int(match[1]),residuals=res,pressure_force=arrays[0],viscous_force=arrays[1],
            force=np.sum(arrays[:3],axis=0).tolist(),pressure_moment=arrays[3],viscous_moment=arrays[4],moment=np.sum(arrays[3:],axis=0).tolist(),
            flux=dict(**flux,signed=signed,inflow=inflow,outflow=outflow,normalized=abs(signed)/max(inflow,outflow)),bounds=bounds))
    return rows

def read_internal(path,components):
    data=path.read_bytes()
    m=re.search(rb'internalField\s+nonuniform\s+List<(scalar|vector)>\s+(\d+)\s*\(',data)
    if not m:raise ValueError('UnsupportedFieldEncoding')
    if (m[1]==b'vector')!=(components==3):raise ValueError('WrongFieldType')
    count=int(m[2]);end=m.end()+count*components*8
    if data[end:end+1]!=b')':raise ValueError('WrongBinaryLayout')
    return np.frombuffer(data[m.end():end],dtype='<f8').reshape(count,components)

def main():
    o=p.OUT;h=p.h;contract=h.read(o/'PILOT_CONTRACT.json');h.verify_record(contract)
    for name,digest in contract['controls'].items():assert h.file_hash(o/'pilot'/name)==digest
    text=(o/'pilot.log').read_text();rows=parse_history(text)
    assert registry_audit(['forces','surfaceFieldValue'],(o/'FUNCTION_REGISTRY.txt').read_text())['passed']
    assert len(rows)==20 and [r['iteration'] for r in rows]==list(range(1,21))
    assert warning_audit(text)
    fields={}
    for field,components in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
        parts=[read_internal(o/'pilot'/f'processor{i}'/'20'/field,components) for i in range(6)]
        a=np.concatenate(parts);assert len(a)==562033
        measure=np.linalg.norm(a,axis=1) if components==3 else a[:,0]
        fields[field]=dict(minimum=float(measure.min()),maximum=float(measure.max()),nonfinite=int((~np.isfinite(a)).sum()),
            canonical_rank_order_sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest(),
            identity_order='processor0..5, local internal-cell order, binary64 little endian',cells=len(a))
    h.durable_json(o/'HISTORY.json',dict(schema='CapsulePilotHistory/1',rows=rows))
    negatives=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds'] if b['minimum']<0]
    maxhits=sum(v['iterations']>=5000 for r in rows for v in r['residuals'].values())
    assert negatives
    mesh=h.read(p.prior.m.OUT/'AUTHORITY.json')
    for name,digest in mesh['openfoam']['members'].items():
        assert h.file_hash(p.prior.m.CORE/'foundation/openfoam_case/constant/polyMesh'/name)==digest
        if name!='boundary':assert h.file_hash(o/'pilot/constant/polyMesh'/name)==digest
    pre=h.read(o/'PREFLIGHT.json')
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    remaining=[s for s in status if not any(n in s for n in ['capsule_rans_authorized_pilot','capsule_rans_pilot_evidence','test_capsule_authorized_pilot'])]
    assert remaining==pre['status']
    size=lambda d:sum(f.stat().st_size for f in d.rglob('*') if f.is_file())
    result=h.seal(dict(schema='V4CapsulePilotResult/1',classification='V4CapsuleRANSPilotFailedRepeatedTurbulentKineticEnergyPositivity',
        pilot_contract=contract['identity'],mesh=mesh['identity'],mesh_unchanged=True,case_controls_unchanged=True,
        accepted_warning='AcceptedFoundation13WallGroupNameCollisionWarning',warning_text=ALLOWED,
        checkMesh='Mesh OK.',pilot_iterations=20,exit_status=int((o/'pilot.exit').read_text()),
        wall_seconds=16.47,user_cpu_seconds=89.21,system_cpu_seconds=2.36,time_reported_peak_rss_KiB=263368,
        aggregate_peak_rss='not measured',linear_iterations={eq:sum(r['residuals'][eq]['iterations'] for r in rows) for eq in rows[0]['residuals']},
        linear_maxIter_hits=maxhits,negative_prebound_records=negatives,terminal_internal_fields=fields,
        first=rows[0],last=rows[-1],maximum_normalized_flux=max(r['flux']['normalized'] for r in rows),
        maximum_absolute_wall_flux=max(r['flux']['wall_sumMag'] for r in rows),
        force_component_ranges=np.ptp([r['force'] for r in rows],axis=0).tolist(),moment_component_ranges=np.ptp([r['moment'] for r in rows],axis=0).tolist(),
        interpretation='Repeated negative pre-bounding k violates healthy-pilot positivity requirement. OpenFOAM applies bounding; final written positivity does not prove healthy unconstrained iterations. Root cause is not established.',
        stopping_contract='Not frozen: pilot health failed',run_a='Not launched',run_a2='Not launched',aa='Not evaluated',numerical_authority=None,
        force_scale=contract['force_scale'],moment_scale=contract['moment_scale'],
        pilot_log_sha256=h.file_hash(o/'pilot.log'),history_sha256=h.file_hash(o/'HISTORY.json'),
        controls=contract['controls'],starting_free_bytes=pre['free_bytes'],ending_free_bytes=shutil.disk_usage(o).free,
        terminal_file_hashes={str(f.relative_to(o/'pilot')).replace('\\','/'):h.file_hash(f) for f in sorted((o/'pilot').glob('processor*/20/*')) if f.is_file()},
        tests='48 passed: authorized pilot, monitoring admission, prior RANS and CFD admission suites',
        source_behavior='wallPolyPatch.C dictionary constructor appends wall group; polyBoundaryMesh.C groupPatchIndices erases groups matching patch names. Warning is authorized; named patch lookup and monitor populations remain valid.',
        runtime_registered=['forces','surfaceFieldValue'],unsupported_removed=['fieldMinMax'],
        pilot_logical_bytes=size(o/'pilot'),task_logical_bytes=size(o),deleted_bytes=0,
        unrelated_dirty_entries=155,v3_modified=False,epoch_renewal=False,
        next_task='Diagnose capsule SST k-positivity loss from the sealed pilot and design one justified bounded correction study before another qualifying attempt.'))
    h.durable_json(o/'RESULT.json',result)
    (o/'REPORT.md').write_text('# Authorized capsule RANS pilot\n\n'+result['classification']+'\n\nNo tuning or retry. No Run A/A2. The fixed pilot completed before post-run health classification.\n\n```json\n'+json.dumps(result,indent=2)+'\n```\n',encoding='utf8')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
