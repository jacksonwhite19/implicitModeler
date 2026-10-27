"""Read-only admission audit for the first capsule RANS case; no solver launch."""
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import formalize_capsule_complete_mesh as mesh

V4=mesh.V4
V3=V4.parent/'sdf_cfd_mesher_v3'
OUT=V4/'results/capsule_cfd_first_aa_admission'
CLASS='V4CapsuleCFDNumericalCaseBlockedMissingRANSAuthorityAndWallPatchCompatibility'
PHYSICAL=dict(fluid='air',rho=1.225,mu=1.7894e-5,temperature_K=288.15,speed=33.528,
    velocity=[0.,0.,33.528],radius=.04,diameter=.08,length=.16,
    bounds=[[-.75,.75],[-.75,.75],[-1.25,1.25]],fluid_side='positive_sdf')

def admission(record):
    if record['mesh_authority']!='307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e':
        raise ValueError('WrongMeshAuthority')
    if record['physical']!=PHYSICAL: raise ValueError('WrongPhysicalBinding')
    reasons=[]
    if record['wall_patch_type']!='wall':reasons.append('WallFunctionsRequireWallFvPatch')
    if not record['turbulence_capsule_authority']:reasons.append('MissingCapsuleFreestreamTurbulenceAuthority')
    if record['stopping_scope']!='coupled_steady_incompressible_RANS':reasons.append('PotentialStoppingIsNotRANSStopping')
    if not record['complete_rans_controls']:reasons.append('MissingAcceptedRANSControlsAndForceStopping')
    return dict(passed=not reasons,blockers=reasons,solver_launch_allowed=not reasons)

def inspect():
    OUT.mkdir(exist_ok=True)
    tick=mesh.h.measured()
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    unrelated=[s for s in status if not any(n in s for n in ('capsule_cfd_first_aa_admission','audit_capsule_cfd_admission.py','test_capsule_cfd_admission.py'))]
    mesh.h.eq(len(unrelated),155,'DirtyStateChanged')
    head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    mesh.h.eq(head,'f146a17a6831fbb8bef677ea72cf5303625e658a','WrongHEAD')
    a=mesh.h.read(mesh.OUT/'AUTHORITY.json')
    mesh.validate(a,mesh.h.read(mesh.OUT/'CFD_HANDOFF.json'),a['identity'])
    access=mesh.h.read(mesh.OUT/'ACCESS.json')
    with mesh.Store(access['directory']).open(access['root']):pass
    sources={}
    def read(path):
        sources[str(path.relative_to(V3))]=mesh.h.file_hash(path)
        return path.read_text(encoding='utf-8-sig')
    policy_dir=V3/'inner_pcg_tolerance_qualification_2026-09-09'
    policy=json.loads(read(policy_dir/'FIXED_1E13_POLICY.json'))
    report=read(policy_dir/'FINAL_AUTHORITATIVE_REPORT.md')
    entry=read(policy_dir/'qualified_entry_draft.py')
    bootstrap=read(V3/'outer_iteration_stopping_qualification_2026-09-08/bootstrap.py')
    node=next(n.value for n in ast.parse(bootstrap).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='c' for t in n.targets))
    contract=eval(compile(ast.Expression(node),'<read-only literal contract>','eval'),{'__builtins__':{},'dict':dict})
    handoff=json.loads(read(policy_dir/'FROZEN_HANDOFF_CONTRACT.json'))
    mesh.h.need(policy['production_promoted'] and policy['tolerance']==1e-13 and policy['relTol']==0 and policy['maxIter']==5000,'V3PolicyMismatch')
    mesh.h.need('harmonic potential flow' in contract['qualification_scope'] and contract['consecutive_passes']==2,'V3ScopeMismatch')
    near=V3/'physics_driven_prism_75mph/practical_near_wall_retry'
    read(near/'analyze_policy.py')
    flat=near/'flat_plate_cases_lts/yplus_10'
    read(flat/'README')
    inlet={}
    for field in ('k','omega'):
        text=read(flat/'8000'/field)
        block=re.search(r'\binlet\s*\{([^}]+)\}',text)[1]
        inlet[field]=float(re.search(r'value\s+uniform\s+([^;]+)',block)[1])
    for field in ('U','p','nut'):read(flat/'8000'/field)
    missing=[]
    for root in (flat,V3/'foundation_aware_core_maximin/numerical/potential_case'):
        for name in ('fvSchemes','fvSolution','controlDict'):
            path=root/'system'/name
            if not path.exists():missing.append(str(path.relative_to(V3)))
    source='/opt/openfoam13/src/MomentumTransportModels/momentumTransportModels/derivedFvPatchFields/wallFunctions/nutWallFunctions/nutWallFunction/nutWallFunctionFvPatchScalarField.C'
    command=['wsl','-d','Ubuntu-22.04','bash','-lc','source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; foamRun -help 2>&1; sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun; cat '+source]
    native=subprocess.check_output(command,text=True)
    mesh.h.need('!isA<wallFvPatch>(patch())' in native and 'must be wall' in native,'MissingWallTypeEvidence')
    (OUT/'FOUNDATION_SOURCE_CHECK.txt').write_text(native,encoding='utf-8')
    record=dict(mesh_authority=a['identity'],physical=PHYSICAL,wall_patch_type=a['openfoam']['patches'][0]['type'],
        turbulence_capsule_authority=None,stopping_scope=contract['qualification_scope'],complete_rans_controls=False)
    decision=admission(record)
    mesh.h.need(not decision['passed'],'UnexpectedAdmission')
    result=dict(schema='V4CapsuleFirstRANSAdmissionAudit/1',classification=CLASS,starting_head=head,
        inputs=record,decision=decision,mesh_verified=True,mesh_scientific=mesh.SCI,mesh_content=mesh.CONTENT,
        access=access['root'],checkpoint=access['checkpoint'],epoch_renewal_required=False,
        physical_derived=dict(nu=PHYSICAL['mu']/PHYSICAL['rho'],frontal_area=math.pi*PHYSICAL['radius']**2),
        proposed_solver='Foundation 13 foamRun -solver incompressibleFluid; steady incompressible kOmegaSST',
        executable_sha256=re.search(r'([0-9a-f]{64})\s+/opt/openfoam13/platforms/[^\n]+/foamRun',native)[1],
        reusable_wall_profile=dict(U='noSlip',nut='nutUSpaldingWallFunction',omega='omegaWallFunction blended=true',k='kLowReWallFunction'),
        wall_pressure_candidate='zeroGradient from retained flat-plate field; full capsule pressure contract not qualified',
        farfield_proposal=dict(U='freestreamVelocity',p='freestreamPressure',turbulence='unbound mixed freestream inputs; not constructed'),
        historical_flat_plate_inlet=inlet,flat_plate_transfer_accepted=False,
        recovered_potential_policy=policy,recovered_potential_stopping=contract,
        recovered_terminal_handoff=handoff['application'],potential_gauge_is_not_RANS_pressure_authority=True,
        missing_control_files=missing,source_hashes=sources,
        runs=dict(A='NOT_STARTED_PRE_SOLVE_BLOCK',A2='NOT_STARTED_PRE_SOLVE_BLOCK'),
        numerical_case_authority=None,case_control_files={},terminal_fields={},
        mesh_modified=False,hardlinks_created=0,CFD_solves=0,
        starting_free_bytes=shutil.disk_usage(V4).free,unrelated_git_status=unrelated,metrics=mesh.h.metrics(*tick))
    result=mesh.h.seal(result);mesh.h.durable_json(OUT/'AUDIT.json',result)
    print(json.dumps(dict(classification=CLASS,identity=result['identity'],blockers=decision['blockers'])))

if __name__=='__main__':inspect()
