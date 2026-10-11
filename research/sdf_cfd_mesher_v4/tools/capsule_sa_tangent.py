"""Conservative tangent qualification. Never advance a retained CFD state."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import capsule_sa_operator as old

h=old.h
c=old.c
ROOT=old.ROOT.parent/'capsule_sa_tangent'
HEAD='e04c4f422b9a60d93387ad702184e767c8fe3de9'


def verify():
    prior=h.read(old.ROOT/'RESULT.json');h.verify_record(prior)
    assert prior['identity']=='0a6ee264ec539f3c21757b827198d0273dfa9cad6cc77682d9fa6e5855f4803e'
    for rel,digest in prior['evidence'].items():
        assert h.file_hash(old.ROOT/rel)==digest,rel
    for rel,digest in prior['implementation'].items():
        assert h.file_hash(c.s.f.prior.control.V4.parents[1]/rel)==digest,rel
    arrays=h.read(old.ROOT/'ARRAY_MANIFEST.json');h.verify_record(arrays)
    for rel,entry in arrays['files'].items():assert h.file_hash(old.ROOT/rel)==entry['sha256'],rel
    audit=h.read(c.ROOT/'UNRELATED_AUDIT.json')
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=[s for s in status if 'capsule_sa_tangent' not in s and '/sa_tangent/' not in s]
    assert unrelated==audit['status']
    exclusions=[':(exclude)research/sdf_cfd_mesher_v4/'+p for p in (
        'tools/capsule_sa_tangent*.py','tools/sa_tangent/**',
        'results/capsule_sa_tangent/**','tests/test_capsule_sa_tangent.py')]
    assert hashlib.sha256(subprocess.check_output(['git','diff','--binary','--','.',*exclusions],stderr=subprocess.DEVNULL)).hexdigest()==audit['tracked_diff_sha256']
    retained=h.read(c.ROOT/'RETENTION.json');h.verify_record(retained)
    for rel,digest in retained['keep'].items():assert h.file_hash(c.ROOT/rel)==digest,rel
    scratch=h.read(old.ROOT/'SCRATCH_INPUTS.json');h.verify_record(scratch)
    for rel,digest in scratch['files'].items():assert h.file_hash(old.ROOT/'scratch'/rel)==digest,rel
    assert {str(x.relative_to(old.ROOT/'scratch')) for x in (old.ROOT/'scratch').rglob('*') if x.is_file()}==set(scratch['files'])
    cases={}
    for name in c.NAMES:
        con=c.audit(name)
        source=c.sa.mesh_source(name[3:]);candidate=h.read(source/'CANDIDATE.json');h.verify_record(candidate)
        assert candidate['identity']==con['candidate']
        family=h.read(source.parent/'FAMILY_CONTRACT.json');h.verify_record(family)
        assert family['identity']==candidate['family_contract']=='dd239298d7e671b70e1248120e6ebd51c25a52dd6c7013c10fbf3059d1eb0c90'
        for rel,digest in candidate['evidence'].items():assert h.file_hash(source/rel)==digest,rel
        for field,digest in candidate['assembly']['hashes'].items():assert h.array_hash(np.load(source/'assembly'/(field+'.npy')))==digest,field
        measured=h.read(c.ROOT/name/'MEASUREMENTS.json')
        checkpoints={str(t):c.canonical(c.ROOT/name/'pilot',t) for t in (900,1000)}
        for t,x in checkpoints.items():assert x==measured['checkpoints'][t]
        cases[name]=dict(candidate=con['candidate'],frozen=con['frozen'],checkpoints=checkpoints)
    return dict(prior=prior['identity'],cases=cases,retained_files=len(retained['keep']),
                scratch_files=len(scratch['files']),unrelated_count=len(unrelated))


def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    ROOT.mkdir(exist_ok=True)
    (ROOT/'.gitignore').write_text('build/\noutputs/\ndirections/\n*.npz\n')
    result=verify();result['starting_head']=HEAD;result['free_bytes']=shutil.disk_usage(ROOT).free
    (ROOT/'sources').mkdir()
    sources={}
    oldsources=h.read(old.ROOT/'ADMISSION.json')['source_files']
    extra=h.read(old.ROOT/'EXTRA_SOURCE_PROVENANCE.json')
    paths=list(oldsources)
    paths+=['/opt/openfoam13/src/finiteVolume/lnInclude/'+p for p in (
        'freestreamVelocityFvPatchVectorField.C','freestreamPressureFvPatchScalarField.C',
        'freestreamFvPatchField.C','inletOutletFvPatchField.C','constrainPressure.C',
        'adjustPhi.C','constrainHbyA.C','mixedFvPatchField.C','calculatedFvPatchField.C')]
    paths+=['/opt/openfoam13/src/MomentumTransportModels/momentumTransportModels/derivedFvPatchFields/wallFunctions/nutWallFunctions/nutUSpaldingWallFunction/nutUSpaldingWallFunctionFvPatchScalarField.C']
    paths+=['/opt/openfoam13/src/finiteVolume/fvMatrices/fvMatrix/'+p for p in ('fvMatrix.C','fvMatrixSolve.C')]
    for source in paths:
        data=subprocess.check_output(['wsl','cat',source])
        target=ROOT/'sources'/Path(source).name;target.write_bytes(data)
        digest=h.file_hash(target)
        if source in oldsources:assert digest==oldsources[source]['sha256']
        if source in extra:assert digest==extra[source]['sha256']
        sources[source]=dict(sha256=digest,lines=len(data.splitlines()))
    exe=subprocess.check_output(['wsl','sha256sum','/opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun'],text=True).split()[0]
    assert exe=='f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e'
    result['source_files']=sources;result['foamRun_sha256']=exe
    h.durable_json(ROOT/'ADMISSION.json',h.seal(result))
    print('Admitted',result['free_bytes'],flush=True)


def sa_nut(n,nu=1.4607346938775508e-5,cv1=7.1):
    chi=n/nu;t=chi**3;b=cv1**3
    return n*t/(t+b)


def sa_derivative(n,nu=1.4607346938775508e-5,cv1=7.1):
    t=(n/nu)**3;b=cv1**3
    return t*(t+4*b)/(t+b)**2


def boundary_velocity_tangent(ui,ub,di,normal):
    w=.5*(ui+ub);m=np.linalg.norm(w,axis=-1,keepdims=True)
    dot=np.sum(w*normal,axis=-1,keepdims=True)
    f=.5-.5*dot/m
    a=-.5*(normal/m-dot*w/m**3);b=np.array([0.,0.,33.528])-ui
    rhs=(1-f)*di+.5*b*np.sum(a*di,axis=-1,keepdims=True)
    return rhs+.5*b*np.sum(a*rhs,axis=-1,keepdims=True)/(1-.5*np.sum(a*b,axis=-1,keepdims=True))


def qualification(generator,flux,boundary,pressure,simple):
    if not all((generator,flux,boundary,pressure)):
        return 'V4CapsuleSteadySAPhysicalRotationalTangentStillNotQualified'
    return ('ConservativeBoundaryConsistentRotationalTangentQualified' if simple else
            'ConservativeRotationalTangentQualifiedButReducedSIMPLETangentNotQualified')


if __name__=='__main__':
    import sys
    {'prepare':prepare,'verify':lambda:print(verify())}[sys.argv[1]]()
