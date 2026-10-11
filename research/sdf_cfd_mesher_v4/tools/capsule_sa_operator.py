"""Read-only discrete-operator study orchestration. No CFD advancement."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
import numpy as np
import capsule_sa_rotation as r
c=r.c;h=r.h
ROOT=c.ROOT.parent/'capsule_sa_operator'
HEAD='cdaa7967e47edae2e72192daa289edc1273686cc'
SOURCES=(
'/opt/openfoam13/applications/modules/incompressibleFluid/momentumPredictor.C',
'/opt/openfoam13/applications/modules/incompressibleFluid/correctPressure.C',
'/opt/openfoam13/applications/modules/incompressibleFluid/incompressibleFluid.C',
'/opt/openfoam13/src/MomentumTransportModels/momentumTransportModels/RAS/SpalartAllmaras/SpalartAllmaras.C',
'/opt/openfoam13/src/MomentumTransportModels/momentumTransportModels/RAS/SpalartAllmaras/SpalartAllmaras.H',
'/opt/openfoam13/src/MomentumTransportModels/momentumTransportModels/linearViscousStress/linearViscousStress.C',
'/opt/openfoam13/src/MomentumTransportModels/momentumTransportModels/eddyViscosity/eddyViscosity.C')

def linux(path):return '/mnt/c/'+str(Path(path).resolve())[3:].replace('\\','/')

def verify():
    old=h.read(c.ROOT/'UNRELATED_AUDIT.json')
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    assert [s for s in status if 'capsule_sa_operator' not in s and '/sa_operator/' not in s]==old['status']
    assert hashlib.sha256(subprocess.check_output(['git','diff','--binary'],stderr=subprocess.DEVNULL)).hexdigest()==old['tracked_diff_sha256']
    prior=h.read(r.ROOT/'RESULT.json');h.verify_record(prior)
    assert prior['identity']=='ba385213940beb3bdb05d18f126c3806a3be8369bb5fca15a2a20a0e153755b6'
    for rel,d in prior['evidence'].items():assert h.file_hash(r.ROOT/rel)==d,rel
    for rel,d in prior['implementation'].items():assert h.file_hash(c.s.f.prior.control.V4/rel)==d,rel
    for name in ('ANALYSIS','SENSITIVITY','WALL_ANALYSIS','SUPPLEMENT','DERIVED','FINAL_VERIFICATION'):
        record=h.read(r.ROOT/(name+'.json'));h.verify_record(record)
    retained=h.read(c.ROOT/'RETENTION.json');h.verify_record(retained)
    for rel,d in retained['keep'].items():assert h.file_hash(c.ROOT/rel)==d,rel
    cases={}
    for name in c.NAMES:
        con=c.audit(name);measured=h.read(c.ROOT/name/'MEASUREMENTS.json')
        for step in (800,900,1000):assert c.canonical(c.ROOT/name/'pilot',step)==measured['checkpoints'][str(step)]
        cases[name]=dict(candidate=con['candidate'],frozen=con['frozen'],checkpoints={str(t):measured['checkpoints'][str(t)] for t in (800,900,1000)})
    return dict(prior=prior['identity'],cases=cases,retained_files=len(retained['keep']),unrelated_count=len(old['status']))

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    ROOT.mkdir(exist_ok=False);(ROOT/'sources').mkdir()
    audit=verify();audit['starting_head']=HEAD;audit['free_bytes']=shutil.disk_usage(ROOT).free
    sources={}
    for path in SOURCES:
        data=subprocess.check_output(['wsl','cat',path]);target=ROOT/'sources'/Path(path).name;target.write_bytes(data)
        sources[path]=dict(sha256=h.file_hash(target),lines=len(data.splitlines()))
    exe=subprocess.check_output(['wsl','sha256sum','/opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun'],text=True).split()[0]
    assert exe=='f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e'
    audit['foamRun_sha256']=exe;audit['source_files']=sources
    h.durable_json(ROOT/'ADMISSION.json',h.seal(audit))
    (ROOT/'.gitignore').write_text('scratch/\noutputs/\nbuild/\n*.npz\n')
    # Only exact processor geometry and required snapshots are copied. No decomposition runs.
    for name in c.NAMES:
        src=c.ROOT/name/'pilot';dst=ROOT/'scratch'/name
        for folder in ('constant','system'):shutil.copytree(src/folder,dst/folder)
        for rank in range(6):
            shutil.copytree(src/f'processor{rank}/constant',dst/f'processor{rank}/constant')
            for step in (900,1000):shutil.copytree(src/f'processor{rank}/{step}',dst/f'processor{rank}/{step}')
    files={str(p.relative_to(ROOT/'scratch')):h.file_hash(p) for p in (ROOT/'scratch').rglob('*') if p.is_file()}
    h.durable_json(ROOT/'SCRATCH_INPUTS.json',h.seal(dict(files=files)))
    print('admitted',audit['free_bytes'],flush=True)

if __name__=='__main__':
    import sys
    {'prepare':prepare,'verify':lambda:print(verify())}[sys.argv[1]]()
