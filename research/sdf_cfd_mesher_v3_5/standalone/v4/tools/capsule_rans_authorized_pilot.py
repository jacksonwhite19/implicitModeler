"""Bounded non-qualifying pilot. Never retunes or launches qualifying runs."""
import json
import re
import shutil
import subprocess
from pathlib import Path
import capsule_rans_first_aa as prior
from capsule_rans_checks import registry_audit

h=prior.m.h
OUT=prior.m.RESULTS/'capsule_rans_authorized_pilot'
MON=prior.m.RESULTS/'capsule_rans_monitoring_aa'

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()=='aec3afffd3d21ae0e8ebc1d12b88a9290b180811'
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    unrelated=[s for s in status if 'capsule_rans_authorized_pilot' not in s]
    assert len(unrelated)==155
    for filename,want in [('CONTRACT','a4a92d173de9a757b3fee68da2b5e69f2a2f73f2f8df363a2feb6169a6263a2c'),('FAILURE','6c1618cca64853b0f5cc93f22a4e313cc90e1e73805b153362aa0aab20187b3f')]:
        record=h.read(prior.OUT/(filename+'.json'));h.verify_record(record);assert record['identity']==want
    admission=h.read(MON/'ADMISSION.json');h.verify_record(admission)
    authority=h.read(prior.m.OUT/'AUTHORITY.json')
    prior.m.validate(authority,h.read(prior.m.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    access=h.read(prior.m.OUT/'ACCESS.json')
    with prior.m.Store(access['directory']).open(access['root']):pass
    OUT.mkdir(exist_ok=True)
    h.durable_json(OUT/'PREFLIGHT.json',dict(status=unrelated,free_bytes=shutil.disk_usage(OUT).free,epoch_renewal=False,access=access['root']))
    prior.write(OUT/'.gitignore','pilot/\nA/\nA2/\n')
    case=OUT/'pilot'
    if case.exists():raise ValueError('PilotAlreadyExists')
    shutil.copytree(MON/'proposed_controls',case)
    pm=case/'constant/polyMesh'
    shutil.copytree(prior.OUT/'A/constant/polyMesh',pm)
    for name,digest in authority['openfoam']['members'].items():
        if name!='boundary':assert h.file_hash(pm/name)==digest
    assert h.file_hash(pm/'boundary')==h.read(prior.OUT/'MESH_ADAPTER.json')['adapted_boundary']
    control=case/'system/controlDict'
    prior.write(control,control.read_text().replace('endTime 500','endTime 20').replace('writeInterval 500','writeInterval 20'))
    for name in admission['controls']:
        if name!='system/controlDict':assert h.file_hash(case/name)==admission['controls'][name]
    config=h.seal(dict(schema='V4CapsuleAuthorizedPilot/1',status='NON_QUALIFYING',iterations=20,
        length_reason='Fixed short startup/monitor-health segment, not a convergence window or threshold-selection trajectory.',
        mesh=authority['identity'],previous_contract=h.read(prior.OUT/'CONTRACT.json')['identity'],
        warning='AcceptedFoundation13WallGroupNameCollisionWarning',
        allowed_warning="Removing patchGroup 'wall' which clashes with patch 0 of the same name.",
        controls={n:h.file_hash(case/n) for n in admission['controls']},
        adapter=h.read(prior.OUT/'MESH_ADAPTER.json')['identity'],no_hardlinks=True,
        residual_semantics='Record initial and final residual per p,Ux,Uy,Uz,k,omega solve and linear iterations. Qualifying acceptance not yet frozen.',
        force_scale=.5*prior.RHO*prior.U**2*prior.AREA,moment_scale=.5*prior.RHO*prior.U**2*prior.AREA*.16))
    h.durable_json(OUT/'PILOT_CONTRACT.json',config)
    for n in admission['controls']:prior.write(OUT/'controls'/n,(case/n).read_text())
    linux='/mnt/c/'+str(OUT.resolve())[3:].replace('\\','/')
    script=f'''source /opt/openfoam13/etc/bashrc
set -e
cd {linux}
foamToC -solver incompressibleFluid -libs '("libfieldFunctionObjects.so" "libforces.so")' -functionObjects > FUNCTION_REGISTRY.txt
sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun > EXECUTABLE.txt
/usr/bin/time -v -o checkMesh.resources checkMesh -case pilot -constant -allTopology -allGeometry > checkMesh.log 2>&1
grep -q 'Mesh OK.' checkMesh.log
/usr/bin/time -v -o decompose.resources decomposePar -case pilot > decompose.log 2>&1
set +e
/usr/bin/time -v -o pilot.resources mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot > pilot.log 2>&1
code=$?
echo "$code" > pilot.exit
exit "$code"
'''
    prior.write(OUT/'run_pilot.sh',script)
    print(json.dumps(dict(contract=config['identity'],script=linux+'/run_pilot.sh')))

if __name__=='__main__':prepare()
