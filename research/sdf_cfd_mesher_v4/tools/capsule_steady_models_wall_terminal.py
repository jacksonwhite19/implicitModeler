"""Terminal wall diagnostic after a successful new SST run; no CFD steps."""
import subprocess
import sys
import capsule_steady_models as s
import capsule_steady_models_wall as wall
h=s.h


def run(name):
    assert name in ('GC2F-S1','GC2F-S2')
    folder=s.ROOT/name;case=folder/'pilot'
    assert (folder/'pilot.exit').read_text().strip()=='0'
    contract=h.read(folder/'CFD_CONTRACT.json');h.verify_record(contract)
    for rel,digest in contract['controls'].items():assert h.file_hash(case/rel)==digest
    files={str(p.relative_to(case)):h.file_hash(p) for rank in range(6) for p in (case/f'processor{rank}/500').rglob('*') if p.is_file()}
    assert len(files)>30
    linux='/mnt/c/'+str(folder.resolve())[3:].replace('\\','/')
    script=folder/'yplus.sh'
    script.write_text(f'''source /opt/openfoam13/etc/bashrc
set -e
cd {linux}
mpirun -np 6 foamPostProcess -solver incompressibleFluid -parallel -case pilot -time 500 -func yPlus > yPlus.log 2>&1
''',newline='\n')
    subprocess.run(['wsl','bash',linux+'/yplus.sh'],check=True)
    for rel,digest in files.items():assert h.file_hash(case/rel)==digest,rel
    for rel,digest in contract['controls'].items():assert h.file_hash(case/rel)==digest
    wall.analyze(name)
    result=h.seal(dict(schema='NewSteadyCaseWallPostprocessAudit/1',contract=contract['identity'],iteration=500,preexisting_files=files,preexisting_files_unchanged=True,controls_unchanged=True,new_CFD_iterations=0,yPlus=h.read(folder/'WALL_YPLUS.json')['identity'],log=h.file_hash(folder/'yPlus.log')))
    h.durable_json(folder/'WALL_POSTPROCESS_AUDIT.json',result);print(result['identity'])


if __name__=='__main__':run(sys.argv[1])
