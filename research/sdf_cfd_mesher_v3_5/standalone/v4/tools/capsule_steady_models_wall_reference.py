"""Solved y+ on ordinary copies of retained GW1 states. No CFD iteration."""
import shutil
import subprocess
import sys
from pathlib import Path
import capsule_steady_models as s
import capsule_steady_models_wall as wall
h=s.h


def run(name):
    assert name in ('GW1-S1','GW1-S2')
    source=s.OLD/name
    candidate=h.read(source/'CANDIDATE.json');h.verify_record(candidate)
    assert candidate['frozen']==s.f.prior.frozen()
    folder=s.ROOT/'wall_reference'/name
    folder.mkdir(parents=True,exist_ok=False)
    case=folder/'pilot'
    for rel in ('0','constant','system',*[f'processor{i}/{part}' for i in range(6) for part in ('constant','500')]):
        shutil.copytree(source/'pilot'/rel,case/rel)
    (folder/'assembly').mkdir()
    for namepart in ('coordinates','prisms','wall'):
        shutil.copy2(source/'assembly'/f'{namepart}.npy',folder/'assembly'/f'{namepart}.npy')
    files={str(p.relative_to(case)):h.file_hash(p) for p in case.rglob('*') if p.is_file()}
    assert all(p.stat().st_nlink==1 for p in case.rglob('*') if p.is_file())
    assert all(h.file_hash(source/'pilot'/rel)==digest for rel,digest in files.items())
    before=h.seal(dict(schema='ReadOnlyReferenceWallCopy/1',candidate=candidate['identity'],source=str(source),files=files,ordinary_copy=True,new_CFD_iterations=0))
    h.durable_json(folder/'COPY.json',before)
    (folder/'.gitignore').write_text('pilot/\nassembly/\n')
    linux='/mnt/c/'+str(folder.resolve())[3:].replace('\\','/')
    script=folder/'yplus.sh'
    script.write_text(f'''source /opt/openfoam13/etc/bashrc
set -e
cd {linux}
mpirun -np 6 foamPostProcess -solver incompressibleFluid -parallel -case pilot -time 500 -func yPlus > yPlus.log 2>&1
''',newline='\n')
    subprocess.run(['wsl','bash',linux+'/yplus.sh'],check=True)
    for rel,digest in files.items():
        assert h.file_hash(case/rel)==digest,rel
        assert h.file_hash(source/'pilot'/rel)==digest,rel
    wall.analyze(name,folder.parent)
    result=h.seal(dict(schema='ReadOnlyReferenceWallPostprocess/1',copy=before['identity'],yPlus=h.read(folder/'WALL_YPLUS.json')['identity'],original_files_unchanged=True,copy_preexisting_files_unchanged=True,new_CFD_iterations=0,log=h.file_hash(folder/'yPlus.log')))
    h.durable_json(folder/'RESULT.json',result)
    print(result['identity'])


if __name__=='__main__':run(sys.argv[1])
