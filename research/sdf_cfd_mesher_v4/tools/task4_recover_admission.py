"""Preserve the admission-defect run and recover the unaffected checkpoint.

This moves only enumerated Task-4 artifacts within the Task-4 results root.
It does not delete scientific evidence or alter Git state.
"""
from task4_correct_first import *
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json
import re,shutil

def recover():
    root=OUT.resolve();archive=root/'development_A0'
    if archive.exists():raise RuntimeError('RecoveryAlreadyStartedInspectManifest')
    assert root.name=='task4' and root.is_relative_to((ROOT/'results').resolve())
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule()
    old=CommitStore(root/'build_A',front,top,schedule).restart();assert old.layer==4
    initial=read(root/'CORRECTION_HISTORY.json')
    assert all(not row.get('new_entity_failures') for row in initial['history'] if row.get('accepted'))
    from sdf_cfd_mesher_v4.forecast.detailed.rolling_spots import earlier_pair_families
    assert all(not earlier_pair_families(s['records']) for s in read(root/'LAYER1_FUTURE_SPOTS.json'))
    selected=[]
    for path in sorted(root.iterdir()):
        if (path.name=='build_A' or re.match(r'(LAYER[2-5]_|LAYER_0[2-5]_|certificate_layer_0[2-5]$|build_layer_0[2-5]_|VERIFICATION_LAYER_02)',path.name)
            or path.name in ('BUILD_OPERATIONS.json','BUILD_OPERATIONS_INITIAL.json','rolling_build.log','rolling_resume.log','resume.log')):
            assert path.resolve().is_relative_to(root);selected.append(path)
    manifest={}
    for path in selected:
        for f in ([path] if path.is_file() else sorted(path.rglob('*'))):
            if f.is_file():manifest[str(f.relative_to(root)).replace('\\','/')]=file_hash(f)
    archive.mkdir();atomic_json(archive/'ARCHIVE_MANIFEST.json',dict(previous_layer=old.layer,previous_state=old.state_hash,
        reason='New earlier entity events were incorrectly restricted to the verification ring',
        retained_policy_valid_layer=1,files=manifest,shared_unchanged=['INITIAL_FORECAST.npz','certificate_layer_01','LAYER1_CANDIDATE.npz']))
    for path in selected:
        destination=(archive/path.name).resolve();assert destination.is_relative_to(root) and not destination.exists()
        shutil.move(str(path),str(destination))
    assert all(file_hash(archive/name)==h for name,h in manifest.items())
    recovered=root/'build_A';recovered.mkdir()
    for name in ('LAYER_00.json','LAYER_01.json','FRONT_01.npy'):
        shutil.copyfile(archive/'build_A'/name,recovered/name)
    atomic_json(recovered/'CURRENT.json',dict(layer=1,checkpoint='LAYER_01.json',sha256=file_hash(recovered/'LAYER_01.json')))
    current=CommitStore(recovered,front,top,schedule).restart()
    assert current.state_hash=='dc9f120e08913585539cd986dbd5fe94096933d03e0f5e4213b2e62816d13620'
    atomic_json(root/'ADMISSION_RECOVERY.json',dict(passed=True,archived_file_count=len(manifest),
        archived_bytes=sum((archive/name).stat().st_size for name in manifest),
        previous_layer=4,last_policy_valid_layer=1,recovered_state_hash=current.state_hash,
        all_archived_bytes_verified=True,layer1_coordinates_and_proofs_reused=True))
    print('RECOVERED',current.layer,current.state_hash,flush=True)

if __name__=='__main__':recover()
