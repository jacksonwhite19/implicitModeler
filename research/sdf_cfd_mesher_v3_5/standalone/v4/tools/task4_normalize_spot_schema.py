"""Preserve and normalize the early, incomplete derived-summary schema only."""
from task4_initial_scan import *
from task3c_io import read
from sdf_cfd_mesher_v4.forecast.detailed.rolling_spots import canonical_spot_record
from sdf_cfd_mesher_v4.common.authority import file_hash
import shutil

if __name__=='__main__':
    archive=OUT/'REPLAY_SCHEMA_REPRODUCER';archive.mkdir(exist_ok=False)
    names=['LAYER1_FUTURE_SPOTS.json','FINAL_VALIDATION.json']
    for name in names:
        pointer=OUT/name;shutil.copy2(pointer,archive/name)
        payload=json.loads(pointer.read_text()).get('payload')
        if payload:shutil.copy2(pointer.parent/payload,archive/Path(payload).name)
    shutil.copy2(ROOT/'tools/task4_replay.py',archive/'launcher_before.py.txt')
    old=read(archive/names[0]);current=[canonical_spot_record(row) for row in old]
    peer=read(OUT/'replay_A2'/names[0])
    assert scientific_hash(current)==scientific_hash(peer)
    for before,after in zip(old,current):
        assert scientific_hash(before)==scientific_hash({k:v for k,v in after.items() if k in before})
    prior=read(archive/names[1]);old_hash=prior['shell_scientific_hash']
    assert old_hash==scientific_hash({k:v for k,v in prior.items() if k!='shell_scientific_hash'})
    assert prior['initial_future_spots_sha256']==scientific_hash(old)
    normalized=dict(prior,initial_future_spots_sha256=scientific_hash(current));normalized.pop('shell_scientific_hash')
    normalized['shell_scientific_hash']=scientific_hash(normalized)
    changed=[k for k in prior if scientific_hash(prior[k])!=scientific_hash(normalized[k])]
    assert sorted(changed)==['initial_future_spots_sha256','shell_scientific_hash']
    write(OUT/names[0],current);write(OUT/names[1],normalized)
    record=dict(cause='Two legacy records omitted an empty derived summary added after their creation. Every underlying exact pair record is identical to A2.',
        added_field='new_earlier_pair_families',derived_values=[row['new_earlier_pair_families'] for row in current],
        primary_raw_hash=old_hash,primary_normalized_hash=normalized['shell_scientific_hash'],
        changed_final_fields=changed,all_other_final_scientific_fields_identical=True,
        pair_records_sha256=scientific_hash([row['records'] for row in current]),
        normalized_spot_records_match_A2=True,checkpoint_sha256=file_hash(OUT/'build_A/LAYER_20.json'),
        preserved_files={p.name:file_hash(p) for p in archive.iterdir() if p.is_file()},
        controller_note='The already running launcher retains the pre-normalization hash in memory. After its build finishes, compare the final artifacts with task4_compare_replay.py; no scientific build needs restarting.')
    write(archive/'REPRODUCER.json',record);print(json.dumps(record,sort_keys=True))
