"""Physical allocation monitoring for a bounded production step."""
import argparse,ctypes,json,shutil,time
from pathlib import Path
from ctypes import wintypes as W
from collections import defaultdict
from path_preserving_deduplication import api,standard,cluster_allocation,cluster_size,metadata,physical_key,file_hash
from qualify_path_preserving_deduplication import TREES,totals,ROOT
from production_single_layer_cycle import OUT,save,resources


def sample():
    k=api();by_id={};temporary={};errors=[]
    for path in OUT.rglob('*'):
        if not path.is_file():continue
        try:
            st=path.stat();h=k.CreateFileW(str(path.resolve()),0,7,None,3,0,None)
            if h==W.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
            try:allocated=cluster_allocation(h,cluster_size(path.anchor))
            finally:k.CloseHandle(h)
            key=(st.st_dev,st.st_ino);by_id[key]=allocated
            if 'working' in path.parts or '.pending' in path.name or path.name.endswith('-journal'):
                temporary[key]=allocated
        except OSError as exc:errors.append(dict(path=str(path),error=str(exc)))
    return dict(time=time.time(),tree_unique_allocated=sum(by_id.values()),temporary_allocated=sum(temporary.values()),
        free_bytes=shutil.disk_usage(ROOT).free,errors=errors)


def monitor():
    with (OUT/'STORAGE_SAMPLES.jsonl').open('a',encoding='utf8',newline='\n') as f:
        while not (OUT/'STOP_MONITOR').exists():
            f.write(json.dumps(sample(),sort_keys=True)+'\n');f.flush();time.sleep(10)


def final():
    started=time.perf_counter()
    before=json.loads((OUT/'DISK_START.json').read_text());known={physical_key(r) for r in before['rows']}
    hashes={};all_rows=[];new_rows=[]
    for tree in [*TREES,OUT]:
        for path in sorted(tree.rglob('*')):
            if not path.is_file():continue
            row=metadata(path);key=physical_key(row)
            if key not in hashes:hashes[key]=file_hash(path)
            row['sha256']=hashes[key];all_rows.append(row)
            if tree==OUT:new_rows.append(row)
    unique={physical_key(r):r for r in new_rows}
    aliases=defaultdict(set)
    for row in new_rows:aliases[physical_key(row)].add(Path(row['path']).relative_to(OUT).parts[0])
    replay_shared={key for key,labels in aliases.items() if {'A','B'}<=labels}
    temporary={physical_key(r):r for r in new_rows if 'working' in Path(r['path']).parts
        or '.pending' in Path(r['path']).name or Path(r['path']).name.endswith('-journal')}
    samples=[json.loads(s) for s in (OUT/'STORAGE_SAMPLES.jsonl').read_text().splitlines()]
    old_now={r['path']:r for r in all_rows if not Path(r['path']).is_relative_to(OUT)}
    assert all((old_now[r['path']]['bytes'],old_now[r['path']]['sha256'])==(r['bytes'],r['sha256']) for r in before['rows'])
    assert all(physical_key(old_now[r['path']])==physical_key(r)
        and old_now[r['path']]['cluster_bytes']==r['cluster_bytes'] for r in before['rows'])
    for row in json.loads((OUT/'INPUT_SEALS.json').read_text()):assert file_hash(row['path'])==row['sha256']
    result=dict(rows=new_rows,relevant_totals=totals(all_rows),task_totals=totals(new_rows),
        new_unique_physical_bytes=sum(r['cluster_bytes'] for k,r in unique.items() if k not in known),
        reused_prior_physical_bytes=sum(r['cluster_bytes'] for k,r in unique.items() if k in known),
        A_B_shared_allocated_bytes=sum(unique[k]['cluster_bytes'] for k in replay_shared),
        A_B_new_shared_allocated_bytes=sum(unique[k]['cluster_bytes'] for k in replay_shared if k not in known),
        A_B_shared_objects=len(replay_shared),
        retained_temporary_allocated=sum(r['cluster_bytes'] for r in temporary.values()),
        added_duplicate_physical_bytes=totals(all_rows)['duplicate_physical_allocation']-before['totals']['duplicate_physical_allocation'],
        ending_free_bytes=shutil.disk_usage(ROOT).free,observed_peak_task_allocated=max(s['tree_unique_allocated'] for s in samples),
        observed_peak_temporary_allocated=max(s['temporary_allocated'] for s in samples),sampling_seconds=10,
        temporary_peak_is_sampled_lower_bound=True,old_payloads_preserved=True,
        old_physical_ids_and_allocation_preserved=True,
        explicit_historical_deletions=[],L4_generated=False,post_L3_L4_forecast=False)
    result['free_bytes_decrease']=before['free_bytes']-result['ending_free_bytes']
    result['unattributed_free_space_difference']=result['free_bytes_decrease']-result['new_unique_physical_bytes']
    result['free_space_note']='Free-space difference includes activity outside the inventoried trees and filesystem metadata; it is not the attribution measure.'
    result.update(seconds=time.perf_counter()-started,**resources())
    save('STORAGE_FINAL.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['monitor','final']);a=p.parse_args();globals()[a.mode]()
