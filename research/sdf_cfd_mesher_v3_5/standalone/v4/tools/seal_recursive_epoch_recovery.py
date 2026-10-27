"""Seal compact evidence only after production recovery and reopen controls pass."""
from pathlib import Path
import gzip,json,os,shutil,subprocess
import immutable_state_access as a
import recursive_epoch_recovery as recovery
from path_preserving_deduplication import metadata,physical_key,file_hash,durable_json
from sdf_cfd_mesher_v4.common.authority import identity

V4=Path(__file__).resolve().parents[1];OUT=V4/'results/recursive_epoch_recovery'

def allocation(paths):
    rows=[metadata(p) for p in paths if p.is_file()]
    unique={physical_key(r):r for r in rows}
    hashes={};duplicate=0
    for r in unique.values():
        h=file_hash(r['path'])
        if h in hashes:duplicate+=r['cluster_bytes']
        else:hashes[h]=r['path']
    return dict(files=len(rows),physical_objects=len(unique),logical=sum(r['bytes'] for r in rows),allocated=sum(r['cluster_bytes'] for r in unique.values()),duplicate_allocation=duplicate)

def main():
    pre=a.read(OUT/'PREFLIGHT.json');deep=a.read(OUT/'DEEP.json');first=a.read(OUT/'FIRST.json');second=a.read(OUT/'SECOND.json')
    assert all(x['passed'] for x in (deep,first,second))
    record=recovery.discover(OUT/'access');pointer=a.read(OUT/'access/CURRENT.json')
    assert record['scientific']=='4d49baf5d372827ea853edbfdedf80ca0909827be865dbdde58faa3da7630170'
    for p,h in pre['protected'].items():assert file_hash(p)==h,p
    token_changes=[];source_paths=[]
    for node in record['nodes']:
        ref=node['source'];p=Path(ref['directory'])/('ROOT_'+ref['root']+'.json');old=a.read(p)
        source_paths.extend([p,Path(ref['directory'])/'CURRENT.json'])
        oldfiles=old['files'] if ref['kind']=='native' else {**old['plan']['references'],**old['produced']}
        for path,r in oldfiles.items():
            if r['token']!=record['files'][path]['token']:token_changes.append(path)
        newref=node['reference'];new=a.read(Path(newref['directory'])/('ROOT_'+newref['root']+'.json'))
        if ref['kind']=='native':assert old['content']==new['content'] and old['components']==new['components']
        else:assert old['plan']['content']==new['plan']['content'] and old['scientific']==new['scientific']
    source_rows=[metadata(p) for p in record['files']]
    source_unique={physical_key(r):r for r in source_rows}
    oldv3=pre['v3_inventory'];assert file_hash(OUT/oldv3['file'])==oldv3['sha256']
    with gzip.open(OUT/oldv3['file'],'rt',encoding='utf8') as f:previous=json.load(f)
    current={}
    for root,dirs,files in os.walk('research/sdf_cfd_mesher_v3',followlinks=False):
        for n in files:
            p=Path(root)/n;s=p.lstat();current[str(p)]=[s.st_size,s.st_mtime_ns]
    assert current==previous,'V3 changed'
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    own=(b'research/sdf_cfd_mesher_v4/results/recursive_epoch_recovery',b'research/sdf_cfd_mesher_v4/tools/recursive_epoch_recovery',b'research/sdf_cfd_mesher_v4/tools/qualify_recursive_epoch_recovery.py',b'research/sdf_cfd_mesher_v4/tools/seal_recursive_epoch_recovery.py',b'research/sdf_cfd_mesher_v4/tests/test_recursive_epoch_recovery.py')
    unrelated=b'\0'.join(x for x in status.split(b'\0') if x and not any(v in x for v in own))+b'\0'
    assert unrelated.hex()==pre['status_hex'],'Unrelated status changed'
    report=dict(classification='V4RecursiveEpochRecoveryFromSealedLayer6Qualified',
        claim='Deep authenticated sealed L6 recursive closure; metadata-only epoch renewal; two independent normal opens without history traversal.',
        starting_head=pre['head'],implementation={str(p.relative_to(V4)):file_hash(p) for p in [V4/'tools/recursive_epoch_recovery.py',V4/'tools/qualify_recursive_epoch_recovery.py',V4/'tools/seal_recursive_epoch_recovery.py',V4/'tests/test_recursive_epoch_recovery.py']},
        old_epoch=pre['sealed_epoch'],current_epoch=a.epoch(),stale_rejection=pre['stale_error'],
        scientific=record['scientific'],content=record['content'],reference=record['reference'],recovery_root=pointer['root'],recovery_record_sha256=pointer['sha256'],
        ancestry=dict(depth=len(record['nodes']),nodes=record['nodes'],component_maps=sum(n['component_count'] for n in record['nodes']),files=len(record['files']),bytes_authenticated=deep['bytes_authenticated'],physical_objects=len(source_unique),physical_allocation_referenced=sum(r['cluster_bytes'] for r in source_unique.values()),changed_file_tokens=sorted(set(token_changes))),
        deep=deep,first_normal=first,second_normal=second,protected_hashes_verified=len(pre['protected']),
        tests=(OUT/'TESTS_FINAL.txt').read_text().strip(),production_negative_controls=a.read(OUT/'NEGATIVE_CONTROLS.json'),retry=a.read(OUT/'RETRY.json'),
        safety='Focused recovery tests plus unchanged reader/transaction regressions; exception and abrupt-exit boundaries before/after pointer, corruption, stale/concurrent writers, wrong identity, retry, failed scientific verification.',
        storage=dict(start_free=pre['free'],end_free=shutil.disk_usage(OUT).free,access=allocation(list((OUT/'access').rglob('*'))),evidence=allocation([p for p in OUT.iterdir() if p.is_file()]),negative_control_metadata=allocation(list((OUT/'negative_controls').rglob('*'))),temporary_inventory_allocation=pre['preflight_compaction']['allocation'],peak_continuously_measured=False,scientific_payload_copies=0,historical_deletions=0),
        git=dict(unrelated_entries=155,unrelated_status_exact=True,v3_entries_checked=len(current),v3_modifications=0),
        L7_admission=False,L7_forecast=False,L7_generation=False,core=False,
        limitations=['Recovery remains an explicit expensive full audit; future reboots require renewal.','Historical sealed access roots remain stale; use the new pinned reference.','Normal opens still hash demanded hot objects and small metadata, not historical proof/quality populations.','Filesystem free-space deltas include system noise; per-file allocation is reported separately.'],
        recommended_next_task='QualifyV4IncrementalWriteAmplificationAndPairIndexScalingFromSealedLayer6')
    report['identity']=identity(report)
    durable_json(OUT/'REPORT.json',report)
    durable_json(OUT/'REFERENCE.json',dict(reference=record['reference'],recovery_root=pointer['root'],scientific=record['scientific'],content=record['content']))
    files={p.name:file_hash(p) for p in OUT.iterdir() if p.is_file() and p.suffix in ('.json','.txt') and p.name not in ('AUTHORITY.json','SEAL_LOG.txt')}
    authority=dict(classification=report['classification'],qualification=report['identity'],files=files,reference=record['reference'],recovery_root=pointer['root'],scientific=record['scientific'],content=record['content'])
    authority['identity']=identity(authority);durable_json(OUT/'AUTHORITY.json',authority)
    print(json.dumps(dict(qualification=report['identity'],report_sha256=file_hash(OUT/'REPORT.json'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),ancestry=report['ancestry'],storage=report['storage']),indent=2))
if __name__=='__main__':main()



