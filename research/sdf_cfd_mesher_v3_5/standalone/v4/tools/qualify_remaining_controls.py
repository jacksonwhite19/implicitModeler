"""Separate exact A/B and full closure audit; never part of normal timings."""
from pathlib import Path
import capsule_user_domain_handoff as h
import compare_prism_assembly as compare
import audit_persistent_geometry as deep

OUT=Path(__file__).resolve().parents[1]/'results/remaining_prism_critical_path'

if __name__=='__main__':
    compare.OUT=OUT;deep.OUT=OUT
    folders=[OUT/('normal_'+name+'_prism') for name in ('candidate_clean','candidate_clean_replay')]
    for folder in folders:
        assert compare.compare(folder)['passed']
    a,b=folders
    checks={}
    first=h.read(a/'NORMAL.json');second=h.read(b/'NORMAL.json')
    checks['implementation']=first['implementation']==second['implementation']
    checks['controls']=first['controls']==second['controls']
    for name,digest in first['implementation'].items():
        checks['source:'+name]=h.file_hash(Path(__file__).parent/name)==digest
    for sub in ('numeric_delta','numeric_lookup'):
        for path in (a/sub).glob('*.npy'):
            checks[sub+'/'+path.name]=h.file_hash(path)==h.file_hash(b/sub/path.name)
    for name in ('GEOMETRY_DELTA.json','QUALITY_DELTA.json','NEW_ATLAS.json','NEW_RECEIPT_INPUTS.json','VOLUME_DELTA.json'):
        checks[name]=h.file_hash(a/name)==h.file_hash(b/name)
    for folder in folders:
        index=h.read(folder/'INDEX_PIPELINE.json')
        checks[folder.name+'_index']=index['sha256']=='ee85cf43f84edc67e42ecc841b32e4e09a936b315c8c2b98c5fb5de13de974ea'
        db=folder/'candidate/objects/04308917005ab305e3d3750076c8c03944a7090fd3885e1601ffb76f49cb973f.sqlite'
        checks[folder.name+'_quality']=db.exists() and h.file_hash(db)==db.stem
    h.durable_json(OUT/'DETERMINISTIC_REPLAY.json',dict(passed=all(checks.values()),checks=checks,
        implementation=first['implementation'],implementation_identity=h.identity(first['implementation'])))
    assert all(checks.values())
    deep.audit('candidate_clean')
