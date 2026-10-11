"""Seal two already-qualified independent milestones and preservation checks."""
from production_bounded_layers import *

def seal():
    layers={str(k):read(OUT/f'L{k}/QUALIFIED.json') for k in (4,5)}
    for k,q in layers.items():
        assert q['passed'] and q['retention_verified']
        for name,digest in q['evidence'].items():assert file_hash(OUT/f'L{k}'/name)==digest
    historical_l4=read(OUT/'L4/A/AFTER_L5.json')
    assert historical_l4['passed'] and historical_l4['scientific']==layers['4']['committed']
    preserved=[]
    for name,digest in read(RET/'AUTHORITY.json')['evidence'].items():
        p=RET/name;assert file_hash(p)==digest;preserved.append(dict(path=str(p),sha256=digest))
    for p,digest in read(L3/'AUTHORITY.json')['scientific']['implementation'].items():
        assert file_hash(V4/p)==digest
    for name,digest in [('AUTHORITY.json','b14ee357feda1394980a48902c61f90e83a428ec8ecaad085d4069b6bc037fbe'),('report.md','3c8aabf822c1538e69642a5373f75dc3d1949fc3137fc48fb1e5d36cce4dc7a6')]:
        assert file_hash(L3/name)==digest;preserved.append(dict(path=str(L3/name),sha256=digest))
    save(OUT/'PROTECTED_AUTHORITY_VERIFICATION.json',dict(passed=True,rows=preserved))
    old=bytes.fromhex(read(OUT/'BASELINE.json')['status_hex']).split(b'\0');old=[r for r in old if r]
    current=[r for r in subprocess.check_output(['git','status','--porcelain=v1','-z']).split(b'\0') if r]
    assert len(old)==155 and all(r in current for r in old)
    additional=[r for r in current if r not in old]
    assert all(b'research/sdf_cfd_mesher_v4/' in r for r in additional)
    assert subprocess.check_output(['git','diff',START,'--','research/sdf_cfd_mesher_v3'])==b''
    save(OUT/'GIT_PRESERVATION.json',dict(starting_head=START,original_dirty_entries=len(old),original_status_preserved=True,V3_task_modifications=0,
        additional_status=[r.decode() for r in additional]))
    science=dict(classification='V4ProductionAdaptiveCapsuleAdvancingFrontLayers4To5Qualified',
        claim='L4 and L5 independently admitted, certified, committed, replayed, durably restored and retained. Complete advancing-front qualification remains incomplete.',
        starting_head=START,layers={k:dict(identity=q['identity'],committed=q['committed'],compact=q['scientific']['compact_identity']) for k,q in layers.items()},
        scope=dict(authoritative_layer=5,L6_generated=False,post_L5_L6_forecast=False,core_generated=False,complete_advancing_front_qualified=False),
        report_sha256=file_hash(OUT/'REPORT.md'),
        implementation={str(p.relative_to(V4)):file_hash(p) for p in (V4/'tools').glob('*bounded*.py')})
    evidence={str(p.relative_to(OUT)):file_hash(p) for p in OUT.rglob('*') if p.is_file() and p.suffix in ('.json','.xml','.md') and 'checkpoints' not in p.parts and 'RECEIPT_RETIREMENTS' not in p.parts and 'regenerated' not in p.parts and p.name not in ('AUTHORITY.json','QUALITY.json')}
    save(OUT/'AUTHORITY.json',dict(identity=identity(science),scientific=science,evidence=evidence))
    print(json.dumps(dict(identity=identity(science),report_sha256=science['report_sha256'],authority_sha256=file_hash(OUT/'AUTHORITY.json')),indent=2))

if __name__=='__main__':seal()
