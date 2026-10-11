"""Evidence closure for the exact G1/W1 pair. No production promotion."""
import json
import shutil
import subprocess
import capsule_gw_pair as g
from capsule_family_seal import allocation
h=g.h


def storage(label):
    r=dict(free_bytes=shutil.disk_usage(g.ROOT).free,total=allocation(g.ROOT),variants={n:allocation(g.ROOT/n) for n in g.NAMES},method='NTFS FileStandardInfo AllocationSize; one link required')
    h.durable_json(g.ROOT/('STORAGE_'+label+'.json'),r);print(json.dumps(r))


def seal():
    pre=h.read(g.ROOT/'PREFLIGHT.json');decision=h.read(g.ROOT/'DECISION.json');h.verify_record(decision)
    assert g.f.prior.frozen()==pre['frozen']
    reused=h.read(g.ROOT/'REUSED_IMPLEMENTATIONS.json');h.verify_record(reused)
    for name,digest in reused.items():
        if name!='identity':assert h.file_hash(g.f.prior.control.V4/'tools'/name)==digest
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==g.HEAD
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=lambda rows:[r for r in rows if 'capsule_gw_' not in r]
    assert unrelated(status)==unrelated(pre['status']) and len(unrelated(status))==155
    for rel,identity in pre['authorities'].items():
        r=h.read(g.f.prior.control.V4/rel);h.verify_record(r);assert r['identity']==identity
        for file,digest in r.get('evidence',{}).items():
            if isinstance(digest,str) and len(digest)==64:assert h.file_hash((g.f.prior.control.V4/rel).parent/file)==digest
    for name in g.NAMES:
        d=g.ROOT/name;c=h.read(d/'CANDIDATE.json');h.verify_record(c)
        assert (d/'EXECUTABLE.txt').read_bytes()==(g.PREVIOUS/'G1/EXECUTABLE.txt').read_bytes()
        for rel,digest in c['evidence'].items():assert h.file_hash(d/rel)==digest
        foundation=h.read(d/'FOUNDATION.json')
        for key,stem in [('conversion','gmshToFoam'),('audit','foundationAudit'),('check','checkMesh')]:
            assert foundation[key]['exit_code']==0
            for suffix in ('stdout','stderr'):assert h.file_hash(d/'foundation'/(stem+'.'+suffix))==foundation[key][suffix+'_sha256']
        assert 'Mesh OK.' in (d/'foundation/checkMesh.stdout').read_text()
        assert 'Mesh OK.' in (d/'adapted_checkMesh.log').read_text()
        m=h.read(d/'MEASUREMENTS.json');h.verify_record(m)
        assert h.file_hash(d/'HISTORY.json')==m['history'] and h.file_hash(d/'pilot.log')==m['log']
        assert (d/'pilot.exit').read_text().strip()=='0'
        con=h.read(d/'CFD_CONTRACT.json');h.verify_record(con)
        old=h.read(g.PREVIOUS/'G1/CFD_CONTRACT.json');assert con['controls']==old['controls']
        for rel,digest in con['controls'].items():assert h.file_hash(d/'pilot'/rel)==digest
        source=d/'foundation/openfoam_case/constant/polyMesh';mesh=d/'pilot/constant/polyMesh'
        for member in ('points','faces','owner','neighbour'):assert h.file_hash(source/member)==h.file_hash(mesh/member)
        assert (source/'boundary').read_text().replace(con['adapter']['before'],con['adapter']['after'])==(mesh/'boundary').read_text()
    dense=h.read(g.ROOT/'DENSE_WAKE.json');h.verify_record(dense)
    for name,v in dense['variants'].items():assert h.file_hash(g.ROOT/'wake'/(name+'.npz'))==v['archive_sha256']
    sources={str(p.relative_to(g.f.prior.control.V4)):h.file_hash(p) for p in (g.f.prior.control.V4/'tools').glob('capsule_gw_*') if p.is_file()}
    sources['tests/test_capsule_gw_pair.py']=h.file_hash(g.f.prior.control.V4/'tests/test_capsule_gw_pair.py')
    evidence={str(p.relative_to(g.ROOT)):h.file_hash(p) for p in g.ROOT.rglob('*') if p.is_file() and not any(part in ('pilot','assembly','foundation','W1','wake') for part in p.relative_to(g.ROOT).parts) and p.name not in ('RESULT.json','seal.log')}
    r=h.seal(dict(schema='V4G1PlusW1PairResult/1',classification=decision['classification'],starting_head=g.HEAD,prism_changes=0,URANS_runs=0,Run_A=False,Run_A2=False,production_authority_replaced=False,frozen=pre['frozen'],decision=decision,evidence=evidence,implementation=h.seal(sources),unrelated_dirty_entries=155,V3_changed=False))
    h.durable_json(g.ROOT/'RESULT.json',r);print(r['identity'])


if __name__=='__main__':
    import sys
    if sys.argv[1]=='seal':seal()
    else:storage(sys.argv[1])
