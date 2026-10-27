"""Close read-only rotation evidence and verify old authorities again."""
import hashlib
import subprocess
import json
import capsule_sa_rotation as r
from capsule_family_seal import allocation
h=r.h;c=r.c

def unchanged():
    audit=r.verify()
    old=h.read(c.ROOT/'UNRELATED_AUDIT.json')
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    assert [s for s in status if 'capsule_sa_rotation' not in s]==old['status']
    diff=subprocess.check_output(['git','diff','--binary'],stderr=subprocess.DEVNULL)
    assert hashlib.sha256(diff).hexdigest()==old['tracked_diff_sha256']
    return audit

def manifest():
    audit=unchanged()
    names={'ANALYSIS.json','SENSITIVITY.json','SUPPLEMENT.json','EXTRACTION.json','WALL_ANALYSIS.json','WALL_EXTRACTION.json','ADMISSION.json','REPLAY.json','wallShearStress.C'}
    files={str(p.relative_to(r.ROOT)):h.file_hash(p) for p in r.ROOT.rglob('*') if p.is_file() and 'scratch' not in p.parts and (p.suffix=='.npz' or p.name in names or p.name.endswith('_wall.log'))}
    record=h.seal(dict(schema='RetainedSARotationData/1',source_audit=audit,files=files,new_CFD_iterations=0,mesh_changes=0,prism_changes=0))
    h.durable_json(r.ROOT/'DATA_MANIFEST.json',record)
    print(record['identity'])

def seal():
    audit=unchanged()
    data=h.read(r.ROOT/'DATA_MANIFEST.json');h.verify_record(data)
    # Scientific arrays, native outputs and primary analysis must remain sealed through cleanup.
    for rel,d in data['files'].items():
        assert h.file_hash(r.ROOT/rel)==d,rel
    whitespace=[]
    code=list((c.s.f.prior.control.V4/'tools').glob('capsule_sa_rotation*.py'))+[c.s.f.prior.control.V4/'tests/test_capsule_sa_rotation.py']
    for p in code:
        for i,line in enumerate(p.read_text().splitlines(),1):
            if line.rstrip(' \t')!=line:whitespace.append((str(p),i))
    assert not whitespace
    tests=dict(new_tests=21,prior_tests=105,unique_total=126,failures=0,
        batches='96 passed (20 new plus 76 prior); 49 passed (20 repeated new plus 29 other prior); final 21 new passed',
        replay=h.read(r.ROOT/'REPLAY.json'),whitespace='CRLF-aware task-owned source lines pass')
    h.durable_json(r.ROOT/'TESTS.json',h.seal(tests))
    storage=h.read(r.ROOT/'STORAGE_FINAL.json')
    report=r.ROOT/'REPORT.md'
    footer='\n## Final verification and storage\n\n'+json.dumps(dict(tests=tests,storage=storage,unrelated_dirty_entries=155,V3_unchanged=True,source_authorities_reverified=True),indent=2)+'\n'
    report.write_text(report.read_text()+footer,encoding='utf-8')
    evidence={str(p.relative_to(r.ROOT)):h.file_hash(p) for p in r.ROOT.rglob('*') if p.is_file() and 'scratch' not in p.parts and p.name not in ('RESULT.json','FINAL_VERIFICATION.json')}
    implementation={str(p.relative_to(c.s.f.prior.control.V4)):h.file_hash(p) for p in code}
    result=h.seal(dict(schema='RetainedSANeutralAzimuthDiagnostic/1',classification='V4CapsuleSteadySARotationAlignmentReducesButDoesNotEliminateLateEvolution',
        starting_head=r.HEAD,prior=audit['prior'],evidence=evidence,implementation=implementation,qualifying=False,
        new_CFD_iterations=0,mesh_changes=0,prism_changes=0,L11_changes=0,stopping_contract=None,
        discrete_residual='DiscreteResidualProjectionNotEvaluated',production_authority_replaced=False,
        next_task='Read-only discrete steady-operator/Jacobian diagnostic with rotation-consistent interpolation to separate rotational near-neutrality from mesh/domain pinning and non-rotational residuals.'))
    h.durable_json(r.ROOT/'RESULT.json',result)
    print(result['identity']);print('report SHA',h.file_hash(report))

if __name__=='__main__':
    import sys
    {'manifest':manifest,'seal':seal}[sys.argv[1]]()
