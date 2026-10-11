"""Independent semantic comparison and isolated publication of performance controls."""
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
import capsule_user_domain_handoff as h
from profile_meshing_hotpath import OUT, SEALED


def certificate_reopen():
    tick=h.measured();digest=hashlib.sha256();digest.update(b'[');count=0
    path=OUT/'raw_release_optimized_certificate/PROOFS.jsonl.gz'
    with gzip.open(path,'rb') as stream:
        for line in stream:
            if count:digest.update(b',')
            digest.update(line.rstrip(b'\r\n'));count+=1
    digest.update(b']')
    oracle=h.read(SEALED/'CONTINUOUS.json')
    result=dict(passed=count==oracle['required_pairs'] and digest.hexdigest()==oracle['proof_hash'],
        rows=count,proof_hash=digest.hexdigest(),file_sha256=h.file_hash(path),metrics=h.metrics(*tick))
    h.durable_json(OUT/'CERTIFICATE_REOPEN.json',result)
    assert result['passed']


def publish():
    from incremental_transaction import Store
    import immutable_state_access as access
    tick=h.measured();old=h.read(SEALED/'COMPLETE_AUTHORITY.json');h.verify_record(old)
    assert h.read(OUT/'CERTIFICATE_REOPEN.json')['passed']
    assert h.read(OUT/'raw_release_optimized_certificate/RESULT.json')['exact_oracle_equal']
    for name in ('optimized_core','raw_release_optimized_core'):
        assert all(h.read(OUT/name/'EQUALITY.json').values())
    with Store(SEALED/'complete_mesh').open(old['access']) as body:
        prior=json.loads(json.dumps(body['plan']))
    components=dict(prior['components'])
    references=dict(prior['references'])
    replacement=OUT/'raw_release_optimized_certificate/PROOFS.jsonl.gz'
    components['NEW_PROOFS.jsonl.gz']=h.file_hash(replacement)
    references.pop(str((SEALED/'NEW_PROOFS.jsonl.gz').resolve()))
    references[str(replacement.resolve())]=access.attest(replacement,components['NEW_PROOFS.jsonl.gz'])
    patch=h.read(SEALED/'CORE_INTERFACE_BINDINGS.json')
    sizing=h.read(SEALED/'SIZING_AUTHORITY.json')
    scientific=h.identity(dict(schema='V4CapsuleCompleteCore/1',parent=h.EXPECTED['scientific'],
        sizing=sizing['identity'],assembly=h.read(SEALED/'ASSEMBLY.json')['hashes'],
        new_proofs=h.read(OUT/'raw_release_optimized_certificate/RESULT.json')['proof_hash'],interface=h.identity(patch),
        core_cells=190387,prism_cells=371646))
    assert scientific==old['scientific']
    plan=dict(prior,scientific=scientific,components=components,references=references,
        implementation=h.file_hash(__file__),control_only=True)
    implementation={p.name:h.file_hash(p) for p in Path(__file__).parent.glob('optimized_*') if p.is_file()}
    plan['optimized_implementation']=implementation
    for name,sha in implementation.items():
        p=Path(__file__).parent/name
        plan['references'][str(p.resolve())]=access.attest(p,sha)
    store=Store(OUT/'publication_release_control')
    base=store.initialize(dict(epoch=access.epoch(),scientific=h.EXPECTED['scientific'],
        plan=dict(references={},parent=old['parent']),produced={}))
    acceptance=dict(passed=True,base=base,plan=h.identity(plan),quality_delta=h.identity([]),
        quality_count=0,binding_patch=h.identity(patch))
    root=store.publish(base,plan,[],patch,acceptance,h.identity(acceptance))
    assert store.publish(base,plan,[],patch,acceptance,h.identity(acceptance))==root
    h.durable_json(OUT/'PUBLICATION_RELEASE_CONTROL.json',dict(scientific=scientific,content=h.identity(components),
        access=root,checkpoint=h.file_hash(store.path/('ROOT_'+root+'.json')),stats=dict(store.stats),metrics=h.metrics(*tick)))


def reopen():
    from incremental_transaction import Store
    tick=h.measured();record=h.read(OUT/'PUBLICATION_RELEASE_CONTROL.json')
    with Store(OUT/'publication_release_control').open(record['access']) as state,h.guards(),h.CommittedState(state['plan']['parent']) as parent:
        parent.check();assert state['scientific']==record['scientific']
        assert state['plan']['implementation']==h.file_hash(__file__)
    h.durable_json(OUT/'PUBLICATION_RELEASE_REOPEN.json',dict(passed=True,scientific=record['scientific'],
        historical_proof_rows=0,historical_quality_rows=0,metrics=h.metrics(*tick)))


def prism(mode='raw_release_optimized'):
    oracle=Path('research/sdf_cfd_mesher_v4/results/recursive_layers9to11/L11')
    a=OUT/'baseline_prism';b=OUT/(mode+'_prism');checks={}
    for name in ('GENERATED_NEXT.npz','LOCALIZATION.npz','SCREEN_ARRAYS.npz','FORECAST.npz'):
        x=np.load(a/name);y=np.load(b/name)
        checks[name]=x.files==y.files and all(x[k].tobytes()==y[k].tobytes() for k in x.files)
    for name in ('GEOMETRY_DELTA.json','CHANGED_INPUTS.json','NEW_QUALITY.json','AFFECTED_INCIDENCE.json','VOLUME_DELTA.json'):
        checks[name]=h.read(a/name)==h.read(b/name)==h.read(oracle/name)
    ca=h.read(a/'FRESH_CERTIFICATE.json');cb=h.read(b/'FRESH_CERTIFICATE.json');co=h.read(oracle/'FRESH_CERTIFICATE.json')
    for key in ('proof_hash','methods','required_pairs','failures','segment'):
        checks['certificate_'+key]=ca[key]==cb[key]==co[key]
    # All geometry, quality and proof segment bytes, including interned codes,
    # match the sealed layer. Timing/report/transaction paths are excluded.
    publication=h.read(b/'PUBLICATION.json')
    checks['content_root']=publication['content_root']=='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870'
    sources=[h.read(b/(phase+'_IMPLEMENTATION.json')) for phase in ('admission','generation','assembly','publication')]
    checks['same_implementation_all_phases']=all(s==sources[0] for s in sources)
    h.durable_json(OUT/'PRISM_EQUIVALENCE.json',dict(passed=all(checks.values()),checks=checks))
    assert all(checks.values())


def prism_reopen():
    from composable_state import CommittedState
    folder=OUT/'raw_release_optimized_prism';publication=h.read(folder/'PUBLICATION.json');tick=h.measured()
    reference=dict(kind='recursive',directory=str((folder/'candidate').resolve()),root=publication['root'])
    with h.guards(),CommittedState(reference) as state:
        state.check();assert state.layer==11
        assert state.content_root=='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870'
        result=dict(passed=True,content=state.content_root,access=state.root,stats=dict(state.stats),metrics=h.metrics(*tick))
    h.durable_json(OUT/'PRISM_REOPEN.json',result)


if __name__=='__main__':
    import sys
    {'certificate':certificate_reopen,'publish':publish,'reopen':reopen,'prism':prism,'prism_reopen':prism_reopen}[sys.argv[1]]()
