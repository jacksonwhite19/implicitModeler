"""Complete-core component publication using the qualified delta transaction."""
import json
from pathlib import Path
import numpy as np
import capsule_user_domain_handoff as h
import immutable_state_access as access
from incremental_transaction import Store
from qualify_capsule_c1_core import OUT, G, HMAX

def validate_sizing(record):
    h.verify_record(record)
    h.need(record['g']==G and record['g_hex']==G.hex(),'WrongC1Growth')
    h.need(record['h_max']==HMAX and record['h_max_hex']==HMAX.hex(),'WrongC1Cap')
    h.need(record['regional_caps']==[] and not record['universal_default'],'WrongC1Scope')
    h.eq(record['l11'],h.EXPECTED['identity'],'WrongL11')
    h.eq(record['domain'],'63c749602dfd1f0ae481a640127249b7fc031f894cabf31c565cd0163c3eec43','WrongDomain')
    h.eq(record['handoff'],'55e31c4497e0629b7951f90347a555fbb3510755c00d531d0fe4d6bdf9fe44c5','WrongHandoff')
    h.eq(record['architecture'],'d31aaa9068836db45b70381e6cd16404ccf825cb84ae7bbcd918b9e7b8fb0d7a','WrongSizingArchitecture')
    h.eq(record['implementation'],'6c7fb1f68cb7326800e92cee15362701f485af8f161a79ae7feb6b234daf8377','WrongSizingImplementation')
    h.eq(record['status'],'CapsuleCaseProductionSizingControl','WrongSizingStatus')

def gates(root):
    sizing=h.read(root/'SIZING_AUTHORITY.json');validate_sizing(sizing)
    for filename in ('CONTINUOUS.json','CERTIFICATE_REOPEN.json','FOUNDATION.json','WHOLE_VOLUME.json','REPLAY_GEOMETRY.json','REPLAY_EQUIVALENCE.json'):
        h.need(h.read(root/filename)['passed'],'QualificationGate:'+filename)
    certificate=h.read(root/'CONTINUOUS.json')
    h.need(not certificate['failures'],'ContinuousFailures')
    h.eq(certificate['immutable_prism_proofs_reused'],33336032,'WrongInheritedCertificate')
    accounting=h.read(root/'BINDING_ACCOUNTING.json')
    h.eq(accounting['changed']+accounting['unchanged'],33336032,'IncompleteBindingAccounting')
    h.eq(accounting['prior_pair_rows_scanned'],0,'HistoricalPairScan')
    h.need(not any(h.read(root/'C1/NETGEN_AUDIT.json')['failures'].values()),'QualityGate')
    return sizing

def publish():
    tick=h.measured();sizing=gates(OUT)
    authority,_,_,_,_=h.verify_sources()
    with h.guards(),h.CommittedState(authority['reference']) as parent:parent.check()
    rows=h.read(h.OUT/'INTERFACE.json');owners=np.load(OUT/'C1/FINAL_CORE_OWNERS.npy')
    patch={r['face']:dict(prism_owner=r['prism_owner'],core_neighbour='core:'+str(int(owner)),
        vertices=r['vertices'],opposite_winding=True) for r,owner in zip(rows,owners)}
    h.need(len(patch)==33786,'IncompleteJoin')
    h.durable_json(OUT/'CORE_INTERFACE_BINDINGS.json',patch)
    components={}
    paths=list((OUT/'assembly').glob('*.npy'))+list((OUT/'foundation/foundation_audit').iterdir())
    paths+=list((OUT/'foundation/openfoam_case/constant/polyMesh').iterdir())
    paths+=[OUT/n for n in ('SIZING_AUTHORITY.json','CONTINUOUS.json','NEW_PROOFS.jsonl.gz',
        'CORE_INTERFACE_BINDINGS.json','BINDING_ACCOUNTING.json','C1/NETGEN.npz','C1/OUTER.npz','C1/NETGEN_AUDIT.json','FOUNDATION.json','WHOLE_VOLUME.json')]
    for p in paths:
        if p.is_file():components[str(p.relative_to(OUT))]=h.file_hash(p)
    scientific=h.identity(dict(schema='V4CapsuleCompleteCore/1',parent=h.EXPECTED['scientific'],
        sizing=sizing['identity'],assembly=h.read(OUT/'ASSEMBLY.json')['hashes'],
        new_proofs=h.read(OUT/'CONTINUOUS.json')['proof_hash'],interface=h.identity(patch),
        core_cells=190387,prism_cells=371646))
    references={str((OUT/name).resolve()):access.attest(OUT/name,sha) for name,sha in components.items()}
    plan=dict(schema='V4CapsuleCompleteCorePlan/1',scientific=scientific,parent=authority['reference'],
        references=references,components=components,old_prism_proofs=33336032,
        implementation=h.file_hash(__file__),
        proof_scope='Inherited unchanged prism geometry plus complete new unique-face relations')
    store=Store(OUT/'complete_mesh')
    base=store.initialize(dict(epoch=access.epoch(),scientific=h.EXPECTED['scientific'],
        plan=dict(references={},parent=authority['reference']),produced={}))
    acceptance=dict(passed=True,base=base,plan=h.identity(plan),quality_delta=h.identity([]),
        quality_count=0,binding_patch=h.identity(patch))
    root=store.publish(base,plan,[],patch,acceptance,h.identity(acceptance))
    h.eq(store.publish(base,plan,[],patch,acceptance,h.identity(acceptance)),root,'RetryMismatch')
    record=h.seal(dict(classification='V4CapsuleCorePublishedPendingFreshReopen',scientific=scientific,
        content=h.identity(components),access=root,checkpoint=h.file_hash(store.path/('ROOT_'+root+'.json')),
        sizing=sizing['identity'],parent=authority['reference'],metrics=h.metrics(*tick),transaction=dict(store.stats)))
    h.durable_json(OUT/'COMPLETE_AUTHORITY.json',record)

def reopen():
    tick=h.measured();record=h.read(OUT/'COMPLETE_AUTHORITY.json');h.verify_record(record)
    store=Store(OUT/'complete_mesh')
    with store.open(record['access']) as body,h.guards(),h.CommittedState(record['parent']) as parent:
        h.eq(body['scientific'],record['scientific'],'WrongScientificRoot')
        h.eq(body['plan']['implementation'],h.file_hash(__file__),'WrongCoreImplementation')
        h.eq(parent.content_root,h.EXPECTED['content'],'WrongPrismComponentScientificRoot')
        h.eq(parent.layer,11,'WrongPrismLayer')
        validate_sizing(h.read(OUT/'SIZING_AUTHORITY.json'));parent.check()
        x=np.load(OUT/'assembly/coordinates.npy',mmap_mode='r')
        p=np.load(OUT/'assembly/prisms.npy',mmap_mode='r');t=np.load(OUT/'assembly/tets.npy',mmap_mode='r')
        f=np.load(OUT/'assembly/farfield.npy',mmap_mode='r')
        front=np.load(h.OUT/'FRONT.npy')
        h.eq((len(x),len(p),len(t),len(f)),(227901,371646,190387,5182),'WrongMeshCounts')
        h.need(np.array_equal(x[11*16895:12*16895],front),'ChangedL11')
        h.need(len(h.read(OUT/'CORE_INTERFACE_BINDINGS.json'))==33786,'MissingInterface')
    h.durable_json(OUT/'FRESH_REOPEN.json',dict(passed=True,scientific=record['scientific'],
        vertices=227901,prisms=371646,tetrahedra=190387,interface_faces=33786,outer_faces=5182,
        historical_prism_proof_rows=0,historical_prism_quality_rows=0,metrics=h.metrics(*tick)))

if __name__=='__main__':
    import sys
    {'publish':publish,'reopen':reopen}[sys.argv[1]]()
