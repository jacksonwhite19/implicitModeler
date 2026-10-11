"""Independent component-root publication and cold views; no target oracle.

The prior root may be native or incremental. New proof segments and factorized bindings are produced
by the guarded clean-sheet certifier. Legacy digest streaming is deep-audit
work and is never part of normal publication or next-increment binding.
"""
from collections import ChainMap, Counter
from collections.abc import Mapping
from contextlib import ExitStack
from pathlib import Path
import json, sqlite3, time, zlib
import numpy as np
import immutable_state_access as access
from incremental_transaction import Store, canonical
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity, array_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER, NativeCheckpointStore, scientific_digest
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items


def read(p):return json.loads(Path(p).read_text())


def prepare(base,adapter,out):
    """Seal independently produced inputs, not an external target snapshot."""
    from composable_state import SCHEMA
    from accepted_production_contract import plan_transition, seal_publication
    import composable_state
    transition=plan_transition(base,adapter)
    started=time.perf_counter();info=read(out/'ASSEMBLY.json');g=read(out/'GENERATION.json')
    delta=read(out/'GEOMETRY_DELTA.json');cert=read(out/'FRESH_CERTIFICATE.json');objects=out/'objects'
    if g['transition_plan']!=transition or g['hard_failures'] or not g['topology_audit']['passed'] or not cert['passed'] or cert['failures']: raise ValueError('UnacceptedTransitionEvidence')
    if not read(out/'VOLUME_DELTA.json')['exact_balance'] or not read(out/'VOLUME_DELTA.json')['global_degree']['passed']: raise ValueError('UnacceptedGlobalEvidence')
    inputs=read(out/'CHANGED_INPUTS.json');prior_hashes=0
    fresh_faces=read(objects/cert['segment']['faces']['file'])
    for f in fresh_faces:
        dep='face:'+f
        if dep not in inputs:
            row=base['faces'][f]
            inputs[dep]=identity(dict(face=row,coordinates={v:base['vertices'][v] for v in row['vertices']}))
            prior_hashes+=1
    durable_json(out/'NEW_RECEIPT_INPUTS.json',inputs)
    pairs=np.load(objects/cert['segment']['pairs']['file'],mmap_mode='r',allow_pickle=False)
    index=objects/'new-proof-key-index.sqlite'
    if index.exists():raise ValueError('PreserveExistingProofIndex')
    from incremental_pair_storage import write_proof_index
    index_metrics=write_proof_index(index,fresh_faces,pairs)
    if index_metrics['rows']!=cert['required_pairs']:raise ValueError('IncompleteNewProofIndex')
    durable_json(out/'PROOF_INDEX_METRICS.json',index_metrics)
    with np.load(out/'GENERATED_NEXT.npz',allow_pickle=False) as a:
        coordinates=a['coordinates'];triangles=a['topology']
    hot=dict(vertices=delta['vertices'],columns=delta['columns'],prism_front=delta['prism_front'],
             faces={k:delta['faces'][k] for k in delta['prism_front'].values()})
    durable_json(out/'CANDIDATE_HOT.json',hot)
    components={k:dict(base=base.content_root,role=k) for k in base.components}
    for k in delta:components[k]['overlay']=file_hash(out/'GEOMETRY_DELTA.json')
    components['outer']=dict(replace=identity(sorted(delta['prism_front'].values())))
    components['atlas']=dict(replace=file_hash(out/'NEW_ATLAS.json'))
    components['forecast']=dict(replace=file_hash(out/'NEW_FORECAST_DEPENDENCIES.json'))
    components['quality']=dict(base=base.components['quality'],delta=file_hash(out/'QUALITY_DELTA.json'),
        count=len(base['quality'])+info['quality_new'])
    components['certificate']=dict(base=base.components['certificate'],new_segment=cert['segment'],
        bindings=file_hash(out/'BINDING_PATCH.json'),new_inputs=file_hash(out/'NEW_RECEIPT_INPUTS.json'),
        lookup=file_hash(index),count=cert['combined_pairs'])
    content=dict(schema=SCHEMA,parent_scientific=base.scientific,
        parent_content=base.content_root,components=components)
    content_root=identity(content)
    names=['GEOMETRY_DELTA.json','QUALITY_DELTA.json','BINDING_PATCH.json','NEW_RECEIPT_INPUTS.json',
           'NEW_ATLAS.json','NEW_FORECAST_DEPENDENCIES.json','CANDIDATE_HOT.json','GENERATION.json',
           'ASSEMBLY.json','FRESH_CERTIFICATE.json','VOLUME_DELTA.json','ADMISSION.json','AFFECTED_INCIDENCE.json']
    refs={}
    for p in [out/n for n in names]+[objects/cert['segment'][k]['file'] for k in ('faces','pairs','codes','payloads')]+[index]:
        refs[str(p.resolve())]=access.attest(p,file_hash(p))
    from incremental_pair_storage import PairIncidence
    accounting=info['pair_accounting']
    with PairIncidence(base,accounting['index_manifest'],accounting['index_root']) as inherited_index:
        refs.update(inherited_index.body['files'])
        manifest=Path(accounting['index_manifest']).resolve()
        refs[str(manifest)]=access.attest(manifest,file_hash(manifest))
    expected=dict(base.expected,committed=content_root,layer=transition["target_layer"],front_coordinates=array_hash(coordinates))
    plan=dict(schema=SCHEMA,scope='production',pair_accounting_root=accounting['index_root'],scientific=content_root,content=content,components=components,
        references=refs,parent=base.reference,output=str(out.resolve()),expected=expected,
        hot=str((out/'CANDIDATE_HOT.json').resolve()),geometry_counts=g['counts'],
        implementation=file_hash(composable_state.__file__))
    plan=seal_publication(base,adapter,out,plan)
    durable_json(out/'CANDIDATE_PLAN.json',plan)
    durable_json(out/'PREPARE_METRICS.json',dict(seconds=time.perf_counter()-started,new_proof_index_rows=len(pairs),
        old_proof_payloads_read=0,old_face_hashes_for_new_interaction_bindings=prior_hashes,
        content_root=content_root,complete_component_map=components))
    return plan


def publish_prepared(base,out,expected_plan,expected_acceptance,interrupt=None):
    """Retry an externally pinned prepared transaction, without rebuilding it."""
    plan=read(out/'CANDIDATE_PLAN.json'); acceptance=read(out/'CANDIDATE_ACCEPTANCE.json')
    if identity(plan)!=expected_plan or identity(acceptance)!=expected_acceptance:
        raise ValueError('PreparedTransactionIdentityMismatch')
    if plan['parent']!=base.reference or plan['content']['parent_scientific']!=base.scientific:
        raise ValueError('WrongPreparedTransactionBase')
    base.check(); store=Store(out/'candidate')
    from guarded_publication_inputs import quality_snapshot as snapshot
    bindings=read(out/'BINDING_PATCH.json')
    def boundary(stage):
        if stage=='validated': base.check()
        if interrupt: interrupt(stage)
    import os
    with snapshot(out/'QUALITY_DELTA.json',legacy_read=read,physical_hash=file_hash,retain_verified=os.environ.get('V4_VERIFIED_QUALITY')=='1') as quality:
        root=store.publish(acceptance['base'],plan,quality['rows'],bindings,acceptance,expected_acceptance,boundary)
    return root,dict(store.stats)


def publish(base,adapter,out):
    import os
    if os.environ.get('V4_GUARDED_PUBLICATION')=='1':
        if os.environ.get('V4_VERIFIED_QUALITY')!='1':raise ValueError('VerifiedPublicationQualityRequired')
        from guarded_publication_inputs import scope
        with scope(out):return _publish(base,adapter,out)
    return _publish(base,adapter,out)


def _publish(base,adapter,out):
    plan=prepare(base,adapter,out);store=Store(out/'candidate')
    initial=store.initialize(dict(epoch=access.epoch(),scientific=base.scientific,
                                  plan=dict(references={})))
    from guarded_publication_inputs import quality_snapshot as snapshot
    from single_quality_component import quality_identity
    patch=read(out/'BINDING_PATCH.json')
    import os
    with snapshot(out/'QUALITY_DELTA.json',legacy_read=read,physical_hash=file_hash,retain_verified=os.environ.get('V4_VERIFIED_QUALITY')=='1') as quality:
        acceptance=dict(passed=True,base=initial,plan=identity(plan),quality_delta=quality_identity(quality['rows']),
            binding_patch=identity(patch),quality_count=len(quality['rows']),
            authority='Generic changed-state certification bound to authenticated parent root',
            evidence={n:file_hash(out/n) for n in ('ADMISSION.json','GENERATION.json','ASSEMBLY.json','VOLUME_DELTA.json','FRESH_CERTIFICATE.json')})
    durable_json(out/'CANDIDATE_ACCEPTANCE.json',acceptance)
    tick=time.perf_counter()
    root,stats=publish_prepared(base,out,identity(plan),identity(acceptance))
    durable_json(out/'PUBLICATION.json',dict(root=root,content_root=plan['scientific'],parent=initial,
        seconds=time.perf_counter()-tick,stats=stats,oracle_unlocked=False))
    return root
