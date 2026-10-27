"""Reuse a same-process assembly witness under held immutable-file guards.

A resumed process without a witness performs the original derivation. No file
can manufacture a witness. The witness is bound to the exact guarded parent
object and to all source files used to derive the receipt-input component.
"""
from composable_publication import *
import composable_publication as publication
from contextlib import ExitStack

_WITNESSES={}

def file_hash(path):return publication.file_hash(path)
def read(path):return publication.read(path)

def receipt_inputs(base,out):
    witness=_WITNESSES.get(str(out))
    if witness is None:return None
    if witness['base'] is not base or witness['root']!=base.root:raise ValueError('WrongReceiptWitnessBase')
    base.check()
    for path,token in witness['tokens'].items():
        if access.token(path)!=token:raise ValueError('ChangedReceiptWitness')
    return read(out/'NEW_RECEIPT_INPUTS.json'),witness['prior_hashes']


def install():
    import composable_assembly as assembly
    import recursive_production_publication as recursive
    original_assembly=assembly.assemble;original_publish=recursive.publish
    def assemble(base,adapter,out,*args,**kwargs):
        if str(out) in _WITNESSES:raise ValueError('DuplicateReceiptWitness')
        from unittest.mock import patch
        held=ExitStack();tokens={}
        try:
            cert=read(out/'FRESH_CERTIFICATE.json')
            paths=[out/n for n in ('GEOMETRY_DELTA.json','CHANGED_INPUTS.json','FRESH_CERTIFICATE.json')]
            paths.append(out/'objects'/cert['segment']['faces']['file'])
            for path in paths:
                held.enter_context(access.guard(path));tokens[path]=access.token(path)
            writer=assembly.durable_json
            changed=read(out/'CHANGED_INPUTS.json')
            required=set(changed)|{'face:'+f for f in read(paths[-1])}
            def sealed_write(path,value):
                if path==out/'NEW_RECEIPT_INPUTS.json':
                    if not required<=value.keys() or any(value[k]!=v for k,v in changed.items()):
                        raise ValueError('IncompleteOrChangedReceiptInputs')
                    value={k:value[k] for k in required}
                writer(path,value)
                if path==out/'NEW_RECEIPT_INPUTS.json':
                    held.enter_context(access.guard(path))
                    if read(path)!=value:raise ValueError('ChangedAssemblyReceipt')
                    tokens[path]=access.token(path)
            with patch.object(assembly,'durable_json',sealed_write):
                result=original_assembly(base,adapter,out,*args,**kwargs)
            if out/'NEW_RECEIPT_INPUTS.json' not in tokens:raise ValueError('MissingAssemblyReceipt')
            base.check()
            _WITNESSES[str(out)]=dict(base=base,root=base.root,held=held,tokens=tokens,
                prior_hashes=result['old_face_hashes_for_new_interactions'])
        except BaseException:
            held.close();raise
        return result
    def publish(base,adapter,out,parent):
        try:return original_publish(base,adapter,out,parent)
        finally:
            witness=_WITNESSES.pop(str(out),None)
            if witness is not None:witness['held'].close()
    assembly.assemble=assemble
    publication.prepare=prepare
    recursive.publish=publish

def prepare(base,adapter,out):
    """Seal independently produced inputs, not an external target snapshot."""
    from composable_state import plan_transition, SCHEMA
    import composable_state
    transition=plan_transition(base,adapter)
    started=time.perf_counter();info=read(out/'ASSEMBLY.json');g=read(out/'GENERATION.json')
    delta=read(out/'GEOMETRY_DELTA.json');cert=read(out/'FRESH_CERTIFICATE.json');objects=out/'objects'
    if g['transition_plan']!=transition or g['hard_failures'] or not g['topology_audit']['passed'] or not cert['passed'] or cert['failures']: raise ValueError('UnacceptedTransitionEvidence')
    if not read(out/'VOLUME_DELTA.json')['exact_balance'] or not read(out/'VOLUME_DELTA.json')['global_degree']['passed']: raise ValueError('UnacceptedGlobalEvidence')
    fresh_faces=read(objects/cert['segment']['faces']['file'])
    reused=receipt_inputs(base,out)
    if reused is None:
        inputs=read(out/'CHANGED_INPUTS.json');prior_hashes=0
        for f in fresh_faces:
            dep='face:'+f
            if dep not in inputs:
                row=base['faces'][f]
                inputs[dep]=identity(dict(face=row,coordinates={v:base['vertices'][v] for v in row['vertices']}))
                prior_hashes+=1
        durable_json(out/'NEW_RECEIPT_INPUTS.json',inputs)
    else:
        inputs,prior_hashes=reused
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
    geometry=out/'PERSISTENT_GEOMETRY.json'
    if geometry.exists():
        binding=read(geometry)
        if binding['parent']['content']!=base.content_root or binding['geometry_delta']!=file_hash(out/'GEOMETRY_DELTA.json'):
            raise ValueError('WrongPersistentGeometryBinding')
        plan['persistent_geometry']=dict(binding,child_content=content_root,content=content_root)
        closure=binding.get('version_closure')
        if closure is not None and 'path' in closure:
            if identity(read(closure['path']))!=closure['root']:raise ValueError('ChangedVersionClosure')
            refs[closure['path']]=access.attest(closure['path'],file_hash(closure['path']))
        refs[str(geometry.resolve())]=access.attest(geometry,file_hash(geometry))
        def segments(item):
            if 'child_content' in item:
                yield from segments(item['parent']);yield item['delta']
            else:yield item
        for item in (*segments(binding['parent']),binding['delta']):
            manifest=Path(item['directory'])/'MANIFEST.json'
            body=read(manifest)
            if identity(body)!=item['root']:raise ValueError('WrongPersistentGeometryRoot')
            refs[str(manifest.resolve())]=access.attest(manifest,file_hash(manifest))
            refs.update({r['path']:dict(sha256=r['sha256'],token=r['token']) for r in body['files'].values()})
            if 'lookup' in item:
                lookup=item['lookup'];path=Path(lookup['directory'])/'MANIFEST.json';data=read(path)
                if identity(data)!=lookup['root'] or data['index_root']!=item['root']:raise ValueError('WrongSourceLookupRoot')
                refs[str(path.resolve())]=access.attest(path,file_hash(path))
                refs.update({r['path']:dict(sha256=r['sha256'],token=r['token']) for r in data['files'].values()})
    durable_json(out/'CANDIDATE_PLAN.json',plan)
    durable_json(out/'PREPARE_METRICS.json',dict(seconds=time.perf_counter()-started,new_proof_index_rows=len(pairs),
        old_proof_payloads_read=0,old_face_hashes_for_new_interaction_bindings=prior_hashes,
        content_root=content_root,complete_component_map=components))
    return plan
