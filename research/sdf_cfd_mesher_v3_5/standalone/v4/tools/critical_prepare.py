"""Same accepted preparation with a guarded prebuilt index handoff."""
from prism_receipt_witness import *
from preparation_reference import attest_content

def prepare(base,adapter,out):
    from current_front_dependencies import authenticated_snapshot
    from guarded_publication_inputs import quality_snapshot as snapshot, atlas_snapshot
    with authenticated_snapshot(out/'NEW_FORECAST_DEPENDENCIES.json') as dependencies, snapshot(out/'QUALITY_DELTA.json',require_rows=False,physical_hash=file_hash) as quality:
        import os
        from compact_front_atlas import encoded
        if os.environ.get('V4_INDEXED_ATLAS')=='1' or encoded(out/'NEW_ATLAS.json'):
            with atlas_snapshot(out/'NEW_ATLAS.json') as atlas:
                return _prepare(base,adapter,out,dependencies,quality,atlas)
        return _prepare(base,adapter,out,dependencies,quality)

def _prepare(base,adapter,out,dependencies,quality,atlas=None):
    """Seal independently produced inputs, not an external target snapshot."""
    from composable_state import SCHEMA
    from accepted_production_contract import plan_transition, seal_publication
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
    from critical_index_pipeline import prepared
    index_metrics=prepared(base,out,fresh_faces,pairs)
    if index_metrics is None:
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
    components['atlas']=dict(replace=atlas['semantic'] if atlas is not None else file_hash(out/'NEW_ATLAS.json'))
    components['forecast']=dict(replace=dependencies['semantic'])
    components['quality']=dict(base=base.components['quality'],delta=quality['semantic'],
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
    refs=dict(dependencies['references'])
    refs.update(quality['references'])
    if atlas is not None:refs.update(atlas['references'])
    boundary=out/'FIRST_LAYER_RECEIPT_CONNECTION.json'
    if boundary.exists():
        names.append(boundary.name)
    for p in [out/n for n in names]+[objects/cert['segment'][k]['file'] for k in ('faces','pairs','codes','payloads')]+[index]:
        if str(p.resolve()) in refs:continue
        refs[str(p.resolve())]=attest_content(p)
    from incremental_pair_storage import PairIncidence
    accounting=info['pair_accounting']
    with PairIncidence(base,accounting['index_manifest'],accounting['index_root']) as inherited_index:
        refs.update(inherited_index.body['files'])
        manifest=Path(accounting['index_manifest']).resolve()
        refs[str(manifest)]=attest_content(manifest)
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
            refs[closure['path']]=attest_content(closure['path'])
        refs[str(geometry.resolve())]=attest_content(geometry)
        def segments(item):
            if 'child_content' in item:
                yield from segments(item['parent']);yield item['delta']
            else:yield item
        for item in (*segments(binding['parent']),binding['delta']):
            manifest=Path(item['directory'])/'MANIFEST.json'
            body=read(manifest)
            if identity(body)!=item['root']:raise ValueError('WrongPersistentGeometryRoot')
            refs[str(manifest.resolve())]=attest_content(manifest)
            refs.update({r['path']:dict(sha256=r['sha256'],token=r['token']) for r in body['files'].values()})
            if 'lookup' in item:
                lookup=item['lookup'];path=Path(lookup['directory'])/'MANIFEST.json';data=read(path)
                if identity(data)!=lookup['root'] or data['index_root']!=item['root']:raise ValueError('WrongSourceLookupRoot')
                refs[str(path.resolve())]=attest_content(path)
                refs.update({r['path']:dict(sha256=r['sha256'],token=r['token']) for r in data['files'].values()})
    plan=seal_publication(base,adapter,out,plan)
    durable_json(out/'CANDIDATE_PLAN.json',plan)
    durable_json(out/'PREPARE_METRICS.json',dict(seconds=time.perf_counter()-started,new_proof_index_rows=len(pairs),
        old_proof_payloads_read=0,old_face_hashes_for_new_interaction_bindings=prior_hashes,
        content_root=content_root,complete_component_map=components))
    return plan
