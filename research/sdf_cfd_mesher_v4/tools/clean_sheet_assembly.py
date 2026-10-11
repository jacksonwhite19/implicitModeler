"""Independently bind new/affected L5 evidence to the authenticated L4 prefix."""
from collections import ChainMap, Counter
import json, sqlite3, time
import numpy as np
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key, geometry_inputs, rebuild_dependencies


def read(p):return json.loads(p.read_text())


def assemble(base,adapter,out):
    start=time.perf_counter();generation=read(out/'GENERATION.json')
    if generation['hard_failures'] or not generation['topology_audit']['passed']:
        raise ValueError('UnacceptedGeneratedGeometry')
    certificate=read(out/'FRESH_CERTIFICATE.json')
    if not certificate['passed'] or certificate['failures']:raise ValueError('UnacceptedFreshCertificate')
    from sdf_cfd_mesher_v4.certificate.efficient import SOURCE
    import termination_geometry_certificate as exact
    if base['authority']['certificate_filter']!=file_hash(SOURCE) or base['authority']['certificate']!=file_hash(exact.__file__):
        raise ValueError('PrefixProofPredicateAuthorityMismatch')
    change=read(out/'GEOMETRY_DELTA.json');inputs=read(out/'CHANGED_INPUTS.json')
    state=ChainMap({k:ChainMap(v,base[k]) for k,v in change.items()},base)
    state['outer']=sorted(change['prism_front'].values())
    q=read(out/'NEW_QUALITY.json');inc=read(out/'AFFECTED_INCIDENCE.json')
    exact_rows={str(r['id']):r for r in q['exact']}
    prior_rows=[r for r in inc['cells'] if str(r['id']) not in change['cells']]
    # Prior cell geometry is needed only for the new shared-face bindings.
    prior_cells={str(r['id']):base['cells'][str(r['id'])] for r in prior_rows}
    inputs.update(geometry_inputs(dict(vertices=state['vertices'],cells=prior_cells,faces={},
                                      wall=base['wall'],authority=base['authority'])))
    quality=[]
    def row(key,value,deps):
        bound={k:inputs[k] for k in deps}
        quality.append(dict(key=key,inputs=bound,payload=dict(value,inputs=bound) if value is not None else None))
    for r in q['cells']:
        k=str(r['id']);c=change['cells'][k]
        deps=['cell:'+k]+['face:'+face_key(f) for f in c['faces']]+['authority:quality']
        row('cell:'+k,dict(cell_type=c['kind'],value=r,exact=exact_rows[k]),deps)
    n=len(adapter.front.coordinates)
    def source(v):return f'layer:{v//n}:source:{v%n}'
    for r in q['faces']:
        k=face_key(tuple(source(v) for v in r['vertices']))
        deps=['face:'+k,'authority:quality']+['cell:'+str(i) for i in r['owners']]
        row('face:'+k,dict(value={k:v for k,v in r.items() if k!='vertices'}),deps)
    for r in inc['cells']:
        k=str(r['id'])
        if k not in change['cells']:continue
        deps=['cell:'+k,'authority:quality']+['face:'+face_key(f) for f in change['cells'][k]['faces']]
        row('final_incidence:'+k,dict(value=r,receipt_class='FinalIncidenceDeterminant',final_core_quality=False),deps)
    # Point lookups only. Do not load the old quality population.
    descriptor=base.body['components']['quality'];path=base._object_path(descriptor)
    base._verify_read(path);db=sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True)
    examined=0;differences=[]
    try:
        for r in prior_rows:
            k=str(r['id'])
            for prefix in ('cell:','final_incidence:'):
                found=db.execute('SELECT value FROM receipts WHERE key=?',(prefix+k,)).fetchone()
                if found is None:raise ValueError('MissingPriorQuality')
                old=json.loads(found[0]);examined+=1
                if old['value']['determinant']!=r['determinant']:
                    differences.append(dict(key=prefix+k,terminal=old['value']['determinant'],actual=r['determinant']))
                bound={key:inputs.get(key,value) for key,value in old['inputs'].items()}
                quality.append(dict(key=prefix+k,inputs=bound,payload=None))
    finally:db.close()
    durable_json(out/'PRIOR_DETERMINANT_COMPARISON.json',dict(examined=examined,differences=differences))
    if differences:raise ValueError('PriorDeterminantRequiresNumericalOverlay')
    vertex_faces={}
    for k,r in change['faces'].items():
        for v in r['vertices']:vertex_faces.setdefault(v,set()).add(k)
    # A new front vertex has no incidence with an earlier face. Its exact
    # neighbourhood is wholly in the changed incidence set.
    rebuild_dependencies({},state,adapter.schedule.targets.tolist(),dependency_inputs=inputs,vertex_incidence=vertex_faces)
    durable_json(out/'NEW_ATLAS.json',state['atlas']);durable_json(out/'NEW_FORECAST_DEPENDENCIES.json',state['forecast'])
    patch={k:v for k,v in inputs.items() if k.startswith('face:') and k[5:] in base['faces']}
    durable_json(out/'BINDING_PATCH.json',patch)
    fresh_faces=read(out/'objects'/certificate['segment']['faces']['file'])
    old_face_hashes=0
    for f in fresh_faces:
        dep='face:'+f
        if dep not in inputs:
            r=base['faces'][f]
            inputs[dep]=identity(dict(face=r,coordinates={v:base['vertices'][v] for v in r['vertices']}))
            old_face_hashes+=1
    durable_json(out/'NEW_RECEIPT_INPUTS.json',inputs)
    # Count affected old bindings from compact pair indices only. This is an
    # explicitly measured qualification scan, not proof deserialization.
    affected=0;scanned=0;segments=base.body['components']['certificate']['proof_segments']
    for segment in segments:
        faces=base._read_object_json(segment['faces'])
        marked=np.array(['face:'+f in patch for f in faces],bool)
        p=base._object_path(segment['pairs']);base._verify_read(p)
        pairs=np.load(p,mmap_mode='r',allow_pickle=False)
        for i in range(0,len(pairs),100000):
            part=pairs[i:i+100000];affected+=int(np.count_nonzero(marked[part[:,0]]|marked[part[:,1]]));scanned+=len(part)
    if scanned!=len(base['certificate']):raise ValueError('IncompletePriorPairIndexCoverage')
    durable_json(out/'QUALITY_DELTA.json',quality)
    info=dict(prior_proofs_reused=len(base['certificate']),prior_proof_payloads_read=0,prior_proof_payloads_recomputed=0,
        prior_bindings_changed=affected,prior_bindings_unchanged=len(base['certificate'])-affected,
        qualification_compact_pair_rows_scanned=scanned,new_proofs=certificate['required_pairs'],
        old_quality_rows_examined=examined,quality_input_only_updates=examined,
        quality_new=len(quality)-examined,quality_reused_unchanged=len(base['quality'])-examined,
        old_numeric_quality_recomputed=len(prior_rows),prior_determinant_exact=True,
        face_binding_factors=len(patch),old_face_hashes_for_new_interactions=old_face_hashes,
        new_atlas_rows=len(state['atlas']),oracle_unlocked=False,
        seconds=time.perf_counter()-start)
    durable_json(out/'ASSEMBLY.json',info)
    return info
