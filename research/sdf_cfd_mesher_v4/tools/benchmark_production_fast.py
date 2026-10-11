"""One preserved transition; independent comparison is outside its timer."""
import argparse,json,time,traceback
from pathlib import Path
import psutil
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from production_fast_layer import FastLayer


def save(path,value):path.write_text(json.dumps(value,indent=2,default=str)+'\n')


def seal_degree(run):
    """Final sealing always rechecks the full original filling predicate."""
    import cavity_volume_audit as audit
    started=time.perf_counter();ids=sorted(run.volume.witness)
    result=audit.degree(run.points,run.wall+list(run.outer),[run.volume.witness[i] for i in ids])
    if not result['passed']:raise ValueError('FinalGlobalDegreeFailure')
    if result!=run.degree_state.result():raise ValueError('FinalDegreeReceiptMismatch')
    return dict(passed=True,witnesses=len(ids),witness_and_direction_exact=True,seconds=time.perf_counter()-started)


def seal(run,reference):
    from incremental_layer_validation import Quality,t
    from final_shell_incidence import check
    from identity_qualification_audit import population
    from validation_row_stream import clean_rows_digest
    from audit_capsule_scaling import final_coverage
    out={'global_degree':seal_degree(run)};tick=time.perf_counter();state=run.tx.current
    for name in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','versions','atlas','forecast','authority','events'):
        a=scientific_digest(state[name]);b=scientific_digest(reference[name])
        if a!=b:raise ValueError('ScientificComponentMismatch:'+name)
        out[name]=a
    out['component_seconds']=time.perf_counter()-tick;tick=time.perf_counter()
    baseline=Quality.receipts(reference,[r[0][1] for r in run.topology.owners.values()])
    for name in ('cells','faces','exact','prospective'):
        if getattr(run.quality,name)!=getattr(baseline,name):raise ValueError('QualityMismatch:'+name)
    out['preserved_quality_equal']=True
    cells=list(run.topology.cells.values())
    full=t.quality(run.points,cells,run.topology.owners,1)
    if not full['passed']:raise ValueError('IndependentQualityFailure')
    if {r['id']:r for r in full['cells']}!=run.quality.cells:raise ValueError('IndependentCellMismatch')
    if {r['id']:r for r in full['exact']}!=run.quality.exact:raise ValueError('IndependentExactMismatch')
    from incremental_layer_validation import key
    if {key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in full['faces']}!=run.quality.faces:raise ValueError('IndependentFaceMismatch')
    incidence=check(run.points,cells,run.outer)
    if not incidence['passed'] or {r['id']:r for r in incidence['cells']}!=run.quality.prospective:raise ValueError('IndependentIncidenceMismatch')
    out['independent_quality_seconds']=time.perf_counter()-tick;tick=time.perf_counter()
    faces=[run.topology.owners[k][0][1] for k in sorted(run.topology.owners)]
    out['coverage']=final_coverage(run.points,faces,run.proofs)
    out['coverage_seconds']=time.perf_counter()-tick;tick=time.perf_counter()
    _,reference_proofs=population(reference)
    actual=clean_rows_digest(run.proofs);expected=clean_rows_digest(reference_proofs)
    if actual!=expected:raise ValueError('ProofOutcomeMismatch')
    out.update(proof_digest=actual,proof_outcomes_equal=True,forbidden=0,unresolved=0)
    out['proof_comparison_seconds']=time.perf_counter()-tick
    # All geometry, dependency inputs and metric/proof outcomes matched above.
    # Bind the identical existing sealed receipt representation for the final
    # scientific identity comparison. No unproved candidate borrows authority.
    state=dict(state,quality=reference['quality'],certificate=reference['certificate'])
    state.pop('_production_unsealed',None)
    actual=scientific_hash(state);expected=scientific_hash(reference)
    if actual!=expected:raise ValueError('FinalScientificMismatch')
    out.update(scientific=actual,passed=True,seal_kind='Independent validation and exact binding to preserved accepted authority')
    return out


def seal_new(run,output,*,proof_checkpoint=None):
    """Independent final audit without a reference mesh or borrowed receipts."""
    from incremental_layer_validation import t,key
    from final_shell_incidence import check
    from audit_capsule_scaling import final_coverage
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER
    from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import buffers
    from sdf_cfd_mesher_v4.advancing_front.canonical_blocks import build,canonical_root,SCHEMA,AUTHORITY
    from sdf_cfd_mesher_v4.common.authority import identity
    from native_proof_segments import persist_segment
    import numpy as np
    started=time.perf_counter();global_degree=seal_degree(run)
    quality_started=time.perf_counter();times={'global_degree':global_degree['seconds']};cells=list(run.topology.cells.values())
    q=t.quality(run.points,cells,run.topology.owners,1)
    if not q['passed'] or {r['id']:r for r in q['cells']}!=run.quality.cells or {r['id']:r for r in q['exact']}!=run.quality.exact:raise ValueError('IndependentCellMismatch')
    if {key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in q['faces']}!=run.quality.faces:raise ValueError('IndependentFaceMismatch')
    incidence=check(run.points,cells,run.outer)
    if not incidence['passed'] or {r['id']:r for r in incidence['cells']}!=run.quality.prospective:raise ValueError('IndependentIncidenceMismatch')
    times['quality']=time.perf_counter()-quality_started;tick=time.perf_counter()
    faces=[run.topology.owners[k][0][1] for k in sorted(run.topology.owners)]
    coverage=final_coverage(run.points,faces,run.proofs);times['coverage']=time.perf_counter()-tick;tick=time.perf_counter()
    inputs=geometry_inputs(run.tx.current)
    def rows():
        for row in run.proofs:
            pair=sorted(run.face_keys[i] for i in row['pair'])
            deps=['face:'+f for f in pair]+['authority:certificate']
            if 'authority:certificate_filter' in inputs:deps.append('authority:certificate_filter')
            value=dict(faces=pair,inputs={k:inputs[k] for k in deps},value={k:v for k,v in row.items() if k!='pair'})
            yield identity(pair),ENCODER.encode(value),0
    k,v,a=buffers(rows());del a
    directory=output/'seal_objects';directory.mkdir(exist_ok=True)
    checkpoint_binding=None
    if proof_checkpoint is None:root=build(directory,k,v,8)
    else:
        # The same canonical root can be independently rebuilt without another
        # persisted copy of every receipt hash. Exact compact proofs and current
        # dependency state must remain in the verified durable milestone.
        from fast_milestone import signature
        from sdf_cfd_mesher_v4.common.authority import file_hash
        proof_checkpoint=Path(proof_checkpoint);manifest=proof_checkpoint/'CURRENT.json'
        cp=json.loads(manifest.read_text())
        if cp['signature']!=signature(run) or file_hash(proof_checkpoint/cp['payload'])!=cp['sha256']:raise ValueError('FinalProofCheckpointMismatch')
        root=dict(schema=SCHEMA,authority=AUTHORITY,depth=8,root=canonical_root(k,v,8),count=len(k),storage='Reconstruct from verified compact milestone; standalone block store not published')
        checkpoint_binding=dict(path=str(proof_checkpoint),manifest_sha256=file_hash(manifest),payload_sha256=cp['sha256'])
    del k,v
    times['independent_receipt_root']=time.perf_counter()-tick
    delta=persist_segment(run.proofs.fresh,run.face_keys,directory) if proof_checkpoint is None else None
    # Final-seal geometry is durable even though ordinary steps publish no
    # checkpoint. Face roles are the unchanged qualified prism template.
    geometry_path=output/'SEALED_GEOMETRY.npz'
    arities={len(c['vertices']) for c in cells}
    connectivity=(dict(cell_vertices=np.asarray([c['vertices'] for c in cells],dtype=np.int64))
                  if len(arities)==1 else
                  dict(cell_vertex_offsets=np.cumsum([0]+[len(c['vertices']) for c in cells],dtype=np.int64),
                       cell_vertices_flat=np.asarray([v for c in cells for v in c['vertices']],dtype=np.int64)))
    boundary=(dict(outer=np.asarray(run.outer,dtype=np.int64)) if len({len(f) for f in run.outer})<=1 else
              dict(outer_offsets=np.cumsum([0]+[len(f) for f in run.outer],dtype=np.int64),
                   outer_flat=np.asarray([v for f in run.outer for v in f],dtype=np.int64)))
    np.savez_compressed(geometry_path,points=run.points,**connectivity,**boundary,
        cell_ids=np.asarray([c['id'] for c in cells],dtype=np.int64),
        source_columns=np.asarray([c['column'] for c in cells],dtype=np.int64),
        layers=np.asarray([c['layer'] for c in cells],dtype=np.int64),
        wall_triangles=run.provider.tri,schedule=run.provider.schedule.targets)
    from sdf_cfd_mesher_v4.common.authority import file_hash
    geometry_components={k:scientific_digest(run.tx.current[k]) for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','versions','atlas','forecast','authority','events')}
    quality_digest=scientific_digest(q);prospective_digest=scientific_digest(incidence)
    scientific=identity(dict(schema='V4ProductionFastFinalSeal/1',geometry=geometry_components,
        quality=quality_digest,prospective=prospective_digest,certificate=dict(authority=root['authority'],root=root['root'],count=root['count'])))
    return dict(passed=True,coverage=coverage,forbidden=0,unresolved=0,certificate=root,scientific=scientific,global_degree=global_degree,
                scientific_schema='V4ProductionFastFinalSeal/1',
                geometry_file=geometry_path.name,geometry_sha256=file_hash(geometry_path),
                quality_equal=True,quality_digest=quality_digest,prospective_digest=prospective_digest,
                geometry_components=geometry_components,
                  inherited_segments=[s.descriptor for s in run.proofs.segments] if proof_checkpoint is None else None,delta_segment=delta,proof_checkpoint=checkpoint_binding,
                times=times,seconds=time.perf_counter()-started,
                seal_kind='Independent complete geometry and canonical receipt root; inherited source checkpoint remains required')


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--new-layer',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    started=time.perf_counter()
    try:
        import hashlib
        sources=[Path(__file__),Path(__file__).with_name('production_fast_layer.py'),Path(__file__).with_name('incremental_layer_validation.py')]
        save(a.output/'SOURCE.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
        state=NativeCheckpointStore(a.source).restart()
        spec=json.loads((a.reference.parent/'GEOMETRY_AUTHORITY.json').read_text())['spec']
        run=FastLayer(state,spec,a.output)
        save(a.output/'BOOTSTRAP.json',dict(seconds=time.perf_counter()-started,source=str(a.source)))
        print('BOOTSTRAP COMPLETE',flush=True)
        result=run.step();process=psutil.Process();result.update(peak_rss=process.memory_info().peak_wset,retained_rss=process.memory_info().rss)
        save(a.output/'LAYER.json',result);print('LAYER '+json.dumps(result),flush=True)
        if a.new_layer:sealed=seal_new(run,a.output)
        else:
            reference=NativeCheckpointStore(a.reference).restart()
            sealed=seal(run,reference)
        save(a.output/'SEAL.json',sealed);print('SEAL '+json.dumps({k:v for k,v in sealed.items() if k not in ('inherited_segments','certificate')}),flush=True)
    except Exception as e:
        save(a.output/'FAILURE.json',dict(error=str(e),traceback=traceback.format_exc(),seconds=time.perf_counter()-started))
        raise


if __name__=='__main__':main()
