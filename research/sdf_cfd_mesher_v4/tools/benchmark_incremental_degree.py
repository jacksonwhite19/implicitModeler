"""One preserved production transition and independent global reference."""
import argparse,json,time,traceback
from pathlib import Path
import numpy as np
import psutil
from benchmark_production_fast import save,seal
from production_fast_layer import FastLayer
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest
from sdf_cfd_mesher_v4.common.authority import file_hash
import cavity_volume_audit as audit

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--saved-layer',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    started=time.perf_counter()
    try:
        save(a.output/'SOURCE.json',{str(q):file_hash(q) for q in [Path(__file__),Path(__file__).with_name('incremental_degree.py'),Path(__file__).with_name('production_fast_layer.py'),Path(audit.__file__)]})
        state=NativeCheckpointStore(a.source).restart();spec=json.loads((a.reference.parent/'GEOMETRY_AUTHORITY.json').read_text())['spec']
        run=FastLayer(state,spec,a.output);prior=run.degree_state;old_cells=set(run.topology.cells)
        save(a.output/'BOOTSTRAP.json',dict(seconds=time.perf_counter()-started,root=prior.root,source=str(a.source)))
        print('BOOTSTRAP',flush=True)
        result=run.step();proc=psutil.Process();result.update(peak_rss=proc.memory_info().peak_wset,retained_rss=proc.memory_info().rss)
        save(a.output/'LAYER.json',result);print('LAYER '+json.dumps({k:v for k,v in result.items() if k!='degree_metrics'}),flush=True)
        print('DEGREE '+json.dumps({k:v for k,v in result['degree_metrics'].items() if k!='delta'}),flush=True)
        tick=time.perf_counter();stats={};ids=sorted(run.volume.witness)
        global_result=audit.degree(run.points,run.wall+list(run.outer),[run.volume.witness[i] for i in ids],stats=stats)
        if global_result!=run.degree_state.result():raise ValueError('GlobalWitnessOrDirectionMismatch')
        reference_seconds=time.perf_counter()-tick
        np.savez_compressed(a.output/'GLOBAL_DEGREE.npz',ids=ids,degrees=np.array([r['degree'] for r in global_result['values']],dtype=np.int8),
            rays=np.array([r['ray'] for r in global_result['values']],dtype=np.int8))
        tick=time.perf_counter();again,replayed=prior.update(run.points,run.wall+list(run.outer),run.volume.witness,
            [c for i,c in run.topology.cells.items() if i not in old_cells])
        for k in ('root','affected_identity','delta_identity','counts'):
            if replayed[k]!=result['degree_metrics'][k]:raise ValueError('DegreeReplayMismatch:'+k)
        if again.result()!=global_result:raise ValueError('DegreeReplayResultMismatch')
        save(a.output/'GLOBAL_COMPARISON.json',dict(passed=True,witnesses=len(ids),witness_and_direction_exact=True,
            unresolved=len(global_result['unresolved']),seconds=reference_seconds,stats=stats,
            replay_seconds=time.perf_counter()-tick,replay_exact=True,root=again.root,
            result_sha256=file_hash(a.output/'GLOBAL_DEGREE.npz')))
        if a.saved_layer:
            from incremental_layer_validation import t,key
            from final_shell_incidence import check
            from audit_capsule_scaling import final_coverage
            z=np.load(a.saved_layer/'SEALED_GEOMETRY.npz');cells=list(run.topology.cells.values())
            if not np.array_equal(z['points'],run.points):raise ValueError('SavedCoordinatesMismatch')
            expected={int(i):tuple(v) for i,v in zip(z['cell_ids'],z['cell_vertices'])}
            if {c['id']:tuple(c['vertices']) for c in cells}!=expected:raise ValueError('SavedConnectivityMismatch')
            prior_seal=json.loads((a.saved_layer/'SEAL.json').read_text())
            for name,digest in prior_seal['geometry_components'].items():
                if scientific_digest(run.tx.current[name])!=digest:raise ValueError('SavedComponentMismatch:'+name)
            q=t.quality(run.points,cells,run.topology.owners,1);incidence=check(run.points,cells,run.outer)
            if scientific_digest(q)!=prior_seal['quality_digest'] or scientific_digest(incidence)!=prior_seal['prospective_digest']:raise ValueError('SavedQualityMismatch')
            if not q['passed'] or not incidence['passed']:raise ValueError('FinalQualityFailure')
            if {r['id']:r for r in q['cells']}!=run.quality.cells or {r['id']:r for r in q['exact']}!=run.quality.exact:raise ValueError('CurrentCellReceiptMismatch')
            if {key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in q['faces']}!=run.quality.faces:raise ValueError('CurrentFaceReceiptMismatch')
            faces=[run.topology.owners[k][0][1] for k in sorted(run.topology.owners)]
            coverage=final_coverage(run.points,faces,run.proofs)
            from native_proof_segments import LoadedSegment
            previous=LoadedSegment(prior_seal['delta_segment'],a.saved_layer/'seal_objects',{k:i for i,k in enumerate(run.face_keys)},True)
            from validation_row_stream import clean_rows_digest
            if clean_rows_digest(run.proofs.fresh)!=clean_rows_digest(previous.rows()):raise ValueError('NewProofOutcomeMismatch')
            sealed=dict(passed=True,geometry_exact=True,quality_exact=True,coverage=coverage,forbidden=0,unresolved=0,
                global_degree_exact=True,new_proof_digest=clean_rows_digest(run.proofs.fresh),
                inherited_proofs='Same verified source checkpoint; immutable geometry and proof segments',
                reference_seal_sha256=file_hash(a.saved_layer/'SEAL.json'))
        else:sealed=seal(run,NativeCheckpointStore(a.reference).restart())
        sealed['global_degree_exact']=True;sealed['global_degree_seconds']=reference_seconds
        save(a.output/'SEAL.json',sealed);print('SEALED',flush=True)
    except Exception as e:
        save(a.output/'FAILURE.json',dict(error=str(e),traceback=traceback.format_exc(),seconds=time.perf_counter()-started));raise

if __name__=='__main__':main()
