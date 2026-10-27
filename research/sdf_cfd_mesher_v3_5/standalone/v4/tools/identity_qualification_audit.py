"""Independent canonical checks, deliberately outside ordinary commits."""
import argparse,json,time
from pathlib import Path
import numpy as np
import psutil
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest,ENCODER
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs
from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import buffers
from sdf_cfd_mesher_v4.advancing_front.authenticated_map import canonical_root
from sdf_cfd_mesher_v4.common.authority import identity
from native_proof_segments import LoadedSegment,SegmentProofRows
from compact_proof_rows import CompactProofRows
from validation_row_stream import clean_rows_digest


def legacy_state_hash(state):
    names=('version','wall','first_layer','vertices','cells','faces','columns','outer','prism_front',
           'versions','authority','atlas','forecast','quality','certificate','events')
    data={k:state[k] for k in names}
    if 'local_schedule' in state:data['local_schedule']=state['local_schedule']
    return scientific_digest(data)


def population(state):
    n=len(state['wall']['vertices'])
    def vertex(v):
        a=v.split(':')
        if len(a)!=4 or a[0]!='layer' or a[2]!='source':raise ValueError('AuditRequiresProductionFaceOrder')
        return int(a[1])*n+int(a[3])
    keys=sorted(state['faces'],key=lambda k:tuple(sorted(vertex(v) for v in state['faces'][k]['vertices'])))
    indices={k:i for i,k in enumerate(keys)};table=state['certificate']
    segments=[LoadedSegment(s,table.proof_objects,indices,True) for s in table.proof_segments]
    return keys,SegmentProofRows(segments,CompactProofRows())


def crosscheck(state,expected_scientific=None,expected_proof=None):
    start=time.perf_counter();legacy=legacy_state_hash(state);a=time.perf_counter()
    keys,proofs=population(state);proof=clean_rows_digest(proofs);b=time.perf_counter()
    if expected_scientific is not None and legacy!=expected_scientific:raise ValueError('LegacyScientificMismatch')
    if expected_proof is not None and proof!=expected_proof:raise ValueError('LegacyProofMismatch')
    return dict(passed=True,legacy_scientific=legacy,legacy_proof_digest=proof,
                new_scientific=scientific_hash(state),certificate_root=state['certificate'].authenticated['root'],
                count=len(proofs),legacy_scientific_seconds=a-start,legacy_proof_seconds=b-a,
                scope='Independent complete canonical transition cross-check; excluded from production commit')


def final(checkpoints,output):
    from audit_capsule_scaling import final_coverage
    from audit_native_receipt_tables import audit_tables
    from run_production_scale_resume import summary
    start=time.perf_counter();state=NativeCheckpointStore(checkpoints).restart();times={}
    checkpoint=scientific_hash(state);keys,proofs=population(state);inputs=geometry_inputs(state)
    def progress(phase):
        Path(str(output)+'.progress.json').write_text(json.dumps(dict(
            complete=False,phase=phase,checkpoint=checkpoint,relations=len(proofs),
            times=times,elapsed_seconds=time.perf_counter()-start),indent=2)+'\n')
    progress('independent_root_rebuild')
    tick=time.perf_counter()
    def rows():
        for row in proofs:
            faces=sorted(keys[i] for i in row['pair'])
            deps=['face:'+f for f in faces]+['authority:certificate']
            if 'authority:certificate_filter' in inputs:deps.append('authority:certificate_filter')
            receipt=dict(faces=faces,inputs={k:inputs[k] for k in deps},value={k:v for k,v in row.items() if k!='pair'})
            yield identity(faces),ENCODER.encode(receipt),0
    k,v,actions=buffers(rows());del actions
    times['independent_row_stream_encode_sort']=time.perf_counter()-tick
    from sdf_cfd_mesher_v4.advancing_front import canonical_blocks as blocks
    descriptor=state['certificate'].authenticated;root_tick=time.perf_counter()
    if descriptor['authority']==blocks.AUTHORITY:
        rebuilt=blocks.canonical_root(k,v,descriptor['depth'])
    else:rebuilt=canonical_root(k,v).hex()
    times['independent_root_construction']=time.perf_counter()-root_tick
    del k,v
    if rebuilt!=state['certificate'].authenticated['root']:raise ValueError('IndependentCompleteRootMismatch')
    times['independent_root_rebuild']=time.perf_counter()-tick;tick=time.perf_counter()
    progress('independent_complete_coverage')
    # Production source keys define the array view; no wall coordinate moves.
    vertex_keys=sorted(state['vertices'],key=lambda s:(int(s.split(':')[1]),int(s.split(':')[3])))
    index={k:i for i,k in enumerate(vertex_keys)}
    points=np.asarray([state['vertices'][k] for k in vertex_keys])
    faces=[tuple(index[v] for v in state['faces'][k]['vertices']) for k in keys]
    coverage=final_coverage(points,faces,proofs)
    times['independent_complete_coverage']=time.perf_counter()-tick;tick=time.perf_counter()
    progress('full_receipt_outcome_audit')
    receipts=audit_tables(state['quality'],state['certificate'],len(state['cells']))
    times['full_receipt_outcome_audit']=time.perf_counter()-tick
    progress('legacy_digest_crosscheck')
    comparison=crosscheck(state)
    values=summary(state);gates=values['extrema']
    assert gates['determinant']>=.0011 and gates['weight']>=.055
    assert gates['volume']>0 and gates['symmetric_volume_ratio']>=.01
    assert gates['skewness']<=3.63636 and gates['aspect']<=1000
    restart_exact=checkpoint==json.loads((Path(checkpoints)/'CURRENT.json').read_text())['scientific']
    if not restart_exact:raise ValueError('FinalAuditRestartIdentityMismatch')
    result=dict(passed=True,coverage=coverage,receipts=receipts,comparison=comparison,
                independently_rebuilt_root=rebuilt,summary=values,times=times,
                restart_exact=restart_exact,
                total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset)
    Path(output).write_text(json.dumps(result,indent=2)+'\n');print('FINAL_AUDIT '+json.dumps(result),flush=True)
    Path(str(output)+'.progress.json').write_text(json.dumps(dict(complete=True,checkpoint=checkpoint,
        times=times,elapsed_seconds=time.perf_counter()-start),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('checkpoints',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();final(a.checkpoints,a.output)
