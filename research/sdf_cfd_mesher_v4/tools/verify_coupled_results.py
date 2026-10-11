"""Fail-closed scientific checks on the completed native task evidence."""
import sys
sys.dont_write_bytecode=True
import json,time
from pathlib import Path
from collections import Counter
import numpy as np
import coupled_shell_validation as c
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.complete_neighbor_receipt import verify_complete_neighbor
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs
from sdf_cfd_mesher_v4.common.authority import identity
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/coupled_shell_remesh_determinant'
MAIN='D'
REPLAY='E'


def read(path):return json.loads(path.read_text())


def run():
    base=OUT/'general_asymmetric_passage';b=read(base/MAIN/'SHELL_RESULT.json');replay=read(base/REPLAY/'SHELL_RESULT.json')
    assert b['final']==replay['final'] and b['operations']==replay['operations']
    assert b['remesh_count']==1 and [r['operation'] for r in b['operations']]==['RollbackRemesh','PrismCommit']
    assert b['wall_exact'] and b['first_layer_exact'] and b['restart_exact']
    assembly=read(base/MAIN/'ASSEMBLY.json');assert assembly['passed']
    state=NativeCheckpointStore(base/MAIN/'checkpoints').restart()
    assert scientific_hash(state)==b['final']
    keys=sorted(state['vertices']);idx={k:i for i,k in enumerate(keys)}
    p=np.array([state['vertices'][k] for k in keys])
    cells=[dict(id=int(k),column=int(v['source']),layer=v['layer'],kind=v['kind'],vertices=[idx[x] for x in v['vertices']],
        faces=[tuple(idx[x] for x in f) for f in v['faces']]) for k,v in state['cells'].items()]
    outer=[tuple(idx[x] for x in state['faces'][k]['vertices']) for k in state['outer']]
    before=identity(geometry_inputs(state));tick=time.perf_counter();checked=c.check(p,cells,outer,True);elapsed=time.perf_counter()-tick
    receipts=[r for r in state['quality'].values() if r.get('receipt_class')=='CompleteNeighborQualityReceipt']
    assert len(receipts)==1 and verify_complete_neighbor(state,receipts[0],checked)
    assert before==identity(geometry_inputs(state))
    manifest=read(base/MAIN/'ROLLBACK_MANIFEST.json');attempts=manifest['cavity']['expansion']
    assert len(attempts)==2 and not attempts[0]['complete_neighbor']['passed'] and attempts[1]['complete_neighbor']['passed']
    assert len(attempts[1]['active'])==len(attempts[0]['active'])+1
    assert attempts[0]['depth']==attempts[1]['depth']
    for run in (MAIN,REPLAY):
        assert read(base/run/'COUPLED_5.json')['closure_identity']==read(base/MAIN/'COUPLED_5.json')['closure_identity']
        assert read(base/run/'COUPLED_6.json')['closure_identity']==read(base/MAIN/'COUPLED_6.json')['closure_identity']
    arrays=sorted(p.name for p in (base/MAIN/'core').glob('*.npy'))
    assert arrays==sorted(p.name for p in (base/REPLAY/'core').glob('*.npy'))
    for name in arrays:
        assert np.array_equal(np.load(base/MAIN/'core'/name),np.load(base/REPLAY/'core'/name))
    quality=read(base/MAIN/'assembly/QUALITY.json');assert quality['passed'] and not quality['failures']
    sf=read(OUT/'SF2/BOUNDED_REPLAY.json');assert sf['passed'] and sf['remesh_count']==0
    assert sf['audit']['certificate']['active']==56462864
    controls=[]
    for name in ('curved_wide','clean_curved_passage','analytic_cavity_with_control','clean_narrowing_channel'):
        result=read(OUT/'regression'/name/'RESULT.json');assert result['passed'] and result['coupled_failures']==0 and result['revisions']==0
        controls.append(name)
    result=dict(passed=True,native_replay_exact=True,core_replay_exact=True,wall_exact=True,first_layer_exact=True,
        complete_receipt_restart_exact=True,closure_restart_exact=True,closure_rebuild_seconds=elapsed,
        cavity_attempts=len(attempts),removed=dict(Counter(str(r['layer'])+':'+r['kind'] for r in manifest['removed'])),
        final_mixed_determinant=min(r['determinant'] for r in checked['cells']),
        complete_min_determinant=min(r['determinant'] for r in quality['cells']),
        complete_min_volume=min(r['volume'] for r in quality['cells']),
        complete_max_aspect=max(r['aspect'] for r in quality['cells']),
        complete_min_weight=min(r['weight'] for r in quality['faces']),
        complete_min_ratio=min(r['symmetric_volume_ratio'] for r in quality['faces']),
        complete_max_skewness=max(r['skewness'] for r in quality['faces']),
        unchanged_controls=controls+['SF2'],false_coupled_revisions=0,missed_historical_coupled_failure=0,
        scope='One historical positive coupled failure and five unchanged negative geometry controls; no untested generalization claimed')
    c.g.save(OUT/'QUALIFICATION_CHECKS.json',result)


if __name__=='__main__':run()
