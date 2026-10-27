"""Bounded warm ProductionFast configuration and evidence collection.

Only the existing authorized L6->L8 control is used. No durable publication.
"""
import json
import subprocess
import time
from pathlib import Path
import sys

V4=Path(__file__).resolve().parents[1]
ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from path_preserving_deduplication import durable_json,file_hash
from record_receipt_resume import disk,status

OUT=V4/'results/live_wing_warm_fast'
OWN=['tools/check_warm_fast_scope.py','tools/accepted_fast_binding.py',
     'tools/production_fast_layer.py','tools/check_live_fast_scope.py',
     'tests/test_accepted_fast_binding.py','tests/test_warm_fast_scope.py',
     'CURRENT_TASK.md','README.md','LIVE_WING_WARM_FAST_REPORT.md']

def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))

def baseline():
    if OUT.exists():raise ValueError('PreserveWarmEvidence')
    prior=read(V4/'results/live_wing_fast_scope/BASELINE.json')
    current=[r for r in status() if r[3:]!='research/sdf_cfd_mesher_v4/tools/check_warm_fast_scope.py']
    for group in ('protected','inputs'):
        if any(file_hash(p)!=h for p,h in prior[group].items()):raise ValueError('ChangedProtectedInput',group)
    previous={str(p):file_hash(p) for p in (V4/'results/live_wing_fast_scope').rglob('*') if p.is_file()}
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        status=current,protected=prior['protected'],inputs=prior['inputs'],previous=previous,disk=disk()))

def prepare_review():
    from run_successor_benchmark import execution_sources,execution_binaries,retained_execution_inputs
    previous=read(V4/'results/live_wing_fast_scope/REVIEW_REQUEST.json')
    inputs=retained_execution_inputs(previous['inputs'])
    for layer in (7,8):
        for name in ('GENERATED_NEXT.npz','NEW_QUALITY.json','FRESH_CERTIFICATE.json','GENERATION.json','REOPEN.json'):
            p=V4/f'results/fresh_successor_benchmark/current_parent_run/L{layer}'/name
            inputs[str(p)]=file_hash(p)
    durable_json(OUT/'REVIEW_REQUEST.json',dict(runtime=execution_sources([V4/'tools'/n for n in (
        'check_warm_fast_scope.py','check_live_fast_scope.py','accepted_fast_binding.py','production_fast_layer.py')]),
        binary=execution_binaries(),inputs=inputs,baseline=file_hash(OUT/'BASELINE.json'),
        scope='One fresh warm ProductionFast L6->L7->L8 session. Private parent identities are not sealed identities. No L9 or publication.'))

def worker():
    import check_live_fast_scope as existing
    existing.OUT=OUT
    existing.worker(warm=True)

def compare_components(run,accepted):
    """Exact geometry/dependency comparison with explicit parent-kind variance.

    A private generation cannot use the sealed reference's parent identity.
    Verify every typed schedule field, including the two different valid owners,
    rather than claiming byte equality for that metadata or dropping it.
    """
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
    from accepted_fast_binding import PrivateProductionTrajectory
    from accepted_production_contract import ProductionTrajectory
    from sdf_cfd_mesher_v4.common.authority import identity
    result={}
    for name in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','versions','atlas','forecast','authority','events'):
        a=scientific_digest(run.tx.current[name]);b=scientific_digest(accepted[name])
        result[name]=dict(actual=a,reference=b,equal=a==b)
        if a!=b and name!='forecast':raise ValueError('FastScientificComponentMismatch:'+name)
    owner=run.provider.last_transition_owner
    if result['forecast']['equal']:return result
    if not isinstance(owner,PrivateProductionTrajectory):
        if not result['forecast']['equal']:raise ValueError('FastForecastMismatch')
        return result
    if (run.provider.private_receipt['previous']!=owner.parent or
        run.provider.layer!=owner.layer+1):raise ValueError('FastReferenceParentLinkMismatch')
    if (identity([vars(t) for t in owner.schedule.targets])!=owner._schedule_targets or
        any(t.owner!=owner.source_identity or t.parent!=owner.parent for t in owner.schedule.targets)):
        raise ValueError('AlteredPrivateDependencyTargets')
    expected=ProductionTrajectory(owner.original,owner.authorization,owner.authorization_identity,accepted.parent)
    actual_schedule=[dict(owner.schedule.row_record(k),target=vars(t)) for k,t in enumerate(owner.schedule.targets,1)]
    reference_schedule=[dict(expected.schedule.row_record(k),target=vars(t)) for k,t in enumerate(expected.schedule.targets,1)]
    a=run.tx.current['forecast'];b=accepted['forecast']
    if set(a)!=set(b):raise ValueError('FastForecastInventoryMismatch')
    for source,row in a.items():
        original=b[source];layer=run.tx.current['columns'][source]['last_prism_layer']
        if (set(row)!=set(original) or set(row)!={'region','current_layer','remaining_schedule'} or
            row['region']!=original['region'] or row['current_layer']!=original['current_layer'] or
            row['current_layer']!=layer or row['remaining_schedule']!=actual_schedule[layer:] or
            original['remaining_schedule']!=reference_schedule[layer:]):
            raise ValueError('FastTypedDependencyMismatch')
    for arow,brow in zip(actual_schedule,reference_schedule):
        av=dict(arow,target={k:v for k,v in arow['target'].items() if k not in ('owner','parent')})
        bv=dict(brow,target={k:v for k,v in brow['target'].items() if k not in ('owner','parent')})
        if av!=bv:raise ValueError('FastScheduleMeaningMismatch')
    result['forecast'].update(dependency_meaning_exact=True,rows=len(a),
        difference='Only verified declared-target parent and owner identifiers; private generation vs sealed parent',
        actual_parent=owner.parent,reference_parent=expected.parent,
        actual_owner=owner.source_identity,reference_owner=expected.source_identity,
        actual_schedule=identity(actual_schedule),reference_schedule=identity(reference_schedule))
    return result

def compare_fresh_proofs(run,accepted,reference):
    import numpy as np
    from native_proof_segments import LoadedSegment,segment_objects
    certificate=read(accepted.verify(reference/'FRESH_CERTIFICATE.json'))
    accepted.verify(reference/'GENERATION.json')
    descriptor=certificate['segment']
    for row in segment_objects(descriptor):
        accepted.verify(reference/'objects'/row['file'])
        if file_hash(reference/'objects'/row['file'])!=row['sha256']:raise ValueError('ChangedReferenceProofSegment')
    expected=LoadedSegment(descriptor,reference/'objects',{k:i for i,k in enumerate(run.face_keys)},True)
    actual=run.proofs.fresh
    result=dict(pairs_bit_equal=np.asarray(expected.mapping)[np.asarray(expected.pairs)].tobytes()==np.frombuffer(actual.pairs,dtype=np.uint64).reshape(-1,2).tobytes(),
                codes_bit_equal=np.asarray(expected.codes).tobytes()==np.frombuffer(actual.codes,dtype=np.uint64).tobytes(),
                payloads_equal=expected.payloads==actual.payloads,count=len(actual))
    if not all(result[k] for k in ('pairs_bit_equal','codes_bit_equal','payloads_equal')):raise ValueError('FastFreshProofMismatch')
    return result

def require_reference(accepted,reference,layer):
    if accepted.layer!=layer or accepted.scientific!=read(reference/'REOPEN.json')['scientific']:
        raise ValueError('WrongIntermediateQualifiedReference')
    accepted.verify(reference/'FRESH_CERTIFICATE.json')
    accepted.verify(reference/'GENERATION.json')

def bounds(run):
    """Reduce existing cache results; no new gate or numerical predicate."""
    return dict(cells=len(run.topology.cells),vertices=len(run.points),outer_triangles=len(run.outer),
        extrema=dict(volume=min(r['volume'] for r in run.quality.cells.values()),
            raw_determinant=min(r['determinant'] for r in run.quality.cells.values()),
            prospective_determinant=min(r['determinant'] for r in run.quality.prospective.values()),
            aspect=max(r['aspect'] for r in run.quality.cells.values()),
            weight=min(r['weight'] for r in run.quality.faces.values()),
            skewness=max(r['skewness'] for r in run.quality.faces.values()),
            symmetric_volume_ratio=min(r['symmetric_volume_ratio'] for r in run.quality.faces.values())),
        nonpositive_exact_volume=sum(r['volume_sign']<=0 for r in run.quality.exact.values()),
        nonpositive_exact_jacobian=sum(r['jacobian_sign']<=0 for r in run.quality.exact.values()),
        forbidden=0,unresolved=0,prospective_top='Internal interface; no future neighbour centroids or terminal cap')

def collect():
    from contextlib import ExitStack
    from immutable_state_access import guard
    names=['BASELINE.json','REVIEW_REQUEST.json','REVIEW.json','OUTCOME_REVIEW.json',
        'run/RESULT.json','run/QUALIFICATION.json','run/BINDING.json','run/BOOTSTRAP.json',
        'run/PRODUCTION_L7.json','run/PRODUCTION_L8.json','run/L7_BOUNDS.json','run/L8_BOUNDS.json',
        'run/L7_COMPONENT_COMPARISON.json','run/L7_PROOF_COMPARISON.json',
        'run/COMPONENT_COMPARISON.json','run/L7_PRIVATE_PARENT.json','run/L8_PRIVATE_PARENT.json',
        'monitor/MONITOR_FINAL.json']
    with ExitStack() as held:
        for name in names:held.enter_context(guard(OUT/name))
        _collect()

def _collect():
    from check_current_front_encoding import allocation
    baseline=read(OUT/'BASELINE.json');request=read(OUT/'REVIEW_REQUEST.json')
    for name in ('REVIEW.json','OUTCOME_REVIEW.json'):
        review=read(OUT/name)
        if review.get('passed') is not True or review['request_sha256']!=file_hash(OUT/'REVIEW_REQUEST.json'):
            raise ValueError('IndependentWarmReviewRequired',name)
    if file_hash(OUT/'BASELINE.json')!=request['baseline']:raise ValueError('ChangedWarmBaseline')
    own={'research/sdf_cfd_mesher_v4/'+p for p in OWN}
    current=[r for r in status() if r[3:] not in own and not r[3:].startswith('research/sdf_cfd_mesher_v4/results/live_wing_warm_fast/')]
    if current!=[r for r in baseline['status'] if r[3:] not in own]:raise ValueError('ChangedUnrelatedStatus')
    checks={}
    for name,pins in [('protected',baseline['protected']),('baseline_inputs',baseline['inputs']),('previous_evidence',baseline['previous'])]+[(n,request[n]) for n in ('runtime','binary','inputs')]:
        failures=[p for p,h in pins.items() if file_hash(p)!=h]
        if failures:raise ValueError('ChangedWarmDependency',name,failures)
        checks[name]=dict(files=len(pins),bytes_equal=True)
    run=OUT/'run';monitor=read(OUT/'monitor/MONITOR_FINAL.json')
    result=read(run/'RESULT.json');q=read(run/'QUALIFICATION.json')
    if not result['passed'] or not q['passed'] or monitor['exit_code']!=0 or monitor['resource_stopped']:
        raise ValueError('IncompleteWarmBenchmark')
    summary=dict(classification='V4AcceptedFrontWarmProductionFastL6L8QualifiedControl',result=result,
        binding=read(run/'BINDING.json'),bootstrap=read(run/'BOOTSTRAP.json'),qualification=q,
        layers={},publication=False,last_durable_layer=8,restart_qualified=False)
    for layer in (7,8):
        production=read(run/f'PRODUCTION_L{layer}.json')
        summary['layers'][str(layer)]={k:production[k] for k in ('layer','seconds','timings','counts','cells','relations','new_relations','degree_passed','exact_volume_balance','profile')}
        summary['layers'][str(layer)].update(bounds=read(run/f'L{layer}_BOUNDS.json'),degree_counts=production['degree_metrics']['counts'])
    durable_json(OUT/'SUMMARY.json',summary)
    paths=[p for p in OUT.rglob('*') if p.is_file() and p.name!='EVIDENCE.json']
    storage=allocation(paths);storage.pop('records')
    durable_json(OUT/'EVIDENCE.json',dict(starting_head=baseline['head'],branch=baseline['branch'],
        elapsed_seconds=time.time()-baseline['start'],status_exact=True,unrelated_status_count=len(current),
        checks=checks,artifacts={str(p.relative_to(ROOT)):dict(sha256=file_hash(p),bytes=p.stat().st_size) for p in paths},
        allocation_before_manifest=storage,disk_initial=baseline['disk'],disk_final=disk(),cleanup=False,
        scope='Task output only. Original ancestry and prior fast run unchanged. No published state or new schedule.'))
    print(json.dumps(dict(storage=storage,disk=disk(),checks=checks)))

def monitor():
    import check_live_fast_scope as existing
    existing.OUT=OUT
    return existing.monitor(worker_command=[sys.executable,'-B',__file__,'worker'])

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','review','worker','monitor','collect']);a=p.parse_args()
    result=globals()[{'review':'prepare_review'}.get(a.mode,a.mode)]()
    if a.mode=='monitor':sys.exit(result)
