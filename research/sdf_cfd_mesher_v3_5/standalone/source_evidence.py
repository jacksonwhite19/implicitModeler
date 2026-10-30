"""Persist original-run source evidence; never grant live source authority."""
from pathlib import Path
import json
import numpy as np
from immutable_state_access import guard
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import array_hash,identity

FILES=('shell/SOURCE_EVALUATION.json','shell/SOURCE_SEGMENTS.json','native/SEALED_PLAN.npz')
BINDING='V35RetainedSourceEvidence/1'

def validate_execution(record,positions,source_identity,h1,proof_identity):
    """Authenticate retained original execution, never issue live authority."""
    if record is None:return # Historical serial evidence retains its schema.
    requested=record.get('requested_workers');effective=record.get('effective_workers')
    if type(requested) is not int or requested not in (1,2,4,8) or type(effective) is not int:raise ValueError('InvalidSourceExecutionSelection')
    if record.get('schema')=='V35SourceExecutionSelection/1':
        if effective!=1 or not isinstance(record.get('reason'),str) or not record['reason']:raise ValueError('InvalidSerialSourceExecution')
        return
    from source_parallel import VERSION
    if (record.get('schema')!=VERSION or requested not in (2,4,8) or effective!=requested or record.get('state')!='closed' or record.get('geometry')!=source_identity or record.get('height_hex')!=float(h1).hex() or record.get('positions_shape')!=list(positions.shape) or record.get('positions_sha256')!=array_hash(positions) or record.get('proof_identity')!=proof_identity or record.get('contained_active_processes')!=0 or type(record.get('runtime_files')) is not int or record['runtime_files']<1 or type(record.get('sequence')) is not int or record['sequence']<1):raise ValueError('InvalidClosedSourceExecution')
    if requested==8:
        affinity=record.get('parent_cpu_affinity')
        if (record.get('parent_cpu_affinity_check')!='pool_pre_spawn' or not isinstance(affinity,list) or len(affinity)<8 or any(type(i) is not int or i<0 for i in affinity)
                or len(set(affinity))!=len(affinity)):raise ValueError('InvalidEightWorkerAffinityEvidence')
    for key in ('runtime_identity','case_sha256','positions_sha256','proof_identity','semantic_context_sha256','transcript_sha256'):
        value=record.get(key)
        if type(value) is not str or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):raise ValueError('InvalidSourceExecutionDigest',key)
    rows=record.get('workers')
    if not isinstance(rows,list) or len(rows)!=effective:raise ValueError('InvalidSourceExecutionCoverage')
    for i,row in enumerate(rows):
        if (row.get('worker')!=i or row.get('state')!='closed' or row.get('passed') is not True or row.get('session')!=record.get('session') or row.get('runtime')!=record['runtime_identity'] or row.get('files')!=record['runtime_files'] or row.get('case_sha256')!=record['case_sha256']):raise ValueError('InvalidSourceExecutionWorkerClosure')

def validate(evaluation,segments,positions,source_identity,h1,proof_identity):
    from source_handoff import VERSION
    n=len(positions)-1;vertices=len(positions[0])
    method=evaluation.get('method')
    if (evaluation.get('schema')!=VERSION or evaluation.get('mode')!='production'
        or method not in ('live_final_population_authenticated','recomputed')
        or evaluation.get('independent_source_recomputation') is not (method=='recomputed')
        or type(evaluation.get('layers')) is not int or evaluation['layers']!=n
        or type(evaluation.get('segments')) is not int or evaluation['segments']!=n*vertices):raise ValueError('InvalidRetainedSourceEvaluation')
    rows=segments.get('layers')
    if (evaluation.get('execution') or {}).get('schema')=='V35OwnedLocalSourceExecution/1' and method!='live_final_population_authenticated':raise ValueError('ParallelExecutionRequiresLiveSourceHandoff')
    validate_execution(evaluation.get('execution'),positions,source_identity,h1,proof_identity)
    if segments.get('passed') is not True or not isinstance(rows,list) or len(rows)!=n or identity(rows)!=proof_identity:raise ValueError('InvalidRetainedSourcePopulation')
    for k,row in enumerate(rows):
        if (row.get('passed') is not True or row.get('segments')!=vertices
            or len(row.get('segment_proofs',[]))!=vertices or row.get('geometry_identity')!=source_identity
            or row.get('start')!=array_hash(positions[k]) or row.get('end')!=array_hash(positions[k+1])
            or row.get('first_source_normal_height_m')!=(h1 if k==0 else None)
            or row.get('exact_field_progress') is not True or row.get('positive_endpoints') is not True):raise ValueError('InvalidRetainedSourceRow',k)

def retain(folder,evaluation,segments,positions,source_identity,h1,scope):
    """Write the exact producer values and hold them until publication overlap."""
    if scope is None:raise ValueError('ProductionSourceEvidenceLifetimeRequired')
    folder=Path(folder)
    validate(evaluation,segments,positions,source_identity,h1,identity(segments['layers']))
    for name,value in (('SOURCE_EVALUATION.json',evaluation),('SOURCE_SEGMENTS.json',segments)):
        path=folder/name;durable_json(path,value);scope.enter_context(guard(path))
        if json.loads(path.read_text())!=value:raise ValueError('ChangedProducerSourceEvidence',name)

def validate_completion(folder,case,parent):
    folder=Path(folder)
    evaluation=json.loads((folder/FILES[0]).read_text());segments=json.loads((folder/FILES[1]).read_text())
    with np.load(folder/FILES[2],allow_pickle=False) as saved:positions=saved['positions']
    if array_hash(positions)!=parent.get('actual_positions_sha256'):raise ValueError('ChangedSourceEvidencePrismOperands')
    h1=float.fromhex(case['prisms']['first_full_normal_thickness_m_hex'])
    validate(evaluation,segments,positions,parent['source_identity'],h1,parent['source_segments_identity'])
    execution=(evaluation.get('execution') or {})
    if execution.get('schema')=='V35OwnedLocalSourceExecution/1':
        import hashlib
        if execution['case_sha256']!=hashlib.sha256((folder/'CASE.json').read_bytes()).hexdigest():raise ValueError('ChangedSourceExecutionCase')

def record(folder):
    evaluation=json.loads((Path(folder)/FILES[0]).read_text())
    return dict(schema=BINDING,status='original_run_evidence_authenticated',evaluation=evaluation,
        fresh_source_authority=False,independent_offline_source_recomputation=False)

def validate_runtime(folder,runtime):
    execution=(record(folder)['evaluation'].get('execution') or {})
    if execution.get('schema')=='V35OwnedLocalSourceExecution/1' and execution.get('runtime_identity')!=runtime:
        raise ValueError('SourceWorkerProducerRuntimeMismatch')
