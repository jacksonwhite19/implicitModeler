"""Persist original-run source evidence; never grant live source authority."""
from pathlib import Path
import json
import numpy as np
from immutable_state_access import guard
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import array_hash,identity

FILES=('shell/SOURCE_EVALUATION.json','shell/SOURCE_SEGMENTS.json','native/SEALED_PLAN.npz')
BINDING='V35RetainedSourceEvidence/1'

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

def record(folder):
    evaluation=json.loads((Path(folder)/FILES[0]).read_text())
    return dict(schema=BINDING,status='original_run_evidence_authenticated',evaluation=evaluation,
        fresh_source_authority=False,independent_offline_source_recomputation=False)
