"""Complete immutable immediate-layer evidence with deterministic entity ranges.

Ranges enumerate every source cell/side/interface ID, not a sampled aggregate.
All rows are bound to complete ordered input arrays, including the prior cell
layer for interface metrics. A single changed bit invalidates the receipt.
Final continuous pair completeness is deliberately NOT replaced by this ledger.
"""
from pathlib import Path
from functools import lru_cache
import json
import numpy as np
from ..common.authority import array_hash,identity,file_hash
from ..forecast.detailed.scientific import plain

VERSION='V4CommittedValidation/1'

@lru_cache(maxsize=1)
def implementation():
    root=Path(__file__).resolve().parents[1]
    names=('forecast/detailed/metrics.py','advancing_front/rolling_metrics.py','advancing_front/local_relaxation.py',
        'advancing_front/validation_evidence.py','forecast/forensics.py')
    return identity({name:file_hash(root/name) for name in names})

def input_identity(top,low,high,prior_low,layer,targets,sdf_identity):
    return dict(version=VERSION,implementation=implementation(),topology=top.sha256,
        triangles=array_hash(top.triangles),edges=array_hash(top.edges),low=array_hash(low),high=array_hash(high),
        prior_low=None if prior_low is None else array_hash(prior_low),layer=int(layer),target=float(targets[layer-1]),schedule=array_hash(targets),sdf=sdf_identity)

def populations(top,layer):
    return [dict(kind='cell',layer=layer,start=0,stop=len(top.triangles),evidence=['volume','signed_volume','jacobian','determinant','aspect','exact_sign','ownership']),
        dict(kind='side',layer=layer,start=0,stop=len(top.edges),evidence=['weight','skewness','symmetric_volume_ratio']),
        dict(kind='cap',layer=layer-1,start=0,stop=len(top.triangles) if layer>1 else 0,evidence=['weight','skewness','symmetric_volume_ratio']),
        dict(kind='vertex',layer=layer,start=0,stop=len(top.vertex_triangles.offsets)-1,evidence=['sdf_residual'])]

def make_receipt(top,inputs,gate,sdf_record,exact,certificate,conditioning):
    required={'volume','signed_volume','jacobian','determinant','aspect','weight','skewness','symmetric_volume_ratio'}
    if set(gate['violations'])!=required:raise ValueError('IncompleteNumericEvidence')
    if set(conditioning)!={'sigma','G_bound','G_bound_available'} or any(v.shape!=(len(top.triangles),) for v in conditioning.values()):raise ValueError('IncompleteConditioningEvidence')
    if not {'target','maximum_residual','phi_hash','gradient_hash','gradient_norm_min','gradient_norm_max'}<=set(sdf_record):raise ValueError('IncompleteSdfEvidence')
    if sdf_record['target']!=inputs['target']:raise ValueError('SdfTargetMismatch')
    if not gate['passed'] or any(gate['violations'].values()):raise ValueError('InvalidNumericEvidence')
    if not gate['topology_valid'] or not gate['source_wall_ownership_valid']:raise ValueError('InvalidOwnershipEvidence')
    if not exact['passed'] or exact['cells']!=len(top.triangles) or exact['failures']:raise ValueError('IncompleteExactSigns')
    if not certificate['complete'] or any(certificate[k] for k in ('forbidden','unresolved','invalid_surfaces')):raise ValueError('InvalidContinuousEvidence')
    if certificate['candidate_pairs']!=certificate['completed_pairs']:raise ValueError('IncompleteContinuousEvidence')
    result=dict(inputs=inputs,populations=populations(top,inputs['layer']),gate=plain(gate),sdf=sdf_record,exact=exact,
        certificate_hash=identity(certificate),conditioning={k:array_hash(v) for k,v in conditioning.items()})
    return dict(result,receipt_hash=identity(result))

def verify_receipt(receipt,top,inputs,checkpoint,conditioning,sdf_record):
    r=dict(receipt);h=r.pop('receipt_hash',None)
    if h!=identity(r):raise ValueError('ValidationReceiptCorrupt')
    if r['inputs']!=inputs:raise ValueError('ValidationInputChanged')
    if r['populations']!=populations(top,inputs['layer']):raise ValueError('ValidationCoverageIncomplete')
    if r['gate']!=plain(checkpoint['gate']) or r['exact']!=checkpoint['exact'] or r['certificate_hash']!=identity(checkpoint['certificate']):raise ValueError('ValidationCheckpointMismatch')
    rebuilt=make_receipt(top,inputs,r['gate'],sdf_record,checkpoint['exact'],checkpoint['certificate'],conditioning)
    if rebuilt!=receipt:raise ValueError('ValidationEvidenceChanged')
    return receipt

def resolve_receipt(path,top,inputs,checkpoint,conditioning,sdf_record,recompute):
    """A missing/invalid receipt invokes the existing scientific evaluator.

    The caller still audits topology, committed transaction chains, and the
    complete final pair population. No receipt can bypass those global proofs.
    """
    p=Path(path);reason='MissingReceipt'
    if p.exists():
        try:return verify_receipt(json.loads(p.read_text()),top,inputs,checkpoint,conditioning,sdf_record),True,None
        except (ValueError,KeyError,TypeError) as error:reason=str(error)
    receipt=recompute()
    verify_receipt(receipt,top,inputs,checkpoint,conditioning,sdf_record)
    p.write_bytes((json.dumps(receipt,sort_keys=True,indent=2)+'\n').encode())
    return receipt,False,reason
