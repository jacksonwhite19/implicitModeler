"""Separate provisional complete-neighbour receipts from shell-local proofs."""
from copy import deepcopy
from .mixed_front import geometry_inputs,face_key
from ..common.authority import identity


def bind_complete_neighbor(state,check):
    if not check['passed']:raise ValueError('CompleteNeighborQualityFailed')
    if check.get('scope')=='AllPrismUnchanged':return None
    inputs=geometry_inputs(state);deps=set()
    for row in check['cells']:
        key=str(row['id']);cell=state['cells'][key];deps.add('cell:'+key)
        for f in cell['faces']:
            face=face_key(f);deps.add('face:'+face)
            deps.update('cell:'+str(k) for k in state['faces'][face]['owners'])
    # Temporary closure vertex/cell numbering is not scientific identity.
    stencils={tuple(sorted(s['face'])):s for s in check.get('stencils',[])}
    face_results=[]
    for row in check['faces']:
        stencil=stencils[tuple(sorted(row['vertices']))]
        key=str(stencil['shell_cell']);coordinates=sorted(stencil['coordinates'][:3])
        matches=[face_key(f) for f in state['cells'][key]['faces']
                 if sorted(state['vertices'][v] for v in f)==coordinates]
        if len(matches)!=1:raise ValueError('UnboundClosureFace')
        face_results.append(dict(face=matches[0],shell_owner=key,
            metrics={k:deepcopy(v) for k,v in row.items() if k not in ('vertices','owners')}))
    receipt=dict(receipt_class='CompleteNeighborQualityReceipt',
        scope=check['scope'],determinant_scope=check['determinant_scope'],
        inputs={k:inputs[k] for k in sorted(deps)},
        versions={k:deepcopy(state['versions'][k]) for k in sorted(deps)},
        closure_identity=check['closure_identity'],metric_version=check['metric_version'],
        closure_version=check['closure_version'],
        result=dict(passed=True,cells=deepcopy(check['cells']),
                    faces=sorted(face_results,key=lambda r:(r['shell_owner'],r['face'])),closures=check['closures']),
        final_core_receipt=False)
    receipt['identity']=identity(receipt)
    # Native restart tables are immutable disk-backed evidence. Add a bounded
    # overlay instead of expanding the entire historical receipt population.
    from .native_checkpoint import ReceiptTable,ReceiptOverlay
    if isinstance(state['quality'],ReceiptTable):
        state['quality']=ReceiptOverlay(state['quality'])
    state['quality']['complete-neighbor:'+receipt['closure_identity']]=receipt
    return receipt


def verify_complete_neighbor(state,receipt,check):
    inputs=geometry_inputs(state)
    if receipt['identity']!=identity({k:v for k,v in receipt.items() if k!='identity'}):raise ValueError('CorruptCompleteNeighborReceipt')
    if any(inputs.get(k)!=v or state['versions'].get(k)!=receipt['versions'][k] for k,v in receipt['inputs'].items()):
        raise ValueError('StaleCompleteNeighborShell')
    for key in ('closure_identity','metric_version','closure_version'):
        if receipt[key]!=check[key]:raise ValueError('StaleCompleteNeighborClosure')
    if not check['passed']:raise ValueError('CompleteNeighborQualityFailed')
    return True
