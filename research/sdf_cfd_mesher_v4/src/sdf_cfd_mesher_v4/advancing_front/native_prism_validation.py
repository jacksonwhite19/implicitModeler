"""Final prism-block validation through native receipts and relation lookup."""
import numpy as np
from ..common.authority import array_hash
from .native_front_providers import NativeForecast,native_array
from .validation_evidence import input_identity,verify_receipt


def audit_native_prisms(state,objects,manifest):
    forecast=NativeForecast(state,objects);backing=forecast.backing;top=forecast.top
    targets=np.asarray(state['local_schedule']['nominal'],dtype=np.float64)
    sdf=state['quality']['layer:1']['receipt']['inputs']['sdf']
    for layer in range(1,backing.layer+1):
        row=state['quality'][f'layer:{layer}'];receipt=row['receipt']
        conditioning={k:native_array(objects,v) for k,v in row['conditioning'].items()}
        prior=None if layer==1 else backing.coordinates(layer-2)
        inputs=input_identity(top,backing.coordinates(layer-1),backing.coordinates(layer),prior,layer,targets,sdf)
        proof=state['certificate'][f'layer:{layer}']
        if proof!=manifest.metadata()['blocks'][layer-1]:raise ValueError('NativeCertificateBindingMismatch')
        verify_receipt(receipt,top,inputs,dict(gate=receipt['gate'],exact=receipt['exact'],certificate=proof['ledger']),conditioning,receipt['sdf'])
    if not np.all(np.diff(top.edge_triangles.offsets)==2):raise ValueError('NativePrismBoundaryTopology')
    if len(state['cells'])!=backing.layer*len(top.triangles):raise ValueError('NativeCellPopulation')
    return dict(quality_receipts=backing.layer,cells=len(state['cells']),certificate=manifest.audit(backing.layer),
                wall=array_hash(backing.coordinates(0)),first_layer=array_hash(backing.coordinates(1)),
                layer=backing.layer,all_hard_receipts_pass=True)
