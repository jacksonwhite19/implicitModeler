"""Final immutable receipt coverage; continuous completeness stays separate.

Call only with receipts verified against current geometry, checkpoint, SDF and
conditioning by validation_evidence.verify_receipt/resolve_receipt. This audit
then proves complete ordered layer/entity coverage and links adjacent inputs.
"""
from ..common.authority import identity,array_hash
from .validation_evidence import populations

def final_receipt_manifest(receipts,top,targets):
    if len(receipts)!=len(targets):raise ValueError('FinalReceiptCoverageIncomplete')
    rows=[];previous=None
    for layer,r in enumerate(receipts,1):
        content=dict(r);h=content.pop('receipt_hash',None)
        if h!=identity(content):raise ValueError('FinalReceiptCorrupt')
        inp=r['inputs']
        if inp['layer']!=layer or inp['schedule']!=array_hash(targets) or inp['target']!=float(targets[layer-1]) or inp['topology']!=top.sha256:raise ValueError('FinalReceiptIdentityMismatch')
        if r['populations']!=populations(top,layer):raise ValueError('FinalReceiptEntityCoverageIncomplete')
        if previous and (inp['low']!=previous['high'] or inp['prior_low']!=previous['low'] or inp['sdf']!=previous['sdf']):raise ValueError('FinalReceiptFrontChainMismatch')
        if layer==1 and inp['prior_low'] is not None:raise ValueError('FinalReceiptFirstLayerMismatch')
        rows.append(dict(layer=layer,receipt_hash=h,populations=r['populations'],certificate_hash=r['certificate_hash']))
        previous=inp
    manifest=dict(complete=True,requested_layers=len(targets),receipts=rows,cell_evidence=len(targets)*len(top.triangles),
        side_evidence=len(targets)*len(top.edges),internal_cap_evidence=max(0,len(targets)-1)*len(top.triangles),
        vertex_sdf_evidence=len(targets)*(len(top.vertex_triangles.offsets)-1),
        continuous_completeness='Required separate complete final pair-population audit; this manifest does not replace it.')
    return dict(manifest,scientific_hash=identity(manifest))
