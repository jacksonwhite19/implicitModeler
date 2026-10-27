"""Receipt delta plan produced while the qualified certifier visits proofs."""
from copy import deepcopy
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ReceiptTable, ReceiptOverlay, receipt_table_from_rows


def patch_table(base, rows):
    import tempfile
    from pathlib import Path
    parent=(base.path.parent.parent/'working').resolve()
    if not parent.is_relative_to(base.path.parent.parent.resolve()):raise ValueError('ProofScratchEscapesCheckpoint')
    parent.mkdir(parents=True,exist_ok=True)
    owner=tempfile.TemporaryDirectory(prefix='proof-patches-',dir=parent)
    if Path(owner.name).resolve().parent!=parent:raise ValueError('InvalidProofScratchPath')
    try:return receipt_table_from_rows(owner.name,rows),owner
    except BaseException:
        owner.cleanup()
        raise


def immutable_base(value):
    while isinstance(value, ReceiptOverlay):
        if value.edits or value.removed:
            return None
        value = value.base
    return value if isinstance(value, ReceiptTable) else None


def retained_proofs(before, old_inputs, new_inputs, unchanged, indices, rows):
    """Validate old dependencies and stream only changed receipt bindings.

    rows receives the exact legacy proof population. The receipt plan never
    keeps the old native state or millions of expanded native receipts alive.
    """
    base = immutable_base(before['certificate'])
    trusted = base is not None and getattr(base,'qualified_context',None)==identity(old_inputs)
    retired = []
    def updates():
        for key, receipt in before['certificate'].items():
            if not trusted and not all(old_inputs.get(k) == v for k, v in receipt['inputs'].items()):
                raise ValueError('StaleReusableProof')
            a, b = receipt['faces']
            if a not in unchanged or b not in unchanged:
                retired.append(key)
                continue
            rows.append(dict(pair=sorted((indices[a], indices[b])), **receipt['value']))
            if base is not None and any(new_inputs.get(k) != v for k, v in receipt['inputs'].items()):
                updated = dict(receipt, inputs={k: new_inputs[k] for k in receipt['inputs']})
                yield key, updated
    if base is None:
        for _ in updates():
            pass
        return None
    patches,owner = patch_table(base, updates())
    return dict(base=base, patches=patches, retired=retired,
                context=identity(new_inputs), inherited_coverage=trusted,scratch_owner=owner)
