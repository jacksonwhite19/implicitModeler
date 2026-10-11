"""Connect authenticated prospective L1 evidence to realized incidence receipts.

No receipt is renamed. The immutable aggregate remains historical L1 evidence;
new per-cell receipts certify the actual neighbour update supplied by generation.
"""
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity, file_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front import from_mesh
from exercise_mixed_remesh_transactions import mesh
from first_layer_metric_state import verify


def validate_boundary(base, adapter, prior_rows):
    if base.layer != 1:
        return None
    try:
        aggregate = base['quality']['first_prism_incidence']
    except KeyError:
        return None  # Legacy per-cell receipt path is unchanged.
    keys = [str(r['id']) for r in prior_rows]
    if len(keys) != len(set(keys)) or set(keys) != set(base['cells']):
        raise ValueError('FirstPrismReceiptCoverageMismatch')
    # Same reconstruction/receipt verifier as AnalyticProvider.validate_initial_commit.
    # This only views authenticated existing L1 entities; it creates no new layer.
    prefix = adapter.committed_prefix
    if len(prefix) != 2:
        raise ValueError('FirstPrismReceiptPrefixMismatch')
    numeric = mesh(prefix[0], adapter.topology.triangles, [prefix[1]], np.ones(len(prefix[0]), dtype=int))
    # Pass values, never copy the authenticated reader/guard lifetime itself.
    restored = from_mesh(numeric, dict(base['authority']))
    for kind in ('vertices', 'cells', 'faces', 'outer'):
        actual = base[kind]
        if restored[kind] != (dict(actual) if kind != 'outer' else actual):
            raise ValueError('FirstPrismCommittedTopologyMismatch:' + kind)
    verify(aggregate, np.asarray(numeric['points']), numeric['cells'], numeric['outer'], base['versions'])
    base.check()
    return dict(parent=base.scientific, aggregate=identity(aggregate), affected_cells=len(keys),
                implementation=file_hash(__file__), verifier=file_hash(__import__('first_layer_metric_state').__file__),
                receipt_class=aggregate['receipt_class'], complete_neighbor_metrics=False,
                disposition='Immutable historical L1 receipt; not actual-neighbour evidence',
                actual_incidence=identity(prior_rows))


def validate_cell_dependencies(key, old, cell, old_inputs):
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key
    expected={'cell:'+key,'authority:quality'}|{'face:'+face_key(f) for f in cell['faces']}
    # Existing mixed_evidence.attach_validation binds this additional premise
    # on prospective per-cell receipts. It must not be dropped during updates.
    if old.get('receipt_class')=='ProspectiveFirstPrismDeterminant':
        expected.add('authority:first_prism_metric')
    if set(old['inputs'])!=expected:
        raise ValueError('IncompleteFirstPrismCellDependencies:'+key)
    if any(old_inputs.get(dep)!=value for dep,value in old['inputs'].items()):
        raise ValueError('StaleFirstPrismCellReceipt:'+key)


def validate_absent_final_receipts(base):
    """Inventory keys once, without fetching the large adjacent aggregate value.

Use the same guarded immutable native table as ColdReceipts. No unverified
path, payload traversal, replacement table or synthesized receipt is accepted.
"""
    import sqlite3
    from contextlib import closing
    base.check()
    if base.kind!='native' or base.components['quality']['encoding']!='receipt_table':
        raise ValueError('FirstPrismAggregateRequiresNativeReceiptTable')
    path=base.wrapped._object_path(base.components['quality'])
    base.wrapped._verify_read(path)
    with closing(sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
        rows=db.execute("SELECT key FROM receipts WHERE key GLOB 'final_incidence:*'").fetchall()
    base.check()
    if rows:raise ValueError('ConflictingFirstPrismReceiptPopulation:'+rows[0][0])
