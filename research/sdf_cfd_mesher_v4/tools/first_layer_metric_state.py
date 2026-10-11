"""First-prism determinant admission for a declared internal outer interface.

Foundation's area tensor needs face roles, not the absent neighbor centroid.
This contract supplies only those roles. It is not a complete-neighbor receipt.
Pair metrics and mixed transition closure retain their existing validators.
"""
from collections import Counter
import numpy as np
import final_shell_incidence as incidence
from sdf_cfd_mesher_v4.common.authority import identity, array_hash

VERSION = 'FoundationAreaTensorFirstPrismIncidence/1'


def eligible(cells):
    return bool(cells) and all(c['kind']=='prism' and c['layer']==1 for c in cells)


def contract(points, cells, outer, versions):
    if not eligible(cells): raise ValueError('FirstPrismIncidenceScope')
    counts=Counter(tuple(sorted(f)) for c in cells for f in c['faces'])
    tops={tuple(sorted(c['vertices'][3:])) for c in cells}
    actual=[tuple(sorted(f)) for f in outer]
    if len(actual)!=len(set(actual)) or set(actual)!=tops:
        raise ValueError('FirstPrismOuterRoleMismatch')
    if any(counts[f]!=1 for f in tops): raise ValueError('FirstPrismCapNotExposed')
    walls={tuple(sorted(c['vertices'][:3])) for c in cells}
    boundary=tops|walls
    if any(n!=2 for f,n in counts.items() if f not in boundary):
        raise ValueError('FirstPrismLateralNeighborMissing')
    if any(counts[f]!=1 for f in walls): raise ValueError('FirstPrismWallRoleMismatch')
    return dict(receipt_class='ProspectiveFirstPrismDeterminant',metric_version=VERSION,
                versions=dict(versions),geometry=array_hash(np.asarray(points)),
                topology=identity(cells),face_roles=identity([sorted(walls),sorted(tops)]),
                closure_identity=identity(sorted(tops)),
                obligation='Every top face remains internal in the final shell/core domain',
                complete_neighbor_metrics=False)


def apply(checked, points, cells, outer):
    """Replace only the inapplicable exposed-cap determinant, not other gates."""
    contract(points,cells,outer,{})
    result=incidence.check(points,cells,outer)
    determinants={r['id']:r['determinant'] for r in result['cells']}
    q=checked['quality']
    # Keep a bounded diagnostic summary; never duplicate the full trial arrays.
    diagnostic=dict(minimum=min(r['determinant'] for r in q['cells']),
                    failures=sum(r['determinant']<.0011 for r in q['cells']))
    q['cells']=[dict(r,determinant=determinants[r['id']]) for r in q['cells']]
    q['failures']=[r for r in q['failures'] if r['kind'] not in ('cell_quality','final_incidence_determinant')]
    q['failures'] += [dict(kind='cell_quality',**r) for r in q['cells']
                      if not (r['volume']>0 and r['determinant']>=.0011 and r['aspect']<=1000)]
    from local_correction_validation import failure_order
    q['failures'].sort(key=failure_order)
    q['passed']=not q['failures']
    checked.update(complete_incidence=result,first_prism_incidence=dict(metric_version=VERSION,
                   exposed_shell_diagnostic=diagnostic),passed=False)
    return checked


def receipt(points,cells,outer,versions):
    value=contract(points,cells,outer,versions)
    rows=incidence.check(points,cells,outer)
    if not rows['passed']:raise ValueError('FirstPrismDeterminantRejected')
    value.update(values=identity(rows['cells']),minimum=min(r['determinant'] for r in rows['cells']))
    return value


def verify(expected,points,cells,outer,versions):
    if receipt(points,cells,outer,versions)!=expected:
        raise ValueError('StaleFirstPrismDeterminantReceipt')
    return True
