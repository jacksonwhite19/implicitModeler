"""Generic private transition completion; no saved-case/shape-name dispatch.

The existing placement owner calls this only after coordinate work stalls.
Each adopted closed-star edit uses existing scientific owners; unchanged full
mesh obligations still belong to ordinary transition admission and bulk join.
"""
import time
import numpy as np
from background_topology_repair import key
from background_core_repair import atoms,affected_rows,replace_rows
from refined_local_repair import record_key
from tet_layer_adapter import quality
from shape_transition_cavity_geometry import interior_point_valid
from interior_contraction import attachment_cells, propose
from transaction import prepare
REJECTIONS = frozenset(('ContractionLinkCondition', 'DegenerateReplacement', 'InvalidReplacementOrientation', 'DuplicateContractionReplacement', 'ChangedContractionBoundary', 'InvalidOrientedCavityIncidence', 'NonmanifoldCavityBoundary', 'UnexpectedContractionMovingSites'))

def index(state, check):
    byid = {c['id']: c for c in state['cells']}
    editable = {c['id'] for c in state['additions']}
    stars = {}
    if len(byid) != len(state['cells']) or not editable <= byid.keys():
        raise ValueError('InvalidContractionSourceIdentities')
    for j, c in enumerate(state['additions']):
        if j % 1024 == 0:
            check()
        if c['kind'] != 'tetrahedron':
            raise ValueError('NonTetContractionRegion')
        for v in c['vertices']:
            if v >= state['old_point_count']:
                stars.setdefault(v, []).append(c['id'])
    return dict(state, byid=byid, editable_ids=editable, stars=stars, physical_outer_keys={key(f) for f in state['outer']}, physical_outer_vertices=set(map(int, np.asarray(state['outer']).ravel())))

def inventory(source, quality_result, protected, check):
    seeds = sorted({i for r in quality_result['failures'] for i in (r['owners'] if r['kind'] == 'face_quality' else [r['id']])} & source['editable_ids'])
    fixed = {v for i in protected for v in source['byid'][i]['vertices']}
    pairs = set()
    for i in seeds:
        check()
        for u in source['byid'][i]['vertices']:
            if u < source['old_point_count'] or u in fixed or u in source['physical_outer_vertices']:
                continue
            for c in source['stars'][u]:
                for v in source['byid'][c]['vertices']:
                    if u != v and v >= source['old_point_count']:
                        pairs.add((u, v))
    return sorted(pairs, key=lambda uv: (float(np.linalg.norm(source['points'][uv[0]] - source['points'][uv[1]])), *uv))

def complete(state, quality_result, *, check, handoff, max_proposals, _session=None):
    """Call-scoped source/QA pair, owned by place. Never consumes a saved proof."""
    from repair_timing import RepairProgress
    progress = RepairProgress.from_check(check)
    if type(max_proposals) is not int or max_proposals < 1:
        raise ValueError('ExplicitContractionProposalBudgetRequired')
    check()
    started = time.perf_counter()
    if _session is not None:_session.require(state,quality_result)
    initial_count = len(atoms(quality_result))
    trials = []
    edits = []
    source = state
    q = quality_result
    reports={record_key(r):r for r in q['failures']}
    if any((r['kind'] == 'exact_cell' or (r['kind'] == 'cell_quality' and r['volume'] <= 0) for r in q['failures'])):
        raise ValueError('ContractionPositiveSourceRequired')
    while not q['passed'] and len(trials) < max_proposals:
        check()
        with progress.span('contraction.index'):
            source = index(source, check)
        protected = attachment_cells(source)
        first = max(source['byid']) + 1
        adopted = False
        with progress.span('contraction.inventory'):
            candidates = inventory(source, q, protected, check)
        progress.set(contraction_candidates=len(candidates), contraction_trials=len(trials), contraction_remaining_records=len(q['failures']))
        for u, v in candidates:
            check()
            if len(trials) >= max_proposals:
                break
            row = dict(removed_vertex=u, retained_vertex=v)
            try:
                with progress.span('contraction.proposal'):
                    patch, receipt = propose(source, u, v, protected_ids=protected, first_id=first, check=check)
            except ValueError as exc:
                if str(exc) not in REJECTIONS:
                    raise
                row['rejection'] = str(exc)
                trials.append(row)
                progress.count('contraction_trials')
                continue
            row.update(receipt)
            trials.append(row)
            progress.count('contraction_trials')
            if not patch.initial_quality['passed']:
                continue
            old_faces={key(f) for c in patch.old for f in c['faces']}
            affected=affected_rows(reports,patch.ids,old_faces)
            if atoms(dict(failures=affected))!=atoms(patch.old_quality):
                raise ValueError('ContractionFailureInventoryClosureMismatch')
            with progress.span('contraction.exact_transaction'):
                updated, transaction, mapping = prepare(source, patch, patch.points, patch.initial_quality,
                    membership=interior_point_valid, check=check,_return_mapping=True)
            previous_count=len(atoms(q))
            replace_rows(reports,patch.ids,old_faces,[])
            remapped={}
            for r in reports.values():
                r=dict(r)
                if r['kind']=='face_quality':
                    vertices=np.asarray(r['vertices'],dtype='<i8')
                    if np.any(vertices<0) or np.any(vertices>=len(mapping)) or np.any(mapping[vertices]<0):
                        raise ValueError('RetiredContractionFailureFace')
                    r['vertices']=list(map(int,mapping[vertices]))
                k=record_key(r)
                if k in remapped:raise ValueError('AliasedContractionFailureRecords')
                remapped[k]=r
            reports=remapped
            current=dict(passed=not reports,failures=list(reports.values()),
                diagnostic_failure_inventory=True,scientific_mesh_acceptance=False)
            check()
            if len(atoms(current)) >= previous_count:
                raise ValueError('ContractionWholeFailureCountDidNotDecrease')
            after = updated['byid']
            if any((not np.array_equal(source['points'][source['byid'][i]['vertices']], updated['points'][after[i]['vertices']]) for i in protected)):
                raise ValueError('ChangedProtectedAttachmentGeometry')
            row['adopted'] = True
            edits.append(dict(proposal=receipt, transaction=transaction, whole_records_before=len(q['failures']), whole_records_after=len(current['failures'])))
            if _session is not None:
                _session.require(source,q)
                current=_session.advance(updated,current['failures'])
            source = updated
            q = current
            adopted = True
            break
        if not adopted:
            break
        if len(edits) > initial_count:
            raise ValueError('ContractionStrictReductionBound')
    # The inventory is only scheduling metadata. The ordinary full owner must
    # independently match it before any result leaves this private call.
    if edits and _session is None:
        selected={c['id'] for c in source['additions']}
        selected|={i for f in source['cap_ids'][source['cap_faces']] for i,_ in source['owners'][key(f)]}
        with progress.span('contraction.whole_quality'):
            final=quality(source['points'],source['cells'],source['owners'],selected,source['outer'],handoff)
        check()
        if atoms(final)!=atoms(q):raise ValueError('ContractionFinalFailureInventoryMismatch')
        q=final
    return (source, dict(passed=q['passed'], quality=q, trials=trials, edits=edits, maximum_proposals=max_proposals, seconds=time.perf_counter() - started, stop_reason='QualityPassed' if q['passed'] else 'ProposalBudgetReached' if len(trials) >= max_proposals else 'NoAdmissibleContraction', scientific_mesh_acceptance=False, scope='Coherent private placement state only; ordinary full transition/source/bulk acceptance remains mandatory.'))
