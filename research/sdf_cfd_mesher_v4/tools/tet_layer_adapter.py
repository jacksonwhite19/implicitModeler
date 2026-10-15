"""Generic conformal copied-front tetrahedral adapter; current owners accept.

Coordinates are proposals until whole-stage scientific admission. No source
geometry, saved acceptance or backend-success authority is invented here.
"""
from local_constraint_plan import prepared
from collections import defaultdict
import math, time
import numpy as np
from background_materialization import build
from background_local_repair import LocalRepair, star_index
from background_topology_repair import key, boundary
from sdf_cfd_mesher_v4.common.authority import array_hash, identity
from prism_hotpath_v2 import batch_quality, completed_incidence, reverse_cycle
from background_core_repair import atoms as raw_failures
import local_termination_topology as t
from tet_layer_controls import SCHEMA, validate, CUSTOM_SCHEMA, variable_shape

def product(faces, n, rows):
    """V3 Freudenthal product, with source winding; never flip a folded tet."""
    f = np.asarray(faces, dtype='<i8')
    if f.ndim != 2 or f.shape[1] != 3 or len(np.unique(f)) != n or (f.min() != 0) or (f.max() != n - 1):
        raise ValueError('CompleteTetCapRequired')
    if any((np.any(f[:, i] == f[:, j]) for i, j in ((0, 1), (0, 2), (1, 2)))):
        raise ValueError('DistinctTetCapVerticesRequired')
    order = np.sort(f, axis=1)
    a, b, c = order.T
    one = np.empty((3 * len(f), 4), dtype='<i8')
    one[0::3] = np.column_stack((a, b, c, c + n))
    one[1::3] = np.column_stack((a, b, b + n, c + n))
    one[2::3] = np.column_stack((a, a + n, b + n, c + n))
    inv = (f[:, 0] > f[:, 1]).astype(int) + (f[:, 0] > f[:, 2]) + (f[:, 1] > f[:, 2])
    flip = np.repeat(inv % 2 == 1, 3)
    flip[1::3] = ~flip[1::3]
    one[flip, :2] = one[flip, :2][:, ::-1]
    return np.concatenate([one + j * n for j in range(rows)])

def quality(points, cells, owners, selected, outer, handoff=None):
    """Reuse current QA and declared final-incidence determinant owner.

    Outer faces have no invented neighbour. Their eventual internal mask is
    checked separately; all actual neighbour metrics use real current owners.
    """
    q = batch_quality(points, cells, owners, 0, active_cells=selected, mixed_cells=True, compact=True, face_batch_size=4096, _handoff=handoff, prospective_outer=outer)
    d = q.pop('prospective_incidence')
    det = {r['id']: r['determinant'] for r in d['cells']}
    fail = []
    reported = set()
    for r in q['failures']:
        if r['kind'] == 'cell_quality':
            r = dict(r, determinant=det[r['id']])
            reported.add(r['id'])
            if r['volume'] > 0 and r['aspect'] <= 1000 and (r['determinant'] >= 0.0011):
                continue
        fail.append(r)
    byid = {c['id']: c for c in cells}
    for r in d['failures']:
        if r['id'] in reported:
            continue
        c = byid[r['id']]
        g = t.geometry(points, c)
        fail.append(dict(kind='cell_quality', id=c['id'], column=c['column'], cell_type=c['kind'], volume=g['volume'], aspect=g['aspect'], determinant=r['determinant']))
    return dict(passed=not fail, failures=fail, summary=dict(q['summary'], minimum_determinant=min(det.values())), prospective_incidence=dict(passed=d['passed'], failures=d['failures'], cells_checked=len(d['cells']), outer_neighbours_unresolved=True), scope='Current actual-neighbour face QC and exact positive cells; outer-face final determinant mask only. Outer bulk-neighbour metrics remain unresolved.')

def initialize(points, oldcells, oldowners, ids, faces, terminal, controls, bounds=None, *, prism_volumes=None, prism_centres=None, check=None, _fronts=None, _construction=None, _direction=None, _output=None, _resources=None):
    if controls.get('schema') == CUSTOM_SCHEMA:
        if any(v is not None for v in (_fronts, _construction, _direction)):
            raise ValueError('LegacyRowHintNotValidForCustomTransition')
        from custom_transition_builder import initialize as custom_initialize
        return custom_initialize(points,oldcells,oldowners,ids,faces,terminal,controls,bounds,
            prism_volumes=prism_volumes,prism_centres=prism_centres,check=check,output=_output,resources=_resources)
    if controls.get('schema') == 'V4TetLayerAdapter/2':
        if any((v is not None for v in (_fronts, _construction, _direction))):
            raise ValueError('LegacyRowHintNotValidForShapeTransition')
        from shape_transition_builder import initialize as shape_initialize
        return shape_initialize(points, oldcells, oldowners, ids, faces, terminal, controls, bounds, prism_volumes=prism_volumes, prism_centres=prism_centres, check=check, output=_output, resources=_resources)
    from tet_layer_planning import complete_rows
    validate(controls)
    points = np.asarray(points, dtype='<f8')
    ids = np.asarray(ids, dtype='<i8')
    faces = np.asarray(faces, dtype='<i8')
    n = len(ids)
    if faces.ndim != 2 or faces.shape[1:] != (3,) or len(terminal) != len(faces) or (ids.ndim != 1) or (n == 0) or np.any(ids < 0) or np.any(ids >= len(points)):
        raise ValueError('InvalidTetLayerInterfacePopulation')
    if len(set(map(int, ids))) != n or points.ndim != 2 or points.shape[1] != 3 or (not np.isfinite(points).all()):
        raise ValueError('InvalidTetLayerInputs')
    product(faces, n, 1)
    byid = {c['id']: c for c in oldcells}
    for face, owner in zip(ids[faces], terminal):
        uses = oldowners.get(key(face), [])
        if len(uses) != 1 or uses[0][0] != owner or byid[owner]['kind'] != 'prism' or (not reverse_cycle(face[::-1], uses[0][1])):
            raise ValueError('ChangedTetCapOwner')
    if prism_volumes is None and prism_centres is None:
        from background_mixed_geometry import batch_geometry
        cached = batch_geometry(points, [byid[i] for i in dict.fromkeys(terminal)])
        prism_volumes = np.asarray([cached[i]['volume'] for i in terminal])
        prism_centres = np.asarray([cached[i]['centre'] for i in terminal])
    if prism_volumes is None or prism_centres is None or np.shape(prism_volumes) != (len(faces),) or (np.shape(prism_centres) != (len(faces), 3)) or (not np.isfinite(prism_volumes).all()) or (not np.isfinite(prism_centres).all()) or np.any(np.asarray(prism_volumes) <= 0):
        raise ValueError('InvalidTetPrismProposalGeometry')
    if bounds is not None:
        bounds = np.asarray(bounds, dtype=float)
        if bounds.shape != (3, 2) or not np.isfinite(bounds).all() or np.any(bounds[:, 0] >= bounds[:, 1]):
            raise ValueError('InvalidTetTransitionDomain')
        if np.any(points <= bounds[:, 0]) or np.any(points >= bounds[:, 1]):
            raise ValueError('TetPrismOutsideBackgroundDomain')
    if _fronts is None:
        if _construction is not None or _direction is not None:
            raise ValueError('TransitionRowsRequiredForConstructionHint')
        fronts, planned = complete_rows(points[ids], faces, np.asarray(prism_centres), np.asarray(prism_volumes), controls, bounds, check)
    else:
        import json
        from sdf_cfd_mesher_v4.common.authority import frozen
        if type(_fronts) is not np.ndarray or _fronts.dtype != np.dtype('<f8') or _fronts.shape != (controls['rows'] + 1, n, 3) or (not np.isfinite(_fronts).all()) or (_fronts[0].tobytes() != points[ids].tobytes()) or (type(_construction) is not dict) or (type(_direction) is not np.ndarray) or (_direction.dtype != np.dtype('<f8')) or (_direction.shape != (n, 3)) or (not np.isfinite(_direction).all()) or (not np.allclose(np.linalg.norm(_direction, axis=1), 1, atol=1e-13, rtol=0)):
            raise ValueError('InvalidTransitionRowsHint')
        fronts = frozen(_fronts, '<f8')
        planned = json.loads(json.dumps(_construction, sort_keys=True, allow_nan=False))
        for j in range(controls['rows']):
            if check:
                check()
            delta = fronts[j + 1] - fronts[0]
            if np.any(np.linalg.norm(delta, axis=1) > (j + 1) / controls['rows'] * controls['maximum_distance_m']):
                raise ValueError('TransitionRowsHintDistanceFailure')
            if np.any(np.einsum('ij,ij->i', delta, _direction) <= 0):
                raise ValueError('TransitionRowsHintDirectionFailure')
        if bounds is not None and (np.any(fronts <= bounds[:, 0]) or np.any(fronts >= bounds[:, 1])):
            raise ValueError('TransitionRowsHintDomainFailure')
        if check:
            check()
    result = np.concatenate((points, fronts[1:].reshape(-1, 3)))
    compactids = np.r_[ids, np.arange(len(points), len(result))]
    tetrows = compactids[product(faces, n, controls['rows'])]
    additions = build(tetrows, max(byid) + 1)
    owners = {k: list(v) for k, v in oldowners.items()}
    for c in additions:
        for f in c['faces']:
            owners.setdefault(key(f), []).append((c['id'], f))
    outer = compactids[faces + controls['rows'] * n]
    chain, bound, unique = boundary(additions)
    expected = {key(f) for f in ids[faces]} | {key(f) for f in outer}
    if set(chain) != expected:
        raise ValueError('ChangedTetLayerProductBoundary')
    if any((len(row) > 2 or (len(row) == 2 and (not reverse_cycle(row[0][1], row[1][1]))) for row in owners.values())):
        raise ValueError('TetLayerOrientedOwnershipFailure')
    direction = fronts[1] - fronts[0]
    depth = np.linalg.norm(direction, axis=1)
    if np.any(depth <= 0):
        raise ValueError('NonpositiveTetLayerExpansion')
    direction = direction / depth[:, None] if _direction is None else frozen(_direction, '<f8')
    return dict(points=result, cells=oldcells + additions, additions=additions, owners=owners, outer=outer, compact_ids=compactids, cap_ids=ids, cap_faces=faces, direction=direction, depth=depth, old_point_count=len(points), original=points.copy(), controls=dict(controls), old_face_keys=frozenset(oldowners), bounds=bounds, construction=planned)

def local_outer_faces(cells, outer_keys):
    return [f for c in cells for f in c['faces'] if key(f) in outer_keys]

def _coordinates(s, q, tick, handoff, progress, *, enrich=False, allow_enriched=True, _session=None):
    # Each phase rebuilds all indices and models from the actual current state.
    from scipy.optimize import minimize
    shape = s['controls']['schema'] == 'V4TetLayerAdapter/2'
    x = s['points']
    new = s['additions']
    outer = s['outer']
    active = {c['id'] for c in new}
    terminal = {i for f in s['cap_ids'][s['cap_faces']] for i, _ in s['owners'][key(f)] if i not in active}
    selected = active | terminal
    initial = len(q['failures'])
    records = []
    fixed = set(range(s['old_point_count']))
    n = len(s['cap_ids'])
    outerkeys = {key(f) for f in outer}
    byid = {c['id']: c for c in s['cells']}

    def record_key(r):
        return (r['kind'], key(r['vertices'])) if r['kind'] == 'face_quality' else (r['kind'], r['id'])
    failures = {record_key(r): r for r in q['failures']}
    sweeps = []
    guided_pending=set();guided_seen=set()
    while failures:
        tick()
        oldcount = len(failures)
        progress.set(sweep=len(sweeps), remaining_records=len(failures), enriched=enrich)
        groups = set()
        made_progress = False
        for r in failures.values():
            ownerids = r['owners'] if r['kind'] == 'face_quality' else [r['id']]
            g = tuple(sorted({v for i in set(ownerids) & active for v in byid[i]['vertices']} - fixed))
            if g:
                groups.add(g)
        if not groups:
            break
        if not (enrich and shape):
            with progress.span('coordinate.star_index'):
                stars = star_index(new, {v for row in groups for v in row})
        if enrich and shape:
            groups=(groups|guided_pending)-guided_seen
        elif enrich:
            groups = {tuple(sorted(set(row) | {v for u in row for c in stars[u] for v in c['vertices'] if v not in fixed})) for row in groups}
        if enrich:
            with progress.span('coordinate.star_index'):
                stars = star_index(new, {v for row in groups for v in row})
        pass_start = len(records)
        for row in sorted(groups, key=lambda row: (len(row), row)):
            if enrich and shape:guided_seen.add(row)
            progress.count('coordinate_groups_visited')
            progress.set(group_index=len(records), groups_planned=len(groups), group_vertices=len(row))
            tick()
            with progress.span('coordinate.group_closure'):
                union = {c['id']: c for v in row for c in stars[v]}
            faces = {key(f) for c in union.values() for f in c['faces']}
            owners = {f: s['owners'][f] for f in faces}
            needed = {i for uses in owners.values() for i, _ in uses}
            progress.set(group_cells=len(union), group_neighbours=len(needed))
            cells = [byid[i] for i in sorted(needed)]
            local_selected = set(union)
            local_outer = local_outer_faces(union.values(), outerkeys)
            with progress.span('coordinate.local_quality'):
                before = quality(x, cells, owners, local_selected, local_outer)
            if _session is not None:_session.before(s,q,local_selected,faces,before)
            bad = raw_failures(before)
            if not bad:
                continue
            count = s['controls'].get('seed_rows', s['controls'].get('rows'))
            with progress.span('coordinate.model_init'):
                model = LocalRepair(x, cells, owners, local_selected, set(range(len(x))) - set(row), before['failures'])
            base = x[list(row)].copy()
            scale = s['controls']['maximum_distance_m'] / count
            with progress.span('coordinate.volume_scale'):
                volume_scale = np.asarray([abs(t.geometry(x, c)['volume']) for c in union.values()])
            if np.any(volume_scale <= 0):
                raise ValueError('NonpositiveTetLayerInitializer')
            ordered = list(union.values())
            with progress.span('coordinate.fixed_neighbour_geometry'):
                model.scope(ordered)
            cache = {}

            def values(z):
                tick()
                k = tuple(z)
                progress.count('constraint_requests')
                if k not in cache:
                    model.points[list(row)] = base + scale * z.reshape(-1, 3)
                    with progress.span('coordinate.constraints'):
                        vals = evaluate()
                    extra = []
                    if s['controls']['schema'] == 'V4TetLayerAdapter/2':
                        from shape_transition_builder import envelope_constraints
                        with progress.span('coordinate.envelope'):
                            extra = envelope_constraints(s, row, model.points)
                    else:
                        for j, v in enumerate(row):
                            index = v - s['old_point_count']
                            level = index // n + 1
                            source = index % n
                            delta = model.points[v] - s['original'][s['cap_ids'][source]]
                            cap = level / s['controls']['rows'] * s['controls']['maximum_distance_m']
                            extra.extend((1 - float(delta @ delta) / (cap * cap), float(delta @ s['direction'][source]) / cap))
                            if s['bounds'] is not None:
                                extra.extend((model.points[v] - s['bounds'][:, 0]) / cap)
                                extra.extend((s['bounds'][:, 1] - model.points[v]) / cap)
                    cache.clear()
                    cache[k] = np.r_[vals, extra]
                return cache[k]
            with progress.span('coordinate.plan'), prepared(model, ordered, owners, needed, volume_scale, local_outer) as evaluate:
                zero = np.zeros(3 * len(row))
                old = values(zero)
                passing = old >= 0
                if shape and enrich and getattr(tick,'repair_diagnostics',None) is not None:
                    from transition_repair_plan import group_diagnostic,constraint_labels
                    with progress.span('diagnostic.prepare'):
                        diagnostic=group_diagnostic(s,row,cells,owners,
                            local_selected,local_outer,constraint_labels(ordered,owners),old)
                    tick.repair_diagnostics('guided_group_start',diagnostic)
                with progress.span('coordinate.solve', publish=True):
                    fit = minimize(lambda y: -float(y[-1]), np.r_[zero, old.min()], method='SLSQP', bounds=[(-1, 1)] * len(zero) + [(None, None)], constraints=[dict(type='ineq', fun=lambda y: np.r_[values(y[:-1]) - y[-1], values(y[:-1])[passing]])], options=dict(maxiter=s['controls']['maxiter'], ftol=s['controls']['ftol']))
                after = values(fit.x[:-1])
                with progress.span('coordinate.candidate_copy'):
                    candidate = model.points.copy()
                candidate[list(row)] = base + scale * fit.x[:-1].reshape(-1, 3)
            with progress.span('coordinate.local_quality'):
                current = quality(candidate, cells, owners, local_selected, local_outer)
            atoms = raw_failures(current)
            adopted = bool(atoms <= bad and (len(atoms) < len(bad) or after.min() > old.min()) and np.all(after[len(after) - (2 if s['bounds'] is None else 8) * len(row):] >= 0))
            if adopted and s['controls']['schema'] == 'V4TetLayerAdapter/2':
                from shape_transition_builder import envelope_subset_passed
                adopted = envelope_subset_passed(s, row, candidate)
            if adopted:
                made_progress = True
                x[list(row)] = candidate[list(row)]
                for k, r in list(failures.items()):
                    ownerids = r['owners'] if r['kind'] == 'face_quality' else [r['id']]
                    if set(ownerids) & local_selected:
                        del failures[k]
                for r in current['failures']:
                    failures[record_key(r)] = r
                if _session is not None:
                    q=_session.coordinate_commit(s,q,local_selected,faces,current)
                    failures={record_key(r):r for r in q['failures']}
            growth=None
            if shape and enrich and not current['passed']:
                from transition_repair_plan import constraint_labels,guided_vertex
                labels=constraint_labels(ordered,owners)
                vertex,growth=guided_vertex(row,labels,after,byid,s['old_point_count'])
                if vertex is not None:
                    expanded=tuple(sorted((*row,vertex)))
                    if expanded not in guided_seen:guided_pending.add(expanded)
                    growth=dict(growth,added_vertex=int(vertex),expanded_vertices=list(expanded))
            emit=getattr(tick,'repair_diagnostics',None)
            if emit is not None and shape:
                emit('coordinate_trial',dict(moving_vertices=list(row),adopted=adopted,
                    before_failures=before['failures'],after_failures=current['failures'],
                    constraint_count=len(after),affected_cells=len(union),total_closure_cells=len(needed),
                    base=base.tolist(),proposal=candidate[list(row)].tolist(),growth=growth,
                    support_policy='constraint_guided' if enrich else 'owner_vertices',proposal_only=True))
            records.append(dict(vertices=list(row), adopted=adopted, raw_before=len(bad), raw_after=len(atoms), solver_success=bool(fit.success), support_enriched=enrich, minimum_slack_before=float(old.min()), minimum_slack_after=float(after.min())))
            progress.count('coordinate_groups_finished')
            progress.set(remaining_records=len(failures), last_adopted=adopted, groups_planned=len(groups), group_vertices=len(row), group_cells=len(union))
        sweeps.append(dict(enriched=enrich, groups=len(records) - pass_start, remaining=len(failures), adopted_progress=made_progress))
        if enrich and shape:
            if len(failures)<oldcount:guided_seen.clear();guided_pending.clear()
            elif guided_pending-guided_seen:continue
        if len(failures) >= oldcount and (s['controls']['schema'] == 'V4TetLayerAdapter/2' or not made_progress):
            if not enrich and allow_enriched:
                enrich = True
            else:
                break
    tick()
    if _session is not None:
        q=_session.require(s)
    elif any((r['adopted'] for r in records)) if shape else initial:
        with progress.span('placement.post_coordinate_quality', publish=True):
            q = quality(x, s['cells'], s['owners'], selected, outer, handoff)
    return q, records, sweeps


def _place(s, check=None, handoff=None, progress=None):
    start=time.perf_counter();deadline=start+s['controls']['seconds']
    def tick():
        if check is not None:
            with progress.span('placement.source_guard'):check()
        if time.perf_counter()>=deadline:raise ValueError('TetLayerPlacementTimeLimit')
    tick.repair_progress=progress
    sink=getattr(check,'repair_diagnostics',None)
    if sink is not None:
        from transition_repair_plan import diagnostic_plain
        def retain(event,payload):
            with progress.span('diagnostic.write'):sink(event,diagnostic_plain(payload))
        tick.repair_diagnostics=retain
    else:tick.repair_diagnostics=None
    tick()
    shape=s['controls']['schema']=='V4TetLayerAdapter/2'
    active={c['id'] for c in s['additions']}
    terminal={i for f in s['cap_ids'][s['cap_faces']] for i,_ in s['owners'][key(f)] if i not in active}
    with progress.span('placement.initial_quality',publish=True):
        initial_quality=quality(s['points'],s['cells'],s['owners'],active|terminal,s['outer'],handoff)
    initial=len(initial_quality['failures']);progress.set(initial_records=initial,remaining_records=initial)
    if not shape:
        q,records,sweeps=_coordinates(s,initial_quality,tick,handoff,progress,allow_enriched=True)
        return _placement_result(s,q,records,sweeps,initial,start)
    from transition_repair_plan import quality_session,stage_diagnostic
    from shape_transition_repair import topology_pass
    from shape_transition_contraction import complete
    with quality_session(s,initial_quality) as session:
        q=session.require(s);stage_diagnostic(tick,'initial',s,q)
        q,records,sweeps=_coordinates(s,q,tick,handoff,progress,allow_enriched=False,_session=session)
        for r in records:r['point_namespace']='initial'
        stages=['small_coordinates'];reports=[];topology=None;contraction=None;used=0
        stage_diagnostic(tick,'small_coordinates',s,q)
        for epoch,phase in enumerate(('early','expanded','guided'),1):
            if q['passed']:break
            if phase!='early':
                with progress.span('placement.coordinates_'+phase,publish=True):
                    q,more,steps=_coordinates(s,q,tick,handoff,progress,
                        enrich=phase=='guided',allow_enriched=False,_session=session)
                for r in more:r['point_namespace']='post_connectivity:'+str(epoch)
                records.extend(more);sweeps.extend(steps);stages.append(phase+'_coordinates')
                stage_diagnostic(tick,phase+'_coordinates',s,q)
            if not q['passed']:
                with progress.span('placement.topology_'+phase,publish=True):
                    updated,topology=topology_pass(s,tick,allow_expanded=phase!='early',_session=session)
                for name in ('byid','editable_ids','stars','physical_outer_keys','physical_outer_vertices'):s.pop(name,None)
                s.update(updated);q=session.require(s);stages.append(phase+'_topology')
                reports.append(dict(stage=stages[-1],phase=phase,report=topology))
                stage_diagnostic(tick,phase+'_topology',s,q)
            if not q['passed'] and used<128:
                with progress.span('placement.contraction_'+phase,publish=True):
                    updated,contraction=complete(s,q,check=tick,handoff=handoff,max_proposals=128-used,_session=session)
                s.update(updated);q=session.require(s);used+=len(contraction['trials']);stages.append('contraction')
                reports.append(dict(stage='contraction',phase=phase,report=contraction))
                stage_diagnostic(tick,phase+'_contraction',s,q)
        session.require(s,q);tick()
        if session.edits:
            active={c['id'] for c in s['additions']}
            terminal={i for f in s['cap_ids'][s['cap_faces']] for i,_ in s['owners'][key(f)] if i not in active}
            with progress.span('placement.final_quality',publish=True):
                final=quality(s['points'],s['cells'],s['owners'],active|terminal,s['outer'],handoff)
            if raw_failures(final)!=raw_failures(q):raise ValueError('FinalRepairInventoryMismatch')
        else:final=initial_quality
        stage_diagnostic(tick,'final_quality',s,final);tick()
        result=_placement_result(s,final,records,sweeps,initial,start)
        from shape_transition_builder import envelope_binding,measure
        result.update(interior_contraction=contraction,repair_stage_order=stages,repair_stage_reports=reports,
            contraction_proposals_total=used,inventory_generations=session.generation,
            whole_quality_scans=1+bool(session.edits),
            legacy_topology_and_contraction_fields='Last call only; complete history is repair_stage_reports.',
            vertex_envelope_binding=envelope_binding(s),shape=measure(s),construction=s['construction'],
            topology_construction=topology,coordinate_initial_records=initial)
        return result


def _placement_result(s,q,records,sweeps,initial,start):
    x=s['points'];new=s['additions']
    if x[:s['old_point_count']].tobytes()!=s['original'].tobytes():raise ValueError('ChangedTetLayerPrisms')
    return dict(passed=q['passed'],initial_records=initial,quality=q,groups=records,sweeps=sweeps,
        seconds=time.perf_counter()-start,outer_bulk_neighbours_required=True,points_sha256=array_hash(x),
        tets_sha256=array_hash(np.asarray([c['vertices'] for c in new],dtype='<i8')),controls_identity=identity(s['controls']))


def filling(s, handoff=None):
    """Extend the existing whole accepted domain, including every old witness."""
    x = s['points']
    if handoff is None:
        from cavity_volume_audit import audit
        _, outside, _ = boundary(s['cells'])
        return audit(x, s['cells'], outside, [])
    from background_volume import audit
    run = handoff.view
    fresh = {c['id']: dict(id=c['id'], **t.exact_cell(x, c)) for c in s['additions']}
    return audit(x, handoff.volume_inputs(), set(run.topology.cells), s['additions'], fresh, list(map(tuple, s['outer'])), [tuple(reversed(f)) for f in run.wall])

def admit(s, placement, fluid, check=None, handoff=None, trace=None):
    """Existing collision, independent coverage and exact filling owners."""
    from native_mixed_face_certificate import certify
    from rolling_fast_state import verify_fresh_coverage
    from background_independent import AABBIndex as IndependentIndex

    def mark(name, event):
        if trace is not None:
            trace(name, event)
    mark('admission_prechecks', 'enter')
    if not placement['passed']:
        raise ValueError('TetLayerHardQualityFailure')
    x = s['points']
    n = len(s['cap_ids'])
    controls = s['controls']
    if s['bounds'] is not None and (np.any(x <= s['bounds'][:, 0]) or np.any(x >= s['bounds'][:, 1])):
        raise ValueError('TetTransitionOutsideBackgroundDomain')
    if placement['points_sha256'] != array_hash(x) or placement['tets_sha256'] != array_hash(np.asarray([c['vertices'] for c in s['additions']], dtype='<i8')) or placement['controls_identity'] != identity(controls):
        raise ValueError('ChangedTetLayerPlacementInputs')
    envelope = None
    if variable_shape(controls):
        from shape_transition_builder import envelope_binding
        from shape_transition_domain import verify
        if placement.get('vertex_envelope_binding') != envelope_binding(s):
            raise ValueError('ChangedTransitionEnvelopeWitness')
        envelope = verify(x[s['compact_ids']], s['original'][s['cap_ids']], s['cap_faces'], controls['maximum_distance_m'], s['vertex_envelope'], check)
    else:
        for j in range(controls['rows']):
            delta = x[s['old_point_count'] + j * n:s['old_point_count'] + (j + 1) * n] - s['original'][s['cap_ids']]
            if np.any(np.linalg.norm(delta, axis=1) > (j + 1) / controls['rows'] * controls['maximum_distance_m']) or np.any(np.einsum('ij,ij->i', delta, s['direction']) <= 0):
                raise ValueError('TetLayerExpansionDistanceFailure')
    if check:
        check()
    mark('admission_prechecks', 'exit')
    mark('source_fluid', 'enter')
    fluid_report = fluid(x, s['additions'])
    mark('source_fluid', 'exit')
    if type(fluid_report) is not dict or fluid_report.get('passed') is not True:
        raise ValueError('TetLayerFluidSideFailure')
    mark('owner_preparation', 'enter')
    faces = [rows[0][1] for _, rows in sorted(s['owners'].items())]
    new = {c['id'] for c in s['additions']}
    if handoff is not None:
        from rolling_fast_state import BackgroundInputs
        if type(handoff) is not BackgroundInputs:
            raise ValueError('LiveTetPrismProofOwnerRequired')
        handoff.require(handoff.view)
        if x[:s['old_point_count']].tobytes() != handoff.view.points.tobytes():
            raise ValueError('ChangedInheritedTetPrismGeometry')
    active = [i for i, (k, uses) in enumerate(sorted(s['owners'].items())) if (k not in s['old_face_keys'] if handoff is not None else any((owner in new for owner, f in uses)))]
    mark('owner_preparation', 'exit')
    mark('collision', 'enter')
    c = certify(x, faces, active, workers=4)
    mark('collision', 'exit')
    if not c['passed'] and getattr(check,'repair_diagnostics',None) is not None:
        from termination_geometry_certificate import rplain
        check.repair_diagnostics('transition_collision_failure',rplain({k:v for k,v in c.items() if k!='proofs'}))
    mark('coverage', 'enter')
    methods = verify_fresh_coverage(x, faces, active, c, index_type=IndependentIndex)
    mark('coverage', 'exit')
    if not c['passed']:
        raise ValueError('TetLayerContinuousCollisionFailure')
    mark('filling', 'enter')
    v = filling(s, handoff)
    mark('filling', 'exit')
    if not (v['positive_cells'] and v['exact_volume_balance'] and v['interior_degree']['passed']):
        raise ValueError('TetLayerExactFillingFailure')
    if check:
        check()
    result = dict(passed=True, fluid=fluid_report, collision=dict(passed=True, required_pairs=c['required_pairs'], proof_hash=c['proof_hash'], methods=dict(methods), independent_coverage_passed=True), filling={k: z for k, z in v.items() if k not in ('exact_cells', 'interior_degree', 'witness_ids')}, interior_degree=dict(passed=v['interior_degree']['passed'], unresolved=v['interior_degree']['unresolved'], witnesses=len(v['interior_degree']['values']), scope='Every inherited prism and fresh transition witness against the complete extended boundary'), inherited_prism_face_relations=handoff.view.live_acceptance.count if handoff is not None else 0, inherited_prism_mesh=handoff.view.live_acceptance.mesh if handoff is not None else None, scope='Constructed transition only; unchanged old-old face relations inherit live prism proof under protected custody. Actual outer bulk-neighbour quality remains mandatory.')
    if envelope is not None:
        result['vertex_envelope'] = envelope
    return result

def place(s, check=None, handoff=None):
    from repair_timing import RepairProgress
    progress = RepairProgress.from_check(check)
    try:
        with progress.span('placement', publish=True):
            if s['controls']['schema']==CUSTOM_SCHEMA:
                from custom_transition_placement import place as custom_place
                result=custom_place(s,check,handoff,progress)
            else:
                result = _place(s, check, handoff, progress)
    except BaseException as error:
        progress.outcome='raised';progress.terminal_error=repr(error)
        raise
    else:
        progress.outcome='returned';progress.numeric_passed=bool(result['passed'])
    finally:
        try:progress.close()
        except BaseException as error:
            progress.outcome='raised';progress.terminal_error=repr(error);progress.numeric_passed=None
            raise
    result['timing'] = progress.snapshot()
    return result
