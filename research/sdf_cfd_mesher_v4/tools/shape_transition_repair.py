"""Private elementary topology construction through existing V4 owners.

No new mesher/predicate, coordinate motion, point insertion,
publication or proof credential. The caller still runs ordinary place/admit.
The required native seed row is not a lock on subsequent fresh construction.
"""
from collections import Counter
import time
import numpy as np
from background_topology_repair import CoreTopology, key, cavity_geometry
from background_core_repair import atoms, affected_rows, replace_rows, PROPOSAL_REJECTIONS
from refined_local_repair import record_key
from sdf_cfd_mesher_v4.common.authority import array_hash, identity

def topology_pass(s, check=None, *, allow_expanded=True, _session=None):
    """Return (private coherent state, diagnostic), with coordinates unchanged.

    Existing elementary proposals precede existing expanded cavity proposals.
    Expanded generation starts only after elementary proposals stall. Actual-neighbour
    quality uses the existing transition owner and declared prospective outer
    mask. Every committed cavity has existing exact relative admission before
    and after, and strictly fewer complete failure atoms with no new atoms.
    """
    if type(allow_expanded) is not bool:raise ValueError('ExplicitTopologyExpansionPolicyRequired')
    from repair_timing import RepairProgress
    progress = RepairProgress.from_check(check)
    from tet_layer_adapter import quality, local_outer_faces
    started = time.perf_counter()

    def tick():
        if check is not None:
            check()
    tick()
    points = s['points']
    frozen = points.view()
    frozen.flags.writeable = False
    point_hash = array_hash(points)
    initial_ids = {c['id'] for c in s['additions']}
    old = [c for c in s['cells'] if c['id'] not in initial_ids]
    old_ids = {c['id'] for c in old}
    old_hash = identity(old)
    owners = {k: list(uses) for k, uses in s['owners'].items()}
    state = CoreTopology(frozen, s['cells'], owners, initial_ids, s['outer'])
    cap = np.asarray(s['cap_ids'])[np.asarray(s['cap_faces'])]
    terminal = {i for face in cap for i, _ in owners[key(face)] if i in old_ids}
    cap_hash = array_hash(cap)
    outer_hash = array_hash(np.asarray(s['outer']))
    if _session is not None:initial=_session.require(s)
    else:
        with progress.span('topology.quality'):
            initial = quality(frozen, s['cells'], owners, initial_ids | terminal, s['outer'])
    tick()
    if any((r['kind'] == 'exact_cell' or (r['kind'] == 'cell_quality' and r['volume'] <= 0) for r in initial['failures'])):
        raise ValueError('ShapeTopologyInitialPositiveGeometryRequired')
    reports = {record_key(r): r for r in initial['failures']}
    initial_atoms = atoms(initial)
    progress.set(topology_initial_atoms=len(initial_atoms), topology_remaining_atoms=len(initial_atoms))
    current_atoms = set(initial_atoms)
    progress.set(topology_remaining_atoms=len(current_atoms))
    seen = set()
    trials = []
    edits = []
    rejections = Counter()
    touched_old = set()
    phase = 'elementary'
    progress.set(topology_phase=phase)
    for sweep in range(2 * len(initial_atoms) + 2):
        made_progress = False
        seeds = sorted({i for r in reports.values() for i in (r['owners'] if r['kind'] == 'face_quality' else [r['id']])} & state.core_ids)
        for seed in seeds:
            tick()
            if seed not in state.core_ids or not any((seed in (r['owners'] if r['kind'] == 'face_quality' else [r['id']]) for r in reports.values())):
                continue
            passing = []
            with progress.span('topology.inventory'):
                inventory = state.inventory(seed) if phase == 'elementary' else state.expanded_inventory(seed, tick)
            for order, proposal in enumerate(progress.iterate(inventory, 'topology.enumeration')):
                tick()
                allowed = ('2to3', '3to2', '4to4') if phase == 'elementary' else ('closed_edge_retriangulation', 'local_delaunay')
                if proposal['operation'] not in allowed:
                    raise ValueError('UnexpectedShapeTopologyProposal')
                if 'generation_rejection' in proposal:
                    reason = proposal['generation_rejection']
                    rejections[reason] += 1
                    trials.append(dict(phase=phase, operation=proposal['operation'], remove=list(proposal['remove']), rejection=reason, detail=proposal.get('detail')))
                    progress.count('topology_trials')
                    continue
                stamp = state.stamp(proposal)
                if stamp in seen:
                    continue
                seen.add(stamp)
                row = dict(phase=phase, operation=proposal['operation'], remove=list(proposal['remove']))
                try:
                    with progress.span('topology.proposal_view'):
                        view = state.view(proposal)
                except ValueError as error:
                    if not error.args or error.args[0] not in PROPOSAL_REJECTIONS:
                        raise
                    row['rejection'] = str(error)
                    rejections[str(error)] += 1
                    trials.append(row)
                    progress.count('topology_trials')
                    continue
                selected = {c['id'] for c in view['added']}
                outer = local_outer_faces(view['added'], state.far)
                with progress.span('topology.quality'):
                    q = quality(frozen, view['cells'], view['owners'], selected, outer)
                tick()
                removed = affected_rows(reports, set(proposal['remove']), view['touched'])
                previous = atoms(dict(failures=removed))
                following = atoms(q)
                candidate_atoms = current_atoms - previous | following
                row.update(before_atoms=len(previous), after_atoms=len(following), new_atoms=len(following - previous))
                if following <= previous and candidate_atoms < current_atoms:
                    passing.append((len(candidate_atoms), -q['summary']['minimum_determinant'], -q['summary']['minimum_weight'], order, proposal, view, q, candidate_atoms, row))
                else:
                    row['rejection'] = 'NoStrictFailureAtomDecrease'
                    rejections[row['rejection']] += 1
                trials.append(row)
                progress.count('topology_trials')
            if not passing:
                continue
            _, _, _, _, proposal, view, q, candidate_atoms, row = min(passing, key=lambda z: z[:4])
            tick()
            if _session is not None:
                old_keys={key(f) for c in view['removed'] for f in c['faces']}
                old_owners={f:owners[f] for f in old_keys}
                old_needed={i for uses in old_owners.values() for i,_ in uses}
                with progress.span('topology.local_before_quality'):
                    old_q=quality(frozen,[state.byid[i] for i in sorted(old_needed)],old_owners,
                        set(proposal['remove']),local_outer_faces(view['removed'],state.far))
                affected=affected_rows(reports,set(proposal['remove']),view['touched'])
                if atoms(old_q)!=atoms(dict(failures=affected)):
                    raise ValueError('TopologyAffectedInventoryMismatch')
            with progress.span('topology.exact_cavity'):
                before = cavity_geometry(frozen, view['removed'])
            tick()
            with progress.span('topology.exact_cavity'):
                after = cavity_geometry(frozen, view['added'])
            tick()
            if before.get('passed') is not True or after.get('passed') is not True:
                raise ValueError('ShapeTopologyRequiredCavityAdmissionFailed')
            if before['boundary_chain'] != after['boundary_chain']:
                raise ValueError('ChangedAdmittedShapeCavityBoundary')
            if array_hash(points) != point_hash:
                raise ValueError('ChangedShapeTopologyCoordinates')
            for f in view['touched']:
                if any((i in old_ids for i, _ in owners.get(f, []))):
                    touched_old.add(f)
            old_count = len(current_atoms)
            replace_rows(reports, set(proposal['remove']), view['touched'], q['failures'])
            state.commit(view)
            current_atoms = candidate_atoms
            progress.set(topology_remaining_atoms=len(current_atoms))
            row['committed'] = True
            edits.append(dict(phase=phase, operation=proposal['operation'], removed=list(proposal['remove']), added=[dict(id=c['id'], vertices=c['vertices']) for c in view['added']], atoms_before=old_count, atoms_after=len(current_atoms), before_geometry=before, after_geometry=after, unchanged_coordinates=True))
            made_progress = True
        if not made_progress:
            if phase == 'elementary' and current_atoms and allow_expanded:
                phase = 'expanded'
                progress.set(topology_phase=phase)
                continue
            break
        phase = 'elementary'
        progress.set(topology_phase=phase)
    else:
        raise ValueError('ShapeTopologyStrictReductionBound')
    tick()
    additions = [state.byid[i] for i in sorted(state.core_ids)]
    cells = old + additions
    if _session is not None:
        final=dict(passed=not reports,failures=list(reports.values()),diagnostic_failure_inventory=True,scientific_mesh_acceptance=False)
    else:
        with progress.span('topology.quality'):
            final = quality(frozen, cells, owners, state.core_ids | terminal, s['outer']) if edits else initial
    tick()
    if atoms(final) != current_atoms or not atoms(final) <= initial_atoms:
        raise ValueError('ShapeTopologyFullFailureDeltaMismatch')
    if identity([state.byid[c['id']] for c in old]) != old_hash:
        raise ValueError('ChangedShapeTopologyOldCells')
    for f in touched_old:
        if [(i, face) for i, face in owners[f] if i in old_ids] != [(i, face) for i, face in s['owners'][f] if i in old_ids]:
            raise ValueError('ChangedShapeTopologyOldOwnerBinding')
    if array_hash(points) != point_hash or array_hash(cap) != cap_hash or array_hash(np.asarray(s['outer'])) != outer_hash:
        raise ValueError('ChangedShapeTopologyCoordinatesOrBoundary')
    diagnostic = dict(expanded_allowed=allow_expanded, method='ExistingElementaryThenExpandedClosedCavityConstruction/2', initial_records=len(initial['failures']), remaining_records=len(final['failures']), initial_atoms=len(initial_atoms), remaining_atoms=len(current_atoms), remaining=final['failures'], trials=trials, rejections=dict(rejections), edits=edits, points_sha256=point_hash, tetrahedra_sha256=array_hash(np.asarray([c['vertices'] for c in additions], dtype='<i8')), old_cells_unchanged=True, coordinates_unchanged=True, cap_outer_boundary_unchanged=True, native_seed_row_locked=False, fresh_place_and_admit_required=True, quality=final, scientific_mesh_acceptance=False, seconds=time.perf_counter() - started)
    result = dict(s, cells=cells, additions=additions, owners=owners, topology_construction=diagnostic)
    # Connectivity is current; optional contraction indices belonged to the
    # preceding namespace. Their owners rebuild them when next requested.
    for name in ('byid','editable_ids','stars','physical_outer_keys','physical_outer_vertices'):
        result.pop(name,None)
    if _session is not None:
        _session.require(s,initial)
        diagnostic['quality']=_session.advance(result,final['failures'],changed=bool(edits))
    return (result, diagnostic)
