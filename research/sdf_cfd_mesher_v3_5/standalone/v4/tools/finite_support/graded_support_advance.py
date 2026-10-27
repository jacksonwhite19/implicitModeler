"""Diagnostic shared-front cell addition; never issues live acceptance."""
import heapq
import math
import time
import numpy as np

SCHEMA = 'RetainedGradedSupportAdvance/1'


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError('InvalidDiagnosticPopulation')
    return dict(zip(('min', 'p05', 'median', 'p95', 'max'),
                    map(float, np.quantile(values, (0, .05, .5, .95, 1)))))


def front_geometry(points, faces):
    normals = np.zeros_like(points)
    triangles = points[faces]
    weighted = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    for column in range(3):
        np.add.at(normals, faces[:, column], weighted)
    lengths = np.linalg.norm(normals, axis=1)
    if not np.isfinite(lengths).all() or np.any(lengths <= 0):
        raise ValueError('InvalidCurrentFrontNormals')
    normals /= lengths[:, None]
    edges = np.unique(np.sort(np.concatenate((faces[:, :2], faces[:, 1:], faces[:, ::2])), axis=1), axis=0)
    lengths = np.linalg.norm(points[edges[:, 1]]-points[edges[:, 0]], axis=1)
    if np.any(lengths <= 0) or not np.isfinite(lengths).all():
        raise ValueError('InvalidCurrentFrontEdges')
    shortest = np.full(len(points), np.inf)
    for column in range(2):
        np.minimum.at(shortest, edges[:, column], lengths)
    if not np.isfinite(shortest).all():
        raise ValueError('UnreferencedFrontVertex')
    return normals, edges, lengths, shortest


def envelope(steps, edges, lengths, growth, check):
    if not math.isfinite(growth) or growth <= 0:
        raise ValueError('ExplicitPositiveGrowthRequired')
    result = np.asarray(steps, dtype=float).copy()
    if not np.isfinite(result).all() or np.any(result <= 0):
        raise ValueError('PositiveRepresentableStepRequired')
    neighbours = [[] for unused in result]
    for (left, right), length in zip(edges, lengths):
        neighbours[left].append((int(right), growth*float(length)))
        neighbours[right].append((int(left), growth*float(length)))
    queue = [(float(value), index) for index, value in enumerate(result)]
    heapq.heapify(queue)
    count = 0
    while queue:
        value, index = heapq.heappop(queue)
        count += 1
        if count % 1024 == 0:
            check()
        if value != result[index]:
            continue
        for neighbour, cost in neighbours[index]:
            proposed = value+cost
            if proposed < result[neighbour]:
                result[neighbour] = proposed
                heapq.heappush(queue, (proposed, neighbour))
    return result


def implicated(quality, cells, new_ids):
    selected = set()
    by_id = {cell['id']: cell for cell in cells}
    lookup = {int(vertex): index for index, vertex in enumerate(new_ids)}
    for failure in quality.get('failures', []):
        vertices = list(failure.get('vertices', []))
        owner_ids = list(failure.get('owners', []))
        if 'id' in failure:
            owner_ids.append(failure['id'])
        for owner in owner_ids:
            if owner in by_id:
                vertices.extend(by_id[owner]['vertices'])
        selected.update(lookup[vertex] for vertex in vertices if vertex in lookup)
    return selected


def construct(base, field, nominal, seconds, adapter, construction, check, publish):
    from background_materialization import build
    from shape_transition_topology import verify_attachment
    if not math.isfinite(nominal) or nominal <= 0 or not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('ExplicitNominalAndConstructionBudgetRequired')
    started = time.perf_counter()
    def guard():
        check()
        if time.perf_counter()-started >= seconds:
            raise TimeoutError('ConstructionBudgetExhausted')
    original = base['original']
    cap_ids, faces = base['cap_ids'], base['cap_faces']
    protected = base['cells'][:len(base['protected_cell_ids'])]
    if ([cell['id'] for cell in protected] != base['protected_cell_ids']
            or any(cell['kind'] != 'prism' for cell in protected)
            or any(cell['kind'] != 'tetrahedron' for cell in base['cells'][len(protected):])):
        raise ValueError('AuthenticatedProtectedPrismPopulationRequired')
    protected_ids = {cell['id'] for cell in protected}
    oldowners = {key: [(owner, face) for owner, face in rows if owner in protected_ids]
                 for key, rows in base['owners'].items()}
    oldowners = {key: rows for key, rows in oldowners.items() if rows}
    digest = construction.inherited_digest(protected, oldowners, guard)
    if digest != base['protected_digest']:
        raise ValueError('AuthenticatedProtectedDigestMismatch')
    points = original.copy()
    cells = list(protected)
    owners = oldowners
    additions = []
    front_ids = cap_ids.copy()
    prior_steps = None
    prior_normals = None
    records = []
    state = None
    reason = 'NOT_STARTED'
    initial = original[cap_ids]
    bounds = base['bounds']
    try:
        construction.surface(initial, faces, guard)
        while True:
            guard()
            front = points[front_ids]
            normals, edges, lengths, shortest = front_geometry(front, faces)
            target = []
            for index, position in enumerate(front):
                if index % 64 == 0:
                    guard()
                target.append(field.evaluate(position))
            target = np.asarray(target)
            steps = np.minimum(target, math.sqrt(2/3)*shortest)
            if prior_steps is not None:
                steps = np.minimum(steps, prior_steps*(1+field.g))
            steps = envelope(steps, edges, lengths, field.g, guard)
            new_ids = np.arange(len(points), len(points)+len(front), dtype='<i8')
            mapped = np.r_[front_ids, new_ids][adapter.product(faces, len(front), 1)]
            new_cells = build(mapped, max(cell['id'] for cell in cells)+1)
            trial_cells = cells+new_cells
            trial_owners = {key: list(rows) for key, rows in owners.items()}
            for cell in new_cells:
                for face in cell['faces']:
                    trial_owners.setdefault(tuple(sorted(face)), []).append((cell['id'], face))
            affected = {cell['id'] for cell in new_cells}
            affected.update(owner for face in front_ids[faces]
                            for owner, unused in owners[tuple(sorted(face))])
            outer = new_ids[faces]
            attachment = verify_attachment(new_cells, trial_owners, front_ids[faces], outer,
                                           {cell['id'] for cell in cells}, guard)
            if not attachment['passed']:
                raise ValueError('IncrementAttachmentFailure')
            attempts = []
            while True:
                guard()
                fresh = front+normals*steps[:, None]
                actual_steps = np.linalg.norm(fresh-front, axis=1)
                if np.any(actual_steps <= 0) or not np.isfinite(fresh).all():
                    raise ValueError('NoRepresentableProgress')
                temporal_limit = target if prior_steps is None else np.minimum(target, prior_steps*(1+field.g))
                excess = np.maximum(actual_steps-temporal_limit, 0.)
                differences = actual_steps[edges[:, 0]]-actual_steps[edges[:, 1]]
                residual = np.abs(differences)-field.g*lengths
                for column in range(2):
                    larger = differences > 0 if column == 0 else differences < 0
                    np.maximum.at(excess, edges[larger, column], np.maximum(residual[larger], 0.))
                if np.any(excess > 0):
                    corrected = np.nextafter(steps-excess, 0.)
                    steps = envelope(corrected, edges, lengths, field.g, guard)
                    continue
                trial_points = np.vstack((points, fresh))
                invalid = np.flatnonzero(np.any((fresh <= bounds[:, 0]) | (fresh >= bounds[:, 1]), axis=1))
                quality = None
                collision = None
                if len(invalid):
                    rejected = set(map(int, invalid))
                    failure = dict(reason='WORLD_BOUNDS', vertices=len(rejected))
                else:
                    quality = adapter.quality(trial_points, trial_cells, trial_owners, affected, outer, None)
                    guard()
                    if not quality['passed']:
                        rejected = implicated(quality, trial_cells, new_ids)
                        failure = dict(reason='HARD_QA', quality=quality)
                        if not rejected:
                            raise ValueError('UnmappedQualityFailure')
                    else:
                        all_faces = [rows[0][1] for rows in trial_owners.values()]
                        new_cell_ids = {cell['id'] for cell in new_cells}
                        active = [index for index, rows in enumerate(trial_owners.values())
                                  if any(owner in new_cell_ids for owner, unused in rows)]
                        collision = construction.collision(trial_points, all_faces, active, guard)
                        if collision['passed'] and collision['independent_coverage_passed']:
                            break
                        failure = dict(reason='COLLISION', collision=collision)
                        attempts.append(failure)
                        publish('increment_%04d_rejected' % len(records), dict(attempts=attempts))
                        raise ValueError('RequiredCollisionFailure')
                attempts.append(failure)
                publish('increment_%04d_attempt_%04d' % (len(records), len(attempts)), failure)
                if quality is not None:
                    from finite_support.failure_local_witness import capture
                    witness = capture(trial_points, trial_cells, trial_owners, front_ids, new_ids,
                        faces, normals, steps, target, prior_steps, edges, lengths, quality,
                        len(records)+1, len(attempts), len(original), field.g)
                    publish('FIRST_REJECTION_WITNESS', witness)
                raise ValueError('ObservationStoppedAtFirstRejectedProposal')
                reduced = steps.copy()
                reduced[list(rejected)] *= .5
                updated = envelope(reduced, edges, lengths, field.g, guard)
                if np.array_equal(updated, steps):
                    raise ValueError('NoRepresentableBackoff')
                steps = updated
            guard()
            candidate = dict(points=trial_points, original=original, cells=trial_cells,
                additions=additions+new_cells, owners=trial_owners, outer=outer,
                tets=np.asarray([cell['vertices'] for cell in additions+new_cells], dtype='<i8'),
                cap_ids=cap_ids, cap_faces=faces,
                compact_ids=np.r_[cap_ids, np.arange(len(original), len(trial_points), dtype='<i8')],
                old_point_count=len(original), old_face_keys=set(oldowners),
                outer_vertex_ids=new_ids, wall=base['wall'], bounds=bounds,
                controls=dict(quality_targets=construction.d.TARGETS,
                    envelope=dict(schema='TransitionExtentPolicy/1', mode='nominal', nominal_distance_m=nominal)),
                producer=SCHEMA, provenance=dict(schema=SCHEMA, growth=field.g, h_max=field.h_max,
                    sizing_authority=base['sizing_authority'],
                    protected_cell_ids=base['protected_cell_ids'],protected_digest=digest,
                    extent_measure='unweighted_current_front_original_cap_displacement_median',
                    copied_front=True, lateral_coarsening=False))
            candidate['backend_to_global'] = candidate['compact_ids'].copy()
            candidate['inherited_preservation'] = construction.verify_inherited_join(
                candidate, protected, oldowners, digest, guard)
            if candidate['points'][:len(original)].tobytes() != original.tobytes():
                raise ValueError('ChangedProtectedCoordinates')
            displacement = np.linalg.norm(fresh-initial, axis=1)
            record = dict(increment=len(records)+1, added_vertices=len(new_ids), added_cells=len(new_cells),
                total_support_cells=len(candidate['additions']), target_size_m=distribution(target),
                accepted_step_m=distribution(actual_steps), step_to_field_ratio=distribution(actual_steps/target),
                tangential_edges_m=distribution(np.linalg.norm(fresh[edges[:, 1]]-fresh[edges[:, 0]], axis=1)),
                displacement_m=distribution(displacement), normal_projection_m=distribution(np.sum((fresh-initial)*normals, axis=1)),
                rejected_attempts=len(attempts), quality=quality, collision=collision,
                attachment=attachment, realized_spatial_residual_max_m=float(np.max(residual)),
                realized_growth_residual_max_m=None if prior_steps is None else float(np.max(actual_steps-prior_steps*(1+field.g))),
                temporal_ratio=None if prior_steps is None else distribution(actual_steps/prior_steps),
                direction_dot=None if prior_normals is None else distribution(np.sum(normals*prior_normals, axis=1)))
            publish('increment_%04d_committed' % len(records), record)
            state = candidate
            records.append(record)
            points, cells, owners = trial_points, trial_cells, trial_owners
            additions, front_ids = candidate['additions'], new_ids
            prior_steps, prior_normals = actual_steps, normals
            if float(np.median(displacement)) >= nominal:
                reason = 'NOMINAL_MEDIAN_REACHED'
                break
    except (TimeoutError, ValueError) as error:
        reason = type(error).__name__+':'+str(error)
    report = dict(schema=SCHEMA, stop_reason=reason, extent_reached=reason=='NOMINAL_MEDIAN_REACHED',
        nominal_distance_m=nominal, increments=len(records), records=records,
        elapsed_seconds=time.perf_counter()-started, scientific_acceptance=False,
        original_sdf_fluid_proof='NOT_PERFORMED', lateral_coarsening=False)
    publish('construction', report)
    return state, report
