"""Diagnostic local state only; no acceptance or restart authority."""
from math import isfinite


def capture(points, cells, owners, front_ids, new_ids, faces, normals, steps,
            target, prior_steps, edges, lengths, quality, increment, attempt,
            protected_point_count, growth):
    failures = quality.get('failures', [])
    candidates = [row['id'] for row in failures if row.get('kind') == 'exact_cell']
    candidates.extend(row['id'] for row in failures if row.get('kind') == 'cell_quality')
    if not candidates:
        candidates.extend(owner for row in failures for owner in row.get('owners', []))
    by_id = {cell['id']: cell for cell in cells}
    seed = next((stable for stable in candidates if stable in by_id), None)
    if seed is None:
        raise ValueError('MissingFailureWitnessCell')
    new_lookup = {int(vertex): index for index, vertex in enumerate(new_ids)}
    moving = sorted(set(by_id[seed]['vertices']) & set(new_lookup))
    if not moving:
        raise ValueError('MissingFailureWitnessNewVertices')
    moving_set = set(moving)
    active = {cell['id'] for cell in cells if moving_set.intersection(cell['vertices'])}
    required = set(active)
    for cell in cells:
        if cell['id'] in active:
            for face in cell['faces']:
                required.update(owner for owner, unused in owners[tuple(sorted(face))])
    retained_cells = [cell for cell in cells if cell['id'] in required]
    local_indices = {new_lookup[vertex] for vertex in moving}
    selected_edges = [index for index, edge in enumerate(edges)
                      if any(int(vertex) in local_indices for vertex in edge)]
    expanded_indices = set(local_indices)
    for index in selected_edges:
        expanded_indices.update(map(int, edges[index]))
    local_faces = [list(map(int, face)) for face in faces
                   if any(int(vertex) in local_indices for vertex in face)]
    for face in local_faces:
        expanded_indices.update(face)
    vertex_ids = {int(vertex) for cell in retained_cells for vertex in cell['vertices']}
    vertex_ids.update(int(front_ids[index]) for index in expanded_indices)
    vertex_ids.update(int(new_ids[index]) for index in expanded_indices)
    vertex_ids = sorted(vertex_ids)
    coordinates = [[float(value) for value in points[vertex]] for vertex in vertex_ids]
    if not all(isfinite(value) for point in coordinates for value in point):
        raise ValueError('NonfiniteFailureWitness')
    keys = {tuple(sorted(face)) for cell in retained_cells for face in cell['faces']}
    return dict(schema='GradedFailureLocalWitness/1',diagnostic_only=True,
        scientific_acceptance=False,restart_supported=False,increment=increment,attempt=attempt,
        selection='First exact-failing cell, else first quality-failing cell, else first failing-face owner; no fixture dispatch',
        seed_cell=seed,movable_vertex_ids=moving,active_cell_ids=sorted(active),
        vertex_ids=vertex_ids,points=coordinates,cells=retained_cells,
        owners=[dict(vertices=list(key),rows=[dict(owner=owner,face=list(face)) for owner,face in owners[key]])
                for key in sorted(keys)],
        front=[dict(index=index,old_id=int(front_ids[index]),new_id=int(new_ids[index]),
                    normal=list(map(float,normals[index])),step=float(steps[index]),
                    target=float(target[index]),
                    prior_step=None if prior_steps is None else float(prior_steps[index]))
               for index in sorted(expanded_indices)],
        front_faces=local_faces,
        envelope_edges=[dict(indices=list(map(int,edges[index])),length=float(lengths[index]))
                        for index in selected_edges],
        protected_point_count=protected_point_count,growth=float(growth),
        outer_faces=[list(map(int,new_ids[face])) for face in faces
                     if any(int(vertex) in local_indices for vertex in face)],
        quality=quality,
        limitations='Local diagnostic patch, not a closed domain, complete transition, restart state or admission credential')
