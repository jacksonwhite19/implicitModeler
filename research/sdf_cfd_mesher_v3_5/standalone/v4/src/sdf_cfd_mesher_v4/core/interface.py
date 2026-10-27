"""Source-keyed outer interface, shared by prism and mixed shells."""
import numpy as np
from ..advancing_front.native_checkpoint import scientific_digest
from ..common.authority import array_hash

def _extract(state, *, digest):
    face_ids = sorted(state['outer'])
    faces = []
    owners = []
    ancestry = []
    for key in face_ids:
        row = state['faces'][key]
        if len(row['owners']) != 1:
            raise ValueError('OuterFaceMustHaveOneShellOwner:' + key)
        owner, ordered = next(iter(row['owners'].items()))
        if sorted(ordered) != sorted(row['vertices']):
            raise ValueError('OwnerFaceConnectivityMismatch')
        cell = state['cells'][owner]
        source = cell['source']
        column = state['columns'].get(source)
        ancestry.append(dict(source=source, cell_type=cell['kind'], layer=cell['layer'],
                             column=column, patch='shell_core_internal'))
        faces.append(ordered)
        owners.append(owner)
    keys = sorted({v for face in faces for v in face})
    index = {key: i for i, key in enumerate(keys)}
    points = np.asarray([state['vertices'][v] for v in keys], dtype='<f8')
    if not np.isfinite(points).all():
        raise ValueError('NonfiniteInterfaceCoordinates')
    offsets = np.cumsum([0] + [len(f) for f in faces], dtype='<i8')
    connectivity = np.asarray([index[v] for f in faces for v in f], dtype='<i8')
    edges = {}
    links = {}
    seen = set()
    for face in faces:
        if len(face) < 3 or len(set(face)) != len(face):
            raise ValueError('InvalidOuterFaceConnectivity')
        q = points[[index[v] for v in face]]
        if not np.any(np.sum(np.cross(q, np.roll(q, -1, axis=0)), axis=0)):
            raise ValueError('ZeroAreaOuterFace')
        canonical = tuple(sorted(face))
        if canonical in seen:
            raise ValueError('DuplicateOuterFace')
        seen.add(canonical)
        for i, vertex in enumerate(face):
            previous, following = face[i-1], face[(i+1) % len(face)]
            graph = links.setdefault(vertex, {})
            graph.setdefault(previous, set()).add(following)
            graph.setdefault(following, set()).add(previous)
        for a, b in zip(face, face[1:] + face[:1]):
            edge = tuple(sorted((a, b)))
            edges.setdefault(edge, []).append(1 if a < b else -1)
    if any(len(v) != 2 or sum(v) != 0 for v in edges.values()):
        raise ValueError('OuterInterfaceNotClosedOrOriented')
    for graph in links.values():
        visited = set()
        pending = [next(iter(graph))]
        while pending:
            vertex = pending.pop()
            if vertex not in visited:
                visited.add(vertex)
                pending.extend(graph[vertex] - visited)
        if len(visited) != len(graph) or any(len(v) != 2 for v in graph.values()):
            raise ValueError('NonManifoldOuterVertex')
    metadata = dict(schema='V4NativeCoreInterface/1', face_ids=face_ids,
                    vertex_ids=keys, shell_owners=owners,
                    ancestry=ancestry,
                    versions=state['versions'], wall_identity=digest(state['wall']),
                    first_layer_identity=digest(state['first_layer']),
                    columns_identity=digest(state['columns']),
                    coordinates=array_hash(points), connectivity=array_hash(connectivity),
                    offsets=array_hash(offsets), faces=len(faces), vertices=len(keys),
                    edges=len(edges), face_arities=sorted(set(map(len, faces))),
                    orientation='outward from shell owner', closed_oriented=True, vertex_links_manifold=True)
    metadata['identity'] = digest(metadata)
    return points, offsets, connectivity, metadata


def extract(state):
    """Compatibility owner: unchanged protocol-aware scientific digest."""
    return _extract(state, digest=scientific_digest)
