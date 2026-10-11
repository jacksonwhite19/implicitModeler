"""Scientific mixed-front state, independent of prism-array storage.

Source IDs are authority. Mesh storage indices are an adapter detail. Ordered
face perimeters define geometry; canonical cycles define oriented identity.
Validation receipts must come from the existing qualified geometry validators.
"""
from copy import deepcopy
from collections import OrderedDict
import math
from ..common.authority import identity

VERSION = 'V4MixedFront/1'
_SCIENTIFIC_HASH_CACHE=OrderedDict()
SCIENTIFIC_HASH_STATS=dict(full=0,reused=0)


def cycle(vertices):
    v = tuple(vertices)
    if len(v) < 3 or len(set(v)) != len(v):
        raise ValueError('DegenerateFaceConnectivity')
    return min(v[i:]+v[:i] for i in range(len(v)))


def face_key(vertices):
    return identity(min(cycle(vertices), cycle(tuple(reversed(vertices)))))


def wall_authority(vertices, triangles):
    """Source-keyed binary64 coordinates and oriented source triangles."""
    rows = {str(k): list(v) for k, v in vertices.items()}
    faces = {str(k): list(cycle(tuple(map(str, v)))) for k, v in triangles.items()}
    if any(v not in rows for f in faces.values() for v in f):
        raise ValueError('MissingWallSourceVertex')
    data = dict(vertices=rows, triangles=faces)
    return dict(**data, sha256=identity(data))


def geometry_inputs(state):
    vertices = state['vertices']
    cells = {k: identity(dict(cell=v, coordinates={i: vertices[i] for i in v['vertices']}))
             for k, v in state['cells'].items()}
    faces = {k: identity(dict(face=v, coordinates={i: vertices[i] for i in v['vertices']}))
             for k, v in state['faces'].items()}
    return {**{'cell:'+k: v for k, v in cells.items()},
            **{'face:'+k: v for k, v in faces.items()},
            'wall': state['wall']['sha256'], **{'authority:'+k: v for k, v in state['authority'].items()}}


def _initial_versions(state):
    """Initial versions need input keys, not unused coordinate digests.

    Only the native adapter's serializable materialization takes this route.
    Exceptional values/types keep geometry_inputs and its original failures.
    This is not a geometry/acceptance cache; no previous state uses this path.
    """
    vertices = state['vertices']
    if (type(state['authority']) is not dict
            or any(type(k) is not str or type(v) is not str for k,v in state['authority'].items())
            or any(type(k) is not str or any(type(x) is not float or not math.isfinite(x) for x in p)
                   for k,p in vertices.items())):
        return None
    for c in state['cells'].values():
        if (type(c['kind']) is not str or type(c['layer']) is not int
                or not -(1 << 63) <= c['layer'] < (1 << 63)
                or any(type(v) is not str or v not in vertices for v in c['vertices'])
                or any(type(v) is not str or v not in vertices for f in c['faces'] for v in f)):
            return None
    if any(type(v) is not str or v not in vertices for f in state['faces'].values() for v in f['vertices']):
        return None
    keys = ['cell:'+k for k in state['cells']] + ['face:'+k for k in state['faces']]
    keys += ['wall'] + ['authority:'+k for k in state['authority']]
    return {k: dict(topology=0, geometry=0) for k in keys}


def scientific_hash(state):
    if state.get('_production_unsealed'):
        raise ValueError('UnsealedProductionStateRequiresFinalSeal')
    # Dictionaries are source/scientific keyed. Transaction timing, journal
    # storage, and proof compression are deliberately outside this identity.
    data={k: state[k] for k in ('version', 'wall', 'first_layer', 'vertices',
        'cells', 'faces', 'columns', 'outer', 'prism_front', 'versions', 'authority',
        'atlas', 'forecast', 'quality', 'certificate', 'events')}
    if 'local_schedule' in state:data['local_schedule']=state['local_schedule']
    from .native_checkpoint import scientific_digest,verified_receipt_token
    certificate=data['certificate']
    from .native_checkpoint import ReceiptOverlay
    root_base=certificate
    while isinstance(root_base,ReceiptOverlay):root_base=root_base.base
    while isinstance(certificate,ReceiptOverlay):
        if hasattr(root_base,'authenticated'):
            if certificate.removed or any(certificate.base.get(k)!=v for k,v in certificate.edits.items()):
                raise ValueError('UnsealedAuthenticatedReceiptEdits')
        elif certificate.edits or certificate.removed:break
        certificate=certificate.base
    if hasattr(certificate,'authenticated'):
        from .authenticated_receipts import component
        components={}
        for k,v in data.items():
            if k=='certificate':continue
            if k=='quality' and verified_receipt_token(v) is not None:
                while isinstance(v,ReceiptOverlay):v=v.base
                components[k]=v.scientific
            else:components[k]=scientific_digest(v)
        components['certificate']=component(certificate)
        from .canonical_blocks import AUTHORITY as BLOCK_AUTHORITY
        schema='V4MixedScientificBlocks/1' if certificate.authenticated['authority']==BLOCK_AUTHORITY else 'V4MixedScientificComponents/1'
        return identity(dict(schema=schema,components=components))
    tokens=tuple(verified_receipt_token(data[k]) for k in ('quality','certificate'))
    key=None
    if all(token is not None for token in tokens):
        # Mutable geometry is rehashed on every call. Only authenticated,
        # immutable receipt components may avoid another historical row walk.
        key=(tokens,scientific_digest({k:v for k,v in data.items() if k not in ('quality','certificate')}))
        if key in _SCIENTIFIC_HASH_CACHE:
            SCIENTIFIC_HASH_STATS['reused']+=1
            _SCIENTIFIC_HASH_CACHE.move_to_end(key)
            return _SCIENTIFIC_HASH_CACHE[key]
    SCIENTIFIC_HASH_STATS['full']+=1
    digest=scientific_digest(data)
    if key is not None:
        _SCIENTIFIC_HASH_CACHE[key]=digest
        if len(_SCIENTIFIC_HASH_CACHE)>32:_SCIENTIFIC_HASH_CACHE.popitem(last=False)
    return digest


def from_mesh(mesh, authority, active_columns=None, previous=None):
    """Boundary adapter for qualified column-template meshes.

    The adapter immediately removes implicit array-index semantics. New
    unstructured backends provide vertex_ids explicitly. The existing template
    backend uses immutable (source vertex, layer) ancestry for every vertex.
    """
    n = len(mesh['wall']['coordinates'])
    wall_indices=mesh.get('wall_vertex_indices',list(range(n)))
    first_indices=mesh.get('first_layer_vertex_indices',list(range(n,2*n)))
    if len(wall_indices)!=n or len(first_indices)!=n or len(set(wall_indices+first_indices))!=2*n:
        raise ValueError('InvalidSourceLocalMapping')
    ids = mesh.get('vertex_ids', [f'layer:{i//n}:source:{i%n}' for i in range(len(mesh['points']))])
    if len(ids) != len(set(ids)) or len(ids) != len(mesh['points']):
        raise ValueError('DuplicateMeshVertexIdentity')
    wall = wall_authority({str(i): p for i, p in enumerate(mesh['wall']['coordinates'])},
                          {str(i): f for i, f in enumerate(mesh['wall']['triangles'])})
    first = {str(i): p for i, p in enumerate(mesh['first_layer'])}
    for i in range(n):
        if mesh['points'][wall_indices[i]] != wall['vertices'][str(i)] or mesh['points'][first_indices[i]] != first[str(i)]:
            raise ValueError('WallFirstLayerMappingMismatch')
    if previous and (previous['wall'] != wall or previous['first_layer'] != first):
        raise ValueError('WallFirstLayerMutation')
    cells = {}; faces = {}; used = set()
    for c in mesh['cells']:
        key = str(c['id'])
        if key in cells:
            raise ValueError('DuplicateCellIdentity')
        vertices = [ids[i] for i in c['vertices']]; used.update(vertices)
        cf = [list(cycle(tuple(ids[i] for i in f))) for f in c['faces']]
        cells[key] = dict(kind=c['kind'], source=str(c['column']), layer=c['layer'],
                          vertices=vertices, faces=sorted(cf))
        for f in cf:
            fk = face_key(f)
            row = faces.setdefault(fk, dict(vertices=list(min(cycle(f), cycle(tuple(reversed(f))))), owners={}))
            row['owners'][key] = f
    for row in faces.values():
        owners = list(row['owners'].values())
        if len(owners) > 2:
            raise ValueError('NonmanifoldCellFace')
        if len(owners) == 2 and cycle(owners[0]) != cycle(tuple(reversed(owners[1]))):
            raise ValueError('InternalFaceOrientationMismatch')
    wall_faces = {face_key(tuple(ids[wall_indices[int(i)]] for i in f)) for f in wall['triangles'].values()}
    outer = sorted(k for k, f in faces.items() if len(f['owners']) == 1 and k not in wall_faces)
    requested = {face_key(tuple(ids[i] for i in f)) for f in mesh['outer']}
    if set(outer) != requested:
        raise ValueError('OuterInterfaceOwnershipMismatch')
    active = set(map(str, active_columns if active_columns is not None else range(len(wall['triangles']))))
    if not active <= set(wall['triangles']):
        raise ValueError('UnknownActiveSourceColumn')
    source_prisms={source:[] for source in wall['triangles']}
    source_outer={source:[] for source in wall['triangles']}
    cell_outer={}
    for key,c in cells.items():
        if c['kind']=='prism':source_prisms[c['source']].append((c['layer'],key))
    for face in outer:
        for owner in faces[face]['owners']:
            source_outer[cells[owner]['source']].append(face)
            cell_outer.setdefault(owner,[]).append(face)
    columns = {}; prism_front = {}
    for source in wall['triangles']:
        last, key = max(source_prisms[source], default=(0, None))
        candidates = cell_outer.get(key,[])
        # A prism top is an exposed triangle above the wall. Side faces have
        # four vertices in the qualified prism template.
        tops = [f for f in candidates if len(faces[f]['vertices']) == 3]
        if source in active and len(tops) != 1:
            raise ValueError('ActiveColumnHasNoUniqueExposedPrismTop')
        columns[source] = dict(status='ActivePrismColumn' if source in active else 'TerminatedPrismColumn',
                               last_prism_layer=last, exposed_prism_face=tops[0] if len(tops) == 1 else None,
                               outer_ownership=source_outer[source])
        if source in active:
            prism_front[source] = tops[0]
    vertices = {ids[i]: list(p) for i, p in enumerate(mesh['points']) if ids[i] in used}
    state = dict(version=VERSION, wall=wall, first_layer=first, vertices=vertices, cells=cells, faces=faces,
                 columns=columns, outer=outer, prism_front=prism_front, authority=deepcopy(authority),
                 versions={}, atlas={}, forecast={}, quality={}, certificate={}, events={})
    if previous is None:
        versions = _initial_versions(state)
        if versions is not None:
            state['versions'] = versions
            return state
    old_inputs = geometry_inputs(previous) if previous else {}; new_inputs = geometry_inputs(state)
    for key, value in new_inputs.items():
        old_version = previous['versions'].get(key, dict(topology=0, geometry=0)) if previous else dict(topology=0, geometry=0)
        family, _, entity = key.partition(':')
        table = {'cell': 'cells', 'face': 'faces'}.get(family)
        topology_changed = bool(previous and table and previous[table].get(entity) != state[table].get(entity))
        state['versions'][key] = dict(topology=old_version['topology']+int(topology_changed),
                                      geometry=old_version['geometry']+int(previous is not None and old_inputs.get(key) != value))
    if previous:
        state['events'] = deepcopy(previous['events'])
        if 'local_schedule' in previous:state['local_schedule']=deepcopy(previous['local_schedule'])
    return state


CACHE_BINDINGS = {
    'cheap_forecast': ['geometry', 'topology', 'schedule'],
    'detailed_metrics': ['geometry', 'topology', 'implementation'],
    'reconstruction_rows': ['geometry', 'topology', 'implementation'],
    'exact_jacobians': ['geometry', 'topology', 'implementation'],
    'future_pairs': ['geometry', 'topology', 'schedule'],
    'certificate_proofs': ['geometry', 'topology', 'implementation'],
    'capacity_evidence': ['geometry', 'topology', 'schedule'],
    'planner_state': ['geometry', 'topology', 'schedule'],
}
INVALIDATION_MATRIX = {op: {name: ('revalidate_scientific_dependencies' if op != 'restart' else 'verify_and_retain')
                         for name in CACHE_BINDINGS}
                       for op in ('coordinate_correction', 'layer_height_change', 'prism_commit', 'termination', 'rollback_remesh', 'restart')}


def rebuild_dependencies(before, after, nominal_schedule, *, dependency_inputs=None, vertex_incidence=None):
    """Local mixed-front atlas and forecast dependency descriptions.

    This builds dependency identities, not geometric forecast samples. A
    production forecaster must populate and validate the proposed geometry.
    """
    old = before or {}; atlas = {}; forecast = {}; counts = dict(atlas_retained=0, forecast_retained=0)
    inputs = geometry_inputs(after) if dependency_inputs is None else dependency_inputs
    vertex_faces=vertex_incidence
    if vertex_faces is None:
        vertex_faces={}
        for key,row in after['faces'].items():
            for vertex in row['vertices']:vertex_faces.setdefault(vertex,set()).add(key)
    for source, face in after['prism_front'].items():
        f = after['faces'][face]
        neighbors = sorted(set().union(*(vertex_faces[v] for v in f['vertices'])))
        deps = {'face:'+k: inputs['face:'+k] for k in neighbors}
        row = dict(source=source, front=face, dependencies=deps,
                   versions={k: after['versions'][k] for k in deps})
        atlas[source] = row
        forecast[source] = dict(region=identity(row), current_layer=after['columns'][source]['last_prism_layer'],
                                remaining_schedule=list(nominal_schedule[after['columns'][source]['last_prism_layer']:]))
        counts['atlas_retained'] += old.get('atlas', {}).get(source) == row
        counts['forecast_retained'] += old.get('forecast', {}).get(source) == forecast[source]
    after['atlas'] = atlas; after['forecast'] = forecast
    counts['atlas_invalidated'] = len(old.get('atlas', {}))-counts['atlas_retained']
    counts['forecast_invalidated'] = len(old.get('forecast', {}))-counts['forecast_retained']
    return counts
