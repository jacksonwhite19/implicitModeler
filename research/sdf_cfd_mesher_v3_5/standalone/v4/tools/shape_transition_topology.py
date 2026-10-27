"""Private proposal attachment structure, replacing copied-product guarantees.

This is not a mesh/proof authority. Protected callers must supply the actual
owner map and old IDs. Existing geometry, quality, source, collision, coverage
and filling admission remain mandatory. No global tetrahedron uniqueness scan.
"""
from collections import deque
from numbers import Integral
from background_materialization import PATTERNS
from background_topology_repair import key, boundary
from prism_hotpath_v2 import reverse_cycle


def _tick(check):
    if check is not None:
        check()


def _integer(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError('InvalidAttachmentIdentity')
    return int(value)


def _face(row):
    values = tuple(_integer(v) for v in row)
    if len(values) != 3 or len(set(values)) != 3:
        raise ValueError('InvalidAttachmentTriangle')
    return values


def _same_cycle(a, b):
    return reverse_cycle(a, tuple(reversed(b)))


def _surface(rows, name, check):
    _tick(check)
    result = {}
    for index, row in enumerate(rows):
        if index and index % 1024 == 0:
            _tick(check)
        face = _face(row)
        k = key(face)
        if k in result:
            raise ValueError('DuplicateAttachment'+name+'Face')
        result[k] = face
    if not result:
        raise ValueError('EmptyAttachment'+name)
    _tick(check)
    return result


def verify_attachment(additions, owners, cap_global_faces, outer_global_faces,
                      old_cell_ids, check=None):
    """Check declared cap/outer incidence and reachability of every new tet.

    The cap faces have the old owners' outward winding. The band must use
    their reverse. Outer faces have the band's outward winding. All owner
    cycles are checked against the added cells. Old owner IDs must be known;
    authenticating which old cell owns a cap face remains the caller's duty.
    Multiple cap/band components and merged band components are allowed.
    """
    _tick(check)
    old = {_integer(i) for i in old_cell_ids}
    cap = _surface(cap_global_faces, 'Cap', check)
    outer = _surface(outer_global_faces, 'Outer', check)
    cap_vertices = {v for face in cap.values() for v in face}
    outer_vertices = {v for face in outer.values() for v in face}
    if cap_vertices & outer_vertices:
        raise ValueError('SharedAttachmentBoundaryVertices')
    cells = []
    byid = {}
    for index, c in enumerate(additions):
        if index % 1024 == 0:
            _tick(check)
        i = _integer(c['id'])
        if i in old or i in byid or c.get('kind') != 'tetrahedron':
            raise ValueError('InvalidAttachmentNewCellPopulation')
        vertices = tuple(_integer(v) for v in c['vertices'])
        if len(vertices) != 4 or len(set(vertices)) != 4:
            raise ValueError('InvalidAttachmentTetrahedron')
        expected = {key(tuple(vertices[j] for j in pattern)):
                    tuple(vertices[j] for j in pattern) for pattern in PATTERNS}
        faces = [_face(f) for f in c['faces']]
        if len(faces) != 4 or len({key(f) for f in faces}) != 4 or any(
                key(f) not in expected or not _same_cycle(f, expected[key(f)])
                for f in faces):
            raise ValueError('ChangedAttachmentCellFaces')
        cell = dict(id=i, kind='tetrahedron', vertices=vertices, faces=faces)
        cells.append(cell)
        byid[i] = cell
    if not cells:
        raise ValueError('EmptyAttachmentBand')
    def checked_cells():
        for index, c in enumerate(cells):
            if index % 1024 == 0:
                _tick(check)
            yield c
    _tick(check)
    chain, band_boundary, unique_faces = boundary(checked_cells())
    _tick(check)
    expected_boundary = dict(outer)
    expected_boundary.update({k:tuple(reversed(f)) for k, f in cap.items()})
    if set(chain) != set(expected_boundary) or any(
            not _same_cycle(f, expected_boundary[key(f)]) for f in band_boundary):
        raise ValueError('ChangedAttachmentOrientedBoundary')
    seeds = set()
    touched = 0
    for face in unique_faces:
        if touched % 1024 == 0:
            _tick(check)
        touched += 1
        k = key(face)
        raw = owners.get(k)
        if raw is None:
            raise ValueError('MissingAttachmentOwnerFace')
        uses = [(_integer(i), _face(f)) for i, f in raw]
        if len({i for i, _ in uses}) != len(uses) or any(key(f) != k for _, f in uses):
            raise ValueError('InvalidAttachmentOwnerRecords')
        new_uses = [(i, f) for i, f in uses if i in byid]
        old_uses = [(i, f) for i, f in uses if i in old]
        if len(new_uses)+len(old_uses) != len(uses):
            raise ValueError('UnknownAttachmentOwner')
        # New owner records must correspond to actual oriented cell faces.
        for i, f in new_uses:
            actual = next((z for z in byid[i]['faces'] if key(z) == k), None)
            if actual is None or not _same_cycle(f, actual):
                raise ValueError('ForgedAttachmentNewOwner')
        if k in cap:
            if (len(uses) != 2 or len(old_uses) != 1 or len(new_uses) != 1 or
                    not _same_cycle(old_uses[0][1], cap[k]) or
                    not reverse_cycle(new_uses[0][1], cap[k])):
                raise ValueError('InvalidAttachmentCapOwners')
            seeds.add(new_uses[0][0])
        elif k in outer:
            if len(uses) != 1 or len(new_uses) != 1 or not _same_cycle(new_uses[0][1], outer[k]):
                raise ValueError('InvalidAttachmentOuterOwner')
        elif (len(uses) != 2 or len(new_uses) != 2 or
              not reverse_cycle(new_uses[0][1], new_uses[1][1])):
            raise ValueError('InvalidAttachmentInternalOwners')
    reached = set(seeds)
    queue = deque(seeds)
    visited = 0
    _tick(check)
    while queue:
        if visited and visited % 1024 == 0:
            _tick(check)
        visited += 1
        i = queue.popleft()
        for face in byid[i]['faces']:
            for j, _ in owners[key(face)]:
                if j in byid and j not in reached:
                    reached.add(j)
                    queue.append(j)
    _tick(check)
    if reached != set(byid):
        raise ValueError('DetachedAttachmentBandComponent')
    _tick(check)
    return dict(schema='ShapeTransitionAttachmentStructure/1', passed=True,
                new_tetrahedra=len(cells), cap_faces=len(cap), outer_faces=len(outer),
                touched_faces=touched, reached_tetrahedra=len(reached),
                actual_old_face_owner_binding_required=True,
                unexpected_untouched_old_changes_checked=False,
                proposal_structure_only=True, scientific_mesh_acceptance=False)
