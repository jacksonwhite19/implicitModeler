"""Explicit exported fluid-boundary wall cycles; never infer or repair orientation."""
from collections import defaultdict,Counter

SCHEMA = 'StoredFluidBoundaryWall/1'


def cycle_equal(actual, expected):
    return len(actual) == len(expected) and any(
        actual == expected[index:]+expected[:index] for index in range(len(expected)))


def compare(cells, outer, expected_wall, check):
    expected = {}
    for role, faces in (('outer', outer), ('wall', expected_wall)):
        for raw in faces:
            face = tuple(map(int, raw))
            key = tuple(sorted(face))
            if len(face) != 3 or len(set(face)) != 3 or key in expected:
                return dict(passed=False, reason='DuplicateOrInvalidDeclaredBoundary', role=role, face_key=key, expected_cycle=face)
            expected[key] = (role, face)
    owners = defaultdict(list)
    cell_ids = set()
    for index, cell in enumerate(cells):
        if index % 1024 == 0:
            check()
        if cell['id'] in cell_ids:
            return dict(passed=False, reason='DuplicateCellID', cell_id=cell['id'])
        cell_ids.add(cell['id'])
        for raw in cell['faces']:
            face = tuple(map(int, raw))
            owners[tuple(sorted(face))].append((cell['id'], face))
    from prism_hotpath_v2 import reverse_cycle
    boundary_count = 0
    for index, (key, rows) in enumerate(sorted(owners.items())):
        if index % 1024 == 0:
            check()
        wanted = expected.pop(key, None)
        if wanted is not None:
            role, face = wanted
            if len(rows) != 1 or not cycle_equal(rows[0][1], face):
                return dict(passed=False, reason='DeclaredBoundaryOwnerCycleMismatch', role=role,
                    face_key=key, expected_cycle=face, actual_owners=[dict(cell_id=owner, cycle=cycle) for owner, cycle in rows])
            boundary_count += 1
        elif len(rows) != 2 or not reverse_cycle(rows[0][1], rows[1][1]):
            return dict(passed=False, reason='UnexpectedBoundaryOrInvalidInternalOwners', face_key=key,
                actual_owners=[dict(cell_id=owner, cycle=cycle) for owner, cycle in rows])
    if expected:
        key = min(expected)
        role, face = expected[key]
        return dict(passed=False, reason='MissingDeclaredBoundary', role=role, face_key=key, expected_cycle=face)
    check()
    return dict(passed=True, boundary_faces=boundary_count, cells=len(cell_ids), unique_faces=len(owners),
                oriented_cycles_checked=True, scientific_acceptance=False)


def verify(cells, outer, wall, contract, check):
    if contract != SCHEMA or type(contract) is not str:
        raise ValueError('ExplicitStoredFluidBoundaryWallContractRequired')
    from background_topology_repair import boundary
    check()
    manifold_error = None
    try:
        boundary(cells)
    except ValueError as error:
        manifold_error = repr(error)
    check()
    result = compare(cells, outer, wall, check)
    if result['passed'] and manifold_error is not None:
        edges = Counter(tuple(sorted((int(face[index]),int(face[(index+1)%3]))))
            for faces in (outer,wall) for face in faces for index in range(3))
        bad = sorted((edge,count) for edge,count in edges.items() if count != 2)
        result = dict(passed=False,reason='OriginalBoundaryManifoldPredicateFailed',error=manifold_error,
            first_bad_edge=None if not bad else bad[0],empty_boundary=not edges)
    return dict(result, contract=SCHEMA,
                meaning='Stored wall cycles already point outward from the fluid-cell domain; consume unchanged')
