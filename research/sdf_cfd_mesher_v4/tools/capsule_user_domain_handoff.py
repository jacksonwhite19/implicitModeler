"""Case-specific analytic boundary and read-only prism/core contract.

No mesher is imported or invoked. Coordinates retain their sealed binary64
values. Rational predicates below certify signs for those exact values.
"""
from collections import defaultdict
from contextlib import ExitStack
from fractions import Fraction as F
from pathlib import Path
from unittest.mock import patch
import json
import os
import sqlite3
import sys
import time

import numpy as np

from audit_sealed_l11_core_domain import (
    EXPECTED, SEALED, V4, REPO, read, require_equal as eq, verify_record,
    CommittedState, SURFACE, REQUEST, load_production_wall, read_request,
    NativeCheckpointStore, measured, metrics, denied, durable_json, file_hash,
)
from sdf_cfd_mesher_v4.common.authority import identity, array_hash
from composable_state import Rows

OUT = V4/'results/capsule_user_domain_handoff'
# The complete digest is deliberately pinned independently of generated data.
COORD = 'e7a2493262644cc4bb29c76785aea1900e9eb8d2f09cec793853437287ce15c1'
TOPO = 'd14f00450f2f67f7b98ab3efa1a7946393b7b793ccd64d4bdcdbf516342de2db'
CONTRACT = 'V4CapsulePrismCoreInterface/1'


def seal(record):
    result = dict(record)
    result['identity'] = identity(result)
    return result


def need(condition, label):
    if not condition:
        raise ValueError(label)


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def rational(points):
    return [[F(float(v)) for v in p] for p in points]


def cycle(face):
    face = list(face)
    return min(face[i:]+face[:i] for i in range(len(face)))


def make_domain(center):
    center = list(map(float, center))
    bounds = [[c-d/2, c+d/2] for c, d in zip(center, [1.5, 1.5, 2.5])]
    # Analytic corners and six quads, not a boundary/core mesh discretization.
    corners = [[bounds[0][i], bounds[1][j], bounds[2][k]]
               for i, j, k in [(0,0,0),(1,0,0),(1,1,0),(0,1,0),
                               (0,0,1),(1,0,1),(1,1,1),(0,1,1)]]
    facets = {'x_min':[0,4,7,3], 'x_max':[1,2,6,5],
              'y_min':[0,1,5,4], 'y_max':[3,7,6,2],
              'z_min':[0,3,2,1], 'z_max':[4,5,6,7]}
    return seal(dict(schema='V4UserSpecifiedCapsuleDomain/1',
        authority_status='UserSpecifiedCaseProductionDomain',
        implementation=file_hash(Path(__file__)),
        provenance='Explicit user request: 1.5 m X, 1.5 m Y, 2.5 m Z; capsule-centered; +Z flow.',
        universal_policy=False, farfield_convergence_qualified=False,
        bindings={k:EXPECTED[k] for k in ('identity','scientific','content','geometry','request')},
        fluid_side='positive_sdf', flow_axis=[0,0,1], center=center,
        dimensions=[1.5,1.5,2.5], bounds=bounds, corners=corners,
        patches={k:dict(vertices=v, role=('upstream' if k=='z_min' else
            'downstream' if k=='z_max' else 'radial'), normal='outward_from_box')
            for k, v in facets.items()},
        region='box interior minus closed L11 interior', auxiliary_boundaries=[],
        exclusions=['capsule and sealed prism shell enclosed by L11'],
        additional_cavities=[], solver_boundary_conditions=None,
        discretization='analytic planes and exact rectangular facets; mesh spacing deferred'))


def validate_domain(domain, center):
    verify_record(domain)
    eq(domain, make_domain(center), 'case_domain_contract')
    x = rational(domain['corners'])
    c = list(map(F, center))
    for patch in domain['patches'].values():
        a, b, d, _ = [x[i] for i in patch['vertices']]
        need(dot(cross(sub(b,a), sub(d,a)), sub(a,c)) > 0, 'OuterOrientation')
    return topology(domain['corners'], [p['vertices'] for p in domain['patches'].values()])


def topology(points, polygons):
    x = np.asarray(points, float)
    need(x.ndim == 2 and x.shape[1] == 3 and np.isfinite(x).all(), 'NonfiniteCoordinates')
    edges = defaultdict(list)
    links = defaultdict(list)
    faces = set()
    for f in polygons:
        need(len(f) >= 3 and len(set(f)) == len(f), 'DegenerateFace')
        need(all(isinstance(v, (int, np.integer)) and 0 <= v < len(x) for v in f), 'VertexIndex')
        key = tuple(sorted(f))
        need(key not in faces, 'DuplicateFace')
        faces.add(key)
        a,b,c = rational(x[list(f[:3])])
        need(any(cross(sub(b,a), sub(c,a))), 'ZeroArea')
        for i, a in enumerate(f):
            b = f[(i+1)%len(f)]
            edges[tuple(sorted((a,b)))].append((a,b))
            links[a].append((f[i-1],b))
    need(len(links) == len(x), 'UnusedVertex')
    adjacency = defaultdict(set)
    for (a,b), directions in edges.items():
        need(len(directions) == 2, 'EdgeIncidence')
        need(directions[0] == directions[1][::-1], 'InconsistentOrientation')
        adjacency[a].add(b); adjacency[b].add(a)
    def connected(graph):
        remaining = set(graph); count = 0
        while remaining:
            count += 1; todo = [next(iter(remaining))]
            while todo:
                a = todo.pop()
                if a not in remaining: continue
                remaining.remove(a); todo.extend(graph[a] & remaining)
        return count
    for pairs in links.values():
        link = defaultdict(set)
        for a,b in pairs:
            link[a].add(b); link[b].add(a)
        need(all(len(v)==2 for v in link.values()) and connected(link)==1, 'VertexLink')
    components = connected(adjacency)
    need(components == 1, 'UnexpectedComponent')
    return dict(vertices=len(x), edges=len(edges), faces=len(faces), components=components,
                euler=len(x)-len(edges)+len(faces), closed=True, oriented=True,
                vertex_links='single cycles', duplicate_faces=0, zero_area_faces=0)


def nesting(points, domain):
    x = rational(points)
    bounds = rational(domain['bounds'])
    distances = [(min(p[j]-bounds[j][0], bounds[j][1]-p[j]), i, j)
                 for i,p in enumerate(x) for j in range(3)]
    distance, vertex, axis = min(distances)
    need(distance > 0, 'InvalidNestingOrContact')
    return dict(passed=True, minimum_separation_m=float(distance), exact_separation=str(distance),
                controlling_vertex=vertex, axis=axis, intersections=0, contacts=0,
                proof='Every triangle is a convex combination of strictly interior vertices; nearest box plane distance is affine on each triangle.')


def interface_orientation(points, triangles, lower, center, half_length, radius):
    x = rational(points); low = rational(lower); center = list(map(F, center))
    h, radius = F(half_length), F(radius)
    min_prism = min_gradient = None
    volume6 = F(0)
    for tri in triangles:
        a,b,c = [x[i] for i in tri]
        n = cross(sub(b,a),sub(c,a))
        for i in tri:
            side = -dot(n,sub(low[i],a))
            need(side > 0, 'PrismOwnerSide')
            min_prism = side if min_prism is None else min(side,min_prism)
        midpoint = [sum(v[j] for v in (a,b,c))/3-center[j] for j in range(3)]
        nearest_z = max(-h,min(h,midpoint[2]))
        outward = [midpoint[0],midpoint[1],midpoint[2]-nearest_z]
        gradient = dot(n,outward)
        need(gradient > 0 and dot(outward,outward) > radius*radius, 'SdfCoreSide')
        min_gradient = gradient if min_gradient is None else min(gradient,min_gradient)
        volume6 += dot(a,cross(b,c))
    need(volume6 > 0, 'InnerSignedVolume')
    return dict(faces_checked=len(triangles), lower_vertex_halfspace_checks=3*len(triangles),
        failures=0, minimum_prism_side_exact=str(min_prism), minimum_sdf_dot_exact=str(min_gradient),
        enclosed_volume_m3=float(volume6/6), exact_enclosed_volume=str(volume6/6),
        prism_outward='toward positive SDF and core',
        core_domain_inner_outward='reverse prism-owner triangle winding; toward excluded shell')


def metric_contract():
    return dict(
        prism_local_final=dict(cells=371646, L11_cells=33786,
            scalar_fields=['volume','positive exact Jacobian','aspect ratio'],
            scalar_obligations=1114938, L11_scalar_obligations=101358,
            unchanged_prism_prism_face_records=895329,
            sigma_G='Geometry/face-area and fixed wall-mask diagnostics remain invariant; diagnostics only, not core qualification.'),
        interface_deferred=dict(interface_faces=33786, affected_terminal_cells=33786,
            existing_cell_determinant_records=33786, existing_final_incidence_records=33786,
            existing_records_requiring_actual_neighbour_recertification=67572,
            new_shared_face_records_required=33786,
            new_shared_face_scalar_fields=['interpolation weight','skewness','non-orthogonality','symmetric volume ratio'],
            new_scalar_values_required=135144,
            inputs='Actual prism/core centroids and volumes; exact shared triangle coordinates, centroid and prism-outward area vector.',
            face_metric_definition='src/sdf_cfd_mesher_v4/legacy_full_stack_planner/_frozen/component_quality.py:face_metrics',
            face_metric_source_sha256=file_hash(V4/'src/sdf_cfd_mesher_v4/legacy_full_stack_planner/_frozen/component_quality.py'),
            interpolation_weight='min(abs(S dot (Cf-Co)),abs(S dot (Cn-Cf))) / max(sum of these distances,1e-300)',
            skewness='Existing qualified internal_face_skewness with actual owner/neighbour centroids.',
            non_orthogonality='acos(abs(S dot (Cn-Co))/(norm(S)*norm(Cn-Co))) in degrees; diagnostic, no new threshold.',
            symmetric_volume_ratio='min(Vo,Vn)/max(Vo,Vn)',
            determinant='Replace declared terminal role by exactly one actual core neighbour; recompute incidence-dependent determinant and rebind evidence, never assume terminal equality.'),
        full_core_deferred=dict(population=None,
            reason='Core cell and internal-face counts do not exist until core generation.',
            obligations=['core cell volume/Jacobian/determinant/aspect', 'core internal-face metrics',
                'global volume balance and topology', 'complete collision coverage', 'core sigma/G diagnostics']),
        record_accounting='Cell rows contain both invariant and deferred fields; categories are not disjoint record tables.',
        stored_sigma_G_rows_in_recursive_quality_schema=0)


def verify_sources():
    authority = read(SEALED/'AUTHORITY.json')
    eq(file_hash(SEALED/'AUTHORITY.json'),EXPECTED['authority_sha256'],'authority_sha')
    verify_record(authority)
    for key in ('identity','qualification','scientific','content'):
        eq(authority[key],EXPECTED[key],key)
    eq(authority['reference']['root'],EXPECTED['access'],'access')
    checkpoint = Path(authority['reference']['directory'])/('ROOT_'+EXPECTED['access']+'.json')
    eq(file_hash(checkpoint),EXPECTED['checkpoint'],'checkpoint')
    deep = read(SEALED/'FINAL_DEEP.json')
    eq(file_hash(SEALED/'FINAL_DEEP.json'),authority['files']['FINAL_DEEP.json'],'deep_evidence')
    need(deep['passed'] and all(deep[k]==0 for k in ('forbidden','unresolved','missing','duplicate','hard_failures')), 'PriorQualification')
    wall, tri, surface = load_production_wall(SURFACE)
    bound = read_request(REQUEST,surface)
    for key, value in [('surface',surface['surface_identity']), ('geometry',surface['geometry_identity']),
                       ('request',identity(bound['declaration'])), ('schedule',bound['schedule_identity'])]:
        eq(value,EXPECTED[key],key)
    eq(bound['declaration']['fluid_side'],'positive_sdf','fluid_side')
    geometry = read(SURFACE.parent/'GEOMETRY.json')
    definition = geometry['definition']
    eq(geometry['source_hash'],file_hash(V4/'tools/geometry_family_authority.py'),'geometry_implementation')
    eq(identity(dict(implementation_hash=geometry['source_hash'],definition=definition)),
       EXPECTED['geometry'],'geometry_definition')
    eq(definition['primitive'],'capsule','primitive')
    eq(array_hash(tri),TOPO,'source_topology')
    return authority, deep, wall, tri, definition


def extract(state, wall, source_tri, with_geometry):
    eq(state.layer,11,'layer'); eq(state.content_root,EXPECTED['content'],'content')
    eq(state.root,EXPECTED['access'],'access'); eq(state.expected['front_coordinates'],COORD,'front_binding')
    for field,key in [('geometry_identity','geometry'),('surface_identity','surface'),('request','request'),('schedule','schedule')]:
        eq(state.expected[field],EXPECTED[key],field)
    x = np.array([state['vertices'][f'layer:11:source:{i}'] for i in range(len(wall))])
    eq(array_hash(x),COORD,'coordinates')
    eq(set(state.hot['prism_front']),{str(i) for i in range(len(source_tri))},'source_columns')
    eq(set(state['outer']),set(state.hot['prism_front'].values()),'outer_faces')
    cells = read(state.verify(state.out/'GEOMETRY_DELTA.json'))['cells'] if with_geometry else None
    records = []; triangles = []
    for i, original in enumerate(source_tri):
        face_id = state.hot['prism_front'][str(i)]
        face = state.hot['faces'][face_id]
        need(len(face['owners']) == 1,'AmbiguousPrismOwner')
        owner, ordered = next(iter(face['owners'].items()))
        row = [int(v.split(':')[-1]) for v in ordered]
        eq(set(ordered),{f'layer:11:source:{int(v)}' for v in original},'interface_vertices')
        eq(cycle(row),cycle(original.tolist()),'interface_winding')
        column = state.hot['columns'][str(i)]
        eq(column['exposed_prism_face'],face_id,'source_column_face')
        eq(column['last_prism_layer'],11,'source_column_layer')
        if cells is not None:
            cell = cells[owner]
            eq(cell['source'],str(i),'owner_source'); eq(cell['layer'],11,'owner_layer')
            eq(cell['kind'],'prism','owner_kind')
            need(ordered in cell['faces'],'owner_perimeter')
        records.append(dict(face=face_id, prism_owner=owner, source_column=str(i),
            vertices=ordered, core_neighbour=None, core_neighbour_required=1))
        triangles.append(row)
    need(len({r['face'] for r in records}) == len(records),'DuplicateInterfaceFace')
    lower = np.array([state['vertices'][f'layer:10:source:{i}'] for i in range(len(wall))]) if with_geometry else None
    state.check()
    return x, np.asarray(triangles,dtype=np.int64), records, lower


def guards():
    stack = ExitStack()
    for obj,name in [(sqlite3,'connect'), (os,'link'), (NativeCheckpointStore,'restart'),
                     (Rows,'__getitem__'), (Rows,'items_deep'), (CommittedState,'deep_verify')]:
        stack.enter_context(patch.object(obj,name,denied))
    return stack


def validate_payload(handoff, domain, x, tri, records, expected_x, expected_tri, expected_records):
    verify_record(handoff)
    eq(handoff['domain'],domain['identity'],'domain_binding')
    eq(handoff['bindings'],{k:EXPECTED[k] for k in ('identity','scientific','content','geometry','request','schedule')},'handoff_bindings')
    eq(handoff['contract'],CONTRACT,'contract')
    eq(array_hash(x),array_hash(expected_x),'front_coordinates')
    eq(array_hash(tri),array_hash(expected_tri),'front_topology')
    eq(records,expected_records,'ownership_and_complete_faces')
    eq(handoff['coordinates'],array_hash(x),'coordinates_binding')
    eq(handoff['topology'],array_hash(tri),'topology_binding')
    eq(handoff['ownership'],identity(records),'ownership_binding')
    eq(handoff['metrics'],metric_contract(),'metric_contract')
    eq(handoff['orientation'],dict(stored='prism-owner outward into core',
        core_boundary='reverse each stored triangle',outer='box outward'),'orientation_contract')
    eq(handoff['matching'],'One identical unsplit triangle; same global vertex identities; one prism and one future core owner; opposite owner windings. No Steiner points on interface.','matching')


def build():
    need(not (OUT/'AUTHORITY.json').exists(),'PreserveSealedHandoffAuthority')
    tick = measured(); phases = {}
    authority, deep, wall, source_tri, definition = verify_sources()
    phases['source_verification'] = metrics(*tick)
    center = definition.get('centre',[0,0,0])
    t = measured(); domain = make_domain(center); outer = validate_domain(domain,center)
    phases['domain'] = metrics(*t)
    t = measured()
    with guards(), CommittedState(authority['reference']) as state:
        phases['trusted_open'] = metrics(*t)
        t = measured(); x,tri,records,lower = extract(state,wall,source_tri,True)
        phases['interface_extraction'] = metrics(*t)
        t = measured(); inner = topology(x,tri.tolist())
        eq((inner['vertices'],inner['edges'],inner['faces'],inner['euler']),(16895,50679,33786,2),'front_topology')
        orientation = interface_orientation(x,tri,lower,center,definition['half_length'],definition['radius'])
        phases['topology_orientation'] = metrics(*t)
        t = measured(); separation = nesting(x,domain); phases['cross_boundary'] = metrics(*t)
        state.check(); stats = dict(state.stats)
    t = measured()
    durable_json(OUT/'DOMAIN.json',domain)
    np.save(OUT/'FRONT.npy',x,allow_pickle=False)
    np.save(OUT/'TRIANGLES.npy',tri,allow_pickle=False)
    durable_json(OUT/'INTERFACE.json',records)
    files = {n:file_hash(OUT/n) for n in ('DOMAIN.json','FRONT.npy','TRIANGLES.npy','INTERFACE.json')}
    handoff = seal(dict(schema='V4CapsuleCoreHandoff/1',contract=CONTRACT,
        bindings={k:EXPECTED[k] for k in ('identity','scientific','content','geometry','request','schedule')},
        domain=domain['identity'],coordinates=array_hash(x),topology=array_hash(tri),source_topology=TOPO,
        ownership=identity(records),files=files,inner_tag='prism_core_interface',
        component='sealed_L11',vertex_identity='layer:11:source:<array ordinal>',
        orientation=dict(stored='prism-owner outward into core',core_boundary='reverse each stored triangle',outer='box outward'),
        matching='One identical unsplit triangle; same global vertex identities; one prism and one future core owner; opposite owner windings. No Steiner points on interface.',
        metrics=metric_contract(), reference=authority['reference'],
        deferred_inputs=['outer-boundary discretization','qualified core sizing field','interior point placement',
            'tetrahedralization backend','interface preservation','quality optimization','cross-interface certification'],
        self_intersection_evidence=dict(sha256=authority['files']['FINAL_DEEP.json'],
            proof_population=deep['populations']['proof'],reused=True,
            justification='Exact unchanged interface is a subset of the sealed complete shell collision certificate; no proof payload replay.'),
        core_generation_performed=False))
    validate_payload(handoff,domain,x,tri,records,x,tri,records)
    durable_json(OUT/'HANDOFF.json',handoff)
    access = seal(dict(schema='V4CoreHandoffAccess/1', handoff=handoff['identity'],
        handoff_sha256=file_hash(OUT/'HANDOFF.json'), parent_reference=authority['reference'],
        implementation=file_hash(Path(__file__)),domain=domain['identity'],
        trust='Normal recursive parent epoch/file guards plus bounded handoff content hashes. No stale-epoch bypass.'))
    durable_json(OUT/'ACCESS.json',access)
    phases['serialization'] = metrics(*t)
    result = dict(passed=True,authorities=EXPECTED,center=center,domain=domain['identity'],
        handoff=handoff['identity'],access=access['identity'],outer_topology=outer,inner_topology=inner,
        combined=dict(boundary_components=2,fluid_components=1,euler=4,
            analytic_corners_plus_interface_vertices=16903,edges=50691,triangles=33786,outer_quads=6,
            exact_box_volume='45/8',core_volume_m3=float(F(45,8)-F(orientation['exact_enclosed_volume'])),
            exact_core_volume=str(F(45,8)-F(orientation['exact_enclosed_volume']))),
        orientation=orientation,separation=separation,metrics_contract=metric_contract(),
        self_intersections=handoff['self_intersection_evidence'],phases=phases,stats=stats,
        historical_proof_rows=0,historical_quality_rows=0,sqlite_rows=0,
        full_prefix_hashes=0,deep_copies=0,attested_hardlinks=0,
        generated=dict(extra_prism_layers=0,core_vertices=0,core_interior_points=0,tetrahedra=0,core_cells=0),
        metrics=metrics(*tick),sealed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    durable_json(OUT/'BUILD.json',result)
    print(json.dumps({k:result[k] for k in ('passed','domain','handoff','access','metrics')},indent=2))


def reopen(expected_access):
    tick = measured()
    authority, _, wall, source_tri, definition = verify_sources()
    access = read(OUT/'ACCESS.json'); verify_record(access)
    eq(access['identity'],expected_access,'trusted_access_anchor')
    eq(access['implementation'],file_hash(Path(__file__)),'implementation')
    eq(access['parent_reference'],authority['reference'],'parent')
    eq(file_hash(OUT/'HANDOFF.json'),access['handoff_sha256'],'handoff_file')
    handoff = read(OUT/'HANDOFF.json'); verify_record(handoff)
    eq(handoff['identity'],access['handoff'],'handoff_access')
    for n,h in handoff['files'].items():
        need(n in ('DOMAIN.json','FRONT.npy','TRIANGLES.npy','INTERFACE.json'),'UnexpectedPayloadPath')
        eq(file_hash(OUT/n),h,'payload:'+n)
    domain = read(OUT/'DOMAIN.json'); validate_domain(domain,definition.get('centre',[0,0,0]))
    eq(domain['identity'],access['domain'],'access_domain')
    with guards(), CommittedState(authority['reference']) as state:
        x,tri,records,_ = extract(state,wall,source_tri,False)
        validate_payload(handoff,domain,np.load(OUT/'FRONT.npy',allow_pickle=False),
            np.load(OUT/'TRIANGLES.npy',allow_pickle=False),read(OUT/'INTERFACE.json'),x,tri,records)
        state.check(); stats = dict(state.stats)
    result = dict(passed=True,layer=11,handoff=handoff['identity'],domain=domain['identity'],
        access=access['identity'],stats=stats,metrics=metrics(*tick),historical_proof_rows=0,
        historical_quality_rows=0,sqlite_rows=0,full_prefix_hashes=0,deep_copies=0,
        core_start='Boundary contract ready; generation algorithms and sizing remain unqualified.')
    if not (OUT/'AUTHORITY.json').exists():
        durable_json(OUT/'REOPEN.json',result)
    print(json.dumps(result,indent=2))
    return result


if __name__ == '__main__':
    sys.dont_write_bytecode = True
    if sys.argv[1] == 'build': build()
    elif sys.argv[1] == 'reopen': reopen(sys.argv[2])
    else: raise ValueError('UnknownMode')
