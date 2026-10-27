"""Isolated retained-prism diagnostics. No restart or acceptance issuer.

Preparation uses the ordinary selected pipeline and stops at its existing
terminal background dispatch. Baselines use selected schema 4 on protected raw
arrays. A saved fixture never supplies a live BackgroundInputs credential.
"""
from pathlib import Path
from contextlib import ExitStack, contextmanager
from collections import defaultdict
import argparse
import copy
import importlib.util
import json
import math
import os
import re
import sys
import time
import traceback
import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
V4 = HERE.parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src'), str(HERE)]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


supervisor = module('fixture_existing_supervisor', HERE/'supervisor.py')
supervisor.HERE = HERE
from immutable_state_access import guard, token
from sdf_cfd_mesher_v4.common.authority import array_hash, identity, frozen
from path_preserving_deduplication import durable_json

TARGET_PATH = HERE/'transition_front_targets.py'
TARGETS = dict(outer_tetrahedra=dict(minimum=.25, population_targets=[
    dict(fraction=.99, minimum=.40), dict(fraction=.90, minimum=.60)]),
    outer_triangles=dict(minimum=.30, population_targets=[
        dict(fraction=.99, minimum=.60), dict(fraction=.90, minimum=.80)]))
MESH_FIELDS = {'points', 'cell_vertices', 'cell_ids', 'source_columns', 'layers', 'outer', 'wall'}
CAP_FIELDS = {'cap_ids', 'cap_triangles', 'terminal_owners', 'centres', 'volumes', 'world_probes'}
REFERENCE_CASE = V4/'results/front_transition_20261016/CASE_PHYSICAL.json'


def plain(value):
    if isinstance(value, np.ndarray): return plain(value.tolist())
    if isinstance(value, np.generic): return plain(value.item())
    if isinstance(value, dict): return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [plain(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return str(value)
    return value


def publish(path, data):
    if Path(path).exists(): raise ValueError('PreserveExistingEvidence', str(path))
    durable_json(path, plain(data))
    return supervisor.sha(path)


class Deadline:
    """One absolute deadline passed unchanged through all numerical stages."""
    def __init__(self, seconds, clock=time.perf_counter):
        if not math.isfinite(seconds) or seconds <= 0: raise ValueError('PositiveDeadlineRequired')
        self.clock = clock
        self.end = clock() + seconds

    def __call__(self):
        if self.clock() >= self.end: raise TimeoutError('FixtureSharedStageDeadline')

    def remaining_seconds(self):
        return max(0., self.end-self.clock())


def _stage_event(name, event, started, error=None):
    """Append one flushed event; stage completion is never mesh acceptance."""
    record = dict(schema='V4FixtureStage/1', pid=os.getpid(), stage=name,
        event=event, utc_ns=time.time_ns(), elapsed_seconds=time.perf_counter()-started,
        acceptance=False)
    if error is not None: record['error_type'] = type(error).__name__
    print(json.dumps(record, sort_keys=True, separators=(',', ':')), flush=True)


@contextmanager
def stage(name, check=None, timings=None):
    started = time.perf_counter()
    _stage_event(name, 'enter', started)
    try:
        if check is not None: check()
        yield
        if check is not None: check()
    except BaseException as error:
        # A logging failure must not replace the original operation exception.
        try: _stage_event(name, 'error', started, error)
        except Exception: pass
        raise
    else:
        if timings is not None: timings[name.rsplit('.', 1)[-1]] = time.perf_counter()-started
        _stage_event(name, 'exit', started)


def checked_items(values, check):
    """Bound Python collection work between deadline checks by 1024 items.

    Individual hashes, serialization and library calls remain non-preemptible;
    surrounding checks reject expiry on return and the hard supervisor remains.
    """
    for index, value in enumerate(values):
        if index % 1024 == 0: check()
        yield value
    check()


def array_record(a):
    return dict(dtype=a.dtype.str, shape=list(a.shape), sha256=array_hash(a))


def direct_input(root, name):
    root = Path(root).resolve()
    p = root/name
    if not p.resolve().is_relative_to(root): raise ValueError('FixturePathEscapesRoot')
    if p.absolute() != p.resolve(): raise ValueError('IndirectFixturePath')
    return p


def read_arrays(path, fields):
    with np.load(path, allow_pickle=False) as z:
        if set(z.files) != fields: raise ValueError('FixtureArrayFieldsChanged')
        result = {name: frozen(z[name], z[name].dtype.str) for name in sorted(fields)}
    for name, a in result.items():
        expected = '<f8' if name in ('points', 'centres', 'volumes', 'world_probes') else '<i8'
        if a.dtype.str != expected or not np.isfinite(a).all():
            raise ValueError('FixtureArrayTypeOrFiniteness', name)
    return result


def rebuild(mesh, cap):
    """Use existing prism geometry, physical faces and canonical incidence keys."""
    from prism_hotpath_v2 import prism_cell, reverse_cycle
    from background_topology_repair import key
    from background_mixed_geometry import batch_geometry
    x = mesh['points']; vertices = mesh['cell_vertices']; n = len(vertices)
    if x.ndim != 2 or x.shape[1] != 3 or vertices.shape != (n, 6) or not n:
        raise ValueError('FixtureMeshShape')
    if any(mesh[k].shape != (n,) for k in ('cell_ids', 'source_columns', 'layers')):
        raise ValueError('FixtureCellIdentityShape')
    if np.any(vertices < 0) or np.any(vertices >= len(x)) or len(set(mesh['cell_ids'])) != n:
        raise ValueError('FixtureCellIdentity')
    cells = [prism_cell(v, int(c), int(l)) for v, c, l in
             zip(vertices, mesh['source_columns'], mesh['layers'])]
    if [c['id'] for c in cells] != mesh['cell_ids'].tolist(): raise ValueError('FixtureCellIdsChanged')
    owners = defaultdict(list)
    for c in cells:
        for f in c['faces']: owners[key(f)].append((c['id'], f))
    ids = cap['cap_ids']; faces = cap['cap_triangles']; terminal = cap['terminal_owners']
    if (ids.ndim != 1 or not len(ids) or len(set(ids)) != len(ids) or
            np.any(ids < 0) or np.any(ids >= len(x)) or faces.ndim != 2 or faces.shape[1] != 3 or
            not len(faces) or np.any(faces < 0) or np.any(faces >= len(ids)) or
            terminal.shape != (len(faces),) or cap['centres'].shape != (len(faces), 3) or
            cap['volumes'].shape != (len(faces),) or cap['world_probes'].shape != (27, 3)):
        raise ValueError('FixtureCapShape')
    byid = {c['id']: c for c in cells}
    if mesh['outer'].shape != faces.shape or {key(f) for f in mesh['outer']} != {key(f) for f in ids[faces]}:
        raise ValueError('FixtureOuterCapMismatch')
    for f, owner in zip(ids[faces], terminal):
        uses = owners[key(f)]
        if (len(uses) != 1 or uses[0][0] != int(owner) or
                byid[int(owner)]['layer'] != int(mesh['layers'].max()) or
                not reverse_cycle(tuple(f[::-1]), tuple(uses[0][1]))):
            raise ValueError('FixtureTerminalIncidenceChanged')
    geometry = batch_geometry(x, [byid[i] for i in dict.fromkeys(map(int, terminal))])
    centres = np.asarray([geometry[int(i)]['centre'] for i in terminal], dtype='<f8')
    volumes = np.asarray([geometry[int(i)]['volume'] for i in terminal], dtype='<f8')
    # These are captured using this same geometry owner; no tolerance-based identity.
    if centres.tobytes() != cap['centres'].tobytes() or volumes.tobytes() != cap['volumes'].tobytes():
        raise ValueError('FixtureActualOwnerGeometryChanged')
    return cells, owners


def stats(values):
    a = np.asarray(values, dtype=float).ravel(); finite = a[np.isfinite(a)]
    return dict(count=len(a), invalid_count=int(len(a)-len(finite)),
                quantiles=[0, .01, .1, .5, .9, .99, 1],
                values=np.quantile(finite, [0, .01, .1, .5, .9, .99, 1]).tolist() if len(finite) else [])


def front_metrics(points, faces):
    """Diagnostic discrete normal turning, not an SDF curvature certificate."""
    d = module('fixture_shape_measurements', TARGET_PATH)
    tri = points[faces]; centres = tri.mean(axis=1)
    normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    twice = np.linalg.norm(normal, axis=1)
    if np.any(twice <= 0): raise ValueError('DegenerateDiagnosticFace')
    normal /= twice[:, None]
    edges = defaultdict(list)
    lengths = np.stack([np.linalg.norm(tri[:, j]-tri[:, i], axis=1) for i, j in ((0, 1), (1, 2), (2, 0))], axis=1)
    for i, f in enumerate(faces):
        for u, v in zip(f, np.roll(f, -1)): edges[tuple(sorted((int(u), int(v))))].append(i)
    turning = []; adjacent = []; examples = []
    equivalent = np.sqrt(2*twice/np.sqrt(3))
    for edge, uses in edges.items():
        if len(uses) != 2: raise ValueError('DiagnosticFrontNotClosedManifold')
        i, j = uses
        signed = float(np.dot(normal[j]-normal[i], centres[j]-centres[i]))
        angle = float(math.acos(np.clip(np.dot(normal[i], normal[j]), -1, 1)))
        turning.append(math.copysign(angle, signed) if signed else 0.)
        adjacent.append(max(equivalent[i]/equivalent[j], equivalent[j]/equivalent[i]))
        if signed < 0 and len(examples) < 32: examples.append(dict(edge=edge, faces=uses, midpoint=points[list(edge)].mean(axis=0)))
    return plain(dict(edge_length_m=stats(lengths), equivalent_face_size_m=stats(equivalent),
        triangle_mean_ratio=stats(d.triangle_ratio(points, faces)), signed_normal_turning_rad=stats(turning),
        negative_turning_edges=sum(v < 0 for v in turning), negative_turning_examples=examples,
        adjacent_equivalent_size_ratio=stats(adjacent), bounds_m=[tri.reshape(-1, 3).min(axis=0), tri.reshape(-1, 3).max(axis=0)],
        challenge_status='RequiresReview',
        limitations=['Discrete turning is diagnostic; roundoff signs are not resolved concavity proof.',
            'Turning follows supplied winding; wall and outer-cap normals use different conventions.',
            'Section plots, connected waist width/depth and source differential curvature are not implemented.',
            'No automatic blend challenge acceptance. Review retained wall/cap arrays before a challenge claim.']))


def inlet_metrics(mesh, cap, cells):
    x = mesh['points']; tri = x[cap['cap_ids'][cap['cap_triangles']]]
    normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]); twice = np.linalg.norm(normal, axis=1)
    if np.any(twice <= 0): raise ValueError('DegeneratePrismInlet')
    normal /= twice[:, None]
    distance = np.einsum('ij,ij->i', normal, tri.mean(axis=1)-cap['centres'])
    equivalent = np.sqrt(2*twice/np.sqrt(3))
    byid = {c['id']: c for c in cells}; first = {c['column']: c for c in cells if c['layer'] == 1}
    step = []; total = []
    for owner in cap['terminal_owners']:
        c = byid[int(owner)]; p = x[c['vertices']]
        step.append(np.linalg.norm(p[3:]-p[:3], axis=1))
        initial = first.get(c['column'])
        if initial is None: raise ValueError('MissingOriginalPrismColumn')
        total.append(np.linalg.norm(p[3:]-x[initial['vertices'][:3]], axis=1))
    return dict(actual_terminal_centroid_normal_distance_m=stats(distance),
        actual_terminal_vertex_displacement_m=stats(step), total_column_displacement_m=stats(total),
        mean_terminal_displacement_to_face_size=stats(np.mean(step, axis=1)/equivalent),
        volume_over_cap_area_m=stats(2*cap['volumes']/twice),
        scope='Actual cell and column geometry. Path length is not substituted for normal thickness.',
        clearance_status='Existing captured prism acceptance only; no new scalar-distance or clearance certificate.')


def seal_fixture(folder, cap, provenance, evidence):
    """Bind retained data and existing evidence; issue no scientific credential."""
    folder = Path(folder)
    if (folder/'FIXTURE.json').exists() or (folder/'CAP_INPUT.npz').exists(): raise ValueError('ExistingFixture')
    expected_cap = {k: array_record(a) for k, a in cap.items()}
    np.savez(folder/'CAP_INPUT.npz', **cap)
    paths = ['MESH.npz', 'CAP_INPUT.npz', 'CASE.json', *evidence]
    with ExitStack() as held:
        for name in paths: held.enter_context(guard(direct_input(folder, name)))
        mesh = read_arrays(folder/'MESH.npz', MESH_FIELDS)
        captured = read_arrays(folder/'CAP_INPUT.npz', CAP_FIELDS)
        if {k: array_record(a) for k, a in captured.items()} != expected_cap:
            raise ValueError('CapturedCapSerializationChanged')
        rebuild(mesh, captured)
        files = {name: supervisor.sha(direct_input(folder, name)) for name in paths}
        record = dict(schema='V4ResearchPrismFixture/1', files=files,
            arrays=dict(mesh={k: array_record(a) for k, a in mesh.items()}, cap=expected_cap),
            provenance=provenance, restart_supported=False, live_credentials=False,
            scope='Exact retained diagnostic inputs and captured evidence; no recreated live scientific authority.')
        return publish(folder/'FIXTURE.json', record)


@contextmanager
def open_fixture(path, expected_sha256):
    path = Path(path).resolve()
    if not re.fullmatch('[0-9a-f]{64}', expected_sha256): raise ValueError('ExplicitFixtureIdentityRequired')
    with ExitStack() as held:
        held.enter_context(guard(path))
        if supervisor.sha(path) != expected_sha256: raise ValueError('FixtureManifestChanged')
        record = supervisor.read(path)
        if (record.get('schema') != 'V4ResearchPrismFixture/1' or record.get('restart_supported') is not False or
                record.get('live_credentials') is not False): raise ValueError('UnsupportedFixtureScope')
        captured = {}
        for name, sha in record['files'].items():
            p = direct_input(path.parent, name)
            held.enter_context(guard(p))
            if supervisor.sha(p) != sha: raise ValueError('FixtureInputChanged', name)
            captured[p] = (sha, token(p))
        if not {'MESH.npz', 'CAP_INPUT.npz', 'CASE.json'} <= set(record['files']): raise ValueError('IncompleteFixture')
        mesh = read_arrays(path.parent/'MESH.npz', MESH_FIELDS)
        cap = read_arrays(path.parent/'CAP_INPUT.npz', CAP_FIELDS)
        if record['arrays'] != dict(mesh={k: array_record(a) for k, a in mesh.items()}, cap={k: array_record(a) for k, a in cap.items()}):
            raise ValueError('FixtureArrayBindingChanged')
        cells, owners = rebuild(mesh, cap)
        try: yield record, mesh, cap, cells, owners
        finally:
            if any(direct_input(path.parent, name) not in captured for name in record['files']):
                raise ValueError('FixturePathBindingChanged')
            if any(supervisor.sha(p) != h or token(p) != t for p, (h, t) in captured.items()):
                raise ValueError('ProtectedFixtureChanged')
            if supervisor.sha(path) != expected_sha256: raise ValueError('ProtectedFixtureManifestChanged')


def fixture_input_paths(path, expected_sha256):
    """Pin only the small manifest here; worker owns array readback and incidence."""
    path = Path(path).resolve()
    if not re.fullmatch('[0-9a-f]{64}', expected_sha256): raise ValueError('ExplicitFixtureIdentityRequired')
    with guard(path):
        if supervisor.sha(path) != expected_sha256: raise ValueError('FixtureManifestChanged')
        record = supervisor.read(path)
        if (record.get('schema') != 'V4ResearchPrismFixture/1' or record.get('restart_supported') is not False or
                record.get('live_credentials') is not False): raise ValueError('UnsupportedFixtureScope')
        if not {'MESH.npz','CAP_INPUT.npz','CASE.json'} <= set(record['files']): raise ValueError('IncompleteFixture')
        return [path, *[direct_input(path.parent,name) for name in record['files']]]


class PrismCaptureStop(Exception):
    """Deliberate research-stage stop, never successful complete-mesh output."""


@contextmanager
def capture_dispatch(callback, maximum_faces):
    import standalone_background as background
    import standalone_tool as cli
    from production_surface_authority import load_production_wall
    original_background = background.execute; original_surface = cli.surface_stage
    def surface(*args, **kwargs):
        result = original_surface(*args, **kwargs)
        if len(load_production_wall(result[0])[1]) > maximum_faces: raise ValueError('FixtureSurfaceFaceCap')
        return result
    def stop(*args, **kwargs):
        callback(*args, **kwargs)
        raise PrismCaptureStop('AcceptedPrismsCapturedBeforeTransitionDispatch')
    background.execute = stop; cli.surface_stage = surface
    try: yield
    finally: background.execute = original_background; cli.surface_stage = original_surface


def prepare_worker(args, out):
    import standalone_tool as cli
    import rolling_fast_engine as engine
    from standalone_case import load, hold_inputs
    from standalone_background import _interface
    from background_mixed_geometry import batch_geometry
    from tet_layer_planning import kernel
    captured = {}; custody = ExitStack()
    case, schedule, _ = load(args.case)
    if len(schedule) != 6: raise ValueError('SixLayerFixtureRequired')
    def capture(run, case_path, folder, backend, *, _handoff, _preparation, _transition_only):
        if not _transition_only or _handoff is None: raise ValueError('LiveTerminalCaptureRequired')
        _handoff.require_origin(run)
        evidence = [name for name in ('CURRENT.json', 'LAST_STEP.json', 'INITIAL_SUMMARY.json', 'PREFLIGHT.json', 'CASE_IMPORT.json') if (out/name).is_file()]
        evidence += [p.relative_to(out).as_posix() for p in (out/'surface').rglob('SURFACE_AUTHORITY.json')]
        # Hold existing exported files before reading; keep custody through sealing.
        for name in ('MESH.npz', 'CASE.json', *evidence):
            custody.enter_context(guard(direct_input(out, name)))
        exported_case, exported_schedule, _ = load(out/'CASE.json')
        if identity(exported_case) != identity(case) or exported_schedule.tobytes() != schedule.tobytes():
            raise ValueError('CaptureEffectiveCaseChanged')
        evidence_pins = {name: supervisor.sha(direct_input(out, name)) for name in ('MESH.npz', 'CASE.json', *evidence)}
        token_, front, faces, ids, volumes, terminal, metadata, centres = _interface(
            _handoff.view, _handoff=_handoff, _preparation=_preparation)
        if len(faces) > args.maximum_faces: raise ValueError('FixtureTerminalFaceCap')
        # Match the captured supporting geometry to the same selected owner used on readback.
        byid = {c['id']: c for c in _handoff.cells()}
        g = batch_geometry(_handoff.view.points, [byid[i] for i in dict.fromkeys(terminal)])
        cap = dict(cap_ids=ids, cap_triangles=faces, terminal_owners=np.asarray(terminal, dtype='<i8'),
            centres=np.asarray([g[i]['centre'] for i in terminal], dtype='<f8'),
            volumes=np.asarray([g[i]['volume'] for i in terminal], dtype='<f8'),
            world_probes=np.asarray(np.meshgrid(*[np.linspace(front[:, i].min(), front[:, i].max(), 3) for i in range(3)], indexing='ij')).reshape(3, -1).T.copy())
        cap = {k: frozen(a, a.dtype.str) for k, a in cap.items()}
        mesh = read_arrays(out/'MESH.npz', MESH_FIELDS)
        if any(mesh[k].tobytes() != np.asarray(getattr(_handoff.view, k), dtype=mesh[k].dtype).tobytes()
               for k in ('points', 'outer', 'wall')): raise ValueError('CaptureExportMismatch')
        cells, _ = rebuild(mesh, cap)
        if identity(cells) != identity([byid[i] for i in sorted(byid)]): raise ValueError('CaptureCellIncidenceMismatch')
        try:
            _, capability = kernel(front, faces, case['background']['tet_transition'], Deadline(60.))
            capability = dict(passed=True, **capability)
        except Exception as error: capability = dict(passed=False, error=repr(error))
        release = supervisor.verify_release()
        captured.update(cap=cap, evidence=evidence, evidence_pins=evidence_pins,
            mesh_arrays={k: array_record(a) for k, a in mesh.items()},
            provenance=dict(capture_kind='LiveAcceptedPrismHandoff',
            prism_mesh=token_.mesh, layer=run.provider.layer, generation=token_.generation,
            geometry_identity=run.provider.binding.geometry_identity, interface_metadata=metadata,
            inherited_relations=token_.count, inherited_methods=dict(token_.methods),
            surface_assurance='FastProdNumericalSurface/1', surface_formally_qualified=False,
            effective_case_identity=identity(case), schedule_sha256=array_hash(schedule),
            prism_accepted=True, fixture_challenge_accepted=False, kernel=capability,
            wall=front_metrics(mesh['points'], mesh['wall']), cap=front_metrics(front, faces),
            inlet=inlet_metrics(mesh, cap, cells),
            source_inventory_identity=release['source_inventory_identity'],
            release_sha256=supervisor.sha(V4/'runtime/RELEASE.json')))
        _handoff.require_origin(run)
    tokens = [(cli._transition_only, cli._transition_only.set(True)),
        (cli._surface_assurance, cli._surface_assurance.set('numerical')),
        (cli._carrier_assessment, cli._carrier_assessment.set(False))]
    try:
        with hold_inputs(args.case), capture_dispatch(capture, args.maximum_faces):
            try: engine.execute(args.case, out, 6)
            except PrismCaptureStop: pass
            else: raise ValueError('FixtureCaptureBoundaryNotReached')
        if not captured: raise ValueError('NoPrismFixtureCaptured')
        if any(supervisor.sha(direct_input(out, name)) != sha for name, sha in captured['evidence_pins'].items()):
            raise ValueError('LiveCaptureCustodyChanged')
        sha = seal_fixture(out, captured['cap'], captured['provenance'], captured['evidence'])
        sealed = supervisor.read(out/'FIXTURE.json')
        if sealed['arrays']['mesh'] != captured['mesh_arrays']: raise ValueError('LiveCaptureArrayBindingChanged')
    finally:
        for variable, token_ in reversed(tokens): variable.reset(token_)
        custody.close()
    return dict(status='PrismFixtureCaptured', fixture_sha256=sha, prism_accepted=True,
        fixture_challenge_accepted=False, kernel=captured['provenance']['kernel'],
        complete_mesh_success=False, scientific_transition_accepted=False, background_invoked=False)


def run_selected(mesh, cap, cells, owners, controls, bounds, check):
    import tet_layer_adapter as adapter
    from tet_layer_controls import validate, PHYSICAL_SCHEMA
    validate(controls)
    if controls['schema'] != PHYSICAL_SCHEMA: raise ValueError('SelectedSchema4BaselineRequired')
    if controls['seconds'] > 600: raise ValueError('FixtureStageBudgetExceedsScope')
    expected = supervisor.read(HERE/'A.json')['background']['tet_transition']
    if dict(controls, seconds=expected['seconds']) != expected:
        raise ValueError('FixedThreeRowBaselineControlsRequired')
    timings = {}
    with stage('worker.initialization_and_prefix_quality', check, timings):
        original = mesh['points'].tobytes(); check()
        oldcells = identity(cells); check()
        state = adapter.initialize(mesh['points'], cells, owners, cap['cap_ids'], cap['cap_triangles'],
            cap['terminal_owners'].tolist(), copy.deepcopy(controls), bounds,
            prism_volumes=cap['volumes'], prism_centres=cap['centres'], check=check)
    with stage('worker.placement_including_final_quality', check, timings):
        placed = adapter.place(state, check)
    with stage('worker.preservation_and_binding', check, timings):
        if mesh['points'].tobytes() != original: raise ValueError('FixturePrismsModified')
        check()
        if identity(cells) != oldcells: raise ValueError('FixturePrismsModified')
        check()
        if state['points'][:len(mesh['points'])].tobytes() != original: raise ValueError('FixturePrismsModified')
        check()
        original_ids = set(checked_items(mesh['cell_ids'], check))
        retained = [c for c in checked_items(state['cells'], check) if c['id'] in original_ids]
        if identity(retained) != oldcells: raise ValueError('FixturePrismsModified')
        check()
        tets = np.asarray([c['vertices'] for c in checked_items(state['additions'], check)], dtype='<i8')
        check()
        if placed['points_sha256'] != array_hash(state['points']): raise ValueError('DetachedFinalNumericalQuality')
        check()
        if placed['tets_sha256'] != array_hash(tets): raise ValueError('DetachedFinalNumericalQuality')
        check()
        if placed['controls_identity'] != identity(controls): raise ValueError('DetachedFinalNumericalQuality')
        check()
    return state, placed, timings


def diagnostics(state, cap, policy):
    d = module('fixture_shape_measurements', TARGET_PATH)
    from background_mixed_geometry import batch_geometry
    from standalone_background import interface_seeds
    from generic_core_sizing import CoreSizingField
    from background_topology_repair import key
    tets, faces = d.populations(state); points = state['points']
    shape = {}
    for name, values in zip(d.POPULATIONS, (d.tetrahedron_ratio(points, tets), d.triangle_ratio(points, faces))):
        if np.any(~np.isfinite(values)) or np.any(values <= 0):
            shape[name] = dict(passed=False, status='InvalidShapePopulation', values=stats(values))
        else:
            r = d.population_report(values, TARGETS[name])
            for row in r['population_targets']: row['count_shortfall'] = max(0, row['required_count']-row['actual_count'])
            r['below_minimum_count'] = int(np.count_nonzero(values < TARGETS[name]['minimum']))
            shape[name] = r
    byid = {c['id']: c for c in state['additions']}
    ownerids = [state['owners'][key(f)][0][0] for f in faces]
    g = batch_geometry(points, [byid[i] for i in dict.fromkeys(ownerids)])
    volumes = np.asarray([g[i]['volume'] for i in ownerids]); centres = np.asarray([g[i]['centre'] for i in ownerids])
    seeds = interface_seeds(policy, points, faces, volumes, centres)
    field = CoreSizingField(points[faces], seeds, g=policy['growth_per_m'], h_max=policy['maximum_size_m'])
    selected = np.unique(np.linspace(0, len(faces)-1, min(256, len(faces)), dtype=int))
    tri = points[faces[selected]]; c = tri.mean(axis=1)
    h = np.asarray([field.evaluate(p) for p in c])
    equivalent = np.sqrt(2*np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)/np.sqrt(3))
    edge_ratio = [np.linalg.norm(row[1]-row[0])/field.evaluate((row[1]+row[0])/2) for row in tri]
    return plain(dict(outer_shape=shape, outer_front=front_metrics(points, faces),
        all_new_tet_shape=stats(d.tetrahedron_ratio(points, np.asarray([c['vertices'] for c in state['additions']]))),
        sizing=dict(scope='Candidate-derived backend compatibility diagnostics; no independent application resolution or acceptance bands.',
            seeds_m=stats(seeds), sampled_face_ids=selected, sampled_equivalent_size_to_field=stats(equivalent/h),
            sampled_first_edge_to_midpoint_field=stats(edge_ratio), world_probes=cap['world_probes'],
            world_field_m=[field.evaluate(p) for p in cap['world_probes']],
            sampling='At most256 deterministic faces; fixed27 fixture-world probes, possibly outside fluid. Not exhaustive.'),
        supporting_volumes_m3=stats(volumes), supporting_centres_bounds_m=[centres.min(axis=0), centres.max(axis=0)]))


def failure_locations(state, placed):
    byid = {c['id']: c for c in state['cells']}
    cap_vertices = set(map(int, state['cap_ids']))
    outer_vertices = set(map(int, state['outer'].ravel()))
    rows = []
    for record in placed['quality']['failures']:
        ids = record.get('owners', [record.get('id')])
        cells = [byid[i] for i in ids if i in byid]
        vertices = sorted({v for c in cells for v in c['vertices']})
        rows.append(dict(predicate=record['kind'], owner_ids=[c['id'] for c in cells],
            touches_prism=any(c['kind'] == 'prism' for c in cells),
            touches_cap=bool(cap_vertices.intersection(vertices)),
            touches_outer_front=bool(outer_vertices.intersection(vertices)),
            diagnostic_vertex_centroid_m=state['points'][vertices].mean(axis=0) if vertices else None))
    return plain(rows)


def proposal_arrays(state, cap):
    compact = state['compact_ids']
    lookup = np.full(len(state['points']), -1, dtype='<i8'); lookup[compact] = np.arange(len(compact))
    tetrahedra = np.asarray([lookup[c['vertices']] for c in state['additions']], dtype='<i8')
    cap_ids = lookup[cap['cap_ids']]; outer = lookup[state['outer']]
    if any(np.any(a < 0) for a in (tetrahedra, cap_ids, outer)): raise ValueError('ProposalMappingIncomplete')
    return dict(points=state['points'][compact], original_point_ids=compact, tetrahedra=tetrahedra,
        cap_ids=cap_ids, original_cap_ids=cap['cap_ids'], cap_triangles=cap['cap_triangles'], outer_triangles=outer)


def baseline_worker(args, out):
    if Path(out).resolve().is_relative_to(Path(args.fixture).resolve().parent):
        raise ValueError('FixtureOutputOverlap')
    out.mkdir(); timings = {}
    with ExitStack() as held:
        with stage('worker.guarded_readback_and_incidence', timings=timings):
            record, mesh, cap, cells, owners = held.enter_context(open_fixture(args.fixture, args.fixture_sha256))
            if record['provenance'].get('capture_kind') != 'LiveAcceptedPrismHandoff': raise ValueError('CapturedLivePrismProvenanceRequired')
            if not record['provenance'].get('kernel', {}).get('passed'): raise ValueError('FixtureKernelCapabilityUnmet')
            case = supervisor.read(Path(args.fixture).resolve().parent/'CASE.json')
            controls = case['background']['tet_transition']
        check = Deadline(float(controls['seconds']))
        state, placed, stage_times = run_selected(mesh, cap, cells, owners, controls, np.asarray(case['background']['bounds_m']), check)
        timings.update(stage_times)
        with stage('worker.diagnostics', timings=timings):
            report = diagnostics(state, cap, dict(case['background'], tet_transition=controls))
            report['failure_locations'] = failure_locations(state, placed)
        with stage('worker.serialization', timings=timings):
            np.savez(out/'PROPOSAL.npz', **proposal_arrays(state, cap))
            publish(out/'PLACEMENT.json', placed)
            publish(out/'DIAGNOSTICS.json', report)
        return dict(status='RetainedNumericalDiagnostic', numerical_quality_passed=placed['passed'],
            fixture_sha256=args.fixture_sha256, original_prisms_preserved=True,
            controls=controls, controls_identity=identity(controls), timings=timings,
            proposal_sha256=supervisor.sha(out/'PROPOSAL.npz'),
            scientific_transition_accepted=False, background_invoked=False, restart_supported=False,
            challenge_accepted=False, outer_bulk_neighbours_required=True)


def pins(extra=()):
    release = supervisor.verify_release()
    result = {V4/r['path']: r['sha256'] for r in release['sources']}
    paths = [Path(__file__), Path(supervisor.__file__), TARGET_PATH, V4/'runtime/RELEASE.json',
        V4/'results/repair_geometry_cavities_20261013/retry/support.py',
        HERE/'A.json', HERE/'B.json', REFERENCE_CASE, *HERE.glob('test_*.py'),
        *(HERE/'retirement').glob('*.py'), *extra]
    result.update({Path(p).resolve(): supervisor.sha(p) for p in paths})
    return result


def loaded_source_closure(out):
    loaded = {name: str(Path(m.__file__).resolve()) for name, m in list(sys.modules.items())
        if getattr(m, '__file__', None) and Path(m.__file__).suffix == '.py' and V4 in Path(m.__file__).resolve().parents}
    expected = {str(p.resolve()) for p in pins()}
    missing = sorted(set(loaded.values())-expected)
    publish(out/'LOADED_SOURCE_CLOSURE.json', dict(modules=loaded, unpinned_paths=missing,
        scope='V4 Python modules only; external interpreter and installed library versions are not authenticated here.'))
    if missing: raise ValueError('UnpinnedLoadedSource', missing)


def worker(args):
    out = HERE/args.label; started = time.perf_counter()
    if out.exists(): raise ValueError('OutputMustBeNew')
    try:
        if args.action == 'prepare':
            from native_scope import require_transport
            require_transport()
            result = prepare_worker(args, out)
        elif args.action == 'baseline': result = baseline_worker(args, out)
        else: raise ValueError('UnknownFixtureAction')
        code = 0 if result.get('numerical_quality_passed', True) else 1
    except Exception as error:
        out.mkdir(exist_ok=True)
        result = dict(status='Stopped', error=repr(error), traceback=traceback.format_exc(),
            scientific_transition_accepted=False, complete_mesh_success=False, background_invoked=False)
        code = 1
    result.update(seconds=time.perf_counter()-started, restart_supported=False)
    publish(out/'STUDY_RESULT.json', result)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'baseline', 'validate'))
    parser.add_argument('--label', required=True)
    parser.add_argument('--case', type=Path)
    parser.add_argument('--fixture', type=Path)
    parser.add_argument('--fixture-sha256')
    parser.add_argument('--maximum-faces', type=int)
    parser.add_argument('--hard-seconds', type=int, required=True)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker and os.environ.get('V4_FIXTURE_SUPERVISOR_PID') != str(os.getppid()):
        parser.error('Existing guarded supervisor required for worker execution')
    if not re.fullmatch('[A-Za-z0-9_-]+', args.label): parser.error('Plain unique output label required')
    hard_ceiling = 1800 if args.action == 'prepare' else 900
    if args.hard_seconds <= 0 or args.hard_seconds > hard_ceiling: parser.error('Out-of-scope hard bound')
    if args.action == 'prepare' and (args.case is None or args.maximum_faces is None or args.maximum_faces <= 0 or args.maximum_faces > 40000):
        parser.error('Preparation requires explicit case and face cap <=40000')
    if args.action in ('prepare','baseline') and args.hard_seconds <= 70:
        parser.error('Private fixture execution requires time for startup and verified retirement')
    if args.action == 'baseline' and (args.fixture is None or args.fixture_sha256 is None): parser.error('Pinned fixture required')
    if args.action == 'validate':
        if args.hard_seconds > 180: parser.error('Focused checks have a180s bound')
        if args.worker:
            import pytest
            out = HERE/args.label; out.mkdir()
            try:
                return pytest.main(['-q', '-p', 'no:cacheprovider', '--basetemp='+str(out/'tmp'),
                    '--junitxml='+str(out/'TESTS.xml'), str(HERE/'test_fixture_study.py')])
            finally: loaded_source_closure(out)
    elif args.worker: return worker(args)
    extra = [p for p in (args.case,) if p is not None]
    if args.fixture is not None:
        with stage('parent.guarded_manifest'):
            extra += fixture_input_paths(args.fixture,args.fixture_sha256)
    original = list(sys.argv[1:] if argv is None else argv)
    command = [sys.executable, '-B', str(Path(__file__).resolve()), *original, '--worker']
    previous = os.environ.get('V4_FIXTURE_SUPERVISOR_PID')
    os.environ['V4_FIXTURE_SUPERVISOR_PID'] = str(os.getpid())
    try:
        if args.action in ('prepare','baseline'):
            from native_scope import bounded
            from resource_limits import APPROVED
            return bounded(args.label, command, args.hard_seconds, pins(extra), supervisor,resources=APPROVED)['returncode']
        return supervisor.bounded(args.label, command, args.hard_seconds, pins(extra))['returncode']
    finally:
        if previous is None: os.environ.pop('V4_FIXTURE_SUPERVISOR_PID', None)
        else: os.environ['V4_FIXTURE_SUPERVISOR_PID'] = previous


if __name__ == '__main__': raise SystemExit(main())
