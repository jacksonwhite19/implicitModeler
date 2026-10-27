"""Pure MMG transfer helpers. Caller owns physical custody and bounded execution.

Transfer helpers grant no scientific acceptance. Scoped cancellation retires
only the derived owned executable/input tuple and its exact timeout wrapper.
"""
from collections import defaultdict, Counter
import hashlib
import math
import os
from pathlib import Path
import subprocess
import numpy as np
from owned_runtime import BIN

OUTER_REF = 1000000
NATIVE_DISTRIBUTION = 'Ubuntu-22.04'

_STOP_SCRIPT = r'''
import json,math,os,signal,sys,time
target=sys.argv[1:]
if len(target)!=4:raise ValueError('ExplicitNativeCancellationTupleRequired')
def numeric(argv):
    if len(argv)!=10 or argv[:4]!=target:return False
    try:
        a,b,c,d,mem,noinsert=map(float,argv[4:])
        return all(math.isfinite(v) for v in (a,b,c,d,mem,noinsert)) and a>=1 and b>0 and c>0 and d>=c and mem>=1 and mem.is_integer() and noinsert in (0,1)
    except ValueError:return False
def kind(argv,pid):
    if numeric(argv):return 'native'
    if len(argv)==13 and argv[1]=='--kill-after=5s' and numeric(argv[3:]):
        try:
            duration=argv[2]
            if not duration.endswith('s') or not math.isfinite(float(duration[:-1])) or float(duration[:-1])<=0:return None
            if os.path.realpath('/proc/'+str(pid)+'/exe')==os.path.realpath('/usr/bin/timeout'):return 'timeout'
        except (ValueError,FileNotFoundError,ProcessLookupError):return None
    return None
def read(pid):
    try:
        raw=open('/proc/'+str(pid)+'/cmdline','rb').read()
        argv=[os.fsdecode(v) for v in raw.rstrip(b'\0').split(b'\0')] if raw else []
        role=kind(argv,pid)
        if role is None:return None
        stat=open('/proc/'+str(pid)+'/stat').read().rsplit(')',1)[1].split()
        if stat[0]=='Z':return None
        return (int(pid),int(stat[19]),role)
    except (FileNotFoundError,ProcessLookupError):return None
def scan():
    found=[]
    for name in os.listdir('/proc'):
        if name.isdecimal():
            value=read(int(name))
            if value is not None:found.append(value)
    return sorted(found,key=lambda v:(v[2]!='timeout',v[0]))
initial=scan();sent=[]
for sig in (signal.SIGTERM,signal.SIGKILL):
    # Rescan for a wrapper's newly forked child. The entire Windows launch
    # tree must already be retired by the caller before entering this scan.
    for row in scan():
        if read(row[0])==row:
            try:os.kill(row[0],sig);sent.append(dict(pid=row[0],starttime=row[1],kind=row[2],signal=sig.name))
            except ProcessLookupError:pass
    absent=0
    for _ in range(10):
        time.sleep(.2)
        current=scan()
        absent=absent+1 if not current else 0
        if absent>=2:
            print(json.dumps(dict(passed=True,initial=initial,signals=sent,remaining=[],exact_argv_scope=True,wrapper_and_native_checked=True)))
            sys.exit(0)
remaining=scan()
print(json.dumps(dict(passed=False,initial=initial,signals=sent,remaining=remaining)))
sys.exit(1)
'''


def stop_native(folder):
    """Retire this folder's exact native tuple and pending timeout wrapper.

    No output/receipt can supply a kill target. The common supervisor must
    retire the Windows launch tree first. Launch and scan use the same pinned
    WSL distribution. A failed scan or unconfirmed retirement fails closed.
    """
    if os.name != 'nt': return dict(passed=True, scope='Windows supervisor only', skipped=True)
    folder = Path(folder).resolve()
    paths = [BIN/'shape_transition_mmg', folder/'INPUT.mesh', folder/'METRIC.sol', folder/'OUTPUT.txt']
    native = [subprocess.check_output(['wsl.exe', '-d', NATIVE_DISTRIBUTION, '-e', 'wslpath', '-u', str(p.resolve())],
                                    text=True, encoding='utf-8', timeout=15).strip() for p in paths]
    completed = subprocess.run(['wsl.exe', '-d', NATIVE_DISTRIBUTION, '-e', '/usr/bin/python3', '-c', _STOP_SCRIPT, *native],
                               capture_output=True, text=True, encoding='utf-8', timeout=15)
    if completed.returncode:
        raise ValueError('ShapeTransitionNativeCancellationFailed', completed.returncode, completed.stdout, completed.stderr)
    import json
    try: result = json.loads(completed.stdout)
    except (ValueError, TypeError) as error: raise ValueError('ShapeTransitionNativeCancellationReadback') from error
    if result.get('passed') is not True or result.get('remaining') != []:
        raise ValueError('ShapeTransitionNativeStillRunning', result)
    return result


def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify_pins(pins):
    for path, expected in pins.items():
        if type(expected) is not str or len(expected) != 64 or sha256(path) != expected:
            raise ValueError('NativeInputIdentityChanged', str(path))


def _indices(value, width, count, name, *, boundary_duplicates=False):
    a = np.asarray(value)
    if a.ndim != 2 or a.shape[1] != width or not len(a) or a.dtype.kind not in 'iu':
        raise ValueError(name)
    a = a.astype('<i8', copy=False)
    ordered = np.sort(a, axis=1)
    if np.any(a < 0) or np.any(a >= count) or np.any(np.diff(ordered, axis=1) == 0):
        raise ValueError(name)
    # Full-volume duplicate inventories belong to the existing producer and
    # mandatory ownership/exact admission. Transfer checks only within each tet.
    # Boundary triangles need a unique population for persistent face ancestry.
    if boundary_duplicates and len(np.unique(ordered, axis=0)) != len(a):
        raise ValueError('Duplicate' + name)
    return a


def _vertex_ids(value, count):
    ids = np.asarray(value)
    if ids.ndim != 1:
        raise ValueError('InvalidProtectedVertexIDs')
    if not len(ids): return np.empty(0, dtype='<i8')
    if ids.dtype.kind not in 'iu' or np.any(ids < 0) or np.any(ids >= count) or len(np.unique(ids)) != len(ids):
        raise ValueError('InvalidProtectedVertexIDs')
    return ids.astype('<i8', copy=False)


def write_inputs(directory, points, tets, cap_triangles, outer_triangles, metric, *, protected_tetrahedra=(), protected_vertices=()):
    """Write a proposal with unique inner-face refs and one common outer ref.

    cap_triangles point outward from the prism and are reversed for the band.
    To protect seed rows, caller supplies their explicit tetrahedron IDs. No
    assumption about output row counts, matching vertices or topology is made.
    """
    directory = Path(directory)
    p = np.asarray(points, dtype='<f8')
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('InvalidNativePoints')
    t = _indices(tets, 4, len(p), 'NativeTetrahedra')
    f = _indices(cap_triangles, 3, len(p), 'NativeCapTriangles', boundary_duplicates=True)
    outer = _indices(outer_triangles, 3, len(p), 'NativeOuterTriangles', boundary_duplicates=True)
    if len(f) >= OUTER_REF or set(np.unique(f)) & set(np.unique(outer)):
        raise ValueError('AmbiguousNativeBoundaryPopulation')
    m = np.asarray(metric, dtype='<f8')
    if m.shape != (len(p), 3, 3) or not np.isfinite(m).all() or not np.array_equal(m, m.transpose(0, 2, 1)) or np.any(np.linalg.eigvalsh(m) <= 0):
        raise ValueError('InvalidNativeSPDMetric')
    required = list(protected_tetrahedra)
    if any(type(i) not in (int, np.int64, np.int32) or i < 0 or i >= len(t) for i in required) or len(set(required)) != len(required):
        raise ValueError('InvalidRequiredSeedTetrahedra')
    protected = _vertex_ids(protected_vertices, len(p))
    mesh = directory / 'INPUT.mesh'; sol = directory / 'METRIC.sol'
    if mesh.exists() or sol.exists():
        raise ValueError('NativeInputsAlreadyExist')
    def coords(rows):
        return [' '.join(format(float(v), '.17g') for v in row) for row in rows]
    lines = ['MeshVersionFormatted 2', 'Dimension 3', 'Vertices', str(len(p))]
    vertex_refs = np.zeros(len(p), dtype='<i8'); vertex_refs[protected] = protected + 1
    lines += [row + ' ' + str(int(ref)) for row, ref in zip(coords(p), vertex_refs)]
    lines += ['Tetrahedra', str(len(t))] + [' '.join(str(int(v) + 1) for v in row) + ' 1' for row in t]
    lines += ['Triangles', str(len(f) + len(outer))]
    lines += [' '.join(str(int(v) + 1) for v in row[::-1]) + ' ' + str(j + 1) for j, row in enumerate(f)]
    lines += [' '.join(str(int(v) + 1) for v in row) + ' ' + str(OUTER_REF) for row in outer]
    ids = np.union1d(np.unique(f), protected)
    lines += ['RequiredVertices', str(len(ids))] + [str(int(v) + 1) for v in ids]
    lines += ['RequiredTriangles', str(len(f))] + [str(j + 1) for j in range(len(f))]
    if required:
        lines += ['RequiredTetrahedra', str(len(required))] + [str(int(v) + 1) for v in required]
    with mesh.open('x', encoding='ascii', newline='\n') as stream:
        stream.write('\n'.join(lines + ['End']) + '\n')
    packed = np.stack((m[:, 0, 0], m[:, 0, 1], m[:, 1, 1], m[:, 0, 2], m[:, 1, 2], m[:, 2, 2]), axis=1)
    with sol.open('x', encoding='ascii', newline='\n') as stream:
        stream.write('\n'.join(['MeshVersionFormatted 2', 'Dimension 3', 'SolAtVertices', str(len(p)), '1 3'] + coords(packed) + ['End']) + '\n')
    return {str(mesh.resolve()): sha256(mesh), str(sol.resolve()): sha256(sol)}


def parse_output(path):
    """Parse the existing %.17g native dump without numeric reference coercion."""
    with Path(path).open('r', encoding='ascii') as stream:
        header = stream.readline().split()
        if len(header) != 5 or header[0] != 'COUNTS':
            raise ValueError('NativeOutputHeader')
        try:
            n, nt, nf, ne = map(int, header[1:])
            if min(n, nt, nf) <= 0 or ne < 0:
                raise ValueError('NativeOutputCounts')
            def block(label, count, width, floating=0):
                if stream.readline().strip() != label:
                    raise ValueError('NativeOutputSection', label)
                rows = []
                for _ in range(count):
                    row = stream.readline().split()
                    if len(row) != width:
                        raise ValueError('NativeOutputRecord', label)
                    rows.append([float(v) if j < floating else int(v) for j, v in enumerate(row)])
                return rows
            p = block('VERTICES', n, 6, 3)
            t = np.asarray(block('TETRAHEDRA', nt, 6), dtype='<i8')
            f = np.asarray(block('TRIANGLES', nf, 5), dtype='<i8')
            if stream.read().strip():
                raise ValueError('NativeOutputTrailingData')
        except (OverflowError, TypeError) as exc:
            raise ValueError('MalformedNativeOutput') from exc
    points = np.asarray([row[:3] for row in p], dtype='<f8')
    if not np.isfinite(points).all() or any(row[4] not in (0, 1) or row[5] not in (0, 1) for row in p) or np.any((t[:, 5] != 0) & (t[:, 5] != 1)) or np.any((f[:, 4] != 0) & (f[:, 4] != 1)):
        raise ValueError('InvalidNativeOutputValues')
    tetrahedra = _indices(t[:, :4] - 1, 4, n, 'NativeOutputTetrahedra')
    faces = _indices(f[:, :3] - 1, 3, n, 'NativeOutputTriangles', boundary_duplicates=True)
    return dict(points=points, vertex_refs=np.asarray([row[3] for row in p], dtype='<i8'),
                tets=tetrahedra, required_tetrahedra=t[:, 5].copy(), faces=faces, refs=f[:, 3].copy(), required_faces=f[:, 4].copy(),
                required_vertices=np.asarray([row[5] for row in p], dtype='<i8'))


def restore_protected(points, protected_vertices, protected_tets, output):
    """Recover only explicit protected references and required tetrahedra.

    protected_tets is input connectivity, not cell IDs. Vertex references are
    input index + 1. Required connectivity may be reordered by even vertex
    permutations; reversed orientation, missing or extra required cells fail.
    This checks only the required subset and grants no mesh acceptance.
    """
    source = np.asarray(points, dtype='<f8')
    if source.ndim != 2 or source.shape[1] != 3 or not len(source) or not np.isfinite(source).all():
        raise ValueError('InvalidProtectedSourcePoints')
    ids = _vertex_ids(protected_vertices, len(source))
    expected = np.asarray(protected_tets)
    if expected.shape == (0, 4) and expected.dtype.kind in 'iu':
        expected = expected.astype('<i8', copy=False)
    else: expected = _indices(expected, 4, len(source), 'ProtectedTetrahedra')
    if not set(np.unique(expected)) <= set(ids.tolist()):
        raise ValueError('RequiredTetVerticesNotProtected')
    proposed = np.asarray(output['points'], dtype='<f8')
    refs = np.asarray(output['vertex_refs']); flags = np.asarray(output['required_vertices'])
    if proposed.ndim != 2 or proposed.shape[1] != 3 or not len(proposed) or not np.isfinite(proposed).all():
        raise ValueError('InvalidNativeOutputPoints')
    if refs.shape != (len(proposed),) or refs.dtype.kind not in 'iu' or flags.shape != refs.shape or flags.dtype.kind not in 'iu' or np.any((flags != 0) & (flags != 1)):
        raise ValueError('InvalidProtectedVertexReferences')
    mapping = {}
    # Sort only the explicitly referenced subset. No repeated full-population
    # search per protected vertex and no whole-tetrahedron duplicate inventory.
    selected = np.flatnonzero(np.isin(refs, ids + 1))
    order = np.argsort(refs[selected], kind='stable'); selected = selected[order]
    if len(selected) != len(ids) or not np.array_equal(refs[selected], np.sort(ids + 1)):
        raise ValueError('LostOrDuplicateProtectedVertexReferences')
    if np.any(flags[selected] != 1):
        raise ValueError('LostProtectedVertexRequiredFlag')
    mapping = {int(refs[v]) - 1: int(v) for v in selected}
    tets = np.asarray(output['tets']); required = np.asarray(output['required_tetrahedra'])
    if tets.ndim != 2 or tets.shape[1] != 4 or tets.dtype.kind not in 'iu' or required.shape != (len(tets),) or required.dtype.kind not in 'iu' or np.any((required != 0) & (required != 1)):
        raise ValueError('InvalidRequiredNativeTetrahedra')
    actual = tets[required == 1]
    if len(actual) != len(expected):
        raise ValueError('LostOrExtraRequiredTetrahedra')
    inverse = {value: key for key, value in mapping.items()}
    if not set(np.unique(actual)) <= inverse.keys():
        raise ValueError('ChangedRequiredTetVertexPopulation')
    def signed_key(row):
        values = tuple(map(int, row))
        parity = sum(values[i] > values[j] for i in range(4) for j in range(i+1, 4)) % 2
        return tuple(sorted(values)), parity
    expected_keys = Counter(signed_key(row) for row in expected)
    actual_keys = Counter(signed_key([inverse[int(v)] for v in row]) for row in actual)
    if expected_keys != actual_keys:
        raise ValueError('ChangedRequiredTetConnectivityOrOrientation')
    result = {name: value.copy() if isinstance(value, np.ndarray) else value for name, value in output.items()}
    result['points'] = proposed.copy()
    for source_id, target in mapping.items(): result['points'][target] = source[source_id]
    if any(result['points'][target].tobytes() != source[source_id].tobytes() for source_id, target in mapping.items()):
        raise ValueError('InexactProtectedVertexRestoration')
    changed = proposed[selected] != result['points'][selected]
    return result, mapping, dict(protected_vertices_recovered=len(mapping), protected_tetrahedra_verified=len(expected),
                                exact_protected_coordinates=True, orientation_preserved=True,
                                restored_coordinate_components=int(np.sum(changed)),
                                maximum_restoration=float(np.max(np.abs(proposed[selected]-result['points'][selected]))) if len(selected) else 0.,
                                scientific_authority=False)


def restore_cap(cap_points, cap_triangles, output):
    """Restore exact protected coordinates using unique face-incidence ancestry.

    Reject any ambiguous incidence, missing/duplicate/unknown face reference or
    changed inner triangle orientation. Return new arrays; never mutate output.
    Subsequent V4 quality and exact scientific owners must admit restored cells.
    """
    cap = np.asarray(cap_points, dtype='<f8')
    if cap.ndim != 2 or cap.shape[1] != 3 or not len(cap) or not np.isfinite(cap).all():
        raise ValueError('InvalidProtectedCapPoints')
    faces = _indices(cap_triangles, 3, len(cap), 'ProtectedCapTriangles', boundary_duplicates=True)
    if set(np.unique(faces)) != set(range(len(cap))) or len(faces) >= OUTER_REF:
        raise ValueError('InvalidProtectedCapPopulation')
    outpoints = np.asarray(output['points'], dtype='<f8')
    if outpoints.ndim != 2 or outpoints.shape[1] != 3 or not len(outpoints) or not np.isfinite(outpoints).all():
        raise ValueError('InvalidNativeOutputPoints')
    outfaces = _indices(output['faces'], 3, len(outpoints), 'NativeOutputTriangles', boundary_duplicates=True)
    refs = np.asarray(output['refs'])
    if refs.shape != (len(outfaces),) or refs.dtype.kind not in 'iu':
        raise ValueError('InvalidNativeFaceReferences')
    if np.any((refs < 1) | ((refs > len(faces)) & (refs != OUTER_REF))) or not np.any(refs == OUTER_REF):
        raise ValueError('UnknownOrMissingNativeFaceReferences')
    mask = refs != OUTER_REF; innerrefs = refs[mask]
    if len(innerrefs) != len(faces) or set(innerrefs.tolist()) != set(range(1, len(faces) + 1)):
        raise ValueError('LostOrDuplicateProtectedCapReferences')
    before = defaultdict(list); after = defaultdict(list)
    for ref, face in enumerate(faces, 1):
        for v in face: before[int(v)].append(ref)
    for ref, face in zip(innerrefs, outfaces[mask]):
        for v in face: after[int(v)].append(int(ref))
    def index(values):
        result = {tuple(sorted(refs)): v for v, refs in values.items()}
        if len(result) != len(values):
            raise ValueError('AmbiguousProtectedVertexIncidence')
        return result
    a, b = index(before), index(after)
    if a.keys() != b.keys() or len(a) != len(cap):
        raise ValueError('ChangedProtectedVertexIncidence')
    mapping = {a[k]: b[k] for k in a}
    if set(mapping.values()) & set(np.unique(outfaces[~mask])):
        raise ValueError('OverlappingNativeBoundaryPopulation')
    for ref, face in zip(innerrefs, outfaces[mask]):
        expected = tuple(mapping[int(v)] for v in faces[int(ref) - 1][::-1])
        actual = tuple(map(int, face))
        if actual not in (expected, expected[1:] + expected[:1], expected[2:] + expected[:2]):
            raise ValueError('ChangedProtectedCapOrientation')
    result = {name: value.copy() if isinstance(value, np.ndarray) else value for name, value in output.items()}
    result['points'] = outpoints.copy()
    old = result['points'].copy()
    for source, target in mapping.items(): result['points'][target] = cap[source]
    return result, mapping, dict(required_cap_recovered=True, restored_coordinate_components=int(np.sum(old != result['points'])),
                                maximum_restoration=float(np.max(np.abs(old - result['points']))), scientific_authority=False)


def native_command(binary, mesh, metric, output, *, pins, to_native_path, seconds, kill_after_seconds,
                   hgrad, hausd, hmin, hmax, memory_mb, noinsert, distribution):
    """Prepare an explicitly bounded WSL command; caller holds pins and launches.

    Use the existing campaign subprocess/cancellation owner around this command.
    Hash checks here do not substitute for the caller's physical input guards.
    """
    paths = tuple(Path(p).resolve() for p in (binary, mesh, metric))
    actualpins = {str(Path(p).resolve()): value for p, value in pins.items()}
    if any(str(p) not in actualpins for p in paths):
        raise ValueError('MissingNativeDependencyPin')
    verify_pins(actualpins)
    if Path(output).exists(): raise ValueError('NativeOutputAlreadyExists')
    values = (seconds, kill_after_seconds, hgrad, hausd, hmin, hmax)
    if any(type(v) not in (float, int) or not math.isfinite(v) or v <= 0 for v in values) or hgrad < 1 or hmax < hmin:
        raise ValueError('InvalidExplicitNativeControls')
    if type(memory_mb) is not int or memory_mb < 1 or type(noinsert) is not bool or not distribution or not callable(to_native_path):
        raise ValueError('InvalidExplicitNativeControls')
    return ['wsl.exe', '-d', distribution, '--', 'timeout', '--kill-after=' + str(kill_after_seconds) + 's', str(seconds) + 's',
            *[to_native_path(p) for p in (*paths, Path(output).resolve())],
            *map(str, (hgrad, hausd, hmin, hmax, memory_mb, int(noinsert)))]
