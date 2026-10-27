"""V4 input binding for the sealed Foundation 13 numerical protocol.

Run under WSL with Foundation 13 sourced. V3 is read-only reference code;
all cases, generated bindings, checkpoints and results belong to V4.
No numerical expressions or acceptance predicates are replaced.
"""
from pathlib import Path
import ast
import hashlib
import importlib
import json
import os
import re
import shutil
import subprocess
import sys
import time

sys.dont_write_bytecode = True
import numpy as np

V4 = Path(__file__).resolve().parents[1]
V3 = V4.parent / 'sdf_cfd_mesher_v3'
FAMILY = V3 / 'fresh_supported_refinement_family_qualification_2026-09-09'
REFERENCE = FAMILY / 'source_complete_numerics_v2_20260910'
OUTPUT = V4 / 'results/complete_mesh_numerical_qualification'
MESH_AUTHORITY = V4 / 'results/fixed_interface_bulk_core_repair'
EXE = REFERENCE / 'bin/solverFieldAudit'
EXE_HASH = '4c851057485d74c3d218aa5a996b28cbe1195272bef4d4b7b879b3d2581c162b'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def write(path, value):
    path = Path(path)
    if not path.resolve().is_relative_to(OUTPUT.resolve()):
        raise RuntimeError('OutputOutsideV4Task:' + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')
    os.replace(temp, path)


def hashes(path):
    return {p.name: sha(p) for p in sorted(Path(path).iterdir()) if p.is_file()}


def enable_native_field_names(root):
    """Phi and phi are distinct scientific fields on Foundation.

    DrvFS inherits the Windows directory flag. Set it before any solver writes,
    including on directories created by an interrupted preparation.
    """
    root = Path(root)
    for directory in [root, *sorted(p for p in root.rglob('*') if p.is_dir() and not p.is_symlink())]:
        assert directory.resolve().is_relative_to(root.resolve())
        if str(directory).startswith('/mnt/c/'):
            windows = subprocess.check_output(['wslpath', '-w', str(directory)], text=True).strip()
            subprocess.run(['fsutil.exe', 'file', 'setCaseSensitiveInfo', windows, 'enable'], check=True, stdout=subprocess.DEVNULL)


def command(args, stem):
    stem = Path(stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with Path(str(stem) + '.stdout').open('w') as out, Path(str(stem) + '.stderr').open('w') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(stem) + '.resources', *map(str, args)], stdout=out, stderr=err)
    resources = Path(str(stem) + '.resources').read_text()
    match = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', resources)
    row = dict(arguments=list(map(str, args)), exit_code=result.returncode,
               seconds=time.monotonic() - started,
               peak_rss_bytes=int(match[1]) * 1024 if match else None)
    write(str(stem) + '.json', row)
    if result.returncode:
        raise RuntimeError('NativeCommandFailed:' + str(stem))
    return row


def reference():
    if sha(EXE) != EXE_HASH:
        raise RuntimeError('ExecutableAuthorityMismatch')
    qualification = read(FAMILY / 'SOURCE_COMPLETE_EXECUTION_QUALIFICATION.json')
    assert qualification['pass_gate']
    for path, digest in qualification['source_manifest'].items():
        assert sha(path) == digest, path
    # controls.bind only verifies existing generated files. Never create them in V3.
    for name in ['generated_execution_binding.py', 'generated_protocol_observer.py']:
        assert (REFERENCE / name).is_file(), name
    sys.path.insert(0, str(REFERENCE))
    controls = importlib.import_module('terminal_controls')
    controls.c.sha = sha
    controls.c.write = write
    executor, _ = controls.bind()
    writer = controls.c.physical_binding()
    executor.old.native_boundary = writer.native_boundary
    assert executor.contract['inner']['tolerance'] == 1e-13
    return controls, executor, writer


def prepare():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    enable_native_field_names(OUTPUT)
    for name, expected in [('report.md', '0fa9fe7a2a816f91e5a129802ee8b2c38cba958891f6cd3a7c10c2bf54824d9b'),
                           ('AUTHORITY.json', 'a93c8520b2735b9938ba13f71115ec9e572cf974dc11819dd526ab0b38b72fa6')]:
        assert sha(MESH_AUTHORITY / name) == expected
    controls, executor, writer = reference()
    source = MESH_AUTHORITY / 'foundation/openfoam_case/constant/polyMesh'
    mesh = OUTPUT / 'input/openfoam_case/constant/polyMesh'
    original = hashes(source)
    if not mesh.exists():
        shutil.copytree(source, mesh)
    assert hashes(mesh) == original
    write(OUTPUT / 'MESH_INPUT.json', dict(source=str(source), members=original,
          mesh=controls.c.mesh_identity(mesh), cells=3776026, wall_faces=106302,
          farfield_faces=52272, authority_sha256=sha(MESH_AUTHORITY / 'AUTHORITY.json')))
    for rep in ['A', 'A2']:
        case = OUTPUT / 'cases' / rep
        if not case.exists():
            writer.make_potential_case(OUTPUT / 'input', case)
        executor.verify_case_sensitive(case / '0')
        writer.native_boundary(case, 'pre', OUTPUT / (rep + '_PHYSICAL_PRE.json'))
        for name in ['controlDict', 'fvSchemes', 'fvSolution']:
            assert sha(case / 'system' / name) == sha(controls.B / 'authority/system' / name)
        fields = OUTPUT / 'initial_fields' / rep
        if not fields.exists():
            shutil.copytree(case / '0', fields)
        write(OUTPUT / (rep + '_CASE_INPUT.json'), dict(mesh=controls.c.mesh_identity(mesh),
              initial_fields=hashes(fields), dictionaries=hashes(case / 'system'),
              executable_sha256=sha(EXE), stopping_contract=executor.contract,
              source_manifest=qualification_manifest(), clean_initial=True))
    assert hashes(OUTPUT / 'initial_fields/A') == hashes(OUTPUT / 'initial_fields/A2')
    command(['checkMesh', '-case', OUTPUT / 'cases/A', '-allTopology', '-allGeometry'], OUTPUT / 'precheck')
    assert 'Mesh OK.' in (OUTPUT / 'precheck.stdout').read_text()
    geo = OUTPUT / 'geometry'
    if not (geo / 'GEOMETRY_BINDING.json').exists():
        gc = OUTPUT / 'cases/geometry'
        gc.mkdir(parents=True, exist_ok=True)
        (gc / '0').mkdir(exist_ok=True)
        (gc / 'constant').mkdir(exist_ok=True)
        if not (gc / 'constant/polyMesh').exists():
            (gc / 'constant/polyMesh').symlink_to(mesh)
        if not (gc / 'system').exists():
            shutil.copytree(OUTPUT / 'cases/A/system', gc / 'system')
        geo.mkdir(exist_ok=True)
        command([EXE, '-case', gc, '-outputDir', geo / 'native', '-compatibility', '-initializeAnalytical', 'affine'], OUTPUT / 'geometry_initialize')
        command([EXE, '-case', gc, '-outputDir', geo / 'native', '-compatibility', '-fluxOnly'], OUTPUT / 'geometry_read')
        shutil.copyfile(geo / 'native/volume.f64', geo / 'cell_V.f64')
        shutil.copyfile(geo / 'native/Sf.f64', geo / 'face_Sf.f64')
        inc = np.memmap(geo / 'native/incidence.bin', '<i8', mode='r').reshape(-1, 2)
        inc[:, 0].tofile(geo / 'owner.i64')
        inc[inc[:, 1] >= 0, 1].tofile(geo / 'neighbour.i64')
        assert sha(geo / 'cell_V.f64') == sha(MESH_AUTHORITY / 'foundation/foundation_audit/cell_volumes.f64')
        write(geo / 'GEOMETRY_BINDING.json', dict(mesh=original, volumes_exact=True, files=hashes(geo)))
    print('V4 numerical cases prepared; precheck Mesh OK.; frozen controls exact', flush=True)


def qualification_manifest():
    return read(FAMILY / 'SOURCE_COMPLETE_EXECUTION_QUALIFICATION.json')['source_manifest']


def verify_sealed_mesh():
    authority = read(MESH_AUTHORITY / 'AUTHORITY.json')
    assert sha(MESH_AUTHORITY / 'FOUNDATION.json') == authority['evidence']['FOUNDATION.json']
    expected = read(MESH_AUTHORITY / 'FOUNDATION.json')['polyMesh_files']
    for mesh in [MESH_AUTHORITY / 'foundation/openfoam_case/constant/polyMesh',
                 OUTPUT / 'input/openfoam_case/constant/polyMesh']:
        assert hashes(mesh) == expected, str(mesh)
    return expected


def terminal_binding(controls, writer, rep, started):
    source = V3 / 'inner_pcg_tolerance_qualification_2026-09-09/heldout_replay.py'
    tree = ast.parse(source.read_text())
    tree.body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'terminal']
    assert len(tree.body) == 1
    text = ast.unparse(tree)
    replacements = {
        "H / 'bin/solverFieldAuditHandoff'": 'EXE',
        "H / 'handoffPatch.dict'": "c.Q / 'handoffPatch.dict'",
        "AUDIT / 'bin/solverFieldAudit'": 'EXE',
        "H / 'bin/physicalBoundaryAuditHandoff'": "c.Q / 'bin/physicalBoundaryAuditHandoff'",
    }
    for old, new in replacements.items():
        assert old in text
        text = text.replace(old, new)
    generated = OUTPUT / 'terminal_binding.py'
    generated.write_text(text + '\n')
    out = OUTPUT / 'runs' / rep
    ns = dict(np=np, Path=Path, sha=sha, read=read, write=write, shutil=shutil,
              time=time, re=re, c=controls.c, EXE=EXE, writer=writer,
              case=OUTPUT / 'cases' / rep, out=out,
              source=MESH_AUTHORITY / 'foundation/openfoam_case',
              binding=dict(cell_count=3776026), started=started, name='V4_SF2', rep=rep,
              handoff_reduce=controls.handoff_reducer(),
              call=lambda args, label: command(args, out / label))
    exec(compile(text, str(generated), 'exec'), ns)
    return ns['terminal']


def run(rep):
    assert rep in ['A', 'A2']
    verify_sealed_mesh()
    controls, executor, writer = reference()
    case = OUTPUT / 'cases' / rep
    out = OUTPUT / 'runs' / rep
    assert hashes(case / '0') == hashes(OUTPUT / 'initial_fields' / rep), 'Replay must start clean'
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    stop = executor.run_persistent(case, out, OUTPUT / 'geometry', limit=10800,
                                   physical=True, executable=EXE, audit_executable=EXE)
    if stop['accepted'] and stop['state'] == 'OuterConverged' and not stop.get('resource_stop'):
        terminal_binding(controls, writer, rep, started)(stop)
    else:
        write(out / 'RESULT.json', dict(all_gates_pass=False, stop=stop, terminal_not_run=True))
    assert hashes(OUTPUT / 'input/openfoam_case/constant/polyMesh') == read(OUTPUT / 'MESH_INPUT.json')['members']
    print(json.dumps(read(out / 'RESULT.json')), flush=True)


HISTORY_KEYS = ['solve', 'pcg', 'criteria', 'consecutive_passes', 'floor_evidence_count',
                'state', 'accepted', 'failure', 'metrics', 'array_hashes', 'PCG_tolerance']


def compare_histories(left, right):
    return dict(counts_equal=len(left) == len(right),
                states=[dict(solve=a['solve'], exact={k: a[k] == b[k] for k in HISTORY_KEYS})
                        for a, b in zip(left, right)])


def compare():
    roots = [OUTPUT / 'runs' / r for r in ['A', 'A2']]
    results = [read(p / 'RESULT.json') for p in roots]
    assert all(r['all_gates_pass'] for r in results)
    histories = [read(p / 'OUTER_ITERATION_HISTORY.json') for p in roots]
    history = compare_histories(*histories)
    assert history['counts_equal'] and all(all(s['exact'].values()) for s in history['states'])
    keys = ['all_gates_pass', 'gates', 'PCG_iterations', 'handoff', 'summary',
            'native_U_imbalance', 'interface_jump_ratios', 'maximum_speed',
            'readback_exact', 'fields_before', 'fields_after', 'warnings']
    exact = {k: results[0][k] == results[1][k] for k in keys}
    assert all(exact.values())
    members = [{str(p.relative_to(root)) for p in root.rglob('*')
                if p.suffix in ['.f64', '.i64', '.u64', '.i32', '.u32', '.bin', '.tsv']} for root in roots]
    assert members[0] == members[1]
    arrays = {n: [sha(p / n) for p in roots] for n in sorted(members[0])}
    assert all(a == b for a, b in arrays.values())
    fields = [hashes(OUTPUT / 'cases' / r / '0') for r in ['A', 'A2']]
    assert fields[0] == fields[1]
    obs = [read(OUTPUT / 'observers' / r / 'RESULT.json') for r in ['A', 'A2']]
    assert obs[0] == obs[1] and all(o['pass_gate'] for o in obs)
    write(OUTPUT / 'AA2_COMPARISON.json', dict(pass_gate=True, history=history, result_exact=exact,
          arrays=arrays, terminal_fields=fields[0], observer_exact=True,
          solve_counts=[len(h) for h in histories], total_PCG_iterations=[r['PCG_iterations'] for r in results],
          maximum_PCG_iterations=max(e['pcg']['iterations'] for h in histories for e in h),
          mesh_unchanged=bool(verify_sealed_mesh()), excluded=['wall-clock runtime', 'resource usage', 'case path']))


def restart(step):
    """Use the solver's existing field-checkpoint read path, not a new solver API."""
    controls, executor, writer = reference()
    original = OUTPUT / 'runs/A'
    checkpoint = original / f'state_{step:03d}'
    manifest = read(checkpoint / 'CHECKPOINT.json')
    for name, digest in manifest['fields'].items():
        assert sha(checkpoint / 'fields' / name) == digest
    case = OUTPUT / 'cases/restart'
    (case / 'constant').mkdir(parents=True, exist_ok=False)
    (case / 'constant/polyMesh').symlink_to(OUTPUT / 'input/openfoam_case/constant/polyMesh')
    shutil.copytree(OUTPUT / 'cases/A/system', case / 'system')
    shutil.copytree(checkpoint / 'fields', case / '0')
    shutil.copyfile(OUTPUT / 'initial_fields/A/p', case / '0/p')
    out = OUTPUT / 'runs/restart'
    stop = executor.run_persistent(case, out, OUTPUT / 'geometry', limit=10800,
                                   physical=True, executable=EXE, audit_executable=EXE)
    reference_stop = read(original / 'STOPPING_RESULT.json')
    assert stop['accepted'] and stop['state'] == 'OuterConverged'
    assert stop['array_hashes'] == reference_stop['array_hashes']
    expected = original / f"state_{reference_stop['solve']:03d}/fields"
    exact = {n: sha(case / '0' / n) == sha(expected / n) for n in ['Phi', 'U', 'phi', 'conservativeFlux']}
    assert all(exact.values())
    write(OUTPUT / 'RESTART.json', dict(pass_gate=True, intermediate_checkpoint=step,
          final_arrays_exact=True, terminal_fields_exact=exact, remaining_solves=stop['solve'],
          stopping_state=stop['state'], checkpoint_sha256=sha(checkpoint / 'CHECKPOINT.json'),
          semantics='Existing solver reads saved fields. Outer observer starts clean; terminal scientific fields and all eight arrays must match the uninterrupted solve.'))


def construction_probe():
    controls, executor, writer = reference()
    verify_sealed_mesh()
    case = OUTPUT / 'cases/construction_replay'
    started = time.monotonic()
    writer.make_potential_case(OUTPUT / 'input', case)
    executor.verify_case_sensitive(case / '0')
    writer.native_boundary(case, 'pre', OUTPUT / 'CONSTRUCTION_PHYSICAL_PRE.json')
    elapsed = time.monotonic() - started
    assert hashes(case / '0') == hashes(OUTPUT / 'initial_fields/A')
    assert hashes(case / 'system') == hashes(OUTPUT / 'cases/A/system')
    write(OUTPUT / 'CASE_CONSTRUCTION.json', dict(seconds=elapsed,
          initial_fields_exact=True, controls_exact=True, physical_boundary_pass=True,
          scope='Fresh qualified writer and native pre-audit; immutable mesh backing already imported and verified. No numerical solve.'))


if __name__ == '__main__':
    if sys.argv[1] == 'prepare':
        prepare()
    elif sys.argv[1] == 'run':
        run(sys.argv[2])
    elif sys.argv[1] == 'compare':
        compare()
    elif sys.argv[1] == 'restart':
        restart(int(sys.argv[2]))
    elif sys.argv[1] == 'construction-probe':
        construction_probe()
    else:
        raise ValueError(sys.argv[1])
