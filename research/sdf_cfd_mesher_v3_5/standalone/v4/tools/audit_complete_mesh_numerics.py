"""Existing observer and manufactured-operator protocols bound to V4 outputs."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
from qualify_complete_mesh_numerics import *
import math
import types

REMAINING = FAMILY / 'sf2_continuation_20260911/remaining_family'
MANUFACTURED = REMAINING / 'prospective_manufactured_controls_20260911'
ANALYSIS = REMAINING / 'frozen_analysis'


def extract_functions(path, names, namespace):
    tree = ast.parse(Path(path).read_text())
    tree.body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert {n.name for n in tree.body} == set(names)
    exec(compile(tree, str(path), 'exec'), namespace)


def operator_gates(out):
    # Frozen operator_stage.py finite-array and integrated conservation gates.
    qualified = read(MANUFACTURED / 'QUALIFIED_AUTHORITY.json')
    expected = next(v for k, v in qualified.items() if k.endswith('_finite_arrays'))
    assert {p.name for p in Path(out).glob('*.f64')} == set(expected)
    arrays = {}
    for p in sorted(Path(out).glob('*.f64')):
        values = np.memmap(p, '<f8', mode='r')
        finite = all(np.isfinite(values[i:i + 1048576]).all() for i in range(0, len(values), 1048576))
        assert finite, p.name
        arrays[p.name] = dict(elements=len(values), finite=True, sha256=sha(p))
    ns = dict(Path=Path)
    extract_functions(V3 / 'generic_v3_cohort_qualification/run_generic_numerical_qualification.py', ['tsv'], ns)
    summary = ns['tsv'](Path(out) / 'native_summary.tsv')
    imbalance = abs(summary['linearVectorDiscreteDivergenceIntegral'] - summary['linearVectorBoundaryFlux']) / abs(summary['linearVectorBoundaryFlux'])
    assert imbalance <= 1e-10
    return dict(pass_gate=True, finite_arrays=arrays, conservation_relative=imbalance)


def observe(rep):
    started = time.monotonic()
    result = read(OUTPUT / 'runs' / rep / 'RESULT.json')
    assert result['all_gates_pass']
    dest = OUTPUT / 'observers' / rep
    dest.mkdir(parents=True, exist_ok=True)
    q = V3 / 'flux_consistent_velocity_handoff_qualification_2026-09-09'
    exe = V3 / 'numerical_backend_forensics_2026-09-07/bin/probeAudit'
    source = q / 'observer_tool/affine_observer.py'
    common = q / 'observer_tool/common_region.py'
    assert sha(exe) == '3e3c01f43c8692cb8e0de7e4ac3e8dfeb32e14bbd45d9ebaa34fc5030a6a690a'
    assert sha(source) == 'b4a8f24b0c93444b25fe7536e052d62c7fc3935ca67d8477dbd1a745a33e302f'
    fields = OUTPUT / 'runs' / rep / f"state_{result['stop']['solve']:03d}/native"
    view = OUTPUT / 'observer_mesh'
    (view / '04_complete').mkdir(parents=True, exist_ok=True)
    link = view / '04_complete/foundation_audit'
    if not link.exists():
        link.symlink_to(MESH_AUTHORITY / 'foundation/foundation_audit')
    write(dest / 'OBSERVER_BINDING.json', dict(mesh=verify_sealed_mesh(),
          terminal_fields=hashes(OUTPUT / 'cases' / rep / '0'),
          geometry=hashes(link), source_sha256=sha(source), native_sha256=sha(exe),
          method='Native physical point containment, then unchanged affine face-neighbour observation'))
    text = source.read_text()
    assert text.count("H/'REGION_OBSERVER_CONTRACT.json'") == 2
    text = text.replace("H/'REGION_OBSERVER_CONTRACT.json'", "H/'OBSERVER_BINDING.json'")
    bound = dest / 'bound_affine_observer.py'
    bound.write_text(text)
    results = {}
    for label, filename, digest, count in [
        ('near94', 'PROBE_POINTS.npy', '70dca6d0c822c680b6e825434baae1165da103865aee9ab3f15a81394f09787a', 94),
        ('outer90', 'OUTER_PROBE_POINTS.npy', '6de4f79c399c9454c498bcd59546969a1128ede69dad03d71e4e36b0d62d8444', 90)]:
        probe = FAMILY / filename
        assert sha(probe) == digest
        points = np.load(probe)
        assert points.shape == (count, 3)
        coords = dest / (label + '_coordinates.txt')
        np.savetxt(coords, points, fmt='%.17g')
        stock = dest / (label + '_STOCK.tsv')
        command([exe, '-case', OUTPUT / 'cases' / rep, '-probes', coords, '-output', stock, '-gradPhi'], dest / (label + '_native'))
        module = types.ModuleType('common_region')
        module.__file__ = str(common)
        exec(compile(common.read_text(), str(common), 'exec'), module.__dict__)
        module.H = dest
        module.LEVELS = {rep: view}
        sys.modules['common_region'] = module
        previous = sys.argv
        sys.argv = [str(bound), rep, '--fields-dir', str(fields), '--vectors-dir', str(fields),
                    '--stock-probes', str(stock), '--tag', label, '--probe-file', str(probe), '--expected-hash', digest]
        try:
            exec(compile(text, str(bound), 'exec'), dict(__name__='__main__', __file__=str(bound)))
        finally:
            sys.argv = previous
        gate = read(dest / 'observer' / rep / label / 'OBSERVER.json')
        assert gate['pass_gate'] and not gate['unresolved']
        data = np.genfromtxt(stock, names=True)
        assert np.array_equal(np.column_stack([data[k] for k in ['x', 'y', 'z']]), points)
        results[label] = dict(count=count, pass_gate=True, native_sha256=sha(stock),
                              affine_sha256=sha(dest / 'observer' / rep / label / 'AFFINE_PROBE_VALUES.tsv'),
                              physical_bindings=[{key: float(row[key]) for key in data.dtype.names
                              if key in ['probe', 'x', 'y', 'z', 'cell', 'face_clearance', 'weight_min', 'reconstruction_error']} for row in data])
    write(dest / 'RESULT.json', dict(pass_gate=True, observations=results))
    write(dest / 'PERFORMANCE.json', dict(seconds=time.monotonic() - started,
          scope='Both native point-location probes and both affine observations; no solver execution'))


def manufactured():
    root = OUTPUT / 'manufactured'
    root.mkdir(parents=True, exist_ok=True)
    frozen = read(FAMILY / 'FROZEN_FAMILY_CONVERGENCE_AUTHORITY.json')
    assert sha(MANUFACTURED / 'FINAL_RESULT.json') == frozen['result_sha256']
    assert sha(MANUFACTURED / 'FINAL_AUTHORITATIVE_REPORT.md') == frozen['report_sha256']
    control = read(MANUFACTURED / 'COMMON_CONTROL_BINDING.json')
    assert sha(MANUFACTURED / 'QUALIFIED_AUTHORITY.json') == control['authority_sha256']
    dictionaries = V3 / 'manufactured_field_qualification/case/system'
    for name, digest in control['controls'].items():
        assert sha(dictionaries / name) == digest
    exe = V3 / 'numerical_backend_forensics_2026-09-07/bin/manufacturedFieldOperatorAudit'
    assert sha(exe) == '1ddfe8c57e72410c5befafb350a6094a1496be3a497ea21251e837642677597e'
    write(root / 'CONTROL_AUTHORITY.json', dict(controls=control, source=str(dictionaries),
          current_family_authority=frozen, executable_sha256=sha(exe),
          historical_missing_dictionary_blocker_resolved=True))
    results = {}
    for level in ['SB', 'SF1', 'SF2', 'V4_SF2']:
        case = root / 'cases' / level
        out = root / 'operators' / level
        if level == 'V4_SF2':
            mesh = OUTPUT / 'input/openfoam_case/constant/polyMesh'
            native = MESH_AUTHORITY / 'foundation/foundation_audit'
            expected = verify_sealed_mesh()
        else:
            mesh = FAMILY / level / 'A/04_complete/openfoam_case/constant/polyMesh'
            native = FAMILY / level / 'A/04_complete/foundation_audit'
            expected = read(FAMILY / level / 'A/CANONICAL_MESH_IDENTITY.json')['canonical_members']
            assert hashes(mesh) == expected
        if not case.exists():
            (case / 'constant').mkdir(parents=True)
            (case / 'constant/polyMesh').symlink_to(mesh)
            shutil.copytree(dictionaries, case / 'system')
        out.mkdir(parents=True, exist_ok=True)
        if not (out / 'EXECUTION.json').exists():
            execution = command([exe, '-case', case, '-outputDir', out, '-decompose'], root / (level + '_operator'))
            write(out / 'EXECUTION.json', execution)
        gates = operator_gates(out)
        fields = dict(affine_gradient=np.linalg.norm(np.fromfile(out / 'linear_scalar_grad_error_components.f64').reshape(-1, 3), axis=1),
                      quadratic_laplacian=abs(np.fromfile(out / 'quadratic_selected_cell_error.f64')))
        volumes = np.fromfile(native / 'cell_volumes.f64')
        assert all(len(v) == len(volumes) and np.isfinite(v).all() for v in fields.values())
        ns = dict(np=np, Vv=volumes)
        extract_functions(V3 / 'farfield_normal_support_qualification_2026-09-08/reduce_results.py', ['norms'], ns)
        glob = {k: ns['norms'](v, np.arange(len(volumes))) for k, v in fields.items()}
        regions = {}
        exact = {}
        if level != 'V4_SF2':
            previous = read(ANALYSIS / 'reductions' / level / 'RESULT.json')
            assert glob == previous['global_errors']
            groups = np.load(ANALYSIS / 'geometry' / level / 'NATIVE_REGIONS.npz')
            for name in groups.files:
                regions[name] = {k: ns['norms'](v, groups[name]) for k, v in fields.items()}
                for field, row in regions[name].items():
                    assert all(row[k] == previous['regions'][name]['errors'][field][k] for k in row)
            for p in sorted(out.glob('*.f64')):
                prior = ANALYSIS / 'operators' / level / p.name
                exact[p.name] = prior.exists() and sha(p) == sha(prior)
            assert all(exact.values()), level
        assert hashes(mesh) == expected
        results[level] = dict(global_errors=glob, regions=regions, gates=gates,
                              preserved_operator_array_equality=exact, mesh=expected,
                              execution=read(out / 'EXECUTION.json'), mesh_unchanged=True)
        write(root / (level + '_RESULT.json'), results[level])
        print(level, 'manufactured operator complete; ' + ('preserved norms exact' if level != 'V4_SF2' else 'separate V4 mesh point evaluated'), flush=True)
    # Reevaluate the preserved slope function on freshly measured family errors.
    ns = dict(np=np, math=math)
    extract_functions(REMAINING / 'convergence.py', ['na', 'error_orders'], ns)
    scales = [read(ANALYSIS / 'scales' / l / 'RESULT.json')['core']['per_cell_median_edge_m']['median'] for l in ['SB', 'SF1', 'SF2']]
    orders = {}
    for field, norm in [('affine_gradient', 'L2'), ('quadratic_laplacian', 'L1'), ('quadratic_laplacian', 'L2')]:
        values = [results[l]['global_errors'][field][norm] for l in ['SB', 'SF1', 'SF2']]
        orders[field + '_' + norm] = dict(values=values, **ns['error_orders'](values, scales))
    prior = read(ANALYSIS / 'CONVERGENCE_RESULT.json')
    resolution = read(MANUFACTURED / 'CLASSIFICATION_RESOLUTION.json')
    assert sha(ANALYSIS / 'CONVERGENCE_RESULT.json') == resolution['raw_result_sha256']
    assert resolution['classification'] == frozen['classification']
    write(root / 'RESULT.json', dict(classification='SpatialConvergencePartiallyEstablished',
          family_classification=frozen['classification'], fresh_levels=['SB', 'SF1', 'SF2'],
          controls_authoritative=True, fresh_operator_arrays_exact=True, fresh_norms_exact=True,
          orders=orders, preserved_complete_family_result=prior, sealed_classification_resolution=resolution,
          V4_single_mesh_operator=results['V4_SF2']['global_errors'],
          limitation='The supported family is preserved V3 SB/SF1/SF2. V4 SF2 is a separate measured point, not a substituted refinement level. No V4 spatial-convergence family is claimed.'))


if __name__ == '__main__':
    if sys.argv[1] == 'observe':
        observe(sys.argv[2])
    elif sys.argv[1] == 'manufactured':
        manufactured()
    else:
        raise ValueError(sys.argv[1])
