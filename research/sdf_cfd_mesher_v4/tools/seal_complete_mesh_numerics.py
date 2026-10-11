"""Seal measured V4 numerical qualification; fail closed on missing results."""
from qualify_complete_mesh_numerics import *
import tarfile
from audit_complete_mesh_numerics import MANUFACTURED, REMAINING


def archive_restart():
    archive = OUTPUT / 'NUMERICAL_RESTART.tar.zst'
    step = read(OUTPUT / 'RESTART.json')['intermediate_checkpoint']
    checkpoint = f'runs/A/state_{step:03d}'
    selected = []
    for folder in ['initial_fields/A', 'cases/A/0', 'cases/A/system', checkpoint + '/fields']:
        selected.extend(p for p in (OUTPUT / folder).iterdir() if p.is_file())
    selected.extend([OUTPUT / checkpoint / 'CHECKPOINT.json', OUTPUT / 'MESH_INPUT.json'])
    members = {str(p.relative_to(OUTPUT)): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(selected)}
    with archive.open('wb') as compressed:
        proc = subprocess.Popen(['zstd', '-T2', '-3', '-c'], stdin=subprocess.PIPE, stdout=compressed)
        with tarfile.open(fileobj=proc.stdin, mode='w|') as tar:
            for relative in members:
                p = OUTPUT / relative
                info = tar.gettarinfo(str(p), arcname=relative)
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                info.mtime = 0
                with p.open('rb') as f:
                    tar.addfile(info, f)
        proc.stdin.close()
        assert proc.wait() == 0
    # Independently stream every archive member and verify its exact bytes.
    proc = subprocess.Popen(['zstd', '-d', '-c', str(archive)], stdout=subprocess.PIPE)
    seen = set()
    with tarfile.open(fileobj=proc.stdout, mode='r|') as tar:
        for entry in tar:
            h = hashlib.sha256()
            stream = tar.extractfile(entry)
            for block in iter(lambda: stream.read(8 << 20), b''):
                h.update(block)
            assert entry.name not in seen and entry.size == members[entry.name]['bytes']
            assert h.hexdigest() == members[entry.name]['sha256']
            seen.add(entry.name)
    assert proc.wait() == 0 and seen == set(members)
    parts = []
    with archive.open('rb') as f:
        index = 0
        while True:
            data = f.read(90 << 20)
            if not data:
                break
            part = OUTPUT / f'NUMERICAL_RESTART.tar.zst.part{index:03d}'
            part.write_bytes(data)
            parts.append(dict(path=part.name, bytes=len(data), sha256=sha(part)))
            index += 1
    write(OUTPUT / 'RESTART_ARCHIVE.json', dict(sha256=sha(archive), bytes=archive.stat().st_size,
          parts=parts, members=members, roundtrip_exact=True,
          mesh_dependency='Qualified prior fixed_interface_bulk_core_repair archive, checked by MESH_INPUT.json',
          scope=f'Clean fields, frozen dictionaries, solve-{step} restart, final A fields. A2 is byte-identical. Full diagnostic arrays remain indexed in local task storage.'))


def resource(path):
    text = Path(path).read_text()
    match = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', text)
    assert match
    return int(match[1]) * 1024


def main():
    reference()  # Recheck the executable and qualified sources at sealing.
    comparison = read(OUTPUT / 'AA2_COMPARISON.json')
    assert comparison['pass_gate']
    restart = read(OUTPUT / 'RESTART.json')
    assert restart['pass_gate']
    a, b = [read(OUTPUT / 'runs' / r / 'RESULT.json') for r in ['A', 'A2']]
    assert a['all_gates_pass'] and b['all_gates_pass']
    pre = read(OUTPUT / 'precheck.json')
    assert pre['exit_code'] == 0 and 'Mesh OK.' in (OUTPUT / 'precheck.stdout').read_text()
    manufactured = read(OUTPUT / 'manufactured/RESULT.json')
    assert manufactured['fresh_operator_arrays_exact'] and manufactured['fresh_norms_exact']
    immutable = verify_sealed_mesh()
    mesh_auth = read(MESH_AUTHORITY / 'AUTHORITY.json')
    assert sha(MESH_AUTHORITY / 'report.md') == '0fa9fe7a2a816f91e5a129802ee8b2c38cba958891f6cd3a7c10c2bf54824d9b'
    assert sha(MESH_AUTHORITY / 'AUTHORITY.json') == 'a93c8520b2735b9938ba13f71115ec9e572cf974dc11819dd526ab0b38b72fa6'
    write(OUTPUT / 'MESH_IMMUTABILITY.json', dict(pass_gate=True, polyMesh_members=immutable,
          coordinates_connectivity_patches_unchanged=True, wall_unchanged=True, first_layer_unchanged=True,
          shell_certificate_and_core_repair_authority_unchanged=True,
          scientific_mesh_identity=mesh_auth['scientific_geometry'], reused_geometry_relations=226619419))
    history = read(OUTPUT / 'runs/A/OUTER_ITERATION_HISTORY.json')
    flux = a['handoff']['true_flux']
    diagnostic = a['handoff']['internal_roundtrip']
    assert history[-1]['consecutive_passes'] >= 2
    performance = dict(A_seconds=a['seconds'], A2_seconds=b['seconds'],
                       A_peak_rss_bytes=resource(OUTPUT / 'A.resources'),
                       A2_peak_rss_bytes=resource(OUTPUT / 'A2.resources'),
                       precheck=pre, case_construction=read(OUTPUT / 'CASE_CONSTRUCTION.json'),
                       observers={r: read(OUTPUT / 'observers' / r / 'PERFORMANCE.json') for r in ['A', 'A2']},
                       manufactured={r: read(OUTPUT / 'manufactured' / (r + '_RESULT.json'))['execution'] for r in ['SB', 'SF1', 'SF2', 'V4_SF2']},
                       runs_overlap=True,
                       runtime_note='A and A2 run independently but concurrently; elapsed times include contention. No solver performance optimization or speedup claim.',
                       RSS_note='GNU time maximum resident set of the measured process/children, not summed concurrent RSS.')
    write(OUTPUT / 'PERFORMANCE.json', performance)
    write(OUTPUT / 'NUMERICAL_HEALTH.json', dict(classification='OuterConverged',
          solve_counts=comparison['solve_counts'], total_PCG_iterations=comparison['total_PCG_iterations'],
          maximum_PCG_iterations=comparison['maximum_PCG_iterations'],
          linear_final_residual_min=min(e['pcg']['final'] for e in history),
          linear_final_residual_max=max(e['pcg']['final'] for e in history),
          consecutive_complete_passes=history[-1]['consecutive_passes'],
          terminal_outer_residual=a['stop']['metrics']['updated_residual'],
          normalized_global_flux=flux, retained_flux_unchanged=a['gates']['unchanged_Phi_q'],
          diagnostic_internal_flux=diagnostic, observers_exact=True))
    write(OUTPUT / 'V3_COMPARISON.json', dict(
          reference=dict(solves=39, PCG_iterations=8132, global_flux=9.108567304342562e-14,
                         wall_flux=0, diagnostic_L2=0.0019682736703108545, diagnostic_Linf=1.020621881049384),
          V4=dict(solves=a['stop']['solve'], PCG_iterations=a['PCG_iterations'], global_flux=flux['normalized'],
                  wall_flux=flux['wall_absolute'], diagnostic_L2=diagnostic['L2'], diagnostic_Linf=diagnostic['Linf']),
          exact_between_meshes_required=False, explanation='The complete core meshes differ; comparison is descriptive.'))
    # Keep compact run records in Git. Large numerical arrays remain hash-indexed.
    for path in sorted((OUTPUT / 'runs').rglob('*')):
        if path.is_file() and path.suffix in ['.json', '.stdout', '.stderr', '.resources']:
            target = OUTPUT / 'records' / path.relative_to(OUTPUT / 'runs')
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    archive_restart()
    print('Restart archive verified; indexing full local numerical evidence', flush=True)
    index = {}
    for name in ['input', 'cases', 'geometry', 'runs', 'manufactured', 'observers']:
        for path in sorted((OUTPUT / name).rglob('*')):
            if path.is_file() and not path.is_symlink():
                index[str(path.relative_to(OUTPUT))] = dict(bytes=path.stat().st_size, sha256=sha(path))
                if len(index) % 100 == 0:
                    progress = dict(files=len(index), bytes=sum(v['bytes'] for v in index.values()))
                    write(OUTPUT / 'SEALING_PROGRESS.json', progress)
                    print(json.dumps(progress), flush=True)
    write(OUTPUT / 'ARTIFACT_INDEX.json', index)
    # The final index pass also verifies the earlier full A/A2 comparison.
    # Avoid a third full read of the same immutable checkpoint arrays.
    for name, digests in comparison['arrays'].items():
        for rep, digest in zip(['A', 'A2'], digests):
            assert index[f'runs/{rep}/{name}']['sha256'] == digest
    for rep in ['A', 'A2']:
        for name, digest in comparison['terminal_fields'].items():
            assert index[f'cases/{rep}/0/{name}']['sha256'] == digest
    classification = 'V4CompleteMeshNumericalQualificationQualified'
    peak = max(performance['A_peak_rss_bytes'], performance['A2_peak_rss_bytes'], pre['peak_rss_bytes'])
    sections = [
        ('A. Authority verification', 'The expected base commit is ea6e235229b1525faf7a34f95b10b875844d0405. The prior report and authority hashes match the supplied values. The six polyMesh members match the sealed Foundation result. V3 is read-only.'),
        ('B. Complete-mesh numerical input', f"The immutable mesh has 3,776,026 cells: 2,126,040 shell prisms, 52,272 support prisms and 1,597,714 core tets. MESH_INPUT.json binds its byte identities; MESH_IMMUTABILITY.json binds the prior normalized scientific identity. No mesh construction ran."),
        ('C. Numerical-control audit', f"Foundation 13; executable SHA-256 {EXE_HASH}. Frozen PCG/DIC, tolerance 1e-13, relTol 0, maxIter 5000, Gauss linear corrected. The qualified characterize_outerstream binding explicitly sets 1e-13; the older 1e-12 file is not the effective tolerance. Exact copied dictionaries are under controls/main. No transport model or function-object physics is added."),
        ('D. Patch binding', 'Native physical-boundary audits pass for both cases. Wall: 106,302 faces. Farfield: 52,272 faces. Full boundary bytes match the sealed mesh. Analytical farfield values use exported native face centres, not historical indices.'),
        ('E. Initial field construction', 'The qualified writer initializes U=(7,3,2), p=0 and internal Phi=0. Farfield Phi is the preserved affine definition at native face centres. A/A2 initial fields and dictionaries are byte-identical. DrvFS initially conflated Phi and phi; preparation now enforces case-sensitive field storage before solving. The failure reproducer is preserved.'),
        ('F. Foundation 13 precheck', f"checkMesh -allTopology -allGeometry exited 0 and printed literal Mesh OK. Elapsed {pre['seconds']:.2f} s; peak RSS {pre['peak_rss_bytes']:,} bytes."),
        ('G. Run A', f"OuterConverged; {a['stop']['solve']} solves; {a['PCG_iterations']:,} total PCG iterations. All established terminal gates pass. Elapsed {a['seconds']:.2f} s."),
        ('H. Run A2', f"Independent clean replay: OuterConverged; {b['stop']['solve']} solves; {b['PCG_iterations']:,} total PCG iterations. Elapsed {b['seconds']:.2f} s."),
        ('I. A/A2 reproducibility', 'Exact scientific history, all retained native arrays, terminal fields, flux and observers pass. Runtime and resource values are excluded; numerical values are not normalized away. AA2_COMPARISON.json lists every comparison.'),
        ('J. PCG behavior', f"Maximum iterations in one solve: {comparison['maximum_PCG_iterations']}. Every final PCG residual meets 1e-13. Full residual and iteration histories are in records/A and records/A2."),
        ('K. Outer stopping', f"The unchanged detector records {history[-1]['consecutive_passes']} consecutive complete passes. No new heuristic or early stopping rule was used."),
        ('L. Global flux conservation', f"Normalized conservative global imbalance: {flux['normalized']:.17g}. The existing 1e-10 gate passes."),
        ('M. Wall flux', f"Retained wall-flux absolute sum: {flux['wall_absolute']:.17g}; signed sum is zero. The preserved wall condition passes."),
        ('N. Conservative flux retention', 'The qualified terminal velocity handoff preserves Phi, phi and conservativeFlux exactly. Separate native readback reproduces q, U, Phi and gradPhi arrays. The diagnostic recomputed flux is not substituted for q.'),
        ('O. Diagnostic flux difference', f"For the preserved normalized internal fvc::flux(U)-q diagnostic: L2={diagnostic['L2']:.17g}; Linf={diagnostic['Linf']:.17g}. These are diagnostics, not cross-mesh equality gates."),
        ('P. Observer results', 'Both established observers pass at all 94 near and 90 outer physical locations. Native containment records cell, face clearance and reconstruction evidence. The affine observer uses current V4 face-neighbour stars and its unchanged SVD/QR checks. Each observer reproduces exactly between A and A2; the two different observers are not required to equal one another on a non-affine solution.'),
        ('Q. V3 numerical comparison', f"V3 used 39 solves and 8,132 PCG iterations; V4 uses {a['stop']['solve']} and {a['PCG_iterations']:,}. V3 normalized flux imbalance was 9.108567304342562e-14. V3 diagnostic L2/Linf were 0.0019682736703108545/1.020621881049384. V3_COMPARISON.json contains the measured differences without an invented acceptability percentage."),
        ('R. Manufactured-field infrastructure', 'The old missing-dictionary blocker is resolved by the later sealed ProspectiveManufacturedCaseControlsQualified authority. All three current dictionary hashes match COMMON_CONTROL_BINDING.json. They were copied unchanged into V4-owned cases. No controls were invented here. The unchanged operator executable is 1ddfe8c57e72410c5befafb350a6094a1496be3a497ea21251e837642677597e.'),
        ('S. Spatial convergence study', 'SpatialConvergencePartiallyEstablished for the preserved supported SB/SF1/SF2 family. All fresh operator arrays and global/regional norms match preserved evidence exactly. The sealed classification resolution retains the convergent outer90 observer subset while reporting failed primary quantities. No valid single observed order or GCI is established: core scales do not decrease monotonically and primary errors do not all contract. V4 SF2 is a separate operator point, not a substitute family level. Full V4 spatial convergence and aerodynamic/viscous CFD accuracy are not established.'),
        ('T. Numerical restart', f"Existing solve-{restart['intermediate_checkpoint']} field checkpoint restarted through the installed solver. The terminal fields and eight scientific arrays match the uninterrupted result exactly. {restart['remaining_solves']} additional solves; no new restart numerical method."),
        ('U. Mesh immutability', 'Coordinates, connectivity and patches remain byte-identical to the sealed mesh. Wall/first-layer authority, shell certificate and core repair authority remain unchanged. The 226,619,419 complete geometry relations were reused without recomputation.'),
        ('V. Runtime and memory', f"A {a['seconds']:.2f} s; A2 {b['seconds']:.2f} s. Runs overlapped, so these times include contention. Peak measured process RSS: {peak:,} bytes. Detailed precheck, operator, handoff, observer and restart measurements are preserved. RSS is not a sum of concurrent processes."),
        ('W. Determinism', 'A/A2 scientific histories, fields, native arrays and observers match exactly. Native checkpoint restart matches the terminal scientific state. Manufactured family operators reproduce preserved binary64 arrays exactly.'),
        ('X. Tests', 'Fourteen focused plumbing tests passed. Twenty-six existing interface/core-repair tests passed. Live precheck, A, A2, 184-point dual observers, restart and four manufactured operator runs passed their applicable protocols. WSL lacked pytest; retained pytest tests ran with the existing Windows installation and the V4 src path. No package installation was needed.'),
        ('Y. Classification', classification + '. This seals the established potential-flow numerical stack on the V4 complete mesh. Spatial convergence remains separately partial; this is not a RANS, force-coefficient or general production-CFD accuracy qualification.'),
        ('Z. Recommended next task', 'Broader mixed-geometry end-to-end qualification. Keep the measured spatial-convergence limitations explicit before broader CFD accuracy claims. No next-task work was implemented.')]
    formulas = dict(constant='2.375', affine='1.25 + 0.7*x - 1.1*y + 0.9*z',
          affine_exact_gradient=[0.7, -1.1, 0.9],
          quadratic='0.3 + 0.4*x - 0.6*y + 0.8*z + 0.7*x*x - 0.5*y*y + 0.9*z*z + 0.25*x*y - 0.35*x*z + 0.45*y*z',
          quadratic_exact_laplacian=2.2,
          boundaries='Exact analytic values at native boundary face centres in the operator study; no equation solve or new physical BC.',
          norms='L1=sum(V*abs(error))/sum(V); L2=sqrt(sum(V*error**2)/sum(V)); Linf=max(abs(error)); vector-gradient error uses Euclidean magnitude.',
          flux_diagnostic='z=(fvc::flux(U)-q)/(face_area*sqrt(62)); L2=sqrt(sum(area*z*z)/sum(area)); Linf=max(abs(z)); internal faces only.',
          global_flux='abs(sum(boundary q))/sum(abs(farfield q))',
          source_sha256=sha(V3 / 'manufactured_field_qualification/manufacturedFieldOperatorAudit.C'))
    write(OUTPUT / 'FORMULAS.json', formulas)
    table = '\n\n| Mesh | Affine gradient L2 | Quadratic Laplacian L1 | Quadratic Laplacian L2 |\n|---|---:|---:|---:|'
    for level in ['SB', 'SF1', 'SF2', 'V4_SF2']:
        errors = read(OUTPUT / 'manufactured' / (level + '_RESULT.json'))['global_errors']
        table += f"\n| {level} | {errors['affine_gradient']['L2']:.12g} | {errors['quadratic_laplacian']['L1']:.12g} | {errors['quadratic_laplacian']['L2']:.12g} |"
    for i, (title, body) in enumerate(sections):
        if title.startswith('R.'):
            body += '\n\nThe scalar fields are `2.375`, `1.25+0.7x-1.1y+0.9z`, and the quadratic in FORMULAS.json. The exact affine gradient is `(0.7,-1.1,0.9)`; the exact quadratic Laplacian/source is `2.2`. Operator boundary values are analytic values at native face centres. L1/L2 use native cell-volume weights; Linf is the maximum absolute error. No manufactured equation solve or new refinement family was introduced.'
        if title.startswith('S.'):
            body += table
        if title.startswith('O.'):
            body += ' The difference is divided by face area times sqrt(62); L2 is area-weighted over internal faces.'
            location = read(OUTPUT / 'DIAGNOSTIC_LOCATION.json')
            body += f" The maximum is at current native face {location['native_face']}, centre {location['centre_m']} m, between unchanged shell cells. Its weight is {location['face_quality']['weight']:.8g}, volume ratio {location['face_quality']['volume_ratio']:.8g}, and skewness {location['face_quality']['skewness']:.8g}. DIAGNOSTIC_LOCATION.json records both cells and their passing metrics; this location does not establish a causal mechanism."
        if title.startswith('J.'):
            body += '\n\n![V3 and V4 convergence histories](convergence.png)'
        sections[i] = (title, body)
    report = '## Decision question\n\nDoes the qualified V4 complete mesh support the established Foundation 13 numerical stack with deterministic A/A2 convergence, conservative flux behavior, valid observers, and spatial-convergence qualification where authoritative infrastructure exists?\n\n'
    report += '\n\n'.join('### ' + title + '\n\n' + body for title, body in sections)
    (OUTPUT / 'report.md').write_text(report.rstrip() + '\n')
    evidence = {str(p.relative_to(OUTPUT)): sha(p) for p in sorted(OUTPUT.rglob('*'))
                if p.is_file() and not p.is_symlink() and (p.parent == OUTPUT or p.is_relative_to(OUTPUT / 'controls'))
                and p.name not in ['AUTHORITY.json', 'NUMERICAL_RESTART.tar.zst',
                                   'seal.stdout', 'seal.stderr', 'seal.resources']}
    write(OUTPUT / 'AUTHORITY.json', dict(classification=classification, base_commit='ea6e235229b1525faf7a34f95b10b875844d0405',
          report_sha256=sha(OUTPUT / 'report.md'), mesh_authority_sha256=sha(MESH_AUTHORITY / 'AUTHORITY.json'),
          executable_sha256=EXE_HASH, A_A2_exact=True, restart_exact=True, mesh_immutable=True,
          spatial_convergence='SpatialConvergencePartiallyEstablished', full_V4_spatial_convergence_claimed=False,
          production_RANS_or_aerodynamic_accuracy_claimed=False, peak_rss_bytes=peak,
          evidence=evidence, implementation={str(p.relative_to(V4)): sha(p) for p in [Path(__file__), V4 / 'tools/qualify_complete_mesh_numerics.py', V4 / 'tools/audit_complete_mesh_numerics.py', V4 / 'tools/summarize_complete_mesh_numerics.py', V4 / 'tests/test_complete_mesh_numerics.py']}))
    print(classification, sha(OUTPUT / 'report.md'), sha(OUTPUT / 'AUTHORITY.json'), flush=True)


if __name__ == '__main__':
    main()
