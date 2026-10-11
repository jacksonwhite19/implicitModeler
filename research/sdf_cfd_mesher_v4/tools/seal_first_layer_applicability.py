"""Seal first-layer applicability only after all bounded production checks."""
import sys
sys.dont_write_bytecode=True
import json,subprocess
from pathlib import Path
from first_layer_determinant_trace import OUT,g
from sdf_cfd_mesher_v4.common.authority import file_hash


def read(name):return json.loads((OUT/name).read_text())


def run():
    cases=read('CASE_RESULTS.json');numeric=read('CURRENT_NUMERIC_REPLAY.json')
    sf2=read('SF2/BOUNDED_REPLAY.json');box=read('rounded_box_regression_final/RESULT.json')
    invalid=read('KNOWN_INVALID.json');high=read('HIGH_PRECISION.json')
    assert len(cases)==4 and all(r['admitted'] and r['post_commit_metrics_exact'] and r['restart_exact'] and r['other_metric_values_exact'] for r in cases)
    assert all(r['layer']>=2 and not r['determinant_failures'] and not r['weight_failures'] for r in cases)
    assert sf2['passed'] and box['passed'] and invalid['rejected'] and high['passed']
    assert len(numeric)==4 and all(r['all_quality_rows_exact'] and r['exposed_baseline_rows_exact'] for r in numeric)
    assert read('SF2/FIRST_LAYER_METRICS.json')['all_arrays_exact']
    root=OUT.parents[1];repo=root.parents[1]
    current=subprocess.check_output(['git','status','--short'],cwd=repo,text=True).splitlines()
    unrelated=[r for r in current if 'research/sdf_cfd_mesher_v4' not in r]
    assert unrelated==read('UNRELATED_STATUS.json')
    classification='V4FirstLayerDeterminantApplicabilityQualified'
    peak=max(r['peak_rss'] for r in cases)
    g.save(OUT/'RUNTIME_MEMORY.json',dict(cases=cases,numerical_replay=numeric,local_recheck=read('LOCAL_RECHECK.json'),
        maximum_production_peak_rss=peak,numerical_replay_peak_rss=max(r['peak_rss'] for r in numeric),
        comparison_scope='Old runs ended after rejected first-layer correction. New runs certify and checkpoint first and second layers; total runtimes are not equivalent work.'))
    paragraphs=[]
    def section(name,body):paragraphs.append('### '+name+'\n\n'+body+'\n')
    section('A. Authority verification',
        'Parent HEAD: `24ad4b03f7cdee69e17b7cc9674aa2c94eece3c1`. Branch: `feature/sdf-cfd-mesher-v4-advancing-front`. '
        'The prior report SHA-256 is `42801d08530ce434b1ebebae25c28f457c4e86d24cab88f357c3310a0b95b1ed`; the prompt supplied a shortened value. '
        'The prior authority SHA-256 is `b506786aef2164cbf60f486560ac9645877ad8937023808d57130d6d5c9f9226`. '
        f'Prior evidence is preserved. {len(unrelated)} unrelated dirty entries remain unchanged. V3 task modifications: 0. '
        'The installed Foundation 13 determinant function matches the preserved source exactly (`FOUNDATION_SOURCE.json`).')
    section('B. Historical production-scale blocker',
        'The unchanged capsule, bent passage, interacting features and separated regions had 4,912 / 4,944 / 10,944 / 29,184 exposed-cap determinant failures. '
        'All exhausted correction before any commit. Their prospective-cap diagnostics passed. This contradiction was traced through executable face selection, not resolved by selecting the larger value.')
    section('C. Determinant implementation trace',
        '`meshCheck::cellDeterminant` selects only internal/coupled faces. It averages their area magnitudes and forms the sum of outer products of normalized face-area vectors. '
        'In 3D the gate uses the magnitude of the tensor determinant. The V4 mixed implementation uses `np.linalg.det` on this positive-semidefinite tensor. '
        '`local_termination_topology.quality` selected `len(owners[face]) == 2`. `final_shell_incidence.check` also selected declared outer-interface faces. '
        'The SF2 prism evaluator excludes the uncoupled lower wall cap and includes its upper cap. The formula and 0.0011 gate are unchanged.')
    section('D. Metric dependency map',
        '`METRIC_DEPENDENCIES.json` binds source hashes and each dependency. Determinant needs cell coordinates, oriented face connectivity, internal/coupled face roles and the selected-face normalization. '
        'It does not use cell centres, face centres, neighbor coordinates or neighbor volumes. Weight, skewness, volume ratio and non-orthogonality have different dependencies.')
    section('E. Candidate-only state',
        'An isolated prism with all faces treated as uncoupled has no selected faces and Foundation returns zero. This is not sufficient topology for production shell admission. '
        'Each `*_TRACE.json` records this state, exact coordinates, faces, owners, area vectors, centres and tensor contributions.')
    section('F. Shell-local state',
        'The first-layer shell snapshot has lateral neighbors but no upper neighbor. Excluding both caps leaves three nearly coplanar lateral area vectors on refined smooth walls. '
        'The value is correct for an artificial mesh whose upper cap is a physical uncoupled boundary. It is not the intended final shell/core domain.')
    section('G. Prospective-cap state',
        'For a closed first-prism layer with an explicitly declared internal continuation/core interface, adding the top face supplies all determinant inputs exactly. '
        'The scalar became authoritative for that restricted determinant scope. It did not become a complete-neighbor quality receipt. '
        'No neighbor centroid, cell volume, face weight or certificate is invented.')
    section('H. Committed first-layer state',
        'Raw snapshot incidence still produces the exposed-cap diagnostic. The committed production state records the declared future-internal face role explicitly. '
        'After actual layer-2 attachment, every layer-1 cell metric row equals its accepted prospective row exactly. `CASE_RESULTS.json` records this full-population comparison.')
    section('I. Complete-neighbor state',
        'Each reproducer attaches an explicit successor-prism connectivity probe and verifies the actual owner-count mask equals the prospective mask. '
        'The full production next-layer commits provide the independent realized-neighbor check. '
        'Mixed transition candidates still require the existing bounded provisional core closure and complete-neighbor validation. This change does not apply the first-prism shortcut to mixed cells.')
    section('J. High-precision comparison',
        f'{len(high["records"])} reproducers were recomputed at 80 decimal digits, first from binary64 area vectors and then from exact binary64 coordinates. '
        'All gate decisions agree. The large changes arise from face-role selection and normalization, not floating-point noise. '
        'SF2 retains its existing arithmetic path; no cross-implementation last-bit difference is substituted for its authoritative values.')
    section('K. Root cause',
        'The analytic provider merged prospective failures into shell-local failures. It therefore required two different domain states to pass the same determinant gate. '
        'The first changed scientific input is the top-cap selection bit. This changes selected-face count, mean area, every normalized vector and the tensor. '
        'Admission now replaces only the inapplicable exposed-cap determinant for a conformal all-prism first layer. Other failures remain authoritative.')
    section('L. SF2 why-pass analysis',
        'SF2 already uses the prospective cap mask. Its authoritative first-layer minimum remains 0.0021820778946444897 with zero failures. '
        'At the selected weak cell, the shell-local value is 0.21428027046833065. Cap inclusion lowers it to about 0.00218207789465. '
        'Accepted extrusion directions differ by about 82.0857 degrees there. The refined failing samples have approximately 0–3.15 degree source-normal differences. '
        'This is measured state dependence, not a new wall-quality threshold.')
    section('M. Rounded-box why-pass analysis',
        'The preserved tangentially corrected layer has a shell-local minimum of 0.0018146029657583258 and prospective minimum of 0.003975617297568933. '
        'Its selected flat-wall sample has equal source normals; the preserved correction changes the side-area tensor. '
        'The sealed corrected geometry is re-admitted and restarted exactly. This regression does not claim that a newly regenerated nominal box must repeat obsolete exposed-cap correction decisions.')
    section('N. Prospective-cap semantics',
        'Classification A applies to the determinant inputs only: exact reconstruction of final face-role inputs. '
        'The receipt carries an obligation that every top face remains internal in the final domain. A topology/role change invalidates it. '
        'The complete domain must still pass all actual neighbor-pair and continuous-geometry gates.')
    section('O. Receipt semantics',
        '`ProspectiveFirstPrismDeterminant` binds metric implementation/version, geometry identity, topology identity/version, geometry version, face roles and closure identity. '
        'Native cell receipts explicitly mark their scope and bind the first-prism metric authority. '
        'Tests reject stale geometry, topology, version, closure and role receipts. No full geometry array is copied into a determinant receipt.')
    section('P. Provisional first-layer closure',
        'Closure is a bounded incidence contract, not a speculative whole future mesh. It requires prism-only layer 1, exactly one exposed top per cell, uncoupled wall faces and actual lateral neighbors. '
        'The role calculation is linear in the existing first-layer topology. A measured quadratic set-union defect in the new check was repaired without changing any result; its reproducer is preserved.')
    section('Q. Post-commit validation',
        'The provider rebuilds native first-layer entities and verifies receipt identity. The checkpoint writer restores and validates staged serialized state before atomically publishing CURRENT. '
        'The runner also revalidates the published state before returning; failure revokes only the fresh initialization pointer and preserves evidence. '
        'Injected staged and post-publication failures prove fail-closed behavior. Actual layer-2 metrics match accepted first-layer metrics exactly.')
    section('R. Other hard-metric applicability',
        'No other metric definition or applicability changed. All existing first-layer face rows, exact Jacobian rows, cell volumes and aspects equal the prior nominal evaluation exactly in all four cases. '
        'Top-cap pair metrics wait for an actual neighboring cell and are checked when that pair is constructed; the determinant receipt does not claim those metrics.')
    for title,row in zip(('S. Capsule replay','T. Bent-passage replay','U. Interacting-ellipsoid replay','V. Four-region replay'),cases):
        section(title,f"Admitted. Determinant failures: {row['before_determinant_failures']:,} → {row['determinant_failures']}. "
            f"Minimum determinant: {row['determinant_min']!r}. Minimum weight: {row['weight_min']!r}; weight failures: {row['weight_failures']}. "
            f"First-layer corrections: {row['initial_correction_iterations']}. Layer {row['layer']} committed; remesh events: {row['remesh_events']}. "
            'Wall, first-layer geometry, other metrics, realized first-layer tensor and restart remain exact.')
    section('W. Known-invalid rejection',
        f"The unchanged coarse ellipsoid-concavity authority still rejects at h1={invalid['h1']!r}: {invalid['failures']} determinant failures, minimum {invalid['determinant_min']!r}. "
        'A separate reversed-extrusion test also rejects on exact volume/Jacobian signs. Favorable determinant alone cannot bypass other gates.')
    section('X. SF2 regression',
        'Fresh first-layer evaluation reproduces every preserved cell, side-face and cap metric-array hash exactly, including determinant and conditioning. It took 5.209 s. '
        f"Fresh bounded native replay passed in {sf2['seconds']:.3f} s. It verified 2,126,040 prisms, all hard receipts, 56,462,864 preserved relations, zero missing/duplicate/forbidden/unresolved, exact atlas/forecast and restart, and zero remesh. "
        'Wall and first-layer hashes remain `3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94` and `2d265914e38be50b62ac8d2bd82bf19723386d726358c83edcef6c238dbfcb67`. No full SF2 rebuild was needed.')
    section('Y. Rounded-box regression',
        'The preserved corrected first layer retains its exact wall, coordinates, connectivity and historical exposed-shell determinant array. '
        'Current scoped admission, exact certification, staged/native commit and restart pass. See `rounded_box_regression_final/RESULT.json`. '
        'This is immutable-authority replay, not a claim of unchanged fresh nominal search under the corrected semantics.')
    table='| Case | Prior rejected first layer s | New certified initialization s | Through first native commit s | Including layer 2 s | Peak RSS bytes |\n|---|---:|---:|---:|---:|---:|\n'
    for r in cases:table+=f"| {r['case']} | {r['before_rejected_seconds']:.3f} | {r['after_initialization_seconds']:.3f} | {r['after_first_commit_seconds']:.3f} | {r['total_including_next_layer_seconds']:.3f} | {r['peak_rss']:,} |\n"
    section('Z. Runtime and memory',table+'\nOld rejection time and new accepted/certified time represent different work. Cold runs overlapped on this host. '
        'The first three cold attempts include the superseded quadratic role-contract implementation; exact current numerical replay verifies the optimized implementation. '
        'Current role-contract times are '+', '.join(f"{r['role_contract_seconds']:.3f} s" for r in numeric)+'. '
        f"The largest current numerical replay peak is {max(r['peak_rss'] for r in numeric):,} bytes. Native proof/checkpoint and next-layer work has a much larger peak ({peak:,} bytes); this task does not claim full production memory is solved. "
        'The retained trial-local optimization gives 6.483 s full versus 0.045 s local with exact row equality. No full future layer is retained for determinant admission. '
        'A separate lifetime defect retained the original expanded first-layer proof dictionary after native publication. The runner now retains the returned disk-backed state instead. '
        'A real native-run weak-reference regression failed before this fix and passes after it. The large cold measurements precede this lifetime fix; no reduced large-run RSS is claimed.')
    section('AA. Determinism',
        'Seven role/geometry reproducers have deterministic closure identity and high-precision agreement. All four current numerical replays reproduce every quality row exactly. '
        'All four native final states, atlas and forecast restart exactly; realized first-layer metrics match accepted metrics. '
        'No repeated full cold production study was substituted for these bounded restart checks.')
    section('AB. Tests',
        'See `TESTS_FOCUSED.log`, `TESTS_WINDOWS.log` and `TESTS_WSL.log`. The 35 focused applicability, checkpoint, local numerical and production-scale tests pass. '
        'The broad Windows run passed 875 tests and failed 11; it is not reported green. Failures concern historical branch/hash/large-file preservation assertions, a preserved Task-3 blocker assertion and a SymPy CRootOf polynomial-domain test. '
        'The Linux-only suite passed 11 tests under the existing system Python. Initial Windows collection failed on Linux-only `resource`; no dependency was installed or gate changed.')
    section('AC. Classification',classification+'. The declaration is limited to first-layer determinant applicability, not full production-scale geometry completion.')
    section('AD. Production-scale implications',
        'The prior broad first-layer rejection was a domain-state applicability defect. All four fixed geometries now pass that admission and resume native advancement. '
        'No wall, h1, schedule, gate, remesh science, certificate mathematics or V3 source changed. Later capacity, mixed transition, core quality and memory limits remain for the broader task.')
    section('AE. Recommended next task','Resume broader production-scale geometry qualification from these qualified first-layer semantics. Preserve full runtime/memory traces. Do not infer complete-mesh or CFD qualification from this bounded result.')
    question='## Decision question\n\nWhich determinant evaluation state has the complete and scientifically applicable inputs for V4 first-layer admission, and does correcting that applicability resolve the refined-wall production-scale rejection?\n\n'
    (OUT/'report.md').write_text(question+'\n'.join(paragraphs),encoding='utf-8',newline='\n')
    changed=subprocess.check_output(['git','diff','--name-only','--','research/sdf_cfd_mesher_v4'],cwd=repo,text=True).splitlines()
    new=subprocess.check_output(['git','ls-files','--others','--exclude-standard','--','research/sdf_cfd_mesher_v4/tools','research/sdf_cfd_mesher_v4/tests'],cwd=repo,text=True).splitlines()
    source={p:file_hash(repo/p) for p in sorted(set(changed+new)) if p.endswith('.py')}
    excluded={'AUTHORITY.json','SEAL.json','EVIDENCE_INDEX.json','COMMITTED_PATHS.json','GIT_STATUS.json'}
    evidence={p.relative_to(OUT).as_posix():dict(bytes=p.stat().st_size,sha256=file_hash(p))
              for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in excluded}
    g.save(OUT/'EVIDENCE_INDEX.json',evidence)
    g.save(OUT/'AUTHORITY.json',dict(classification=classification,parent='24ad4b03f7cdee69e17b7cc9674aa2c94eece3c1',
        report_sha256=file_hash(OUT/'report.md'),evidence_index_sha256=file_hash(OUT/'EVIDENCE_INDEX.json'),sources=source,cases=cases,
        determinant_state='Complete lateral first-prism topology plus declared internal top-cap incidence',
        complete_neighbor_quality=False,threshold=.0011,h1=.000166527069,
        other_gates_unchanged=True,wall_unchanged=True,v3_modifications=0,unrelated_dirty_entries=len(unrelated),
        sf2_regression=sf2['passed'],rounded_box_immutable_authority_replay=box['passed'],known_invalid_rejected=True,
        current_numerical_replay_exact=True,broader_production_scale_should_resume=True,
        limitations=['Fresh nominal rounded-box correction history is not claimed identical to obsolete exposed-cap search.',
                    'Full historical test suite has 11 failures; see raw log.',
                    'Full production-scale completion and memory are outside this bounded qualification.']))
    g.save(OUT/'SEAL.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json')))
    print((OUT/'SEAL.json').read_text())

if __name__=='__main__':run()
