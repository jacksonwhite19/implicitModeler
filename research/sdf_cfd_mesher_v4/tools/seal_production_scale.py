"""Seal measured larger-case limits and regression evidence without pass inflation."""
import hashlib
import json
import re
import subprocess
from pathlib import Path
from run_production_scale import OUT
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.advancing_front.production_authority import configuration

ROOT=OUT.parents[1]
BASE='e09fc8bddea259e77024132352f104651c9181c8'


def write(path,value):
    path.write_text(json.dumps(value,sort_keys=True,indent=2)+'\n',encoding='utf8',newline='\n')


def run():
    cases=json.loads((OUT/'CASE_RESULTS.json').read_text())
    assert len(cases)==4 and all(all(r['replay'].values()) for r in cases)
    assert all(not r['committed_shell'] for r in cases)
    prior=ROOT/'results/broader_geometry_family_qualification'
    expected={'report.md':'3f57ce0ccc0665933c0857e427e204427ddb35c4a913188e1f47b3a05bed746f',
              'AUTHORITY.json':'0662d3de9f35feee580b496032d4e536b2b60fb27d15d150665df1a6d432d426'}
    actual={name:file_hash(prior/name) for name in expected};assert actual==expected
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==BASE
    frozen=ROOT/'results/fixed_interface_bulk_core_repair'
    authority=json.loads((frozen/'AUTHORITY.json').read_text())
    preserved={}
    for name in ('ASSEMBLY.json','FOUNDATION.json','CERTIFICATE_INPUT.json'):
        digest=file_hash(frozen/name);assert digest==authority['evidence'][name];preserved[name]=digest
    assert authority['foundation_literal_mesh_ok'] and authority['complete_relations']==226619419
    assert configuration()['automatic_remesh']
    write(OUT/'AUTHORITY_VERIFICATION.json',dict(base_commit=BASE,prior=actual,
        preserved_SF2=preserved,production_configuration=configuration(),
        v3_task_modifications=0,scientific_gates_changed=False,certificate_mathematics_changed=False))
    scaling=[]
    for row in cases:
        scaling.append(dict(case=row['case'],wall_vertices=row['wall']['vertices'],wall_triangles=row['wall']['triangles'],
            requested_layers=20,committed_shell_cells=0,remesh_events=0,complete_cells=0,
            cold_seconds=row['performance']['seconds'],peak_rss_bytes=row['performance']['peak_rss_bytes'],
            classification=row['classification'],reason=row['reason']))
    write(OUT/'SCALING.json',scaling)
    write(OUT/'STAGE_OUTCOMES.json',dict(initialization='Four rejected first-layer searches; exact replay',
        remesh='NotReached',coupled_closure='NotReached',outer_interface='NotReached',core_generation='NotReached',
        core_repair='NotReached',complete_assembly='NotReached',complete_certificate='NotReached',foundation='NotReached',
        event_cost_median_seconds=None,event_cost_max_seconds=None,false_remesh_triggers=None,missed_remesh_triggers=None,
        false_coupled_revisions=None,missed_coupled_failures=None,
        explanation='No first layer was admitted. Unreached stages and confusion populations are not counted as passes.'))
    testlog=(OUT/'TESTS.log').read_text();match=re.search(r'(\d+) passed in ([\d.]+)s',testlog);assert match
    tests=dict(passed=int(match[1]),seconds=float(match[2]),wsl=(OUT/'TESTS_WSL.log').read_text())
    write(OUT/'TEST_RESULTS.json',tests)
    table='| Case | Wall vertices / triangles | Initial determinant failures | Trials / provisional updates | Cold s | Peak RSS bytes |\n|---|---:|---:|---:|---:|---:|\n'
    for r in cases:
        table+=f"| {r['case']} | {r['wall']['vertices']} / {r['wall']['triangles']} | {r['initial_determinant_failures']} | {r['trials']} / {r['provisional_local_updates']} | {r['performance']['seconds']:.2f} | {r['performance']['peak_rss_bytes']} |\n"
    cap='; '.join(f"{r['case']}: {r['prospective_cap_incidence']['minimum_determinant']:.12g}" for r in cases)
    slow=max(cases,key=lambda r:r['performance']['seconds'])
    peak=max(r['performance']['peak_rss_bytes'] for r in cases)
    all_attempt_peak=max(json.loads(line)['peak_rss_bytes'] for path in OUT.glob('*/*/COLD_MEMORY.jsonl') for line in path.read_text().splitlines())
    sections=[
      ('A. Authority verification',f'HEAD {BASE} and both supplied family hashes match. AUTHORITY_VERIFICATION.json binds the preserved SF2 assembly, certificate input and Foundation records. Native production authority and automatic remeshing remain enabled. V3 task modifications: zero. Unrelated dirty files remain outside this task.'),
      ('B. Existing production-scale baseline','SF2 bounded native replay passes: 20 layers, 2,126,040 prisms, 56,462,864 active certificate relations; zero missing, duplicate, forbidden, unresolved or recomputed relations. Wall, first layer, forecast, interface and restart identities remain exact. Remesh count is zero. Preserved complete SF2 authority remains 3,776,026 cells with literal Foundation 13 Mesh OK. Full numerical A/A2 was not rerun.'),
      ('C. Cohort design','Four larger cases reuse existing constructions. No new SDF family or dimensional sweep was introduced. Five octahedral subdivisions per component give 8,192 triangles per component, 64 times the old level-2 wall population. Dimensions, source definitions and the stored 20-layer schedule were frozen before any execution. No resolution or dimension was changed after observing a failure.'),
      ('D. Geometry authorities','COHORT_MANIFEST.json and each WALL_AUTHORITY.json record full preserved primitive parameters, analytic source identity, sign, wall coordinate/topology hashes, edge-length range and schedule identity. Wall extraction remains deterministic radial bisection. h1 is 0.000166527069 m. No new production CFD patch/support authority exists for these bounded cases.'),
      ('E. Wall scale',table+'\nThese are full wall populations, not subsets. The largest attempted first-layer mesh contains 65,536 prisms; none is an admitted shell.'),
      ('F. Multi-region cases','Four separated competing pairs give eight source surfaces and four difficult gap regions. Three interacting ellipsoids give nearby competing branches. The intended multi-event qualification was not reached: first-layer admission failed first.'),
      ('G. Interacting regions','The interacting design preserves the previous equilateral component placement and dimensions. No region merge/separation outcome is claimed. Dependency interaction during committed events remains untested at this scale.'),
      ('H. Shell production traces','D is the final cold primary attempt; E is an independent cold replay. Both use AnalyticProvider and MixedFrontRunner. All four finish with FirstLayerCorrectionExhausted. CASE_RESULTS.json distinguishes provisional local updates from committed production state. Earlier A/B/C experiments and their interruptions remain recorded separately.'),
      ('I. Correction behavior','Existing tangent directions, trial scales, projection and strict failure-set reduction are unchanged. Two measured scaling defects were repaired: whole-shell metric recomputation on each local trial, and full quality arrays retained in rejected-search history. Local rechecks use the original metric functions on changed-cell stars and adjacent faces; successful candidates still undergo full metric equality checking and certification. Trial evidence is streamed to deterministic lossless gzip objects. No gate or search decision was relaxed.'),
      ('J. Remesh event histories','Committed remesh events: zero. Initialization precedes the termination/remesh policy. First-layer height cannot be compressed or rolled back. These failures are not evidence of missed remesh triggers.'),
      ('K. Remesh locality','Maximum committed rollback depth and removed-cell count are zero. Event locality, retained atlas regions, forecasts, quality receipts and proofs were not exercised at larger scale. Local correction supports were exercised; the real 8,192-cell equivalence probe changed one vertex incident to four cells.'),
      ('L. Coupled shell/core validation','No mixed candidate reached provisional core closure. The qualified coupled acceptance path remains enabled and unmodified. False revisions and missed coupled failures are not evaluated.'),
      ('M. Shell quality','All four frozen designs fail shell-local first-layer determinant admission after the existing tangent searches. Exact initial failure populations and trial records are preserved. The diagnostic prospective-cap determinant minima are '+cap+'. This diagnostic does not replace the frozen shell-local gate. Failure is specific to the fixed wall/schedule/provider combination; it is not proof that the analytic shapes cannot be meshed.'),
      ('N. Outer-interface qualification','Not reached for a new admitted shell. No rejected first layer was exported as a qualified core interface. SF2 interface identity remains exact in the bounded regression.'),
      ('O. Core generation','Not reached. Generating a core from a rejected shell would violate the qualification sequence. No new core result is inferred from preserved bounded examples.'),
      ('P. Core repair','Not reached. No new case is labelled core-unsupported; all four are shell-admission limited.'),
      ('Q. Complete assembly','New complete assemblies: zero. New qualified production-domain meshes: zero. New bounded complete qualifications: zero. No gap, overlap, ownership or patch pass is claimed for an unbuilt complete mesh.'),
      ('R. Complete certification','No new complete certificate population. SF2 preserved shell proofs remain 56,462,864; its preserved complete authority contains 226,619,419 relations. The generic mixed certificate AABB scan now uses an inclusive bound tree and bounded pair batches. Tests establish the exact old ordered population and unchanged proof records; the exact kernel and hull predicate mathematics are untouched.'),
      ('S. Foundation qualification','Preserved SF2 literal Mesh OK. remains hash verified. No new Foundation run was possible because no new shell admitted its first layer.'),
      ('T. Production-domain outcomes','No additional full-domain authority or complete qualification. Numerical status for all new cases: NotEvaluatedDueToMissingAuthoritativeCaseControls. This is separate from the observed geometric admission failures.'),
      ('U. Bounded-domain outcomes','Four ProductionScaleGeometryUnsupported results with reason FixedDesignFirstLayerCorrectionExhausted. Unsupported means the current frozen production path exhausted its supported first-layer search. It does not mean a geometric impossibility theorem.'),
      ('V. Memory scaling',f'Final primary peak RSS is {peak} bytes. Across all attempts, including interrupted profiling baselines, the observed high-water is {all_attempt_peak} bytes. COLD_MEMORY.jsonl records sampled phases and OS high-water values. Retained correction-search memory remains stable after replacing retained full trial graphs. No post-remesh memory trace exists because no event committed. The old 9,470,873,600-byte multi-event peak is a different workload and is not declared solved. Interrupted pre-fix peaks remain separate evidence.'),
      ('W. Runtime scaling',f"Slowest final cold primary rejection: {slow['case']}, {slow['performance']['seconds']:.2f} s. These are time-to-rejection measurements, not completed-shell runtimes. Cases ran in separate processes with overlap; timings are not isolated-machine scaling benchmarks. The 8,192-cell numerical probe measured 7.2813866 s full versus 0.0591935 s local with identical rows. The 24,576-face AABB scan measured 6.8128644 s versus 3.3994048 s with identical ordered pairs; at 8,192 faces the index was slightly slower. No universal complexity law is inferred."),
      ('X. Event-cost distribution','No remesh event cost population. Median and maximum are null, not zero-time events. Cold initial searches and interrupted profiling work are separately retained.'),
      ('Y. Checkpoint/restart','No new cohort checkpoint exists; therefore between-event restart was not reached. SF2 native checkpoint restart passes. A preserved corrected rounded-box first-layer regression also admits 128 cells, reloads exactly, and matches the sealed wall and first-layer geometry. It is a regression control, not a new production-scale qualification. The completed rejected searches have independent cold replays rather than a falsely claimed restart of a nonexistent shell.'),
      ('Z. Determinism','D/E reproduce exact initial numerical state, complete chronological tangent trial history, coordinate hashes, archived gate hashes, provisional update count and terminal rejection for all four cases. No final shell/core/complete-mesh replay is claimed.'),
      ('AA. Unsupported cases','All four cases are preserved without dimensional tuning. The prospective-cap diagnostic distinguishes shell-local incidence from eventual complete-neighbor incidence. Resolving the admission limitation must preserve the frozen gates and validate their correct applicability; this task does not silently drop the shell-local test. A mixed-code intermediate attempt failed with an API mismatch during repair deployment; it is explicitly excluded and was rerun cold.'),
      ('AB. Tests',f"{tests['passed']} Windows tests passed in {tests['seconds']} s, plus the preserved Linux core suite (see TESTS_WSL.log). Coverage includes exact local/full metric equality for prism and mixed variable-layer cells, ordered AABB candidates and proof identity, deterministic archive hashes, no retained scientific arrays, frozen cohort design, and fail-closed classification of infrastructure errors. Existing native checkpoint, mixed front, coupled validation, core repair and certificate tests are retained."),
      ('AC. Classification','V4BroaderProductionScaleGeometryInsufficient. Production-scale admission fails across all four new cases. The generic scaling fixes pass regression, but the requested larger multi-event and complete-mesh qualifications are not established.'),
      ('AD. Supported production-scale claim','No expanded production-scale geometric operating envelope is established. Four preserved constructions were tested at 4,098–32,784 wall vertices and 8,192–65,536 wall triangles. All stopped before an admitted first layer. SF2 remains the qualified production-scale reference. No universal robustness, broad CFD accuracy or new spatial convergence claim is made.'),
      ('AE. Recommended next task','Resolve first-layer determinant admission on refined production walls, including shell-local versus prospective complete-neighbor incidence, without changing thresholds, wall geometry, h1 or certificate mathematics. Reuse this frozen cohort afterward. Broader CFD qualification and performance claims are premature.'),
    ]
    report='## Decision question\n\nCan the current V4 production architecture qualify larger multi-feature geometries with multiple local correction/remesh events while keeping quality, memory, runtime, and state management bounded?\n\nNo expanded qualification was established in this cohort. The four cold runs exhausted first-layer correction. Generic scaling defects were fixed and tested; no scientific gate was weakened.\n\n'
    report+='\n\n'.join('### '+title+'\n\n'+body for title,body in sections)+'\n'
    (OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
    inputs={str(p.relative_to(OUT)):dict(sha256=file_hash(p),bytes=p.stat().st_size) for p in sorted(OUT.rglob('*'))
            if p.is_file() and p.name not in ('AUTHORITY.json','EVIDENCE_INDEX.json','HASHES.json') and p.suffix!='.tmp'}
    write(OUT/'EVIDENCE_INDEX.json',inputs)
    source_paths=subprocess.check_output(['git','-c','core.longpaths=true','diff','HEAD','--name-only','--','research/sdf_cfd_mesher_v4'],text=True).splitlines()
    source_paths+=subprocess.check_output(['git','-c','core.longpaths=true','ls-files','--others','--exclude-standard','--','research/sdf_cfd_mesher_v4'],text=True).splitlines()
    sources={p:file_hash(Path(p)) for p in sorted(set(source_paths)) if p.endswith('.py')}
    authority=dict(classification='V4BroaderProductionScaleGeometryInsufficient',base_commit=BASE,
        report_sha256=file_hash(OUT/'report.md'),evidence_index_sha256=file_hash(OUT/'EVIDENCE_INDEX.json'),
        sources=sources,new_cases=4,completed_shells=0,new_complete_qualifications=0,committed_remesh_events=0,
        rejected_search_replay_exact=True,completed_mesh_replay=False,automatic_remesh_enabled=True,
        native_production_authority_unchanged=True,v3_task_modifications=0,
        scientific_gates_changed=False,certificate_mathematics_changed=False,
        storage='Full first-layer arrays and compressed trial records remain local and hash-bound in EVIDENCE_INDEX.json; deterministic inputs and code are committed')
    write(OUT/'AUTHORITY.json',authority)
    write(OUT/'HASHES.json',{name:file_hash(OUT/name) for name in ('report.md','AUTHORITY.json')})


if __name__=='__main__':run()
