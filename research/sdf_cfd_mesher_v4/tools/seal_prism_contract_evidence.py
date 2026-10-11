"""Seal decision C evidence. Never construct or publish prism geometry."""
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET

from production_adaptive_surface import REPO, V4, save, sha
from production_prism_sizing import contract_identity
from qualify_fresh_prism_contract import OUT


def read(name):
    return json.loads((OUT/name).read_text())


def main():
    pre=read('PREFLIGHT.json'); decision=read('GENERATION_AUTHORIZATION.json')
    wall=read('WALL_VERIFICATION.json'); trace=read('V3_IMPLEMENTATION_TRACE.json')
    runtime=read('RUNTIME_MEMORY.json'); replay=read('DETERMINISM.json')
    assert decision['decision']=='C' and not decision['generation_authorized']
    assert decision['contract_identity']==contract_identity()==replay['contract_identity']
    assert not decision['first_layer_qualified'] and replay['passed']
    assert not any(OUT.glob('*.npz')) and not (OUT/'CURRENT.json').exists()
    suite=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite')
    assert int(suite.attrib['failures'])==int(suite.attrib['errors'])==int(suite.attrib['skipped'])==0
    tests=dict(count=int(suite.attrib['tests']),seconds=float(suite.attrib['time']),failures=0,errors=0,skipped=0,
        command='python -B -m pytest -q research/sdf_cfd_mesher_v4/tests/test_production_prism_sizing.py research/sdf_cfd_mesher_v4/tests/test_production_adaptive_surface.py research/sdf_cfd_mesher_v4/tests/test_first_layer_metric_state.py::test_high_precision_resolves_role_difference',
        exclusions='No prism-generating fixture tests, first-layer geometry tests, layer-2 tests, full-front or core suites were run.',
        scope='Admission, source bindings, surface contract, binary64 schedules, preservation and existing tensor-role regression. Not capsule prism qualification.')
    recheck=ET.parse(OUT/'TRACE_RECHECK.xml').getroot().find('testsuite')
    assert int(recheck.attrib['failures'])==int(recheck.attrib['errors'])==int(recheck.attrib['skipped'])==0
    tests['source_trace_recheck']=dict(count=int(recheck.attrib['tests']),seconds=float(recheck.attrib['time']),
        reason='Added direct hash-bound SF2 schedule control after the selected suite passed.')
    save(OUT/'TESTS.json',tests)
    for row in trace['sources']+trace['additional_bindings']:
        assert sha(REPO/row['path'])==row['sha256']
    for row in pre['surface_input_files']:
        assert sha(row['path'])==row['sha256']
    current=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True).splitlines()
    unrelated=[s for s in current if not s[3:].startswith('research/sdf_cfd_mesher_v4/')]
    assert unrelated==pre['unrelated_dirty_state'], 'UnrelatedDirtyStateChanged'
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    assert head==pre['starting_head'], 'StartingHeadChangedBeforeSeal'
    preservation=dict(source_hashes_unchanged=True,wall_file_hashes_unchanged=True,
        v3_files_modified_by_task=0,unrelated_dirty_state_unchanged=True,
        unrelated_dirty_entries=len(unrelated),starting_head=head,
        limitation='V3 was already untracked as a tree. Modification count is task-attributable; all inspected reference hashes and the initial unrelated status are preserved.')
    save(OUT/'PRESERVATION.json',preservation)
    files=sorted((p for p in OUT.iterdir() if p.is_file()),key=lambda p:p.stat().st_size,reverse=True)
    disk=dict(starting_free_bytes=pre['starting_free_bytes'],ending_free_bytes=shutil.disk_usage(REPO).free,
        accessible_v4_result_tree_bytes=pre['v4_result_tree_bytes'],inaccessible_preexisting_entries=len(pre['inaccessible_entries']),
        adaptive_surface_input_bytes=pre['adaptive_surface_input_bytes'],cleanup_bytes_deleted=0,
        retained_task_bytes_at_preseal_snapshot=sum(p.stat().st_size for p in files),
        measurement_scope='Read-only snapshot immediately before report/authority sealing; later metadata and git object writes are not subtracted.',
        largest_task_artifacts=[dict(path=str(p.relative_to(REPO)),bytes=p.stat().st_size) for p in files[:8]],
        scratch_estimate='Not measured.',task_peak_disk_consumption='Not measured.')
    save(OUT/'DISK_FINAL.json',disk)
    classification='V4ProductionPrismSizingContractQualifiedInputRequired'
    lines=['## Decision question','',
        'Can V4 generate and qualify one fresh prism layer from the production adaptive capsule wall under a scientifically justified prism-sizing contract?','',
        '### A. Authority verification','',
        f'Classification: **{classification}**. Verified starting HEAD: `{head}`. Branch: `{pre["branch"]}`.',
        f'V3 modifications by this task: 0. All inspected reference hashes remain exact. The {len(unrelated)} unrelated dirty entries remain unchanged. V3 was already untracked; it was not staged or written.','',
        '### B. Production adaptive surface input','',
        f'Surface identity: `{wall["surface_identity"]}`. Geometry identity: `{wall["geometry_identity"]}`.',
        f'Coordinates: `{wall["coordinate_hash"]}`. Topology: `{wall["topology_hash"]}`. Sizing field: `{wall["sizing_field_hash"]}`.',
        'The production loader verified the sealed adaptive wall: 16,895 vertices, 33,786 triangles and 50,679 edges. No fixture wall or old capsule prism arrays were loaded.',
        'Preserved wall edge measurements: min 0.00099562504667614 m; median 0.001659583566434563 m; p95 0.002043220877557101 m; p99 0.0022160788495991287 m; max 0.0026833258652030293 m. Minimum triangle angle 29.503340348417925 degrees; maximum edge ratio 1.9955523014585648. These are surface descriptors, not prism-sizing gates.','',
        '### C. V3 prism implementation trace','',
        'Exact first-row entry: `research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09/first_row.py::main`. Native publication: that family\'s `materialize.py` -> `native_io.prepare_first` -> `contractDirect`; `contractStaged` is later. None was executed.',
        'The following source files were inspected and SHA-256 bound. Symbols and line numbers are preserved in V3_IMPLEMENTATION_TRACE.json. V3 study modules with import-time writers were not imported.','',
        '| V3 source relative to research/sdf_cfd_mesher_v3 | Measured implementation role |','|---|---|']
    for row in trace['sources']:
        lines.append(f'| `{row["path"].split("research/sdf_cfd_mesher_v3/")[1]}` | {row["role"]} |')
    lines += ['', 'The preserved V3 bounded-prism metric and incidence tests were inspected, not executed. Their manufactured multilayer stacks are outside this task.', '',
        '### D. V3 prism-sizing contract','',
        'The mesher consumes a request. In the practical NACA producer, `nu=mu/rho`, `u_tau=U*sqrt(Cf_design/2)`, and full first prism thickness `h1=2*y_plus*nu/u_tau`. The factor two accounts for the cell centre at half-height. The producer selects a wall treatment using flat-plate comparisons and a surface-weighted p75 positive attached NACA Cf. It then searches layer count and growth for a declared total thickness.',
        'This is an external physical policy, not a generic SDF mesher default. Geometry constrains feasibility. Surface topology supplies columns and lateral neighbours. The prism-host metric consumes h1 to constrain tangential sizing; it does not derive h1 from wall edges.',
        'The original practical-policy and frozen-schedule JSON outputs are absent. Their producer, historical report and later independent SF2 constants are available. Exact reproduction of the missing historical physics output is not claimed.', '',
        '### E. Parameter provenance','',
        'A = user/CFD physics; B = geometry-derived; C = surface-derived; D = numerical safeguard; E = V3 case-specific; F = diagnostic. No A value has been supplied for this capsule.','',
        '| Parameter | Class | Recovered value and units | Scope |','|---|---|---|---|']
    for row in read('PARAMETER_PROVENANCE.json')['parameters']:
        value='Not supplied.' if row['value'] is None else str(row['value'])
        lines.append(f'| {row["parameter"]} | {row["category"]} | {value}; {row["units"]} | {row["scope"]} |')
    lines += ['', 'The practical NACA policy tested nominal y+ values 1, 5, 10 and 30. It used the converged y+=5 comparison anchor, 2% integrated-Cf and 5% local-p95-Cf guides, then selected historical y+=10. Those are wall-treatment study criteria, not capsule mesh gates. Growth bisection used 100 iterations. The earlier y+=0.8 producer is a different historical policy.', '',
        '### F. Generic versus case-specific sizing','',
        'Generic: explicit request consumption, binary64 persistence, source-normal/incident-sheet constraints, source ownership, geometric feasibility and fail-closed quality/publication. Case-specific: NACA shear/model, SF2 sizes, prism-host construction constants and later selector study bounds. The held-out fixture rule N=15, r=1.06, H=min(0.04*S,0.20*clearance) is separate development evidence, not capsule production authority.',
        'V4 adds a provenance admission adapter. It does not port NACA physics as capsule defaults, change scientific gates, or run geometry. The implemented request scope is global h1 with explicit increments. Local h1 needs a separately qualified source map; no arbitrary local values are substituted.', '',
        '### G. Capsule normal-schedule provenance','',
        'No independent capsule request was found in the inspected production, physics, planner, case and authority evidence. The old capsule Provider called `forecast.schedule.load_schedule`, which reads `TASK1_SOURCE_BINDING.json` and the SF2/A/wing increments. The production-scale cohort retained that fixture schedule. Capsule dimensions based on T=0.02 m are geometry design data, not a physical boundary-layer request. The new adaptive surface authority has no normal schedule.', '',
        '### H. Production prism-sizing contract','',
        f'Contract identity: `{contract_identity()}`. Entry: `tools/production_prism_sizing.py::inspect_first_layer_inputs`.',
        'Required: geometry and surface binding; request owner; physical basis and hash-bound evidence; fluid side; full first-prism height semantics; positive finite explicit binary64 increments/count; exact nominal schedule total; separate physical total-thickness intent. Canonical float.hex encoding and immutable little-endian arrays preserve the schedule. No h1, growth, count or thickness default exists.',
        'An explicit schedule can be the output of a qualified physical producer or a justified production-test request. Hash consistency does not certify the external CFD model. A future generator must reload the request and wall, check feasibility, and qualify all geometry before publication. No saved admission receipt is a publication token.',
        'First height cannot be compressed. Later local modifications require their own qualified authority and separate requested/actual records. Existing frozen gates and the actual-lateral/internal-top first-layer determinant interpretation remain unchanged.', '',
        '### I. Layer-generation authorization decision','',
        '**Decision C: required external normal schedule is missing. Generation is not authorized.** Missing: a capsule-bound physical or justified production-test request, fluid side, full normal h1 with global/local meaning, schedule count/increments and total intent, and the owner/basis evidence. If derived from y+, preserve the wall treatment, flow/fluid properties and shear estimate. Do not choose an arbitrary number.', '']
    sections=[('J','Fresh first-layer construction'),('K','Normal and target construction'),('L','SDF projection'),
        ('M','First-layer topology'),('N','Hard prism quality'),('O','Wall-to-top tangential evolution'),
        ('P','Triangle-shape evolution'),('Q','Normal-height accuracy'),('R','Size-height diagnostics'),
        ('S','Curvature-region results'),('T','Surface-sizing correlation'),('U','Continuous geometry')]
    for letter,title in sections:
        lines += [f'### {letter}. {title}', '', 'Not run because authoritative normal schedule is missing. All corresponding layer-1 measurements: **Not measured.**', '']
    lines += ['The recovered normal rule uses live oriented source normals. At incident sheets solve N*v=1 and do not renormalize the compatible vector. The accepted SF2 first row used wall+H1*v. Offset-SDF projection of that top was **Not used by V3 in this recovered path**. Source-wall projection is separate. The current task computes neither normals nor targets.', '',
        'Preserved SF2 first-row control (not regenerated, not capsule authority): 106,302 prisms, 159,453 shared sides; minimum determinant 0.021681324615089134; minimum side weight 0.12010820812662282; maximum skewness 2.518464807545247; minimum exact Jacobian 8.846350679061339e-11; minimum volume 4.4465901675836166e-11 m^3; maximum normal-height residual 1.1083256708233069e-16 m. All recorded determinant/Jacobian/orientation/side-weight/skew/volume failure counts are zero. Complete collision and topological audits passed with zero nonincident overlap/contact. SF2 aspect and volume-ratio measurements in this control: Not measured. The 102,379 isolated-cap determinant failures were diagnostic, not the authoritative prospective-cap result.', '',
        '### V. Wall preservation','',
        'Coordinates, connectivity, coordinate hash, topology hash, sizing field and surface identity are unchanged. Every file in the sealed fresh surface input directory retains its preflight SHA-256. No wall remeshing or displacement occurred.', '',
        '### W. First-layer authority','',
        'Not run because authoritative normal schedule is missing. No first-layer authority was published. First-layer scientific identity: Not measured. AUTHORITY.json seals the input-required contract result only.', '',
        '### X. Determinism','',
        'Two independent production wall loads produced exactly equal missing-request decisions, identities and wall bindings. This is contract replay, not first-layer replay. Coordinates/connectivity/quality/projection/certificate replay for a generated first layer: Not measured.', '',
        '### Y. Runtime and memory','',
        f'Recorded read-only study: {runtime["study_seconds"]} s; authority load/decision {runtime["authority_load_and_missing_request_decision_seconds"]} s; repeated load/decision {runtime["authority_load_and_missing_request_replay_seconds"]} s. Peak RSS {runtime["peak_rss_bytes"]} bytes; retained RSS {runtime["retained_rss_bytes"]} bytes. These are study-process measurements, not whole-session or pytest peaks.',
        f'Selected tests: {tests["seconds"]} s. Cold layer generation, layer replay and their stage runtimes: Not measured. Total session active scientific runtime: Not measured. Native mesher invocations: 0. One bounded existing surface-intersection regression ran; no prism or core solver ran.', '',
        '### Z. Disk management','',
        f'Starting free space: {pre["starting_free_bytes"]} bytes. Accessible V4 result tree: {pre["v4_result_tree_bytes"]} bytes; {len(pre["inaccessible_entries"])} preexisting inaccessible entries excluded. Adaptive surface input: {pre["adaptive_surface_input_bytes"]} bytes.',
        'Largest inputs: QUALITY.npz 5,026,067 bytes; surface.off 1,750,411; raw.mesh 1,635,245; SURFACE.npz 838,502; GEOMETRY_CERTIFICATE.npz 518,133. Expected scratch: Not measured. No meshing scratch was needed.',
        f'Cleanup: 0 files, 0 bytes. Ending free space at the preseal evidence snapshot: {disk["ending_free_bytes"]} bytes. Retained result bytes at that snapshot: {disk["retained_task_bytes_at_preseal_snapshot"]}. Final metadata/git writes are outside that snapshot. No accepted authority, old dump or unrelated artifact was deleted. Full peak task disk consumption: Not measured.', '',
        '### AA. Tests','',
        f'{tests["count"]} passed; 0 failed; 0 errors; 0 skipped in the selected suite. Command: `{tests["command"]}`.',
        f'The final source-trace extension was checked with the {tests["source_trace_recheck"]["count"]} admission tests again; all passed in {tests["source_trace_recheck"]["seconds"]} s.',
        'Coverage: production wall loading and fixture rejection; physical provenance/source hashes; geometry/surface binding; explicit fluid side; exact binary64 and immutable schedule arrays; overflow/lost increments; separate design intent; unsupported local mapping rejection; exact source trace; wall preservation; no geometry/publication on missing input; deterministic admission; existing surface validation and first-layer tensor-role regression.',
        'Capsule prism normal/target/projection, Jacobian, pair-quality, ownership, evolution, curvature, continuous certificate and replay tests were not run because no authoritative request exists. The preserved SF2 result is a control, not a fresh test. Tests that construct procedural capsule layers or layer 2 were deliberately excluded.', '',
        '### AB. Classification','',
        f'**{classification}**. Supported claim: the V3-derived production sizing contract and the new provenance admission path are qualified in the tested global-schedule scope. The adaptive capsule wall remains SurfaceQualified. No capsule FirstLayerQualified, shell, core-handoff, complete-mesh, Foundation or CFD-accuracy claim is made.', '',
        '### AC. Recommended next task','',
        '**BindV4CapsulePhysicalPrismRequest**: obtain and bind the authoritative physical/domain request or justified production-test schedule to this geometry and surface. Preserve fluid side, full normal h1, global/local meaning, schedule/count and total intent, physical basis, owner and source hashes. If physics-derived, preserve the selected wall treatment/y+ basis, flow/fluid properties and shear estimate. Then rerun **QualifyV4FreshCapsuleFirstLayerFromAdaptiveSurface** for exactly one fresh layer. Do not return to old core repair or generate layer 2.', '']
    (OUT/'report.md').write_text('\n'.join(lines),encoding='utf-8')
    evidence={p.name:sha(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')}
    authority=dict(schema='V4PrismSizingContractQualification/1',classification=classification,
        starting_head=head,decision='C',generation_authorized=False,first_layer_qualified=False,
        surface_identity=wall['surface_identity'],geometry_identity=wall['geometry_identity'],
        contract_identity=contract_identity(),schedule_identity=None,first_layer_identity=None,
        generated_layers=0,forecast_layers=0,generated_core_cells=0,wall_preservation=wall,
        provenance=read('CAPSULE_REQUEST_SEARCH.json'),tests=tests,determinism=replay,
        implementation_files={str(p.relative_to(REPO)):sha(p) for p in [
            V4/'tools/production_prism_sizing.py',V4/'tools/qualify_fresh_prism_contract.py',
            V4/'tools/seal_prism_contract_evidence.py',V4/'tests/test_production_prism_sizing.py',
            V4/'PRODUCTION_PRISM_SIZING_CONTRACT.md']},evidence=evidence,
        claim='Qualified physical request provenance/admission contract. No capsule layer generated or qualified.',
        limitation='External capsule normal schedule absent; original practical NACA policy JSON unavailable; local request mapping not qualified.',
        next_task='BindV4CapsulePhysicalPrismRequest')
    save(OUT/'AUTHORITY.json',authority)
    hashes={name:sha(OUT/name) for name in ('report.md','AUTHORITY.json')}
    save(OUT/'HASHES.json',hashes)
    print(json.dumps(dict(classification=classification,hashes=hashes,preservation=preservation,disk=disk),indent=2))


if __name__=='__main__':main()
