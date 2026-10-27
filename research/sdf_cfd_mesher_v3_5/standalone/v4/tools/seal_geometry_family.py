"""Seal measured family evidence without promoting prefixes to full cases."""
import json,hashlib,subprocess
from pathlib import Path
from collections import Counter
from summarize_geometry_family import OUT,read,write,committed_history

ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def run():
    inventory=read(OUT/'COHORT_MEASUREMENTS.json');rows=[]
    walls={r['case']:r for r in read(OUT/'WALL_AUTHORITIES.json')}
    for case in inventory['cases']:
        runs=case['runs'];preferred=(case.get('determinism') or {}).get('primary_label');label=preferred if preferred in runs else next((k for k in ('V','T','Q') if k in runs),None)
        r=runs.get(label,{})
        shell=r.get('shell_result') or {};assembly=r.get('assembly') or {}
        schedule_complete=bool(shell and shell.get('final')==r.get('scientific') and not r.get('remaining_active_columns',[]))
        complete=bool(schedule_complete and assembly.get('passed') and assembly.get('shell_scientific')==r.get('scientific'))
        failure=(r.get('failure') or {}).get('reason')
        if schedule_complete:
            failure=None if complete else 'PreservedFixedInterfaceCoreRoutesNotQualified'
        classification='GeometryFamilyCaseBoundedQualified' if complete else ('GeometryFamilyCasePartiallyQualified' if r.get('total_shell_cells') else 'GeometryFamilyCaseUnsupported')
        if not walls[case['spec']['name']]['passed']:
            classification='GeometryFamilyCaseBlockedByMissingAuthority'
            failure='NoAdmittedUnionWallAuthority:'+walls[case['spec']['name']]['error']
        selection=read(OUT/case['spec']['name']/str(label)/'CORE_SELECTED.json',{})
        selected=selection.get('directory')
        if selected is None:
            selected=next((k for k in ('repaired_core','core') if r.get('core_trials',{}).get(k,{}).get('passed')),None)
        row=dict(name=case['spec']['name'],classes=case['spec']['classes'],spec=case['spec'],run=label,classification=classification,
            reason=None if complete else failure or 'CompleteQualificationNotEstablished',shell_cells=r.get('total_shell_cells',0),
            shell_types=r.get('cells',{}),local_layers=r.get('local_layer_distribution',{}),active_pending=len(r.get('remaining_active_columns',[])),
            shell_schedule_complete=schedule_complete,events=r.get('events',[]),
            mechanisms=r.get('mechanisms',{}),accepted_corrections=r.get('accepted_corrections',{}),initial_correction=r.get('initial_correction',False),quality=r.get('quality',{}),shell_relations=r.get('shell_relations',0),
            complete_assembly=assembly if assembly.get('shell_scientific')==r.get('scientific') else None,
            performance=r.get('performance'),accepted_step_seconds=r.get('accepted_step_seconds',0),complete_performance=r.get('complete_performance'),
            memory_trace=r.get('retained_rss_by_step',[]),core_trials=r.get('core_trials',{}),determinism=case.get('determinism'),
            qualified_core=r.get('core_trials',{}).get(selected) if complete else None,
            core_repair=read(OUT/case['spec']['name']/str(label)/str(selected)/'REPAIR.json') if complete else None,
            ordinary_commits=sum(t['operation']=='PrismCommit' for t in r.get('trace',[])),
            first_layer_evidence=r.get('first_layer'),
            wall_exact=shell.get('wall_exact'),first_layer_exact=shell.get('first_layer_exact'),scientific=r.get('scientific'))
        rows.append(row)
    qualified=[r for r in rows if r['classification']=='GeometryFamilyCaseBoundedQualified']
    partial=[r for r in rows if r['classification']=='GeometryFamilyCasePartiallyQualified']
    unsupported=[r for r in rows if r['classification']=='GeometryFamilyCaseUnsupported']
    events=[e for r in rows for e in r['events']]
    assemblies=[r['complete_assembly'] for r in rows if r['complete_assembly'] and r['complete_assembly']['passed']]
    peaks=[(r['performance'] or {}).get('peak_rss_bytes',0) for r in rows]
    measured=read(OUT/'PERFORMANCE_INVENTORY.json',{}).get('peaks',[])
    summary=dict(classification='V4BroaderGeometryFamilyPartiallyQualified',new_constructions=8,new_design_records=len(rows),
        preserved_controls=6,total_cohort_designs_plus_controls=len(rows)+6,bounded_qualified_new=len(qualified),
        production_domain_qualified_new=0,completed_native_shells=sum(r['shell_schedule_complete'] for r in rows),partially_qualified_new=len(partial),unsupported_fixed_designs=len(unsupported),
        rejected_wall_authorities=sum(r['classification']=='GeometryFamilyCaseBlockedByMissingAuthority' for r in rows),
        largest_shell=max(r['shell_cells'] for r in rows),largest_complete=max((a['total_cells'] for a in assemblies),default=0),
        largest_complete_certificate=max((a['total_relations'] for a in assemblies),default=0),
        largest_certificate=max([r['shell_relations'] for r in rows]+[a['total_relations'] for a in assemblies]),largest_primary_peak_rss=max(peaks,default=0),
        largest_recorded_peak_rss=max((r['bytes'] for r in measured),default=max(peaks,default=0)),
        slowest_accepted_step_history=max((dict(case=r['name'],seconds=r['accepted_step_seconds']) for r in rows),key=lambda r:r['seconds']),
        largest_shell_certificate=max(r['shell_relations'] for r in rows),
        committed_remesh_events=len(events),largest_removed_cell_set=max((len(e['removed_cells']) for e in events),default=0),
        maximum_removed_layer_span=max((max(0,int(layer)-e['retained_layer']) for e in events if e['retained_layer'] is not None for layer in e['removed_layers']),default=0),
        lowest_retained_layer=min((e['retained_layer'] for e in events if e['retained_layer'] is not None),default=None),
        worst_completed_shell_determinant=min((r['quality']['minimum']['determinant'] for r in rows if r['shell_schedule_complete']),default=None),
        worst_completed_shell_weight=min((r['quality']['minimum']['weight'] for r in rows if r['shell_schedule_complete']),default=None),
        worst_qualified_core_determinant=min((r['qualified_core']['core']['minimum']['determinant'] for r in qualified if r['qualified_core']),default=None),
        worst_qualified_core_weight=min((r['qualified_core']['core']['minimum']['interpolation_weight'] for r in qualified if r['qualified_core']),default=None),
        maximum_active_cavity_vertices=max((e['active_vertices'] for e in events),default=0),
        scope='Fixed analytic designs and their admitted prefixes; no universal geometric or CFD claim',cases=rows)
    write(OUT/'CASE_SUMMARY.json',summary)
    matrix=[]
    for r in rows:
        operations=committed_history(OUT/r['name']/str(r['run']),'OPERATIONS.json') if r['run'] else []
        direct=sum(op['operation']=='RollbackRemesh' and any(t['state']=='RemeshCandidate' and 'Exact collision' in t['reason'] for t in op['policy']['trace']) for op in operations)
        matrix.append(dict(case=r['name'],policy_states_visited=r['mechanisms'],accepted_corrections=r['accepted_corrections'],
            ordinary_prism_commits=r['ordinary_commits'],first_layer_tangent_correction=r['initial_correction'],
            direct_exact_collision_escalations=direct,rollback_remesh=len(r['events']),
            deeper_rollback_events=sum(any(int(layer)-e['retained_layer']>1 for layer in e['removed_layers']) for e in r['events'] if e['retained_layer'] is not None),
            coupled_candidate_rejections=sum(e['coupled_rejections'] for e in r['events']),
            coupled_closure_required_events=len(r['events']),fixed_interface_core_repair=r['core_repair'],classification=r['classification']))
    write(OUT/'MECHANISM_MATRIX.json',dict(meaning='Visited or inapplicable policy stages are not accepted corrections. Counts are descriptive, not scores.',cases=matrix))
    write(OUT/'UNSUPPORTED_DESIGNS.json',dict(scope='Limits of the exact wall extraction, schedule and preserved repair paths; no shape-wide infeasibility claim',
        cases=[dict(case=r['name'],classification=r['classification'],reason=r['reason'],first_layer=r['first_layer_evidence'],
                    last_layers=r['local_layers'],shell_schedule_complete=r['shell_schedule_complete'],core_trials=r['core_trials'])
               for r in rows if r['classification']!='GeometryFamilyCaseBoundedQualified']))
    write(OUT/'PARAMETER_RESULTS.json',dict(authority='PARAMETER_DESIGN.json',results=[dict(case=r['name'],scale=r['spec'].get('scale_factor',1),classification=r['classification'],shell_regime=('LocallyTerminated' if r['events'] else ('CorrectedFullPrismStack' if r['initial_correction'] or r['accepted_corrections'] else 'FullPrismStack')) if r['shell_schedule_complete'] else 'UnsupportedRequestedSchedule',last_layers=r['local_layers'],reason=r['reason']) for r in rows[:24]],interpretation='Fixed h1 and wall incidence. These are not shape-wide capacity thresholds.'))
    table='| Case | Result | Shell cells | Events | Pending active columns |\n|---|---|---:|---:|---:|\n'+'\n'.join(f"| {r['name']} | {r['classification'].replace('GeometryFamilyCase','')} | {r['shell_cells']} | {len(r['events'])} | {r['active_pending']} |" for r in rows)
    large=next(r for r in rows if r['name']=='four_separated_competing_regions')
    sections={
    'A. Authority verification':'Start HEAD: `2607f8990c00319d74235c78d251f3130a05ec80`. Prior coupled report SHA-256: `ead1450a4697c9828ef9b1456286827c76a4e16ef7305cfe3a7dbffe23cc5d18`. Prior authority SHA-256: `d30a59247ddcba71081f3a7e89e93d1cc752f5d6572fea628db0560236b755d1`. See AUTHORITY.json for current verification and source hashes.',
    'B. Existing production architecture':'All admitted new cases use MixedFrontRunner, native checkpoints, source-keyed columns, the existing trigger ledger, generalized cavity remesh, complete-neighbor closure, one interface exporter, the preserved core routes, and fixed-interface core repair. Automatic remesh and native production authority remain enabled. IMPLEMENTATION_AUDIT.md records generic repairs.',
    'C. Geometry-family design':f'Eight distinct constructions, 24 uniform-scale designs, and three multi-feature designs: {len(rows)} new design records. One intersecting-source wall extraction is deliberately retained as invalid authority evidence. Six preserved controls are separate. Parameter duplicates do not increase the count of eight constructions.\n\n'+table,
    'D. Geometry authorities':'GEOMETRY_FAMILY_MANIFEST.json, PARAMETER_DESIGN.json, MULTIREGION_DESIGN*.json and WALL_AUTHORITIES.json record definitions and wall audits. The executable definitions are geometry_family_authority.py. Gradients are analytic; ellipsoid distance uses deterministic nearest-point secular isolation. Exterior fields are negative inside solids and positive outside. Cavity signs are reversed for inward fluid growth. Min-union ownership is deterministic. Wall coordinates/topology, source identity and stored binary64 schedule are hashed per run. Standard h1 is 0.000166527069 m. No dimension was fitted to a successful mesh; the same 1, 1/2, 1/4 scale sweep was applied to every base construction.',
    'E. Wide-clearance controls':'The quarter-scale capsule completes 20 nominal layers without remesh. Fresh SF2 bounded replay preserves 20 layers, 2,126,040 prisms and all 56,462,864 relations. The preserved wide curved control remains clean. Larger coarse capsule wall samplings can fail fixed first-layer determinant admission; clearance alone does not imply this discretization meets the frozen gate.',
    'F. Strong concavity':'The inward ellipsoid tests converging normals and variable curvature. Its quarter-scale 12-layer prefix passes complete bounded assembly. The requested total offset exceeds its 0.0075 m minimum semi-axis. Rejected continuation and remesh candidates remain recorded; the prefix is not a completed 20-layer case.',
    'G. Opposing surfaces':'Competing ellipsoids use disjoint analytic walls and decreasing future clearance. The native policy records exact collision evidence, local rollback/remesh and continued growth. CASE_SUMMARY.json records the final local layer distribution; no full-stack claim is made for unfinished columns.',
    'H. Thin curved passages':'The bent capsule combines restricted clearance and changing normals. Its frozen extracted walls fail first-layer quality after the existing correction trials. This bounds the tested wall/schedule combination; it does not prove all smooth bent passages are infeasible.',
    'I. Rapid curvature variation':'Rounded-cylinder walls test rapid curvature change. The quarter-scale shell reaches 20 layers. The complete core did not pass the preserved Delaunay/HXT and relocation paths. CORE_ATTEMPTS and hard-quality arrays preserve each failure.',
    'J. Mixed convex/concave geometry':'The torus includes both principal-curvature signs and inner crowding. Its mixed shell finishes requested active schedules after a layer-19 remesh and layer-20 continuation. It contains 4,864 prisms, 64 pyramids and 64 tets. Complete core quality remains unqualified.',
    'K. Multiple nearby features':'The two- and three-ellipsoid designs exercise competing source ownership. The equilateral three-feature design uses the original pair spacing. The earlier intersecting raw-source extraction is retained separately; it is not silently replaced or counted as a qualified wall.',
    'L. Branch-switching / medial-axis cases':'Nearest-feature changes and offset crowding occur in the competing ellipsoid designs. Exact collision evidence may bypass inapplicable ordinary trials through the existing policy. A visited branch-aware policy state is not reported as successful branch correction without an accepted receipt.',
    'M. Saddle geometry':'The torus is the smooth saddle feature class. Its remesh replay is exact: 1c121f92ecb77b6b37c6031bdea58861eac0f804aaed12febbe4110a7a1b9845. The successful shell does not qualify its failed core.',
    'N. Multi-region geometry':f'Four separated competition pairs form the largest new design, with 528 wall vertices and 1,024 wall triangles. The completed shell has {large["shell_cells"]} cells: {large["shell_types"]}. All active schedules finish; {len(large["events"])} remesh events commit and {large["shell_relations"]} shell relations certify. T/X reproduce scientific state, chronological event order, wall and first layer exactly. Actual topology changes touch at most two connected source components per event. Earlier inflated candidate support is preserved separately and was repaired. The final core fails the frozen gates under both preserved routes; this is a partial complete-mesh result. T/X are primary/replay labels; Q/S are preserved defect reproducers. Event seeds, removed cells, local layers and RSS traces are retained.',
    'O. Parameter sweeps':'PARAMETER_RESULTS.json records all 24 scale points. Fixed h1 and coarse wall incidence change the layer-thickness/edge-length relation across the sweep. RESOLUTION_PROBE.json is a numerical-only diagnostic, not a production qualification. Do not interpolate a universal curvature or gap threshold from these points.',
    'P. Shell production behavior':'Per-case TRACE, OPERATIONS, checkpoint envelopes and CASE_SUMMARY distinguish committed prefixes, completed active schedules, terminal columns and failed candidates. A global maximum layer is not completion; every active column must reach its requested schedule.',
    'Q. Correction mechanisms':'Existing bounded tangent and compression kernels receive full native validation. The bounded-qualified rounded box required six local tangent updates in its first layer, followed by full admission. The capsule and rounded cone have nominal first-layer acceptance. Adjusted targets and thickness debt remain scientific state. Ordinary advancement cannot change committed cells. Lower-layer trials retain higher-layer neighboring geometry. MECHANISM_MATRIX separates visited policy states from accepted outcomes.',
    'R. Termination/remesh behavior':f'The primary new runs contain {len(events)} committed remesh events. Largest removed-cell set: {summary["largest_removed_cell_set"]}; largest recorded active cavity: {summary["maximum_active_cavity_vertices"]} vertices. Per-event retained layers and removed-cell IDs are in CASE_SUMMARY. Failed cavity searches are preserved, not accepted with weaker gates.',
    'S. Coupled shell/core validation':'Every accepted mixed candidate passes bounded provisional closure and complete-neighbor checks before commit. Final prism-cap incidence uses the same Foundation determinant definition. Coupled rejection traces and final receipts remain separate. No shell-only receipt substitutes for a complete-neighbor receipt.',
    'T. Outer-interface qualification':'The same source-keyed exporter handles prism, pyramid and tet owners. Successful exports verify orientation, incidence, vertex links, exact coordinates and deterministic connectivity. Interface artifacts include source ancestry, local layers and versions. Invalid or unfinished full cases are not promoted by interface validity alone.',
    'U. Core generation and repair':'Preserved Delaunay and HXT controls are used without geometry-specific sizing. Existing Laplacian, constrained movement and feasibility relocation freeze all interface/farfield vertices. The largest case has 43 determinant, 85 weight and 24 volume-ratio failures in its 5,643-tet Delaunay core. Its 3,212-tet HXT core also fails. All existing relocation methods were tried with the interface fixed. LARGE_REJECTED_CORE_REPLAY.json records exact normalized geometry and quality-extrema replay for both rejected routes. Failed searches do not prove fixed-boundary infeasibility. Core reuse now binds exact interface inputs; unbound prior outputs remain evidence but cannot silently become current authority.',
    'V. Complete-mesh qualification':f'{summary["completed_native_shells"]} new native shells complete their requested active schedules. {len(qualified)} new cases complete their requested shell schedule and bounded assembly. {len(assemblies)} primary assemblies pass when valid prefixes are also counted. Prefix results are explicitly partial. Successful assemblies require frozen quality, ownership, volume accounting and exact new continuous relations; unchanged shell proofs are reused.',
    'W. Foundation qualification where applicable':'The new analytic fixtures use the preserved bounded envelope and have no authoritative full CFD patch controls. No artificial OpenFOAM production domain was invented. Their qualification is bounded complete geometry. Prior SF2 Foundation Mesh OK. authority remains unchanged; no new broad CFD or spatial-convergence claim is made.',
    'X. Trigger false-positive/false-negative audit':'See TRIGGER_AUDIT.json. Truth is limited to admitted controls and direct geometric evidence. First-layer admission failures are not missed remesh triggers. A triggered but rejected cavity is not a missed trigger. Cases without an independent expected-outcome witness remain unclassified in the confusion population.',
    'Y. Mechanism matrix':'MECHANISM_MATRIX.json records policy states visited and actual committed event counts. Correction files record accepted versus rejected searches. This matrix is descriptive and is not a geometry ranking.',
    'Z. Unsupported geometry analysis':'Unsupported labels apply to the frozen design, wall extraction and schedule. Reasons include first-layer hard quality, unavailable inward offset, exhausted qualified cavity continuation, and rejected fixed-interface core quality. The report does not infer shape-wide infeasibility from optimizer exhaustion. Generic implementation defects were repaired and rerun; earlier attempts remain evidence.',
    'AA. Geometric operating envelope':'The three completed bounded designs use these analytic parameters in metres: capsule radius 0.01 and half-length 0.01; rounded-box half parameters (0.01, 0.0075, 0.005) and rounding radius 0.005; rounded-cone height parameter 0.02 and radii (0.01, 0.0025). Each requests the stored 20-layer, 0.020000000131704564 m schedule. These are discrete tested designs, not continuous parameter ranges. Supported complete new evidence is limited to the listed bounded-qualified rows and their exact authorities. Mixed torus and competing-feature states extend shell/remesh evidence but do not imply universal complete-mesh support. Measured dimensions, local schedules and failure records define the tested envelope. No new physical or CFD accuracy claim is supported.',
    'AB. Runtime and memory':f'Largest recorded peak across preserved attempts: {summary["largest_recorded_peak_rss"]} bytes. Largest primary shell-run process-segment peak RSS: {summary["largest_primary_peak_rss"]} bytes. The slowest accepted-step history is {summary["slowest_accepted_step_history"]["case"]}: {summary["slowest_accepted_step_history"]["seconds"]} s. Runtime records include failed attempts. Resumed PERFORMANCE records describe a process segment; TRACE sums describe accepted steps and must not be called a cold end-to-end runtime. Core generation, repair and assembly are timed separately. Large-case RSS is recorded after each commit, before candidate scratch is cleared. MEMORY_TRACE.json joins those measured samples to exact receipt counts and disk bytes. These samples are not a steady-state allocation profile. Candidate scratch is released at each step; native checkpoint storage is disk-backed. No dense global optimizer is constructed. An obsolete rejected-candidate proof graph was removed: the isolated 1,906,807-record probe released 1,353,220,096 RSS bytes; the actual event released 1,268,604,928 bytes and reproduced its committed state exactly. Its single-event replay peak was 7,370,387,456 bytes. This is not a cold full-run memory comparison.',
    'AC. Determinism':'DETERMINISM.json records exact native state comparisons. Remesh replays begin before or between actual transactions; terminal geometry is not a construction input. CORE_DETERMINISM.json records normalized complete core replay where run. Failed designs have deterministic source/wall authority tests but no claimed successful terminal replay.',
    'AD. Tests':'Final current-source qualification: TESTS_QUALIFICATION_FINAL.log has 253 passing Windows-compatible tests; TESTS_QUALIFICATION_WSL.log has 11 passing Linux core tests. FINAL_TEST_COMMAND.json records the command. The Windows collection failure for the Linux resource module is preserved in TESTS_QUALIFICATION_WINDOWS_COLLECTION.log and resolved by running that suite in existing WSL. See TESTS_CURRENT_FINAL.log (187 passed after the local-front and certificate representation repairs), TESTS_CERTIFICATE_FINAL.log (56 passed), TESTS_LOCAL_SUPPORT_FIXED.log (65 passed), and TESTS_WSL_FINAL.log (11 passed), TESTS_EVENT_ORDER.log (44 passed), and TESTS_PROOF_LIFETIME.log (85 passed after the rejected-proof lifetime fix). These suites overlap and must not be added together. TESTS_FINAL_ALL.log preserves the earlier 246-test run. The WSL pytest attempt is separately recorded as unavailable; existing Windows pytest and WSL unittest paths were used. Focused coverage includes source gradients, wall determinism, local layers, terminal suppression, exact root representation, unchanged mathematical authority, certificate reuse, checkpoint release, input-bound core reuse and committed-cell preservation.',
    'AE. Classification':summary['classification']+'. Broad measured evidence exists, but important thin-passage, coarse first-layer and complete-core cases remain outside the qualified envelope. The three successful complete cases do not qualify the whole family.',
    'AF. Supported production claim':'One generic native production architecture handles the successful listed bounded cases while retaining exact gates, wall authority and deterministic replay. It does not qualify universal automatic meshing, every parameter point, or broad CFD accuracy. Native production selection and automatic remeshing remain unchanged.',
    'AG. Recommended next task':'C. Broader production-scale geometry qualification, focused on the measured unsupported wall/core combinations. Preserve the existing CFD and spatial-convergence authority limits.'}
    report='## Decision question\n\nAcross a broader family of SDF geometries, which geometric feature classes can the current V4 production architecture qualify without changing scientific gates or adding geometry-specific heuristics?\n\n'+'\n\n'.join('### '+k+'\n\n'+v for k,v in sections.items())+'\n'
    (OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')


if __name__=='__main__':run()
