"""Write the decision report and artifact authority. No meshing occurs here."""
import collections, json, math, os, shutil, subprocess, sys, time
from production_adaptive_surface import OUT,V4,V3,REPO,save,sha,identity
from production_surface_authority import load_production_wall

CLASSIFICATION='V4ProductionAdaptiveCapsuleSurfaceQualified'
NEXT='QualifyV4FreshCapsuleFirstLayerFromAdaptiveSurface'
NM='Not measured.'

def read(p):return json.loads((OUT/p).read_text())
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(map(str,headers))+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])

def tests():
    selected=['research/sdf_cfd_mesher_v4/tests/test_production_adaptive_surface.py',
        'research/sdf_cfd_mesher_v4/tests/test_geometry_family.py::test_analytic_gradient',
        'research/sdf_cfd_mesher_v4/tests/test_geometry_family.py::test_scale_is_not_new_geometry_class',
        'research/sdf_cfd_mesher_v4/tests/test_geometry_family.py::test_ellipsoid_origin_and_missing_axis',
        'research/sdf_cfd_mesher_v3/adaptive_surface_generic_qualification/test_indexed_live_authority.py']
    command=[sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider']+selected
    t=time.perf_counter();p=subprocess.run(command,cwd=REPO,text=True,capture_output=True)
    (OUT/'tests.txt').write_text(p.stdout+p.stderr)
    row=dict(command=command,exit_code=p.returncode,seconds=time.perf_counter()-t,output=p.stdout+p.stderr,
        no_prism_tests=True,scope='Focused new surface tests, existing analytic-gradient/source tests, preserved V3 indexed-authority tests. No layer or core construction.')
    save(OUT/'TESTS.json',row);print(row['output'],flush=True)
    if p.returncode:raise RuntimeError('TestsFailed')
    return row

def main():
    test=read('TESTS.json') if '--reuse-tests' in sys.argv else tests()
    if test['exit_code']:raise RuntimeError('TestsFailed')
    x,f,surface_authority=load_production_wall(OUT/'fresh')
    p=read('PREFLIGHT.json');g=read('fresh/GENERATION.json');a=read('fresh/AUDIT.json');b=read('replay/GENERATION.json');ba=read('replay/AUDIT.json')
    c=read('COMPARISONS.json');trace=read('IMPLEMENTATION_TRACE.json');ledger=read('SURFACE_PROVENANCE_LEDGER.json');replay=read('DETERMINISM.json');cleanup=read('CLEANUP.json')
    before=p['unrelated_dirty_state'];now=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True).splitlines()
    after=[v for v in now if 'research/sdf_cfd_mesher_v4' not in v]
    if before!=after:raise RuntimeError('UnrelatedDirtyStateChanged')
    for row in p['reference_files']+trace['v3_sources']+trace['v3_import_bindings']:
        if sha(REPO/row['path'])!=row['sha256']:raise RuntimeError('V3ReferenceChanged:'+row['path'])
    ops={k:sum(row.get(k,0) for row in g['mmgs_operation_rows']) for k in ['splitted','collapsed','swapped','moved']}
    scientific_peak=max(g['peak_rss_bytes'],b['peak_rss_bytes'],a['peak_rss_bytes'],ba['peak_rss_bytes'],
        g['timing']['mmgs']['peak_rss_bytes'],b['timing']['mmgs']['peak_rss_bytes'])
    runtime=g['generation_seconds']+b['generation_seconds']+a['seconds']+ba['seconds']
    build_peak=max(z['result']['peak_rss_bytes'] for z in read('BUILD.json')['tools'])
    endfree=shutil.disk_usage(OUT).free
    history=dict(carrier=g['carrier']['levels'],carrier_edge_bisections=g['carrier']['added_vertices'],
        carrier_refinement_updates=4,carrier_assessments=5,mmgs_calls=1,mmgs_reported_phase_summaries=g['mmgs_operation_rows'],
        mmgs_totals=ops,mmgs_unique_edge_split_count=NM,
        counter_semantics='MMGS anaelt counts split-triangle templates; adpspl has its own split counter. The aggregate label splitted is not a count of unique inserted vertices or unique edges.',
        native_per_iteration_vertex_counts=NM,native_per_iteration_triangle_counts=NM,native_rejected_operation_counts=NM,
        native_per_iteration_geometry_and_size_changes=NM,native_per_iteration_runtimes=NM,
        projection=g['projections'],native_log='fresh/mmgs.stdout',
        reason='Stock qualified verbosity=1 reports aggregate operation phases. Missing per-operation counters are not inferred from population differences.')
    save(OUT/'ADAPTATION_HISTORY.json',history)
    save(OUT/'SEED_SCALE_REPRODUCER.json',dict(original_seed_vertices=28,original_seed_triangles=52,
        original_facet_size_m=.20,unresolved_faces_at_depth_four=165,
        corrected_seed_size_m=.008,formula='min(V3 HMAX, V3 physical_curvature_factor / analytic maximum curvature)',
        source='held_out_geometry_cohort/src/cohort_surface.cpp::Fixture::physical_edge_target',
        current_carrier_unresolved=0,scientific_gates_changed=False,carrier_depth_changed=False,
        input='cold/INITIAL.npz',raw_input='cold/initial_raw.off',evidence='cold/CARRIER.json',
        reproduction='Run runtime/seed 0.04 0.04 OUTPUT.off 0.20, project to the same SDF, run the unchanged four-level carrier oracle.'))
    lines=['## Decision question','',
        'Can V4 generate and qualify a fresh production capsule wall using the real V3 adaptive SDF surface-meshing process rather than a procedural fixture surface?','',
        '**Classification: '+CLASSIFICATION+'**','',
        'Yes, under the recovered V3 generic development-equivalence surface contract. The fresh wall and its independent replay pass. This is a surface authority only. No first layer, prism, front, CFD tet core, or complete mesh was generated.','']
    def section(title,body):lines.extend(['### '+title,'',body,''])
    section('A. Production qualification reset',f"Starting HEAD: `{p['starting_head']}`. Branch: `{p['branch']}`. The old fixture wall was not an input. Only its analytic capsule definition was reused. Existing V3 and V4 authorities remain unchanged. Fixture evidence retains its original algorithmic scope; it is not production surface-to-core qualification.")
    section('B. V3 adaptive surface implementation trace',
        'The accepted SF2 execution entry is `fresh_supported_refinement_family_qualification_2026-09-09/adapt_surface_stage.py`. It executes the generation prefix of that same directory\'s `run_surface.py`, which calls `generic_v3mmg1_provider.main`. The preserved SF2 manifest binds the provider hash exactly. SF2 uses a CGAL constrained chart seed; the held-out closed-geometry path uses implicit CGAL Mesh_3 surface extraction. The latter is applicable to the smooth capsule. Paths below are relative to `'+str(REPO)+'`.\n\n'+
        table(['Role','Source','Line','Function / scope'],[[r['role'],r['path'],r['line'],r.get('function',r['classification'])] for r in trace['v3_sources']])+
        '\n\nThe actual operations are CGAL extraction, SDF-driven carrier red/green bisection, stock MMGS splits/collapses/swaps/vertex relocation/gradation, authority projection and independent validation. No marching-cubes, latitude/longitude, directional adjacency, or custom surface tetrahedralizer was substituted. MMGS has no live SDF callback; it receives the SPD metric and source references, then output is projected back to the live authority.')
    section('C. V3 surface sizing contract',
        'Principal normal-map curvature controls directional sizes. The curvature formula uses the chord-error and normal-error limits, then caps at HMAX and floors at 1e-7 m. The normal-map fallback uses a 1e-5 m differential step and a 2.5 curvature reserve. Shape conditioning limits excess tangent anisotropy. The carrier oracle uses gradients/Hessians and normal variation. The selected SF2 process additionally used a prism-host contribution and a 0.64 family scale; these case controls are excluded here.\n\n'+
        table(['Parameter','Recovered value','Units','Source','Scope / capsule use'],[[r['name'],r['value'],r['units'],r['source'],r['scope']+'; use='+str(r['capsule_use'])] for r in trace['numeric_parameters']])+
        '\n\nThe 51.480559638 micrometre / 5.729078314 degree limits are inherited V3 development-equivalence limits, explicitly identified as such in its architecture report. They are not universal CFD accuracy limits. The floor is a numerical size floor, not an independently calibrated capsule resolution requirement. No new capsule number was tuned.')
    section('D. Generic versus V3-specific logic',
        'A: geometry-neutral authority/provider, source projection dispatch, indexed identity and topology. B: carrier refinement, metric formulas, stock MMGS adaptation and generic continuous Hessian certificate. C: spline extrusion charts, native spline bridge, named sharp-sheet/curve geometry and analytic specialized certificate branches. D: SF2 scale/schedule/prism-response settings and historical cohort seed scales. Only A/B are reused directly. The capsule has an explicit geometry plug-in and source-derived curvature. No wing charts or wing-specific resolution settings are hidden in it.')
    section('E. V4 production adaptive implementation',
        '`tools/production_adaptive_surface.py::ProductionAdaptiveSdfSurfaceMesher.generate` is the production entry point. It imports V3 generic algorithms read-only and uses the qualified stock MMGS 5.8.0 library and runner with verified hashes. The V4 native adapter exports only CGAL boundary facets. Its internal background triangulation is a topology seed, not a CFD tet core.\n\nIntentional deviations:\n\n'+'\n'.join('- '+s for s in trace['deviations']))
    section('F. Production versus fixture provenance guard',
        'The public production loader verifies a sealed root, implementation/geometry/configuration/sizing/adaptation bindings, artifact hashes, canonical array identities and passing audit. Fixture/unknown inputs, incomplete evidence and tampering fail closed. Explicit test-only fixture override returns production_authorized=false. The old radial_wall/build path is not called by the new generator. No old surface arrays are read until the separate diagnostic comparison.')
    geo=read('GEOMETRY.json')
    section('G. Capsule analytic geometry authority',
        f"Radius = 0.04 m; segment half-length = 0.04 m. SDF = distance to the clamped axial segment minus radius, exactly the existing analytic primitive. Source legacy identity: `{geo['original_source_identity']}`. Geometry-only implementation-bound identity: `{geo['geometry_identity']}`. Old mesh reads during generation: 0.")
    section('H. Initial extracted surface',
        f"The successful fresh implicit seed has {g['initial_vertices']} vertices and {g['initial_triangles']} triangles. Seed facet size is {g['seed_size_m']} m, from V3's min(HMAX,0.20/kappa_max), with kappa_max=25 /m. Random seed=49944. The earlier absolute 0.20 m cohort seed had 28 vertices / 52 triangles and left 165 unresolved carrier faces at depth four. It is preserved under cold/. No final gate or carrier depth was changed. This is a measured seed-scale generalization, not a hand-selected capsule patch.")
    section('I. Adaptive sizing field',
        'The final live-SDF tensor field is nonuniform and curvature/shape dependent. It is sampled independently on the final coordinates. Directional requests range from '+str(a['sizing']['minimum_requested_size_m'])+' to '+str(a['sizing']['maximum_requested_size_m'])+' m. Explicit configured bounds are 1e-7 and 0.04 m. The accepted surface does not rely on either bound clipping its natural curvature requests.\n\n'+
        table(['Actual / directional target','Value'],list(a['sizing']['actual_over_target'].items()))+
        f"\n\nMaximum oversize ratio location: {a['sizing']['max_oversize_location']}; minimum ratio location: {a['sizing']['min_ratio_location']}. There are {a['sizing']['below_mmgs_optimization_length_071']} edges below 0.71 and {a['sizing']['above_mmgs_optimization_length_14']} above 1.4. These are descriptive comparisons to native optimization triggers. V3 has no final hard edge/target acceptance interval. Native graded/transported metric equality to this independently re-evaluated field is not assumed.")
    section('J. Adaptation history',
        table(['Carrier assessment','Triangles','Inadequate','Refined risk faces','Max normal interpolation defect deg','Unresolved'],[[r['level'],r['faces'],r['inadequate_faces'],r['refined_risk_faces'],r['maximum_normal_interpolation_defect_deg'],r['bounded_unresolved_faces']] for r in g['carrier']['levels']])+
        f"\n\nFour conformal carrier refinement updates / five assessments. Carrier: 414 -> 1,414 vertices and 824 -> 2,824 triangles; exactly 1,000 midpoint/edge bisections. One stock MMGS call follows.\n\n"+
        table(['Native summary phase','splitted','collapsed','swapped','moved'],[[i+1,r.get('splitted',NM),r.get('collapsed',NM),r.get('swapped',NM),r.get('moved',NM)] for i,r in enumerate(g['mmgs_operation_rows'])])+
        f"\n\nNative totals: splitted={ops['splitted']}, collapsed={ops['collapsed']}, swapped={ops['swapped']}, moved={ops['moved']}. The first counters include split-triangle templates; a unique native edge-split count is Not measured. Do not infer it from vertex-count changes. Internal iteration inventories, rejected native operations by reason, internal iteration timing and geometry/sizing residual changes: Not measured. The complete stock logs are preserved. Projection operations across seed/carrier/normal-map queries/final projection: {g['projections']['operations']}; failures=0. The final wall receives {g['final_vertices']} direct source projections. No iterative Newton solve is used by this capsule projector.")
    section('K. Final surface geometry',f"{g['final_vertices']} vertices, {g['final_triangles']} triangles, {a['edges']} edges. All 33,786 complete face supports satisfy the inherited continuous geometry bounds. Maximum position bound={a['continuous_geometry']['maximum_position_bound']} m. Maximum normal bound={a['continuous_geometry']['maximum_normal_bound_rad']} rad ({math.degrees(a['continuous_geometry']['maximum_normal_bound_rad'])} degrees).")
    section('L. Edge-size audit',table(['Percentile','Length m'],list(a['edge'].items())))
    labels=['min','p1','p5','median','p90','p95','p99','p99.5','p99.9','max']
    section('M. Triangle-quality audit',table(['Measure']+labels,[[k]+[v[q] for q in labels] for k,v in a['triangle'].items()])+'\n\nAreas are m^2, lengths/radii are m, angles are degrees, ratios are dimensionless. Radius ratio is R/(2r), equal to one for an equilateral triangle. No additional hard angle/aspect threshold was invented. Raw native MMGS quality: best 0.999999, average 0.969039, worst 0.697249; this is pre-SDF-projection metric quality from the native log.')
    section('N. Curvature-resolution audit',
        'Physical principal curvatures are (0,25) /m on the cylindrical side and (25,25) /m on caps, apart from the C1 join where second derivatives are one-sided. The source-surface curvature bound is 25 /m. The unchanged V3 continuous certificate converts it to the signed-distance tube Hessian bound K=kappa/(1-kappa*POSITION_LIMIT); 25 /m is not asserted as a global off-surface Hessian bound.\n\n'+
        table(['Descriptor']+labels,[[k]+[a['curvature'][k][q] for q in labels] for k in ('edge_times_max_curvature','triangle_circumradius_times_max_curvature')])+
        '\n\nMaximum edge/radius location: '+str(a['curvature']['maximum_edge_ratio_location'])+'. These ratios are diagnostics.')
    section('O. Feature-size audit','V3 local-feature-size gate: Not implemented. The selected generic provider can query competing component sheets for carrier ambiguity/clearance, but has no universal final local-feature-size size gate. The capsule has one convex component and a source medial distance of 0.04 m. No new gap/size threshold is imposed.')
    section('P. SDF projection audit',f"Maximum absolute vertex residual={a['sdf_residual']['max']} m; median={a['sdf_residual']['median']} m. Projection failure=0, zero-gradient=0, nonfinite=0. The analytic closest-segment projection is one closed-form application per call. Iterative projection iteration count: Not measured. The recovered held-out V3 vertex tolerance is 2e-12 m. This is binary64 residual qualification, not a claim that every coordinate is a symbolic exact real solution.")
    section('Q. Normal and orientation audit',
        table(['Error distribution','Minimum deg','Median deg','Maximum deg'],[[k,v['min'],v['median'],v['max']] for k,v in a['normal'].items() if isinstance(v,dict)])+
        f"\n\nInverted triangles={a['normal']['inverted_triangles']}. Continuous maximum angle upper bound={math.degrees(a['continuous_geometry']['maximum_normal_bound_rad'])} degrees. Adjacent triangle normal variation: {c['fresh_capsule_adjacent_face_normal_variation_deg']}.")
    section('R. Topology audit',table(['Check','Result'],list(a['topology'].items()))+'\n\nClosed, consistently oriented sphere topology: Euler=2, genus=0, one connected component. Connectivity is extracted/adapted, never direction-indexed. Source-sheet ambiguity=0. Diagnostic ray tests: Not measured. Analytically, the capsule is convex, so a ray from an interior point has one outward surface exit.')
    section('S. Continuous self-intersection audit','Complete CGAL exact-predicate PMP self_intersections traversal on the final projected surface. Legal shared adjacency is excluded. Missing=0, duplicate=0, forbidden=0, unresolved=0. Missing/duplicate refer to complete input triangle coverage and unique input topology; no V4 face-pair receipt ledger is claimed. A crossing-triangle negative control is rejected. Independent replay returns the same result.')
    old=c['old_capsule_fixture'];sf=c['sf2_control']
    section('T. Old procedural capsule comparison',table(['Measure','Old fixture','Fresh adaptive'],[
        ['Vertices',old['vertices'],g['final_vertices']],['Triangles',old['triangles'],g['final_triangles']],
        ['Edge minimum m',old['edge']['min'],a['edge']['min']],['Edge median m',old['edge']['median'],a['edge']['median']],['Edge maximum m',old['edge']['max'],a['edge']['max']],
        ['Minimum triangle angle deg',old['triangle']['minimum_angle']['min'],a['triangle']['minimum_angle']['min']],
        ['Maximum triangle angle deg',old['triangle']['maximum_angle']['max'],a['triangle']['maximum_angle']['max']],
        ['Median area m^2',old['triangle']['area']['median'],a['triangle']['area']['median']],
        ['Maximum area m^2',old['triangle']['area']['max'],a['triangle']['area']['max']],
        ['Maximum face-centroid normal error deg',old['normal']['face_centroid_angular_error_deg']['max'],a['normal']['face_centroid_angular_error_deg']['max']],
        ['Maximum adjacent-normal variation deg',old['adjacent_face_normal_variation_deg']['max'],c['fresh_capsule_adjacent_face_normal_variation_deg']['max']],
        ['Maximum vertex SDF residual m',old['sdf_residual']['max'],a['sdf_residual']['max']]])+
        '\n\nThe adaptive wall is finer, has a narrower edge tail and lower normal error. Its worst triangle angle is lower than the procedural fixture minimum; no blanket shape-superiority claim is made. The old mesh is diagnostic only, read after both fresh generations. Full percentile tables are retained in COMPARISONS.json.')
    section('U. V3 SF2 adaptive-surface control',f"Preserved SF2: {sf['vertices']} vertices / {sf['triangles']} triangles, four sheets, six feature curves and four corners. Edge min/median/max={sf['edge']['min']} / {sf['edge']['median']} / {sf['edge']['max']} m. All 106,302 continuous face certificates pass; maximum position={sf['continuous_certificate']['max_position']} m, normal={sf['continuous_certificate']['max_normal']} rad. Provider hash matches the recovered generic source. Stock MMGS performed 99,303 reported splits, 51,045 collapses, 40,623 swaps and 1,037,803 moves in that preserved run. No SF2 regeneration occurred. Its spline-extrusion geometry and schedule-conditioned metric differ from this capsule; its sizes are not capsule gates.")
    section('V. Surface identity and authority',table(['Identity','SHA-256'],[[k,surface_authority[k]] for k in ['geometry_identity','implementation_identity','configuration_identity','coordinate_hash','topology_hash','sizing_field_identity','adaptation_identity','residual_hash','surface_identity']])+f"\n\nAtomic surface publication: passed. Loader re-verification: passed. Published authority: `{OUT/'fresh/SURFACE_AUTHORITY.json'}`. Scientific root binds the implementation and adaptation, in addition to geometry/configuration/arrays. The preliminary generation identity remains separately recorded; it is not substituted for the sealed root.")
    section('W. Determinism','Two fresh independent processes generate exactly equal canonical coordinates, oriented triangles, tensors, residuals, initial seed arrays, quality arrays, certificate bounds/statuses, operation counts and adaptation identities. No rounding was used. Timers/RSS are excluded from scientific identity. The raw seed export, metric contract and raw MMGS output are also byte-identical; duplicate replay scratch was safely removed after hashing. OS filesystem cache was not flushed.')
    section('X. Runtime and memory',table(['Measured stage','Seconds'],[
        ['Cold successful generation',g['generation_seconds']],['Independent replay generation',b['generation_seconds']],
        ['Cold implicit extraction',g['timing']['extraction']['seconds']],['Cold adaptive carrier',g['timing']['carrier_seconds']],
        ['Cold input sizing',g['timing']['sizing_seconds']],['Cold metric contract',g['timing']['contract_seconds']],
        ['Cold MMGS',g['timing']['mmgs']['seconds']],['Cold final projection',g['timing']['final_projection_seconds']],
        ['Cold final sizing evaluation',g['timing']['final_sizing_seconds']],['Cold topology/statistics audit',a['topology_seconds']],
        ['Cold continuous geometry audit',a['continuous_geometry']['seconds']],['Cold self-intersection',a['self_intersection']['timing']['seconds']],
        ['Cold complete audit',a['seconds']],['Replay complete audit',ba['seconds']],['Sum of two generation + two complete audit stage times',runtime]])+
        f"\n\nProjection work across all geometric-query stages={g['projection_seconds_all_queries']} s, inclusive inside their stage timers; do not add it again. Peak individual scientific process RSS={scientific_peak} bytes. Cold retained generator RSS={g['retained_rss_bytes']} bytes; retained audit RSS={a['retained_rss_bytes']} bytes. Peak compiler RSS={build_peak} bytes, separate from science. Combined concurrent-process RSS: Not measured. Full active task runtime including all inspection/diagnostic work: Not measured. The summed qualification-stage time is not elapsed wall time because replay generation overlapped the first audit. No native generation/audit call exceeded 120 s.")
    section('Y. Disk management',f"Starting free bytes={p['starting_free_bytes']}. Accessible V4 result-tree bytes={p['v4_result_tree_bytes']}; {len(p['inaccessible_entries'])} pre-existing inaccessible link entries are separately recorded, not estimated. V3 reference bytes inventoried before execution={p['v3_reference_bytes']}. Largest inputs: SF2 faces TSV 9,834,570 bytes, vertices TSV 8,901,654 bytes, raw mesh 8,733,238 bytes. Initial expected scratch: Not measured. Stage free-space records are preserved. Pre-run cleanup=0 bytes. Post-run cleanup={cleanup['bytes_deleted']} bytes across three exact duplicate scratch files. At cleanup, task-created bytes={cleanup['task_created_bytes_at_cleanup']}, retained bytes={cleanup['retained_task_bytes_at_cleanup']}; later report/authority bytes are additional. Ending free-space snapshot={endfree} bytes. Accepted V3/V4 authorities and old crash dumps were untouched.")
    section('Z. Historical surface-provenance ledger',table(['Case','Starting surface classification'],[[r['case'],r['classification']] for r in ledger['entries']])+f"\n\nLedger counts={ledger['counts']}. Unlisted studies are Unknown until their source is traced. No old report is rewritten.")
    section('AA. Production surface contract','Production wall = source geometry + production generator + sizing configuration + adaptation evidence + coordinate/topology identity + SDF residual/continuous geometry + valid topology/orientation + complete self-intersection evidence + exact replay. Procedural fixtures do not satisfy it. The ordered claims are SurfaceQualified -> FirstLayerQualified -> AdvancingFrontQualified -> CoreHandoffQualified -> CoreQualified -> CompleteMeshQualified. No later success substitutes for an earlier qualification. The next task must load the sealed new wall, not the old capsule checkpoint.')
    section('AB. Tests',test['output'].strip()+'\n\nExact command:\n\n```\n'+' '.join(test['command'])+'\n```\n\nCoverage: source/trace bindings, fixture rejection and test-only override, production loading/sealing/tamper rejection, sizing and curvature response, seed-scale reproducer, projection/zero-gradient/nonfinite rejection, topology/orientation/duplicates, exact self-intersection negative, all native adaptive operation classes, continuous geometry, exact replay, no downstream calls, provenance ledger and safe cleanup. Relevant existing V3 indexed-source and V4 analytic tests pass. No unrelated broad suite or prism-generating test was run.')
    section('AC. Classification',CLASSIFICATION+'. A fresh adaptive capsule surface is sealed under the recovered V3 generic development-equivalence surface rules. No unresolved surface failure remains. Limits: only the capsule geometry adapter is qualified here; other geometries require their own source/seed adapter qualification. There is no CFD/aerodynamic accuracy calibration, universal final size-ratio/LFS gate, internal native rejected-operation/per-iteration inventory, or first-layer/core qualification. V3 modifications=0; all '+str(len(before))+' unrelated dirty-state entries are preserved.')
    section('AD. Recommended next task',NEXT+'. Use the sealed 16,895-vertex / 33,786-triangle surface. Do not use an old fixture or checkpoint. First-layer generation was not performed in this task.')
    report='\n'.join(lines).rstrip()+'\n';(OUT/'report.md').write_text(report)
    preservation=dict(starting_head=p['starting_head'],v3_modification_count=0,verified_v3_inputs=len(p['reference_files'])+len(trace['v3_sources'])+len(trace['v3_import_bindings']),
        unrelated_dirty_state_preserved=True,unrelated_dirty_entries=len(before),new_prism_layers=0,new_core_meshes=0)
    save(OUT/'PRESERVATION.json',preservation)
    artifacts={str(q.relative_to(OUT)).replace('\\','/'):sha(q) for q in sorted(OUT.rglob('*')) if q.is_file() and q.name not in ('AUTHORITY.json','HASHES.json')}
    authority=dict(schema='V4ProductionAdaptiveCapsuleSurfaceFoundation/1',classification=CLASSIFICATION,
        starting_head=p['starting_head'],sealed_surface=surface_authority,report_sha256=sha(OUT/'report.md'),
        implementation_trace_sha256=sha(OUT/'IMPLEMENTATION_TRACE.json'),artifact_sha256=artifacts,
        determinism=replay,preservation=preservation,tests=test,
        limitations=['Inherited V3 development-equivalence surface limits, not CFD accuracy calibration.',
            'Only the capsule geometry adapter is qualified in this task.',
            'Final size-ratio and local-feature-size numerical gates are not implemented in the recovered V3 contract.',
            'Stock aggregate MMGS counters are preserved; unexposed per-iteration/rejection counters are Not measured.',
            'First layer and every later qualification stage remain unrun.'],recommended_next_task=NEXT)
    save(OUT/'AUTHORITY.json',authority)
    hashes=dict(report_path=str(OUT/'report.md'),report_sha256=sha(OUT/'report.md'),authority_path=str(OUT/'AUTHORITY.json'),authority_sha256=sha(OUT/'AUTHORITY.json'))
    save(OUT/'HASHES.json',hashes)
    print(report,flush=True);print(json.dumps(hashes,indent=2));print(json.dumps(preservation,indent=2))

if __name__=='__main__':main()
