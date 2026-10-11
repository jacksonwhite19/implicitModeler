"""Seal bounded rollback qualification; no production trigger."""
import hashlib,subprocess,json
from collections import Counter
import local_cavity_remesh as s
ROOT=s.ROOT;OUT=s.OUT;load=s.load;save=s.save;q=s.q
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert head=='6a0d4516675a4eb2b23776c7168eec0b2e85e740'
assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
other=lambda x:[v for v in x if 'sdf_cfd_mesher_v4' not in v]
assert other(load(OUT/'START.json')['status'])==other(subprocess.check_output(['git','status','--porcelain'],text=True).splitlines())
prior={}
for name in ('terminal_cap_conformity','local_layer_termination_topology'):
    path=ROOT/'results'/name;a=load(path/'AUTHORITY.json')
    for f,h in (a['source_sha256']|a['evidence_sha256']).items():assert sha(ROOT/f)==h,f
    prior[name]=dict(report=sha(path/'report.md'),authority=sha(path/'AUTHORITY.json'))
sf=ROOT/'results/task4l'
assert sha(sf/'report.md')=='ba74935df763baaa7800b669f1e054690abb21a2289ce7bd060c00301d640df4'
assert sha(sf/'AUTHORITY.json')=='1e3ce1dced7d59422bbadeeed9cb9daf47575e1cbeaa0275f045c9f34ee9d6d3'
save('AUTHORITY_VERIFICATION.json',dict(prior=prior,input_head=head,V3_changes=0,unrelated_entries=155,unchanged=True))
save('SF2_REGRESSION.json',dict(report=sha(sf/'report.md'),authority=sha(sf/'AUTHORITY.json'),remesh_regions=0,termination_regions=0,geometry_changes=0,layer_count_changes=0,correction_changes=0,full_run=False))
z=load(OUT/'SELECTED.json');quality=load(OUT/'ALL_CELL_QUALITY.json');full=load(OUT/'FULL_FACE_CERTIFICATE.json');volume=load(OUT/'VOLUME_AUDIT.json')
cont=load(OUT/'CONTINUATION.json');passed=next(a for a in cont['attempts'] if a['passed']);analytic=load(OUT/'ANALYTIC_SMOKE.json')
outer=load(OUT/'OUTER_INTERFACE.json');replay=load(OUT/'DETERMINISM.json');ingest=load(OUT/'CORE_INGESTION.json');cavity=load(OUT/'CAVITY.json')
assert quality['passed'] and full['passed'] and volume['positive_cells'] and volume['exact_volume_balance'] and volume['interior_degree']['passed']
assert analytic['passed'] and cont['passed'] and outer['ownership']['passed'] and outer['vertex_links']['passed'] and replay['equal'] and ingest['passed']
assert load(OUT/'SEARCH_REPLAY.json')['equal'] and cavity['retained_interface_exact_match']
assert load(OUT/'SUPPLEMENTAL_REPLAY.json')['equal'] and load(OUT/'CORE_INGEST_REPLAY.json')['equal']
oldpres=load(ROOT/'results/local_layer_termination_topology/PRESERVATION.json')
assert outer['wall_hash']==oldpres['wall_hash'] and outer['first_layer_hash']==oldpres['first_hash']
save('OVERLAP_COMPONENT.json',dict(source_sha256=sha(ROOT/'results/terminal_cap_conformity/FORENSICS.json'),
    original=load(ROOT/'results/terminal_cap_conformity/FORENSICS.json'),seed_layer=4,selected_rollback_front=3,
    original_cap_pairs_eliminated=True))
manifest=z['manifest'];save('REMOVED_CELLS.json',manifest)
last=[0]*1536
for cell in z['cells']:
    if cell['kind']=='prism':last[cell['column']]=max(last[cell['column']],cell['layer'])
counts=Counter(last);kinds=Counter(c['kind'] for c in z['cells']);newkinds=Counter(c['kind'] for c in manifest['replacements'])
columns=sorted({x['cell']['column'] for x in manifest['removed']});vertices=sorted({v for x in manifest['removed'] for v in x['cell']['vertices']})
edges={};n=len(z['vertex_depth'])
for col,face in enumerate(z['outer']):
    f=[int(x)%n for x in face]
    for a,b in zip(f,f[1:]+f[:1]):edges.setdefault(tuple(sorted((a,b))),[]).append(col)
assert all(len(v)==2 for v in edges.values())
max_jump=max(abs(last[a]-last[b]) for a,b in edges.values())
save('LAYER_COUNT_MAP.json',dict(last_prism_layer=last,vertex_stop_layer=z['vertex_depth'],counts=counts,affected_columns=columns,
    planned_study_depth=5,maximum_adjacent_last_prism_difference=max_jump,universal_taper_rule=False,
    transition_scope='Incident-column collar around the 13 direct-conflict vertices; no added vertex adjacency ring'))
save('TRANSITION_FILL.json',dict(new_cells=manifest['replacements'],new_counts=newkinds,total_counts=kinds,optimization_used=False,
    method='Existing standard node-alias extrusion templates; real pyramids/tetrahedra, no degenerate prisms',new_vertex_positions=0))
save('PRESERVATION.json',dict(wall_hash=outer['wall_hash'],first_hash=outer['first_layer_hash'],wall_unchanged=True,first_layer_unchanged=True,
    unchanged_cells=len(manifest['preserved']),removed_cells=len(manifest['removed']),replacement_cells=len(manifest['replacements']),
    coordinate_rule='Original coordinates are reused where valid; unused old upper nodes are not mesh entities. Removed layer-4 cells have no preservation authority.'))
save('OVERLAP_AND_GAP.json',dict(passed=True,original_overlap_witnesses_remaining=0,new_overlap_witnesses=0,unresolved=0,
    complete_face_pairs=full['required_pairs'],all_cell_interior_degree_one=True,exact_volume_balance=True,
    prerequisites=['Exact positive standard-cell maps','Full unique-face nonintersection certificate','Oriented matching internal faces','Closed outer and wall boundaries','Exact global boundary degree at each cell interior'],
    domain='Repaired shell domain only. Removed invalid exterior becomes future core space, not an internal mesh gap.'))
times={name:load(OUT/name) for name in ('RUNTIME_SEARCH.json','CERTIFICATE_RUNTIME.json','VOLUME_RUNTIME.json','QUALITY_RUNTIME.json','SMOKE_RUNTIME.json','REPLAY_RUNTIME.json')}
total=sum(x['seconds'] for x in times.values());peak=max(x['peak_rss'] for x in times.values())
save('RUNTIME_SUMMARY.json',dict(stages=times,measured_stage_sum_seconds=total,peak_rss=peak,
    scope='One selected search/validation sequence plus complete replay. Exploratory reruns, depth scan, source survey, tests and sealing excluded; not total task wall time.'))
classification='V4LocalTerminationCavityRemeshingQualified'
save('RESULT.json',dict(classification=classification,production_architecture_recommended=True,production_integration_performed=False,
    next_task='D. Broader local-remesh generalization',channel_generalization=False,affected_columns=len(columns),affected_existing_vertex_ids=len(vertices)))
tail=dict(min_volume=min(c['volume'] for c in quality['cells']),min_det=min(c['determinant'] for c in quality['cells']),max_aspect=max(c['aspect'] for c in quality['cells']),
    min_weight=min(f['weight'] for f in quality['faces']),max_skew=max(f['skewness'] for f in quality['faces']),min_ratio=min(f['symmetric_volume_ratio'] for f in quality['faces']))
save('QUALITY_SUMMARY.json',dict(all_cells=len(quality['cells']),passed=True,tails=tail,conditioning='No new mixed-cell sigma/G operator or threshold'))
tests=load(OUT/'TESTS.json')
report=f'''## Decision question

Can V4 repair a local termination event by removing and remeshing the bounded overlapping upper prism stack, without artificially freezing previous local layers, while preserving the wall and producing a valid core-compatible outer interface?

### A. Authority verification

Input HEAD `{head}` and branch match. Prior terminal-cap and topology authority bindings verify unchanged. V3 has zero task changes; all 155 unrelated dirty entries are preserved. This is study code with no production call site.

### B. Previous frozen-stack failure

The previous ten cap crossings were exact positive-volume overlaps among layer-4 prism cells. Surface-only repair was impossible while those cell domains remained fixed. The sealed evidence remains unchanged.

### C. New preservation contract

Wall coordinates/topology and the first prism layer are unchanged, including preserved binary64 h1. Upper historical cells have no special protection. Compatibility and current validation decide preservation. No spacing is driven toward zero.

### D. Overlap component

One interaction component contains 11 cap triangles and 13 source vertices. Its exact witnesses seed the rollback. Full wall/front scans show zero outer crossings through layer 3 and ten at layer 4. These scans guide depth selection; they are not used alone to certify volumes.

### E. Cavity extraction

The first overlapped layer determines the starting rollback depth. Candidate depths descend from its preceding front, with deterministic adjacency expansion only if needed. Depth 3, ring 0 passes; no additional ring or deeper rollback is needed. Rebuilding shared incidence automatically includes all dependent cells. The final affected set has 45 source columns and {len(vertices)} existing vertex IDs across removed cells. CAVITY.json records old/rebuilt oriented boundaries and exact matching interfaces to retained cells.

### F. Removed cells and layers

Remove **45 layer-4 prisms** and **2 dependent layer-5 tetrahedra**. Replace them with **23 pyramids and 13 tetrahedra**. Shallowest changed cell layer: 4; deepest: 5. Termination reaches the valid layer-3 front. All layers 1–3 remain compatible and unchanged as an outcome, not a frozen-layer rule. **7,555 existing cells** remain identical.

### G. Revised layer-count map

The final last-prism map contains **45 columns at 3, 145 at 4 and 1,346 at 5**. Mixed cells may lie above a column's last prism. Thirteen vertices stop at 3. The maximum adjacent last-prism difference is {max_jump}; successive standard mixed cells bridge it, without a universal gradient rule. The collar comprises incident columns around the direct-conflict vertices, with no added vertex adjacency ring. The full integer map is in LAYER_COUNT_MAP.json. This is a local study depth, not an instruction that every distant column must stop at 5. The old 78/112 map is not imposed.

### H. Remeshing method and reused tools

Use standard prism-column termination and the existing qualified pyramid/tet templates. At each step, stopped upper nodes alias their last valid front; reduced connectivity creates real standard cells. The old affected cells are absent from the replacement manifest before new cells are admitted. No custom tetrahedralizer, optimizer, new physical DOF or global remesh is needed. Existing V4 exact cell/face routines and OpenFOAM-compatible quality calculations are reused. Gmsh 4.15.2 is reused only for bounded discrete-interface ingestion. More complex cavity tetrahedralization is unnecessary for this component.

### I. Mixed-cell topology

Final study shell: **7,445 prisms, 91 pyramids and 55 tetrahedra** (7,591 cells). New cells: 23 pyramids and 13 tetrahedra. No new geometric vertex is placed. Invalid prism column 1232 does not survive at layer 4 or 5; it terminates its prism prefix at 3. Original unused upper coordinate slots are not referenced by the repaired mesh.

### J. Ownership and orientation

Every internal face has two opposite owners; boundary faces have one intended owner. No cracks, orphan faces or unsupported hanging nodes occur. Retained cavity-interface faces match exactly. Outer vertex links are manifold. Ordered geometry is kept separate from canonical incidence keys. No automatic orientation flip hides a negative cell.

### K. Hard-quality results

All **7,591 cells** are checked, including unchanged lower cells. All applicable gates pass. Tails: volume {tail['min_volume']:.12g} m³; determinant {tail['min_det']:.12g}; interpolation weight {tail['min_weight']:.12g}; skewness {tail['max_skew']:.12g}; symmetric volume ratio {tail['min_ratio']:.12g}; aspect {tail['max_aspect']:.12g}. Thresholds are unchanged.

Prisms use frozen exact wedge validity. Tetrahedra use an affine exact triple product. Pyramids use the established bilinear-base Jacobian coefficient with the reference apex factor removed, as qualified in the preceding template study. No fabricated prism metric or new conditioning gate is applied to mixed cells.

### L. Complete volume overlap/gap check

All standard cells have positive exact validity. The complete unique-face certificate is combined with oriented two-owner incidence and exact global boundary degree at every cell-interior witness. Every degree is one; no unresolved ray remains. This excludes contained/duplicate cell interiors as well as crossings. Exact summed cell volume equals exact oriented boundary volume. Samples alone are not treated as nonoverlap proof: the full boundary nonintersection and validity prerequisites are mandatory.

All ten original overlaps are gone; no new volume overlap or internal gap is found. The repaired outer envelope recedes where prisms terminate. Removed invalid exterior volume becomes space for a future core; it is not required to remain inside the repaired shell. This is distinct from leaving a hole inside that shell.

### M. Continuous geometry

**536,679 exact unique-face candidate pairs** across the entire small fixture pass with zero forbidden and zero unresolved results. This includes preserved–preserved, preserved–rebuilt and rebuilt–rebuilt faces, rather than only the exposed surface or new faces. Existing certificate mathematics is unchanged.

### N. Outer interface

The 1,536-triangle interface is watertight, oriented, duplicate-free, vertex-manifold and free of exact self-intersections. It has 774 used vertices and 2,304 edges. OUTER_INTERFACE.json binds its deterministic connectivity hash. The three connected surface components are checked together, including intercomponent geometry.

### O. Core-compatibility smoke

Gmsh 4.15.2 ingests the discrete interface and returns identical coordinates and connectivity: 774 nodes, 1,536 triangles, 2,304 edges. No global core is generated. This proves interface ingestion plus the independent closed/exact surface checks; it does not qualify a future core mesh or CFD solution.

### P. Neighbor continuation

Three expanding layer-6 ring trials fail quality and are rejected. A smaller neighboring column, **1161**, advances at the full requested next increment and passes. The local step adds **1 prism, 3 pyramids and 9 tetrahedra**, with **1,854 exact interactions** and zero forbidden/unresolved results. Exact degree and volume accounting also pass. This is one valid local continuation, not blanket qualification of layer 6 over the whole fixture. No recovery horizon or full-depth campaign is run.

### Q. Analytic capacity smoke

The same variable-depth reconstruction terminates the spherical cavity at 12, preserving the analytic limit 13. Quality, ownership, exact outer geometry and volume accounting pass. No invalid layer-13 cell or pathological tiny increment is admitted. The analytic witness is not erased by compression.

### R. Channel smoke

The narrowing channel's preserved depth-9 whole-support reconstruction reports 539 existing hard-quality failures, including degenerate support behavior. The smoke is **SupportBlocked**, not a success. No clean channel remesh or generalization is claimed, and this task does not grow into rebuilding all channel fixtures. This limitation motivates the next broader local-remesh study.

### S. SF2 regression

Sealed Task-4L hashes verify unchanged. Zero remesh regions, termination regions, geometry changes, layer-count changes or correction changes. No full SF2 run is made.

### T. Runtime and locality

Measured selected search/validation plus complete replay totals **{total:.6f} s**; peak RSS **{peak:,} bytes**. RUNTIME_SUMMARY.json records grouped search, full-face certification, volume audit, full-cell quality, smoke and replay timings. These are measured stage times, not total task wall time; exploratory reruns, initial depth scan, tests and sealing are excluded. Full-fixture verification is deliberate to detect collateral errors; remeshing itself changes only 45 columns. No local optimizer or tet backend is invoked.

### U. Determinism

Independent replay matches the selected search, removal/replacement manifest, connectivity, quality, all 536,679 face outcomes, volume evidence, outer interface, analytic smoke and continuation decisions exactly. Timing/RSS are excluded. Supplemental replay covers retained cavity extraction, full-cell quality and Gmsh ingestion.

### V. Tests

{tests['summary']}. TESTS.json records the exact command. Coverage includes multi-layer removal, first-layer protection, mixed templates, positive validity, exact containment degree, disjoint components, cavity matching, complete volume/face evidence, invalid-cell removal, original-overlap elimination, continued prism construction, analytic smoke, SF2 identity and replay. Existing relevant tests are retained. The channel smoke's rejected state is preserved as evidence, not counted as a passing remesh.

### W. Classification

**{classification}** for this bounded general-SDF rollback mechanism and analytic smoke. The local volume, interface and one continuation qualify. Channel generalization remains unqualified.

### X. Production architecture recommendation

**Yes:** production architecture should permit bounded rollback/remeshing of previous upper layers, subject to these complete acceptance gates. This is not authorization to deploy the study implementation or a universal trigger. Wall/h1 protection, local scope, deterministic transactions and full volume verification remain required. No production code or trigger policy is integrated here.

### Y. Recommended next task

**D. Broader local-remesh generalization.** Build clean bounded channel cavities and test rollback/cavity growth beyond this component before choosing production triggers or claiming robust full-front continuation. This next task is not implemented.
'''
(OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
sources=['tools/local_cavity_remesh.py','tools/cavity_volume_audit.py','tools/verify_local_cavity_remesh.py','tools/qualify_cavity_complete.py','tools/cavity_core_ingestion_smoke.py','tools/seal_local_cavity_remesh.py','tests/test_local_cavity_remesh.py']
save('AUTHORITY.json',dict(task='ImplementV4LocalTerminationCavityRemeshingWithoutFrozenPreviousLayers',input_head=head,classification=classification,
    report_sha256=sha(OUT/'report.md'),source_sha256={p:sha(ROOT/p) for p in sources},
    evidence_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')},
    prior=prior,production_modified=False,V3_modified=False,trigger_implemented=False))
save('HASHES.json',dict(report=sha(OUT/'report.md'),authority=sha(OUT/'AUTHORITY.json')))
print(json.dumps(load(OUT/'HASHES.json')))
