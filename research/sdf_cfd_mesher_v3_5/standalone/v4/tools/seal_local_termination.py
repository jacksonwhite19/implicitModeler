"""Seal a promising mixed-cell mechanism with a failed frozen outer interface."""
import hashlib,json,subprocess
import qualify_local_termination as s
t=s.t;q=s.q;r=s.r;np=s.np;OUT=s.OUT;ROOT=r.r.g.ROOT
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda n:r.r.load(OUT/n)
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert head=='472c9966a659e3c17064e5e61d5399c5fc7d9358'
assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
other=lambda x:[v for v in x if 'sdf_cfd_mesher_v4' not in v]
assert other(status)==other(load('START.json')['status'])
assert not subprocess.check_output(['git','diff','--name-only','--','research/sdf_cfd_mesher_v3'],text=True).strip()
old=r.r.load(r.OUT/'AUTHORITY.json');prior={}
for name in ['residual_layer5_failure']+list(old['prior_authorities']):
    folder=ROOT/'results'/name;a=r.r.load(folder/'AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for path,h in a.get(group,{}).items():assert sha(ROOT/path)==h,path
    prior[name]=dict(report=sha(folder/'report.md'),authority=sha(folder/'AUTHORITY.json'))
s.save('AUTHORITY_VERIFICATION.json',dict(prior=prior,V3_modifications=0,production_changes=0,unrelated_dirty_entries=len(other(status)),unrelated_unchanged=True))
case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce();m=load('candidate_1_map.json')
points,cells,outer,m=t.construct(case.x,case.f,prefix,high,m['stopped_vertices']);owners,own=t.ownership(cells,outer,[tuple(reversed(f)) for f in case.f])
links=t.vertex_links(outer);assert links['passed'];s.save('OUTER_VERTEX_LINKS.json',links)
ledger=[dict(vertices=key,owners=[dict(cell=i,oriented_face=f) for i,f in rows]) for key,rows in sorted(owners.items())]
s.save('OWNERSHIP.json',dict(summary=own,faces=ledger))
new=[c for c in cells if c['layer']==5];mixed=[c for c in new if c['kind']!='prism'];preserved=[c for c in cells if c['layer']<=4]
_,baseline_cells,_,_=t.construct(case.x,case.f,prefix,high,[])
assert q.identity(preserved)==q.identity([c for c in baseline_cells if c['layer']<=4])
fixed=np.setdiff1d(np.arange(len(case.x)),m['stopped_vertices']);front=points[np.array(outer).ravel()].reshape(-1,3,3)
delta=max(abs(m['last_prism_layer'][int(a)]-m['last_prism_layer'][int(b)]) for e in range(len(case.top.edges)) for a,b in [case.top.edge_triangles.row(e)])
s.save('PRESERVATION.json',dict(wall_hash=q.array_hash(case.x),first_hash=q.array_hash(prefix[0]),lower_coordinate_hash=q.array_hash(points[:5*len(case.x)]),
    source_lower_hash=q.array_hash(np.concatenate([case.x]+prefix)),lower_prism_connectivity_hash=q.identity(preserved),preserved_prisms=len(preserved),
    lower_connectivity_equal=True,unaffected_upper_equal=bool(np.array_equal(points[5*len(case.x)+fixed],high[fixed])),
    note='Unused candidate upper nodes at stopped vertices are not referenced by cells or interface faces.'))
s.save('TERMINATION_MAP.json',dict(**m,maximum_adjacent_last_prism_difference=delta,study_depth=5,
    final_layer_count_policy=False,scope='190 columns have last prism layer 4: 78 have no new cell, 112 have a mixed transition cell. Other columns reach study layer 5; they are not instructed to stop there.'))
s.save('NEIGHBOR_CONTINUATION.json',dict(layer=5,prisms=len(m['continuing_columns']),columns=m['continuing_columns'],
    cell_and_new_interaction_checks_passed=True,complete_shell_admitted=False,reason='Frozen terminal cap intersections remain'))
quality=load('candidate_1_quality.json');continuous=load('CONTINUOUS.json');oc=load('OUTER_CERTIFICATE.json');analytic=load('ANALYTIC_SMOKE.json')
assert quality['passed'] and continuous['passed'] and not oc['passed'] and analytic['passed']
assert load('DETERMINISM.json')['equal'] and load('CAVITY_DETERMINISM.json')['equal']
tests=load('TESTS.json');assert tests['passed']
tails=dict(minimum_volume=min(z['volume'] for z in quality['cells']),minimum_determinant=min(z['determinant'] for z in quality['cells']),
    maximum_aspect=max(z['aspect'] for z in quality['cells']),minimum_weight=min(z['weight'] for z in quality['faces']),
    maximum_skewness=max(z['skewness'] for z in quality['faces']),minimum_symmetric_volume_ratio=min(z['symmetric_volume_ratio'] for z in quality['faces']))
s.save('QUALITY_SUMMARY.json',dict(passed=True,tails=tails,exact_positive_cells=len(quality['exact']),
    scope='All layer-4 and new layer-5 cells; internal faces touching these layers include layer-3 neighbors. Earlier unchanged gates inherit sealed evidence.',
    applicability=dict(prism='Frozen exact wedge polynomial',tetrahedron='Exact affine triple product',pyramid='Exact bilinear-base Jacobian coefficient after removing reference apex factor',
        determinant='Internal/coupled face area tensor',aspect='Existing general face-area/volume expression',face_metrics='Existing V4/OpenFOAM internal-face definitions; no fabricated exterior neighbor',
        conditioning='No mixed-cell reconstruction operator qualified here; no sigma/G threshold or substitute')))
s.save('TOOLS_SURVEY.json',dict(reused=[
    dict(tool='Standard collapsed-extrusion connectivity templates',role='0/1/2/3 advanced nodes produce no new cell/tet/pyramid/prism; no collapsed prism retained'),
    dict(tool='V4 frozen Foundation face geometry and face_metrics',role='Reuse without changing gates'),
    dict(tool='sdf_layer_v2/openfoam_transition_forensics.rs',role='Read-only reference for general-cell centre, volume and internal-face area tensor; expressions ported locally'),
    dict(tool='sdf_advancing_tet_transition three_face_closure_authority',role='Reuse ordered-face versus canonical-identity ledger principle; no Rust source copied or modified'),
    dict(tool='V4 exact bilinear certificate',role='Unchanged kernel on exact rational face partitions, after exact hull separation')],
    considered=[dict(tool='Gmsh 4.15.2',location='WSL /home/jwhite/.local/bin/gmsh via python3',decision='Available; not invoked for meshing. Direct templates preserve faces without cavity boundary recovery.',source='https://gmsh.info/doc/texinfo/'),
        dict(tool='tetgen/meshio Windows Python packages',decision='Not installed; no installation needed'),
        dict(tool='Existing Rust tolerance-based triangle/SAT checks',decision='Not reused as exact acceptance predicates'),
        dict(tool='Gmsh QuadTri/core tetrahedralization',decision='Deferred: core meshing and protected boundary subdivision are outside this bounded template study')]))
s.save('SF2_REGRESSION.json',dict(termination_regions=0,geometry_changes=0,layer_count_changes=0,correction_changes=0,
    method='Read-only sealed Task-4L authority and artifact verification; no production call site or SF2 rerun',authority=prior['task4l']))
s.save('CHANNEL_SMOKE.json',dict(performed=False,reason='Primary complete outer-interface qualification failed; user makes channel smoke conditional on primary qualification'))
classification='V4LocalLayerTerminationTopologyPromising'
result=dict(classification=classification,complete_general_sdf_qualified=False,new_transition_qualified_locally=True,
    complete_shell_committed=False,production_integration=False,next_task='D. Further termination topology development',
    blocker='Ten exact intersections among unchanged terminal layer-4 caps; fixing them requires a separately scoped preservation/conformity decision',
    stopped_columns=78,mixed_columns=112,continuing_prisms=1346,transition_cells=dict(pyramid=68,tetrahedron=44),
    invalid_layer5_prism_1232_absent=not any(c['column']==1232 for c in new),outer_topologically_watertight=True,outer_embedded_valid=False,
    maximum_adjacent_difference=delta,analytic_passed=True,trigger_policy_implemented=False)
s.save('RESULT.json',result)
perf=load('QUALIFICATION_RUNTIME.json');first=sum(v['seconds'] for v in perf['first'].values())
report=f'''## Decision question

Can V4 locally terminate prism layers in a region where continued prism advancement is impractical, taper neighboring layer counts, and create a valid deterministic transition to a watertight core-compatible outer interface?

### A. Authority verification

Input HEAD `{head}` and the feature branch match. All {len(prior)} prior authority manifests and their bound source/evidence files verify unchanged. V3 has zero task modifications. All {len(other(status))} unrelated dirty entries remain unchanged. No production runner, wall, first layer, stored schedule or certificate mathematics is changed.

### B. Existing difficult-region evidence

The layer-4 seed replays its 43,162-relation side certificate and hard gates. The rejected layer-5 candidate is reproduced from the preceding task, without another optimizer or recovery campaign. Its exact/numeric failures and 26 forbidden side relations identify an explicitly authorized development region. This is not a new production termination trigger or an infeasibility claim.

### C. Reused standard techniques/tools

The implementation uses standard mixed-cell extrusion templates: triangular prisms, quadrilateral-base pyramids and tetrahedra. Existing V4 Foundation face geometry and internal-face quality functions are reused. General polyhedral centre/volume and area-tensor expressions follow the existing read-only `sdf_layer_v2` OpenFOAM evaluator. Ordered geometric faces remain separate from canonical incidence keys, following the existing typed closure work.

Gmsh 4.15.2 is available under WSL. Its documented mixed-cell and tetrahedral capabilities were considered ([official manual](https://gmsh.info/doc/texinfo/)). No meshing backend is needed for this small template. A cavity tetrahedralizer would introduce a separate protected-boundary recovery problem. No new dependency is installed. Existing tolerance-based intersection tests are not used as exact gates. TOOLS_SURVEY.json records the choices.

### D. Termination-map construction

The explicit study map initially stops 21 vertices controlling the measured bad cells and side interactions. It yields 14 fully stopped columns, 64 transition columns and 1,458 continuing prisms. Seven resulting tetrahedra fail the unchanged determinant gate. One adjacency-ring expansion stops 65 vertices and passes the applicable quality gates. No universal layer-count gradient, clearance cutoff, safety factor or minimum height is introduced.

The selected depth-5 map has **190 last-prism-layer-4 columns**: **78 fully capped at layer 4** and **112 with a non-prismatic layer-5 transition cell**. **1,346 columns continue as layer-5 prisms**. The exact map is in TERMINATION_MAP.json. Layer 5 is a study depth, not a final stop instruction for distant columns. The first passing bounded ring is selected; global minimum termination area is not proven or pursued after the frozen-interface blocker is found.

### E. Transition topology

For each source triangle, upper nodes of stopped vertices alias the corresponding existing layer-4 nodes. Three advancing vertices yield a prism; two yield a pyramid; one yields a tetrahedron; zero yield no new cell. Reduced faces are kept in their original geometric order. Zero-area two-node faces disappear combinatorially; no nearly collapsed prism is created. The selected proposal has **68 pyramids, 44 tetrahedra and 1,346 new prisms**.

These templates preserve shared quadrilateral faces next to continuing prisms. They do not split or move any existing lower prism face. IDs and connectivity are deterministic. The cell-ID range guard prevents collisions in this bounded study representation.

### F. Local cavity definition

The transition cavity is the union of the 112 proposed non-prism cells. Its oriented boundary is extracted by face incidence: retained lower triangles, shared lateral faces with continuing prisms, and the new outer triangles. candidate_1_cavity.json records each boundary face and its surrounding owners. There is no global remeshing or core fill.

### G. Cell connectivity and ownership

The reconstructed shell has **17,447 internal faces**, each with two opposite owners, and **3,072 boundary faces**: 1,536 wall faces plus 1,536 outer faces. No missing, duplicate, nonmanifold-edge or orientation conflict is found. Outer vertex links are single cycles. OWNERSHIP.json preserves the face ledger, not only counts.

All **6,144 lower prisms** and all layers 0–4 coordinates retain exact identity. Unaffected upper coordinates match the starting prospective candidate bitwise. Unused upper-coordinate slots at stopped vertices are not mesh entities. No wall-adjacent cells are remeshed.

### H. Quality validation

All **2,994 layer-4 and proposed layer-5 cells** pass appropriate exact validity and the applicable hard numeric gates. Internal-face checks include prior-layer neighbors. Earlier unchanged layers retain sealed evidence. Tails:

| Metric | Measured | Required |
|---|---:|---:|
| Volume minimum | {tails['minimum_volume']:.12g} m³ | positive |
| Determinant minimum | {tails['minimum_determinant']:.12g} | ≥ 0.0011 |
| Interpolation weight minimum | {tails['minimum_weight']:.12g} | ≥ 0.055 |
| Skewness maximum | {tails['maximum_skewness']:.12g} | ≤ 3.63636 |
| Symmetric volume ratio minimum | {tails['minimum_symmetric_volume_ratio']:.12g} | ≥ 0.01 |
| Aspect ratio maximum | {tails['maximum_aspect']:.12g} | ≤ 1000 |

Prisms use the frozen exact wedge polynomial. Tetrahedra use their exact affine triple product. For a bilinear-base pyramid, the Jacobian is `(1-w)^2 D(u,v)`, where D is bilinear. Its four exact rational corner coefficients must be positive; the reference-apex factor is not treated as a collapsed prism. Exact volume is the sum of these coefficients divided by 12. Tests compare the bilinear identity with independent point evaluation and standard tetrahedral volumes.

General-cell determinant uses the existing internal/coupled-face area tensor, with the new exposed outer boundary excluded. General face-area/volume aspect and existing internal interpolation/skew/volume-ratio definitions are retained. An outer face has no core neighbor yet; no synthetic neighbor metric is invented. Mixed-cell sigma/G reconstruction is not qualified here and no diagnostic threshold is added.

### I. Continuous geometry validation

All **151,270 AABB-overlapping unique-face pairs touching proposed new cells** pass: **117,808 exact hull legal-contact proofs** and **33,462 exact hull separation proofs**; zero forbidden or unresolved results. Original binary64 control points are converted to exact rationals. Floating arithmetic only orders candidate separating axes; every proof uses exact signs.

The fallback partitions a planar triangle into three exact rational quads and a bilinear quad into four exact parameter subpatches. Shared midpoint identities preserve legal edges. It calls the unchanged qualified bilinear kernel. No warped quad is planarized. This adapter is exercised by crossing, containment and incidence tests and by the failing outer-cap audit.

This certificate explicitly excludes old–old face pairs. It proves new interactions, not validity of every preserved cap. The complete outer-interface check below is a separate mandatory gate.

### J. Outer-interface validation

The 1,536-face interface is combinatorially watertight, consistently oriented and vertex-manifold, but **not geometrically valid for core meshing**. Of **9,907 outer candidate pairs**, **10 are exactly forbidden**, with zero unresolved results. Every one involves frozen layer-4 cap **784**, intersecting caps **1157, 1168, 1169, 1171, 1200, 1221, 1232, 1233, 1235 and 1264**. All are fully terminated columns, so both faces are unchanged layer-4 geometry.

FROZEN_CAP_REPRODUCER.json independently reproduces all ten intersections using rational segment-plane clipping and triangle halfspaces on the sealed seed alone. Full rational intersection points and source coordinates are stored. The prior side certificate remains unchanged and valid for its stated side-surface population; it did not certify these triangle-cap pairs. This distinction is not a modification of certificate mathematics.

Repairing or conformally reconciling the intersecting frozen caps is outside the instruction to leave layers 0–4 unchanged. No such repair is attempted. No complete general-SDF shell is admitted or exported as core-ready.

### K. General-SDF result

The invalid layer-5 prism at column 1232 is absent. The proposed mixed cells and continuing prisms pass local validity, quality and new-interaction checks. The complete result remains unqualified because of the ten frozen terminal-cap intersections. This is a concrete topology mechanism with a separately reproduced input-interface blocker, not a successful complete mesh.

### L. Neighboring-column continuation

The proposal constructs and checks one next step: 1,346 neighboring/distant prisms advance to layer 5 while the difficult core remains at 4. They pass cell and new-interaction validation. The overall shell is not committed because the outer-interface gate fails. No layer 6 or broad fixture campaign is run.

### M. Layer-count taper

The template supports the local 4/5 study pattern with maximum adjacent last-prism-layer difference **1**. This follows from the single-step cell template, not a universal meshing rule. One graph ring was added to the measured failure support; mixed cells occupy the boundary collar. The proposal changes 190 columns' layer-5 topology and uses 65 stopped vertices, while preserving all lower geometry and distant candidate geometry.

### N. Analytic capacity fixture smoke

The spherical cavity stops all **512 columns at layer 12**, before its unchanged analytic geometric limit at 13. Its **6,144 retained prisms** remain identical. No extra transition cells are needed when the entire bounded cavity stops. Ownership, applicable terminal quality and **3,228 exact outer pairs** pass. This validates clean whole-region capping; the partial mixed taper is exercised in the general-SDF case.

A preliminary whole-parallel-fixture cap audit found 72 pre-existing closure face-quality failures outside the prior measured-core contract. It is preserved in ANALYTIC_PARALLEL_DIAGNOSTIC.json and is not counted as a pass or exempted. The complete spherical fixture supplies the successful analytic smoke.

### O. Channel smoke

Skipped. The user makes this conditional on primary complete qualification, which fails the frozen-cap gate. Existing blocked channel supports are not rebuilt.

### P. SF2 regression

Sealed Task-4L artifacts verify unchanged. Zero termination regions, geometry changes, layer-count changes or correction changes. The tools have no production call site. No SF2 run is made.

### Q. Runtime and locality

First topology/new-certificate/outer-and-analytic stages total **{first:.6f} s**. Independent complete replay: **{perf['replay_seconds']:.6f} s**. Peak replay RSS: **{perf['peak_rss']:,} bytes**. Separate stage timings are preserved. These are qualification execution times, not full task wall time; tool survey, report sealing, tests and small independent reproducer runs are excluded.

The new local transition has 112 cells and alters no lower prism. Full small-fixture verification is used to catch collateral and old-interface issues. No global mesher or optimization is invoked.

### R. Determinism

Maps, cell IDs, connectivity, ownership, exact validity, quality arrays, new-interaction proofs, outer failures and analytic results replay exactly. Main identity: `{load('DETERMINISM.json')['first']}`. Cavity extraction has an additional exact replay. Runtime and RSS are excluded. The study metadata key collision found during initial rejection reporting was fixed without changing numerical behavior; rejection-path testing covers it.

### S. Tests

{tests['summary']}. TESTS.json records the command; tests.txt records output. Tests cover all eight node-stop patterns, exact tet/pyramid validity, face matching, ordered edges, nonmanifold vertex links, rational face tilings, forbidden triangle/bilinear crossings, preservation, new-versus-frozen certificate scope, analytic capping and SF2 authority. Existing focused geometry tests are retained.

### T. Classification

**{classification}**. A valid local mixed transition is demonstrated, but the complete general-SDF outer interface is invalid. No complete-shell qualification or publication is claimed.

### U. Production integration recommendation

**No, not yet.** Preserve the mechanism as study code. Full outer-interface qualification is a prerequisite for integration. No production trigger, failure-count rule, minimum-height rule or general layer-count policy is added.

### V. Recommended next task

**D. Further termination topology development.** First resolve the frozen terminal-cap conformity/authority issue under an explicit preservation contract. Do not proceed directly to production triggers or core fill while the current terminal triangles intersect. This next task is not implemented here.
'''
(OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
sources=['tools/local_termination_topology.py','tools/termination_geometry_certificate.py','tools/qualify_local_termination.py','tools/seal_local_termination.py','tests/test_local_termination_topology.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.suffix!='.log' and p.name not in ('AUTHORITY.json','HASHES.json')}
s.save('AUTHORITY.json',dict(task='ImplementV4LocalLayerTerminationTopology',input_head=head,classification=classification,
    report_sha256=sha(OUT/'report.md'),source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256=evidence,prior_authorities=prior,
    scientific_identity=load('DETERMINISM.json')['first'],production_modified=False,V3_modified=False,production_trigger_implemented=False,
    general_sdf_complete_qualified=False,analytic_smoke_qualified=True))
s.save('HASHES.json',dict(report=sha(OUT/'report.md'),authority=sha(OUT/'AUTHORITY.json')))
print(json.dumps(load('HASHES.json'),indent=2))
