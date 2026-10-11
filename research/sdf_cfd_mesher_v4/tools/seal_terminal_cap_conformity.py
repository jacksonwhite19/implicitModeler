"""Seal the exact preservation obstruction, without claiming a remesh."""
import subprocess,json
import qualify_terminal_cap_conformity as s
ROOT=s.ROOT;OUT=s.OUT;load=s.load;save=s.save;sha=s.sha
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert head=='f60df2e5003849c4e35a01a524e5c6a14140e542'
assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
other=lambda rows:[z for z in rows if 'sdf_cfd_mesher_v4' not in z]
assert other(load(OUT/'START.json')['status'])==other(subprocess.check_output(['git','status','--porcelain'],text=True).splitlines())
sf=ROOT/'results/task4l'
assert sha(sf/'report.md')=='ba74935df763baaa7800b669f1e054690abb21a2289ce7bd060c00301d640df4'
assert sha(sf/'AUTHORITY.json')=='1e3ce1dced7d59422bbadeeed9cb9daf47575e1cbeaa0275f045c9f34ee9d6d3'
save('SF2_REGRESSION.json',dict(report=sha(sf/'report.md'),authority=sha(sf/'AUTHORITY.json'),termination_regions=0,geometry_changes=0,layer_count_changes=0,correction_changes=0,full_run=False))
rows=load(OUT/'FORENSICS.json');groups=load(OUT/'COMPONENTS.json');runtime=load(OUT/'RUNTIME.json')
assert len(rows)==10 and all(x['witness']['proven'] for x in rows)
save('REMESHING_METHOD.json',dict(method='Exact frozen-cell overlap guard before constrained retriangulation',performed=False,
    tools_considered={'sdf_conforming_kernel/cell_triangulation.rs':'Convex planar lattice center fan; incompatible with this crossing 3D frozen interface',
    'sdf_conforming_kernel/rounded_tip_wing_local_retriangulation3d.rs':'Geometry-specific projection and tolerance thresholds; not a preservation repair',
    'Gmsh 4.15.2 from preceding verified survey':'Constrained surface/cavity meshing cannot remove overlap of immutable existing cell domains',
    'V4 termination_geometry_certificate':'Reused unchanged exact clipping, rational geometry and bilinear certificate'},
    reason='Any replacement outer surface leaves overlapping frozen cell interiors intact. A union boundary would hide overlap, not repair cell ownership or volume partition.'))
save('TRANSITION_REGENERATION.json',dict(performed=False,before=dict(pyramids=68,tetrahedra=44,prisms=1346),after=dict(pyramids=68,tetrahedra=44,prisms=1346),
    dependent_cells=groups[0]['dependent_new_cells'],unchanged=True,reason='Preservation obstruction; no admissible volume cavity'))
save('QUALITY.json',dict(new_quality_claim=False,preserved_previous_quality_sha256=sha(s.OLD/'candidate_1_quality.json'),
    previous_applicable_cell_gates_passed=True,complete_geometry_passed=False,reason='Individual positive cell validity does not imply disjoint cell interiors'))
save('OUTER_INTERFACE.json',dict(watertight_topology=True,manifold_topology=True,geometrically_valid=False,core_compatible=False,
    original_forbidden=10,remaining_forbidden=10,new_forbidden=0,unresolved=0,hash=load(OUT/'PRESERVATION.json')['outer_hash'],
    ownership_source='OWNERSHIP.json',certificate_source='OUTER_CERTIFICATE.json',zero_area_faces=False))
save('CHANNEL_SMOKE.json',dict(performed=False,reason='Primary general-SDF qualification failed; conditional channel work skipped'))
classification='V4TerminalCapConformityInsufficient'
save('RESULT.json',dict(classification=classification,production_integration=False,new_surface_vertices=0,termination_map_changed=False,
    lower_prisms_preserved=6144,obstruction='Exact positive-volume overlap of frozen lower prisms',
    next_task='Frozen-stack overlap qualification and explicit minimal cell-repair preservation contract; not production triggers'))
table='\n'.join(f"| {z['faces'][0]} / {z['faces'][1]} | none | strict interior transverse crossing | positive-volume overlap proven |" for z in rows)
report=f'''## Decision question

Can V4 preserve every valid lower prism cell while locally remeshing the newly exposed termination surface into a conforming, non-self-intersecting, deterministic outer boundary suitable for later core meshing?

### A. Authority verification

Input HEAD `{head}` matches. The prior report, authority and every bound source/evidence file verify unchanged. The feature branch matches. All 155 unrelated dirty entries remain unchanged. V3 and production files have zero task changes.

### B. Frozen-cap intersection forensics

The ten pairs are genuine transverse crossings of frozen triangular top interiors. They share neither vertices nor edges. They are not an alternative diagonal of one nonplanar quad, a missing stitch, or a termination-map indexing defect. Each triangle is the boundary of a preserved layer-4 prism. FORENSICS.json stores global/source vertex IDs, coordinates, exact intersection endpoints and rational proof data.

| Cap pair / column IDs | Shared entity | Relation | Frozen-cell result |
|---|---|---|---|
{table}

For each pair, the exact midpoint of the intersection segment has three strictly positive barycentric weights in both triangles. Their normal Gram determinant is strictly positive. At that point, each prism's top Jacobian is strictly positive. An exact rational direction d solves nAÂ·d = nBÂ·d = -1. The inverse function theorem at each regular prism boundary gives an inward neighborhood. Thus, for sufficiently small positive displacement, both cell interiors contain the same open neighborhood. Their overlap has positive volume. No floating tolerance, sample step or unverified optimizer outcome establishes this conclusion.

This is a local geometric obstruction proof for the existing frozen cells, not a proof that all meshing of this SDF is impossible. Earlier positive cell gates and side-only certificates remain evidence for their stated populations; they do not establish pairwise disjoint cell volumes.

### C. Conflict components

One intersection-graph component contains 11 caps. Its combinatorial boundary has two disjoint loops. COMPONENTS.json records ordered frozen boundary edges/coordinates, neighboring outer faces and the two dependent new-cell IDs. An intersection-connected component need not be a single topological disk.

### D. Preservation contract

All lower coordinates, connectivity, sides and geometric cell domains remain frozen. Changing a face's triangulation can preserve its geometric domain; it cannot remove positive-volume overlap between two unchanged domains. Replacing the visible outer envelope or splitting its intersection curves would leave overlapping retained cells underneath. That would violate the required no-overlap volume partition even if the rendered outer surface looked clean.

The newly relaxed cap-triangulation identity does not remove this obstruction. Success requires changing at least one overlapping lower cell domain or removing/repartitioning it, beyond this task's preservation contract. No such change is made.

### E. Local surface cavity

The exact conflict patch boundary and adjacent dependencies are extracted before remeshing. It is not admitted as a fillable volume cavity: the preserved lower boundary already bounds overlapping cell interiors. Boundary-only retriangulation, added upper vertices or new transition cells cannot subtract that overlap from preserved cells.

### F. Remeshing method

Implemented an exact preservation guard for local restitching. It returns `RejectedFrozenVolumeOverlap` without mutation when the direct proof applies. No triangulation algorithm is run on this invalid frozen volume foundation. The existing convex-lattice center-fan triangulator, geometry-specific nonplanar remeshing code and available Gmsh path were reviewed; none can satisfy this contradictory volume contract. Exact rational intersection and original V4 certification are reused. No new dependency, approximate acceptance rule or global mesher is introduced.

### G. Terminal-cap conformity

Zero new vertices or faces. All ten original crossings remain, and no new crossing appears. This is not a successful surface remesh. PROPOSAL.json preserves the rejected, unchanged surface and the reason.

### H. Transition-cell regeneration

Before/after: 68 pyramids, 44 tetrahedra, 1,346 continuing prisms. No cell is regenerated or committed. Only two new cells touch conflict-component vertices directly; their IDs are recorded. All unaffected transition cells remain identical. The invalid layer-5 prism stays omitted.

### I. Ownership and orientation

The unchanged topology still has 17,447 two-owner internal faces and 3,072 boundary faces. Ordered orientations pair correctly; combinatorial manifoldness and watertightness remain as before. These facts cannot override geometric overlap.

### J. Hard-quality validation

The unchanged candidate retains its previous applicable individual-cell numeric and exact-Jacobian results. No repaired candidate exists, so no new passing hard-quality claim is made. The newly evaluated strict top Jacobians support the overlap proof. No thresholds are weakened and no fake mixed-cell prism metric is introduced.

### K. Exact continuous geometry

The complete outer audit is rerun twice: 9,907 candidate pairs, ten forbidden, zero unresolved. The pair population is unchanged. Exact clipping independently supplies the witnesses. Because these frozen-volume conflicts already make complete-shell acceptance impossible, new-cell certification is not rerun unnecessarily; its prior 151,270 passing interactions remain sealed evidence only for that population.

### L. Outer-interface validation

Topologically watertight/manifold, but geometrically self-intersecting. The deterministic outer hash is in OUTER_INTERFACE.json. All original intersections remain. No core-ready shell is published.

### M. Core-compatibility check

Failed: exact self-intersection and positive-volume overlap of retained cells. No local or global tetrahedral fill is attempted over this invalid foundation. An outer union surface alone would not fix cell-volume partitioning.

### N. Preservation audit

All 6,144 lower prisms, wall/first-layer geometry, lower vertex coordinates, all transition cells and distant continuing columns remain unchanged. The termination map is unchanged: 78 capped columns, 112 mixed transition columns, 1,346 continuing columns. PRESERVATION.json binds the lower coordinates to the preceding authority. Rejection occurs before any mutation.

### O. Analytic capacity smoke

The complete spherical-cavity fixture is reconstructed from its 12 sealed checkpoints. Ownership, applicable terminal quality and all 3,228 exact outer pairs pass. Its terminal surface needs no repair. The analytic limit remains 13; no fixture or witness changes. This is a regression of the no-op conforming path, not proof that arbitrary cavities can be remeshed.

### P. Channel smoke

Skipped because the primary case does not qualify. No channel support is rebuilt.

### Q. SF2 regression

Task-4L report/authority hashes verify unchanged. Zero termination, geometry, layer-count or correction changes. No production call site or full SF2 run is introduced.

### R. Runtime and locality

First forensic/cavity/full-outer pass: {runtime['first_seconds']:.6f} s. Exact replay: {runtime['replay_seconds']:.6f} s. Analytic reconstruction/quality/certificate: {runtime['analytic_seconds']:.6f} s. Total measured study execution: {runtime['total_seconds']:.6f} s. Peak RSS: {runtime['peak_rss']} bytes. RUNTIME.json separates seed reconstruction, intersection forensics, cavity extraction, ownership and exact outer certification. The analytic timing groups reconstruction, hard quality and certification. Unchanged general-cell quality is not recomputed after the exact obstruction. Remeshing/regeneration are not executed (zero time), not successful zero-cost operations. Tests, source survey and report writing are excluded.

### S. Determinism

Exact replay matches conflict clustering, rational witnesses, cavity edges, rejected unchanged surface, ownership, complete outer certificate and preservation identities. Runtime/RSS are excluded. There is no generated replacement connectivity to replay.

### T. Tests

TESTS.json records the exact focused command and outcome. Tests cover strict frozen-volume overlap, separated/coplanar/degenerate controls, deterministic components, ordered cavity loops, fail-closed restitching, no mutation, all ten real witnesses, dependency extraction, preservation, outer failure, analytic smoke, SF2 authority and replay. Existing local termination and Task-4H tests are retained. Constrained remeshing and regenerated-cell tests are not claimed because no admissible remesh is constructed.

### U. Classification

**{classification}**. Local surface remeshing cannot remove these conflicts while preserving all frozen lower cell domains. This conclusion follows from exact local geometry, not failed search or a missing triangulation dependency.

### V. Production integration recommendation

**No.** Terminal conformity and core compatibility remain unqualified. No production trigger policy is implemented.

### W. Recommended next task

First qualify pairwise frozen-stack volume disjointness and define an explicitly authorized minimal lower-cell repair/repartition contract for this component. Only then revisit local transition topology. Among the provided topology directions this is **D. Additional topology generalization**, preceded by repairing the frozen-volume foundation. Production trigger policy and core integration are premature. This next task is not implemented.
'''
(OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
sources=['tools/terminal_cap_conformity.py','tools/qualify_terminal_cap_conformity.py','tools/seal_terminal_cap_conformity.py','tests/test_terminal_cap_conformity.py']
save('AUTHORITY.json',dict(task='QualifyV4TerminalCapConformityAndLocalOuterSurfaceRemeshing',input_head=head,classification=classification,
    source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.iterdir()) if p.name not in ('AUTHORITY.json','HASHES.json')},
    report_sha256=sha(OUT/'report.md'),prior_authority=sha(s.OLD/'AUTHORITY.json'),production_modified=False,V3_modified=False,
    remesh_performed=False,exact_frozen_volume_overlap_pairs=10))
save('HASHES.json',dict(report=sha(OUT/'report.md'),authority=sha(OUT/'AUTHORITY.json')))
print(json.dumps(load(OUT/'HASHES.json')))
