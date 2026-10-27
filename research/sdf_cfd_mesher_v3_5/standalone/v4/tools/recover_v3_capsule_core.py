"""Read-only recovery and bounded boundary adapter; never launches a mesher.

V3 algorithms are recovered by name and source hash. Missing case sizing is
not filled from the SF2 configuration or inferred from a geometric box.
"""
import ast
import collections
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np

import capsule_user_domain_handoff as handoff
from capsule_user_domain_handoff import read, eq, need, seal, measured, metrics, durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity, array_hash

V4 = handoff.V4
V3 = V4.parent/'sdf_cfd_mesher_v3'
OUT = V4/'results/v3_capsule_core_adapter'
FAMILY = V3/'fresh_supported_refinement_family_qualification_2026-09-09'
SUPPORTED = V3/'supported_refinement_family_qualification_2026-09-09'
DOWNSTREAM = V3/'airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py'
BASELINE = V3/'farfield_transition_geometry_qualification_2026-09-08/source_before/run_airfoil_downstream.py'
CLASSIFICATION = 'V4CapsuleCoreBlockedMissingQualifiedSizingAndOuterDiscretizationAuthority'
POLY = '3ed2471b2dea7b5ffa26b9a3caf43b2182f7b50b8c434b3117ed7a8e5434804f'
HANDOFF = '27280195a7690d81f1dd11b2bc4191982e1576dedb51c82d2f491345d281eb5b'
DOMAIN = '63c749602dfd1f0ae481a640127249b7fc031f894cabf31c565cd0163c3eec43'

# A: algorithm; B: mechanism needing qualified new inputs; C: case-specific;
# D: incompatible wrapper; E: transfer evidence missing. No row authorizes a run.
PIPELINE = [
 ('entry','D','fresh_supported_stage.py -> supported core_stage.py -> common.driver -> physical_naca run_core_pipeline -> run_airfoil_downstream',
  'Wrappers require wing input_authority, SF2 levels and support arrays. Replace bindings only, not algorithms.'),
 ('boundary transfer','B','surface_boundary_contract, transfer_boundary',
  'Discrete exact coordinates/triangles; inner sheets have negative region orientation; outer surface positive. Parameterize ID namespaces; preserve V4 IDs in a bijection.'),
 ('outer discretization','B/C','transfinite_cube_farfield; grid_controls.py; FRESH_BASELINE_CONTRACT.json',
  'Transfinite mechanism reusable; cube half-width 2.5, ceil(42/scale), SF2 66 and resulting edge scale are case-specific. Rectangular counts remain unauthorised.'),
 ('support','C','support_control.py; boundary_normal_support.py',
  'SF2 has 52,272 support prisms and an inset core boundary. This case requires the box itself as the tetrahedral outer boundary, with no imported support.'),
 ('sizing','B/C','CoreSizeCallback; common.driver scale wrapper',
  'KD nearest interface vertex; h=min(hmax,h0*growth**(distance/h0),optional h_ff+alpha*inward_box_distance). Inputs are not derived by the callback.'),
 ('point insertion and tetrahedralization','A/B','run_delaunay; Gmsh 4.15.2 Algorithm 1',
  'Recover prescribed boundary, insert volume points to scalar size field; linear tetrahedra. Exact boundary recovery remains a tested postcondition, not a blanket guarantee.'),
 ('optimization','A/B','run_gmsh_preconditioner; run_netgen',
  'One Gmsh pass, isolated saved-core reload, volume-only createGeometry([(3,1)]), exact six-array comparison, one Netgen pass; no second fill.'),
 ('coordinate correction','B','bind_coordinate_replay.py; fresh_corrected_core_stage.py',
  'Run only if current failures admit the qualified guarded correction; fixed connectivity/boundaries, movement controls, complete re-audit and rollback. Do not import SF2 moved vertex IDs.'),
 ('ownership','B','validate_core; exact_coordinate_map; compare_surface_records',
  'Bitwise coordinate bijection, exact boundary triangle set and winding, one boundary core owner, complete incidence, connected components and duplicate rejection.'),
 ('join and publication','B/D','assemble_and_check; assemble.py; bind_coordinate_replay.py assembly',
  'Canonical face owner/neighbour merge, transfer MSH, gmshToFoam, exact conversion and canonical polyMesh. Replace native whole-prefix/materialization bindings with V4 inherited state plus core delta.'),
 ('quality','B','core_audit.py; Foundation geometry; native checkMesh',
  'Actual centroids/volumes, area-tensor determinant, face weight/ratio/skewness/aspect. Core audit uses skewness <=4 in places: V4 <=3.63636 must remain. Support-internal masks do not apply to physical box boundaries.'),
 ('geometry coverage','B/E','validate_core; fresh_corrected_core_stage.py; V4 continuous certificate',
  'Positive cells, exact constrained boundary, connected partition and volume checks reusable. Complete new core/core and prism/core continuous coverage must be bound to V4; checkMesh alone is not that certificate.'),
 ('volume and native validation','B','validate_core; assemble_and_check',
  'Signed outer minus inner volume; tolerance max(4096*eps*max(abs(expected),sum(abs(vol))),1e-9). Foundation13 checkMesh -constant -allTopology -allGeometry plus native quality and conversion fidelity.'),
]


def source_definitions(path, names):
    """Execute original pure definitions, excluding V3 module-level I/O."""
    tree = ast.parse(Path(path).read_text(encoding='utf-8-sig'))
    nodes = [n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    eq({n.name for n in nodes},set(names),'V3Definitions')
    namespace = dict(np=np,math=math,collections=collections,GateFailure=ValueError,
                     LayerAuthority=SimpleNamespace)
    # Original annotations may refer to other driver types. Postpone their evaluation.
    future = ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future]+nodes,type_ignores=[]))
    exec(compile(module,str(path),'exec'),namespace)
    return namespace


def boundary_view(manifest, domain, coordinates, triangles, records):
    handoff.verify_record(manifest); handoff.verify_record(domain)
    eq(manifest['identity'],HANDOFF,'sealed_handoff')
    eq(domain['identity'],DOMAIN,'sealed_domain')
    eq(array_hash(coordinates),manifest['coordinates'],'coordinates')
    eq(array_hash(triangles),manifest['topology'],'triangles')
    eq(identity(records),manifest['ownership'],'owners')
    need(len(coordinates)==16895 and len(triangles)==len(records)==33786,'InterfacePopulation')
    vertex_ids = [f'layer:11:source:{i}' for i in range(len(coordinates))]
    need(len(set(vertex_ids))==len(vertex_ids),'VertexBijection')
    for tri,row in zip(triangles,records):
        eq([vertex_ids[int(i)] for i in tri],row['vertices'],'LocalGlobalMapping')
        need(row['core_neighbour'] is None and row['core_neighbour_required']==1,'PrematureCoreNeighbour')
    # V3 consumes owner-outward inner faces and negates the sheet in region setup.
    # Read-only views reference the bounded handoff, never complete prism tables.
    coordinates.setflags(write=False); triangles.setflags(write=False)
    return SimpleNamespace(outer_points=coordinates,outer_faces=triangles,
        face_sheet_ids=('prism_core_interface',)*len(triangles),curves=(),corners=(),
        vertex_ids=vertex_ids,source_columns=[r['source_column'] for r in records],
        prism_owners=[r['prism_owner'] for r in records],
        outer_analytic_patches=domain['patches'],outer_mesh=None,generation_authorized=False)


def generation_gate(sizing=None):
    """No qualified capsule sizing authority was recovered by this audit.

    A merely self-labelled record must not authorize generation. A later bounded
    sizing qualification must add a verified authority and its consuming gate.
    """
    if sizing is None:
        raise ValueError('MissingCapsuleSizingAndOuterDiscretizationAuthority')
    raise ValueError('NoQualifiedCapsuleSizingAuthorityInRecoveredV3Path')


def run():
    tick=measured()
    authority=read(handoff.OUT/'AUTHORITY.json'); handoff.verify_record(authority)
    eq(authority['identity'],'55e31c4497e0629b7951f90347a555fbb3510755c00d531d0fe4d6bdf9fe44c5','handoff_authority')
    for name,digest in authority['files'].items(): eq(file_hash(V4/name),digest,'sealed_closure')
    manifest=read(handoff.OUT/'HANDOFF.json'); domain=read(handoff.OUT/'DOMAIN.json')
    t=measured()
    view=boundary_view(manifest,domain,np.load(handoff.OUT/'FRONT.npy'),
        np.load(handoff.OUT/'TRIANGLES.npy'),read(handoff.OUT/'INTERFACE.json'))
    adapter=dict(passed=True,coordinates=array_hash(view.outer_points),topology=array_hash(view.outer_faces),
        vertices=len(view.vertex_ids),faces=len(view.outer_faces),source_columns=len(view.source_columns),
        owners=len(view.prism_owners),point_tag_mapping='local ordinal + 1 <-> exact V4 layer:11:source ID',
        future_outer_tag_mapping='Disjoint dynamically allocated namespace after interface tags; no SF2 hard-coded tags adopted.',
        inner_sheet='prism_core_interface; owner-outward triangles, negative region sheet',
        outer_geometry='Six original analytic patches; no triangulation generated',
        outer_discretization_ready=False,generation_authorized=False,metrics=metrics(*t))
    durable_json(OUT/'ADAPTER.json',adapter)
    baseline=read(FAMILY/'SF2/A/CANONICAL_MESH_IDENTITY.json')
    canonical=''.join(f'{k}:{v}\n' for k,v in sorted(baseline['canonical_members'].items()))
    eq(hashlib.sha256(canonical.encode()).hexdigest(),POLY,'SF2CanonicalMetadataDigest')
    eq(baseline['canonical_sha256'],POLY,'SF2CanonicalReference')
    result=read(FAMILY/'SF2/A/COMPLETE_RESULT.json')
    need(result['assembly']['pass'] and result['foundation']['literal_mesh_ok'],'SF2Provenance')
    sources=[DOWNSTREAM,BASELINE,FAMILY/'fresh_supported_stage.py',FAMILY/'FRESH_BASELINE_CONTRACT.json',
        FAMILY/'bind_coordinate_replay.py',FAMILY/'fresh_corrected_core_stage.py',
        FAMILY/'sf2_continuation_20260911/mesh_complete/REPORT.md',
        FAMILY/'sf2_continuation_20260911/remaining_family/FINAL_AUTHORITATIVE_REPORT.md',
        FAMILY/'SF2/A/CANONICAL_MESH_IDENTITY.json',FAMILY/'SF2/A/COMPLETE_RESULT.json',
        V3/'physical_naca_complete_mesh_qualification/run_core_pipeline.py',
        V3/'farfield_core_transition/GMSH_ALGORITHM1_BOUNDARY_CONTRACT.md',
        V3/'anisotropic_core_transition_qualification/run_naca_refine_core_transition.py',
        V3/'farfield_transition_geometry_qualification_2026-09-08/callback_controls.py']
    sources += [SUPPORTED/n for n in ('common.py','core_stage.py','grid_controls.py','support_control.py','core_audit.py','assemble.py')]
    evidence={str(p.relative_to(V3)):dict(sha256=file_hash(p),bytes=p.stat().st_size) for p in sources}
    definitions={n.name:dict(first_line=n.lineno,last_line=n.end_lineno,
        ast_sha256=hashlib.sha256(ast.dump(n).encode()).hexdigest())
        for n in ast.parse(DOWNSTREAM.read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))
        and n.name in ('setup_gmsh','transfer_boundary','CoreSizeCallback','FoundationInterfaceSizeCallback',
            'foundation_interface_vertex_sizes','transfinite_cube_farfield','run_delaunay',
            'run_gmsh_preconditioner','run_netgen','validate_core','assemble_and_check')}
    missing=[str(p.relative_to(V3)) for p in [
        V3/'farfield_core_transition/FROZEN_FARFIELD_POLICY.json',
        V3/'preconditioned_netgen_sequence/FROZEN_POLICY.json',
        V3/'farfield_transition_geometry_qualification_2026-09-08/FROZEN_EXPERIMENT_CONTRACT.json'] if not p.exists()]
    try: generation_gate()
    except ValueError as exc: blocker=str(exc)
    report=seal(dict(schema='V4RecoveredV3CapsuleCoreAdaptation/1',classification=CLASSIFICATION,
        starting_head=read(OUT/'PREFLIGHT.json')['head'],authoritative_layer=11,
        L11=handoff.EXPECTED,handoff_authority=authority['identity'],handoff=HANDOFF,domain=DOMAIN,
        pipeline=[dict(stage=a,category=b,source=c,decision=d) for a,b,c,d in PIPELINE],
        sources=evidence,definitions=definitions,adapter=adapter,
        SF2_provenance=dict(canonical_polyMesh=POLY,metadata_digest_recomputed=True,
            bulk_polyMesh_not_rehashed=True,counts=baseline['counts'],literal_mesh_ok=True,
            recorded_foundation=result['foundation']['foundation_audit'],
            maximum_skewness=result['foundation']['maximum_skewness_checkMesh'],
            maximum_nonorthogonality=result['foundation']['maximum_nonorthogonality'],
            purpose='Historical regression/provenance only; no capsule target or current rerun.'),
        sizing=dict(adopted_capsule_values=None,source='FRESH_BASELINE_CONTRACT + CoreSizeCallback',
            SF2_only=dict(h0=.0025170808498235424,hmax=.22399999999999998,growth=1.3,
                scale=.64,divisions=66,alpha=.2,cube_bounds=[[-2.5]*3,[2.5]*3]),
            alternative='foundation_interface_vertex_sizes uses face_h=19*dp and vertex incident minimum; guard requires layers==1. Its callback still uses fixed hmax/growth. Not complete authority for L11.',
            missing=['capsule near-interface sizing authority','far-core maximum size and growth authority',
                     'rectangular outer-boundary discretization/target size authority','optional grading intent and parameters'],
            physical_box_is_not_discretization_authority=True),
        missing_historical_policy_files=missing,
        blocker=blocker,backend_incompatibility_proven=False,replacement_mesher_designed=False,
        pending=['outer discretization','core generation','actual interface preservation by backend','prism/core join',
            '67572 determinant/incidence recertifications','33786 shared-face metric records',
            'core and complete hard quality','complete continuous coverage','complete volume balance',
            'capsule checkMesh','complete-mesh publication/fresh reopen'],
        generated=dict(core_vertices=0,interior_vertices=0,tetrahedra=0,core_cells=0,extra_prism_layers=0),
        immutable_prism_proofs_available=33336032,historical_proof_rows_read=0,historical_quality_rows_read=0,
        complete_mesh_identity=None,core_quality=None,new_continuous_certificate=None,
        metrics=metrics(*tick),recommended_next_task='QualifyV3DerivedCapsuleCoreSizingAndOuterBoundaryDiscretizationFromSealedHandoff'))
    durable_json(OUT/'RECOVERY.json',report)
    print(json.dumps(dict(classification=CLASSIFICATION,blocker=blocker,identity=report['identity'],metrics=report['metrics']),indent=2))


if __name__=='__main__': run()
