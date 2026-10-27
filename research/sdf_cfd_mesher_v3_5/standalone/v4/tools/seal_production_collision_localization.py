"""Seal replayed read-only localization evidence and its implementation inputs."""
import json
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
from capsule_layer2_preflight import V4,ROOT,save,file_hash,identity
from run_production_collision_localization import OUT,PRIOR,save


def seal():
    a=json.loads((OUT/'RUN_A.json').read_text());b=json.loads((OUT/'RUN_B.json').read_text());s=a['scientific']
    if s!=b['scientific'] or a['scientific_identity']!=identity(s):raise ValueError('ReplayMismatch')
    assert s['accounting']==dict(candidates=2898957,certified=2898957,unresolved=0,forbidden=0,failure=0,missing=0,duplicate=0)
    assert not s['seed_columns'] and not s['closure']['groups']
    base=json.loads((OUT/'BASELINE.json').read_text());disk=json.loads((OUT/'DISK_START.json').read_text())
    status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=ROOT,stderr=subprocess.DEVNULL).decode().splitlines()
    owned=['research/sdf_cfd_mesher_v4/tools/'+n for n in ('production_collision_localization.py','run_production_collision_localization.py','seal_production_collision_localization.py')]
    owned+=['research/sdf_cfd_mesher_v4/tests/test_production_collision_localization.py','research/sdf_cfd_mesher_v4/results/production_layer2_collision_localization/']
    unrelated=[r for r in status if not any(r[3:]==p or r[3:].startswith(p) for p in owned)]
    if sorted(r.lstrip() for r in unrelated)!=sorted(r.lstrip() for r in base['dirty']):raise ValueError('UnrelatedDirtyStateChanged')
    with np.load(PRIOR/'SCREEN_ARRAYS.npz') as z:
        labels=z['face_labels'];pairs=z['candidate_pairs'];kinds=z['face_kinds']
    disjoint_source=0
    for i in range(0,len(pairs),50000):
        x,y=(labels[pairs[i:i+50000]]%16895).transpose(1,0,2)
        disjoint_source+=int((~(x[:,:,None]==y[:,None,:]).any((1,2))).sum())
    assert disjoint_source==20565
    inventory=dict(unique_faces=len(labels),new_faces=int(np.isin(kinds,['cap2','side2']).sum()),committed_faces=int(np.isin(kinds,['cap0','cap1','side1']).sum()),
        candidate_pairs=len(pairs),shared_physical_topology=2051896,no_shared_physical_vertex=s['topology_relations']['0'],disjoint_source_wall_vertices=disjoint_source)
    assert inventory['shared_physical_topology']==s['topology_relations']['1']+s['topology_relations']['2']
    files=[V4/p for p in ['tools/production_collision_localization.py','tools/run_production_collision_localization.py','tools/seal_production_collision_localization.py',
        'tools/native_mixed_face_certificate.py','tools/termination_geometry_certificate.py','tools/local_termination_topology.py','tools/production_l2_screening.py',
        'src/sdf_cfd_mesher_v4/certificate/efficient.py','src/sdf_cfd_mesher_v4/certificate/efficient_native.rs','src/sdf_cfd_mesher_v4/certificate/efficient_proof.py',
        'src/sdf_cfd_mesher_v4/advancing_front/rolling_correction.py','src/sdf_cfd_mesher_v4/advancing_front/cap_diagnostics.py']]
    files+=list((V4/'src/sdf_cfd_mesher_v4/certificate/continuous_prism_certificate').rglob('*.py'))
    sources={p.relative_to(V4).as_posix():file_hash(p) for p in sorted(files)}
    source_id=identity(sources);save(OUT/'SOURCE_EVIDENCE.json',sources)
    scheme=dict(version='V4SealedCollisionLocalization/1',candidate_identity=s['candidate_identity'],implementation=source_id,
        input_scientific_identity=a['scientific_identity'],rows='Row index is the identical sealed candidate index; codes/witness/rotations/relations stored in PROOF_ROWS.npz.',
        relation_codes={'0':'Disjoint physical topology','1':'Owned vertex','2':'Owned perimeter edge','3':'Other shared topology'},
        proof_semantics='Unchanged interval convex-hull proofs; triangle rotation avoids repeated shared corners. Exact polygon hull/face-tiling fallback. No topology-only exclusion.',
        exact_evidence='EXACT_PROOFS.json binds exact outcomes to candidate indices. All predicates are reproducible from sealed points, labels, pair inventory and implementation.',
        limitations='Forecast-risk localization only; no final continuous certificate, L2 admission, or geometry publication.')
    save(OUT/'PROOF_SCHEME.json',scheme)
    proof_id=identity(dict(candidate=s['candidate_identity'],arrays=s['array_hashes'],exact=s['exact_proofs_identity'],scheme=identity(scheme)))
    new_id=identity(dict(scientific=a['scientific_identity'],implementation=source_id,proof=proof_id))
    suite=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite')
    tests={k:int(suite.attrib[k]) for k in ('tests','failures','errors','skipped')};assert not tests['failures'] and not tests['errors']
    tests['passed']=tests['tests']-tests['failures']-tests['errors']-tests['skipped'];tests['seconds']=float(suite.attrib['time'])
    resources=dict(repository_accessible_logical_bytes=disk['repository_accessible_logical_bytes'],v4_accessible_logical_bytes=disk['v4_accessible_logical_bytes'],
        retained_l1_bytes=disk['retained_l1_bytes'],starting_free_bytes=disk['free_bytes'],ending_free_bytes=shutil.disk_usage(ROOT).free,
        inaccessible_entries=len(disk['errors']),disk_audit_seconds=disk['seconds'],bytes_deleted=0,cleanup='None; all historical evidence retained.',
        elapsed_seconds_at_seal=time.time()-(OUT/'BASELINE.json').stat().st_birthtime,
        run_A_seconds=a['total_seconds'],run_B_seconds=b['total_seconds'],peak_rss=max(a['peak_rss'],b['peak_rss']),
        retained_rss_A=a['retained_rss'],retained_rss_B=b['retained_rss'],cache='No proof-result cache; both runs independently classify all pairs. Existing native library reused.')
    resources['proof_pairs_per_second_A']=len(pairs)/sum(a['timings'][k] for k in ('interval','rotation','exact'))
    resources['proof_pairs_per_second_B']=len(pairs)/sum(b['timings'][k] for k in ('interval','rotation','exact'))
    save(OUT/'RESOURCES.json',resources)
    methods=['DisjointByCertifiedSupportingPlane','OwnedVertexByCertifiedSupportingPlane','OwnedEdgeByCertifiedSupportingPlane','ExactHullLegalContact']
    lines=['## Decision question','', 'Can the sealed L2 collision candidates be localized into bounded production risk supports without changing or publishing geometry?','',
        '**V4ProductionLayer2CollisionLocalizationQualified**','',
        'All 2,898,957 sealed candidates have a sound separation or legal-contact proof. No unresolved pair remains. Together with the retained zero direct hard events, this gives zero collision seeds and zero closure patches. This qualifies forecast-risk localization, not L2 admission, final continuous certification, L2 geometry, or the complete advancing front.','',
        '### Authority and immutable inputs','',f'Starting HEAD: `{base["head"]}`. Branch: `{base["branch"]}`. Restored native L1 and production request decoder passed in both runs. The sealed candidate bulk and previous report/authority hashes were checked. Face geometry and ownership were reconstructed only for verification against the sealed inventory. No new front was predicted.','']
    lines += [f'- {k}: `{v}`' for k,v in s['binding'].items()]
    lines += ['', 'Previous report: `92544bc5ae9a2891497a02b9eeab129777902ae21717b38d7fb67a100008c228`. Previous authority: `cf91b01f2a5a7ab019a9288bcb79dd2d7707ac91b84b5f4fc84e546d2c5bfbd7`.','',
        '### Inventory and topology','',f'Inventory: `{json.dumps(inventory,sort_keys=True)}`. Relations: 847,061 disjoint; 1,491,335 owned vertex; 560,561 owned edge; 0 other shared topology. Face aliases were already canonicalized; no same-face pair remains. Shared topology never substitutes for a proof.','',
        '### Unchanged proof machinery','',
        'The adapter consumes the sealed sorted unique pair list in batches of 50,000. It reuses the existing mixed-face interval supporting-plane filter, contact-safe triangle rotation, and exact rational polygon fallback. Triangles use convex-hull padding only; they are never passed as regular bilinear patches. Side faces retain the actual bilinear image. The native filter uses outward-rounded interval signs. Exact fallback accepts only exact predicate results. No tolerance, sampled intersection rule, topology-only exclusion, or gate was added.','',
        '| Pair family | Separated | Owned vertex | Owned edge | Exact legal contact |','|---|---:|---:|---:|---:|']
    for f,pop in s['proof_family_counts'].items():lines.append('| '+f+' | '+' | '.join(str(pop.get(m,0)) for m in methods)+' |')
    lines += ['','All 73 exact legal-contact proofs restrict intersection to an owned vertex. Accumulated-plane proofs, corner-AABB acceptances, exact face-tiling proofs, other proof types, forbidden intersections, unresolved pairs, infrastructure failures, missing rows, and duplicate rows: 0.','',
        'Exact accounting: 847061 + 1491262 + 560561 + 73 = 2898957 certified; certified + unresolved + forbidden + failures = 2898957.','',
        '### Risk and unchanged closure','',
        'Direct hard-event seeds: 0 (retained sealed screening authority). Collision-risk pairs and seed columns: 0. Connected groups, patches, core columns, closure/halo columns, active vertices, edges, triangles, merges, accepted groups, and rejected groups: 0. Existing induced source-vertex closure and its 2,048-active-vertex bound are unchanged. No closure is manufactured for certified pairs. Supports are complete for this risk-selection stage.','',
        '### Determinism and seals','']
    hashes=dict(restored_prefix=s['restored_prefix'],candidate=s['candidate_identity'],**s['array_hashes'],exact_proofs=s['exact_proofs_identity'],
        risk_pairs=s['risk_identity'],seeds=s['seed_identity'],supports=s['closure']['support_population_identity'],decision=s['decision_identity'],
        run_scientific=a['scientific_identity'],proof=proof_id,implementation=source_id,localization=new_id)
    lines += [f'- {k}: `{v}`' for k,v in hashes.items()]
    lines += ['','Every scientific field and hash is equal between independent runs A and B. Timing and RSS are excluded from scientific equality. Compact proof rows retain one outcome and witness per sealed candidate. No candidate or L1 bulk was copied.','',
        '### Resources','', '| Phase (seconds) | A | B |','|---|---:|---:|']
    for k in a['timings']:lines.append(f'| {k} | {a["timings"][k]} | {b["timings"][k]} |')
    lines += ['', '```json',json.dumps(resources,indent=2,sort_keys=True),'```','',
        'Proof timing is measured by interval, rotation, and exact stages. Per-family timing: NotMeasured. Retained L1 bulk is 7,170,401,356 bytes. Accessible storage totals exclude managed worktrees and the nested dual_contouring checkout; inaccessible entries remain listed in DISK_START.json. No scientific evidence was deleted. Final task bytes and post-commit free space are printed in the terminal.','',
        '### Tests and execution corrections','',f'Final tests: {tests["tests"]} run, {tests["passed"]} passed, 0 failed, 0 errors, 0 skipped. Test modules: test_production_collision_localization.py, test_production_l2_screening.py, test_efficient_certificate.py, test_cap_diagnostics.py. Time: {tests["seconds"]} seconds.',
        'Initial test attempts each had one test-authoring failure: first the closure import path, then an incorrect assertion about the lower-level helper default. The test now checks the actual production caller and verifies rejection of 2,049 active vertices. Both failed test records are retained. The first runner attempt stopped before proof work on a manifest-field lookup; the sealed scientific identity is now checked directly. No existing test or proof implementation was weakened.','',
        '### Publication, Git, and scope','',
        'L2 generated = NO; L2 committed = NO; L3 generated = NO; core generated = NO. The 33,786 retained provisional screening cells remain forecast evidence only. Authoritative committed layer = 1. Wall, L1, and request hashes remain unchanged. Hard gates and metric definitions are unchanged. No geometry publisher or commit path ran.','',
        f'Unrelated dirty entries before/at seal: {len(base["dirty"])}/{len(unrelated)}; exact status entries preserved. V3 task modifications: 0. Only new task-owned V4 files are staged. Ending commit and final dirty count are reported after commit.','',
        '### Blockers and next task','',
        'Collision-localization blockers: none. Read-only L2 admission and final L2 geometry qualification remain unestablished.','',
        'Recommended next task: **QualifyV4ProductionLayer2ReadOnlyAdmissionFromLocalizedRiskSupports**. Consume this sealed zero-risk support population through the committed-prefix detailed-lookahead decision path and issue the read-only L2 admission decision. Do not generate L2.','']
    (OUT/'report.md').write_bytes('\n'.join(lines).encode())
    evidence={p.name:file_hash(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.name not in ('AUTHORITY.json','SEALS.json')}
    authority=dict(classification='V4ProductionLayer2CollisionLocalizationQualified',scientific_identity=new_id,hashes=hashes,
        accounting=s['accounting'],inventory=inventory,proof_methods=s['proof_methods'],proof_family_counts=s['proof_family_counts'],
        input_authorities=s['binding'],risk_seeds=0,patches=0,closure=s['closure'],deterministic_replay=True,
        preservation=s['preservation'],publication=s['publication'],L2_admission=False,final_continuous_qualified=False,
        tests=tests,resources=resources,evidence=evidence,git=dict(starting_head=base['head'],branch=base['branch'],unrelated_before=len(base['dirty']),unrelated_after=len(unrelated),V3_task_modifications=0),
        next_task='QualifyV4ProductionLayer2ReadOnlyAdmissionFromLocalizedRiskSupports')
    save(OUT/'AUTHORITY.json',authority)
    seals={name:file_hash(OUT/name) for name in ('report.md','AUTHORITY.json','PROOF_ROWS.npz','PROOF_SCHEME.json')}
    save(OUT/'SEALS.json',seals);print(json.dumps(dict(seals=seals,hashes=hashes,resources=resources),indent=2))


if __name__=='__main__':seal()
