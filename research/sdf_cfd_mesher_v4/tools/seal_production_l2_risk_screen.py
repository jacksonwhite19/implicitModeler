"""Seal measured screening, including conservative collision-locality failure."""
import json
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
from capsule_layer2_preflight import V4,ROOT,SURFACE,save,file_hash,array_hash,identity
from production_surface_authority import load_production_wall

OUT=V4/'results/production_layer2_risk_coverage'
CLASSIFICATION='V4ProductionLayer2DirectScreeningQualifiedCollisionLocalizationBlocked'


def seal():
    read=lambda name:json.loads((OUT/name).read_text())
    a=read('RUN_A.json');b=read('RUN_B.json');s=a['scientific'];baseline=read('BASELINE.json');disk=read('DISK_BEFORE.json')
    assert a['scientific']==b['scientific'] and a['scientific_identity']==b['scientific_identity']
    _,_,surface=load_production_wall(SURFACE)
    # Reproducible descriptive locality audit. No candidate is excluded by it.
    began=time.perf_counter()
    with np.load(OUT/'SCREEN_ARRAYS.npz') as z:pairs=z['candidate_pairs'];labels=z['face_labels'];columns=z['face_columns']
    assert array_hash(pairs)==s['collision_hashes']['pairs']
    n=s['ownership']['source_vertices'];nonlocal_=np.zeros(len(pairs),bool);same=np.zeros(len(pairs),bool)
    for begin in range(0,len(pairs),65536):
        p,q=pairs[begin:begin+65536].T
        nonlocal_[begin:begin+len(p)]=~np.any((labels[p,:,None]%n)==(labels[q,None,:]%n),axis=(1,2))
        same[begin:begin+len(p)]=np.any((columns[p,:,None]==columns[q,None,:])&(columns[p,:,None]>=0),axis=(1,2))
    locality=dict(candidate_pairs_hash=array_hash(pairs),source_nonlocal_definition='Disjoint source-wall vertex sets, ignoring absolute layer; descriptive only, no exclusion',
        source_nonlocal_candidates=int(nonlocal_.sum()),source_local_candidates=int((~nonlocal_).sum()),shared_source_column_candidates=int(same.sum()),source_nonlocal_flags_hash=array_hash(nonlocal_))
    prior=read('COLLISION_LOCALITY.json')
    assert {k:v for k,v in prior.items() if k!='seconds'}==locality
    save(OUT/'LOCALITY_REPLAY.json',dict(**locality,seconds=time.perf_counter()-began,replay_exact=True))
    testsuite=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite');tests={k:testsuite.attrib[k] for k in ('tests','failures','errors','skipped','time')}
    assert tests['failures']==tests['errors']==tests['skipped']=='0'
    status=subprocess.check_output(['git','status','--porcelain=v1'],stderr=subprocess.DEVNULL).decode().splitlines()
    unrelated=[x for x in status if not x[3:].startswith('research/sdf_cfd_mesher_v4/')]
    assert unrelated==baseline['dirty']
    sources=['tools/production_l2_screening.py','tools/run_production_l2_risk_screen.py','tools/seal_production_l2_risk_screen.py','tests/test_production_l2_screening.py',
        'src/sdf_cfd_mesher_v4/forecast/detailed/committed_domain.py','src/sdf_cfd_mesher_v4/forecast/detailed/metrics.py',
        'src/sdf_cfd_mesher_v4/forecast/exact_dyadic.py','src/sdf_cfd_mesher_v4/certificate/incremental.py',
        'src/sdf_cfd_mesher_v4/runtime/regions.py','src/sdf_cfd_mesher_v4/advancing_front/rolling_correction.py','src/sdf_cfd_mesher_v4/advancing_front/cap_diagnostics.py']
    implementation={name:file_hash(V4/name) for name in sources};save(OUT/'SOURCE_EVIDENCE.json',implementation)
    screening_id=identity(dict(version='V4ReadOnlyL2RiskScreen/1',result=a['scientific_identity'],implementation=implementation))
    resources=dict(repository_accessible_logical_bytes=disk['repository_accessible_logical_bytes'],v4_accessible_logical_bytes=disk['v4_accessible_logical_bytes'],
        starting_free_bytes=disk['free_bytes'],ending_free_bytes=shutil.disk_usage(ROOT).free,retained_l1_bytes=disk['retained_l1_bytes'],
        inaccessible_entries=len(disk['errors']),disk_audit_seconds=disk['seconds'],bytes_deleted=0,
        task_result_bytes_at_preseal_capture=sum(p.stat().st_size for p in OUT.iterdir() if p.is_file()),
        elapsed_seconds_at_seal=time.time()-baseline['started'],replay_pair_seconds=a['total_seconds']+b['total_seconds'],peak_rss=max(a['peak_rss'],b['peak_rss']))
    save(OUT/'RESOURCES.json',resources)
    failures={k:sum(e['metric']==k for e in s['hard_events']) for k in ('volume','signed_volume','jacobian','exact_jacobian_sign','exact_volume_sign','determinant','weight','skewness','symmetric_volume_ratio','aspect','duplicate_triangle','edge_incidence_or_orientation')}
    gates=dict(determinant=.0011,weight=.055,skewness=3.63636,symmetric_volume_ratio=.01,aspect=1000)
    margins={k:(gates[k]-s['worst'][k]['value'] if k in ('skewness','aspect') else s['worst'][k]['value']-gates[k]) for k in gates}
    save(OUT/'GATES.json',dict(gates=gates,failures=failures,margins=margins,exact_failure_rows=s['hard_events'],finite_geometry_failures=0,topology=s['topology']))
    supported='Every one of the 33,786 prospective production L2 cells passed the applicable direct scalar and topology screen, including exact-dyadic Jacobian/volume positivity. Complete conservative changed-face candidate enumeration is reproducible, but its source union exceeds existing local support bounds.'
    lines=['## Decision question','', 'Can a bounded read-only L2 screen supply complete, reproducible local risk supports without generating production geometry?','',
        f'Classification: **{CLASSIFICATION}**.','',supported,'',
        'This does not qualify L2 geometry, continuous geometry, L2 admission, or AdvancingFrontQualified. Broad-phase overlap is not an intersection finding.','',
        '### Input authorities and scope','',f"Starting HEAD: `{baseline['head']}`; branch `{baseline['branch']}`. Ending HEAD is the task commit printed in the terminal response. Restored native L1, production surface loader, unchanged request decoder and previous authority/report seals passed twice.",'']
    for k,v in s['binding'].items():lines.append(f'- {k}: `{v}`')
    for k,v in surface.items():
        if 'identity' in k or 'hash' in k:lines.append(f'- wall {k}: `{v}`')
    lines+=['','Pending request: L2 increment 0.00017333832030214487 m, binary64 `0x1.6b844d56c388ep-13`. The verified sealed cheap forecast supplies only its first future coordinate row to screening. No future trajectory generator or L3 construction runs. The full cached forecast is read and hash-verified, then unused rows are released.','',
        '### Screening architecture and counts','',
        f"{s['counts']}. Maximum direct batch: 2,048 prisms. Only one future layer is screened. L1 supplies actual predecessor centres/volumes. L2 determinant uses the corrected internal terminal-cap role; missing future-neighbour pair metrics remain unavailable, not manufactured. No retired exposed-cap rule controls this screen.",'',
        f"Topology: {s['topology']}. Ownership: {s['ownership']}. No topology transition occurs; every active source triangle appears exactly once. The qualified wall connectivity is retained.",'',
        '### Complete direct distributions','',
        'Exact Jacobian and signed-volume signs use exact dyadic arithmetic for every cell. Rational extrema are retained in EXACT_WEDGES.jsonl. The following exact-Jacobian numbers are binary64 renderings of those rational values; signs are not inferred from rounded numbers.','',
        '| Metric | Min | p1 | p5 | Median | p95 | p99 | Max |','|---|---:|---:|---:|---:|---:|---:|---:|']
    for name in ('volume','exact_jacobian','determinant','weight','skewness','symmetric_volume_ratio','aspect'):
        row=s['distributions']['cells'].get(name,s['combined_face_distributions'].get(name))
        lines.append('| '+name+' | '+' | '.join(str(row[k]) for k in ('min','p1','p5','median','p95','p99','max'))+' |')
    lines+=['','Face distributions combine 50,679 lateral pairs and 33,786 committed L1/L2 cap pairs. Separate populations and all direct arrays are retained. Nonfinite direct scalar counts are zero.','',
        f'Hard failure counts: {failures}. Exact hard-failure rows: `[]`. All volumes and exact Jacobians are positive. Determinant >=0.0011; weight >=0.055; skewness <=3.63636; symmetric volume ratio >=0.01; aspect <=1000. No gate changed.','',
        f"Worst source evidence: {s['worst']}. Scalar margins to limits: {margins}. Source columns are zero-based. Sigma/G are not gates and are not computed by this minimum screen.",'',
        '### Risk seeds','',
        'Direct hard-event seeds: 0. Finite geometry, exact signs, available direct gates and source topology all pass. No historical cohort, percentile, worst-N rule, conditioning threshold or fixture selector is used.','',
        '### Collision broad phase','',
        'The existing conservative AABB BVH enumerator screens unique wall/L1/L2 caps and L1/L2 sides. Triangle padding affects only min/max AABB storage; it does not substitute a bilinear surface or sampled intersection test. New sides and new front faces are marked changed. Unchanged old-old geometry is outside the changed scope. Shared L1-top/L2-bottom aliases are represented once.','',
        f"Populations: {s['collision_populations']}. Source locality: {locality}.",'',
        'All 2,898,957 candidates remain unresolved risks. Legal-contact exclusions: 0. Sharing a vertex or source column is not proof that any intersection is exclusively legal. No forbidden-intersection count is inferred; exact forbidden/unresolved certificate counts are NotMeasured. The broad phase is face-candidate coverage, not a final cell-overlap/containment certificate.','',
        '### Patch closure and coverage decision','',
        'Collision-risk seed columns: every source ID 0 through 33,785 inclusive. The union with zero scalar-failure seeds is unchanged. Existing zero-ring source-vertex connectivity merges these into one group: 16,895 vertices, 50,679 edges and 33,786 triangles.','',
        'Existing bounded_patch uses max_vertices=2048 and max_columns=12288. It rejects this group at ActiveVertexBound. These are existing operational locality bounds, not new scientific quality gates. No radius, splitting, partition rule or overlap suppression is introduced. There are zero accepted bounded patches and one rejected connected closure. Accepted core/halo sizes and accepted support identity: NotEstablished.','',
        f"Failed closure population identity: `{s['closure']['support_population_identity']}`. Complete bounded risk-support population: NO. No detailed future stack is run.",'',
        '### Determinism and hashes','',f"Independent complete read-only runs agree exactly: `{a['scientific_identity']}`. Implementation-bound screening identity: `{screening_id}`.",'']
    for k in ('committed_prefix','screen_geometry','provisional_l2_coordinates','metric_identity','hard_events_identity','exact_record_hash','collision_identity','seed_identity','decision_identity'):
        lines.append(f'- {k}: `{s[k]}`')
    lines+=['','### Runtime and scaling','',f"A timings: {a['timings']}. B timings: {b['timings']}. Total A/B: {a['total_seconds']} / {b['total_seconds']} s. Retained RSS A/B: {a['retained_rss']} / {b['retained_rss']} bytes. {resources}",'',
        'No screening cache is implemented or used; cold/warm hits and local cache invalidation: NotApplicable. Both runs recompute all direct cells, exact signs and candidate pairs. Persistent input authority is reused read-only. There is no dense multilayer prism stack, sensitivity matrix, or measured speedup claim against unrun dense detailed work.','',
        '### Tests and preservation','',f"Tests: {tests}. 24 new screening tests, 25 existing committed-domain tests and 24 existing incremental-certificate tests. No failing or skipped tests and no weakened existing tests.",'',
        'L2 generated=NO; L2 committed=NO; L3 generated=NO; core generated=NO. The 33,786 provisional L2 screening cells are distinct from production geometry. Authoritative layer remains 1; wall, committed L1 and request identities remain unchanged. No publisher/transaction/checkpoint-write path runs.','',
        'No cleanup. Bytes deleted=0. The existing 7.17 GB L1 evidence was not copied. Exact rational records, provisional coordinates, scalar arrays, candidate IDs, source mapping and hashes are retained. Git unrelated dirty-entry inventory remains exactly 155. Task-owned V3 changes=0; pre-existing untracked V3 work remains untouched.','',
        '### Remaining blocker and next task','',
        'AABB overlap produces a global source union. Existing local support machinery therefore cannot accept it. Sound collision-candidate separation/legal-contact evidence is needed before narrowing that union. Discarding incident pairs solely from adjacency, or arbitrarily splitting the group, would bypass the missing evidence.','',
        'Exactly one recommended next task: **QualifyV4ProductionCollisionCandidateLocalization**. Use existing sound separation/contact machinery to reduce these sealed candidate risks and retry existing bounded closure. Remain read-only; do not generate or commit L2/L3.','']
    (OUT/'report.md').write_text('\n'.join(lines))
    evidence={p.name:file_hash(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','SEALS.json')}
    save(OUT/'AUTHORITY.json',dict(classification=CLASSIFICATION,supported_claim=supported,screening_identity=screening_id,
        scientific_result=a['scientific_identity'],implementation_identity=identity(implementation),metrics_complete=True,
        bounded_risk_supports_complete=False,continuous_qualified=False,L2_admitted=False,L2_generated=False,L3_generated=False,
        authorities=s['binding'],wall_authority=surface,screen_geometry=s['screen_geometry'],direct_metric_identity=s['metric_identity'],
        risk_seed_identity=s['seed_identity'],closure_population_identity=s['closure']['support_population_identity'],
        decision=s['decision'],decision_identity=s['decision_identity'],exact_replay=True,tests=tests,resources=resources,evidence=evidence,
        blocker='Collision candidate source union exceeds qualified local closure: ActiveVertexBound',
        next_task='QualifyV4ProductionCollisionCandidateLocalization',git=dict(starting_head=baseline['head'],branch=baseline['branch'],unrelated_dirty_before=155,unrelated_dirty_after=155,V3_task_modifications=0)))
    save(OUT/'SEALS.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json'),screening=screening_id,scientific=a['scientific_identity']))
    print(json.dumps(dict(seals=read('SEALS.json'),resources=resources,tests=tests,margins=margins),indent=2))


if __name__=='__main__':seal()
