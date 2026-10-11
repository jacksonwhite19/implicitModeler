"""Seal the bounded integration result, including its unresolved dependencies."""
import json
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from capsule_layer2_preflight import V4,ROOT,SURFACE,save,file_hash,identity
from production_surface_authority import load_production_wall

OUT=V4/'results/production_detailed_lookahead_committed_front'
CLASSIFICATION='V4CommittedPrefixDetailedMetricDomainValidatedProductionAdmissionBlocked'


def seal():
    read=lambda name:json.loads((OUT/name).read_text())
    a=read('RUN_A.json');b=read('RUN_B.json');control=read('CONTROL_CACHE.json');base=read('BASELINE.json');disk=read('DISK_BEFORE.json')
    assert a['scientific']==b['scientific'] and a['scientific_identity']==b['scientific_identity']
    s=a['scientific'];_,_,surface=load_production_wall(SURFACE);suite=ET.parse(OUT/'TESTS_FINAL.xml').getroot().find('testsuite')
    tests={k:suite.attrib[k] for k in ('tests','failures','errors','skipped','time')}
    assert tests['failures']==tests['errors']==tests['skipped']=='0'
    audit=[
      ('patch.prisms layer0/1..20 guard',1,'Generalized only through explicit validated CommittedDomain; legacy unbound entry remains guarded.'),
      ('metrics.measure 20 reshapes/loops/counts and 1..20 IDs',1,'Derived from available absolute forecast.layers; tested at committed layers 0,1,5,19 and windows 1,2,3,10.'),
      ('cell_metrics absolute layer1 bottom-wall mask',2,'Preserved physical wall role; L2 is never renumbered L1.'),
      ('frozen determinant special exposed layer20 cap',1,'Retired role excluded from explicit production domain. Same Foundation tensor primitive uses internal caps; original frozen code untouched.'),
      ('first future cap missing prior cell',2,'Uses restored prefix[-2:] to compute actual committed prior cell centre/volume. Later caps use provisional neighbours.'),
      ('terminal cap neighbour-dependent weight/skew/ratio',2,'No outer neighbour exists; remains unavailable. Internal determinant cap role does not manufacture a centroid or certificate.'),
      ('sigma/G/volume/Jacobian/aspect/side-normal separation',2,'Cell-local unchanged primitives; no scientific twenty-layer horizon found. No new sigma/G gates.'),
      ('rolling_metrics.detailed_events k+1',1,'Now consumes returned absolute cell layer; preserves fallback for legacy unlabelled records.'),
      ('dependency_metrics.LocalMetricEngine fixed20 cache allocation',1,'Legacy optimized engine remains isolated. New bounded exact-patch cache reuses generalized measure; no dense sensitivities or old cache fallback.'),
      ('archive fixed SF2 layer chunks',1,'Historical archive loader not called. Production schedule comes from admitted request.'),
      ('continuous.SideIndex enumerate from1 and layers=20',1,'Not used for production. Still requires prefix/absolute-layer generalization before continuous admission.'),
      ('continuous.SideIndex side_cap_and_cell_overlap_certificate=False',2,'Preserved evidence scope. Side-side pairs cannot certify side-cap/cell overlap; required complete coverage remains unresolved.'),
      ('rolling_spots layer-1/layer-2 indexing and sentinel21',1,'Historical comparison path not entered. Horizon/index generalization remains before production use.'),
      ('rolling_metrics first_events/mechanism_tail sentinel21',1,'Historical correction path not entered; this task does not run correction or replace metric horizons.'),
      ('production_risk.select_supports direct event input',2,'Requires current cell/face hard events and their coverage. Cheap descriptors do not contain those measurements.'),
      ('genuine fixed scientific horizon',3,'None established for direct local metrics. Neighbour availability and complete collision coverage are genuine dependencies, not numeric horizon constants.')]
    save(OUT/'ASSUMPTION_AUDIT.json',[dict(assumption=x,classification=k,disposition=d) for x,k,d in audit])
    sources=['src/sdf_cfd_mesher_v4/forecast/detailed/'+x for x in ('committed_domain.py','metrics.py','patch.py','dependency_metrics.py','continuous.py','rolling_spots.py','archive.py')]
    sources+=['src/sdf_cfd_mesher_v4/advancing_front/'+x for x in ('production_risk.py','rolling_metrics.py','local_relaxation.py')]
    sources+=['src/sdf_cfd_mesher_v4/legacy_full_stack_planner/_frozen/component_quality.py','tools/production_forecast_admission.py','tools/run_committed_detailed_admission.py','tools/qualify_committed_detailed_controls.py','tools/seal_committed_detailed_admission.py','tools/production_first_layer_run.py','tests/test_committed_detailed_domain.py']
    save(OUT/'SOURCE_EVIDENCE.json',{p:file_hash(V4/p) for p in sources})
    status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=ROOT,stderr=subprocess.DEVNULL).decode().splitlines()
    unrelated=[x for x in status if not x[3:].startswith('research/sdf_cfd_mesher_v4/')]
    assert unrelated==base['dirty'],'UnrelatedDirtyInventoryChanged'
    v3=subprocess.check_output(['git','diff','--name-only','--','research/sdf_cfd_mesher_v3'],cwd=ROOT).decode().splitlines()
    assert not v3
    resources=dict(repository_accessible_logical_bytes=disk['repository_accessible_logical_bytes'],v4_accessible_logical_bytes=disk['v4_accessible_logical_bytes'],
        starting_free_bytes=disk['free_bytes'],ending_free_bytes=shutil.disk_usage(ROOT).free,retained_l1_bytes=disk['retained_l1_bytes'],
        task_result_bytes_at_preseal_capture=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),disk_audit_seconds=disk['seconds'],inaccessible_entries=len(disk['errors']),bytes_deleted=0,cleanup='None; no scientific evidence deleted or bulk copied',
        final_pair_seconds=a['total_seconds']+b['total_seconds'],peak_rss=max(a['peak_rss'],b['peak_rss']),
        measured_task_elapsed_seconds=time.time()-base['started'],all_recorded_restore_runs_seconds=sum(read(p.name)['total_seconds'] for p in OUT.glob('RUN_*.json')))
    save(OUT/'RESOURCES.json',resources)
    blockers=['No current production direct cell/face hard-event coverage for production_risk.select_supports; zero cheap events is insufficient.',
              'No authorized local production support follows from available evidence. Whole-shell detailed screening is excluded by this task.',
              'Prefix-aware continuous collision coverage remains unqualified; historical side index omits side-cap/cell-overlap certification and uses layer-zero indexing.',
              'Complete read-only production L2 admission and detailed-path replay therefore remain unqualified.']
    next_task='QualifyV4ProductionRiskCoverageForLocalLookahead'
    lines=['## Decision question','', 'Can existing local detailed lookahead operate from a restored committed production prefix and admit L2 without generating production geometry?','',
      f'Classification: **{CLASSIFICATION}**.','',
      'Supported claim: the restored production L1 and admitted remaining request bind deterministically to a variable-window local metric domain. Synthetic local controls validate absolute indexing, committed cap pairs, internal determinant roles, cache isolation and invalidation. Production L2 admission remains blocked.','',
      'Unsupported claims: production detailed quality, collision certification, L2 geometry, FirstLayer changes, complete AdvancingFrontQualified, and universal future-horizon qualification. No new production risk rule was invented.','',
      '### Authorities','',f"Starting HEAD: `{base['head']}`. Branch: `{base['branch']}`. Native checkpoint restoration, production wall loading, unchanged request decoder and prior report/authority hashes passed on each run. Ending HEAD is the enclosing task commit, printed in the terminal response.",'']
    for k,v in surface.items():
        if 'hash' in k or 'identity' in k:lines.append(f'- wall {k}: `{v}`')
    for k,v in s['binding'].items():lines.append(f'- {k}: `{v}`')
    lines+=['','### Fixed-assumption audit','', '| Assumption | Class | Disposition |','|---|---:|---|']
    lines += [f'| {x} | {k} | {d} |' for x,k,d in audit]
    lines+=['','### Architecture and risk','',
      'The production adapter retains only restored prefix coordinates, not another checkpoint copy. CommittedDomain binds prefix, current front, absolute request window, request/schedule/geometry/surface/topology identities, and implementation sources. The local metric engine uses a complete one-face halo. An exact bounded patch cache binds support and local provisional coordinates. The existing first-layer ExactlyOneLayerScope guard is unchanged. No fixture generator, historical schedule loader, SF2 cohort, percentile, or geometric region is a production selection rule.','',
      'All seven cheap hard-event populations are zero. Direct detailed hard events, risk seed IDs, risk patch counts/sizes/closure, worst production source IDs, and detailed horizon per patch: NotEstablished/NotMeasured. These are not reported as zero risks. The existing selector consumes direct hard-event rows normally produced by evaluate_layer/whole-front screening. The available cheap result has no determinant, weight, skewness, volume ratio, Jacobian, aspect, or continuous-intersection event inventory.','',
      f"Available request horizon: {s['future_layers']}. Pending L2 increment: 0.00017333832030214487 m (`0x1.6b844d56c388ep-13`). Cheap provisional states: {s['provisional_cheap_states']}. Production detailed prisms built: 0. Production detailed support: NotEstablished.",'',
      '### Detailed metrics, gates, and collision evidence','',
      'Production distributions, hard failure counts, worst source IDs, compression metrics, side-normal separation, non-orthogonality, sigma/G and cheap-versus-detailed differences: NotMeasured. No production local stack was built without a justified support. Section curvature remains NotMeasured and is optional.','',
      'Unchanged gates: positive volume and exact Jacobian; determinant >=0.0011; interpolation weight >=0.055; skewness <=3.63636; symmetric volume ratio >=0.01; aspect <=1000; valid topology/ownership; zero forbidden/unresolved continuous geometry. These gates were not geometrically tested on L2. Sigma/G remain diagnostics.','',
      'Analytical wedge-Jacobian extrema retain the frozen binary64 primitive; this is not a new exact-dyadic sign certificate. First future cap pairs use the last actual committed cell. Terminal provisional determinant uses an internal cap, while its missing neighbour-dependent pair metrics remain unavailable. Historical side-side certification does not cover all required collision relationships. No sampled checks substitute for continuous evidence. Production candidates/certified/forbidden/unresolved counts: NotMeasured.','',
      '### Cheap descriptor evidence','', '| L2 diagnostic | Minimum | Median | Maximum |','|---|---:|---:|---:|']
    for group,key in [('vertex','step_length'),('vertex','residual'),('edge','length'),('edge','ratio'),('triangle','area'),('triangle','ratio')]:
        r=s['cheap_l2_descriptors'][group][key];lines.append(f"| {group}/{key} | {r['min']} | {r['median']} | {r['max']} |")
    lines+=['','### Cache and replay','',
      f"Production detailed cold/warm/invalidation work: NotMeasured; entries=0. This is not evidence of a successful detailed cache workload. Synthetic controls only: {control['counts']}. Two disjoint test supports; cold times {control['cold_a']['seconds']} and {control['cold_b']['seconds']} s (one miss each); warm {control['warm']['seconds']} s (one hit). Invalidation source vertex 0 removed one entry, retained one, recomputed 12 provisional test prisms in {control['recompute']['seconds']} s. Outside-support and recomputed metrics are exact; independent test replay is exact. Test geometry is not a production risk selection.",'',
      f"Final production read-only runs: {a['total_seconds']} and {b['total_seconds']} s. Scientific equality: exact. This replay covers restore, cheap forecast, domain binding and blocked decision, not an unexecuted production detailed stack.",'']
    for key in ('committed_prefix_hash','remaining_schedule','cheap_forecast','domain_identity','risk_identity','decision_identity','detailed_cache_identity'):
        lines.append(f'- {key}: `{s[key]}`')
    lines += [f"- complete run scientific identity: `{a['scientific_identity']}`",'',
      '### Tests','',f"Final suite: {tests}. 25 new focused tests plus 43 existing detailed, first-layer role and production forecast tests. Two expected divide-by-zero warnings from the existing singular-cell test. An initial new test incorrectly assumed a symmetric control distinguished two tensor formulas; it was corrected to assert that the retired function cannot execute. No existing test was weakened.",'',
      '### Resources and preservation','',json.dumps(resources,sort_keys=True), '',
      f"Phase times A: {a['timings']}; B: {b['timings']}. Cheap cache A: {a['cheap_cache']}. Peak RSS applies to each measured scientific process, not a machine-wide memory total. Retained RSS A/B: {a['retained_rss']}/{b['retained_rss']} bytes. Earlier pre-final-source runs remain as small evidence records. No large array or checkpoint copy was retained.",'',
      'L2 generated=NO; L2 committed=NO; L3 generated=NO; core generated=NO; provisional committed fronts=0. Authoritative committed layer=1. L1 identity, wall, and request unchanged. No publisher or commit path ran. Unrelated dirty entries remain 155; V3 tracked modifications=0. SOURCE_EVIDENCE.json distinguishes inspected evidence paths by their hashes; the enclosing Git commit lists task-owned changed paths.','',
      '### Blockers','']
    lines += ['- '+x for x in blockers]
    lines += ['',f'Exactly one recommended next task: **{next_task}**. Establish the minimum complete direct-event coverage and its admissible local-support rule before attempting production detailed/collision admission. Do not generate L2 or L3.','']
    (OUT/'report.md').write_text('\n'.join(lines))
    evidence={p.name:file_hash(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','SEALS.json')}
    authority=dict(classification=CLASSIFICATION,supported_claim=lines[6],production_admission_qualified=False,
        production_detailed_metrics_measured=False,wall_authority=surface,scientific=s,scientific_identity=a['scientific_identity'],replay_exact=True,
        tests=tests,resources=resources,blockers=blockers,next_task=next_task,evidence=evidence,
        git=dict(starting_head=base['head'],branch=base['branch'],unrelated_dirty_before=155,unrelated_dirty_after=155,v3_modifications=0),
        important_prior_authorities=dict(report='7142551272964625bd847857f12f2d4c7292ef410a3748f7d581348b691be8d0',authority='b759737a40052886832d6b53564f3825f9b2eeb68621afc8942625f11bacae81'))
    save(OUT/'AUTHORITY.json',authority)
    save(OUT/'SEALS.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json'),domain=s['domain_identity'],scientific=a['scientific_identity']))
    print(json.dumps(dict(seals=read('SEALS.json'),resources=resources,tests=tests),indent=2))


if __name__=='__main__':seal()
