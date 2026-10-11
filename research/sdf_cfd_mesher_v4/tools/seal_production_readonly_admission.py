"""Seal the bounded admission decision and its source-based requirement audit."""
import json,subprocess,time,shutil
import xml.etree.ElementTree as ET
from capsule_layer2_preflight import V4,ROOT,file_hash,identity
from run_production_collision_localization import save
from run_production_readonly_admission import OUT


def seal():
    a=json.loads((OUT/'RUN_A.json').read_text());b=json.loads((OUT/'RUN_B.json').read_text());s=a['scientific']
    if s!=b['scientific'] or identity(s)!=a['scientific_identity']:raise ValueError('ReplayMismatch')
    if s['decision']['status']!='ProductionLayer2AdvanceAuthorized':raise ValueError('AdmissionBlocked')
    base=json.loads((OUT/'BASELINE.json').read_text());disk=json.loads((OUT/'DISK_START.json').read_text())
    owned=['tools/production_readonly_admission.py','tools/run_production_readonly_admission.py','tools/seal_production_readonly_admission.py','tests/test_production_readonly_admission.py']
    roots=['research/sdf_cfd_mesher_v4/'+p for p in owned]+['research/sdf_cfd_mesher_v4/results/production_layer2_readonly_admission/']
    status=subprocess.check_output(['git','status','--porcelain=v1'],stderr=subprocess.DEVNULL).decode().splitlines()
    unrelated=[r for r in status if not any(r[3:]==p or r[3:].startswith(p) for p in roots)]
    if sorted(r.lstrip() for r in unrelated)!=sorted(r.lstrip() for r in base['dirty']):raise ValueError('UnrelatedStateChanged')
    audit=[
      ['Restored committed L1 and source ownership','Scientific','Satisfied','Native checkpoint restoration, production binding; sealed committed L1 and wall identities.'],
      ['Admitted full request and exact pending increment','Scientific','Satisfied','Unchanged request decoder; global L2 increment 0x1.6b844d56c388ep-13.'],
      ['Complete cheap forecast over remaining request','Scientific','Satisfied','Qualified L2-L11 forecast hash and seven hard-event counters verified from sealed arrays; all zero.'],
      ['Complete immediate direct hard-metric population','Scientific','Satisfied','33,786 finite prisms, 50,679 lateral pairs, 33,786 committed-cap pairs; sealed array hashes and zero hard-event rows.'],
      ['Topology and ownership','Scientific','Satisfied','Complete source-column inventory; no missing, duplicate, orphan, foreign, or topology event.'],
      ['Collision candidate and proof completeness','Scientific','Satisfied','2,898,957 candidates and equally many certified proof rows; zero forbidden/unresolved/failure/missing/duplicate.'],
      ['Evidence-derived risk supports','Scientific','Satisfied','Zero direct/collision seeds and zero patches, sealed and matched to the current binding.'],
      ['Detailed work for measured risk patches','Conditional scientific','Satisfied: no-op','production_risk.select_supports returns no patches or traces for empty event sets; task4_prepare_next iterates only selected patches and permits nominal-only empty logs.'],
      ['Dense direct future metrics for every nominal step','Not an unconditional requirement','NotApplicable','Future supports are optional guidance (required=False); future direct metrics are NotMeasured, not asserted clear.'],
      ['No new earlier future failures after correction','Conditional scientific','NotApplicable','rolling_metrics.future_admission and future spot comparisons gate correction trials. No correction or trajectory change occurs.'],
      ['Layer 0, exactly 20 future layers, fixed integration-blocked status','Historical implementation artifacts','NotApplicable','Committed-prefix metric domain already qualified. Old production_forecast_admission.admission_result is a dated fixed blocker, not a metric predicate.'],
      ['Final generated-geometry quality, exact signs, continuous proof and transaction','Scientific publication requirements','Deferred; mandatory next task','No publication authorized. Final production geometry must pass complete applicable certification and transactional validation after generation.']]
    save(OUT/'REQUIREMENT_AUDIT.json',audit)
    source_names=owned+['tools/task4_prepare_next.py','tools/production_forecast_admission.py','tools/run_production_l2_risk_screen.py',
        'src/sdf_cfd_mesher_v4/advancing_front/production_risk.py','src/sdf_cfd_mesher_v4/advancing_front/rolling_metrics.py',
        'src/sdf_cfd_mesher_v4/advancing_front/private_rolling.py','src/sdf_cfd_mesher_v4/advancing_front/mixed_front_runner.py',
        'src/sdf_cfd_mesher_v4/forecast/detailed/committed_domain.py','src/sdf_cfd_mesher_v4/forecast/engine.py']
    sources={n:file_hash(V4/n) for n in source_names};save(OUT/'SOURCE_EVIDENCE.json',sources)
    implementation=identity(sources);admission_id=identity(dict(scientific=a['scientific_identity'],implementation=implementation,audit=identity(audit)))
    tests=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite').attrib
    assert tests['failures']==tests['errors']==tests['skipped']=='0'
    resources=dict(starting_free=disk['free_bytes'],ending_free=shutil.disk_usage(ROOT).free,repository_use=disk['repository_accessible_logical_bytes'],
        v4_use=disk['v4_accessible_logical_bytes'],inaccessible_entries=len(disk['errors']),disk_audit_seconds=disk['seconds'],
        elapsed_seconds_at_seal=time.time()-base['started'],peak_rss=max(a['peak_rss'],b['peak_rss']),retained_rss_A=a['retained_rss'],retained_rss_B=b['retained_rss'],
        total_A_seconds=a['total_seconds'],total_B_seconds=b['total_seconds'],bytes_deleted=0,cleanup='None; no bulk copies, no deletions.')
    save(OUT/'RESOURCES.json',resources)
    hashes=dict(binding=identity(s['evidence']['binding']),evidence=s['evidence_identity'],completeness=s['decision']['completeness_identity'],
        zero_risk=s['decision']['zero_risk_identity'],detailed_noop=s['decision']['detailed_noop_identity'],decision=s['decision_identity'],
        complete_record=a['scientific_identity'],implementation=implementation,admission=admission_id)
    lines=['## Decision question','','Does the complete sealed zero-risk evidence authorize an attempt to generate L2 from committed L1?','',
      '**V4ProductionLayer2ReadOnlyAdmissionQualified**','',
      '**ProductionLayer2AdvanceAuthorized**. This authorizes a generation attempt only. It does not qualify L2, publish provisional geometry, certify final continuous geometry, or establish complete AdvancingFrontQualified. Final generation, complete certification and transactional commit remain required.','',
      f'Starting HEAD: `{base["head"]}`. Branch: `{base["branch"]}`.','',
      '### Admission semantics audit','','| Requirement | Type | Status | Evidence / rule |','|---|---|---|---|']
    lines+=['| '+' | '.join(row)+' |' for row in audit]
    lines+=['','The new bounded adapter calls the existing event-driven selector after verifying complete evidence. It does not invoke historical fixture providers. The old incomplete-evidence blocker remains unchanged. Any nonempty risk population or an independently mandatory detailed requirement fails closed; this implementation qualifies only the complete zero-risk branch.','',
      '### Bound evidence identities','']
    lines += [f'- {k}: `{v}`' for k,v in s['evidence']['binding'].items()]
    lines += [f'- {k}: `{v}`' for k,v in s['evidence']['authorities'].items()]
    lines += ['', 'Wall coordinate authority: `2270c352c34bba85dc898da4ea66dda1484123604d96511551899802d16d157c`. Wall topology: `d14f00450f2f67f7b98ab3efa1a7946393b7b793ccd64d4bdcdbf516342de2db`. Production surface loader verifies unchanged coordinates, topology and sizing field.',
      'Previous report SHA-256: `509c0f1b714bc38951f450227c5e415eda4940780e76d58e560445b72acea598`. Previous authority SHA-256: `48ff989f17ee7cc26b20e03990ccf160a2f8d0bccacda0e09f12f5c78fc00dab`. All consumed file and array seals passed.','',
      '### Zero-risk result','','Cheap hard events: 0. Direct hard-event rows: 0. Collision candidates/certified: 2,898,957/2,898,957. Forbidden, unresolved, proof failures, missing and duplicate evidence: 0. Seeds: 0. Patches: 0. Detailed work count: 0. Detailed result: ConditionalDetailedNoOp.',
      'No independent mandatory detailed metric was found in the existing nominal-step semantics. No patch was fabricated. Dense future direct metrics remain NotMeasured; this is not a future-layer pass claim. Sigma/G remain diagnostic. Hard gates remain positive volume/Jacobian, determinant >=0.0011, weight >=0.055, skewness <=3.63636, symmetric volume ratio >=0.01, aspect <=1000, valid topology/ownership and zero forbidden/unresolved continuous geometry.','',
      '### Replay and decision seals','']
    lines += [f'- {k}: `{v}`' for k,v in hashes.items()]
    lines += ['', 'Both independent evaluations restored L1 from its native authority and verified the sealed evidence. All scientific fields match exactly; runtime/RSS are excluded from scientific identity.','',
      '### No geometry','','L2 generated = NO; L2 committed = NO; L3 generated = NO; core generated = NO. Authoritative committed layer = 1. Wall/L1/request unchanged. No new provisional cells, future trajectories, patches or publisher calls were executed. Existing screening coordinates were read only.','',
      '### Performance and tests','','| Phase (seconds) | A | B |','|---|---:|---:|']
    lines += [f'| {k} | {a["timings"][k]} | {b["timings"][k]} |' for k in a['timings']]
    lines += ['', '```json',json.dumps(resources,indent=2),'```','',
      f'Tests: {tests["tests"]} run, all passed, 0 failures, 0 errors, 0 skipped; {tests["time"]} seconds. Modules: test_production_readonly_admission.py and test_production_collision_localization.py. Tests cover completeness, identity/missing evidence rejection, conditional no-op, mandatory detail preservation, direct/collision event rejection, replay, and no publication. Existing tests were not weakened.',
      'Storage numbers cover accessible logical bytes and retain the disk audit errors. No L1, forecast, direct-screen or proof bulk was duplicated. Final created bytes, ending free space, and Git HEAD are printed after commit.','',
      '### Git, blockers and next task','',f'Unrelated dirty status entries preserved: {len(base["dirty"])}/{len(unrelated)}. V3 task modifications: 0. Only task-owned V4 files are committed.',
      'Read-only admission blockers: none. L2 geometry remains ungenerated and unqualified.',
      '**GenerateAndQualifyV4CapsuleLayer2**: generate exactly L2 from committed L1, perform complete final certification and transactional commit only if valid, then stop before L3.','']
    (OUT/'report.md').write_bytes('\n'.join(lines).encode())
    evidence={p.name:file_hash(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.name not in ('AUTHORITY.json','SEALS.json')}
    authority=dict(classification='V4ProductionLayer2ReadOnlyAdmissionQualified',admission_identity=admission_id,decision=s['decision'],hashes=hashes,
        binding=s['evidence']['binding'],input_authorities=s['evidence']['authorities'],completeness=s['evidence']['completeness'],
        scientific_replay_equal=True,publication=s['publication'],preservation=s['preservation'],evidence=evidence,tests=tests,resources=resources,
        git=dict(starting_head=base['head'],branch=base['branch'],unrelated_before=len(base['dirty']),unrelated_after=len(unrelated),V3_task_modifications=0),
        next_task='GenerateAndQualifyV4CapsuleLayer2')
    save(OUT/'AUTHORITY.json',authority)
    seals={n:file_hash(OUT/n) for n in ('report.md','AUTHORITY.json')};save(OUT/'SEALS.json',seals)
    print(json.dumps(dict(seals=seals,hashes=hashes,resources=resources),indent=2))


if __name__=='__main__':seal()
