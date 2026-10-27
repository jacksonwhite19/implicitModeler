"""Read-only frozen evidence audit and exact replay comparisons."""
from task4c_common import *
from task4_preflight import audit
from sdf_cfd_mesher_v4.certificate.replay_equivalence import compare_checkpoint
ALLOWED={'tools/task4_prepare_next.py','tools/task4_certify_candidate.py','tools/task4_final_validation.py','tools/task4_warning_cache.py','tools/task4_compare_replay.py'}

def run():
    started=time.perf_counter();a=read(OUT/'atlas_1_ATLAS.json');b=read(OUT/'atlas_2_ATLAS.json')
    assert a==b and len(a['rows'])==20
    assert all(not r['missing'] and r['all_forecast_coordinates_exact'] for r in a['rows'])
    cert=read(OUT/'CERTIFICATE_REPLAY.json');assert len(cert['rows'])==20
    assert all(r['all_exact_outcomes_equal'] and r['all_optimized_proofs_equal'] for r in cert['rows'])
    assert sum(r['pairs'] for r in cert['rows'])==56462864
    from task4c_archive import verify_reference
    for k in range(1,21):verify_reference(OUT/f'certificate_layer_{k:02}')
    format_rows=[]
    for layer in range(1,21):
        checkpoint=json.loads((BASE/f'build_A/LAYER_{layer:02}.json').read_text())
        # Same preserved transaction in the new certified representation; this
        # checks the bridge without executing or writing an A2 transaction.
        view=dict(checkpoint,certificate=read(OUT/f'certificate_layer_{layer:02}/MANIFEST.json'))
        ref=read(B/f'optimized_2/LAYER_{layer:02}.json');format_rows.append(compare_checkpoint(checkpoint,view,ref))
    cases=[]
    for k in (2,8,12,20):
        r=read(OUT/f'RUNNER_{k:02}.json');assert all(r[v] for v in ('all_decisions_exact','all_coordinates_exact','all_gate_values_exact'))
        cases.append(dict(layer=k,candidate_hash=r['candidate_hash'],decision_hash=r['decision_hash']))
    save('DETERMINISM.json',dict(passed=True,atlas_run_1=a['scientific_hash'],atlas_run_2=b['scientific_hash'],
        certificate_full_scientific_replay=cert['scientific_hash'],certificate_replay_reference='Frozen Task-4B optimized_2; every proof code, witness, outcome and fallback hash matched.',
        required_pairs=56462864,runner_cases=cases,format_bridge=scientific_hash(format_rows),
        excluded='Timing, RSS and process metadata only. Full atlas scientific rows compare exactly.',A2_executed=False))
    start=read(OUT/'PREFLIGHT.json');prior=audit();authorized=[];count=0
    for base in (BASE,B):
        auth=read(base/'AUTHORITY.json')
        for group in ('source_sha256','evidence_sha256'):
            for p,h in auth[group].items():
                now=file_hash(ROOT/p)
                if now!=h:
                    assert base==BASE and group=='source_sha256' and p in ALLOWED,(base,group,p)
                    authorized.append(dict(path=p,before=h,after=now))
                count+=1
    assert file_hash(BASE/'report.md')=='28fcb9be7fac30eec2dd84a3d292fbfbdead3daef3a4ed851a4df7744a213190'
    assert file_hash(BASE/'AUTHORITY.json')=='be5098ec2885cf1a2d0f1480b356a3d25bca3abd8cf0d984952c781cab393ce9'
    assert file_hash(B/'report.md')=='ec5686586f9eb5e2e65f55e72338dae3276b82e8f9d70a4e70efc5f610c7bba2'
    assert file_hash(B/'AUTHORITY.json')=='8fe4af86b569705ac41399dcdbd0a02ddb7ff41b62a75fc254cd7998bd754aac'
    assert file_hash(BASE/'build_A/CURRENT.json')==start['Build_A_checkpoint']
    assert file_hash(BASE/'replay_A2/build_A/CURRENT.json')==start['A2_checkpoint']
    def root(p):
        while 'previous' in p:p=p['previous']
        return p['prior_frozen_audit']
    def v3(p):return {r['path']:r['actual'] for r in root(p)['rows'] if 'sdf_cfd_mesher_v3/' in r['path'].replace('\\','/')}
    assert v3(start['prior'])==v3(prior)
    def outside(s):return [x for x in s.splitlines() if 'research/sdf_cfd_mesher_v4' not in x]
    before=outside(start['status']);after=outside(git('status','--porcelain'))
    save('PRESERVATION.json',dict(sealed_bindings_rechecked=count,authorized_source_changes=authorized,
        sealed_result_changes=0,V3_modified_files=0,V3_bounded_files=len(v3(prior)),Build_A_unchanged=True,A2_unchanged=True,
        unrelated_state_unchanged=before==after,unrelated_added=sorted(set(after)-set(before)),unrelated_removed=sorted(set(before)-set(after)),
        unrelated_changes_by_this_task=0,unrelated_status=after,seconds=time.perf_counter()-started,
        finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    print('DETERMINISM, FORMAT BRIDGE AND PRESERVATION PASS',count,flush=True)

if __name__=='__main__':run()
