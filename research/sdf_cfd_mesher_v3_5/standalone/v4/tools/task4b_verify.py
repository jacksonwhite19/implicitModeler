"""Read-only preservation and independent scientific replay checks."""
from task4b_common import *
from task4_preflight import audit,git

if __name__=='__main__':
    first=read(OUT/'optimized_1/COMPLETENESS.json');second=read(OUT/'optimized_2/COMPLETENESS.json')
    assert first==second,'ScientificReplayMismatch'
    comparison=json.loads((OUT/'FULL_BASELINE_EQUIVALENCE.json').read_text())
    assert len(comparison)==20 and sum(r['pairs'] for r in comparison)==56462864
    assert all(r['all_pairs_identical'] and r['all_outcomes_identical'] for r in comparison)
    save('DETERMINISM.json',dict(passed=True,first=first['scientific_hash'],second=second['scientific_hash'],
        layers=20,pairs=first['candidate_pairs'],fields='All pair IDs, proof codes, witnesses, outcomes, full exact records, geometry/topology identities, transaction records and completeness manifest.',
        different_execution='64-pair jobs without root-key memoization versus 8-pair jobs with exact key memoization; independent empty proof ledgers.'))
    start=json.loads((OUT/'PREFLIGHT.json').read_text());prior=audit();auth=json.loads((BASE/'AUTHORITY.json').read_text());count=0
    for group in ('source_sha256','evidence_sha256'):
        for path,h in auth[group].items():assert file_hash(ROOT/path)==h,path;count+=1
    assert file_hash(BASE/'report.md')=='28fcb9be7fac30eec2dd84a3d292fbfbdead3daef3a4ed851a4df7744a213190'
    assert file_hash(BASE/'AUTHORITY.json')=='be5098ec2885cf1a2d0f1480b356a3d25bca3abd8cf0d984952c781cab393ce9'
    assert file_hash(BASE/'build_A/CURRENT.json')==start['A_checkpoint']
    assert file_hash(BASE/'replay_A2/build_A/CURRENT.json')==start['A2_checkpoint']
    def root(p):
        while 'previous' in p:p=p['previous']
        return p['prior_frozen_audit']
    old=root(start['prior']);now=root(prior);oldv={r['path']:r['actual'] for r in old['rows'] if 'sdf_cfd_mesher_v3/' in r['path'].replace('\\','/')}
    newv={r['path']:r['actual'] for r in now['rows'] if 'sdf_cfd_mesher_v3/' in r['path'].replace('\\','/')}
    assert oldv==newv,'BoundedV3PreservationMismatch'
    def outside(s):return [line for line in s.splitlines() if 'research/sdf_cfd_mesher_v4' not in line]
    current=git('status','--porcelain');before=outside(start['status']);after=outside(current)
    save('PRESERVATION.json',dict(task4_files_reverified=count,V3_files_in_bounded_comparison=len(oldv),V3_modifications=0,
        Build_A_unchanged=True,A2_unchanged=True,sealed_results_unchanged=True,
        starting_head=start['head'],head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),
        unrelated_start=before,unrelated_end=after,unrelated_added=sorted(set(after)-set(before)),unrelated_removed=sorted(set(before)-set(after)),
        known_preexisting_source_drift=now['outside_scope_origin_drift'],finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    print('REPLAY AND PRESERVATION PASSED',count,'Task4 files;',len(oldv),'bounded V3 files')
