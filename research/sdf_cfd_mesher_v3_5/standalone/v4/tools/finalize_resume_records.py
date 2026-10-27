"""Summarize finished workers and preserve the pre-existing dirty state audit."""
import json,subprocess
from pathlib import Path
from collect_resume_fast_ledger import ROOT,collect


def read(path):return json.loads((ROOT/path).read_text(encoding='utf-8-sig'))
def save(name,value):(ROOT/name).write_text(json.dumps(value,indent=2))


def main():
    collect()
    paths=['capsule/SESSION.json','RECOVERY_PROCESS.json','capsule/SEAL_SESSION.json',
        'capsule/RECOVERY_GEOMETRY_CHECK.json','capsule_completion/SESSION.json',
        'capsule_completion/CORE_INTERRUPTED.json','capsule_complete_retry/CORE_SESSION.json',
        'bent/SESSION.json','bent_recovery/SESSION.json','bent_continuation/INTERRUPTED.json',
        'bent_seal18/SEAL_SESSION.json','interacting_recovery/SESSION.json','interacting_continuation/SESSION.json',
        'interacting_continuation/FINAL_RESTART.json']
    intervals=[dict(path=p,seconds=read(p)['seconds']) for p in paths if (ROOT/p).exists()]
    total=sum(r['seconds'] for r in intervals)
    save('EXECUTION_ACCOUNTING.json',dict(seconds=total,minutes=total/60,target_seconds=7200,hard_seconds=10800,
        target_exceeded=total>7200,hard_cap_exceeded=total>10800,
        definition='Non-overlapping scientific-worker wall time including bootstrap, I/O, failures, recovery, core and final sealing; excludes coding, tests, idle waits and report hashing',intervals=intervals))
    end=read('interacting_continuation/END.json') if (ROOT/'interacting_continuation/END.json').exists() else None
    cases={
      'capsule_production_control':dict(start_layer=12,final_layer=20,ordinary_layers=8,end_reason='BoundedCompleteQualified',shell_cells=163840,complete_cells=195206,shell_relations=15756956,complete_relations=19339761,final_seal=True),
      'bent_passage_production':dict(start_layer=6,final_layer=18,ordinary_layers=12,end_reason='BudgetStoppedStagnant',shell_cells=147456,shell_relations=12734448,final_seal=read('bent_seal18/SEAL.json')['passed'],attempted_layer19_committed=False,complete_mesh=False),
      'interacting_features_production':dict(start_layer=3,final_layer=end['final_layer'] if end else 3,end_reason=end['reason'] if end else 'ImplementationBlocked',
        authority_recovered=read('interacting_recovery/RECOVERY.json')['passed'],new_layers=end['layers_committed'] if end else 0,
        result=read('interacting_continuation/SHELL_SUMMARY.json') if (ROOT/'interacting_continuation/SHELL_SUMMARY.json').exists() else None,
        seal=read('interacting_continuation/SEAL.json') if (ROOT/'interacting_continuation/SEAL.json').exists() else None,
        restart=read('interacting_continuation/FINAL_RESTART.json')['passed'] if (ROOT/'interacting_continuation/FINAL_RESTART.json').exists() else None),
      'separated_regions_production':dict(start_layer=2,final_layer=2,ordinary_layers=0,executed=False,end_reason='BudgetStoppedAfterProgress',
        end_reason_scope='Task-level budget; this case was not advanced',
        authority_status='Saved geometry present; raw receipt files missing; no reconstruction attempted in the remaining target budget',
        new_quality='Not measured',new_runtime='Not measured',new_seal='Not measured')}
    save('CASE_RESULTS.json',cases)
    before=read('TASK_MANIFEST.json')['status']
    after=subprocess.check_output(['git','status','--porcelain','-z']).decode('utf-8')
    def outside(value):return sorted(s for s in value.split('\0') if s and not s[3:].replace('\\','/').startswith('research/sdf_cfd_mesher_v4/'))
    prior,current=outside(before),outside(after)
    v3=lambda rows:[s for s in rows if s[3:].replace('\\','/').startswith('research/sdf_cfd_mesher_v3')]
    save('GIT_AUDIT.json',dict(initial_head=read('TASK_MANIFEST.json')['head'],precommit_head=subprocess.check_output(['git','rev-parse','HEAD']).decode().strip(),
        unrelated_status_unchanged=prior==current,unrelated_before_count=len(prior),unrelated_after_count=len(current),
        added_status_entries=sorted(set(current)-set(prior)),removed_status_entries=sorted(set(prior)-set(current)),
        v3_status_before=v3(prior),v3_status_after=v3(current),v3_task_modifications=0))
    print(json.dumps(dict(scientific_seconds=total,hard_cap_exceeded=total>10800,unrelated_status_unchanged=prior==current)))


if __name__=='__main__':main()
