"""Separate observed policy actions from independently labelled truth."""
import json
from pathlib import Path
from run_geometry_family import OUT,g
from summarize_geometry_family import committed_history


def run():
    summary=json.loads((OUT/'CASE_SUMMARY.json').read_text());cases=[]
    for row in summary['cases']:
        folder=OUT/row['name']/str(row['run']);p=folder/'OPERATIONS.json'
        operations=committed_history(folder,'OPERATIONS.json') if folder.exists() else []
        events=[]
        for op in operations:
            if op['operation']!='RollbackRemesh':continue
            policy=op['policy'];events.append(dict(event=op['event']['event'] if isinstance(op['event'],dict) else op['event'],state=op['state'],seed=policy['seed'],
                policy=policy['state'],next_action=policy['next_action'],evidence=policy['evidence'],trace=policy['trace']))
        cases.append(dict(case=row['name'],committed_events=events,requested_schedule_complete=row['shell_schedule_complete'],
            status=row['classification'],reason=row['reason'],
            discovery_attempt_recorded=any((OUT/row['name']).glob('*/DISCOVERY_*.json'))))
    result=dict(preserved_labelled_authority=dict(no_remesh_controls=2,remesh_positive_cases=4,false_triggers=0,missed_triggers=0,
        source='User-specified prior coupled-shell authority; current bounded controls and policy regression tests retain that evidence'),
        new_family=dict(cases_with_committed_remesh=sum(bool(r['committed_events']) for r in cases),
            committed_events=sum(len(r['committed_events']) for r in cases),
            cases_with_discovery_attempts=sum(r['discovery_attempt_recorded'] for r in cases),
            completed_shells_without_remesh=sum(r['requested_schedule_complete'] and not r['committed_events'] for r in cases),
            false_triggers=None,missed_triggers=None,false_coupled_revisions=None,missed_coupled_failures=None,
            reason='No independent binary truth label exists for every failed wall/cavity/core design. Do not derive truth from the policy output itself.'),
        checks='All committed mixed candidates require complete-neighbor validation. A first-layer rejection is not a missed remesh trigger; an unsuccessful triggered repair is not a missed trigger.',cases=cases)
    g.save(OUT/'TRIGGER_AUDIT.json',result)


if __name__=='__main__':run()
