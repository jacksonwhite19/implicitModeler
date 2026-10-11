"""Audit the actual Task-4 boundary and preserved SF2 evidence without meshing."""
import json, subprocess, time
from pathlib import Path
import termination_policy_cohort as c
from sdf_cfd_mesher_v4.advancing_front.termination_policy import evaluate
from sdf_cfd_mesher_v4.common.authority import file_hash
ROOT=c.ROOT; OUT=ROOT/'results/production_automatic_termination_remesh'


def run():
    expected={
        'production_termination_trigger_policy':('fce53f81d4fd8c10706325e0cf043c88fea14f9ed1a2a4313db95c02acacd203','e060bcfe06009beab135f587257c056130c0c206d43bbfe6f586fd6da35a62bc'),
        'local_termination_remesh_generalization':('cc79ca134b7c648303e68715d27442acb41428ec203c96a1f036a4e80ea5f734','c8d249f635b7e0fdee4dd1b8b084a97522f1720a5face5081b707d8c6c0400ba')}
    verified={}
    for case,hashes in expected.items():
        for name,h in zip(('report.md','AUTHORITY.json'),hashes):
            p=ROOT/'results'/case/name
            if file_hash(p)!=h:raise ValueError('PriorAuthorityChanged:'+str(p))
            verified[str(p.relative_to(ROOT))]=h
    start=time.perf_counter(); rows=[]
    for case in c.negative_cases():
        decision=evaluate(case['evidence'])
        rows.append(dict(name=case['name'],state=decision['state'],disposition=decision['disposition'],
                         remesh=decision['next_action']=='RollbackRemesh',trace=decision['trace']))
    sf2=[r for r in rows if r['name'].startswith('SF2')]
    assert len(rows)==63 and not any(r['remesh'] for r in rows)
    c.g.save(OUT/'SF2_REGRESSION.json',dict(negative_snapshots=len(rows),sf2_snapshots=len(sf2),false_triggers=0,
        geometry_changes=0,layer_count_changes=0,correction_changes=0,full_shell_run=False,records=rows))
    c.g.save(OUT/'SOURCE_BINDINGS.json',c.BINDINGS)
    c.g.save(OUT/'AUTHORITY_VERIFICATION.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),verified=verified,
        policy_replay_seconds=time.perf_counter()-start))
    # These are concrete integration gaps in current source, not mathematical
    # impossibility claims or exceptions to the user's requirements.
    boundary=dict(actual_entry_points=['tools/task4l_run.py','tools/task4_prepare_next.py','tools/task4_certify_candidate.py'],
        implemented=['Atomic private mixed-state store','Evidence-based dispatch inside mixed transaction',
                     'Cell/face scientific dependency receipt invalidation','Explicit rectangular-runner mixed-checkpoint rejection'],
        incomplete={
            'actual_transaction_loop':'Task-4 prepare/certify do not dispatch the mixed store',
            'risk_atlas':'RiskAtlas nominal/load still bind a rectangular trajectory and fixed topology',
            'forecast':'Terminated-column masks and asynchronous mixed front advancement not integrated',
            'planner':'PersistentPlanner supports fixed-topology patches, not remeshed incidence',
            'certificate_manifest':'Mixed proof receipts are not bound into Task-4 production manifest',
            'quality_receipts':'Mixed receipts do not migrate existing Task-4 validation receipt format',
            'checkpoint':'Mixed store restart is tested; Task-4 restart into mixed continuation is explicitly rejected',
            'general_sdf':'Historical remesh authority preserved; integrated replay not executed'},
        enabled_in_production=False,classification='V4ProductionAutomaticTerminationRemeshInsufficient')
    c.g.save(OUT/'RUNNER_BOUNDARY.json',boundary)
    print('63 negative snapshots; production integration incomplete',flush=True)


if __name__=='__main__':run()
