"""Configure authorized successors through the existing optimized executor."""
import argparse
from copy import deepcopy
from contextlib import ExitStack
import json
from pathlib import Path
import subprocess
import sys

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
OUT=V4/'results/wing_continuation_layers6to8'
ROUND_TRIP=V4/'results/current_front_transition_round_trip'
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity


def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))


def validate_control(control):
    import immutable_state_access as access
    with ExitStack() as held:
        for row in control['inputs']:
            p=ROOT/row['path'];held.enter_context(access.guard(p))
            if file_hash(p)!=row['sha256']:raise ValueError('ContinuationInputIdentityMismatch',row['path'])
        if control['schema']!='V4AuthorizedContinuationExecution/1' or control['layers']!=[6,7,8]:
            raise ValueError('UnsupportedContinuationExecutionScope')
        auth=control['authorization_record']
        if identity(auth)!=control['authorization'] or auth!=read(OUT/'AUTHORIZATION.json'):
            raise ValueError('ContinuationAuthorizationChanged')
        if control['parent']!=dict(read(ROUND_TRIP/'A/L5/REOPEN.json'),layer=5) or auth['approved_parent']!=control['parent']:
            raise ValueError('WrongStartingQualifiedParent')
        if auth['authorized_layers']!=control['layers']:raise ValueError('ContinuationScopeMismatch')
    return control


def baseline():
    from record_receipt_resume import disk
    from check_current_front_encoding import allocation
    if OUT.exists():raise ValueError('PreservePriorContinuationOutput')
    old=read(V4/'results/wing_continuation_layers2to5/AUTHORIZATION.json')
    if identity(old)!='bd2e3209707fd7f11d3082a069f2490a6903421e73ca16727f375683b4d96772':
        raise ValueError('OriginalAuthorizationChanged')
    diagnostic=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
    parent=dict(read(ROUND_TRIP/'A/L5/REOPEN.json'),layer=5)
    if parent['scientific']!='905511e4d6a2edcf0ca00dadd1f4ee8522bdb39634edfc1e83928eddf7074019' or not parent['passed']:
        raise ValueError('WrongQualifiedL5')
    auth=deepcopy(old)
    auth.update(schema='V4ContinuationAuthorizationSuccessor/1',authorized_layers=[6,7,8],
        authorization='User: Run L6. If no failures or issues, keep going through L8. Monitor storage.',
        predecessor=old,predecessor_identity=identity(old),approved_parent=parent,
        restrictions=['No layer 9','Stop at first required failure or unresolved issue',
                      'No tuning, scientific retry or gate/resource relaxation',
                      'Authenticate each actual parent; this authorization is not admission evidence'],
        production_admission_passed=False,
        increments=[dict(layer=k,quantity='path_length_m',binary64_hex=diagnostic['nominal_increment_hex'][k-1],
            value_m=float.fromhex(diagnostic['nominal_increment_hex'][k-1])) for k in (6,7,8)])
    prior=read(ROUND_TRIP/'BASELINE.json');protected=[];intentional=[]
    for row in prior['protected']:
        p=ROOT/row['path'];now=file_hash(p)
        if p.name=='accepted_production_contract.py':intentional.append(dict(**row,after=now))
        elif now!=row['sha256']:raise ValueError('ProtectedFileChanged',row['path'])
        else:protected.append(row)
    OUT.mkdir()
    durable_json(OUT/'AUTHORIZATION.json',auth)
    names=['results/wing_continuation_layers2to5/AUTHORIZATION.json',
           'results/current_front_transition_round_trip/A/L5/REOPEN.json',
           'results/current_front_transition_round_trip/REVIEW.json',
           'results/wing_continuation_layers6to8/AUTHORIZATION.json',
           'tools/accepted_production_contract.py','tools/run_authorized_continuation.py',
           'tools/current_front_dependencies.py','tools/finish_retained_l5.py','tools/run_wing_layers6to8.py']
    control=dict(schema='V4AuthorizedContinuationExecution/1',layers=[6,7,8],parent=parent,
        authorization_record=auth,authorization=identity(auth),
        inputs=[dict(path=(V4/n).relative_to(ROOT).as_posix(),sha256=file_hash(V4/n)) for n in names])
    durable_json(OUT/'CONTROL.json',control);validate_control(control)
    roots=[V4/'results/wing_receipt_resume/A',ROUND_TRIP,OUT]
    measurement=allocation([p for root in roots for p in root.rglob('*') if p.is_file()])
    durable_json(OUT/'BASELINE.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        disk=disk(),storage_roots=[str(p) for p in roots],storage=measurement,
        protected=protected,intentional_source_change=intentional,
        old_authorization_sha256=file_hash(V4/'results/wing_continuation_layers2to5/AUTHORIZATION.json'),
        scope='Authorize L6-L8 separately. No old state/request rewrite. Unchanged operational cap.'))
    print(json.dumps(dict(authorization=identity(auth),increments=auth['increments'],
        storage={k:v for k,v in measurement.items() if k!='records'},disk=disk()),indent=2))


def preflight():
    from sealed_front_connection import qualified_dispatch
    from composable_state import CommittedState
    c=validate_control(read(OUT/'CONTROL.json'))
    with qualified_dispatch(),CommittedState(c['parent']['reference']) as base:
        base.check()
        if base.layer!=5 or base.scientific!=c['parent']['scientific']:raise ValueError('WrongGuardedL5')
        durable_json(OUT/'PARENT_PREFLIGHT.json',dict(passed=True,layer=base.layer,scientific=base.scientific,root=base.root))


def worker():
    import run_authorized_continuation as runner
    c=validate_control(read(OUT/'CONTROL.json'));runner.OUT=OUT/'A';runner.OUT.mkdir(exist_ok=False)
    runner.execute(continuation=c)


def monitor():
    import finish_retained_l5 as monitored
    validate_control(read(OUT/'CONTROL.json'))
    monitored.OUT=OUT
    return monitored.monitor(worker_command=[sys.executable,'-B',__file__,'worker'],extra_storage_roots=[ROUND_TRIP])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','preflight','worker','monitor']);a=p.parse_args()
    if a.mode=='monitor':sys.exit(monitor())
    globals()[a.mode]()
