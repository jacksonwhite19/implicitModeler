"""Custody and hard process-tree deadline for one retained private replay."""
from pathlib import Path
from contextlib import ExitStack
import sys,json,hashlib,subprocess,time
HERE=Path(__file__).resolve().parent
V4=next(p for p in HERE.parents if p.name=='sdf_cfd_mesher_v4');ROOT=V4.parents[1]
sys.dont_write_bytecode=True
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from path_preserving_deduplication import durable_json
from immutable_state_access import guard,token

def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def write(path,body):
    if path.exists():raise ValueError('ExistingEvidence',str(path))
    durable_json(path,body)
def bounded(command,seconds,out,err):
    """Wall time covers loading, construction, validation and output."""
    import psutil
    start=time.perf_counter();process=subprocess.Popen(command,cwd=ROOT,stdout=out,stderr=err)
    killed=False;kill_output=None
    try:process.wait(timeout=max(0,seconds-(time.perf_counter()-start)))
    except subprocess.TimeoutExpired:
        killed=True
        family=[psutil.Process(process.pid),*psutil.Process(process.pid).children(recursive=True)]
        kill=subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,text=True,timeout=15)
        kill_output=dict(returncode=kill.returncode,stdout=kill.stdout,stderr=kill.stderr)
        # Fail closed if taskkill cannot retire this explicitly owned tree.
        _,alive=psutil.wait_procs(family,timeout=5)
        for child in alive:child.kill()
        _,alive=psutil.wait_procs(alive,timeout=5)
        process.wait(timeout=5)
        if alive:raise RuntimeError('OwnedProcessTreeNotRetired',[p.pid for p in alive])
    return dict(pid=process.pid,returncode=process.returncode,wall_seconds=time.perf_counter()-start,
                hard_limit_seconds=seconds,killed_at_limit=killed,tree_kill=kill_output)

def prepare():
    from owned_runtime import verify_release
    release=verify_release();prior=V4/'results/cli_repair_integration_20261013'
    run=prior/'plumbing_v2/fresh_y_cli'
    relative=['MESH.npz','CURRENT.json','background/INPUT.npz','background/INPUT.json',
              'background/generated/NETGEN.npz','background/generated/OUTER.npz',
              'background/generated/RESULT.json','background/generated/OPTIMIZATION.json']
    pins={V4/r['path']:r['sha256'] for r in release['sources']+release['independent_reviews']}
    pins[V4/'runtime/RELEASE.json']=sha(V4/'runtime/RELEASE.json')
    study=HERE.parent
    for path in [study/'BASELINE.json',study/'SELECTION_PLAN.json',study/'SOURCE_REVIEW.json',study/'SELECTION.json',study/'SELECTION_CLOSURE.json',study/'INSTALLED_CONTROLS.json',study/'INSTALLED_CONTROLS_CLOSURE.json']:
        pins[path]=sha(path)
    for path in [*(run/n for n in relative),prior/'CASE.json',prior/'RESULT_SUMMARY.json',*HERE.glob('*.py'),*HERE.glob('CONTROLS*'),study/'DEFAULT_EQUIVALENCE.json',study/'DEFAULT_EQUIVALENCE_CLOSURE.json']:pins[path]=sha(path)
    controls=read(prior/'CASE.json')['background']['interface_completion'].copy()
    assert controls['seconds_per_sweep']==105
    controls['seconds_per_sweep']=1800
    from background_completion_controls import validate_core
    validate_core(controls)
    status=subprocess.check_output(['git','--no-optional-locks','status','--porcelain=v1'],cwd=ROOT,text=True).splitlines()
    write(HERE/'PLAN.json',dict(authorization='Human Do it approves geometry-derived shared-apex starts, expanded protected cavities, focused tests/review and one private retry; hard30min bound persists.',
        scope='Exactly one private retained-array replay of reviewed installed geometry-start repair and expanded protected cavity search. Exact coordinate replay precedes frozen-point later cavity changes; no witness is replayed on changed connectivity. Search budget1800s and hard entire-worker wall bound1800s. All remaining controls/gates/physical inputs unchanged. No surface/prism/backend generation, live handoff reuse or production acceptance.',
        hard_limit_seconds=1800,controls=controls,source_inventory_identity=release['source_inventory_identity'],
        release_sha256=sha(V4/'runtime/RELEASE.json'),input_pins={str(p):h for p,h in pins.items()},
        run=str(run),prior_case=str(prior/'CASE.json'),original_status=status,
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        index_sha256=sha(ROOT/'.git/index'),expected_initial_records=1798,
        acceptance='Existing full current core/terminal-prism quality, ownership, farfield quality and exact relative edit validation. Diagnostic only; original backend-trusted embedding premise remains. No global collision/coverage/filling or source acceptance added or claimed.',
        command=[sys.executable,'-B',str(HERE/'worker.py')]))

def launch(plan_sha,review_sha):
    planpath=HERE/'PLAN.json';reviewpath=HERE/'LAUNCH_REVIEW.json'
    with ExitStack() as held:
        for path,h in [(planpath,plan_sha),(reviewpath,review_sha)]:held.enter_context(guard(path));assert sha(path)==h
        plan=read(planpath);review=read(reviewpath)
        assert review['passed'] and not review['blockers'] and review['bounded_run_approved']
        assert review['plan_sha256']==plan_sha and review['input_pins']==plan['input_pins'] and review['all_guards_released']
        pins={Path(p):h for p,h in plan['input_pins'].items()}
        for path,h in pins.items():held.enter_context(guard(path));assert sha(path)==h
        before={p:token(p) for p in pins};assert sha(ROOT/'.git/index')==plan['index_sha256']
        write(HERE/'ATTEMPT.json',dict(attempt=1,plan_sha256=plan_sha,review_sha256=review_sha,hard_limit_seconds=1800))
        with (HERE/'stdout.log').open('x',encoding='utf-8') as out,(HERE/'stderr.log').open('x',encoding='utf-8') as err:
            result=bounded(plan['command'],plan['hard_limit_seconds'],out,err)
        assert all(sha(p)==h and token(p)==before[p] for p,h in pins.items())
        assert sha(ROOT/'.git/index')==plan['index_sha256']
        result.update(input_before_after_equal=True,output_hashes={p.name:sha(p) for p in HERE.glob('*.json') if p.name not in ('EXECUTION.json','EXECUTION_CLOSURE.json')},all_guards_released=False)
        write(HERE/'EXECUTION.json',result);receipt=sha(HERE/'EXECUTION.json')
    write(HERE/'EXECUTION_CLOSURE.json',dict(receipt_sha256=receipt,all_guards_released=True))
    print(json.dumps(result),flush=True)

if __name__=='__main__':
    if sys.argv[1]=='prepare':prepare()
    elif sys.argv[1]=='launch':launch(*sys.argv[2:])
    else:raise ValueError('UnknownCommand')
