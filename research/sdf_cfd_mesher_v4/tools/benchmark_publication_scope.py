"""Retained-input controls for existing guarded preparation/publication reads.

No new geometry, certificate, production front or format migration. Payloads are
read in place; only tiny envelopes and private transaction metadata are written.
"""
import argparse
from contextlib import ExitStack
import subprocess
import sys
import time
from pathlib import Path

V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
import benchmark_retained_l6 as control
from path_preserving_deduplication import durable_json,file_hash
from record_receipt_resume import disk,status
from sdf_cfd_mesher_v4.common.authority import identity

OUT=V4/'results/guarded_publication_scope'
SOURCE=V4/'results/compact_reuse_l6/optimized'
OWN=('CURRENT_TASK.md','README.md','GUARDED_PUBLICATION_SCOPE_REPORT.md',
     'tools/benchmark_publication_scope.py','tools/guarded_publication_inputs.py',
     'tools/critical_prepare.py','tools/composable_publication.py',
     'tests/test_guarded_publication_inputs.py','results/guarded_publication_scope/')

def runtime():
    old=control.read(V4/'results/compact_reuse_l6/AUDIT_REVIEW.json')['runtime_sources']
    names=set(old)|{str(V4/'tools'/n) for n in ('guarded_publication_inputs.py','benchmark_publication_scope.py',
        'current_front_dependencies.py','critical_prepare.py','composable_publication.py')}
    return {str(Path(p).resolve()):file_hash(p) for p in sorted(names)}

def unrelated(rows):
    return [r for r in rows if not any(r[3:]=='research/sdf_cfd_mesher_v4/'+p or
        p.endswith('/') and r[3:].startswith('research/sdf_cfd_mesher_v4/'+p) for p in OWN)]

def baseline():
    if OUT.exists():raise ValueError('PreserveScopeEvidence')
    old=control.read(V4/'results/compact_reuse_l6/BASELINE.json')
    protected=dict(old['preservation'])
    for name in ('single_quality_component.py','compact_front_atlas.py','immutable_state_access.py'):
        p=V4/'tools'/name;protected[str(p)]=file_hash(p)
    sources={str(V4/'tools'/n):file_hash(V4/'tools'/n) for n in
        ('critical_prepare.py','composable_publication.py')}
    inputs={str(p):file_hash(p) for p in (SOURCE/'RESULT.json',SOURCE/'QUALITY_DELTA.json',SOURCE/'NEW_ATLAS.json',
        control.SOURCE/'CANDIDATE_PLAN.json',control.SOURCE/'CANDIDATE_ACCEPTANCE.json')}
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),unrelated=unrelated(status()),
        protected=protected,implementation_before=sources,inputs=inputs,disk=disk(),
        authorization='Go: guarded snapshot reuse on retained L6; no new layer or production publication.',
        resources=dict(minimum_free_bytes=45120503439,output_limit_bytes=None,monitor_seconds=1)))
    print(dict(unrelated=len(unrelated(status())),disk=disk()))


def request_review():
    p=OUT/'REVIEW_REQUEST.json'
    if p.exists():raise ValueError('PreserveReviewRequest')
    durable_json(p,dict(baseline_sha256=file_hash(OUT/'BASELINE.json'),runtime_sources=runtime(),
        scope='Existing prepare input boundary and file-backed private transaction; no scientific preparation, new cells/proofs or production publication.',
        options=dict(verified_quality=True,read_in_place=True,modes=['cold','shared'])))
    print(file_hash(p))


def worker():
    import os
    import psutil
    from unittest.mock import patch
    import immutable_state_access as access
    import single_quality_component as quality
    import compact_front_atlas as atlas
    import critical_prepare
    import composable_publication as publication
    from critical_transaction import Store
    from recursive_epoch_recovery import discover
    from recursive_replay_retention import authenticated_closure
    started=time.perf_counter();phases={};results={};stage='runtime'
    try:
        with ExitStack() as held:
            base_record=control.read(OUT/'BASELINE.json');request=control.read(OUT/'REVIEW_REQUEST.json');review=control.read(OUT/'REVIEW.json')
            if (review.get('passed') is not True or not review.get('reviewer') or
                review.get('request_sha256')!=file_hash(OUT/'REVIEW_REQUEST.json') or
                review.get('runtime_sources')!=runtime() or request['runtime_sources']!=runtime() or
                request['baseline_sha256']!=file_hash(OUT/'BASELINE.json')):raise ValueError('UnreviewedScopeBenchmark')
            for p,h in request['runtime_sources'].items():
                held.enter_context(access.guard(p))
                if file_hash(p)!=h:raise ValueError('ChangedReviewedRuntime',p)
            for p,h in base_record['inputs'].items():
                held.enter_context(access.guard(p))
                if file_hash(p)!=h:raise ValueError('ChangedScopeInput',p)
            # Same existing authenticated retained-input validation. No deep
            # recovery or recreation of proof/state payloads is performed.
            control.OUT=OUT
            tick=time.perf_counter();plan,accepted,authentication=control.guarded_inputs(held)
            recovered=discover(V4/'results/component_epoch_recovery/access')
            base,_,_=held.enter_context(authenticated_closure(recovered['reference']))
            if base.scientific!=plan['content']['parent_scientific']:raise ValueError('WrongScopeParent')
            phases['authentication']=time.perf_counter()-tick
            logical_before=control.read(SOURCE/'RESULT.json')
            expected_atlas=dict(sha256=logical_before['atlas_semantic'],count=logical_before['atlas_rows'])
            for mode in ('cold','shared'):
                run=OUT/mode;run.mkdir(exist_ok=False);stage=mode
                durable_json(OUT/'STAGE.json',dict(stage=stage,seconds=time.perf_counter()-started));print(mode,flush=True)
                # Small envelopes only; unchanged SQLite payloads and shared
                # schedule references remain at their authenticated old paths.
                for n,source in (('QUALITY_DELTA.json',SOURCE),('NEW_ATLAS.json',SOURCE),
                    ('NEW_FORECAST_DEPENDENCIES.json',control.SOURCE),('BINDING_PATCH.json',control.SOURCE)):
                    durable_json(run/n,control.read(source/n))
                for n,h in accepted['evidence'].items():
                    durable_json(run/n,dict(retained_path=str(control.SOURCE/n),retained_sha256=h,
                        scope='Private storage control; not a new scientific certificate'))
                calls=[];readbacks=[];checks=[];full=quality._verify;encoded=quality._verify_encoded
                def full_check(*args,**kwargs):calls.append(args[1]['count']);return full(*args,**kwargs)
                def readback(*args,**kwargs):readbacks.append(args[1]['count']);return encoded(*args,**kwargs)
                def input_boundary(parent,adapter,out,dependencies,qsnap,asnap=None):
                    # The existing optimized preparation ENTRY really opens or
                    # borrows the inputs. Its mesh/proof preparation body is not
                    # rerun; this explicitly private control only binds their
                    # already-authenticated original logical components.
                    if (parent is not base or asnap is None or qsnap['semantic']!=plan['components']['quality']['delta'] or
                        asnap['semantic']!=plan['components']['atlas']['replace'] or
                        dependencies['semantic']!=plan['components']['forecast']['replace']):raise ValueError('ScopeMeaningMismatch')
                    checks.append(dict(quality=qsnap['semantic'],atlas=asnap['semantic'],forecast=dependencies['semantic']))
                    value=dict(parent=base.reference,content={'parent_scientific':base.scientific},
                        scientific=plan['scientific'],references={**dependencies['references'],**qsnap['references'],**asnap['references']},
                        scope='Private retained-L6 preparation/publication input control',components=checks[-1])
                    durable_json(out/'CANDIDATE_PLAN.json',value);return value
                tick=time.perf_counter()
                with patch.dict(os.environ,{'V4_VERIFIED_QUALITY':'1','V4_GUARDED_PUBLICATION':'1' if mode=='shared' else '0'}), \
                    patch.object(quality,'_verify',full_check),patch.object(quality,'_verify_encoded',readback), \
                    patch.object(critical_prepare,'_prepare',input_boundary),patch.object(publication,'prepare',critical_prepare.prepare), \
                    patch.object(publication,'Store',Store):
                    root=publication.publish(base,None,run)
                seconds=time.perf_counter()-tick
                private_plan=control.read(run/'CANDIDATE_PLAN.json');acceptance=control.read(run/'CANDIDATE_ACCEPTANCE.json')
                if (acceptance['quality_delta']!=accepted['quality_delta'] or acceptance['quality_count']!=accepted['quality_count'] or
                    acceptance['binding_patch']!=accepted['binding_patch']):raise ValueError('ScopeAcceptanceMeaningMismatch')
                store=Store(run/'candidate')
                with store.open(root) as body:
                    if body['plan']!=private_plan or body['acceptance']!=acceptance:raise ValueError('ScopeReadbackMismatch')
                    if file_hash(body['quality_delta'])!=logical_before['transaction']['sha256']:raise ValueError('ScopePayloadByteMismatch')
                result=dict(seconds=seconds,full_cold_checks=calls,independent_readbacks=readbacks,
                    components=checks[-1],quality_delta=acceptance['quality_delta'],quality_count=acceptance['quality_count'],
                    binding_patch=acceptance['binding_patch'],stats=control.read(run/'PUBLICATION.json')['stats'],
                    private_root=root,scope='Private storage mechanics only; no new mesh authority',
                    storage=control.totals([p for p in run.rglob('*') if p.is_file()]))
                durable_json(run/'RESULT.json',result);results[mode]=result;phases[mode]=seconds
                print(dict(mode=mode,seconds=seconds,checks=calls,readbacks=readbacks),flush=True)
            cold=results['cold'];shared=results['shared']
            for key in ('components','quality_delta','quality_count','binding_patch'):
                if cold[key]!=shared[key]:raise ValueError('ScopeComparisonMismatch',key)
            qcount=accepted['quality_count'];acount=logical_before['atlas_rows']
            if cold['full_cold_checks']!=[qcount,acount,qcount,qcount] or shared['full_cold_checks']!=[qcount,acount]:
                raise ValueError('UnexpectedFullReadCoverage')
            if cold['independent_readbacks']!=shared['independent_readbacks'] or shared['independent_readbacks']!=[accepted['quality_count']]*2:
                raise ValueError('MissingIndependentReadback')
            durable_json(OUT/'RESULT.json',dict(passed=True,results=results,authentication=authentication,phases=phases,
                expected_atlas=expected_atlas,scientific_input=plan['scientific'],source_hashes=request['runtime_sources'],
                seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset,
                no_geometry_generated=True,no_proofs_recomputed=True,no_front_published=True))
    except BaseException as error:
        import traceback
        durable_json(OUT/'FAILURE.json',dict(error=repr(error),stage=stage,traceback=traceback.format_exc(),phases=phases))
        raise


def monitor():
    import finish_retained_l5 as m
    m.OUT=OUT/'monitor';m.OUT.mkdir(exist_ok=False)
    return m.monitor(worker_command=[sys.executable,'-B',__file__,'worker'],
        extra_storage_roots=monitored_roots(),
        enforce_output_cap=False)


def monitored_roots():
    # The monitor already measures OUT/monitor. Never count its parent again.
    return [V4/'results/current_front_transition_round_trip',V4/'results/wing_continuation_layers6to8',
        OUT/'cold',OUT/'shared',SOURCE]

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','request-review','worker','monitor']);args=p.parse_args()
    if args.mode=='baseline':baseline()
    elif args.mode=='request-review':request_review()
    elif args.mode=='worker':worker()
    else:sys.exit(monitor())
