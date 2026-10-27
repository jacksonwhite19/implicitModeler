"""Retained-input benchmarks of existing assembly storage/transaction boundaries.

No geometry, trajectory, mesh certificate or committed front is produced here.
Private transaction metadata tests the actual writer, not a surface/prism seal.
"""
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import subprocess
import sys
import time

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
OUT=V4/'results/l6_efficiency_benchmark'
SOURCE=V4/'results/wing_continuation_layers6to8/A/L6'
from path_preserving_deduplication import durable_json,file_hash,metadata
from record_receipt_resume import disk,status
from check_current_front_encoding import allocation
from sdf_cfd_mesher_v4.common.authority import identity


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def totals(paths):return {k:v for k,v in allocation(paths).items() if k!='records'}


INTENTIONAL=('benchmark_retained_l6.py','single_quality_component.py','critical_assembly.py',
             'critical_prepare.py','composable_publication.py')
EXTRA_RUNTIME=('compact_front_atlas.py','critical_quality_codec.py','critical_prism_install.py',
               'critical_quality_lookup.py','recursive_epoch_recovery.py',
               'recursive_replay_retention.py','check_single_quality_storage.py',
               'incremental_transaction.py','path_preserving_deduplication.py','record_receipt_resume.py',
               'check_current_front_encoding.py','prism_receipt_witness.py')


def runtime_paths(baseline):
    return set(baseline['protected'])|{str(V4/'tools'/n) for n in EXTRA_RUNTIME}


def validate_review(baseline,review,current,baseline_hash):
    """Complete independent review pin, with explicit baseline source deltas."""
    if (review.get('passed') is not True or not review.get('reviewer')
        or review.get('baseline_sha256')!=baseline_hash or review.get('runtime_sources')!=current
        or set(current)!=runtime_paths(baseline)):
        raise ValueError('IncompleteReviewedRuntime')
    changes={}
    for p,h in baseline['protected'].items():
        if current[p]!=h:
            if Path(p).name not in INTENTIONAL:raise ValueError('ProtectedImplementationChanged',p)
            changes[p]=dict(before=h,after=current[p])
    if review.get('intentional_changes')!=changes:raise ValueError('UnreviewedImplementationChange')
    if review.get('options')!=dict(codec_workers=4,codec_process=True,single_quality=True,indexed_atlas=True,verified_quality=True):
        raise ValueError('UnreviewedBenchmarkOptions')


def guarded_runtime(held,mode,ordered_atlas=False):
    import immutable_state_access as access
    p=OUT/'BASELINE.json';held.enter_context(access.guard(p));baseline=read(p)
    current={}
    for path in sorted(runtime_paths(baseline)):
        held.enter_context(access.guard(path));current[path]=file_hash(path)
    if mode=='optimized':
        review_path=OUT/'REVIEW.json';held.enter_context(access.guard(review_path))
        review=read(review_path)
        validate_review(baseline,review,current,file_hash(p))
        if ordered_atlas and review.get('atlas_comparison')!='existing_ordered_rows':
            raise ValueError('UnreviewedAtlasComparison')
    else:
        if any(current[p]!=h for p,h in baseline['protected'].items()):raise ValueError('BaselineImplementationChanged')
    return current


def setup():
    if OUT.exists():raise ValueError('PreserveBenchmarkEvidence')
    expected=read(V4/'results/component_epoch_recovery/BASELINE.json')['unrelated']
    own=['research/sdf_cfd_mesher_v4/'+n for n in ('CURRENT_TASK.md','README.md',
        'tools/benchmark_retained_l6.py','tools/compact_front_atlas.py','tests/test_compact_front_atlas.py',
        'L6_EFFICIENCY_BENCHMARK_REPORT.md','results/l6_efficiency_benchmark/')]
    untouched=[r for r in status() if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in own)]
    if untouched!=expected:raise ValueError('ExactUnrelatedStatusChanged')
    prior=read(V4/'results/single_quality_storage/BASELINE.json')['census']['records']
    for row in prior:
        current=metadata(row['path'])
        if any(current[k]!=row[k] for k in ('device','file_id','bytes','mtime_ns','cluster_bytes','links','security','streams')):
            raise ValueError('RetainedScientificMetadataChanged',row['path'])
    sources=[V4/'tools'/n for n in ('critical_assembly.py','critical_prepare.py','critical_transaction.py',
        'composable_publication.py','composable_state.py','single_quality_component.py','current_front_dependencies.py',
        'incremental_pair_storage.py','immutable_state_access.py','finish_retained_l5.py','benchmark_retained_l6.py')]
    sources += [ROOT/'src/sdf/aerospace'/n for n in ('airfoil.rs','wing.rs','naca_spline.rs')]
    plan=read(SOURCE/'CANDIDATE_PLAN.json');acceptance=read(SOURCE/'CANDIDATE_ACCEPTANCE.json')
    if (identity(plan['content'])!=plan['scientific'] or plan['scientific']!='61831ef2187d1adf083bd6a81858cba9ae811b1e1c47db093405db6356a2fade'
        or not acceptance['passed'] or acceptance['plan']!=identity(plan)):
        raise ValueError('RetainedL6AcceptanceMismatch')
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),unrelated=expected,own=own,
        protected={str(p):file_hash(p) for p in sources},disk=disk(),
        inputs={str(p):file_hash(p) for p in (SOURCE/'CANDIDATE_PLAN.json',SOURCE/'CANDIDATE_ACCEPTANCE.json')},
        resources=read(V4/'results/wing_receipt_resume/A/START.json')['resources'],
        source=str(SOURCE),scientific=plan['scientific'],coverage='Existing 651-file campaign census; selected source hashes only.',
        user_authorization='Benchmark retained L6; improve unnecessary storage/calls; rerun and compare with unchanged accuracy. No fixed total-output cap.',
        scope='All retained L6 quality/atlas writer-reader records and actual private transaction mechanics. Not a full transition, fresh mesh certificate, replay or front publication.'))


def guarded_inputs(held):
    import immutable_state_access as access
    from recursive_epoch_recovery import discover
    from recursive_replay_retention import authenticated_closure
    baseline=read(OUT/'BASELINE.json')
    for p,h in baseline['inputs'].items():
        held.enter_context(access.guard(p))
        if file_hash(p)!=h:raise ValueError('ChangedBenchmarkInput',p)
    plan=read(SOURCE/'CANDIDATE_PLAN.json');acceptance=read(SOURCE/'CANDIDATE_ACCEPTANCE.json')
    recovered=discover(V4/'results/component_epoch_recovery/access')
    if recovered['source']!=plan['parent'] or recovered['content']!=plan['content']['parent_content']:
        raise ValueError('WrongRecoveredParentSource')
    base,closure,nodes=held.enter_context(authenticated_closure(recovered['reference']))
    if base.layer!=5 or base.scientific!=plan['content']['parent_scientific']:raise ValueError('WrongAuthenticatedBenchmarkParent')
    reused=0;hashed=0
    for p,r in plan['references'].items():
        held.enter_context(access.guard(p))
        if access.token(p)!=r['token']:raise ValueError('ChangedRetainedReference',p)
        if p in closure and closure[p]['sha256']==r['sha256']:reused+=1
        else:
            if file_hash(p)!=r['sha256']:raise ValueError('ChangedRetainedBytes',p)
            hashed+=Path(p).stat().st_size
    for n,h in acceptance['evidence'].items():
        if file_hash(SOURCE/n)!=h:raise ValueError('ChangedRetainedQualificationEvidence',n)
    base.check()
    return plan,acceptance,dict(parent=base.scientific,root=base.root,nodes=nodes,
        inherited_reference_tokens_reused=reused,new_dependency_bytes_hashed=hashed,references=len(plan['references']))


def compare_atlas(view,values,ordered=False):
    """Same complete comparison; optionally use the existing ordered reader.

    Avoid one point query per record when the task is to compare EVERY record.
    The indexed point-reader remains available and covered separately in tests.
    """
    if len(view)!=len(values):raise ValueError('AtlasCoverageMismatch')
    if ordered:
        for key,row in zip(sorted(values),view.rows,strict=True):
            if row['key']!=key or row['payload']!=values[key]:raise ValueError('AtlasRowMismatch',key)
    else:
        for key,row in values.items():
            if view[key]!=row:raise ValueError('AtlasRowMismatch',key)


def worker(mode,*,ordered_atlas=False):
    import os
    import psutil
    from check_single_quality_storage import RetainedRows
    started=time.perf_counter();run=OUT/mode;run.mkdir(exist_ok=False);phases={};stage='authenticate'
    def timed(name,call):
        nonlocal stage
        stage=name;durable_json(run/'STAGE.json',dict(stage=stage,seconds=time.perf_counter()-started));print(name,flush=True)
        tick=time.perf_counter();value=call();phases[name]=time.perf_counter()-tick
        durable_json(run/'PHASES.json',phases);return value
    try:
        with ExitStack() as held:
            runtime=guarded_runtime(held,mode,ordered_atlas)
            if os.environ.get('V4_QUALITY_CODEC')!='4':raise ValueError('QualifiedCodecControlMismatch')
            from critical_transaction import Store
            import single_quality_component as quality
            from incremental_transaction import canonical
            plan,accepted,authentication=timed('authenticate',lambda:guarded_inputs(held))
            rows=timed('read_quality',lambda:list(RetainedRows(SOURCE/'QUALITY_DELTA.json')))
            expected=timed('hash_quality',lambda:quality.hashes(rows))
            if expected['sha256']!=plan['components']['quality']['delta'] or expected['identity']!=accepted['quality_delta'] or expected['count']!=accepted['quality_count']:
                raise ValueError('RetainedQualityMeaningMismatch')
            atlas=timed('read_atlas',lambda:read(SOURCE/'NEW_ATLAS.json'))
            target=run/'NEW_ATLAS.json';qpath=run/'QUALITY_DELTA.json'
            if mode!='optimized':
                timed('write_atlas',lambda:durable_json(target,atlas))
                timed('write_quality',lambda:durable_json(qpath,rows))
            else:
                from compact_front_atlas import write as atlas_write,snapshot as atlas_snapshot
                timed('write_atlas',lambda:atlas_write(target,atlas,workers=4,process=True))
                timed('write_quality',lambda:quality.write(qpath,rows,workers=4,process=True))
            if mode!='optimized':
                if file_hash(target)!=plan['components']['atlas']['replace']:raise ValueError('AtlasByteMismatch')
                atlas_semantic=file_hash(target)
            else:
                def check_atlas():
                    with atlas_snapshot(target) as snap:
                        if snap['semantic']!=plan['components']['atlas']['replace']:raise ValueError('AtlasMeaningMismatch')
                        compare_atlas(snap['view'],atlas,ordered_atlas)
                        return snap['semantic']
                atlas_semantic=timed('all_atlas_rows',check_atlas)
            store=Store(run/'private_transaction')
            root=store.initialize(dict(epoch=quality.access.epoch(),scientific=plan['content']['parent_scientific'],plan=dict(references={})))
            with quality.snapshot(qpath,retain_verified=mode=='optimized') as snap:
                if snap['semantic']!=expected['sha256']:raise ValueError('QualityMeaningMismatch')
                # Deliberately private STORAGE control; not scientific acceptance of a new front.
                control=dict(scientific=plan['scientific'],references=snap['references'],scope='Retained-L6-storage-control',
                    retained_plan_sha256=file_hash(SOURCE/'CANDIDATE_PLAN.json'))
                bindings=read(SOURCE/'BINDING_PATCH.json')
                acceptance=dict(passed=True,base=root,plan=identity(control),quality_delta=expected['identity'],
                    quality_count=expected['count'],binding_patch=identity(bindings),scope='Private storage mechanics only; retained qualified rows unchanged')
                published=timed('transaction',lambda:store.publish(root,control,snap['rows'],bindings,acceptance,identity(acceptance)))
                for a,b in zip(rows,snap['rows'],strict=True):
                    if canonical(a)!=canonical(b):raise ValueError('QualityRowMismatch')
            def readback():
                import sqlite3,zlib
                with store.open(published) as body:
                    path=Path(body['quality_delta'])
                    with sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True) as db:
                        count=0
                        for key,inputs,payload in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'):
                            original=by_key[key]
                            actual=dict(key=key,inputs=json.loads(zlib.decompress(inputs)),payload=None if payload is None else json.loads(zlib.decompress(payload)))
                            if canonical(actual)!=canonical(original):raise ValueError('TransactionRowMismatch',key)
                            count+=1
                    if count!=expected['count']:raise ValueError('TransactionCoverageMismatch')
                    return dict(count=count,sha256=file_hash(path),bytes=path.stat().st_size)
            by_key={r['key']:r for r in rows};readback_result=timed('independent_transaction_readback',readback)
            measured=totals([p for p in run.rglob('*') if p.is_file()])
            result=dict(passed=True,mode=mode,scope='Full retained L6 assembly-storage boundaries; not whole-transition timing or publication.',
                scientific=plan['scientific'],authentication=authentication,quality=expected,atlas_rows=len(atlas),atlas_semantic=atlas_semantic,
                transaction=readback_result,transaction_stats=dict(store.stats),storage=measured,phases=phases,
                seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset,
                source_hashes=runtime,
                options=dict(single_quality=mode=='optimized',indexed_atlas=mode=='optimized',verified_quality=mode=='optimized',
                    codec_workers=int(os.environ['V4_QUALITY_CODEC']),codec_process=True,
                    atlas_comparison='existing_ordered_rows' if ordered_atlas else 'indexed_point_rows'),
                no_geometry_generated=True,no_proofs_recomputed=True,no_front_published=True)
            durable_json(run/'RESULT.json',result);print(json.dumps(result),flush=True)
    except BaseException as error:
        import traceback
        durable_json(run/'FAILURE.json',dict(error=repr(error),stage=stage,traceback=traceback.format_exc(),phases=phases,
            seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset))
        raise


def monitor(mode):
    import finish_retained_l5 as m
    m.OUT=OUT/(mode+'_monitor');m.OUT.mkdir(exist_ok=False)
    others=[OUT/n for n in ('baseline',) if n!=mode and (OUT/n).exists()]
    return m.monitor(worker_command=[sys.executable,'-B',__file__,'worker',mode],
        extra_storage_roots=[V4/'results/current_front_transition_round_trip',V4/'results/wing_continuation_layers6to8',OUT/mode,*others],
        enforce_output_cap=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['setup','worker','monitor']);parser.add_argument('mode',nargs='?',choices=['baseline','optimized'])
    args=parser.parse_args()
    if args.command=='setup':setup()
    elif args.command=='monitor':sys.exit(monitor(args.mode))
    else:worker(args.mode)
