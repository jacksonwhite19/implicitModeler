"""Read-only actual preparation authentication boundary on retained L6 files.

No mocked preparation, generated state, proof rewrite or publication. The full
restart index/receipt requirements are reported separately, not bypassed.
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch

V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
import benchmark_retained_l6 as control
from path_preserving_deduplication import durable_json,file_hash
from record_receipt_resume import disk,status
from sdf_cfd_mesher_v4.common.authority import identity

OUT=V4/'results/prepare_reference_cost'
OBS=OUT/'observations'
OWN=('CURRENT_TASK.md','README.md','PREPARATION_REFERENCE_COST_REPORT.md',
     'tools/preparation_reference.py','tools/critical_prepare.py','tools/critical_transaction.py',
     'tools/benchmark_prepare_references.py','tests/test_preparation_reference.py',
     'results/prepare_reference_cost/')


def runtime():
    return {str(V4/'tools'/n):file_hash(V4/'tools'/n) for n in
            ('preparation_reference.py','critical_prepare.py','critical_transaction.py','immutable_state_access.py',
             'critical_index_pipeline.py','prism_receipt_witness.py','benchmark_prepare_references.py',
             'benchmark_retained_l6.py','finish_retained_l5.py','path_preserving_deduplication.py',
             'critical_transaction.py','incremental_transaction.py','recursive_replay_retention.py',
             'recursive_epoch_recovery.py')}


def request():
    p=OUT/'REVIEW_REQUEST.json'
    if p.exists():raise ValueError('PreserveReviewRequest')
    from critical_transaction import Store
    private=V4/'results/guarded_publication_scope/shared/candidate'
    store=Store(private);pointer=store.current()
    expected=control.read(private.parent/'RESULT.json')['private_root']
    if pointer['root']!=expected:raise ValueError('WrongPrivateControlRoot')
    pins={str(q):file_hash(q) for q in (private/'CURRENT.json',private/('ROOT_'+expected+'.json'),private.parent/'RESULT.json')}
    durable_json(p,dict(runtime=runtime(),baseline=file_hash(OUT/'BASELINE.json'),private_root=expected,private_pins=pins,
        scope='Read-only exact reference authentication comparison; no full preparation or front publication',
        resources=dict(output_cap=None,minimum_free_bytes=45120503439)))


def worker():
    import psutil
    import immutable_state_access as access
    import preparation_reference as repair
    started=time.perf_counter();stage='review'
    try:
        request=control.read(OUT/'REVIEW_REQUEST.json');review=control.read(OUT/'REVIEW.json')
        if (not review.get('passed') or not review.get('reviewer') or
            review['request_sha256']!=file_hash(OUT/'REVIEW_REQUEST.json') or
            review['runtime']!=runtime() or request['runtime']!=runtime() or
            request['baseline']!=file_hash(OUT/'BASELINE.json')):raise ValueError('UnreviewedPreparationControl')
        with ExitStack() as held:
            for path,h in runtime().items():
                held.enter_context(access.guard(path))
                if file_hash(path)!=h:raise ValueError('ChangedReviewedSource',path)
            for path,h in request['private_pins'].items():
                held.enter_context(access.guard(path))
                if file_hash(path)!=h:raise ValueError('ChangedPrivateControlRoot',path)
            control.OUT=OUT
            tick=time.perf_counter();stage='authentication'
            plan,acceptance,auth=control.guarded_inputs(held)
            auth_seconds=time.perf_counter()-tick
            # Exact files used by _prepare's newly produced reference loop.
            # Quality, atlas and forecast already supply snapshot references.
            names=('GEOMETRY_DELTA.json','BINDING_PATCH.json','NEW_RECEIPT_INPUTS.json',
                   'CANDIDATE_HOT.json','GENERATION.json','ASSEMBLY.json',
                   'FRESH_CERTIFICATE.json','VOLUME_DELTA.json','ADMISSION.json','AFFECTED_INCIDENCE.json')
            paths=[control.SOURCE/n for n in names]
            cert=control.read(control.SOURCE/'FRESH_CERTIFICATE.json')
            paths += [control.SOURCE/'objects'/cert['segment'][k]['file'] for k in ('faces','pairs','codes','payloads')]
            index_path=control.SOURCE/'objects/new-proof-key-index.sqlite'
            paths.append(index_path)
            if len(paths)!=15 or len(set(paths))!=15:raise ValueError('IncompletePreparationFiles')
            expected={str(p.resolve()):plan['references'][str(p.resolve())] for p in paths}
            from critical_transaction import Store
            private=V4/'results/guarded_publication_scope/shared/candidate'
            store=Store(private)
            if store.current()['root']!=request['private_root']:raise ValueError('WrongPrivateControlRoot')
            body=held.enter_context(store.open(request['private_root']))
            # Actual previous single-component transaction outputs; expected
            # refs come from its authenticated existing root, not a new assertion.
            if len(body['produced'])!=2:raise ValueError('IncompleteProducedFiles')
            for path,record in body['produced'].items():
                p=Path(path);held.enter_context(access.guard(p))
                if access.token(p)!=record['token']:raise ValueError('ChangedTransactionDependency')
                paths.append(p);expected[str(p.resolve())]=record
            if len(set(paths))!=17:raise ValueError('DuplicateDependency')
            modes={}
            for mode in ('old','single_pass'):
                stage=mode;calls=[];real=access.file_hash
                def measured(p):
                    calls.append(dict(path=str(Path(p).resolve()),bytes=Path(p).stat().st_size))
                    return real(p)
                tick=time.perf_counter();records={}
                durable_json(OBS/'STAGE.json',dict(stage=stage,seconds=time.perf_counter()-started))
                with patch.object(access,'file_hash',measured),patch.object(repair,'file_hash',measured):
                    for p in paths:
                        record=access.attest(p,measured(p)) if mode=='old' else repair.attest_content(p)
                        if record!=expected[str(p.resolve())]:raise ValueError('ReferenceAuthenticationMismatch',str(p))
                        records[str(p.resolve())]=record
                modes[mode]=dict(seconds=time.perf_counter()-tick,records=records,calls=calls,
                    logical_bytes_hashed=sum(c['bytes'] for c in calls),
                    scope='SHA-256 input bytes, not physical disk traffic')
                print(mode,modes[mode]['seconds'],flush=True)
            if modes['old']['records']!=modes['single_pass']['records']:raise ValueError('RecordMismatch')
            if (len(modes['old']['calls'])!=len(paths)*2 or len(modes['single_pass']['calls'])!=len(paths)
                or modes['old']['logical_bytes_hashed']!=2*modes['single_pass']['logical_bytes_hashed']):
                raise ValueError('UnexpectedHashCoverage')
            base=control.read(OUT/'BASELINE.json')
            intentional={str(V4/'tools/critical_transaction.py')}
            for p,h in {**base['protected'],**base['inputs']}.items():
                if p in intentional:continue
                if file_hash(p)!=h:raise ValueError('PreservationMismatch',p)
            own=lambda r:any(r[3:]=='research/sdf_cfd_mesher_v4/'+n or n.endswith('/') and r[3:].startswith('research/sdf_cfd_mesher_v4/'+n) for n in OWN)
            if [r for r in status() if not own(r)]!=base['unrelated']:raise ValueError('UnrelatedStatusMismatch')
            preparation=control.read(control.SOURCE/'PREPARE_METRICS.json')
            durable_json(OBS/'RESULT.json',dict(passed=True,scientific=plan['scientific'],parent=auth['parent'],
                authentication=auth,authentication_seconds=auth_seconds,modes=modes,
                retained_preparation_seconds=preparation['seconds'],
                restart=dict(proof_index_rows=cert['required_pairs'],index_bytes=index_path.stat().st_size,
                    prior_face_hashes=preparation['old_face_hashes_for_new_interaction_bindings'],
                    requirement='Existing IndexJob restart rebuilds with the unchanged qualified writer and compares exact bytes. No live same-process receipt witness can be fabricated from a file.'),
                seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset,
                disk=disk(),runtime=request['runtime'],intentional_before={p:base['protected'][p] for p in intentional},no_state_write=True,no_layer_advanced=True,
                limits='Cold old pass first; OS cache order not isolated. Not full preparation, transaction or whole-layer timing.'))
    except BaseException as e:
        import traceback
        durable_json(OBS/'FAILURE.json',dict(stage=stage,error=repr(e),traceback=traceback.format_exc()))
        raise


def monitor():
    import finish_retained_l5 as m
    OBS.mkdir(exist_ok=False)
    m.OUT=OUT/'monitor';m.OUT.mkdir(exist_ok=False)
    return m.monitor(worker_command=[sys.executable,'-B',__file__,'worker'],
        extra_storage_roots=[control.SOURCE.parent.parent,V4/'results/current_front_transition_round_trip',OUT/'observations'],
        enforce_output_cap=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['request','worker','monitor']);a=p.parse_args()
    if a.mode=='monitor':sys.exit(monitor())
    else:globals()[a.mode]()
