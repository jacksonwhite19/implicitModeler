"""Retained-input CPU measurement at the existing capsule codec boundary."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]; ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
from path_preserving_deduplication import durable_json, file_hash
from record_receipt_resume import disk, status

OUT = V4/'results/capsule_fast_reuse'
INPUT = V4/'results/fresh_successor_benchmark/current_parent_run/L7/NEW_FORECAST_DEPENDENCIES.json'
OWN = ('CURRENT_TASK.md', 'README.md', 'CAPSULE_FAST_REUSE_REPORT.md',
       'tools/shared_schedule_hotpath.py', 'tools/check_capsule_critical_speed.py',
       'tools/assembly_quality_handoff.py', 'tests/test_assembly_quality_handoff.py',
       'tools/private_quality_codec_reuse.py','tests/test_private_quality_codec_reuse.py',
       'tools/sealed_front_connection.py',
       'tests/test_capsule_reuse_dispatch.py',
       'tests/test_shared_schedule_hotpath.py', 'results/capsule_fast_reuse/')


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def unrelated():
    prefix = 'research/sdf_cfd_mesher_v4/'
    return [r for r in status() if not any(r[3:] == prefix+p or
            p.endswith('/') and r[3:].startswith(prefix+p) for p in OWN)]


def baseline():
    if OUT.exists():
        raise ValueError('PreserveCriticalSpeedEvidence')
    old = read(V4/'results/live_wing_warm_fast/BASELINE.json')
    pins = dict(old['protected'])
    pins.update(old['inputs'])
    for name in ('current_front_dependencies.py', 'critical_assembly.py', 'critical_prepare.py',
                 'accepted_production_contract.py', 'accepted_trajectory_contract.py'):
        p = V4/'tools'/name; pins[str(p)] = file_hash(p)
    for p, h in pins.items():
        if file_hash(p) != h:
            raise ValueError('ChangedProtectedCriticalInput', p)
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json', dict(start=time.time(),
        head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        status=unrelated(), protected=pins, disk=disk()))


def benchmark():
    import current_front_dependencies as codec
    from shared_schedule_hotpath import installed
    body = read(INPUT); source = Path(body['shared']['path']); shared = read(source)
    hashes = {str(p): file_hash(p) for p in (INPUT, source)}
    result = dict(rows=len(body['rows']), unique_schedules=len(shared['schedules']),
                  expanded_bytes=body['expanded_bytes'], inputs=hashes, phases={})
    for mode in ('original', 'shared'):
        from contextlib import nullcontext
        with (installed() if mode == 'shared' else nullcontext()):
            start = time.perf_counter(); decoded = codec._validated(body, shared)
            validation = time.perf_counter()-start
            def values():
                for key, row in body['rows'].items():
                    yield key, dict(current_layer=row['current_layer'], region=row['region'],
                                    remaining_schedule=decoded[row['schedule']])
            start = time.perf_counter(); encoded = codec.encode_rows(values())
            writing = time.perf_counter()-start
        if encoded != (body['rows'], shared['schedules'], body['expanded_sha256'], body['expanded_bytes']):
            raise ValueError('ChangedDependencyMeaningOrBytes')
        result['phases'][mode] = dict(validation_seconds=validation, encoding_seconds=writing,
            rows_and_schedules_exact=True, expanded_identity_exact=True)
    if hashes != {str(p): file_hash(p) for p in (INPUT, source)}:
        raise ValueError('RetainedDependencyInputChanged')
    import psutil
    result['memory'] = psutil.Process().memory_info()._asdict()
    result['source'] = {str(V4/'tools'/n): file_hash(V4/'tools'/n) for n in
                        ('current_front_dependencies.py', 'shared_schedule_hotpath.py')}
    durable_json(OUT/'CODEC.json', result)
    print(json.dumps(result, indent=2))


def quality_boundary(*, private=False):
    """Actual private writer and original/new guarded readers; no new prism."""
    from contextlib import ExitStack
    import psutil
    import immutable_state_access as access
    import single_quality_component as quality
    import assembly_quality_handoff as handoff
    from run_successor_benchmark import execution_sources
    source = INPUT.parent/'QUALITY_DELTA.json'
    plan_path = INPUT.parent/'CANDIDATE_PLAN.json'
    result = dict(scope='Retained L7 actual writer/readers only; not whole-transition timing', phases={})
    folder='quality_private' if private else 'quality_boundary'
    target = OUT/folder/'QUALITY_DELTA.json'
    if target.parent.exists():
        raise ValueError('PreserveQualityBoundaryEvidence')
    target.parent.mkdir()
    start = time.perf_counter()
    def timed(name, call):
        print(name, flush=True); tick = time.perf_counter()
        value = call(); result['phases'][name] = time.perf_counter()-tick
        return value
    with ExitStack() as held:
        review_path=OUT/('QUALITY_PRIVATE_REVIEW.json' if private else 'QUALITY_REVIEW.json')
        held.enter_context(access.guard(review_path)); review=read(review_path)
        pins=execution_sources([V4/'tools'/n for n in
                               ('check_capsule_critical_speed.py','assembly_quality_handoff.py')])
        if review.get('passed') is not True or not review.get('reviewer') or review.get('runtime')!=pins:
            raise ValueError('UnreviewedQualityHandoffBenchmark')
        for p,h in pins.items():
            held.enter_context(access.guard(p))
            if file_hash(p)!=h:raise ValueError('ChangedQualityHandoffRuntime',p)
        held.enter_context(access.guard(plan_path)); plan = read(plan_path)
        held.enter_context(access.guard(source))
        record = plan['references'][str(source.resolve())]
        if file_hash(source) != record['sha256'] or access.token(source) != record['token']:
            raise ValueError('ChangedRetainedQualityEnvelope')
        source_body = read(source); payload = Path(source_body['payload']['path'])
        held.enter_context(access.guard(payload))
        if plan['references'][str(payload.resolve())] != {k:source_body['payload'][k] for k in ('sha256','token')}:
            raise ValueError('UnboundRetainedQualityPayload')
        if file_hash(payload) != source_body['payload']['sha256'] or access.token(payload) != source_body['payload']['token']:
            raise ValueError('ChangedRetainedQualityPayload')
        def load():
            with quality.snapshot(source, retain_verified=True) as snap:
                if snap['semantic'] != plan['components']['quality']['delta']:
                    raise ValueError('RetainedQualitySemanticMismatch')
                return list(snap['rows'])
        rows = timed('cold_source_validation_and_load', load)
        with handoff.installed(reuse_private_codec=private) as stats:
            import cProfile, pstats
            profiler=cProfile.Profile(); profiler.enable()
            try:
                expected = timed('accepted_byte_private_writer' if private else 'unchanged_private_writer', lambda: quality.write(target, rows, workers=4, process=True))
            finally:
                profiler.disable(); profiler.dump_stats(str(OUT/folder/'WRITER.prof'))
            if any(expected[k] != source_body['logical'][k] for k in ('identity','sha256','bytes','count')):
                raise ValueError('ChangedWrittenQualityMeaning')
            def verify():
                with quality.snapshot(target, retain_verified=True) as snap:
                    if quality.quality_identity(snap['rows']) != expected['identity']:
                        raise ValueError('ChangedHandoffQualityIdentity')
                    return dict(semantic=snap['semantic'],count=len(snap['rows']))
            warm = timed('writer_to_publication_handoff', verify)
            result['handoff'] = dict(stats)
        # A cold process/retry still performs the original full logical check.
        cold = timed('original_cold_publication_reader', verify)
        if warm != cold:
            raise ValueError('WriterColdHandoffMismatch')
        with quality.snapshot(target, retain_verified=True) as snap:
            if list(snap['rows']) != rows:
                raise ValueError('FullWrittenQualityRowMismatch')
        generated=Path(read(target)['payload']['path'])
        result['database_byte_exact']=file_hash(generated)==source_body['payload']['sha256']
        result.update(rows=len(rows),quality=source_body['logical'],equivalence=dict(all_rows_exact=True,
            logical_identity_exact=True,readers_exact=True),runtime=pins,inputs={str(p):file_hash(p) for p in
            (source,payload,plan_path)},seconds=time.perf_counter()-start,
            peak_working_set=psutil.Process().memory_info().peak_wset,disk=disk())
        import io
        report=io.StringIO(); pstats.Stats(profiler,stream=report).sort_stats('cumulative').print_stats(35)
        result['writer_profile']=report.getvalue()
        if any(file_hash(p)!=h for p,h in pins.items()):
            raise ValueError('ChangedQualityBoundaryRuntime')
        durable_json(OUT/('QUALITY_PRIVATE.json' if private else 'QUALITY_BOUNDARY.json'), result)
        print(json.dumps({k:v for k,v in result.items() if k not in ('runtime','writer_profile')}, indent=2), flush=True)


def reviewed():
    """Record the completed independent source review; never self-approve drift."""
    from run_successor_benchmark import execution_sources
    pins = {'assembly_quality_handoff.py':'ca651b28315422e0b6ae2b304e68f149cb08d50d97936ea44fe4241ae3045a3a',
            '../tests/test_assembly_quality_handoff.py':'f9906e5891c3a948b008cb005287a4091a1655350fd789261359b99d4c2ce9d6'}
    if any(file_hash(V4/'tools'/p) != h for p,h in pins.items()):
        raise ValueError('ChangedIndependentlyReviewedHandoff')
    path=OUT/'QUALITY_REVIEW.json'
    if path.exists():raise ValueError('PreserveQualityReview')
    durable_json(path,dict(passed=True,reviewer='McClintock/01a0fdf2-5c6c-7842-ae77-5d20adffcc37',
        scope='Independent source PASS for bounded retained-row writer/readers only; not production qualification or whole-layer speed',
        reviewed=pins,runtime=execution_sources([V4/'tools'/n for n in
                ('check_capsule_critical_speed.py','assembly_quality_handoff.py')]),
        fixes='Envelope readback; provisional registration; borrower guard lifetime; concurrent installer/thread isolation',
        tests='32 focused tests passed; expanded regression suite running',
        source_review_only=True))


def private_review_request():
    from run_successor_benchmark import execution_sources
    path=OUT/'QUALITY_PRIVATE_REQUEST.json'
    if path.exists():raise ValueError('PreservePrivateReviewRequest')
    durable_json(path,dict(runtime=execution_sources([V4/'tools'/n for n in
                ('check_capsule_critical_speed.py','assembly_quality_handoff.py')]),
        scope='Same retained 690963 L7 rows, original writer/storage format and independent readbacks. No new cells/publication.',
        tests={str(p):file_hash(p) for p in (V4/'tests').glob('test_*') if p.name in
               ('test_private_quality_codec_reuse.py','test_assembly_quality_handoff.py','test_capsule_reuse_dispatch.py')},
        reference=file_hash(OUT/'QUALITY_BOUNDARY.json'), baseline=file_hash(OUT/'BASELINE.json')))


def private_reviewed():
    from run_successor_benchmark import execution_sources
    pins={'tools/private_quality_codec_reuse.py':'519bf122e813bb67e8fcd73e6cc58171edfec1ea413d549ead27ff67f36266f1',
          'tests/test_private_quality_codec_reuse.py':'c1cf56aacd91340122d5b9e2c4a1cefa5f77eefcad00fc47fcbc83e84299a880',
          'tools/shared_schedule_hotpath.py':'87078f17185435e6ebe7d8aa8227f6f86b8d21360039e8f040e4acb39fd720e5',
          'tools/sealed_front_connection.py':'51b16fc9eb3e1247a517e699f928fb8d2765e81a049e323c59d5f6561d346b08'}
    if any(file_hash(V4/p)!=h for p,h in pins.items()):
        raise ValueError('ChangedIndependentlyReviewedPrivateCodec')
    q=read(OUT/'QUALITY_PRIVATE_REQUEST.json')
    runtime=execution_sources([V4/'tools'/n for n in ('check_capsule_critical_speed.py','assembly_quality_handoff.py')])
    if q['runtime']!=runtime:raise ValueError('StalePrivateCodecReviewRequest')
    path=OUT/'QUALITY_PRIVATE_REVIEW.json'
    if path.exists():raise ValueError('PreservePrivateReview')
    durable_json(path,dict(passed=True,reviewer='McClintock/01a0fdf2-5c6c-7842-ae77-5d20adffcc37 and Singer/01a0fcd0-4b45-7e63-ae5b-b5c384de9fad',
        runtime=runtime,reviewed=pins,request=file_hash(OUT/'QUALITY_PRIVATE_REQUEST.json'),
        source_review_only=True,scope='Bounded retained private-writer benchmark; no whole-transition speed claim'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('command', choices=('baseline','codec','quality','quality-private','review','private-review-request','private-review'))
    args = parser.parse_args(); {'baseline': baseline, 'codec': benchmark, 'quality':quality_boundary,
                                'quality-private':lambda:quality_boundary(private=True),'review':reviewed,
                                'private-review-request':private_review_request,'private-review':private_reviewed}[args.command]()
