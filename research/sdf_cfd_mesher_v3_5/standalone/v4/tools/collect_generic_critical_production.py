"""Collect one reviewed normal control. No production executor or cleanup."""
from pathlib import Path
from fractions import Fraction
from collections import Counter
import sys,time
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from run_critical_reuse_control import read
from record_receipt_resume import disk,status
from path_preserving_deduplication import durable_json,file_hash
from check_current_front_encoding import allocation

OUT=V4/'results/generic_critical_production';RUN=OUT/'run'
PREVIOUS=V4/'results/critical_reuse_production/run/RESULT.json'
OWN=('CURRENT_TASK.md','README.md','GENERIC_CRITICAL_PRODUCTION_REPORT.md',
     'tools/run_critical_reuse_control.py','tests/test_critical_reuse_control.py',
     'tools/collect_generic_critical_production.py','tests/test_generic_critical_collection.py',
     'results/generic_critical_production/')


def configure_output(output=None,previous=None):
    """Read another existing control without overwriting the original evidence."""
    global OUT,RUN,OWN,PREVIOUS
    if output is not None:
        path=Path(output).resolve()
        if not path.is_relative_to((V4/'results').resolve()) or path==(V4/'results').resolve():
            raise ValueError('CriticalOutputOutsideResults')
        OUT=path;RUN=path/'run'
        OWN=(*OWN,str(path.relative_to(V4)).replace('\\','/')+'/',
             'ASSEMBLY_HASH_PRODUCTION_REPORT.md')
        if path.name=='geometry_binding_production':
            OWN=(*OWN,'results/geometry_binding_efficiency/','GEOMETRY_BINDING_EFFICIENCY_REPORT.md',
                 'tools/profile_geometry_binding.py','tests/test_lazy_geometry_binding.py',
                 'tools/composable_transition.py','tools/critical_prism_transition.py',
                 'tools/prism_hotpath_v2.py','tools/critical_assembly.py')
    if previous is not None:
        path=Path(previous).resolve()
        if (not path.is_relative_to((V4/'results').resolve()) or path.name!='RESULT.json' or
            path.is_relative_to(OUT.resolve()) or not path.is_file()):
            raise ValueError('InvalidPreviousBenchmarkResult')
        PREVIOUS=path
NAMES=['INITIAL.json','BASELINE.json','CONTROL.json','REVIEW.json','REVIEW_REQUEST.json',
    'TESTS_REVIEW_FIX.xml','TESTS_CONSUMERS.xml','run/START.json',
    'run/RESULT.json','run/REOPEN.json','run/REFERENCE_COMPARISON.json','run/GENERATED_NEXT.npz',
    'run/GENERATION.json','run/FRESH_CERTIFICATE.json','run/CANDIDATE_PLAN.json','run/CANDIDATE_ACCEPTANCE.json',
    'run/QUALITY_DELTA.json','run/NEW_ATLAS.json','run/NEW_FORECAST_DEPENDENCIES.json','run/BINDING.json',
    'run/NEW_QUALITY.json','run/AFFECTED_INCIDENCE.json','run/VOLUME_DELTA.json','run/DIRECT.json',
    'run/ADMISSION.json','run/FORECAST.json','run/ASSEMBLY.json','run/REUSE.json','monitor/MONITOR_FINAL.json']


def require_result_links(result,reopen,comparison,plan,control,reference_scientific):
    if (result['layer']!=7 or reopen['layer']!=7 or control['layers']!=[7] or
        result['reference']!=reopen['reference'] or result['scientific']!=reopen['scientific'] or
        result['scientific']!=plan['scientific'] or result['scientific']!=comparison['scientific'] or
        plan['parent']!=control['parent']['reference'] or comparison['reference']!=reference_scientific):
        raise ValueError('MixedBenchmarkResultIdentity')


def collect():
    from contextlib import ExitStack
    from immutable_state_access import guard
    with ExitStack() as held:
        for name in NAMES:held.enter_context(guard(OUT/name))
        held.enter_context(guard(PREVIOUS))
        return _collect()


def _collect():
    if (OUT/'SUMMARY_COMPACT.json').exists():raise ValueError('PreserveGenericBenchmarkSummary')
    initial=read(OUT/'INITIAL.json');q=read(OUT/'REVIEW_REQUEST.json')
    review=read(OUT/'REVIEW.json')
    if (review.get('passed') is not True or not review.get('reviewer') or
        review.get('request_sha256')!=file_hash(OUT/'REVIEW_REQUEST.json') or review.get('runtime')!=q['runtime'] or
        any(q[k]!=file_hash(OUT/p) for k,p in [('baseline','BASELINE.json'),('control','CONTROL.json'),('start','run/START.json')])):
        raise ValueError('UnboundBenchmarkReview')
    prefix='research/sdf_cfd_mesher_v4/'
    def unrelated(rows):return [r for r in rows if not any(r[3:]==prefix+p or p.endswith('/') and r[3:].startswith(prefix+p) for p in OWN)]
    if unrelated(status())!=unrelated(initial['status']):raise ValueError('ChangedUnrelatedBenchmarkStatus')
    changed={str((V4/p).resolve()) for p in OWN if not p.endswith('/')}
    protected={p:h for p,h in initial['protected'].items() if str(Path(p).resolve()) not in changed}
    for name,pins in [('protected',protected)]+[(k,q[k]) for k in ('runtime','binary','inputs')]:
        for p,h in pins.items():
            if file_hash(p)!=h:raise ValueError('ChangedBenchmarkDependency',name,p)
    result=read(RUN/'RESULT.json');reopen=read(RUN/'REOPEN.json');monitor=read(OUT/'monitor/MONITOR_FINAL.json')
    if (result.get('passed') is not True or reopen.get('passed') is not True or (RUN/'STOP.json').exists() or
        monitor['exit_code']!=0 or monitor['worker_exit_code']!=0 or monitor['resource_stopped']):
        raise ValueError('IncompleteGenericProductionBenchmark')
    comparison=read(RUN/'REFERENCE_COMPARISON.json')
    plan=read(RUN/'CANDIDATE_PLAN.json');control=read(OUT/'CONTROL.json')
    require_result_links(result,reopen,comparison,plan,control,
        read(V4/'results/fresh_successor_benchmark/current_parent_run/L7/REOPEN.json')['scientific'])
    generation=read(RUN/'GENERATION.json');assembly=read(RUN/'ASSEMBLY.json');certificate=read(RUN/'FRESH_CERTIFICATE.json')
    volume=read(RUN/'VOLUME_DELTA.json');inc=read(RUN/'AFFECTED_INCIDENCE.json');direct=read(RUN/'DIRECT.json')
    quality=read(RUN/'NEW_QUALITY.json');admission=read(RUN/'ADMISSION.json')
    if (not comparison['passed'] or generation['hard_failures'] or not generation['topology_audit']['passed'] or
        not certificate['passed'] or certificate['failures'] or not quality['passed'] or quality['failures'] or
        not inc['passed'] or inc['failures'] or not volume['exact_balance'] or not volume['global_degree']['passed']):
        raise ValueError('UnacceptedBenchmarkEvidence')
    cells=quality['cells'];faces=quality['faces'];exact=quality['exact'];affected=inc['cells']
    extrema=dict(determinant=min(r['determinant'] for r in affected),
        aspect=max(r['aspect'] for r in cells),weight=min(r['weight'] for r in faces),
        skewness=max(r['skewness'] for r in faces),symmetric_volume_ratio=min(r['symmetric_volume_ratio'] for r in faces),
        volume=min(r['volume'] for r in cells),exact_signed_volume=str(min(Fraction(r['signed_volume']) for r in exact)),
        exact_jacobian=str(min(Fraction(r['jacobian']) for r in exact)))
    paths=[p for p in OUT.rglob('*') if p.is_file()];storage=allocation(paths);storage.pop('records')
    previous=read(PREVIOUS)
    if previous.get('passed') is not True or previous.get('layer')!=7:
        raise ValueError('IncompletePreviousBenchmarkResult')
    summary=dict(classification='V4GenericAlignedCriticalProductionSameParentControlVerified',result=result,reopen=reopen,
        comparison=comparison,admission=admission['decision'],forecast=read(RUN/'FORECAST.json'),
        generation=dict(counts=generation['counts'],phases=generation['phases'],topology=generation['topology_audit']),
        gates=dict(extrema=extrema,populations=dict(determinant='Completed affected cells, including prior layer',
            volume_aspect_exact='New cells only',face_metrics='Affected internal faces'),
            hard_failures=generation['hard_failures'],quality_failures=len(quality['failures']),
            incidence_failures=len(inc['failures']),exact_nonpositive_volume=sum(r['volume_sign']<=0 for r in exact),
            exact_nonpositive_jacobian=sum(r['jacobian_sign']<=0 for r in exact),direct_events=direct['events']),
        proofs={k:certificate[k] for k in ('passed','required_pairs','combined_pairs','prior_population','prior_payloads_read','prior_payloads_recomputed','methods','failures')},
        assembly=assembly,degree=dict(passed=volume['global_degree']['passed'],unresolved=volume['global_degree']['unresolved'],
            witnesses=len(volume['global_degree']['values']),
            degree_counts=dict(Counter(r['degree'] for r in volume['global_degree']['values'])),
            scope='Exact degree values remain in VOLUME_DELTA.json; summary carries counts only'),
        exact_volume_balance=volume['exact_balance'],
        reuse=read(RUN/'REUSE.json'),monitor=dict(peaks=monitor['peaks'],seconds=monitor['seconds'],resource_stopped=False,
            tree_memory_scope='Sampled process-tree maximum; process peak working set reported separately'),
        timing_comparison=dict(previous=previous['phases'],current=result['phases'],
            previous_step_seconds=previous['admission_to_publication'],current_step_seconds=result['admission_to_publication'],
            difference_seconds=previous['admission_to_publication']-result['admission_to_publication']),
        allocation_before_summary=storage,disk_initial=initial['disk'],disk_final=disk(),
        preservation=dict(status_entries=len(unrelated(status())),status_exact=True,protected_files=len(protected),
            runtime_files=len(q['runtime']),binaries=len(q['binary']),inputs=len(q['inputs'])),
        elapsed_seconds=time.time()-initial['start'],no_cleanup=True,canonical_last_qualified_layer=8,
        canonical_scientific='e64b53451463bfe764fd2fb5726b5779658419def6eaf37bb2c63de4605be599')
    durable_json(OUT/'SUMMARY_COMPACT.json',summary)
    historical=PREVIOUS
    durable_json(OUT/'EVIDENCE_COMPACT.json',dict(artifacts={n:dict(sha256=file_hash(OUT/n),bytes=(OUT/n).stat().st_size) for n in ['SUMMARY_COMPACT.json',*NAMES]},
        historical_timing=dict(path=str(historical),sha256=file_hash(historical),bytes=historical.stat().st_size),
        starting_head=initial['head'],branch=initial['branch'],scope='One normal production L6->L7 control, not fresh replay or deep qualification; canonical L8 unchanged'))
    print(summary['timing_comparison']);print(extrema);print(storage);print(disk())


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path)
    parser.add_argument('--previous',type=Path)
    args=parser.parse_args();configure_output(args.output,args.previous);collect()
