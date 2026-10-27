"""Replay the repaired path and new searches; reuse sealed completed prefixes."""
from task3_root_regression import *
from task3_repaired_trial import replay as replay_trial
from task3_repair_study import load,components,run_component
from sdf_cfd_mesher_v4.common.authority import file_hash
import psutil

def read(n):return json.loads((OUT/n).read_text())

def run():
    started=time.perf_counter();root=qualify();hroot=scientific_hash(root)
    if hroot!=scientific_hash(read('ROOT_REGRESSION.json')):
        write('ROOT_REPLAY_MISMATCH.json',root)
        raise AssertionError('RootFixtureReplayDrift')
    context=load();trial=replay_trial(context)
    assert scientific_hash(trial)==scientific_hash(read('BLOCKED_TRIAL_REPLAY.json')),'BlockedTrialReplayDrift'
    results=[];timings={};hashes=[]
    for spec in components()[1:]:
        r,p=run_component(spec,context);expected=read(spec['name']+'.json')
        a,b=scientific_hash(expected),scientific_hash(r)
        if a!=b:
            write('MISMATCH_'+spec['name']+'.json',r)
            raise AssertionError(('ScientificReplayDrift',spec['name'],a,b))
        hashes.append(dict(component=spec['name'],run1=a,run2=b,passed=True));results.append(r);timings[spec['name']]=p
        write('REPLAY_PROGRESS.json',hashes)
        print(spec['name'],r['final']['classification'],a,flush=True)
    inherited={name:file_hash(ROOT/path) for name,path in dict(
        component_00='results/task3/component_00.json',component_01_prefix='results/task3/component_01.json',
        component_03_prefix='results/task3_certificate_repair/INITIAL_component_03.json').items()}
    expected=[read(s['name']+'.json') for s in components()[1:]]
    h1=scientific_hash(dict(root=read('ROOT_REGRESSION.json'),trial=read('BLOCKED_TRIAL_REPLAY.json'),components=expected,inherited=inherited))
    h2=scientific_hash(dict(root=root,trial=trial,components=results,inherited=inherited))
    assert h1==h2
    before=read('REDUNDANT_BASELINE_REPRODUCER.json')['failed_event']['future_certificate']
    after=read('component_03.json')['final']['future_certificate']
    assert before==after,'CompletedFutureCertificateChanged'
    write('DETERMINISM.json',dict(passed=True,run1=h1,run2=h2,components=hashes,root_fixture_hash=hroot,blocked_trial_hash=scientific_hash(trial),
                                inherited=inherited,completed_future_certificate_before_after_identical=True,
                                future_certificate_stream_hash=after['records_sha256'],
                                scope='All newly executed complete scientific histories, metrics, projections, accepted coordinates, and exact records replayed. Sealed leading-cap 60 trials and rejected prefixes are hash-bound and not re-executed. Timing-bound abandoned baseline traversal is preserved as operational evidence, not a final scientific result.'))
    write('REPLAY_TIMINGS.json',dict(components=timings,seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset))
    print(json.dumps(dict(passed=True,scientific_hash=h1,seconds=time.perf_counter()-started)))

if __name__=='__main__':run()
