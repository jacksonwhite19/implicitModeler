"""One capped fresh SF2 execution through MixedFrontRunner and native state."""
import json,os,sys,time,subprocess,traceback
from pathlib import Path
import psutil,numpy as np
from qualify_native_sf2_manifest import OUT,SOURCE
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json

RUN=OUT/'fresh'

def correction_science(reference,actual,layer):
    """Compare decisions and every hard gate, retaining transient diagnostics.

    Conditioning extrema on search trials are descriptive, not hard gates.
    Final committed conditioning arrays are separately required to be exact.
    """
    from task4l_compare import compare_history
    def trial_view(record):
        result=dict(record);result['history']=[]
        for row in record['history']:
            trial=dict(row)
            if 'gate' in row:trial['gate']=dict(row['gate'],extrema=dict(row['gate'].get('extrema',{})))
            result['history'].append(trial)
        return result
    a,b=trial_view(reference),trial_view(actual);diagnostics=[]
    for i,(u,v) in enumerate(zip(a['history'],b['history'])):
        for name in ('sigma','G_bound'):
            x=u.get('gate',{}).get('extrema',{}).pop(name,None)
            y=v.get('gate',{}).get('extrema',{}).pop(name,None)
            if x!=y:diagnostics.append(dict(trial=i,metric=name,reference=x,actual=y))
    return dict(compare_history(a,b,layer),transient_conditioning_differences=diagnostics)

def execute(observer=None):
    from native_prism_production import Provider
    from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
    from sdf_cfd_mesher_v4.advancing_front.native_prism_validation import audit_native_prisms
    from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest
    from sdf_cfd_mesher_v4.common.authority import array_hash
    from task3c_io import read
    from task4l_compare import compare_history,same
    provider=Provider(RUN);runner=MixedFrontRunner(RUN/'checkpoints',provider)
    if observer:observer.mark('authority_load')
    if (RUN/'checkpoints/CURRENT.json').exists():initial=runner.store.restart()
    else:initial=provider.initial();runner.initialize(initial)
    layer=initial['forecast']['current_layer'];del initial
    if observer:observer.mark('initial_front_ready')
    for k in range(layer+1,21):
        if observer:observer.layer=k;observer.mark('before_advance')
        atomic_json(RUN/'PROGRESS.json',dict(stage='advance',layer=k))
        result=runner.step()
        if observer:observer.mark('committed_layer')
        atomic_json(RUN/f'COMMIT_{k:02}.json',result)
        atomic_json(RUN/'EVENT_TIMES.json',provider.records)
        print('NATIVE_COMMIT',k,result['operation'],flush=True)
    state=runner.store.restart();manifest=NativeManifest(RUN/'native_manifest')
    atomic_json(RUN/'PROGRESS.json',dict(stage='final_native_audit',layer=20))
    if observer:observer.stack=['final_validation'];observer.mark('before_final_validation')
    final=audit_native_prisms(state,RUN/'checkpoints/objects',manifest)
    assert final['cells']==2126040 and final['certificate']['active']==56462864
    atomic_json(RUN/'FINAL_NATIVE_AUDIT.json',final)
    # Only after native final validity do reference correction data enter.
    backing=state['cells'].backing;histories=[]
    for k in range(1,21):
        reference=np.load(SOURCE/f'build_A/FRONT_{k:02}.npy',mmap_mode='r')
        assert np.array_equal(reference.view(np.uint64),backing.coordinates(k).view(np.uint64)),f'Coordinates:{k}'
        same(read(SOURCE/f'LAYER_{k:02}_METRIC_ARRAY_HASHES.json'),read(RUN/f'LAYER_{k:02}_METRIC_ARRAY_HASHES.json'),f'Metrics:{k}')
        name='CORRECTION_HISTORY.json' if k==1 else f'LAYER{k}_SEARCH.json'
        histories.append(correction_science(read(SOURCE/name),read(RUN/name),k))
        expected=json.loads((SOURCE/f'build_A/LAYER_{k:02}.json').read_text())['certificate']
        actual=state['certificate'][f'layer:{k}']['ledger']
        for key in ('pair_hash','outcome_hash','candidate_pairs','completed_pairs','geometry_hash','labels_hash'):
            assert expected[key]==actual[key],f'Certificate:{k}:{key}'
    atomic_json(RUN/'SCIENTIFIC_COMPARISON.json',dict(passed=True,layers=20,cells=len(state['cells']),
        relations=final['certificate']['active'],coordinates_equal=True,metric_arrays_equal=True,
        correction_history=histories,remesh_events=len(state['events']),wall=final['wall'],first_layer=final['first_layer']))
    atomic_json(RUN/'PROGRESS.json',dict(stage='qualified',layer=20))

def control(output=None,require_prequalification=True):
    global RUN
    if output is not None:RUN=Path(output)
    if require_prequalification:
        for layer in (1,5,11,12,20):
            row=json.loads((OUT/f'LAYER_{layer:02}_PREQUALIFICATION.json').read_text())
            assert row['native_state'] and row['restart_exact'] and row['audit']['all_hard_receipts_pass']
    else:
        from sdf_cfd_mesher_v4.advancing_front.production_authority import configuration
        assert configuration()['runner']=='native-mixed-front'
    RUN.mkdir(parents=True,exist_ok=True)
    previous=json.loads((RUN/'EXECUTION.json').read_text()) if (RUN/'EXECUTION.json').exists() else {}
    used=previous.get('scientific_seconds',0.)
    if used>=1800:raise ValueError('ScientificBudgetExhausted')
    start=time.perf_counter();peak=0;reviewed=False
    with (RUN/f'EXECUTION_{used:.3f}.log').open('w',encoding='utf8') as log:
        process=subprocess.Popen([sys.executable,__file__,'execute',str(RUN)],stdout=log,stderr=subprocess.STDOUT)
        status='Running'
        while process.poll() is None:
            elapsed=used+time.perf_counter()-start
            try:
                parent=psutil.Process(process.pid)
                peak=max(peak,sum(p.memory_info().rss for p in [parent,*parent.children(recursive=True)] if p.is_running()))
            except (psutil.NoSuchProcess,psutil.AccessDenied):pass
            if elapsed>=1620 and not reviewed:
                reviewed=True;atomic_json(RUN/'MINUTE27.json',dict(seconds=elapsed,policy='Mandatory qualification only; unchanged 1800-second cutoff'))
            if elapsed>=1800:
                try:
                    parent=psutil.Process(process.pid)
                    for child in parent.children(recursive=True):
                        try:child.kill()
                        except psutil.NoSuchProcess:pass
                    parent.kill()
                except psutil.NoSuchProcess:pass
                status='ScientificTimeCap';break
            time.sleep(.2)
        process.wait()
    if status=='Running':status='Completed' if process.returncode==0 else 'ImplementationFailure'
    atomic_json(RUN/'EXECUTION.json',dict(status=status,scientific_seconds=used+time.perf_counter()-start,
        peak_tree_rss=max(peak,previous.get('peak_tree_rss',0)),exit_code=process.returncode))
    print(status,flush=True)

if __name__=='__main__':
    if len(sys.argv)>=2 and sys.argv[1]=='execute':
        if len(sys.argv)==3:RUN=Path(sys.argv[2])
        if os.environ.get('V4_MEMORY_PROFILE'):
            from native_memory_observer import Observer
            observer=Observer(RUN/'memory_profile');observer.install()
            try:execute(observer)
            finally:observer.finish()
        else:execute()
    else:control()
