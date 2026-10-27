"""Post-completion comparison only. Storage envelopes are not mesh science."""
from task4i_common import *

def same(a,b,label):
    if scientific_hash(a)!=scientific_hash(b):raise ValueError('ScientificMismatch:'+label)
def spot_science(spot):
    return {k:([dict(sides=r['sides'],before=r['before']['classification'],after=r['after']['classification']) for r in v]
        if k=='records' else v) for k,v in spot.items()}
def trial_identity(r):
    keys=('region','patch','rings','factor','mode','hash','coordinates_hash','reason')
    return dict(**{k:r[k] for k in keys if k in r},accepted=bool(r.get('accepted',False)))
def compare_history(a,b,layer):
    same(a.get('outcomes'),b.get('outcomes'),f'outcomes:{layer}')
    same(a.get('corner_traces'),b.get('corner_traces'),f'corner_traces:{layer}')
    aa=a['history'];bb=b['history'];same(len(aa),len(bb),f'trial_count:{layer}');early=[]
    for i,(u,v) in enumerate(zip(aa,bb)):
        same(trial_identity(u),trial_identity(v),f'trial:{layer}:{i}')
        if v.get('future_evaluated') is False or v.get('rejection_stage'):
            if v['accepted']:raise ValueError('EarlyRejectedTrialAccepted')
            early.append(i);continue
        for key in ('gate','exact','first_before','first_after','first_future_before','first_future_after','new_earlier','ring_earlier',
                    'target_mechanism','baseline_mechanism','candidate_mechanism','target_pair_comparison','displacement','forecast_hash','new_entity_failures'):
            if key in u or key in v:same(u.get(key),v.get(key),f'{key}:{layer}:{i}')
        if 'future_spots' in u or 'future_spots' in v:same(spot_science(u['future_spots']),spot_science(v['future_spots']),f'spots:{layer}:{i}')
    return dict(layer=layer,trials=len(aa),accepted=sum(bool(r.get('accepted')) for r in bb),early_rejection_records=early,
        decisions_hash=scientific_hash([trial_identity(r) for r in bb]))

def run():
    start=time.perf_counter();reference=ROOT/'results/task4e/a2'
    # No reference is opened until the fresh final qualification exists.
    final=read(RUN/'FINAL_VALIDATION.json');assert final['layers']==20 and final['certificate']['complete']
    ref=read(reference/'FINAL_VALIDATION.json');coverage=[];histories=[];metric_seconds=0.
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));low=front.coordinates;previous=None
    for layer in range(1,21):
        a=read(reference/f'build_A/LAYER_{layer:02}.json');b=read(RUN/f'build_A/LAYER_{layer:02}.json')
        x=np.load(reference/f'build_A/FRONT_{layer:02}.npy');y=np.load(RUN/f'build_A/FRONT_{layer:02}.npy')
        assert np.array_equal(x.view(np.uint64),y.view(np.uint64)),f'CoordinateMismatch:{layer}'
        envelope={'certificate','certificate_hash','state_hash','previous'}
        same({k:v for k,v in a.items() if k not in envelope},{k:v for k,v in b.items() if k not in envelope},f'checkpoint:{layer}')
        proof_keys=('candidate_pairs','completed_pairs','pair_hash','outcome_hash','forbidden','unresolved','invalid_surfaces','complete','geometry_hash','labels_hash','topology')
        proof={k:b['certificate'][k] for k in proof_keys};same({k:a['certificate'][k] for k in proof_keys},proof,f'certificate:{layer}');coverage.append(proof)
        history='CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'
        histories.append(compare_history(read(reference/history),read(RUN/history),layer))
        with np.load(reference/f'LAYER_{layer:02}_CONDITIONING.npz') as u,np.load(RUN/f'LAYER_{layer:02}_CONDITIONING.npz') as v:
            same(u.files,v.files,f'conditioning_keys:{layer}')
            for key in u.files:assert np.array_equal(u[key].view(np.uint8),v[key].view(np.uint8)),f'ConditioningMismatch:{layer}:{key}'
        # The old run stored extrema, not full hard-metric arrays. Reconstruct
        # its arrays from reference geometry with the qualified same definitions.
        t=time.perf_counter();r=evaluate_layer(top,low,x,layer,previous);metric_seconds+=time.perf_counter()-t
        hashes={family:{k:array_hash(v) for k,v in r[family].items() if isinstance(v,np.ndarray)}
            for family in ('cells','sides','caps') if family in r and isinstance(r[family],dict)}
        same(hashes,read(RUN/f'LAYER_{layer:02}_METRIC_ARRAY_HASHES.json'),f'metric_arrays:{layer}')
        previous=r['cells'];low=x
        save('COMPARISON_PROGRESS.json',dict(layer=layer,passed=True))
    same([spot_science(s) for s in read(reference/'LAYER1_FUTURE_SPOTS.json')],[spot_science(s) for s in read(RUN/'LAYER1_FUTURE_SPOTS.json')],'initial_spots')
    excluded={'certificate','shell_scientific_hash','transaction_state_hash','complete_transaction_history_sha256','initial_future_spots_sha256'}
    science={k:v for k,v in final.items() if k not in excluded};same({k:v for k,v in ref.items() if k not in excluded},science,'final_values')
    science.update(certificate=coverage,decisions=histories)
    with np.load(reference/'FINAL_WALL_PRISM_SHELL.npz') as a,np.load(RUN/'FINAL_WALL_PRISM_SHELL.npz') as b:
        for key in a.files:assert np.array_equal(a[key].view(np.uint8),b[key].view(np.uint8)),key
    with np.load(reference/'FINAL_CONDITIONING.npz') as a,np.load(RUN/'FINAL_CONDITIONING.npz') as b:
        for key in a.files:assert np.array_equal(a[key].view(np.uint8),b[key].view(np.uint8)),key
    result=dict(passed=True,layers=20,coordinates_equal=True,connectivity_equal=True,ownership_equal=True,hard_metric_arrays_equal=True,
        hard_metric_reference_method='Reconstructed from preserved reference coordinates with the qualified metric definitions; extrema and conditioning also compared to frozen records.',
        exact_jacobian_records_equal=True,conditioning_arrays_equal=True,required_pair_population=sum(x['candidate_pairs'] for x in coverage),
        pair_and_outcome_hashes_equal=True,forbidden=[],unresolved=[],histories=histories,scientific_mesh_identity=scientific_hash(science),
        raw_fresh=final['shell_scientific_hash'],raw_reference=ref['shell_scientific_hash'],raw_reference_build_A=read(ROOT/'results/task4/FINAL_VALIDATION.json')['shell_scientific_hash'],
        raw_difference_reason='Staged rejection omits unnecessary rejected-trial diagnostics; future proof representation changed. All retained science compared above.',
        seconds=time.perf_counter()-start,reference_metric_seconds=metric_seconds)
    save('SCIENTIFIC_COMPARISON.json',result);print('SCIENCE_IDENTICAL',result['scientific_mesh_identity'],flush=True)
if __name__=='__main__':run()
