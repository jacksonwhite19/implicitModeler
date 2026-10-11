"""Audit preserved timings and decisions without replaying a search."""
from task4f_common import *
from collections import Counter
import re

def rejection(row):
    if row.get('accepted'):return 'Accepted'
    if row.get('reason')=='ProjectionFailure':return 'SdfProjectionFailure'
    if not row.get('gate',{}).get('passed',False):return 'ImmediateNumericGate'
    if not row.get('exact',{}).get('passed',True):return 'ImmediateExactJacobian'
    if row.get('new_earlier') or row.get('new_entity_failures'):return 'FutureTrajectoryRegression'
    if row.get('future_spots',{}).get('new_earlier_pair_families'):return 'FutureCollisionRegression'
    return 'NoObjectiveImprovement'

def run():
    rows=[];baselines=[];whole=[];cert=[];exact_records=0
    for layer in range(1,21):
        h=read(BASE/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'))
        timing=read(BASE/('CORRECTION_TIMINGS.json' if layer==1 else f'LAYER{layer}_PREPARE_TIMINGS.json'))
        assert len(h['history'])==len(timing['stages'])
        scopes={(o.get('region',o.get('patch')),o.get('rings',0)):o.get('scope',o.get('patch')) for o in h['outcomes']}
        for i,(r,t) in enumerate(zip(h['history'],timing['stages'])):
            region=r.get('region',r.get('patch'));scope=scopes[(region,r.get('rings',0))];d=scope['detailed'];why=rejection(r)
            n=len(r.get('exact',{}).get('records',[]));exact_records+=n
            rows.append(dict(layer=layer,region=region,trial=i,rings=r.get('rings',0),mode=r['mode'],factor=r['factor'],accepted=r.get('accepted',False),
                rejection=why,active_vertices=len(scope['active']),active_ids=scope['active'],columns=len(d['columns']),core_columns=len(d['core_columns']),
                full_metric_cells=20*len(d['columns']),core_cells=20*len(d['core_columns']),metric_faces=20*len(d['side_edges'])+19*len(d['core_columns']),
                reconstruction_rows=100*len(d['columns']),future_layers_evaluated=list(range(1,21)),
                committed_prefix_cells_recomputed=(layer-1)*len(d['columns']),max_displacement=r.get('displacement'),
                coordinate_hash=r.get('hash',r.get('coordinates_hash')),numeric_violations=r.get('gate',{}).get('violations'),
                cache=r.get('cache'),detail_cache_hit=t.get('detailed_cache_hit'),timing=t,exact_cells=n,
                metric_keys=list(r.get('gate',{}).get('extrema',{})),
                immediate_rejection_full_detail_seconds=t.get('detailed_seconds',0) if why=='ImmediateNumericGate' else 0,
                exact_seconds_after_known_future_regression=t.get('exact_sign_seconds',0) if why=='FutureTrajectoryRegression' else 0))
        for p in timing.get('region_profiles',[]):baselines.append(dict(layer=layer,**p))
        ct=read(BASE/f'LAYER_{layer:02}_TIMINGS.json');cert.append(dict(layer=layer,**ct))
        whole.append(dict(layer=layer,**read(BASE/f'LAYER_{layer:02}_EXACT_SIGNS.json')))
    count=Counter(r['rejection'] for r in rows)
    summary=dict(trials=len(rows),rejections=dict(count),exact_local_cell_calls=exact_records,
        whole_layer_exact_cells=sum(x['cells'] for x in whole),whole_layer_exact_seconds=sum(x['exact_sign_seconds'] for x in cert),
        immediate_rejection_count=count['ImmediateNumericGate'],immediate_rejection_full_detail_seconds=sum(r['immediate_rejection_full_detail_seconds'] for r in rows),
        immediate_rejection_forecast_seconds=sum(r['timing'].get('forecast',{}).get('seconds',0) for r in rows if r['rejection']=='ImmediateNumericGate'),
        numeric_rejection_metric_cells=sum(r['full_metric_cells'] for r in rows if r['rejection']=='ImmediateNumericGate'),
        numeric_rejection_metric_faces=sum(r['metric_faces'] for r in rows if r['rejection']=='ImmediateNumericGate'),
        trial_detail_seconds=sum(r['timing'].get('detailed_seconds',0) for r in rows),baseline_detail_seconds=sum(r['baseline_detailed_seconds'] for r in baselines),
        baseline_hits=sum(bool(r.get('baseline_cache_hit')) for r in baselines),baseline_misses=sum(not r.get('baseline_cache_hit') for r in baselines),
        trial_hits=sum(r['detail_cache_hit'] is True for r in rows),trial_misses=sum(r['detail_cache_hit'] is False for r in rows),
        exact_after_future_regression_seconds=sum(r['exact_seconds_after_known_future_regression'] for r in rows),
        limitation='Historical detailed timers bracket all detailed work, not the exact instant each gate becomes decidable. Those totals are upper bounds on avoidable time; bounded staged experiments measure net avoided time directly.')
    save('ACTIVE_TRIALS.json',rows);save('BASELINE_REFRESHES.json',baselines);save('PRESERVED_SUMMARY.json',summary);save('CERTIFICATION_TIMINGS.json',cert)
    # The original scan printed direct complete-layer metric times.
    text=(BASE/'00_initial_forecast_scan.log').read_text();scan=[dict(layer=int(k),seconds=float(s)) for k,s in re.findall(r'^SCAN (\d+) ([\d.eE+-]+)',text,re.M)]
    save('INITIAL_SCAN_TIMINGS.json',dict(records=scan,sum_seconds=sum(r['seconds'] for r in scan),original=read(BASE/'INITIAL_TIMINGS.json')))
    final=read(BASE/'FINAL_VALIDATION_TIMINGS.json');save('FINAL_DUPLICATION.json',dict(stages=final['stages'],
        metric_cells_recomputed=20*106302,conditioning_cells_recomputed=20*106302,sdf_vertices_requeried=20*53153,
        exact_jacobian_rational_calls=0,certificate_pair_outcomes_rebound=56462864,
        numeric_seconds=sum(r['numeric_seconds'] for r in final['stages']),sdf_seconds=sum(r['sdf_seconds'] for r in final['stages']),
        completeness_seconds=sum(r['certificate_seconds'] for r in final['stages']),total_seconds=final['total_seconds']))
    print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':run()
