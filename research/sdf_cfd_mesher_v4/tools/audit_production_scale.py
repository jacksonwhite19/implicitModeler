"""Summarize actual attempts; absent later stages never become passing rows."""
import gzip
import json
from collections import Counter
from pathlib import Path
import geometry_family_authority as geometry
import production_scale_authority as authority
import generalize_local_remesh as g
import final_shell_incidence as incidence
from run_production_scale import OUT
from sdf_cfd_mesher_v4.common.authority import identity,file_hash


def case_result(spec, label):
    folder=OUT/spec['name']/label
    failure=json.loads((folder/'FAILURE.json').read_text())
    if failure['reason']!='ValueError:FirstLayerCorrectionExhausted':
        raise ValueError('UnresolvedImplementationFailure:'+failure['reason'])
    initial=json.loads((folder/'FIRST_LAYER.json').read_text())
    history=json.loads((folder/'FIRST_LAYER_CORRECTION.json').read_text())
    trials=[trial for phase in history for trial in phase['trials']]
    quality=initial['quality']
    last_accepted=None
    for trial in trials:
        if trial.get('gate',{}).get('passed'):
            last_accepted=trial['gate']
    return dict(case=spec['name'],label=label,
        classification='ProductionScaleGeometryUnsupported',reason='FixedDesignFirstLayerCorrectionExhausted',
        scope='This wall extraction, fixed dimensions and h1; not a shape-wide infeasibility proof',
        initial_failure_types=dict(Counter(r['kind'] for r in quality['failures'])),
        initial_determinant_failures=sum(r['determinant']<.0011 for r in quality['cells']),
        initial_face_weight_failures=sum(r['weight']<.055 for r in quality['faces']),
        initial_skew_failures=sum(r['skewness']>3.63636 for r in quality['faces']),
        initial_ratio_failures=sum(r['symmetric_volume_ratio']<.01 for r in quality['faces']),
        initial_positive_volume_failures=sum(r['volume']<=0 for r in quality['cells']),
        initial_exact_failures=sum(r['kind']=='exact_cell' for r in quality['failures']),
        minimum_initial_determinant=min(r['determinant'] for r in quality['cells']),
        minimum_initial_weight=min(r['weight'] for r in quality['faces']),
        source_admitted=initial['sdf']['admitted'],
        trials=len(trials),provisional_local_updates=sum(r['passed'] for r in history),
        best_provisional=last_accepted,
        history_identity=identity(history),first_layer_numeric_identity=identity(dict(quality=quality,sdf=initial['sdf'])),
        performance=json.loads((folder/'PERFORMANCE.json').read_text()),
        committed_shell=False,committed_remesh_events=0,core_attempted=False,complete_mesh=False,
        restart_between_events='NotReached: no committed first layer',
        numerical='NotEvaluatedDueToMissingAuthoritativeCaseControls')


def run():
    rows=[]
    for spec in authority.cohort():
        first=case_result(spec,'D');second=case_result(spec,'E')
        comparison={key:first[key]==second[key] for key in
                    ('history_identity','first_layer_numeric_identity','reason','trials','provisional_local_updates')}
        if not all(comparison.values()):raise ValueError('RejectedSearchReplayMismatch:'+spec['name'])
        first['replay']=comparison
        first['replay_performance']=second['performance']
        first['wall']=json.loads((OUT/spec['name']/'WALL_AUTHORITY.json').read_text())
        x,tri=geometry.build(spec);fronts=[geometry.front(spec,g.q.load_schedule().targets[0])]
        assert g.q.array_hash(x)==first['wall']['coordinates']
        assert g.q.array_hash(tri)==first['wall']['topology']
        first['wall_authority_reconstructed_exact']=True
        p,c,o=g.s.construct(x,tri,fronts,[1]*len(x));full=incidence.check(p,c,o)
        first['prospective_cap_incidence']=dict(passed=full['passed'],minimum_determinant=min(r['determinant'] for r in full['cells']),failures=len(full['failures']),
            scope='Diagnostic with future/core cap internal; does not replace the frozen shell-local gate')
        samples=[json.loads(line) for line in (OUT/spec['name']/'D/COLD_MEMORY.jsonl').read_text().splitlines()]
        peak=max(samples,key=lambda r:r['rss_bytes']);steady=[r['rss_bytes'] for r in samples if r['event']=='sample' and r['phase']==['process','initialize']]
        first['memory']=dict(sampled_peak=peak,retained_search_min=min(steady) if steady else None,retained_search_max=max(steady) if steady else None,
                             post_event='NotReached',sampling_interval_seconds=.5)
        rows.append(first)
        g.save(OUT/'CASE_RESULTS.json',rows)
    g.save(OUT/'REPLAY.json',dict(passed=True,scope='Exact rejected first-layer scientific histories; no completed-shell replay claimed',cases=[r['case'] for r in rows]))


if __name__=='__main__':run()
