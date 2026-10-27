"""Compact observations from completed stages, including a timed partial run."""
from task4e_common import *

def optional(p,default=None):return read(p) if p.exists() else default
def outcome_scope(outcome):return outcome['scope'] if 'scope' in outcome else outcome.get('patch',{})
def layer_operations(operations,layer):return [r for r in operations if r['layer']==layer and not r['stage'].startswith('final_')]

def collect():
    operations=optional(OUT/'OPERATIONS.json',[]);rows=[];regions=[];totals={}
    def add(key,value):totals[key]=totals.get(key,0.)+float(value or 0)
    for layer in range(1,21):
        cp=RUN/f'build_A/LAYER_{layer:02}.json';pointer=optional(RUN/'build_A/CURRENT.json',{'layer':0})
        committed=layer<=pointer['layer']
        layer_ops=layer_operations(operations,layer)
        if not layer_ops and not committed:continue
        prepare=optional(RUN/f'LAYER{layer}_PREPARE_TIMINGS.json',{})
        if layer==1:prepare=optional(RUN/'CORRECTION_TIMINGS.json',{})
        cert=optional(RUN/f'LAYER_{layer:02}_TIMINGS.json',{})
        planning=optional(RUN/f'LAYER{layer}_PLANNING.json',{})
        atlas=optional(RUN/f'LAYER{layer}_ATLAS.json',{})
        atlas_file=RUN/f'LAYER{layer}_ATLAS.npz';atlas_counts={};atlas_hash=None
        if atlas_file.exists():
            with np.load(atlas_file) as z:
                atlas_counts={str(k):int(np.count_nonzero(z['states']==k)) for k in range(4)}
                atlas_hash=str(z['scientific_hash'])
        history=optional(RUN/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'),{})
        active=set();columns=set()
        for outcome in history.get('outcomes',[]):
            scope=outcome_scope(outcome);active.update(scope.get('active',[]));columns.update(scope.get('detailed',{}).get('columns',[]))
        detailed=0.;refreshes=0;hits=0;work=dict(columns=0,prisms=0,measured_prisms=0,faces=0)
        for profile in prepare.get('region_profiles',[]):
            detailed+=profile['baseline_detailed_seconds'];hit=profile.get('baseline_cache_hit',False)
            hits+=int(hit);refreshes+=int(not hit)
            if not hit:
                for k in work:work[k]+=profile.get('baseline_counts',{}).get(k,0)
            regions.append(dict(layer=layer,**profile))
        for r in prepare.get('stages',[]):
            detailed+=r.get('detailed_seconds',0)
            if 'detailed_cache_hit' in r:
                hit=r['detailed_cache_hit'];hits+=int(hit);refreshes+=int(not hit)
                if not hit:
                    for k in work:work[k]+=r.get('detailed_counts',{}).get(k,0)
            add('active_sdf_projection_seconds',r.get('proposal_projection_seconds',0)+r.get('forecast',{}).get('projection_seconds',0))
            add('local_forecast_seconds',r.get('forecast',{}).get('seconds',0))
            add('future_spot_checks_seconds',r.get('spot_seconds',0))
        gate=optional(RUN/f'LAYER_{layer:02}_NUMERIC.json',{})
        ledger=optional(RUN/f'LAYER_{layer:02}_CERTIFICATE.json',{})
        forecast_work={k:sum(int(r.get('cache',{}).get(k,0)) for r in history.get('history',[])) for k in
            ('recomputed_entries','unchanged_entries','invalidated_vertices','invalidated_edges','invalidated_triangles')}
        row=dict(layer=layer,committed=committed,prisms=106302 if committed else 0,corrected=any(r.get('accepted',r.get('corrected',False)) for r in history.get('outcomes',[])),
            active_vertices=len(active),active_fraction=len(active)/53153,dependency_columns=len(columns),post_step_atlas_states=atlas_counts,atlas_hash=atlas_hash,
            atlas_active_fraction=atlas.get('partition',{}).get('active_fraction'),atlas_active_columns_fraction=atlas.get('partition',{}).get('active_column_fraction'),
            atlas_invalidation=atlas.get('invalidation'),
            preparation_seconds=sum(r['seconds'] for r in layer_ops if r['stage'] in ('initial_correction','initial_warning_atlas','prepare')),
            active_search_seconds=prepare.get('local_search_seconds',sum(r['seconds'] for r in prepare.get('stages',[]))),
            detailed_seconds=detailed if layer>1 else None,detailed_refreshes=refreshes if layer>1 else None,detailed_hits=hits if layer>1 else None,detailed_work=work if layer>1 else None,
            risk_atlas_seconds=prepare.get('risk_discovery_seconds'),projection_seconds=prepare.get('nominal_projection_seconds'),
            certificate_seconds=cert.get('certificate_wall_seconds'),exact_jacobian_seconds=cert.get('exact_sign_seconds'),
            whole_layer_metric_seconds=sum(prepare.get(k,0) for k in ('nominal_numeric_seconds','final_numeric_seconds'))+cert.get('numeric_seconds',0),
            commit_seconds=cert.get('checkpoint_seconds'),planning_checkpoint_seconds=prepare.get('persistence_seconds'),
            layer_elapsed_seconds=sum(r['seconds'] for r in layer_ops),cumulative_task_seconds=max((r['elapsed_task'] for r in layer_ops),default=None),
            trials=len(history.get('history',[])),accepted=sum(bool(r.get('accepted')) for r in history.get('history',[])),
            rejected=sum(not bool(r.get('accepted')) for r in history.get('history',[])),gate=gate.get('gate'),certificate_pairs=ledger.get('candidate_pairs'),
            exact_calls=ledger.get('exact_calls'),forbidden=len(ledger.get('forbidden',[])),unresolved=len(ledger.get('unresolved',[])),certificate_complete=ledger.get('complete'),
            planner=planning.get('statistics'),forecast_work=forecast_work,checkpoint_sha256=file_hash(cp) if committed else None)
        rows.append(row)
        for key in ('active_search_seconds','risk_atlas_seconds','certificate_seconds','exact_jacobian_seconds','whole_layer_metric_seconds','commit_seconds','planning_checkpoint_seconds'):add(key,row[key])
        if layer>1:add('active_detailed_seconds',detailed)
    initial=optional(RUN/'INITIAL_TIMINGS.json',{});final=optional(RUN/'FINAL_VALIDATION_TIMINGS.json',{})
    totals.update(initial_forecast_seconds=initial.get('forecast_seconds'),initial_projection_seconds=initial.get('projection_seconds'),initial_forecast_and_scan_seconds=initial.get('total_seconds'),
        initial_warning_atlas_seconds=optional(RUN/'WARNING_INIT_TIMINGS.json',{}).get('seconds'),
        final_validation_seconds=final.get('total_seconds'),final_numeric_seconds=sum(x.get('numeric_seconds',0) for x in final.get('stages',[])),
        final_certificate_completeness_seconds=sum(x.get('certificate_seconds',0) for x in final.get('stages',[])),final_sdf_seconds=sum(x.get('sdf_seconds',0) for x in final.get('stages',[])),
        final_comparison_seconds=optional(OUT/'FINAL_COMPARISON_TIMING.json',{}).get('seconds'))
    result=dict(rows=rows,stages=totals,completed_layers=sum(r['committed'] for r in rows),completed_prisms=sum(r['prisms'] for r in rows),
        overlap_note='Active search includes detailed metrics, local forecast, projection and future spot checks. Do not sum nested stage times. First-layer internal metric timing was not separately recorded.',
        regions=regions,detailed_refreshes=sum(r['detailed_refreshes'] or 0 for r in rows),detailed_hits=sum(r['detailed_hits'] or 0 for r in rows))
    save('LAYER_PROGRESS.json',result);return result

if __name__=='__main__':
    r=collect();print('COLLECTED',r['completed_layers'],flush=True)
