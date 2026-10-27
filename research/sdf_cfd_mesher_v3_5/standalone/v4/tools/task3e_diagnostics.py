"""Read-only accepted-history diagnostics; no acceptance or new geometry."""
from task3e_study import *
from sdf_cfd_mesher_v4.certificate.incremental import build_scope

def run(replay=False,refined=False):
    started=time.perf_counter();front,top,sdf,schedule,coords,normals,*_=load()
    rows=[];cache={};timings=[];rejections=[]
    for group in ((3,) if refined else (0,1,2)):
        if not (OUT/f'BRANCH_{group}.json').exists():continue
        branch=read_evidence(OUT/f'BRANCH_{group}.json');ids=np.asarray(branch['patch']['active'])
        ring=set(branch['patch']['verification_ring']);affected=set(branch['patch']['affected_columns'])
        for h in branch['history']:
            if h['accepted']:continue
            ev=read_evidence(OUT/f'BRANCH_{group}_LAYER_{h["layer"]}_TRIAL_{h["trial"]}.json')
            side_owners={int(e):(int(a),int(b)) for kind,k,e,a,b in ev.get('immediate_face_ids',[]) if kind=='side'}
            changed=[]
            for entity,change in ev.get('new_earlier_failures',{}).items():
                kind,number,metric=entity.split(':');number=int(number)
                owners=side_owners.get(number,()) if kind=='side' else (number,)
                changed.append(dict(entity=entity,**change,owners=owners,verification_ring=bool(set(owners)&ring),affected_core=bool(set(owners)&affected)))
            rejections.append(dict(branch=group,layer=h['layer'],trial=h['trial'],mode=h['mode'],factor=h['factor'],
                reasons=h['reasons'],immediate_violations=ev.get('immediate_gate',{}).get('violations'),
                baseline_targets=ev.get('baseline_targets'),candidate_targets=ev.get('targets'),
                next_future_failure=ev.get('next_future_failure'),new_entity_violations=changed,
                exact_certificate_not_run=not bool(ev.get('immediate_certificate'))))
        for step in branch['accepted']:
            layer=step['layer'];beg=time.perf_counter()
            ev=read_evidence(OUT/f'BRANCH_{group}_LAYER_{layer}_TRIAL_{step["trial"]}.json')
            data=read_evidence(OUT/f'BRANCH_{group}_LAYER_{layer}_ACCEPTED.json')
            values=np.asarray(data['private_coordinates']);key=scientific_hash(dict(ids=ids,values=values))
            future=SimpleNamespace(vertex={'coordinates':CoordinateOverlay(coords,ids,values)})
            reused=key in cache
            if not reused:
                index=SideIndex(front,future,top);edges=dependencies(top,ids)[1]
                scope=build_scope(index,edges);pairs,_=scope.pairs()
                layers=np.asarray([k for k,e in scope.sides]);future_pairs=pairs[(layers[pairs[:,0]]>layer)|(layers[pairs[:,1]]>layer)]
                # Store all-layer enumeration identity so rebased identical
                # geometry can reuse the screening, not its old layer filter.
                cache[key]=dict(pairs=pairs,layers=layers,sides=scope.sides,edges=len(scope.edges),surfaces=len(scope.sides),pair_hash=array_hash(pairs))
            entry=cache[key];pairs=entry['pairs'];ls=entry['layers'];mask=(ls[pairs[:,0]]>layer)|(ls[pairs[:,1]]>layer)
            q=sdf.evaluate(np.asarray(ev['proposed_coordinates']))
            residual=q.phi-schedule.targets[layer-1]
            assert np.all(abs(residual)<=q.source_error_bound)
            low=front.coordinates[ids] if layer==1 else values[layer-2]
            actual_step=np.linalg.norm(values[layer-1]-low,axis=1)
            def numbers(a):return np.asarray([[float(v['nonfinite']) if isinstance(v,dict) else v for v in row] for row in a])
            sigma=numbers(data['future_cells']['sigma']);gain=numbers(data['future_cells']['G_bound'])
            available=np.asarray(data['future_cells']['G_bound_available'],bool)
            exact=data['exact_target_jacobians'];first={}
            for s in branch['members']:
                relevant=[r for r in exact if r['component']==s and r['layer']>layer]
                if relevant:first[s]=min((r['layer'] for r in relevant if r['jacobian_sign']<=0),default=None)
            local_cache=RollingForecastCache(front,top,schedule,sdf,coords,normals)
            base=PrivateState(frozen(ids,'<i8'),tuple(frozen(x,'<f8') for x in values[:layer-1]))
            cold=local_cache.forecast(base,values[layer-1]);cold_stats=dict(local_cache.last_stats)
            warm=local_cache.forecast(base,values[layer-1]);warm_stats=dict(local_cache.last_stats)
            assert cold is warm and np.array_equal(warm.local['coordinates'],values)
            rows.append(dict(branch=group,private_layer=layer,future_geometry_hash=key,
                conservative_future_candidate_pairs=int(mask.sum()),all_layer_broad_pairs=len(pairs),pair_hash=entry['pair_hash'],
                interaction_edges=entry['edges'],interaction_surfaces=entry['surfaces'],screening_reused=reused,
                exact_future_pairs=step['known_family']['exact_pairs'] if step['known_family'] else 0,
                future_screening_is_not_separation_proof=True,
                local_warm_cache_hits=local_cache.hits,local_warm_cache_misses=local_cache.misses,local_warm_exact_equal=True,
                maximum_sdf_residual=float(abs(residual).max()),projection_within_frozen_source_error=True,
                actual_step_min=float(actual_step.min()),actual_step_max=float(actual_step.max()),
                all_twenty_layer_sigma_min=float(np.nanmin(sigma)),all_twenty_layer_G_bound_max=float(np.nanmax(gain[available])),
                unavailable_gain_cells=int((~available).sum()),
                private_coordinate_hash=array_hash(values[:layer]),
                gradient_norm_min=float(q.gradient_norm.min()),gradient_norm_max=float(q.gradient_norm.max()),
                exact_first_future_nonpositive_jacobian=first,
                future_sigma_min=step['future_sigma_min'],future_G_bound_max=step['future_G_bound_max']))
            timings.append(dict(branch=group,layer=layer,seconds=time.perf_counter()-beg,local_cold=cold_stats,local_warm=warm_stats,peak_rss=psutil.Process().memory_info().peak_wset))
            print('DIAGNOSTIC',group,layer,rows[-1]['conservative_future_candidate_pairs'],reused,flush=True)
    suffix='_REFINED' if refined else ''
    save_or_compare('DIAGNOSTICS'+suffix+'.json',rows,replay)
    save_or_compare('COLLATERAL_REJECTIONS'+suffix+'.json',rejections,replay)
    dump('DIAGNOSTICS'+suffix+('_REPLAY_TIMINGS.json' if replay else '_TIMINGS.json'),dict(stages=timings,total_seconds=time.perf_counter()-started))
    dump('DIAGNOSTICS'+suffix+('_REPLAY.json' if replay else '_HASH.json'),dict(scientific_hash=scientific_hash(dict(diagnostics=rows,rejections=rejections))))

if __name__=='__main__':run('--replay' in sys.argv,'--refined' in sys.argv)
