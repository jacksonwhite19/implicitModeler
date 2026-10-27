"""Bounded preserved-proposal replay; never executes a builder or commits."""
from task4f_common import *
from task4f_instrument import Timers,install,same_arrays,dependency_counts
import task4_prepare_next as prod
from task4f_preserved import rejection
import psutil,cProfile,pstats,tracemalloc
from sdf_cfd_mesher_v4.forecast.detailed.patch import prisms,patch_key

def previous_low(front,prefix):
    return front.coordinates if len(prefix)==1 else prefix[-2]

def quick_numeric(front,top,forecast,patch,layer,met):
    """Study-only immediate slice. Identical qualified expressions; no new gate."""
    x,p=prisms(front,forecast,top,patch);nc=len(patch.columns);core=np.searchsorted(patch.columns,patch.core_columns)
    use=[layer-2,layer-1] if layer>1 else [0]
    g=met.cell_metrics(p[use].reshape(-1,6,3),np.repeat(np.array(use)+1,nc))
    g={key:value.reshape((len(use),nc)+value.shape[1:]) for key,value in g.items()}
    cq=met.legacy_modules()['component_quality'];fg=cq.FG;k=layer-1
    ed=top.edges[patch.side_edges];local=np.searchsorted(patch.vertices,ed)
    vs=np.stack((x[k,local[:,0]],x[k,local[:,1]],x[k+1,local[:,1]],x[k+1,local[:,0]]),axis=1)
    own=np.array([top.edge_triangles.row(e) for e in patch.side_edges]);ix=np.searchsorted(patch.columns,own)
    S,Cf=fg.face_area_centre_quad(vs)
    vals=cq.face_metrics(vs,S,Cf,g['centre'][-1,ix[:,0]],g['centre'][-1,ix[:,1]],g['volume'][-1,ix[:,0]],g['volume'][-1,ix[:,1]])
    faces=[('side',layer,int(e),int(a),int(b)) for e,(a,b) in zip(patch.side_edges,own)]
    if layer>1:
        f=np.searchsorted(patch.vertices,top.triangles[patch.core_columns]);vs=x[k,f];S,Cf=fg.face_area_centre_tri(vs)
        cap=cq.face_metrics(vs,S,Cf,g['centre'][0,core],g['centre'][-1,core],g['volume'][0,core],g['volume'][-1,core])
        vals=[np.concatenate((a,b)) for a,b in zip(vals,cap)];faces += [('cap',k,int(c),int(c),int(c)) for c in patch.core_columns]
    cells={key:np.zeros((20,len(core))+v.shape[2:],dtype=v.dtype) for key,v in g.items()}
    for key in cells:cells[key][k]=g[key][-1,core]
    result=dict(cells=cells,face_ids=faces,faces=dict(zip(('weight','skewness','symmetric_volume_ratio','nonorthogonality'),vals)))
    return prod.immediate_numeric(result,layer)

def run(layers=(3,6,11),account_only=False):
    start=time.perf_counter();setup=[]
    def setup_call(name,fn):
        t=time.perf_counter();v=fn();setup.append(dict(stage=name,seconds=time.perf_counter()-t));return v
    front=setup_call('wall_load',prod.load_sf2_wall);top=setup_call('topology_build',lambda:prod.build_topology(front.triangles,len(front.coordinates)))
    sdf=setup_call('native_sdf_setup',prod.NativeSdf);schedule=setup_call('preserved_schedule',prod.load_schedule)
    timers=Timers();original,met=install(timers);timers.enabled=False
    rows=read(OUT/'BOUNDED_OPERATION_COUNTS.json') if account_only else []
    samples=read(OUT/'BOUNDED_SAMPLES.json') if account_only else []
    experiments=read(OUT/'INCREMENTAL_EXPERIMENT.json') if account_only else []
    keys=read(OUT/'CACHE_KEY_REPLAY.json') if account_only else [];keys_seen={r['key']:(r['layer'],r['trial']) for r in keys}
    data_moves=read(OUT/'DATA_MOVEMENT.json') if account_only else []
    for layer in layers:
        require_time();t=time.perf_counter()
        _,coords,normals=prod.load_candidate(BASE/f'LAYER{layer-1}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        prefix=tuple(np.load(BASE/'build_A'/f'FRONT_{k:02}.npy') for k in range(1,layer));candidate=coords[layer-1].copy()
        setup.append(dict(stage='preserved_load',layer=layer,seconds=time.perf_counter()-t))
        current=prod.SimpleNamespace(coordinates=prefix[-1]);previous=prod.evaluate_cells(top,previous_low(front,prefix),prefix[-1],layer-1)
        nominal=prod.evaluate_layer(top,prefix[-1],candidate,layer,previous);events=read(BASE/f'LAYER{layer-1}_WARNINGS.json')
        retained=[np.array(o['patch']['active']) for o in read(BASE/'CORRECTION_HISTORY.json')['outcomes'] if o['corrected']]
        for k in range(2,layer):retained += [np.array(o['scope']['active']) for o in read(BASE/f'LAYER{k}_CORRECTIONS.json') if o.get('accepted')]
        specs,_=prod.select_supports(prefix[-1],top,candidate,nominal['events'],events,layer,retained)
        hist=read(BASE/f'LAYER{layer}_SEARCH.json');out={(o['patch'],o['rings']):o for o in hist['outcomes'] if 'scope' in o}
        selected=set();covered=set()
        for i,h in enumerate(hist['history']):
            scope=out[h['patch'],h['rings']]['scope'];category=(len(scope['detailed']['columns'])>100,rejection(h))
            if category not in covered:covered.add(category);selected.add(i)
        if account_only:selected=set()
        last=None;baseline_result=None;group_original=None
        for i,h in enumerate(hist['history']):
            require_time();group=(h['patch'],h['rings'])
            if group!=last:
                spec=specs[h['patch']];p=prod.bounded_patch(top,prod.expand_vertices(top,spec['vertices'],h['rings']))
                assert p.record()==out[group]['scope'],'PreservedScopeMismatch'
                fc=prod.RollingForecastCache(front,top,schedule,sdf,coords,normals);state=prod.PrivateState(p.active,tuple(x[p.active] for x in prefix));nominal_local=candidate[p.active].copy()
                modes=prod.correction_fields(current,top,p,spec['field']);modes.extend(prod.search_fields(current,top,p,candidate,schedule.targets[-1])[0]);modes=dict(modes)
                baseline=prod.SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords});old_x,old_p=prisms(front,baseline,top,p.detailed)
                baseline_result=None;last=group
                bkey=patch_key(front,baseline,top,p.detailed)
                keys.append(dict(layer=layer,trial=None,patch=h['patch'],key=bkey,repeated=bkey in keys_seen));keys_seen[bkey]=[layer,None]
            tick=time.perf_counter();proposed,_=prod.try_tangent_proposal(nominal_local,modes[h['mode']],np.full(len(p.active),h['factor']*schedule.increments[layer-1]),sdf,schedule.targets[layer-1])
            assert array_hash(proposed)==h['hash'],(layer,i,'CandidateIdentityMismatch')
            forecast=fc.forecast(state,proposed);x,prism=prisms(front,forecast,top,p.detailed)
            counts,changed=dependency_counts(top,p.detailed,old_x,x)
            key=patch_key(front,forecast,top,p.detailed);keys.append(dict(layer=layer,trial=i,key=key,repeated=key in keys_seen));keys_seen[key]=[layer,i]
            row=dict(layer=layer,trial=i,patch=h['patch'],rejection=rejection(h),accepted=h['accepted'],candidate_hash=h['hash'],
                moved_vertices=int(np.any(proposed.view(np.uint64)!=nominal_local.view(np.uint64),axis=1).sum()),active_vertices=len(p.active),
                maximum_displacement=float(np.linalg.norm(proposed-nominal_local,axis=1).max()),**counts,
                committed_prefix_cells_recomputed=(layer-1)*len(p.detailed.columns),geometry_reconstruction_seconds=time.perf_counter()-tick,
                cache=dict(fc.last_stats))
            core=np.searchsorted(p.detailed.columns,p.detailed.core_columns)
            row.update(exact_current_calls=len(h.get('exact',{}).get('records',[])),exact_current_unchanged_input_calls=int((~changed[layer-1,core]).sum()) if 'exact' in h else 0)
            rows.append(row)
            if i in selected:
                timers.enabled=False;t=time.perf_counter();full=met.measure(front,forecast,top,p.detailed);full_time=time.perf_counter()-t
                assert prod.immediate_numeric(full,layer)==h['gate'],(layer,i,'NumericGateMismatch')
                t=time.perf_counter();quick=quick_numeric(front,top,forecast,p.detailed,layer,met);quick_time=time.perf_counter()-t
                assert quick==h['gate'],(layer,i,'EarlyGateMismatch')
                timers.enabled=True;t=time.perf_counter();profiled=met.measure(front,forecast,top,p.detailed);profile_time=time.perf_counter()-t
                assert same_arrays(full,profiled),'InstrumentationChangedResult'
                sample=dict(layer=layer,trial=i,category=rejection(h),counts=full['counts'],plain_seconds=full_time,instrumented_seconds=profile_time,
                    early_immediate_seconds=quick_time,immediate_gate_identical=True,scientific_hash=scientific_hash(full),
                    measured_avoided_detail_seconds=full_time-quick_time if not quick['passed'] else 0.,decision_preserved_by_decisive_gate=not quick['passed'])
                samples.append(sample)
                # One metric-family experiment: full versus dependency-local cell metrics.
                if len(p.detailed.columns)>100 and not experiments:
                    timers.enabled=False;flat=prism.reshape(-1,6,3);oldflat=old_p.reshape(-1,6,3);ls=np.repeat(np.arange(1,21),len(p.detailed.columns));mask=changed.ravel()
                    oldg=original(oldflat,ls);t=time.perf_counter();allg=original(flat,ls);full_metric_time=time.perf_counter()-t
                    t=time.perf_counter();part=original(flat[mask],ls[mask]);merged={k:v.copy() for k,v in oldg.items()}
                    for k in merged:merged[k][mask]=part[k]
                    local_time=time.perf_counter()-t
                    assert same_arrays(allg,merged),'DependencyLocalMetricMismatch'
                    experiments.append(dict(layer=layer,trial=i,family='cell_metrics including sigma/G',changed=int(mask.sum()),reused=int((~mask).sum()),full_cells=len(mask),full_seconds=full_metric_time,local_seconds=local_time,exact_equality=True,hash=scientific_hash(allg)))
                    # Allocation observation is separate from wall timings.
                    tracemalloc.start();alloc=met.measure(front,forecast,top,p.detailed);current_bytes,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
                    data_moves.append(dict(layer=layer,trial=i,tracemalloc_peak_bytes=peak,retained_traced_bytes=current_bytes,materialized_local_front_bytes=x.nbytes,materialized_prism_bytes=prism.nbytes,output_array_bytes=sum(v.nbytes for v in alloc['cells'].values())+sum(v.nbytes for v in alloc['faces'].values()),full_shell_forecast_copy=False))
                    profile=cProfile.Profile();profile.enable();met.measure(front,forecast,top,p.detailed);profile.disable();stats=pstats.Stats(profile)
                    table=[dict(file=k[0].replace(str(ROOT),'V4'),line=k[1],function=k[2],primitive_calls=v[0],calls=v[1],self_seconds=v[2],total_seconds=v[3]) for k,v in stats.stats.items()]
                    save('GENERIC_PROFILE.json',sorted(table,key=lambda r:r['self_seconds'],reverse=True)[:80]);timers.enabled=True
                print('SAMPLE',layer,i,rejection(h),full_time,quick_time,flush=True)
                timers.enabled=False
            if h['accepted']:
                coords[:,p.active]=forecast.local['coordinates'];normals[:,p.active]=forecast.local['normal'];candidate[p.active]=proposed
        # Exact accepted front equality proves reconstruction followed historical transactions.
        assert array_hash(candidate)==read(BASE/f'LAYER{layer}_GATE.json')['coordinate_hash']
        save('BOUNDED_OPERATION_COUNTS.json',rows);save('BOUNDED_SAMPLES.json',samples);save('INCREMENTAL_EXPERIMENT.json',experiments)
        save('DATA_MOVEMENT.json',data_moves);save('CACHE_KEY_REPLAY.json',keys)
        if not account_only:save('CALL_TREE.json',timers.table());save('BOUNDED_SETUP.json',setup)
        print('ACCOUNTED',layer,len(rows),flush=True)
    save('ACCOUNT_ONLY_RUN.json' if account_only else 'BOUNDED_RUN.json',dict(seconds=time.perf_counter()-start,task_elapsed=elapsed(),peak_rss=psutil.Process().memory_info().peak_wset,layers=list(layers),rows=len(rows),samples=len(samples),no_production_mutation=True))

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--account-only',action='store_true');p.add_argument('--layers',default='3,6,11');a=p.parse_args()
    run(tuple(map(int,a.layers.split(','))),a.account_only)
