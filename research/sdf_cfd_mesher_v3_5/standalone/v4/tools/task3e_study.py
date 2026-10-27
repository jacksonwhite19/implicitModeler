"""Bounded private cap steering; restored Task-1 cache, no production writes."""
from task3_repair_study import *
from task3e_preflight import OUT
from task3c_io import read as read_evidence, write as write_evidence
from sdf_cfd_mesher_v4.advancing_front.private_rolling import *
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import active_patch
from sdf_cfd_mesher_v4.certificate.incremental import ProofCache, certify, conservative_pairs
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import canonical_bytes
import io, gzip, hashlib

# Development work bounds, not scientific tolerances or convergence claims.
DEPTH=4
FACTORS=(.5,1.,.25,-.25,-.5)
FINE_FACTORS=(.5,1.,.25,.125,.0625,.03125,-.125,-.25,-.5)
CONTINUATION_FACTORS=(.125,.0625,.03125,-.125,-.0625)

def dump(name,value): (OUT/name).write_text(json.dumps(plain(value),sort_keys=True,indent=2)+'\n')
def stable_performance(p): return {k:v for k,v in p.items() if not ('seconds' in k)}

def save_or_compare(name,data,replay):
    h=scientific_hash(data)
    if replay: assert scientific_hash(read_evidence(OUT/name))==h,(name,'ScientificReplayMismatch')
    else: write_evidence(OUT/name,data)
    return h

def proof_stream(name,stream,replay):
    value=stream.getvalue()
    if replay:
        with gzip.open(OUT/name,'rb') as f: assert f.read()==value,name
    else:
        with (OUT/name).open('wb') as f:
            with gzip.GzipFile(filename='',mode='wb',fileobj=f,mtime=0) as z:z.write(value)

def exact_targets(front,top,future,specs):
    records=[]
    for spec in specs:
        if spec['metric']!='jacobian':continue
        for col in spec['columns']:
            ids=top.triangles[col];low=front.coordinates[ids]
            for k in range(1,21):
                high=future.vertex['coordinates'][k-1,ids]
                records.append(dict(component=spec['name'],column=col,layer=k,**exact_wedge(np.concatenate((low,high)))))
                low=high
    return records

def future_family(front,top,future):
    """Known two-edge family across all layers; conservative AABB plus exact pairs.

    The known 1/10 pair is checked even when its boxes no longer overlap.
    Other unrelated provisional future pairs are not claimed certified.
    """
    quads=[];labels=[];sides=[]
    for edge in (1,729):
        a,b=top.edges[edge];previous=front.coordinates[[a,b]]
        for k in range(1,21):
            x=future.vertex['coordinates'][k-1,[a,b]]
            quads.append(np.concatenate((previous,x)));labels.append(((k-1,int(a)),(k-1,int(b)),(k,int(a)),(k,int(b))))
            sides.append((k,edge));previous=x
    pairs,_=conservative_pairs(np.asarray(quads),np.ones(40,bool))
    selected={(int(a),int(b)) for a,b in pairs if sides[a][1]!=sides[b][1]};selected.add((0,29))
    session=CertificateSession();rows=[]
    for a,b in sorted(selected):
        r=session.pair(repaired_kernel.Patch.binary(quads[a],labels[a]),repaired_kernel.Patch.binary(quads[b],labels[b]))
        rows.append(dict(side_a=sides[a],side_b=sides[b],**r))
    return dict(possible_cross_edge_pairs=400,broad_phase_excluded=400-len({(int(a),int(b)) for a,b in pairs if sides[a][1]!=sides[b][1]}),
                exact_pairs=len(rows),records=rows,forbidden=[r for r in rows if r['classification']=='CertifiedForbiddenIntersection'],
                unresolved=[r for r in rows if r['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact','CertifiedForbiddenIntersection')],
                complete_global_future_certificate=False)

def trajectories(result,specs,after):
    return {s['name']:target_trajectory(result,s['columns'],s['metric'],after) for s in specs}

def branch(group,context,replay=False,depth=DEPTH):
    if not 1<=depth<=DEPTH:raise ValueError('Task3EStudyDepthBound: extending this study requires an explicit new work bound and final-stack proof at layer 20')
    started=time.perf_counter();front,top,sdf,schedule,coords,normals,*_=context
    source_group=0 if group in (0,2) else 1
    old=json.loads((ROOT/'results/task3c/CERTIFICATION.json').read_text())['proposals'][source_group]
    specs=[s for s in components() if s['name'] in old['members']]
    p=active_patch(top,np.asarray(old['patch']['active']));ids=p.active;changed=np.asarray(old['changed_vertices'])
    state=PrivateState(ids);fc=RollingForecastCache(front,top,schedule,sdf,coords,normals);dc=DetailedCache();pc=ProofCache()
    baseline=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords});base_result=dc.get(front,baseline,top,p.detailed)
    # A concrete unaffected cache entry, checked on every evaluated proposal.
    control=build_patch(top,[22890]).detailed;control_result=dc.get(front,baseline,top,control);control_hash=scientific_hash(control_result)
    saved=np.asarray(old['layer1_coordinates']);initial=coords[0,ids].copy();initial[np.searchsorted(ids,changed)]=saved
    assert scientific_hash(dict(ids=changed,coordinates=saved))==old['proposal']
    # Replay the source projection from its actual pre-projection values.
    if source_group==0:
        prior=read_evidence(ROOT/'results/task3c/component_00_closed_support.json');projection=next(e['projection'] for e in prior['history'] if e.get('potential'))
    else:
        prior=read_evidence(ROOT/'results/task3c/COMMON_NEGATIVE_CAP.json');projection=prior['projection']
    projected=predict_next_front(np.asarray(projection['pre_projection']),schedule.targets[0],sdf)
    assert np.array_equal(projected.coordinates,np.asarray(projection['projected']))
    del prior
    result=base_result;future=baseline;history=[];accepted=[];timings=[];hashes=[];max_displacement=0.
    previous_spot=future_family(front,top,baseline) if group==1 else None
    start_layer=1
    if group in (2,3):
        # Fork an already proved prefix. Do not repeat its exact work or the
        # preserved failed coarse trials when refining the amplitude poll.
        seed_group,seed_layer=(0,1) if group==2 else (1,2)
        seed_name=f'BRANCH_{seed_group}_LAYER_{seed_layer}_TRIAL_0.json'
        seed=read_evidence(OUT/seed_name)
        saved_prefix=read_evidence(OUT/f'BRANCH_{seed_group}_LAYER_{seed_layer}_ACCEPTED.json')
        prefix=np.asarray(saved_prefix['prefix']);assert np.array_equal(initial,prefix[0]) and seed['decision']['admissible']
        proof_hashes=[]
        for k in range(1,seed_layer+1):
            prior_step=read_evidence(OUT/f'BRANCH_{seed_group}_LAYER_{k}_TRIAL_0.json')
            proof_hashes.append(prior_step['immediate_certificate']['records_sha256'])
            with gzip.open(OUT/f'BRANCH_{seed_group}_LAYER_{k}_TRIAL_0_PROOFS.jsonl.gz','rb') as f:
                assert hashlib.sha256(f.read()).hexdigest()==proof_hashes[-1]
        state=PrivateState(ids,tuple(frozen(x,'<f8') for x in prefix[:-1]),tuple(proof_hashes[:-1]))
        future=fc.forecast(state,prefix[-1])
        assert array_hash(future.local['coordinates'])==seed['forecast_coordinates_hash']
        result=dc.get(front,future,top,p.detailed)
        assert scientific_hash(result)==seed['direct_metric_hash']
        state=state.accept(prefix[-1],seed['decision'],proof_hashes[-1]);assert state.sha256==saved_prefix['state_hash']
        hashes.append(dict(path=seed_name,sha256=scientific_hash(seed)))
        if group==3:previous_spot=seed['future_known_family']
        start_layer=seed_layer+1
    for layer in range(start_layer,depth+1):
        step_start=time.perf_counter();before_state=state.sha256;before_fail=future_failures(result,layer)
        before_target=trajectories(result,specs,layer)
        nominal=coords[0,ids].copy() if layer==1 else future.vertex['coordinates'][layer-1,ids].copy()
        # The positive branch first establishes a certified nominal prefix.
        # On the negative branch, poll steering before nominal maintenance.
        factors=FINE_FACTORS if group==2 else CONTINUATION_FACTORS if group==3 else FACTORS
        trials=[('saved_layer1',None)] if layer==1 else ([('outward_span_tangent',x) for x in factors]+[('nominal',0.)] if group in (1,2,3) else [('nominal',0.)]+[('outward_span_tangent',x) for x in factors])
        step_accepted=False
        for trial_index,(mode,factor) in enumerate(trials):
            trial_start=time.perf_counter();projection_data=None
            if layer==1: proposed=initial.copy();projection_data=projection
            elif factor==0: proposed=nominal.copy()
            else:
                field=np.zeros_like(nominal);mask=np.isin(ids,changed)
                field[mask,1]=np.sign(front.coordinates[ids[mask],1])*np.sign(factor)
                increment=schedule.targets[layer-1]-schedule.targets[layer-2]
                proposed,projection_data=try_tangent_proposal(nominal,field,np.full(len(ids),abs(factor)*increment),sdf,schedule.targets[layer-1])
                if proposed is None:
                    event=dict(layer=layer,trial=trial_index,mode=mode,factor=factor,state_before=before_state,projection=projection_data,
                        accepted=False,reasons=['ProjectionFailure'],production_committed_layer=0)
                    name=f'BRANCH_{group}_LAYER_{layer}_TRIAL_{trial_index}.json'
                    hashes.append(dict(path=name,sha256=save_or_compare(name,event,replay)));history.append(event)
                    assert state.sha256==before_state
                    continue
            prediction=fc.forecast(state,proposed);fperf=dict(fc.last_stats)
            if layer==1:
                sealed=json.loads((ROOT/f'results/task3d/PROPOSAL_{source_group}.json').read_text())
                assert array_hash(prediction.local['coordinates'][:,np.searchsorted(ids,changed)])==sealed['forecast_coordinates_hash']
            detail_start=time.perf_counter();r=dc.get(front,prediction,top,p.detailed);detail_seconds=time.perf_counter()-detail_start
            assert scientific_hash(dc.get(front,prediction,top,control))==control_hash
            gate=immediate_numeric(r,layer);ex=exact_layer(front,top,prediction,p.detailed,layer) if gate['passed'] else dict(passed=False,not_run='Numeric gate rejection')
            if layer==1:
                assert gate['violations']==old['candidate_gate']['violations'];assert gate['extrema']==old['candidate_gate']['extrema']
                assert [dict((k,v) for k,v in e.items() if k!='layer') for e in ex['records']]==old['exact_jacobians']['records']
            after_fail=future_failures(r,layer);earlier=no_new_earlier_failure(before_fail,after_fail)
            after_target=trajectories(r,specs,layer)
            maintained=all(maintained_target(before_target[s['name']],after_target[s['name']]) for s in specs)
            certificate={};cperf={};spot=None;scope_info=None
            if gate['passed'] and ex['passed'] and not earlier and maintained:
                print('CERTIFICATE_START',group,layer,trial_index,flush=True)
                scope=immediate_scope(front,top,prediction,p.detailed,layer);stream=io.BytesIO()
                if replay:
                    previous=read_evidence(OUT/f'BRANCH_{group}_LAYER_{layer}_TRIAL_{trial_index}.json')
                    with gzip.open(OUT/f'BRANCH_{group}_LAYER_{layer}_TRIAL_{trial_index}_PROOFS.jsonl.gz','rb') as f:
                        restore_exact_proofs(scope,previous['immediate_certificate'],f.read(),pc)
                certificate,cperf=certify(scope,cache=pc,seconds_budget=1800,record_sink=stream.write,
                    progress=lambda x:print('CERTIFICATE',group,layer,trial_index,json.dumps(x),flush=True))
                proof_stream(f'BRANCH_{group}_LAYER_{layer}_TRIAL_{trial_index}_PROOFS.jsonl.gz',stream,replay)
                scope_info=dict(edges=len(scope.edges),surfaces=len(scope.sides),new_core_sides=int(scope.core.sum()),
                    prior_layers_included=list(range(1,layer)),outside_source_edges_screened=len(top.edges),global_future_certified=False)
                if source_group==1:spot=future_family(front,top,prediction)
            decision=immediate_admission(gate,ex,certificate,earlier,maintained)
            if spot and spot['unresolved']:
                decision['admissible']=False;decision['reasons'].append('CriticalFutureFamilyUnresolved')
            if spot:
                first=lambda s:min((max(r['side_a'][0],r['side_b'][0]) for r in s['forbidden']),default=21)
                spot['baseline_first_intersection']=first(previous_spot)
                spot['candidate_first_intersection']=first(spot)
                if first(spot)<first(previous_spot):
                    decision['admissible']=False;decision['reasons'].append('EarlierCriticalFutureIntersection')
            # Known future intersection evidence remains a warning. An immediate
            # occurrence would already have failed the complete current proof.
            displacement=float(np.linalg.norm(proposed-nominal,axis=1).max())
            event=dict(layer=layer,trial=trial_index,mode=mode,factor=factor,state_before=before_state,
                proposed_coordinates=proposed,projection=projection_data,projection_phi=sdf.evaluate(proposed).phi,
                target=float(schedule.targets[layer-1]),displacement=displacement,immediate_gate=gate,exact_jacobians=ex,
                immediate_certificate=certificate,certificate_scope=scope_info,decision=decision,
                baseline_targets=before_target,targets=after_target,new_earlier_failures=earlier,
                next_future_failure=min(after_fail.values(),default=None),future_failure_map=after_fail,
                future_known_family=spot,forecast_coordinates_hash=array_hash(prediction.local['coordinates']),
                future_descriptor_hash=scientific_hash({k:prediction.local[k] for k in ('normal','remaining_vertex','edge','triangle')}),
                direct_metric_hash=scientific_hash(r),immediate_face_ids=immediate_subset(r,layer)['face_ids'],
                cache=stable_performance(fperf),unrelated_detailed_hit=True,production_committed_layer=0)
            name=f'BRANCH_{group}_LAYER_{layer}_TRIAL_{trial_index}.json'
            hashes.append(dict(path=name,sha256=save_or_compare(name,event,replay)))
            timings.append(dict(layer=layer,trial=trial_index,forecast=fperf,detailed_seconds=detail_seconds,certificate=cperf,
                total_seconds=time.perf_counter()-trial_start,peak_rss=psutil.Process().memory_info().peak_wset))
            history.append(dict(layer=layer,trial=trial_index,mode=mode,factor=factor,accepted=decision['admissible'],reasons=decision['reasons'],
                gate_violations=gate['violations'],earlier_failures=len(earlier),targets=after_target,displacement=displacement,
                certificate_pairs=certificate.get('candidate_pairs'),certificate_complete=certificate.get('complete')))
            print('TRIAL',group,layer,trial_index,decision,gate['violations'],flush=True)
            if decision['admissible']:
                state=state.accept(proposed,decision,certificate['records_sha256']);future=prediction;result=r
                if source_group==1:previous_spot=spot
                assert np.array_equal(np.stack(state.prefix),prediction.local['coordinates'][:layer])
                ex_targets=exact_targets(front,top,prediction,specs)
                accepted_data=dict(state_hash=state.sha256,private_layer=state.layer,prefix=np.stack(state.prefix),
                    immediate=immediate_subset(r,layer),future_cells=r['cells'],future_faces=r['faces'],face_ids=r['face_ids'],
                    exact_target_jacobians=ex_targets,local_descriptors={k:prediction.local[k] for k in ('vertices','edges','triangles','edge','triangle','remaining_vertex')},
                    private_coordinates=prediction.local['coordinates'],private_normals=prediction.local['normal'])
                accepted_name=f'BRANCH_{group}_LAYER_{layer}_ACCEPTED.json'
                hashes.append(dict(path=accepted_name,sha256=save_or_compare(accepted_name,accepted_data,replay)))
                accepted.append(dict(layer=layer,trial=trial_index,displacement=displacement,gate=gate,certificate=certificate,scope=scope_info,
                    targets=after_target,next_future_failure=event['next_future_failure'],known_family=spot,state_hash=state.sha256,
                    future_sigma_min=float(r['cells']['sigma'][layer:].min()) if layer<20 else None,
                    future_G_bound_max=float(np.nanmax(r['cells']['G_bound'][layer:])) if layer<20 else None))
                max_displacement=max(max_displacement,displacement);step_accepted=True;break
            assert state.sha256==before_state,'RollbackIdentity'
            # Bound detailed memo storage; keep current, nominal control and one trial.
            if len(dc.entries)>5:dc.entries={k:v for k,v in dc.entries.items() if v is result or v is control_result or v is r}
        dump(f'BRANCH_{group}'+('_REPLAY_TIMINGS.json' if replay else '_TIMINGS.json'),dict(stages=timings,total_seconds=time.perf_counter()-started))
        if not step_accepted:break
    final=dict(group=group,members=old['members'],patch=p.record(),tested_depth=depth,private_layers=state.layer,
        accepted=accepted,history=history,rejected=sum(not x['accepted'] for x in history),final_state_hash=state.sha256,
        max_displacement=max_displacement,production_committed_layer=0,scientific_artifacts=hashes,
        forecast_cache_hits=fc.hits,forecast_cache_misses=fc.misses,detailed_cache_hits=dc.hits,detailed_cache_misses=dc.misses,
        authoritative_wall_hash=array_hash(front.coordinates),topology_hash=top.sha256)
    save_or_compare(f'BRANCH_{group}.json',final,replay)
    dump(f'BRANCH_{group}'+('_REPLAY.json' if replay else '_HASH.json'),dict(scientific_hash=scientific_hash(final),private_layers=state.layer))
    print('BRANCH_DONE',group,state.layer,scientific_hash(final),flush=True)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--branch',type=int,required=True);parser.add_argument('--depth',type=int,default=DEPTH);parser.add_argument('--replay',action='store_true');args=parser.parse_args()
    branch(args.branch,load(),args.replay,args.depth)
