"""Replay exact cap proposals; measure bounded cold and warm proof work.

The 180-second cold profile is the existing per-candidate operational budget,
not a scientific acceptance limit. Its measured prefix defines replay work.
No pair-count cap is raised to force completion; incomplete scopes cannot enter
private steering. Baseline cap proofs are not computed when identity analysis
proves none can transfer to the candidate.
"""
from task3_repair_study import *
from task3d_preflight import OUT
from sdf_cfd_mesher_v4.certificate.incremental import build_scope,ProofCache,certify,private_admission
from task3d_decompose import packed_pairs
from task3d_equivalence import store_stream
from task3c_io import read as read_evidence
import io

PROFILE_SECONDS=180

def stable(record):
    # Why scheduling yielded is operational. Complete/prefix coverage stays.
    return {k:v for k,v in record.items() if k!='reason'}

def run(replay=False):
    started=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,full,*_=ctx
    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);sealed=json.loads((ROOT/'results/task3c/CERTIFICATION.json').read_text())
    population=json.loads((OUT/'PAIR_DECOMPOSITION.json').read_text());results=[];timings=[]
    for group,old in enumerate(sealed['proposals']):
        st=time.perf_counter();ids=np.asarray(old['changed_vertices']);trial=np.asarray(old['layer1_coordinates'])
        assert scientific_hash(dict(ids=ids,coordinates=trial))==old['proposal']
        future=fc.forecast(ids,trial);changed_edges=dependencies(top,ids)[1];index=updated_index(full,future,changed_edges)
        if group==1:assert scientific_hash(pair_evidence(front,top,future))==scientific_hash(old['exact_known_pair'])
        # Direct identity with the saved scientific future, no cold whole-front run.
        prior=read_evidence(ROOT/f'results/task3c/PROPOSAL_{group}_DIRECT.json')
        assert np.array_equal(future.local['coordinates'],np.asarray(prior['future_changed_coordinates']))
        assert np.array_equal(future.local['normal'],np.asarray(prior['future_changed_normals']))
        del prior
        scope=build_scope(index,changed_edges);cache=ProofCache();setup_start=time.perf_counter();pairs,_,_=cache.pairs(scope)
        scope_setup_seconds=time.perf_counter()-setup_start
        assert len(pairs)==population[group]['candidate_pairs']
        assert array_hash(np.sort(packed_pairs(scope,pairs,len(top.edges))))==population[group]['candidate_pair_hash']
        del pairs
        assert population[group]['inherited_identical_geometry_pairs']==0
        previous=json.loads((OUT/f'PROPOSAL_{group}.json').read_text()) if replay else None
        limit=previous['certificate']['completed_pairs'] if replay else None
        stream=io.BytesIO()
        def progress(v):print('PROOF',group,json.dumps(v),flush=True)
        certificate,cold=certify(scope,cache=cache,record_sink=stream.write,progress=progress,
            seconds_budget=3600 if replay else PROFILE_SECONDS,max_completed_pairs=limit)
        n=certificate['completed_pairs'];warm_stream=io.BytesIO()
        warm,wperf=certify(scope,cache=cache,record_sink=warm_stream.write,seconds_budget=3600,max_completed_pairs=n)
        assert stable(certificate)==stable(warm) and stream.getvalue()==warm_stream.getvalue()
        assert wperf['cache_misses']==0,'ProfileCacheCapacityInsufficient'
        decision=private_admission(immediate_numeric=old['candidate_gate']['passed'],immediate_exact=old['exact_jacobians']['passed'],
            current_certificate_complete=old['current_layer_valid'],future_certificate=certificate,
            new_earlier_failures=old['new_earlier_numeric_failures'],target_maintained_or_improved=True)
        admission=decision['admissible']
        result=dict(proposal=group,members=old['members'],proposal_coordinates_hash=old['proposal'],
            changed_vertices=ids,layer1_coordinates=trial,forecast_coordinates_hash=array_hash(future.local['coordinates']),
            prior_current_certificate_sha256=file_hash(ROOT/f'results/task3c/CERTIFICATE_{group}.json'),
            immediate_hard_gates=old['candidate_gate'],immediate_exact_jacobians=old['exact_jacobians'],immediate_current_certificate=old['current_certificate'],
            certificate=stable(certificate),baseline_pairs=population[group]['baseline_pairs'],
            baseline_pair_proofs_constructed=0,baseline_skip_reason='All candidate pairs contain changed exact geometry; zero baseline proof identities can transfer.',
            inherited_baseline_proofs=0,invalidated_baseline_proofs=0,
            invalidated_potential_baseline_pair_bindings=population[group]['baseline_pairs'],
            current_passed=old['current_layer_valid'],admitted_to_private_state=admission,production_committed_layer=0,
            known_pair=old['exact_known_pair'],target_trajectories=old['targets'],new_earlier_numeric_failures=old['new_earlier_numeric_failures'],
            cache=dict(cold_hits=cold['cache_hits'],cold_misses=cold['cache_misses'],warm_hits=wperf['cache_hits'],warm_misses=wperf['cache_misses'],unchanged_prefix_records_reused=n),
            full_future_scope_complete=certificate['complete'],full_future_scope_not_examined=certificate['candidate_pairs']-n)
        h=scientific_hash(result);store_stream(f'PROPOSAL_{group}_PROOFS.jsonl.gz',stream.getvalue(),replay)
        if replay:assert scientific_hash(previous)==h
        else:(OUT/f'PROPOSAL_{group}.json').write_text(json.dumps(plain(result),indent=2,sort_keys=True)+'\n')
        results.append(dict(proposal=group,scientific_hash=h,admitted=admission))
        timings.append(dict(proposal=group,cold=cold,warm=wperf,scope_setup_seconds=scope_setup_seconds,total_seconds=time.perf_counter()-st,peak_rss=psutil.Process().memory_info().peak_wset,
            persisted_proof_bytes=(OUT/f'PROPOSAL_{group}_PROOFS.jsonl.gz').stat().st_size))
        (OUT/('PROPOSAL_REPLAY_TIMINGS.json' if replay else 'PROPOSAL_TIMINGS.json')).write_text(json.dumps(dict(stages=timings,total_seconds=time.perf_counter()-started),indent=2,sort_keys=True)+'\n')
        print('DONE',group,n,certificate['candidate_pairs'],admission,flush=True)
    (OUT/('PROPOSALS_REPLAY.json' if replay else 'PROPOSALS_HASH.json')).write_text(json.dumps(results,indent=2,sort_keys=True)+'\n')

if __name__=='__main__':run('--replay' in sys.argv)
