from task4h_spots import *
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner

def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));planner=PersistentPlanner();rows=[];history=[];last=None;checkpoints=[]
    expected_hashes=read(OUT/'SPOT_REPLAY_1.json')['history'];tmp=OUT/'PLANNER_ROUNDTRIP.npz'
    for layer,trial,before,after,patch,known,expected in cohort(front,top):
        require_time()
        if last is not None and layer!=last:
            planner.layer=last;t=time.perf_counter();planner.save(tmp,'preserved-replay');size=tmp.stat().st_size
            new=PersistentPlanner();new.load(tmp,last,'preserved-replay');checkpoint_seconds=time.perf_counter()-t
            checkpoints.append(dict(layer=last,seconds=checkpoint_seconds,bytes=size,proofs=len(new.spots.proofs.cache),selection_entries=len(new.spots.selections)));planner=new
        last=layer;stats=dict(planner.spots.proofs.stats);selection=planner.spots.stats['seconds'];t=time.perf_counter()
        result=spots.compare_spots(front,top,before,after,patch,layer,known,planner=planner.spots);seconds=time.perf_counter()-t
        pairs=[tuple(tuple(v) for v in r['sides']) for r in expected['records']]
        assert [r['sides'] for r in result['records']]==pairs
        for new,old in zip(result['records'],expected['records']):
            for name in ('before','after'):assert new[name]['classification']==old[name]['classification']
        for key in ('broad_before','broad_after','first_forbidden_before','first_forbidden_after','new_earlier_pair_families','warnings','complete_future_certificate'):
            assert plain(result[key])==expected[key],(layer,trial,key)
        digest=scientific_hash(dict(layer=layer,trial=trial,pairs=pairs,broad_before=result['broad_before'],broad_after=result['broad_after'],results=result['records']))
        history.append(digest);assert digest==expected_hashes[len(history)-1]
        rows.append(dict(layer=layer,trial=trial,seconds=seconds,selection_seconds=planner.spots.stats['seconds']-selection,counts={k:planner.spots.proofs.stats[k]-stats.get(k,0) for k in planner.spots.proofs.stats},decision_evidence_equal=True,scientific_hash=digest))
        save('PRODUCTION_REPLAY.json',dict(rows=rows,checkpoints=checkpoints,scientific_hash=scientific_hash(history),total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset))
        print('PRODUCTION_REPLAY',layer,trial,seconds,flush=True)
    assert scientific_hash(history)==read(OUT/'SPOT_REPLAY_1.json')['scientific_hash']
if __name__=='__main__':run()
