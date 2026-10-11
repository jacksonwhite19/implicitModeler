"""Count exact broad-phase work before further bounded certificate scheduling."""
from task3_repair_study import *
OUT=ROOT/'results/task3c'
if __name__=='__main__':
    start=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,full,one,_=ctx
    r=json.loads((OUT/'CERTIFICATE_0_CURRENT.json').read_text());ids=np.array(r['changed_vertices']);x=np.array(r['layer1_coordinates'])
    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=fc.forecast(ids,x);changed=dependencies(top,ids)[1]
    idx=updated_index(full,future,changed);edges=idx.candidates(changed,max_edges=8192);ep=top.edges[edges];q=[];labels=[];previous=front.coordinates
    for k,x in enumerate(future.vertex['coordinates'],1):
        q.extend(np.stack((previous[ep[:,0]],previous[ep[:,1]],x[ep[:,0]],x[ep[:,1]]),axis=1));labels.extend((k,int(e)) for e in edges);previous=x
    lo,hi=kernel.enumerator.boxes(np.asarray(q));pairs,_=kernel.enumerator.bvh_pairs(lo,hi);core=set(map(int,changed))
    selected=sorted((a,b) for a,b in pairs if labels[a][1] in core or labels[b][1] in core)
    layers=collections.Counter(max(labels[a][0],labels[b][0]) for a,b in selected)
    result=dict(candidate_edges=len(edges),changed_edges=len(changed),candidate_pairs=len(selected),pairs_by_latest_layer=dict(layers),
        pair_identity_hash=scientific_hash([[labels[a],labels[b]] for a,b in selected]))
    (OUT/'PAIR_POPULATION.json').write_text(json.dumps(plain(result),indent=2,sort_keys=True)+'\n')
    (OUT/'PAIR_POPULATION_TIMINGS.json').write_text(json.dumps(dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print(result,flush=True)
