"""Measure pair geometry identity before proposing certificate acceleration."""
from task3_repair_study import *
from task3d_preflight import OUT
from sdf_cfd_mesher_v4.certificate.incremental import build_scope

def packed_pairs(scope,pairs,edge_count):
    labels=np.asarray(scope.sides,dtype=np.uint64);sid=labels[:,0]*edge_count+labels[:,1]
    a,b=sid[pairs[:,0]],sid[pairs[:,1]]
    return np.minimum(a,b)*(21*edge_count)+np.maximum(a,b)

def run(replay=False):
    started=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,full,*_=ctx
    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);reports=[];timings=[]
    sealed=json.loads((ROOT/'results/task3c/CERTIFICATION.json').read_text())
    for group,r in enumerate(sealed['proposals']):
        st=time.perf_counter();ids=np.asarray(r['changed_vertices']);trial=np.asarray(r['layer1_coordinates'])
        future=fc.forecast(ids,trial);edges=dependencies(top,ids)[1];idx=updated_index(full,future,edges)
        candidate=build_scope(idx,edges);baseline=build_scope(full,edges)
        def pairs(scope):
            if replay:return scope.pairs()[0]
            lo,hi=kernel.enumerator.boxes(scope.quads);p,_=kernel.enumerator.bvh_pairs(lo,hi)
            return np.asarray(sorted((a,b) for a,b in p if scope.core[a] or scope.core[b]),dtype=np.int32)
        cp=pairs(candidate);bp=pairs(baseline);assert len(cp)==r['future_certificate']['candidate_pairs']
        old={s:i for i,s in enumerate(baseline.sides)}
        same=np.array([s in old and np.array_equal(candidate.quads[i].view(np.uint64),baseline.quads[old[s]].view(np.uint64)) for i,s in enumerate(candidate.sides)])
        changed=~same;touch=changed[cp[:,0]].astype(int)+changed[cp[:,1]]
        cn=packed_pairs(candidate,cp,len(top.edges));bn=packed_pairs(baseline,bp,len(top.edges));inherited=np.isin(cn,bn)
        sideids=np.asarray(candidate.sides);layers=np.sort(sideids[cp,0],axis=1)
        layer_counts=collections.Counter(map(tuple,layers.tolist()))
        # Descriptive source-edge bins, not risk thresholds: chord thirds and span halves.
        wall=front.coordinates;centre=wall[top.edges].mean(1);lo=wall.min(0);hi=wall.max(0)
        chord=np.minimum(2,np.floor(3*(centre[:,0]-lo[0])/(hi[0]-lo[0])).astype(int))
        span=(centre[:,1]>(lo[1]+hi[1])/2).astype(int);regions=chord*2+span
        reg=np.sort(regions[sideids[cp,1]],axis=1);region_counts=collections.Counter(map(tuple,reg.tolist()))
        changed_ids=sideids[changed];vertex_changes=[]
        for k in range(20):
            x=future.vertex['coordinates'][k,ids];different=np.any(x.view(np.uint64)!=coords[k,ids].view(np.uint64),axis=1)
            vertex_changes.append(dict(layer=k+1,vertices=ids[different].tolist()))
        report=dict(proposal=group,members=r['members'],core_edges=len(edges),candidate_edges=len(candidate.edges),baseline_edges=len(baseline.edges),
            candidate_side_surfaces=len(candidate.sides),baseline_side_surfaces=len(baseline.sides),baseline_pairs=len(bp),candidate_pairs=len(cp),
            pair_count_by_layer_pair=[dict(layers=k,count=v) for k,v in sorted(layer_counts.items())],
            pair_count_by_source_region=[dict(regions=k,count=v) for k,v in sorted(region_counts.items())],region_definition=dict(chord='equal thirds of source x extent',span='halves of source y extent',source_min=lo,source_max=hi),
            changed_changed_pairs=int((touch==2).sum()),changed_unchanged_pairs=int((touch==1).sum()),unchanged_unchanged_pairs=int((touch==0).sum()),
            inherited_pair_ids=int(inherited.sum()),inherited_identical_geometry_pairs=int(((touch==0)&inherited).sum()),
            new_pair_ids=int((~inherited).sum()),removed_pair_ids=int((~np.isin(bn,cn)).sum()),
            pairs_outside_changed_interaction_support=int((touch==0).sum()),
            changed_sides=int(changed.sum()),unchanged_sides=int(same.sum()),changed_side_layers=sorted(set(changed_ids[:,0].tolist())),
            changed_side_edges=sorted(set(changed_ids[:,1].tolist())),changed_vertices_by_layer=vertex_changes,
            side_geometry_hash=array_hash(candidate.quads),candidate_pair_hash=array_hash(np.sort(cn)),baseline_pair_hash=array_hash(np.sort(bn)),
            candidate_coordinates_hash=r['proposal'])
        reports.append(report);timings.append(dict(group=group,seconds=time.perf_counter()-st,peak_rss=psutil.Process().memory_info().peak_wset))
        print(json.dumps({k:v for k,v in report.items() if k in ('proposal','candidate_pairs','baseline_pairs','changed_changed_pairs','changed_unchanged_pairs','unchanged_unchanged_pairs','inherited_identical_geometry_pairs','changed_sides','unchanged_sides')},indent=2),flush=True)
    h=scientific_hash(reports)
    if replay:assert scientific_hash(json.loads((OUT/'PAIR_DECOMPOSITION.json').read_text()))==h
    else:(OUT/'PAIR_DECOMPOSITION.json').write_text(json.dumps(plain(reports),indent=2,sort_keys=True)+'\n')
    (OUT/('PAIR_DECOMPOSITION_REPLAY.json' if replay else 'PAIR_DECOMPOSITION_HASH.json')).write_text(json.dumps(dict(scientific_hash=h),indent=2)+'\n')
    (OUT/('DECOMPOSITION_REPLAY_TIMINGS.json' if replay else 'DECOMPOSITION_TIMINGS.json')).write_text(json.dumps(dict(stages=timings,total_seconds=time.perf_counter()-started),indent=2)+'\n')

if __name__=='__main__':run('--replay' in sys.argv)
