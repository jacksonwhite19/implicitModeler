"""Preserved geometry only: independent compact proof transaction runs."""
from task4b_layer_probe import *
from sdf_cfd_mesher_v4.certificate.efficient_proof import prove,PROOF_CLASSES
from sdf_cfd_mesher_v4.certificate.efficient_ledger import ProofLedger,digest
from sdf_cfd_mesher_v4.certificate.incremental import kernel
import gzip
from concurrent.futures import ProcessPoolExecutor

def baseline_compare(k,packed,outcomes,E,mid):
    started=time.perf_counter();pp=[];cc=[];ranks=collections.Counter();methods=collections.Counter();uncompressed=0
    archived=OUT/'ATTEMPT1_BASELINE_EQUIVALENCE.json'
    if archived.exists():
        prior=next((r for r in json.loads(archived.read_text()) if r['layer']==k),None)
        if prior:
            record=read(OUT/f'A/LAYER_{k:02}.json')
            assert array_hash(packed)==record['pair_hash'] and array_hash(outcomes)==record['outcome_hash']
            return dict(prior,decompression_and_json_comparison_seconds=time.perf_counter()-started,reused_complete_baseline_comparison=True)
    for file in sorted((BASE/f'certificate_layer_{k:02}').glob('PAIRS*.gz')):
        for line in gzip.open(file,'rb'):
            uncompressed+=len(line);r=json.loads(line);a=(r['side_a'][0]-1)*E+r['side_a'][1];b=(r['side_b'][0]-1)*E+r['side_b'][1]
            pp.append((min(a,b)<<32)|max(a,b));cc.append(1 if r['classification']=='CertifiedSeparated' else 2 if r['classification']=='CertifiedAllowedIncidentContact' else 3)
            ranks[str(r.get('ranks'))]+=1;methods[r['method']]+=1
    pp=np.array(pp,np.uint64);order=np.argsort(pp);pp=pp[order];cc=np.array(cc,np.uint8)[order]
    assert np.array_equal(pp,packed),'CompletePairPopulationMismatch'
    if not np.array_equal(cc,outcomes):
        bad=np.flatnonzero(cc!=outcomes);save(f'MISMATCH_{k}.json',dict(pairs=packed[bad].tolist(),old=cc[bad].tolist(),new=outcomes[bad].tolist()));raise ValueError('ExactOutcomeMismatch')
    return dict(layer=k,pairs=len(pp),all_pairs_identical=True,all_outcomes_identical=True,ranks=dict(ranks),methods=dict(methods),
        decompression_and_json_comparison_seconds=time.perf_counter()-started,uncompressed_bytes=uncompressed,outcome_hash=array_hash(cc))

def run(name='A',compare_baseline=False):
    began=time.perf_counter();f,t,x=geometry();q,labels=arrays(x,t.edges,20);E=len(t.edges);native=Filter();path=OUT/name;path.mkdir(exist_ok=True)
    forest=None;ledger=ProofLedger(t.sha256,native.source_hash);timings=[];comparisons=[];prev='wall:'+f.coordinate_hash;counts=[];pair_matrix=np.zeros((20,20),np.int64)
    pool=ProcessPoolExecutor(max_workers=8)
    mid=f.coordinates[t.edges].mean(1);region=np.minimum((np.clip(mid[:,0],0,1)*3).astype(int),2)*3+np.minimum((np.clip((mid[:,1]+.25)/.5,0,1)*3).astype(int),2)
    spatial=np.zeros(9,np.int64)
    for k in range(1,21):
        started=time.perf_counter();new=q[(k-1)*E:k*E];newlabels=labels[(k-1)*E:k*E];s=time.perf_counter()
        if forest is None:forest=native.forest(new,E)
        else:forest.append(new)
        index_seconds=time.perf_counter()-s;s=time.perf_counter();packed=forest.pairs(k);enumeration_seconds=time.perf_counter()-s
        s=time.perf_counter();valid=native.valid(new);valid_fallback=[]
        for i in np.flatnonzero(valid==0):
            p=kernel.Patch.binary(new[i],tuple(map(int,newlabels[i])));ok,why=p.valid();valid_fallback.append(dict(side=(k-1)*E+int(i),passed=ok,reason=why));assert ok
        validity_seconds=time.perf_counter()-s
        result,perf=prove(q[:k*E],labels[:k*E],packed,native,executor=pool)
        assert not np.any(result['outcomes']==3),'NonpassingPair'
        a=packed>>np.uint64(32);b=packed&np.uint64(0xffffffff);pair_matrix[:,k-1]=np.bincount((a//E).astype(int),minlength=20)
        spatial+=np.bincount(region[(b%E).astype(int)],minlength=9)
        s=time.perf_counter();geometry_hash=digest([prev,array_hash(new),array_hash(newlabels)])
        rec=dict(layer=k,topology=t.sha256,implementation=native.source_hash,kernel=kernel.VERSION,previous_geometry_hash=prev,geometry_hash=geometry_hash,
            candidate_pairs=len(packed),completed_pairs=len(packed),forbidden=0,unresolved=0,invalid_surfaces=0,
            pair_hash=array_hash(packed),code_hash=array_hash(result['codes']),witness_hash=array_hash(result['witness']),outcome_hash=array_hash(result['outcomes']),
            fallback_hash=scientific_hash(result['fallback']),validity_hash=array_hash(valid),validity_fallback=valid_fallback,
            class_counts={str(c):int(n) for c,n in zip(*np.unique(result['codes'],return_counts=True))},full_exact_calls=perf['exact_calls'])
        hashing_seconds=time.perf_counter()-s;ledger.append(rec);prev=geometry_hash;s=time.perf_counter()
        np.savez_compressed(path/f'PAIRS_{k:02}.npz',packed=packed,codes=result['codes'],witness=result['witness'],outcomes=result['outcomes'])
        write(path/f'EXACT_{k:02}.json',result['fallback'])
        write(path/f'LAYER_{k:02}.json',rec);serialization_seconds=time.perf_counter()-s
        row=dict(layer=k,index_seconds=index_seconds,enumeration_seconds=enumeration_seconds,validity_seconds=validity_seconds,
            **perf,hashing_seconds=hashing_seconds,serialization_seconds=serialization_seconds,production_seconds=time.perf_counter()-started,
            peak_rss=psutil.Process().memory_info().peak_wset,pairs=len(packed),cheap_pairs=len(packed)-perf['exact_calls'],retained_prior_proofs=sum(counts))
        counts.append(len(packed));timings.append(row);save(f'{name}_TIMINGS.json',timings)
        print(name,k,len(packed),'exact',perf['exact_calls'],'seconds',row['production_seconds'],flush=True)
        if compare_baseline:
            c=baseline_compare(k,packed,result['outcomes'],E,mid);comparisons.append(c);save('FULL_BASELINE_EQUIVALENCE.json',comparisons);print('BASELINE MATCH',k,c['decompression_and_json_comparison_seconds'],flush=True)
    pool.shutdown();final=ledger.final(20);final['side_count']=len(q);final['potential_pairs']=len(q)*(len(q)-1)//2;final['disjoint_AABB_complement']=final['potential_pairs']-final['candidate_pairs']
    final['pair_layer_matrix']=pair_matrix.tolist();final['spatial_counts']=spatial.tolist();final['spatial_definition']='3 equal chord-coordinate bins x 3 equal span-coordinate bins; diagnostic population table, not risk or validity thresholds; pair assigned to its greater stable side ID.'
    final['scientific_hash']=digest({k:v for k,v in final.items() if k!='scientific_hash'});write(path/'COMPLETENESS.json',final)
    save(f'{name}_TOTAL.json',dict(elapsed_including_comparison_seconds=time.perf_counter()-began,production_seconds=sum(r['production_seconds'] for r in timings),
        baseline_comparison_seconds=sum(r['decompression_and_json_comparison_seconds'] for r in comparisons),peak_rss=psutil.Process().memory_info().peak_wset))
    print('COMPLETE',name,final['scientific_hash'],flush=True)
if __name__=='__main__':
    from task4_certify_candidate import MemorySampler
    name=sys.argv[1] if len(sys.argv)>1 else 'A'
    with MemorySampler() as memory:run(name,'--baseline' in sys.argv)
    save(name+'_TREE_MEMORY.json',dict(peak_rss=memory.peak,samples=memory.samples))
