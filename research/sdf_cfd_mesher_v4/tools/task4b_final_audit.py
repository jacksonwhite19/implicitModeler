"""Finished-shell completeness audit of immutable exact proof transactions."""
from task4b_layer_probe import *
from sdf_cfd_mesher_v4.certificate.efficient_ledger import ProofLedger,digest
from sdf_cfd_mesher_v4.certificate.incremental import kernel
def run(name='optimized_2'):
    started=time.perf_counter();source=json.loads((OUT/'OPTIMIZED_SOURCE_AUTHORITY.json').read_text())
    for p,h in source.items():assert file_hash(ROOT/p)==h,('ChangedProofImplementation',p)
    f,t,x=geometry();q,labels=arrays(x,t.edges,20);E=len(t.edges);native=Filter();s=time.perf_counter();forest=native.forest(q,E);index_seconds=time.perf_counter()-s
    final=read(OUT/name/'COMPLETENESS.json');assert final['complete'];ledger=ProofLedger(t.sha256,native.source_hash);prev='wall:'+f.coordinate_hash;rows=[]
    for k in range(1,21):
        s=time.perf_counter();pairs=forest.pairs(k);enum_seconds=time.perf_counter()-s;s=time.perf_counter()
        with np.load(OUT/name/f'PAIRS_{k:02}.npz') as z:arrays_={key:z[key] for key in z.files}
        rec=read(OUT/name/f'LAYER_{k:02}.json');exact=read(OUT/name/f'EXACT_{k:02}.json')
        assert np.array_equal(pairs,arrays_['packed']) and (len(pairs)<2 or np.all(pairs[1:]>pairs[:-1]))
        for key,field in [('packed','pair_hash'),('codes','code_hash'),('witness','witness_hash'),('outcomes','outcome_hash')]:assert array_hash(arrays_[key])==rec[field]
        assert scientific_hash(exact)==rec['fallback_hash']
        actual=digest([prev,array_hash(q[(k-1)*E:k*E]),array_hash(labels[(k-1)*E:k*E])]);assert actual==rec['geometry_hash'] and prev==rec['previous_geometry_hash']
        codes=arrays_['codes'];outcomes=arrays_['outcomes'];assert np.all((codes>=0)&(codes<=6));assert np.all(np.isin(outcomes,[1,2]))
        unknown=np.flatnonzero(codes==0);assert len(unknown)==len(exact)==rec['full_exact_calls']
        for i,r in zip(unknown,exact):
            assert r['pair']==[int(pairs[i])>>32,int(pairs[i])&0xffffffff]
            assert r['result']['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
            assert outcomes[i]==(1 if r['result']['classification']=='CertifiedSeparated' else 2)
        assert np.all(outcomes[(codes==1)|(codes==2)]==1) and np.all(outcomes[codes>=3]==2)
        assert rec['candidate_pairs']==rec['completed_pairs']==len(pairs)
        ledger.append(rec);prev=actual;rows.append(dict(layer=k,pairs=len(pairs),enumeration_seconds=enum_seconds,binding_and_io_seconds=time.perf_counter()-s))
    union=ledger.final(20);assert union['layers']==final['layers'] and union['candidate_pairs']==final['candidate_pairs']
    result=dict(complete=True,layer_count=20,pairs=union['candidate_pairs'],reused_pairs=union['candidate_pairs'],missing_pairs=0,extra_pairs=0,duplicate_pairs=0,
        forbidden=0,unresolved=0,new_exact_calls=0,geometry_identity=prev,coverage_hash=final['scientific_hash'],source_authority_hash=file_hash(OUT/'OPTIMIZED_SOURCE_AUTHORITY.json'))
    result['scientific_hash']=scientific_hash(result);save('FINAL_AUDIT.json',result)
    save('FINAL_AUDIT_TIMINGS.json',dict(total_seconds=time.perf_counter()-started,index_seconds=index_seconds,layers=rows,peak_rss=psutil.Process().memory_info().peak_wset))
    print('FINAL COMPLETE AUDIT',result['pairs'],result['scientific_hash'],flush=True)
if __name__=='__main__':run()
