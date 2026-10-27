"""Bounded measured baseline, plus preserved complete transaction accounting."""
from task4b_common import *
import cProfile,pstats,gzip,collections,psutil
from types import SimpleNamespace
from sdf_cfd_mesher_v4.certificate.rolling import owned_scope
from sdf_cfd_mesher_v4.certificate.incremental import certify,ProofCache,kernel
from sdf_cfd_mesher_v4.forecast.detailed.continuous import SideIndex

def run():
    start=time.perf_counter();f,t,x=geometry();layers=[];methods=collections.Counter();classes=collections.Counter()
    for k in range(1,21):
        timing=json.loads((BASE/f'LAYER_{k:02}_TIMINGS.json').read_text());ledger=read(BASE/f'LAYER_{k:02}_CERTIFICATE.json')
        m=collections.Counter();c=collections.Counter()
        for b in ledger['batches']:m.update(b['methods']);c.update(b['counts'])
        methods.update(m);classes.update(c);s=timing['certificate_stages'];E=len(t.edges);N=k*E
        paths=list((BASE/f'certificate_layer_{k:02}').glob('*'))
        layers.append(dict(layer=k,new_surfaces=E,total_surfaces=N,potential_new_pairs=E*(N-E)+E*(E-1)//2,
            broad_pairs=ledger['candidate_pairs'],new_proofs=ledger['completed_pairs'],kernel_calls=sum(v['exact_kernel_calls'] for v in s),
            previous_proofs=sum(r['new_proofs'] for r in layers),cache_hits=sum(v['cache_hits'] for v in s),
            worker_enumeration_seconds=sum(v['enumeration_seconds'] for v in s),worker_total_seconds=sum(v['seconds'] for v in s),
            wall_seconds=timing.get('certificate_wall_seconds'),transaction_total_seconds=timing['total_seconds'],methods=dict(m),classifications=dict(c),
            persistent_bytes=sum(p.stat().st_size for p in paths if p.is_file()),rss=timing.get('observed_tree_peak_rss',timing.get('peak_rss'))))
    mid=f.coordinates[t.edges].mean(1)
    anchors=[('smooth',1,int(np.argmin(np.sum((mid-[.45,.1,.05])**2,axis=1)))),
             ('corrected_cap',1,0),('trailing',10,int(np.argmin(np.sum((mid-[1.,.1,0])**2,axis=1)))),('late_nominal',20,0)]
    samples=[]
    for name,k,e in anchors:
        kernel.clear_kernel_caches();begin=e;end=min(e+16,len(t.edges));p=OUT/f'profile_{name}.jsonl.gz'
        cp=cProfile.Profile();cp.enable();q=time.perf_counter()
        index=SideIndex(f,SimpleNamespace(vertex={'coordinates':x[1:k+1]}),t);scope=owned_scope(index,k,begin,end);scope_seconds=time.perf_counter()-q
        with p.open('wb') as raw:
            with gzip.GzipFile(fileobj=raw,filename='',mode='wb',mtime=0) as z:proof,perf=certify(scope,cache=ProofCache(),seconds_budget=90,record_sink=z.write)
        cp.disable();assert proof['complete'],name
        stats=pstats.Stats(cp);rows=[]
        for (path,line,func),(cc,nc,tt,ct,callers) in stats.stats.items():
            rows.append(dict(file=path.replace(str(ROOT),'V4'),line=line,function=func,primitive_calls=cc,calls=nc,self_seconds=tt,cumulative_seconds=ct))
        rows.sort(key=lambda r:(-r['self_seconds'],r['file'],r['line']))
        save(f'profile_{name}_functions.json',rows)
        pairs,_=scope.pairs();ranks=collections.Counter();patches={i:scope.patch(i) for i in np.unique(pairs)}
        for a,b in pairs:ranks[str(tuple(sorted((patches[a].rank,patches[b].rank))))]+=1
        sample=dict(name=name,layer=k,begin=begin,end=end,scope_surfaces=len(scope.sides),scope_seconds=scope_seconds,
            pairs=len(pairs),proof=proof,performance=perf,ranks=dict(ranks),profile_total_seconds=stats.total_tt,profile_total_calls=stats.total_calls,
            record_bytes=p.stat().st_size,peak_rss=psutil.Process().memory_info().peak_wset)
        save(f'profile_{name}.json',sample);samples.append(sample);print(name,len(pairs),stats.total_tt,proof['methods'],flush=True)
    save('BASELINE.json',dict(layers=layers,methods=dict(methods),classifications=dict(classes),profiles=samples,seconds=time.perf_counter()-start))
if __name__=='__main__':run()
