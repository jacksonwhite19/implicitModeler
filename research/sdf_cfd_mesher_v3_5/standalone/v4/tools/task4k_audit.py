from task4k_common import *
from task4k_instrument import Instruments
from sdf_cfd_mesher_v4.certificate.incremental import kernel
from sdf_cfd_mesher_v4.certificate.efficient_root_keys import memoized_root_keys
from sdf_cfd_mesher_v4.certificate.exact_preparation import memoized_exact_preparation
from concurrent.futures import ProcessPoolExecutor,as_completed
import gc,psutil,threading,argparse,contextlib
from sdf_cfd_mesher_v4.certificate.cad_algebra_cache import memoized_cad_algebra,cache_info

def snapshot():
    result={name:getattr(kernel.adapter,name).cache_info()._asdict() for name in ('normalize','initial_intervals')}
    for name in ('exact_field','root_record'):
        fn=getattr(kernel.module('exact_algebraic'),name)
        for _ in range(8):
            if hasattr(fn,'cache_info'):
                result[name]=fn.cache_info()._asdict();break
            fn=getattr(fn,'__wrapped__',None) or (getattr(fn,'__kwdefaults__',None) or {}).get('_fn')
            if fn is None:break
    return result
def worker(job):
    group,cases,q,labels,submitted,mode=job;process=psutil.Process();started=time.perf_counter();records=[]
    n=kernel.module('noncoincident_rank3_certificate');gc_clock=[None,0.]
    def gc_event(phase,info):
        if phase=='start':gc_clock[0]=time.perf_counter()
        elif gc_clock[0] is not None:gc_clock[1]+=time.perf_counter()-gc_clock[0];gc_clock[0]=None
    gc.callbacks.append(gc_event)
    path=OUT/f'{mode}_worker_{group}.jsonl';path.write_bytes(b'')
    optimized=mode in ('target_after','optimized','target_single_after')
    context=memoized_cad_algebra() if optimized else contextlib.nullcontext()
    context.__enter__()
    try:
        for replay in range(3):
            with Instruments(extra=mode.startswith('target')) as timers:
                for offset in range(0,len(cases),8):
                    kernel.clear_kernel_caches()
                    with memoized_root_keys() as keys,memoized_exact_preparation() as prep:
                        for j in range(offset,min(offset+8,len(cases))):
                            guard();case=cases[j];timers.reset();pure_before=cache_info();before=snapshot();key_before=keys.cache_info();gc_before=gc_clock[1];collections_before=[r['collections'] for r in gc.get_stats()]
                            rss_before=process.memory_info().rss;objects_before=len(gc.get_objects()) if j%16==0 else None
                            a,b=(kernel.Patch.binary(q[j,k],tuple(map(int,labels[j,k]))) for k in (0,1));assert a.rank==b.rank==3
                            a,b=kernel.certificate.canonical_pair(a,b)
                            gc_kernel_start=gc_clock[1];start=time.perf_counter();cpu=time.process_time();result=kernel.certificate.canonical_object(n.cert(a,b));cpu=time.process_time()-cpu;dt=time.perf_counter()-start;finished=time.perf_counter();gc_kernel=gc_clock[1]-gc_kernel_start
                            assert scientific_hash(result)==case['result_hash'],('CADProofMismatch',case['index'],replay)
                            encoding=time.perf_counter();payload=canonical(result);serialization=time.perf_counter()-encoding
                            after=snapshot();ki=keys.cache_info();record=dict(index=case['index'],case_identity=case['identity'],replay=replay,cache_state='cold_worker_start' if replay==0 else 'warm_worker',group=group,pid=os.getpid(),process_start=process.create_time(),process_age_seconds=time.time()-process.create_time(),case_order=j,worker_start=started,submitted=submitted,case_start=start,case_finish=finished,initial_queue_seconds=started-submitted,execution_seconds=dt,cpu_seconds=cpu,cpu_wall_ratio=cpu/dt,rss_before=rss_before,rss_after=process.memory_info().rss,peak_rss=process.memory_info().peak_wset,gc_seconds=gc_kernel,gc_total_seconds=gc_clock[1]-gc_before,gc_collections=[r['collections']-old for r,old in zip(gc.get_stats(),collections_before)],sampled_objects_before=objects_before,sampled_objects_after=len(gc.get_objects()) if objects_before is not None else None,root_keys_hits=ki.hits-key_before.hits,root_keys_misses=ki.misses-key_before.misses,adapter_cache={name:{key:after[name][key]-before[name][key] for key in ('hits','misses')} for name in after},serialization_seconds=serialization,proof_bytes=len(payload),result_hash=scientific_hash(result),**timers.result())
                            record['pure_cache']={name:{key:values[key]-pure_before[name][key] for key in ('hits','misses')} for name,values in cache_info().items()}
                            records.append(record)
                            with path.open('ab') as f:f.write(canonical(record)+b'\n')
    finally:
        context.__exit__(None,None,None)
        gc.callbacks.remove(gc_event)
    return records

def run():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',default='baseline');parser.add_argument('--limit',type=int);parser.add_argument('--indices');args=parser.parse_args();cases=read(OUT/'CAD_CASES.json')
    if args.indices:cases=[cases[i] for i in map(int,args.indices.split(','))]
    if args.limit:cases=cases[:args.limit]
    with np.load(OUT/'CAD_INPUTS.npz') as z:q=z['quads'];labels=z['labels']
    start=time.perf_counter();rows=[];samples=[];stop=threading.Event()
    def sample():
        while not stop.wait(.5):
            try:
                pp=[psutil.Process(),*psutil.Process().children(recursive=True)]
                samples.append(dict(elapsed=time.perf_counter()-start,rss=sum(p.memory_info().rss for p in pp),cpu=sum(sum(p.cpu_times()[:2]) for p in pp),processes=len(pp),system_cpu=psutil.cpu_percent(),memory_percent=psutil.virtual_memory().percent))
            except psutil.Error:pass
    thread=threading.Thread(target=sample,daemon=True);thread.start()
    try:
        with ProcessPoolExecutor(max_workers=8) as pool:
            jobs=[]
            for group in range(min(8,len(cases))):
                selected=cases[group::8];indices=[c['index'] for c in selected];jobs.append(pool.submit(worker,(group,selected,q[indices],labels[indices],time.perf_counter(),args.mode)))
            for job in as_completed(jobs):
                result=job.result();received=time.perf_counter()
                for row in result:row['batch_result_received']=received;row['batch_return_delay']=received-row['case_finish']
                rows.extend(result);print('completed worker cohort',len(rows),flush=True)
    finally:stop.set();thread.join()
    rows.sort(key=lambda r:(r['replay'],r['index']));save(f'{args.mode.upper()}_CASES.json',rows);save(f'{args.mode.upper()}_SYSTEM.json',dict(seconds=time.perf_counter()-start,samples=samples,peak_rss=max((r['rss'] for r in samples),default=0)))
    for case in cases:
        group=[r for r in rows if r['index']==case['index']]
        for key in ('result_hash','comparison_trace','root_trace'):assert len({r[key] for r in group})==1,('TraceMismatch',case['index'],key)
if __name__=='__main__':run()
