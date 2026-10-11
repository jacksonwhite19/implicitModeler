"""Observational production phase/RSS and reachable allocation accounting."""
import os,time,json,threading,functools,sys,types
from pathlib import Path
import numpy as np,psutil

def allocations(value):
    seen=set();buffers={};python_bytes=0;arrays=0;views=0
    def visit(x):
        nonlocal python_bytes,arrays,views
        if id(x) in seen:return
        seen.add(id(x))
        if isinstance(x,np.ndarray):
            arrays+=1;base=x
            while isinstance(base.base,np.ndarray):base=base.base
            views+=int(base is not x)
            buffers[id(base)]=dict(bytes=base.nbytes,mapped=isinstance(base,np.memmap),dtype=str(base.dtype),shape=list(base.shape))
            python_bytes+=sys.getsizeof(x)-(x.nbytes if x.flags.owndata else 0);return
        if isinstance(x,(types.ModuleType,types.FunctionType,type)):return
        python_bytes+=sys.getsizeof(x)
        if isinstance(x,dict):
            for k,v in x.items():visit(k);visit(v)
        elif isinstance(x,(list,tuple,set,frozenset)):
            for v in x:visit(v)
        elif hasattr(x,'__dict__') and type(x).__module__.startswith('sdf_cfd_mesher_v4'):
            visit(vars(x))
    visit(value)
    return dict(python_bytes=python_bytes,array_objects=arrays,views=views,
        owned_array_bytes=sum(r['bytes'] for r in buffers.values() if not r['mapped']),
        mapped_array_bytes=sum(r['bytes'] for r in buffers.values() if r['mapped']),
        buffers=sorted(buffers.values(),key=lambda r:r['bytes'],reverse=True),objects=len(seen))

class Observer:
    def __init__(self,path):
        self.path=Path(path);self.path.mkdir(parents=True,exist_ok=True);self.start=time.perf_counter()
        self.profile_allocations=os.environ.get('V4_MEMORY_PROFILE')!='rss'
        self.stack=['process_start'];self.layer=0;self.rows=[];self.objects=[];self.counts={};self.stop=threading.Event()
        self.process=psutil.Process();self.thread=threading.Thread(target=self.sample,daemon=True);self.thread.start()
    def rss(self):
        rss=private=0;processes=[]
        for p in [self.process,*self.process.children(recursive=True)]:
            try:
                m=p.memory_info();rss+=m.rss;private+=getattr(m,'private',0);processes.append(dict(pid=p.pid,rss=m.rss,private=getattr(m,'private',0)))
            except (psutil.NoSuchProcess,psutil.AccessDenied):pass
        return dict(seconds=time.perf_counter()-self.start,layer=self.layer,phase='/'.join(self.stack),rss=rss,private=private,processes=processes)
    def sample(self):
        while not self.stop.is_set():self.rows.append(self.rss());self.stop.wait(.05)
    def mark(self,name,value=None):
        row=self.rss();row['mark']=name;self.rows.append(row)
        if value is not None and self.profile_allocations:
            self.stack.append('allocation_accounting')
            try:self.objects.append(dict(mark=name,layer=self.layer,**allocations(value)))
            finally:self.stack.pop()
    def wrap(self,owner,name,label,measure=False):
        original=getattr(owner,name)
        @functools.wraps(original)
        def observed(*args,**kwargs):
            self.stack.append(label);tick=time.perf_counter();key=(self.layer,label)
            first=key not in self.counts;self.counts[key]=self.counts.get(key,0)+1
            try:
                if measure and first:self.mark(label+':inputs',args)
                result=original(*args,**kwargs)
                if measure and first:self.mark(label+':outputs',result)
                return result
            finally:
                self.mark(label+':end');self.stack.pop()
        setattr(owner,name,observed)
    def install(self):
        import task3c_io
        self.wrap(task3c_io,'read','evidence_read',True);self.wrap(task3c_io,'write','evidence_write',True)
        from sdf_cfd_mesher_v4.advancing_front import rolling_metrics,native_checkpoint
        from sdf_cfd_mesher_v4.forecast import engine
        from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache
        from sdf_cfd_mesher_v4.forecast.detailed import rolling_spots
        from sdf_cfd_mesher_v4.certificate import production
        from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeRiskAtlas
        self.wrap(engine,'vertex_trajectories','forecast_projection',True)
        self.wrap(rolling_metrics,'evaluate_layer','immediate_metrics',True)
        self.wrap(DetailedCache,'get','detailed_lookahead',True)
        self.wrap(rolling_spots,'compare_spots','future_spot_checks',True)
        self.wrap(production,'surface_arrays','certificate_geometry',True)
        self.wrap(production,'certify_immediate','immediate_certificate',True)
        self.wrap(native_checkpoint.NativeCheckpointStore,'publish','checkpoint_write',True)
        self.wrap(native_checkpoint.NativeCheckpointStore,'restart','checkpoint_restart',True)
        self.wrap(NativeRiskAtlas,'__init__','native_atlas',False)
        import task4_certify_candidate
        self.wrap(task4_certify_candidate,'exact_all','exact_jacobians',False)
    def finish(self):
        self.stop.set();self.thread.join();self.mark('task_end')
        peak=max((r for r in self.rows if 'allocation_accounting' not in r['phase']),key=lambda r:r['rss'])
        (self.path/'RSS_TRACE.json').write_text(json.dumps(self.rows,indent=2)+'\n')
        (self.path/'ALLOCATIONS.json').write_text(json.dumps(self.objects,indent=2)+'\n')
        (self.path/'SUMMARY.json').write_text(json.dumps(dict(peak=peak,seconds=time.perf_counter()-self.start),indent=2)+'\n')
        return peak
