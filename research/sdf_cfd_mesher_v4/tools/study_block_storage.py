"""Actual preserved-population block writes, prior to choosing granularity."""
import json,time,gc,shutil
import numpy as np
import psutil
from study_block_dispersion import ROOT,load
from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import buffers
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items
from sdf_cfd_mesher_v4.advancing_front import canonical_blocks as blocks

def run():
    start=time.perf_counter();base=load('optimized_7_9','aef8b2175a6c1dcdc7d91ca9a549abc4b58f5687985994eaa9fbbec9ddea9a9a')
    k,v,_=buffers((k,v,0) for k,v in raw_items(base))
    np.save(ROOT/'arrays/values6.npy',v)
    delta=load('final_bench');dk,dv,_=buffers((k,v,0) for k,v in raw_items(delta.updates))
    pos=np.searchsorted(k,dk);found=pos<len(k);found[found]=k[pos[found]]==dk[found];actions=found.astype('u1')
    np.savez(ROOT/'arrays/delta7_values.npz',keys=dk,values=dv,actions=actions)
    result=[]
    for depth in (8,12,16):
        directory=ROOT/f'probe_{depth}'/'objects';directory.mkdir(parents=True,exist_ok=True)
        tick=time.perf_counter();prior=blocks.build(directory,k,v,depth);build_seconds=time.perf_counter()-tick
        tick=time.perf_counter();new=blocks.update(directory,prior,dk,dv,actions);update_seconds=time.perf_counter()-tick
        nk=np.union1d(k,dk);nv=np.empty(len(nk),dtype='S32');nv[np.searchsorted(nk,k)]=v;nv[np.searchsorted(nk,dk)]=dv
        tick=time.perf_counter();independent=blocks.canonical_root(nk,nv,depth);audit_seconds=time.perf_counter()-tick
        assert independent==new['root']
        row=dict(depth=depth,build_seconds=build_seconds,update_seconds=update_seconds,independent_seconds=audit_seconds,root=new['root'],build_work=prior['work'],update_work=new['work'],active_blocks=len(new['blocks']),active_bytes=sum(x['bytes'] for x in new['blocks']),storage_bytes=sum(p.stat().st_size for p in directory.iterdir()),rss=psutil.Process().memory_info().rss,peak_rss=psutil.Process().memory_info().peak_wset)
        result.append(row);print(json.dumps(row),flush=True)
        (ROOT/'BLOCK_GRANULARITY.json').write_text(json.dumps(dict(results=result,total_seconds=time.perf_counter()-start),indent=2)+'\n')
        # These disposable probes have no published checkpoint or sealed authority.
        assert directory.resolve().parent.parent==ROOT.resolve() and directory.name=='objects'
        shutil.rmtree(directory);del nk,nv,prior,new;gc.collect()

if __name__=='__main__':run()
