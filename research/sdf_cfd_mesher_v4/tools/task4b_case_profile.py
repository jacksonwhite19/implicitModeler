"""Profile a preserved pair per residual case; never construct new geometry."""
from task4b_layer_probe import *
from sdf_cfd_mesher_v4.certificate.incremental import kernel
import cProfile,pstats

if __name__=='__main__':
    f,t,x=geometry();q,labels=arrays(x[:3],t.edges,2);seen=set();summary=[]
    for record in read(OUT/'RESIDUAL_LAYER2_EXACT.json'):
        method=record['result']['method']
        if method in seen:continue
        seen.add(method);kernel.clear_kernel_caches();cp=cProfile.Profile();cp.enable();started=time.perf_counter()
        a,b=[kernel.Patch.binary(q[i],tuple(map(int,labels[i]))) for i in record['pair']]
        result=kernel.certificate.certify(a,b);wall=time.perf_counter()-started;cp.disable()
        assert result==record['result'];stats=pstats.Stats(cp);rows=[]
        for (path,line,function),(cc,nc,tt,ct,callers) in stats.stats.items():
            rows.append(dict(file=path.replace(str(ROOT),'V4'),line=line,function=function,calls=nc,self_seconds=tt,cumulative_seconds=ct))
        rows.sort(key=lambda r:(-r['self_seconds'],r['file'],r['line']))
        save('CASE_'+method+'.json',dict(pair=record['pair'],result=result,wall_seconds=wall,profile_seconds=stats.total_tt,functions=rows))
        summary.append(dict(method=method,pair=record['pair'],wall_seconds=wall,profile_seconds=stats.total_tt,result_equal=True))
        print(method,wall,flush=True)
    save('EXACT_CASE_PROFILE.json',summary)
