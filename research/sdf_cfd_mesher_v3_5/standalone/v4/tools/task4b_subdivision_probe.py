from task4b_layer_probe import *
from sdf_cfd_mesher_v4.certificate.exact_subdivision import subdivided_separation
from sdf_cfd_mesher_v4.certificate.incremental import kernel
if __name__=='__main__':
    f,t,x=geometry();q,labels=arrays(x,t.edges,2);rows=[]
    for record in read(OUT/'RESIDUAL_LAYER2_EXACT.json'):
        if record['result']['method']!='NoncoincidentRank3ExactCAD':continue
        a,b=[kernel.Patch.binary(q[i],tuple(map(int,labels[i]))) for i in record['pair']]
        start=time.perf_counter();r=subdivided_separation(a,b);elapsed=time.perf_counter()-start
        assert r is None or r['classification']==record['result']['classification']
        row=dict(pair=record['pair'],old=record['result'],new=r,seconds=elapsed);rows.append(row);print(record['pair'],elapsed,r is not None,flush=True)
    save('SUBDIVISION_PROBE.json',rows)
