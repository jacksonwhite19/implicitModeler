"""Inspect completed checkpoint fields without touching running CFD cases."""
import re
import numpy as np
import capsule_gw_pair as g
f=g.f
f.ROOT=g.ROOT
import capsule_pressure_diagnostic as p


def inspect():
    path=f.ROOT/'HEALTH_CHECKPOINTS.json'
    result=f.h.read(path) if path.exists() else {}
    for name in g.NAMES:
        root=f.ROOT/name;log=root/'pilot.log'
        if not log.exists():continue
        times=re.findall(r'^Time = ([0-9]+)s$',log.read_text(),re.M)
        finished=max(map(int,times),default=0)-1
        if (root/'pilot.exit').exists():finished=500
        for step in (20,50,100,200,300,400,500):
            key=name+':'+str(step)
            if step>finished or key in result:continue
            fields={}
            for field,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
                a=np.concatenate([p.l.u.e.read_internal(root/'pilot'/f'processor{i}'/str(step)/field,dim) for i in range(6)])
                values=np.linalg.norm(a,axis=1) if dim==3 else a[:,0]
                fields[field]=dict(min=float(values.min()),max=float(values.max()),nonfinite=int((~np.isfinite(a)).sum()),negative=int((values<0).sum()))
            result[key]=fields
            f.h.durable_json(path,result)
            assert all(v['nonfinite']==0 for v in fields.values()),(key,fields)
            assert fields['k']['negative']==fields['omega']['negative']==0,(key,fields)
            print(key,fields)
    return result


if __name__=='__main__':inspect()
