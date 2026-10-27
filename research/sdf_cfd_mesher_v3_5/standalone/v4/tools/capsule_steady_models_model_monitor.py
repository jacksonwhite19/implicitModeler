"""Read-only health monitoring for admitted steady model runs."""
import json
import re
import numpy as np
import capsule_steady_models as s
import capsule_steady_models_history as history
import capsule_pressure_diagnostic as p

def inspect():
    root=s.ROOT/'model_runs';path=root/'HEALTH_CHECKPOINTS.json'
    checkpoints=s.h.read(path) if path.exists() else {};result={}
    for folder in sorted(root.iterdir()):
        if not folder.is_dir() or not (folder/'pilot.log').exists():continue
        con=s.h.read(folder/'CFD_CONTRACT.json');s.h.verify_record(con);model=con['model']
        for rel,digest in con['controls'].items():assert s.h.file_hash(folder/'pilot'/rel)==digest
        text=(folder/'pilot.log').read_text();assert p.l.u.e.warning_audit(text),folder.name
        assert 'FOAM FATAL' not in text and 'Floating point exception' not in text,folder.name
        matches=list(re.finditer(r'^Time =',text,re.M))
        if not matches:continue
        complete=text if (folder/'pilot.exit').exists() else text[:matches[-1].start()]
        rows=history.parse(complete,model)
        if not rows:continue
        result[folder.name]=dict(completed=len(rows),bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']],maxiter=[r['iteration'] for r in rows if any(v['iterations']>=5000 for v in r['residuals'].values())],flux=rows[-1]['flux']['normalized'],wall=rows[-1]['flux']['wall_sumMag'],controls_exact=True)
        fields={'U':3,'p':1,'nut':1,**{f:1 for f in history.TRANSPORT[model]}}
        for step in (20,50,100,200,300,400,500):
            key=folder.name+':'+str(step)
            if step>len(rows) or key in checkpoints:continue
            stats={}
            for field,dim in fields.items():
                a=np.concatenate([p.l.u.e.read_internal(folder/'pilot'/f'processor{i}'/str(step)/field,dim) for i in range(6)])
                v=np.linalg.norm(a,axis=1) if dim==3 else a[:,0]
                stats[field]=dict(min=float(v.min()),max=float(v.max()),negative=int((v<0).sum()),nonfinite=int((~np.isfinite(a)).sum()))
            checkpoints[key]=stats;s.h.durable_json(path,checkpoints)
            assert all(v['nonfinite']==0 for v in stats.values())
            assert all(stats[f]['negative']==0 for f in ('nut',*history.TRANSPORT[model]))
    print(json.dumps(result))

if __name__=='__main__':inspect()
