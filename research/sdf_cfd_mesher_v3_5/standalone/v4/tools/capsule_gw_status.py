"""Read-only live diagnostics; never changes controls or solver horizon."""
import json
import re
import shutil
import capsule_gw_pair as g
import capsule_pressure_diagnostic as p


def status():
    result={}
    for name in g.NAMES:
        d=g.ROOT/name;con=g.h.read(d/'CFD_CONTRACT.json')
        for file,digest in con['controls'].items():assert g.h.file_hash(d/'pilot'/file)==digest
        text=(d/'pilot.log').read_text();matches=list(re.finditer(r'^Time =',text,re.M))
        complete=text if (d/'pilot.exit').exists() else text[:matches[-1].start()]
        rows=p.l.u.e.parse_history(complete)
        assert [r['iteration'] for r in rows]==list(range(1,len(rows)+1))
        r=rows[-1]
        result[name]=dict(completed=len(rows),bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']],maxiter=[r['iteration'] for r in rows if any(x['iterations']>=5000 for x in r['residuals'].values())],flux=r['flux']['normalized'],wall=r['flux']['wall_sumMag'],controls_exact=True,fatal='FOAM FATAL' in text)
        assert not result[name]['fatal'],result[name]
    result['free_bytes']=shutil.disk_usage(g.ROOT).free
    return result


if __name__=='__main__':print(json.dumps(status(),sort_keys=True))
