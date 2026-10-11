"""Preserve and reproduce the study-only int-width checkpoint defect.

No search or geometry result changes. Re-materialize only the unsealed Task-4C
atlas metadata with canonical ID hashing, then test full SF2 restart/rollback.
"""
from task4c_common import *
from sdf_cfd_mesher_v4.advancing_front.risk_atlas import RiskAtlas
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf

def old_hash(a):
    p=a.checkpoint()
    for k in ('future_ids','states','generations','last_evaluated'):p[k]=array_hash(p[k])
    p['retained']=[array_hash(x) for x in p['retained']]
    return scientific_hash(p)

def run():
    f,t,_=geometry();s=load_schedule();sdf=NativeSdf();rows=[]
    for k in (2,8,12,20):
        path=OUT/f'runner_{k:02}';original=path/f'LAYER{k}_ATLAS.npz';saved=path/f'LAYER{k}_ATLAS.before_integer_fix.npz'
        if not saved.exists():saved.write_bytes(original.read_bytes())
        c,x,n=load_candidate(path/f'LAYER{k}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz');a=RiskAtlas(f,t,s,sdf.source_identity,x,n)
        with np.load(saved) as z:
            p=json.loads(str(z['metadata']));stored=str(z['scientific_hash'])
            for key in ('states','generations','last_evaluated'):p[key]=z[key]
        nold=sum(r['corrected'] for r in read(BASE/'CORRECTION_HISTORY.json')['outcomes'])
        for layer in range(2,k):nold+=sum(r.get('accepted',False) and 'scope' in r for r in read(BASE/f'LAYER{layer}_CORRECTIONS.json'))
        p['future_ids']=a.future_ids.copy()
        p['retained']=[np.asarray(ids,np.int32 if i<nold else np.int64) for i,ids in enumerate(p['retained'])]
        a.restore(p);assert old_hash(a)==stored,'RootCauseMustReproduceOriginalHash'
        fixed=a.scientific_hash();a.save(original)
        b=RiskAtlas(f,t,s,sdf.source_identity,x,n);b.load(original);assert b.scientific_hash()==fixed
        snap=b.checkpoint();b.invalidate([0]);b.restore(snap);assert b.scientific_hash()==fixed
        if k<20:assert np.array_equal(b.nominal(k+1,c).view(np.uint64),x[k].view(np.uint64))
        for filename in (path/f'LAYER{k}_ATLAS.json',OUT/f'RUNNER_{k:02}.json'):
            rec=read(filename)
            if 'active_atlas' in rec:rec['active_atlas']['scientific_hash']=fixed
            else:rec['scientific_hash']=fixed
            if filename.parent==path:write(filename,rec)
            else:save(filename.name,rec)
        rows.append(dict(layer=k,old_hash=stored,canonical_hash=fixed,old_hash_reproduced=True,source_id_values_unchanged=True,
            coordinate_hash=array_hash(c),forecast_hash=array_hash(x),restart_exact=True,rollback_exact=True,next_nominal_exact=True,
            preserved_reproducer=saved.relative_to(ROOT).as_posix()))
    save('METADATA_INTEGER_REPAIR.json',dict(cause='Windows NumPy inferred int32 IDs; checkpoint reload int64 IDs. Hash now uses canonical little-endian int64.',rows=rows))
    save('SF2_ATLAS_RESTART.json',rows)
    print('Original defect reproduced; canonical SF2 restart and rollback passed',len(rows))

if __name__=='__main__':run()
