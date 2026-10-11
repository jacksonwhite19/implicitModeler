"""Dimensional pressure/viscous load decomposition; no convergence gate."""
import numpy as np
import capsule_steady_models_sa as sa
import capsule_pressure_diagnostic as p
s=sa.s;h=s.h


def summarize(rows):
    assert [r['iteration'] for r in rows]==list(range(1,501))
    for kind in ('force','moment'):
        total=np.array([r[kind] for r in rows])
        parts=sum(np.array([r[part+'_'+kind] for r in rows]) for part in ('pressure','viscous'))
        assert np.array_equal(total,parts),'Unexpected porous contribution or load mismatch'
    return {f'{lo}-500':{part+'_'+kind:{axis:p.d.signal(np.array([r[part+'_'+kind][j] for r in rows[lo-1:]])) for j,axis in enumerate('xyz')} for kind in ('force','moment') for part in ('pressure','viscous')} for lo in (201,301,401,451)}


def run():
    paths={mesh:sa.mesh_source(mesh) for mesh in ('GW1-S1','GW1-S2','GC2-S1','GC2-S2','GC2F-S1','GC2F-S2')}
    paths.update({d.name:d for d in (s.ROOT/'model_runs').glob('*') if d.is_dir()})
    out={}
    for name,folder in paths.items():
        measurements=folder/'MEASUREMENTS.json'
        if not measurements.exists():continue
        m=h.read(measurements);h.verify_record(m)
        assert h.file_hash(folder/'HISTORY.json')==m['history']
        rows=h.read(folder/'HISTORY.json')['rows']
        out[name]=dict(history=m['history'],windows=summarize(rows))
    result=h.seal(dict(schema='SteadyDimensionalLoadDecomposition/1',qualifying=False,units=dict(force='N',moment='N m'),runs=out))
    h.durable_json(s.ROOT/'LOAD_DECOMPOSITION.json',result);print(result['identity'])


if __name__=='__main__':run()
