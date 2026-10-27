"""Fine-level collection, preserving the prospectively bound Stage 1 evidence."""
import copy
import sys
import capsule_steady_models_fine as fine
s=fine.s;h=s.h

def wake():
    fine.activate()
    import capsule_gw_wake as w
    w.g.PREVIOUS=s.OLD;w.f.ROOT=s.ROOT
    original=h.durable_json
    def save(path,record):
        if path==s.ROOT/'DENSE_WAKE.json':
            prior=h.read(path);h.verify_record(prior)
            record=copy.deepcopy(record)
            record.pop('identity');record['variants'].update(prior['variants'])
            record=h.seal(record);path=s.ROOT/'FINE_DENSE_WAKE.json'
        return original(path,record)
    h.durable_json=save
    try:
        w.run()
        print('Combined fine wake identity:',h.read(s.ROOT/'FINE_DENSE_WAKE.json')['identity'])
    finally:h.durable_json=original

def compare(write=True):
    import capsule_pressure_diagnostic as p
    prior=h.read(s.ROOT/'COMPARISON.json');h.verify_record(prior)
    meshes=prior['meshes']
    for n in fine.NAMES:
        m=h.read(s.ROOT/n/'MEASUREMENTS.json');h.verify_record(m)
        assert h.file_hash(s.ROOT/n/'HISTORY.json')==m['history']
        rows=h.read(s.ROOT/n/'HISTORY.json')['rows']
        assert [r['iteration'] for r in rows]==list(range(1,501))
        meshes[n]=dict(windows=p.windows(rows),last=rows[-1],linear_totals=m['W1']['summary']['linear_totals'],bounds=m['W1']['summary']['bounds'],flux_max=m['W1']['summary']['flux_max'],wall_max=m['W1']['summary']['wall_max'],maxiter=m['W1']['summary']['maxiter'],checkpoints=m['checkpoints'],wake_probes=m['wake_probes'])
    out=h.seal(dict(schema='CapsuleSteadyGC2FComparison/1',meshes=meshes,prior=prior['identity'],family=h.read(s.ROOT/'FINE_FAMILY_CONTRACT.json')['identity'],steady=True,iterations=500,qualifying=False))
    if write:h.durable_json(s.ROOT/'FINE_COMPARISON.json',out);print(out['identity'])
    return out

if __name__=='__main__':globals()[sys.argv[1]]()
