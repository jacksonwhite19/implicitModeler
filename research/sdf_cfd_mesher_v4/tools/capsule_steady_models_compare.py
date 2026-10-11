"""Same retained history and dense physical sampling for new steady SST pair."""
import json,sys
import capsule_steady_models as s
h=s.h

def wake():
    s.activate()
    import capsule_gw_wake as w
    w.g.PREVIOUS=s.OLD;w.f.ROOT=s.ROOT;w.run()

def compare(write=True):
    import capsule_pressure_diagnostic as p
    prior=h.read(s.OLD/'COMPARISON.json');h.verify_record(prior)
    meshes=prior['meshes']
    for n in s.NAMES:
        m=h.read(s.ROOT/n/'MEASUREMENTS.json');h.verify_record(m)
        assert h.file_hash(s.ROOT/n/'HISTORY.json')==m['history']
        rows=h.read(s.ROOT/n/'HISTORY.json')['rows'];assert [r['iteration'] for r in rows]==list(range(1,501))
        meshes[n]=dict(windows=p.windows(rows),last=rows[-1],linear_totals=m['W1']['summary']['linear_totals'],bounds=m['W1']['summary']['bounds'],flux_max=m['W1']['summary']['flux_max'],wall_max=m['W1']['summary']['wall_max'],maxiter=m['W1']['summary']['maxiter'],checkpoints=m['checkpoints'],wake_probes=m['wake_probes'])
    pairs={}
    for label,names in [('GW1',('GW1-S1','GW1-S2')),('GC2',s.NAMES)]:
        pairs[label]={}
        for window in ('201-500','301-500','401-500','451-500'):
            a,b=[meshes[n]['windows'][window] for n in names]
            # Evidence only. No new percentage gate or convergence threshold.
            metrics={'drag':(a['signals']['Fz'],b['signals']['Fz'])}
            for key in a['lateral']:metrics[key]=(a['lateral'][key]['magnitude'],b['lateral'][key]['magnitude'])
            pairs[label][window]={k:dict(seed_means=[x['mean'],y['mean']],absolute_mean_difference=abs(x['mean']-y['mean']),within_run_ranges=[x['range'],y['range']],slopes=[x['slope'],y['slope']]) for k,(x,y) in metrics.items()}
    out=h.seal(dict(schema='CapsuleSteadyGC2Comparison/1',meshes=meshes,pairs=pairs,prior=prior['identity'],family=h.read(s.ROOT/'FAMILY_CONTRACT.json')['identity'],steady=True,iterations=500,qualifying=False))
    if write:h.durable_json(s.ROOT/'COMPARISON.json',out);print(out['identity'])
    return out

if __name__=='__main__':globals()[sys.argv[1]]()
