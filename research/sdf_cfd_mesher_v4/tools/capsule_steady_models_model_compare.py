"""Deterministic common-signal comparison; no fitted acceptance threshold."""
import capsule_steady_models_model_analysis as a
import capsule_steady_models_loads as loads
s=a.s;h=s.h


def compare(write=True):
    selection=h.read(s.ROOT/'MESH_SELECTION.json');h.verify_record(selection)
    baseline=h.read(s.ROOT/'FINE_COMPARISON.json');h.verify_record(baseline)
    wake=h.read(s.ROOT/'FINE_DENSE_WAKE.json');h.verify_record(wake)
    runs={}
    for mesh in ('GW1-S1','GW1-S2'):
        folder=a.sa.mesh_source(mesh);m=h.read(folder/'MEASUREMENTS.json');h.verify_record(m)
        assert h.file_hash(folder/'HISTORY.json')==m['history']
        rows=h.read(folder/'HISTORY.json')['rows']
        r=baseline['meshes'][mesh]
        runs['SST-'+mesh]=dict(model='kOmegaSST',mesh=mesh,measurement=m['identity'],history=m['history'],windows=r['windows'],
            loads=loads.summarize(rows),summary={k:r[k] for k in ('linear_totals','bounds','flux_max','wall_max','maxiter','last')},
            wake=wake['variants'][mesh]['stations'])
        folder=s.ROOT/'model_runs'/('SA-'+mesh);m=h.read(folder/'MEASUREMENTS.json');h.verify_record(m)
        assert h.file_hash(folder/'HISTORY.json')==m['history']
        rows=h.read(folder/'HISTORY.json')['rows']
        runs['SA-'+mesh]=dict(model=m['model'],mesh=mesh,measurement=m['identity'],history=m['history'],windows=m['windows'],
            loads=loads.summarize(rows),summary=m['summary'],wake=m['wake']['stations'],checkpoints=m['checkpoints'])
    pairs={}
    for model in ('SST','SA'):
        pairs[model]={}
        for window in ('201-500','301-500','401-500','451-500'):
            x,y=[runs[model+'-'+n]['windows'][window] for n in ('GW1-S1','GW1-S2')]
            metrics={'drag':(x['signals']['Fz'],y['signals']['Fz'])}
            metrics.update({k:(x['lateral'][k]['magnitude'],y['lateral'][k]['magnitude']) for k in ('F','M')})
            pairs[model][window]={k:dict(means=[u['mean'],v['mean']],absolute_mean_difference=abs(u['mean']-v['mean']),ranges=[u['range'],v['range']],slopes=[u['slope'],v['slope']]) for k,(u,v) in metrics.items()}
    admissions={}
    for name in ('SA_ADMISSION','SSTLM_ADMISSION'):
        record=h.read(s.ROOT/(name+'.json'));h.verify_record(record);admissions[name]=record['identity']
    out=h.seal(dict(schema='SteadyAdmittedModelComparison/1',selection=selection['identity'],admissions=admissions,runs=runs,pairs=pairs,
        qualifying=False,spatial_convergence_established=False,model_accuracy_ranked=False,
        units=dict(force='N',moment='N m',velocity='m/s',pressure='kinematic p, m^2/s^2',nut='m^2/s',nuTilda='m^2/s'),
        residual_semantics='One initial residual per equation per outer iteration; U components separate; zero nonorthogonal corrections.',
        limitations=['All load means describe finite iterative histories, not a converged steady solution.',
                     'Terminal wake samples include iteration-phase and nearest-centroid sampling sensitivity.',
                     'nuTilda is not equivalent to k or omega; only common nut and physical fields are compared.']))
    if write:h.durable_json(s.ROOT/'MODEL_COMPARISON.json',out);print(out['identity'])
    return out


if __name__=='__main__':compare()
