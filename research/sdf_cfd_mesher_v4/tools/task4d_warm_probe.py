"""Transport prior runtime corrections; different candidates stay unaccepted."""
from task4d_common import *
from types import SimpleNamespace
import psutil
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner,support_identity
from sdf_cfd_mesher_v4.advancing_front.warm_start import warm_proposal
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.advancing_front.private_rolling import PrivateState,RollingForecastCache,immediate_numeric,exact_layer,future_failures
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import future_admission
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf

def run(name):
    f,t,frames=geometry();sdf=NativeSdf();schedule=load_schedule();rows=[];performance=[];arrays={}
    for layer in range(2,12):
        path=OUT/'sequence_2'/f'LAYER{layer-1}_PLANNER.npz'
        started=time.perf_counter();planner=PersistentPlanner();_,coords,normals=load_candidate(BASE/f'LAYER{layer-1}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        # Restore advisory state only. Do not load numerical/spatial caches from
        # the exploratory run into this independent transport diagnostic.
        if path.exists():
            with np.load(path,allow_pickle=False) as z:checkpoint=json.loads(str(z['metadata']))['checkpoint']
        else:checkpoint=next(r['checkpoint'] for r in read(OUT/'PLANNER_CHECKPOINTS.json') if r['directory']=='sequence_2' and r['layer']==layer-1)
        planner.restore(checkpoint,layer-1,array_hash(coords))
        history=read(BASE/f'LAYER{layer}_SEARCH.json');specs=[]
        # These are already verified generic supports in sequence_2, not a new
        # production selector. Frozen outputs provide comparison identities only.
        for outcome in history['outcomes']:specs.append(dict(vertices=outcome['scope']['active'],name=outcome['kind']))
        transitions=planner.begin(layer,specs,t.sha256)
        _,final_forecast,final_normals=load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        for i,(spec,event) in enumerate(zip(specs,transitions)):
            if i:
                prior_ids=np.asarray(specs[i-1]['vertices'],dtype=np.int64)
                coords[:,prior_ids]=final_forecast[:,prior_ids];normals[:,prior_ids]=final_normals[:,prior_ids]
            if event['kind']!='persistent' or support_identity(t.sha256,spec['vertices']) not in planner.states:continue
            patch=planner.patch(t,spec['vertices']);ids=patch.active;nominal=coords[layer-1,ids].copy()
            before=planner.detailed.get(f,SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords}),t,patch.detailed)
            failures=future_failures(before,layer);mechanism=min(failures,key=lambda k:(failures[k],k)) if failures else None
            known=[e for e in read(BASE/f'LAYER{layer-1}_WARNINGS.json') if e['kind']=='pair' and set(e['columns'])&set(map(int,patch.detailed.core_columns))]
            pair=min((e for e in known if e['metric']=='CertifiedForbiddenIntersection'),key=lambda e:(e['layer'],str(e['sides'])),default=None)
            proposal,record=warm_proposal(planner,t,patch,nominal,sdf,schedule.targets[layer-1],schedule.increments[layer-1],hazard=identity(dict(mechanism=mechanism,pair=pair)))
            row=dict(layer=layer,region=i,identity=event['region'],eligible=record['eligible'],projected=proposal is not None)
            if proposal is not None:
                arrays[f'layer{layer}_region{i}_coordinates']=proposal.copy()
                arrays[f'layer{layer}_region{i}_vertices']=ids.copy()
                arrays[f'layer{layer}_region{i}_residual']=record['projection']['residual'].copy()
                fc=RollingForecastCache(f,t,schedule,sdf,coords,normals);state=PrivateState(ids,tuple(x[ids] for x in frames[1:layer]))
                future=fc.forecast(state,proposal);detail=planner.detailed.get(f,future,t,patch.detailed)
                before=planner.detailed.get(f,SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords}),t,patch.detailed)
                gate=immediate_numeric(detail,layer);admission=future_admission(future_failures(before,layer),future_failures(detail,layer))
                accepted=next(x for x in history['history'] if x['patch']==i and x.get('accepted'))
                ex=exact_layer(f,t,future,patch.detailed,layer) if gate['passed'] else dict(passed=False,reason='NumericGateRejectedBeforeExactSigns')
                row.update(proposal_hash=array_hash(proposal),canonical_candidate_hash=accepted['hash'],same_canonical_candidate=array_hash(proposal)==accepted['hash'],
                    numeric=gate,exact_signs=ex,future_admission=admission,forecast_hash=array_hash(future.local['coordinates']),
                    production_accepted=False,continuous_certificate='Not run: advisory candidate is not eligible for publication in this equivalence study',
                    reset_reason='Ordinary canonical search retained',source_layer=record['source_layer'],source_mode=record['source_mode'],factor=record['factor'])
            rows.append(row)
        performance.append(dict(layer=layer,seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset))
        save(name+'.json',dict(rows=rows,scientific_hash=scientific_hash(rows)));save(name+'_TIMINGS.json',performance)
        np.savez_compressed(OUT/(name+'_CANDIDATES.npz'),**arrays)
        print('WARM PROBE',layer,len(rows),flush=True)

if __name__=='__main__':run(sys.argv[1])
