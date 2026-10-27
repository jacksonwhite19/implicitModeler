"""Bounded non-terminating study. Never imports a rolling build entry point."""
from pathlib import Path
import sys,time,json,hashlib,os,copy
os.environ['OPENBLAS_NUM_THREADS']='1'
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import psutil
from types import SimpleNamespace
from fractions import Fraction
from task3c_io import read
from sdf_cfd_mesher_v4.forecast.capacity import *
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_signs
from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import project_tangent_move,tangent_basis
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import CertificateSession
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from capacity_fixtures import cohort,triangle,AnalyticSDF
OUT=ROOT/'results/layer_capacity_forecast'
def save(name,v): (OUT/name).write_text(json.dumps(v,sort_keys=True,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def experiment():
    start=time.perf_counter();s=load_schedule();c=CapacityForecast();sources=cohort(s.targets);records=[];checks=[];probes=[];timing={}
    tick=time.perf_counter();signals=[screen(s,0,f) for f in sources];timing['cheap_screen_seconds']=time.perf_counter()-tick
    session=CertificateSession(cache_capacity=128)
    tick=time.perf_counter()
    for f in sources:
        r=c.analyze(f['id'],s,0,source=f,geometry_identity=identity(f),topology_identity='fixture-triangle/1');records.append(r)
        assert c.analyze(f['id'],s,0,source=f,geometry_identity=identity(f),topology_identity='fixture-triangle/1')==r
        # Independent exact arithmetic reference from the source formula.
        if f['kind']=='parallel_gap':
            bad=[i+1 for i,t in enumerate(s.targets) if 2*Fraction(float(t))>=Fraction(f['spacing'])]
        elif f['kind'].endswith('cavity'):
            bad=[i+1 for i,t in enumerate(s.targets) if Fraction(float(t))>=Fraction(f['radius'])]
        else:bad=[i+1 for i,t in enumerate(s.targets) if 5*Fraction(float(t))>=3*Fraction(f['station'])]
        expected=bad[0] if bad else None; exact_first=None;proofs=[]
        if f['kind']=='parallel_gap':
            # Test every local layer pair through first contact; only two sides.
            h=f['spacing']/2
            for i,t in enumerate(s.targets):
                prev=0. if i==0 else s.targets[i-1]
                a=np.array([[-h+prev,0,0],[-h+prev,1,0],[-h+t,0,0],[-h+t,1,0]])
                b=np.array([[h-prev,0,0],[h-prev,1,0],[h-t,0,0],[h-t,1,0]])
                result=session.pair(kernel.Patch.binary(a,tuple(range(4))),kernel.Patch.binary(b,tuple(range(4,8))))
                proofs.append(dict(layer=i+1,result=result))
                if result['classification']=='CertifiedForbiddenIntersection':exact_first=i+1;break
        elif f['kind'].endswith('cavity'):
            for i,t in enumerate(s.targets):
                p=np.concatenate((triangle(f,0 if i==0 else s.targets[i-1]),triangle(f,t)))
                result=exact_signs(p);proofs.append(dict(layer=i+1,result=result))
                if result['jacobian_sign']<=0:exact_first=i+1;break
        # Beyond the analytic bound, these are invalid nominal continuations,
        # used only to expose exact failure; they are never SDF-admitted fronts.
        checks.append(dict(region=f['id'],analytic_first=expected,exact_geometry_first=exact_first,
                           nominal_station_only=f['kind']=='open_wedge',proofs=proofs,
                           predicted_first=r['predicted_first_infeasible_layer'],
                           offset=None if expected is None or r['predicted_first_infeasible_layer'] is None else r['predicted_first_infeasible_layer']-expected))
        if f['kind']!='open_wedge':assert exact_first==expected,(f,exact_first,expected)
        # Existing V4 tangent proposal plus authoritative fixture SDF projection.
        sdf=AnalyticSDF(f);base=triangle(f);basis=tangent_basis(sdf.evaluate(base).normal)
        target=float(s.targets[0]);bound,_,_=analytic_bound(f)
        if rational(target)>=bound:
            probes.append(dict(region=f['id'],status='NoFeasibleFirstTargetByAnalyticProof',trials=0));continue
        results=[]
        # Reuse the qualified nominal projector on the fixture's own SDF.
        nominal=base.copy();nominal_rows=[]
        for li,level in enumerate(s.targets,1):
            if f['kind']!='open_wedge' and rational(level)>=bound:break
            try:
                pred=predict_next_front(nominal,float(level),sdf)
                nominal_rows.append(dict(layer=li,coordinates=pred.coordinates.tolist(),residual=pred.residual.tolist(),
                                         step_lengths=np.linalg.norm(pred.displacement,axis=1).tolist()))
                nominal=pred.coordinates
            except RuntimeError as exc:
                nominal_rows.append(dict(layer=li,projection_failure=str(exc)));break
        for axis in (0,1):
            for sign in (-1,1):
                field=sign*basis[:,:,axis];moved,meta=project_tangent_move(base,field,np.full(3,target),sdf,target)
                results.append(dict(axis=axis,sign=sign,coordinates=moved.tolist(),residual=meta['residual'].tolist(),
                                    exact_immediate=exact_signs(np.concatenate((base,moved))),
                                    station_after=float(moved[0,0]) if f['kind']=='open_wedge' else None))
        probes.append(dict(region=f['id'],status='DiagnosticOnlyNotFullyHardGateQualified',trials=4,results=results,nominal_forecast=nominal_rows,
                           conclusion='Bound invariant under all tangential motion' if f['kind']!='open_wedge' else 'Station changes; no invariant local capacity established'))
    timing['detailed_fixture_seconds']=time.perf_counter()-tick
    tick=time.perf_counter();run=ROOT/'results/task4l/fresh';final=read(run/'FINAL_VALIDATION.json')
    assert final['layers']==20 and final['certificate']['complete'] and not final['certificate']['forbidden'] and not final['certificate']['unresolved']
    scan=read(run/'INITIAL_SCAN.json');front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
    events=[e for row in scan['records'] for e in row['events']]
    atlas=SimpleNamespace(events=events,layer=0,top=top,authority=top.sha256)
    selected=c.atlas_candidates(atlas);sf=[]
    # Region IDs derive from event column sets. No historical IDs select limits.
    cols=selected['columns']
    for col in cols:
        ev=[e for e in events if col in e['columns']]
        r=c.analyze('column:'+str(col),s,0,events=ev,geometry_identity=scan['forecast_coordinates'],topology_identity=top.sha256)
        observed=classify_steering(r,validated_trajectory=True,proof_identity=sha(run/'FINAL_VALIDATION.json.gz'))
        sf.append({k:observed[k] for k in ('region_id','status','predicted_first_infeasible_layer','scientific_input_identity','termination_authority')})
    # The map is an observed feasible control, not prospective proof for unknown SDFs.
    sf2=dict(records=sf,active_vertices=len(selected['vertices']),active_columns=len(cols),
             active_vertex_fraction=len(selected['vertices'])/len(front.coordinates),active_column_fraction=len(cols)/len(top.triangles),
             total_columns=len(top.triangles),proven_capacity_limits=sum(x['status']=='CapacityLimitPredicted' for x in sf),
             map=dict(default_status='ObservedQualifiedThrough20',columns=len(top.triangles),requested_layers=20,
                      qualification_identity=sha(run/'FINAL_VALIDATION.json.gz')),
             regression_scope='Preserved accepted trajectory proves feasibility; no new full forecast or builder run',
             warnings_before_steering=len(events),retrospective_control=True)
    histories=[]
    for layer in range(1,21):
        history=read(run/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'))
        histories.append(dict(layer=layer,accepted=sum(bool(x.get('accepted')) for x in history['history']),history_identity=identity(history)))
    sf2['steering_histories']=histories;timing['sf2_read_only_seconds']=time.perf_counter()-tick
    science=dict(fixtures=sources,records=records,exact_checks=checks,tangent_probes=probes,sf2=sf2,
                 cache=dict(hits=c.hits,misses=c.misses),exact_pair_calls=session.misses,
                 maps=[dict(region_id=x['region_id'],last=x['predicted_last_feasible_layer'],first=x['predicted_first_infeasible_layer'],status=x['status']) for x in records])
    timing['total_seconds']=time.perf_counter()-start
    timing['peak_rss']=psutil.Process().memory_info().peak_wset
    return science,timing

if __name__=='__main__':
    a,t=experiment();b,u=experiment();assert identity(a)==identity(b)
    save('SCIENTIFIC.json',a);save('TIMINGS.json',dict(first=t,replay=u));save('DETERMINISM.json',dict(equal=True,first=identity(a),replay=identity(b)))
    print(json.dumps(dict(hash=identity(a),times=t,rows=[{k:v for k,v in x.items() if k!='proofs'} for x in a['exact_checks']]),indent=2))
