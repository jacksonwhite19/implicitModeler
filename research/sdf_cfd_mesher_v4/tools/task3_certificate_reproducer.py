"""Preserve and replay the exact Task-3 certificate stop; no further search."""
from pathlib import Path
import sys,time,json,traceback,platform
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
from task3_study import load,components,write,pair_evidence,CONFIG
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import *
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import CertificateSession
import numpy as np,psutil,sympy

def reproduce(context,field_index,radius):
    front,top,sdf,schedule,coords,normals,*_=context;spec=components()[1];patch=build_patch(top,spec['columns'])
    fields,radii=search_fields(front,top,patch,coords[0],schedule.targets[-1])
    trial,projection=project_tangent_move(coords[0,patch.active],fields[field_index][1],radius*radii,sdf,schedule.targets[0])
    cache=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=cache.forecast(patch.active,trial)
    dc=DetailedCache();baseline=dc.get(front,cache.baseline,top,patch.detailed);candidate=dc.get(front,future,top,patch.detailed)
    surfaces=[];patches=[]
    for edge,k in [(1,1),(729,10)]:
        a,b=map(int,top.edges[edge]);lower=front.coordinates[[a,b]] if k==1 else future.vertex['coordinates'][k-2,[a,b]];upper=future.vertex['coordinates'][k-1,[a,b]]
        p=np.concatenate((lower,upper));ids=((k-1,a),(k-1,b),(k,a),(k,b));bp=kernel.Patch.binary(p,ids);patches.append(bp)
        valid,reason=bp.valid();q=sdf.evaluate(p)
        surfaces.append(dict(edge=edge,layer=k,labels=ids,vertices=[a,b],columns=top.edge_triangles.row(edge),points=p,
                             stable_source_ids=[front.vertex_ids[a],front.vertex_ids[b]],rank=bp.rank,valid=valid,validity_reason=reason,phi=q.phi,source_error_bound=q.source_error_bound))
    try:
        result=CertificateSession().pair(*patches)
    except ArithmeticError as error:
        result=dict(classification='ExactCertificateEvaluationUnavailable',exception_type=type(error).__name__,message=str(error))
    return plain(dict(component=spec,patch=patch.record(),field=fields[field_index][0],field_index=field_index,radius=radius,projection=projection,trial_coordinates=trial,
                      accepted=False,baseline_candidate_gate=numeric_gates(baseline,candidate_only=True),candidate_gate=numeric_gates(candidate,candidate_only=True),
                      baseline_tails=sorted_tails(baseline),trial_tails=sorted_tails(candidate),baseline_metrics=baseline,trial_metrics=candidate,
                      target_before=target_status(baseline,spec['columns'],spec['metric']),target_after=target_status(candidate,spec['columns'],spec['metric']),
                      new_earlier_failures=no_new_earlier_failure(failure_times(baseline),failure_times(candidate)),
                      baseline_first_future_invalid=min(failure_times(baseline).values(),default=None),trial_first_future_invalid=min(failure_times(candidate).values(),default=None),
                      changed_future_coordinates=future.local['coordinates'],surfaces=surfaces,result=result,
                      explanation='Unchanged production certificate failed on a modified future side pair. This trial was never accepted; current numeric violations also remain. No removal or separation claim can be made.'))

if __name__=='__main__':
    started=time.perf_counter();context=load();prefix=[];first=None
    # Replay the interrupted component's original deterministic poll order only
    # until the recorded exception. Do not continue past the stop.
    for round_id in range(CONFIG['radius_rounds']):
        for field_index in range(5):
            r=reproduce(context,field_index,2.**(-round_id));prefix.append(dict(field=r['field'],radius=r['radius'],certificate=r['result'],candidate_gate=r['candidate_gate']))
            if r['result']['classification']=='ExactCertificateEvaluationUnavailable':first=r;break
        if first is not None:break
    assert first is not None,'Task3ExactExceptionNotReproduced'
    write('INTERRUPTED_POLL_PREFIX.json',prefix);write('CERTIFICATE_FAILURE_REPRODUCER.json',first)
    second=reproduce(context,first['field_index'],first['radius']);h1,h2=scientific_hash(first),scientific_hash(second);assert h1==h2
    write('CERTIFICATE_FAILURE_REPLAY.json',dict(run1=h1,run2=h2,passed=True))
    write('STOP.json',dict(classification='V4LocalNextFrontRelaxationBlocked',component='component_01',stop_condition=9,
                           reason='Qualified exact certificate cannot evaluate the modified forecast side pair; ExpectedExactRealRoot',
                           accepted_candidates=0,committed_layers=0,remaining_search_not_executed=True,
                           python=platform.python_version(),sympy=sympy.__version__,seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().rss))
    print(json.dumps(dict(reproduced=True,hash=h1,valid_surfaces=[s['valid'] for s in first['surfaces']],ranks=[s['rank'] for s in first['surfaces']],candidate_gate=first['candidate_gate'],seconds=time.perf_counter()-started)))
