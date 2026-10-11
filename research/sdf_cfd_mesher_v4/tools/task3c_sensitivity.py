"""Exact bottom-corner derivatives plus bounded projected +/- tangent probes."""
from task3_repair_study import *
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import *
from task3c_io import read as read_evidence,write as write_evidence
OUT=ROOT/'results/task3c'

def run(replay=False):
    start=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,*_=ctx
    audit=json.loads((OUT/'AUDIT.json').read_text());results=[]
    for a in audit:
        ids=np.asarray(a['closure_plan']['active']);p=active_patch(top,ids);rows=locked_corners(front,top,coords[0],p,normals[0]);assert not any(r['invariant_under_all_active_moves'] for r in rows)
        required=np.array(sorted({r['vertex'] for r in rows}));x=coords[0,required];b=tangent_basis(sdf.evaluate(x).normal)
        epsilon=schedule.targets[0]/32 # Smallest new preserved-height search poll, computational probe only.
        pre=np.stack([x+sg*epsilon*b[:,:,ax] for ax in (0,1) for sg in (1,-1)])
        proj=predict_next_front(pre.reshape(-1,3),schedule.targets[0],sdf);projected=proj.coordinates.reshape(4,len(x),3)
        traces=[];ranks={}
        for v in required:
            associated=[r for r in rows if r['vertex']==int(v)]
            ranks[int(v)]=exact_rank([[Q(z) for z in r['exact']['tangent_derivative']] for r in associated])
        for r in rows:
            j=int(np.searchsorted(required,r['vertex']));ids3=top.triangles[r['column']];values=[]
            for k in range(4):
                y=coords[0,ids3].copy();y[r['corner']]=projected[k,j]
                values.append(bottom_corner(front.coordinates[ids3],y,r['corner']))
            traces.append(dict(**r,projected_signed_jacobians=[v['jacobian'] for v in values],projected_signs=[v['sign'] for v in values],
                has_positive_projected_probe=any(v['sign']>0 for v in values)))
        r=dict(component=a['component'],active=len(p.active),baseline_nonpositive_bottom_corners=len(rows),
            required_vertices=required.tolist(),analytic_control_rank=sum(ranks.values()),vertex_block_ranks=ranks,
            all_corner_derivatives_nonzero=all(r['exact']['tangent_controls_corner'] for r in rows),
            all_corners_have_positive_projected_probe=all(r['has_positive_projected_probe'] for r in traces),
            epsilon=epsilon,probe_order=[[0,1],[0,-1],[1,1],[1,-1]],baseline_coordinates=x,pre_projection=pre,projected=projected,
            projection_residual=proj.residual.reshape(4,len(x)),projection_iterations=proj.iterations.reshape(4,len(x)),
            traces=traces,interpretation='Independent corner controllability. It does not establish a simultaneous hard-gate feasible candidate.')
        results.append(r);print(a['component'],'rank',r['analytic_control_rank'],'rows',len(rows),'positive probes',sum(t['has_positive_projected_probe'] for t in traces),flush=True)
    # Exact pair control is diagnostic even when other gates already reject.
    rr=read_evidence(OUT/'component_01_closed_support.json');ev=next(e for e in rr['history'] if e.get('field')=='outward_span_tangent')
    ids=np.array(rr['final']['patch']['active']);trial=np.array(ev['projection']['projected']);assert array_hash(trial)==ev['coordinate_hash']
    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=fc.forecast(ids,trial)
    pair=pair_evidence(front,top,future)
    paired=dict(baseline=ctx[-1],candidate=pair,candidate_coordinate_hash=array_hash(trial),candidate_gate=ev['candidate_gate'],
        first_future_invalid=ev['first_future_invalid'],new_earlier_failures=ev['new_earlier_failures'],
        statement='Known-pair result only; no claim of complete corrected-scope certification for a rejected numeric candidate.')
    evidence=dict(sensitivities=results,intersection=paired)
    h=scientific_hash(evidence)
    if replay:assert scientific_hash(read_evidence(OUT/'SENSITIVITY.json'))==h
    else:write_evidence(OUT/'SENSITIVITY.json',evidence)
    (OUT/('SENSITIVITY_REPLAY.json' if replay else 'SENSITIVITY_HASH.json')).write_text(json.dumps(dict(scientific_hash=h),indent=2)+'\n')
    (OUT/('SENSITIVITY_REPLAY_TIMINGS.json' if replay else 'SENSITIVITY_TIMINGS.json')).write_text(json.dumps(dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)
