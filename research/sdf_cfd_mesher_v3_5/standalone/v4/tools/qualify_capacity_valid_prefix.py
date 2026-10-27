"""Read-only production-kernel rolling qualification of frozen fixture cores."""
from qualify_complete_capacity_fixtures import *
from sdf_cfd_mesher_v4.forecast.capacity import schedule_demand
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_wedge
from sdf_cfd_mesher_v4.advancing_front.rolling_correction import bounded_patch,correction_fields,controlling_support
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import project_tangent_move
from sdf_cfd_mesher_v4.forecast.detailed.metrics import cell_metrics
from sdf_cfd_mesher_v4.runtime.frozen_imports import legacy_modules
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import CELL_LIMITS,FACE_LIMITS
OUT=ROOT/'results/layer_capacity_valid_prefix'

def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(plain(value),sort_keys=True,indent=2,allow_nan=False)+'\n',newline='\n');tmp.replace(path)

def load_checkpoint(folder,layer,topology_hash):
    record=json.loads((folder/f'checkpoint_{layer:02}.json').read_text())
    points=np.load(folder/f'front_{layer:02}.npy',allow_pickle=False)
    if record['layer']!=layer or record['topology_hash']!=topology_hash or record['coordinate_hash']!=array_hash(points):raise ValueError('CheckpointIdentityMismatch')
    if not record['gate']['passed'] or not record['gate']['committed'] or not record['gate']['certificate']['passed']:raise ValueError('UncommittedCheckpoint')
    return points,record

def events_for(spec,s):
    if spec['capacity'] is not None:return []
    if spec['kind'] in ('wedge','frustum'):
        return [dict(layer=first_limit(s.targets,rational(spec.get('half_gap',spec.get('radius')))),metric='FixedStationOffsetDemand',scope='station not invariant under tangent motion')]
    events=[]
    for i,c in enumerate(spec['centres']):
        for j in range(i):
            gap=float(np.linalg.norm(np.array(c)-spec['centres'][j])-spec['radii'][i]-spec['radii'][j]);k=first_limit(s.targets,rational(gap)/2)
            if k is not None:events.append(dict(layer=k,metric='opposing_nominal_front_contact',components=[j,i]))
    return events

# Identical production primitive/order; optional conditioning is not a hard gate.
def immediate_metrics(top,low,high,layer,previous=None,batch=4096):
    started=time.perf_counter();cq=legacy_modules()['component_quality'];fg=cq.FG
    cells=cell_metrics(np.concatenate((low[top.triangles],high[top.triangles]),axis=1),np.full(len(top.triangles),layer),conditioning=False)
    owners=np.array([top.edge_triangles.row(e) for e in range(len(top.edges))])
    if owners.shape!=(len(top.edges),2):raise ValueError('IncompleteSurfaceOwnership')
    sides={k:[] for k in ('weight','skewness','symmetric_volume_ratio','nonorthogonality')}
    for begin in range(0,len(top.edges),batch):
        e=top.edges[begin:begin+batch];a,b=owners[begin:begin+batch].T
        vs=np.stack((low[e[:,0]],low[e[:,1]],high[e[:,1]],high[e[:,0]]),axis=1);S,C=fg.face_area_centre_quad(vs)
        values=cq.face_metrics(vs,S,C,cells['centre'][a],cells['centre'][b],cells['volume'][a],cells['volume'][b])
        for name,x in zip(sides,values):sides[name].append(x)
    sides={k:np.concatenate(v) for k,v in sides.items()};caps={k:np.empty(0) for k in sides}
    if layer>1:
        if previous is None:raise ValueError('MissingPriorCellGeometry')
        chunks={k:[] for k in sides}
        for begin in range(0,len(top.triangles),batch):
            ids=np.arange(begin,min(begin+batch,len(top.triangles)));vs=low[top.triangles[ids]];S,C=fg.face_area_centre_tri(vs)
            values=cq.face_metrics(vs,S,C,previous['centre'][ids],cells['centre'][ids],previous['volume'][ids],cells['volume'][ids])
            for name,x in zip(chunks,values):chunks[name].append(x)
        caps={k:np.concatenate(v) for k,v in chunks.items()}
    face_ids=[('side',layer,int(e),int(a),int(b)) for e,(a,b) in enumerate(owners)]
    if layer>1:face_ids.extend(('cap',layer-1,int(c),int(c),int(c)) for c in range(len(top.triangles)))
    gate=numeric_gates(dict(cells={k:v[None] for k,v in cells.items()},faces={k:np.r_[sides[k],caps[k]] for k in sides},face_ids=face_ids))
    events=[]
    for name,predicate in CELL_LIMITS.items():
        for col in np.flatnonzero(predicate(cells[name])|~np.isfinite(cells[name])):
            events.append(dict(kind='cell',metric=name,layer=layer,entity=int(col),columns=[int(col)],value=float(cells[name][col])))
    for kind,values in [('side',sides),('cap',caps)]:
        for name,predicate in FACE_LIMITS.items():
            for entity in np.flatnonzero(predicate(values[name])|~np.isfinite(values[name])):
                events.append(dict(kind=kind,metric=name,layer=layer,entity=int(entity),columns=owners[entity].tolist() if kind=='side' else [int(entity)],value=float(values[name][entity])))
    return dict(cells=cells,sides=sides,caps=caps,gate=gate,events=events,seconds=time.perf_counter()-started)

def validate(x,f,top,patch,prefix,high,previous):
    layer=len(prefix)+1;low=prefix[-1] if prefix else x;core=patch.core_columns
    exact=[exact_signs(np.concatenate((low[tri],high[tri]))) for tri in f[core]]
    bad=[dict(column=int(c),**exact_wedge(np.concatenate((low[f[c]],high[f[c]])))) for c,r in zip(core,exact) if r['jacobian_sign']<=0 or r['volume_sign']<=0]
    if bad:
        return dict(layer=layer,passed=False,failures=[],exact_failures=bad,exact_hash=identity(exact),coordinate_hash=array_hash(high),decisive_gate='PositiveExactJacobianOrVolume'),None,0.
    metric=immediate_metrics(top,low,high,layer,previous)
    sides=patch.side_edges
    faces={k:np.r_[v[sides],metric['caps'][k][core] if layer>1 else []] for k,v in metric['sides'].items()}
    ids=[('side',layer,int(e),0,0) for e in sides]+([('cap',layer-1,int(c),0,0) for c in core] if layer>1 else [])
    gate=numeric_gates(dict(cells={k:v[None,core] for k,v in metric['cells'].items()},faces=faces,face_ids=ids))
    exact=[exact_signs(np.concatenate((low[tri],high[tri]))) for tri in f[core]]
    failures=[e for e in metric['events'] if e['entity'] in (sides if e['kind']=='side' else core)]
    exact_bad=[dict(column=int(c),**r) for c,r in zip(core,exact) if r['jacobian_sign']<=0 or r['volume_sign']<=0]
    rec=dict(layer=layer,gate=gate,failures=failures,exact_failures=exact_bad,exact_hash=identity(exact),coordinate_hash=array_hash(high),passed=False)
    ct=0.
    if gate['passed'] and not exact_bad:
        t=time.perf_counter();cert,perf=certificate(x,top,patch,prefix+[high]);ct=time.perf_counter()-t;rec['certificate']=cert;rec['passed']=cert['passed']
    return rec,metric,ct


def projection_diagnostic(low,target,sdf,top,patch,x,f,prefix,previous,spec,do_probe):
    candidate=low.copy();failed=[]
    def split(ids):
        try:candidate[ids]=predict_next_front(low[ids],target,sdf).coordinates
        except (RuntimeError,ValueError):
            if len(ids)==1:failed.append(int(ids[0]))
            else:
                mid=len(ids)//2;split(ids[:mid]);split(ids[mid:])
    split(np.arange(len(low)))
    failed=np.array(sorted(failed),dtype=int)
    evidence=dict(failed_vertices=failed.tolist(),failed_core_vertices=np.intersect1d(failed,patch.core_vertices).tolist(),
                  failed_metric_halo_vertices=np.intersect1d(failed,patch.vertices).tolist(),
                  scope='Projection is an admission failure, not a measured prism quality value')
    probes=[];accepted=None;metric=None;ct=0.
    if do_probe:
        try:
            local=bounded_patch(top,failed);expanded=np.union1d(patch.core_columns,local.detailed.core_columns)
            _,vp,_=foundation(x,f,expanded,sdf,spec)
            field=np.zeros_like(low)
            for name,direction in correction_fields(SimpleNamespace(coordinates=low),top,local,field):
                for factor in (.5,1.,.25,.125,.0625,.03125,2.,4.):
                    row=dict(mode=name,factor=factor,active_vertices=len(failed),verification_columns=len(expanded),layer=len(prefix)+1)
                    try:
                        trial=candidate.copy();trial[local.active],_=project_tangent_move(low[local.active],direction,np.full(len(local.active),factor*(target-(0. if not prefix else float(load_schedule().targets[len(prefix)-1])))),sdf,target)
                        rec,m,c=validate(x,f,top,vp,prefix,trial,previous);ct+=c
                        row.update(passed=rec['passed'],coordinate_hash=array_hash(trial),failures=rec['failures'],exact_failures=rec['exact_failures'])
                        if rec['passed']:
                            accepted=(trial,rec);metric=m;probes.append(row);return evidence,probes,accepted,metric,ct
                    except (RuntimeError,ValueError,np.linalg.LinAlgError) as exc:row.update(passed=False,error=str(exc))
                    probes.append(row)
        except (RuntimeError,ValueError) as exc:probes.append(dict(status='QualifiedSupportUnavailable',error=str(exc),active_vertices=len(failed)))
    return evidence,probes,accepted,metric,ct

def run(persist=True):
    start=time.perf_counter();s=load_schedule();cf=CapacityForecast();results=[];timings=[]
    for spec in definitions(s.targets):
        tick=time.perf_counter();x,f,core=build(spec);sdf=FixtureSDF(spec);top,patch,base=foundation(x,f,core,sdf,spec)
        folder=OUT/spec['name'];prefix=[];previous=None;history=[];warnings=[];probes=[];ct=0.;st=0.;ft=0.
        for current in range(21):
            t=time.perf_counter();w=cf.analyze(spec['name'],s,current,source=spec['capacity'],events=events_for(spec,s),geometry_identity=array_hash(prefix[-1] if prefix else x),topology_identity=top.sha256);ft+=time.perf_counter()-t;warnings.append(w)
            if current==20:break
            layer=current+1;low=prefix[-1] if prefix else x
            try:
                with np.errstate(all='ignore'):prediction=predict_next_front(low,float(s.targets[current]),sdf)
                high=prediction.coordinates
            except (RuntimeError,ValueError) as exc:
                t=time.perf_counter()
                detail,poll,accepted,m,c=projection_diagnostic(low,float(s.targets[current]),sdf,top,patch,x,f,prefix,previous,spec,w['predicted_first_infeasible_layer'] is None)
                st+=time.perf_counter()-t;ct+=c;probes.extend(poll)
                if accepted is None:
                    history.append(dict(layer=layer,passed=False,committed=False,projection_failure=str(exc),failure_kind='SDFProjectionNotAdmitted',projection_diagnostic=detail))
                    break
                high,accepted_record=accepted

            try:
                rec,metric,cost=validate(x,f,top,patch,prefix,high,previous);ct+=cost
            except (RuntimeError,ValueError,np.linalg.LinAlgError) as exc:
                history.append(dict(layer=layer,passed=False,committed=False,validation_error=str(exc),failure_kind='ValidationEvaluationBlocked'));break
            if not rec['passed']:
                if persist:
                    folder.mkdir(parents=True,exist_ok=True);np.save(folder/f'uncommitted_{layer:02}.npy',high)
                # A diagnostic poll uses the qualified support builder and tangent
                # projection. A failed poll is never a proof of incapacity.
                geometric=w['predicted_first_infeasible_layer']
                if rec['failures'] and (geometric is None or layer<geometric):
                    t=time.perf_counter();front=SimpleNamespace(coordinates=low)
                    try:
                        regions,field,traces=controlling_support(front,top,high,rec['failures'])
                        vertices=np.unique(np.concatenate([f[e['columns']].ravel() for e in rec['failures']]))
                        local=bounded_patch(top,vertices)
                        # Require the full qualified verification closure, rather
                        # than moving a failure outside the scientific core.
                        expanded=np.union1d(core,local.detailed.core_columns)
                        _,vp,_=foundation(x,f,expanded,sdf,spec)
                        for name,direction in correction_fields(front,top,local,field):
                            for factor in (.5,1.,.25,.125,.0625,.03125,2.,4.):
                                trial=high.copy();row=dict(layer=layer,mode=name,factor=factor,active_vertices=len(local.active),verification_columns=len(expanded))
                                try:
                                    moved,_=project_tangent_move(high[local.active],direction,np.full(len(local.active),factor*s.increments[current]),sdf,float(s.targets[current]));trial[local.active]=moved
                                    r,m,c=validate(x,f,top,vp,prefix,trial,previous);ct+=c;row.update(passed=r['passed'],coordinate_hash=r['coordinate_hash'],failures=r['failures'])
                                    if r['passed']:
                                        row['admission']='DiagnosticImmediatePass';high=trial;rec=r;metric=m
                                except (RuntimeError,ValueError,np.linalg.LinAlgError) as exc:row.update(passed=False,error=str(exc))
                                probes.append(row)
                                if rec['passed']:break
                            if rec['passed']:break
                    except (RuntimeError,ValueError,AttributeError) as exc:probes.append(dict(layer=layer,error=str(exc),status='QualifiedSupportUnavailable'))
                    st+=time.perf_counter()-t
                if not rec['passed']:
                    rec['committed']=False;history.append(rec);break
            rec['committed']=True;history.append(rec);prefix.append(high);previous=metric['cells']
            if persist:
                folder.mkdir(parents=True,exist_ok=True);np.save(folder/f'front_{layer:02}.npy',high)
                write(folder/f'checkpoint_{layer:02}.json',dict(layer=layer,coordinate_hash=array_hash(high),topology_hash=top.sha256,gate=rec,capacity=w,scope=base['patch']))
        first=len(prefix)+1 if len(prefix)<20 else None;geometric=warnings[0]['predicted_first_infeasible_layer']
        onset=next((i for i,r in enumerate(warnings) if r['status'] in ('CapacityWarning','CapacityLimitPredicted')),None)
        terminal=history[-1] if first else None
        classification='CapacityClear' if first is None else ('QualityGateControlsFailure' if terminal.get('failures') and (geometric is None or first<geometric) else 'UnresolvedLimitMechanism')
        if first is not None and first==geometric and (terminal.get('projection_failure') or terminal.get('exact_failures') or terminal.get('certificate',{}).get('failures')):classification='GeometricLimitControlsFailure'
        if spec['capacity'] is None and spec['kind']=='spheres':classification='GeneralSDFWarningOnly'
        row=dict(fixture=spec['name'],foundation=base,history=history,witness_history=warnings,steering_probes=probes,
            last_valid_layer=len(prefix),first_invalid_or_unadmitted_layer=first,predicted_geometric_failure=geometric,warning_onset=onset,
            warning_lead=None if onset is None or first is None else first-onset,classification=classification,
            remaining=schedule_demand(s,len(prefix)),scope='Frozen scientific core and actual metric halo; closure-only cells are not qualified',
            observed_hard_failure=first if terminal and (terminal.get('failures') or terminal.get('exact_failures') or terminal.get('certificate',{}).get('failures')) else None)
        results.append(row);timings.append(dict(fixture=spec['name'],seconds=time.perf_counter()-tick,certificate_seconds=ct,steering_seconds=st,forecast_seconds=ft))
        if persist:write(folder/'result.json',row)
        print(spec['name'],len(prefix),first,classification,flush=True)
    return results,dict(seconds=time.perf_counter()-start,fixtures=timings,peak_rss=psutil.Process().memory_info().peak_wset)

if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True)
    a,t=run();write(OUT/'RESULTS.json',a);write(OUT/'TIMINGS.json',t)
    b,u=run(False);write(OUT/'REPLAY_TIMINGS.json',u);write(OUT/'DETERMINISM.json',dict(equal=identity(plain(a))==identity(plain(b)),scientific_hash=identity(plain(a))))
