"""Study-only evidence-driven cavity discovery across clean closed fixtures."""
import time,json,sys
import remesh_generalization_fixtures as fixtures
import cavity_volume_audit as volume
s=fixtures.s;t=s.t;q=s.q;np=s.np
OUT=s.ROOT/'results/local_termination_remesh_generalization'
def save(path,z):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(s.r.w.clean(z),sort_keys=True,separators=(',',':'))+'\n',encoding='utf8',newline='\n')

def validation(x,tri,fronts,levels,complete=True):
    times={};st=time.perf_counter();p,c,o=s.construct(x,tri,fronts,levels);times['cell_generation']=time.perf_counter()-st
    owners,own=t.ownership(c,o,[tuple(reversed(f)) for f in tri]);links=t.vertex_links(o)
    st=time.perf_counter();quality=t.quality(p,c,owners,1);times['hard_quality']=time.perf_counter()-st
    result=dict(ownership=own,vertex_links=links,quality=quality,passed=False)
    if not quality['passed'] or not own['passed'] or not links['passed']:return p,c,o,result,times
    if complete:
        st=time.perf_counter();faces=[owners[k][0][1] for k in sorted(owners)];cert=s.cert.certify(p,faces,range(len(faces)));times['exact_certificate']=time.perf_counter()-st
        st=time.perf_counter();v=volume.audit(p,c,o,[tuple(map(int,f)) for f in tri]);times['volume_accounting']=time.perf_counter()-st
        result.update(certificate=cert,volume=v,passed=cert['passed'] and v['interior_degree']['passed'] and v['exact_volume_balance'] and v['positive_cells'])
    return p,c,o,result,times

def failure_vertices(tri,quality,outer_certificate=None):
    columns=set()
    for e in quality['failures']:
        if 'column' in e:columns.add(e['column'])
        for owner in e.get('owners',[]):columns.add(owner%1000000)
    if outer_certificate:
        for e in outer_certificate['failures']:columns.update(e['pair'])
    return set(int(v) for i in columns for v in tri[i]),sorted(columns)

def coupled_failure_vertices(tri,cells,quality):
    """Closure cell IDs are temporary; use the actual shell ancestry map."""
    source={c['id']:c['column'] for c in cells};columns=set()
    for failure in quality['failures']:
        if 'column' in failure:columns.add(failure['column'])
        columns.update(source[k] for k in failure.get('owners',[]) if k in source)
    return {int(v) for column in columns for v in tri[column]}

def discover(x,tri,fronts,seed,first_layer,base_levels=None,candidate_check=None,progress=None,attempt_limit=None):
    """No fixture IDs, fixed depth or fixed ring count control the topology.

    Failed-cell dependencies expand laterally. If those are already inside,
    add their adjacent vertices; when no extension remains, try a deeper
    rollback above layer 1. Every attempt is validated without mutation.
    """
    top=q.build_topology(tri,len(x));active=set(seed);depth=first_layer-1;history=[];visited=set()
    while depth>=1:
        if attempt_limit is not None and len(visited)>=attempt_limit:
            history.append(dict(stop='OperationalAttemptBound',scientific_feasibility='Undetermined'))
            return None,history
        key=(depth,tuple(sorted(active)))
        if key in visited:break
        visited.add(key);levels=np.full(len(x),len(fronts)) if base_levels is None else np.array(base_levels,copy=True)
        selected=sorted(active)
        # Rollback never raises a terminated or lower local stack.
        levels[selected]=np.minimum(levels[selected],depth)
        if progress:progress('remesh_trial_start',trial=len(visited),depth=depth,support_vertices=len(active))
        p,c,o,checked,times=validation(x,tri,fronts,levels,False)
        if progress:progress('remesh_quality',trial=len(visited),times=times,failures=checked['quality']['failures'],
                             cells=len(c),cell_types=dict(t.Counter(z['kind'] for z in c)),outer_faces=len(o))
        row=dict(depth=depth,active=sorted(active),quality_passed=checked['quality']['passed'],failure_counts=t.Counter(z['kind'] for z in checked['quality']['failures']))
        extra,cols=failure_vertices(tri,checked['quality'])
        if progress and candidate_check is not None and checked['quality']['passed']:progress('scope_start',scope='coupled_validation',trial=len(visited))
        coupled=candidate_check(p,c,o) if candidate_check is not None and checked['quality']['passed'] else None
        if progress and coupled is not None:progress('scope_end',scope='coupled_validation',trial=len(visited),passed=coupled['passed'])
        if coupled is not None:
            row['complete_neighbor']=coupled
            extra|=coupled_failure_vertices(tri,c,coupled)
        if checked['quality']['passed']:
            if progress:progress('scope_start',scope='outer_certificate',trial=len(visited))
            oc=s.cert.certify(p,o,range(len(o)));row['outer_passed']=oc['passed'];row['outer_failures']=oc['failures']
            if progress:progress('scope_end',scope='outer_certificate',trial=len(visited),passed=oc['passed'])
            if oc['passed'] and checked['ownership']['passed'] and checked['vertex_links']['passed'] and (coupled is None or coupled['passed']):
                history.append(row);return levels,history
            vv,cc=failure_vertices(tri,checked['quality'],oc);extra|=vv
        history.append(row)
        if progress:progress('remesh_trial_end',trial=len(visited),depth=depth,quality_passed=row['quality_passed'],failure_counts=dict(row['failure_counts']))
        if extra-active:active|=extra;row['expansion_reason']='Measured failing cell/face dependencies'
        else:
            expanded=active|{int(v) for i in sorted(active) for v in top.vertex_neighbors.row(i)}
            if expanded!=active:active=expanded;row['expansion_reason']='Transition quality remains controlling; adjacent support required'
            else:depth-=1;active=set(seed);row['expansion_reason']='No lateral boundary remains; try earlier valid front'
        if len(active)==len(x):
            # A local study must not quietly become a whole-fixture remesh.
            history.append(dict(rejected_expansion='CavityReachedWholeFixture',active_vertices=len(active),next_depth=depth-1))
            depth-=1;active=set(seed)
    return None,history

def execute(spec):
    st=time.perf_counter();x,tri=fixtures.build(spec);top=q.build_topology(tri,len(x));fronts=[];scan=[];schedule=q.load_schedule()
    for k,target in enumerate(schedule.targets,1):
        front=fixtures.front(spec,target);fronts.append(front)
        p,c,o,check,times=validation(x,tri,fronts,[k]*len(x),False)
        sr=fixtures.sdf_record(spec,front,target)
        scan.append(dict(layer=k,quality_passed=check['quality']['passed'],sdf=sr))
        if not check['quality']['passed'] or not sr['admitted']:
            oc=s.cert.certify(p,o,range(len(o)));seed,columns=failure_vertices(tri,check['quality'],oc)
            # Exact geometric crossings and failing quality seed the cavity.
            # SDF residual alone does not manufacture a capacity claim.
            if not seed:raise ValueError('NoDirectInitiationEvidence')
            event=dict(layer=k,columns=columns,quality=check['quality'],outer=oc,sdf=sr,seed_vertices=sorted(seed));break
    else:raise ValueError('NoTerminationEventWithinRequestedSchedule')
    analysis_time=time.perf_counter()-st
    # Prove the entire immediate pre-event support clean before remeshing.
    pp,cc,oo,support,support_times=validation(x,tri,fronts[:-1],[k-1]*len(x))
    if not support['passed']:return dict(spec=spec,classification='SupportBlocked',support=support,scan=scan),dict(support=support_times)
    st=time.perf_counter();levels,expansion=discover(x,tri,fronts,seed,k);expansion_time=time.perf_counter()-st
    if levels is None:return dict(spec=spec,classification='TerminationRemeshInsufficient',failure='CavityDiscoveryFailure',support=support,event=event,expansion=expansion),dict(analysis=analysis_time,expansion=expansion_time,support=support_times)
    p,c,o,result,times=validation(x,tri,fronts,levels)
    vertical=k
    while not result['passed'] and result.get('certificate',{}).get('failures'):
        owners,_=t.ownership(c,o,[tuple(reversed(f)) for f in tri]);faces=[owners[key][0][1] for key in sorted(owners)]
        extra={int(v)%len(x) for bad in result['certificate']['failures'] for i in bad['pair'] for v in faces[i]}
        if extra<=seed:vertical-=1
        else:seed|=extra
        expansion.append(dict(expansion_reason='Complete internal-face certificate conflict',additional_vertices=sorted(extra),first_failure_layer=vertical))
        if vertical<=1:break
        levels,more=discover(x,tri,fronts,seed,vertical)
        expansion.extend(more)
        if levels is None:break
        p,c,o,result,more_times=validation(x,tri,fronts,levels)
        for key,value in more_times.items():times[key]=times.get(key,0.)+value
    if levels is None:return dict(spec=spec,classification='TerminationRemeshInsufficient',failure='CavityDiscoveryFailure',support=support,event=event,expansion=expansion),dict(analysis=analysis_time,expansion=expansion_time,support=support_times)
    _,old,_=s.construct(x,tri,fronts,[k]*len(x));manifest=s.removal_manifest(old,c)
    continuation=continue_local(spec,x,tri,fronts,levels,seed)
    record=dict(spec=spec,scan=scan,support=support,event=event,expansion=expansion,vertex_depth=levels.tolist(),manifest=manifest,
        cells=c,outer=o,outer_hash=q.identity(o),wall_hash=q.array_hash(x),first_hash=q.array_hash(fronts[0]),result=result,continuation=continuation,
        classification='TerminationRemeshQualified' if result['passed'] and continuation['passed'] else 'TerminationRemeshPromising',
        failure=None if result['passed'] and continuation['passed'] else ('NeighborContinuationFailure' if result['passed'] else 'ContinuousGeometryFailure'))
    ids=sorted({v for f in o for v in f});lookup={v:i+1 for i,v in enumerate(ids)}
    record['ingest_input']=dict(coordinates=p[ids].tolist(),triangles=[[lookup[v] for v in f] for f in o],outer_hash=q.identity(o))
    return record,dict(analysis=analysis_time,expansion=expansion_time,support=support_times,remesh_validation=times,continuation=continuation.pop('runtime'),peak_rss=q.psutil.Process().memory_info().peak_wset)

def continue_local(spec,x,tri,fronts,levels,seed,geometry=None,complete_validator=None,front_provider=None,eligible=None,progress=None,attempt_limit=None):
    geometry=geometry or fixtures
    # One immutable, hash-verified schedule snapshot per callback. Reopening,
    # hashing and scanning its full source array for every source column is
    # redundant and can stall before the first continuation trial begins.
    schedule_start=time.perf_counter()
    if progress:progress('scope_start',scope='continuation_schedule_load')
    schedule=q.load_schedule()
    if progress:progress('scope_end',scope='continuation_schedule_load',seconds=time.perf_counter()-schedule_start)
    st=time.perf_counter();k=len(fronts);top=q.build_topology(tri,len(x));distance={v:0 for v in seed};todo=list(sorted(seed))
    for v in todo:
        for j in top.vertex_neighbors.row(v):
            j=int(j)
            if j not in distance:distance[j]=distance[v]+1;todo.append(j)
    allowed=set(range(len(tri))) if eligible is None else set(map(int,eligible))
    choices=[]
    for i,f in enumerate(tri):
        depth=int(levels[f[0]])
        if i not in allowed or depth>=len(schedule.targets):continue
        if all(levels[v]==depth for v in f) and (front_provider is not None or depth==k):
            choices.append((max(distance.get(int(v),len(x)) for v in f),i,depth))
    choices.sort();attempts=[]
    if not choices and all(all(levels[v]>=len(schedule.targets) for v in tri[i]) for i in allowed):
        return dict(passed=True,attempts=[],runtime=time.perf_counter()-st,
                    scope='RequestedScheduleComplete',remaining_eligible_columns=0)
    base_points,base_cells,_=s.construct(x,tri,fronts,levels)
    for dist,col,depth in choices:
        if attempt_limit is not None and len(attempts)>=attempt_limit:
            return dict(passed=False,attempts=attempts,runtime=time.perf_counter()-st,
                        stop='OperationalContinuationAttemptBound',remaining_candidates=len(choices)-len(attempts),scientific_feasibility='Undetermined')
        if progress:progress('continuation_trial_start',column=col,layer=depth+1,radius=dist)
        target=schedule.targets[depth]
        try:
            nextfront=geometry.front(spec,target) if front_provider is None else front_provider(fronts[:depth],levels,target,tri[col])
        except (ValueError,RuntimeError) as exc:
            attempts.append(dict(column=col,layer=depth+1,radius=dist,passed=False,rejection=type(exc).__name__+':'+str(exc)));continue
        assigned=geometry.sdf_record(spec,nextfront[tri[col]],target)
        if not assigned['admitted']:
            attempts.append(dict(column=col,layer=depth+1,radius=dist,passed=False,rejection='AssignedSDFTargetNotReached',sdf=assigned));continue
        proposed=[a.copy() for a in fronts]
        if depth==len(proposed):proposed.append(nextfront)
        else:proposed[depth][tri[col]]=nextfront[tri[col]]
        trial=levels.copy();trial[tri[col]]=depth+1
        p,c,o,check,times=validation(x,tri,proposed,trial,False)
        row=dict(column=col,layer=depth+1,radius=dist,quality_passed=check['quality']['passed']);attempts.append(row)
        if progress:progress('continuation_quality',column=col,layer=depth+1,passed=check['quality']['passed'],failures=check['quality']['failures'],times=times)
        if not check['quality']['passed']:continue
        if front_provider is not None:
            by_id={z['id']:z for z in c}
            preserved=all(by_id.get(z['id'])==z and np.array_equal(p[z['vertices']],base_points[z['vertices']]) for z in base_cells)
            row['committed_cells_preserved']=preserved
            if not preserved:continue
        if complete_validator is None:p,c,o,check,times=validation(x,tri,proposed,trial,True)
        else:p,c,o,check,times=complete_validator(proposed,trial)
        row.update(result=check,passed=check['passed'],outer_hash=q.identity(o),new_cells=[z for z in c if z['layer']==depth+1])
        if check['passed']:return dict(passed=True,attempts=attempts,runtime=time.perf_counter()-st,front=nextfront.tolist(),fronts=[a.tolist() for a in proposed],levels=trial.tolist(),layer=depth+1)
    return dict(passed=False,attempts=attempts,runtime=time.perf_counter()-st)

if __name__=='__main__':
    spec=next(z for z in fixtures.definitions() if z['name']==sys.argv[1]);record,times=execute(spec)
    folder=OUT/spec['name']
    if '--replay' in sys.argv:
        first=s.load(folder/'result.json');assert q.identity(first)==q.identity(record)
        save(folder/'replay.json',dict(equal=True,scientific_hash=q.identity(record)));save(folder/'replay_runtime.json',times)
    else:
        save(folder/'result.json',record);save(folder/'runtime.json',times)
        if 'ingest_input' in record:save(folder/'CORE_INGEST_INPUT.json',record['ingest_input'])
    print(record['classification'],record.get('failure'),flush=True)
