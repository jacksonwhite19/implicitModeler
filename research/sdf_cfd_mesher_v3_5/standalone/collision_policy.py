"""Bounded proposal construction. Heuristic forecast never grants admission.

Fixed topology and X1; finite samples of V3's existing late taper family.
Exact checks use unchanged V4 mixed-face predicates, including incident pairs.
"""
from dataclasses import dataclass,asdict
from pathlib import Path
import hashlib,json,time
import numpy as np

def digest(*arrays):
    h=hashlib.sha256()
    for a in arrays:
        a=np.ascontiguousarray(a);h.update(str((a.dtype.str,a.shape)).encode());h.update(a.tobytes())
    return h.hexdigest()

def state_identity(state):
    metadata=json.dumps(dict(vertex_order=state.vertex_order,incidents=state.incidents,
        scalar_quantity=state.scalar_quantity,ownership=state.ownership,first_height=state.first_height),sort_keys=True)
    return hashlib.sha256((digest(state.wall,state.first,state.faces,state.normals,state.vectors)+metadata).encode()).hexdigest()

@dataclass(frozen=True)
class Policy:
    max_candidates:int=96
    max_seconds:float=900.
    preview_pairs:int=20000
    preview_seed_columns:int=32
    def __post_init__(self):
        if any(type(v) is not int or v<=0 for v in (self.max_candidates,self.preview_pairs,self.preview_seed_columns)) or not np.isfinite(self.max_seconds) or self.max_seconds<=0:raise ValueError('InvalidCollisionBudget')

def topology(faces,layers,n):
    """Same physical cycles and sorted unique keys as V4 shell construction."""
    owners={}
    for k in range(layers):
        for i,(a,b,c) in enumerate(faces):
            a,b,c=map(int,(a+k*n,b+k*n,c+k*n));A,B,C=a+n,b+n,c+n
            for f in ((c,b,a),(A,B,C),(a,b,B,A),(b,c,C,B),(c,a,A,C)):
                owners.setdefault(tuple(sorted(f)),[]).append((k+1,i,f))
    keys=sorted(owners)
    return [owners[k][0][2] for k in keys],[[[r[0],r[1]] for r in owners[k]] for k in keys]

def preview(state,h,policy):
    """All-row swept column boxes and centroid convergence; approximate risk.

    Boxes enclose the piecewise bilinear column boundary. Their overlap does
    not prove collision. Nonincident centroid convergence can miss collisions.
    No SDF value is converted to a Euclidean clearance or separation bound.
    """
    from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
    started=time.perf_counter();x=state.positions(h);tri=state.faces
    lo=np.full((len(tri),3),np.inf);hi=-lo
    for row in x:
        q=row[tri];lo=np.minimum(lo,q.min(1));hi=np.maximum(hi,q.max(1))
    index=AABBIndex(lo,hi);wall=x[0,tri].mean(1);tops=x[-1,tri].mean(1)
    risks=[];scanned=0;truncated=False
    for start in range(0,len(tri),256):
        ids=np.arange(start,min(start+256,len(tri)))
        for i,others in zip(ids,index.overlaps_batch(ids,after=True)):
            for j in others:
                scanned+=1
                if np.intersect1d(tri[i],tri[j]).size:continue
                before=float(np.linalg.norm(wall[i]-wall[j]));after=float(np.linalg.norm(tops[i]-tops[j]))
                if after<before:
                    risks.append(dict(columns=[int(i),int(j)],wall_centroid_gap_m=before,top_centroid_gap_m=after,convergence_m=before-after))
                if len(risks)>=policy.preview_pairs:truncated=True;break
            if truncated:break
        if truncated:break
    risks.sort(key=lambda r:(r['top_centroid_gap_m']/max(r['wall_centroid_gap_m'],1e-300),r['columns']))
    seeds=[]
    for r in risks:
        for v in r['columns']:
            if v not in seeds and len(seeds)<policy.preview_seed_columns:seeds.append(v)
    # Closest grid target suggested before any exact collision query.
    ratio=min((r['top_centroid_gap_m']/max(r['wall_centroid_gap_m'],1e-300) for r in risks),default=1.)
    grid=np.linspace(.05,1.,39)
    scale=float(grid[np.argmin(abs(grid-max(.05,min(.975,ratio))))])
    return dict(kind='HeuristicWholeStackSweptConvergence',guaranteed_prevention=False,
        certified_clearance=False,sdf_distance_assumed=False,rows=len(x)-1,scanned_pairs=scanned,
        truncated=truncated,risks=risks,seed_columns=seeds,suggested_core_scale=scale,
        seconds=time.perf_counter()-started)

def exact_check(points,faces,active):
    from optimized_mixed_certificate import certify
    return certify(points,faces,active,calculate_digest=False)

class SourceSegmentRefusal(ValueError):
    def __init__(self,detail):
        super().__init__('V35CollisionSourceSegmentRefusal');self.detail=detail

def segment_guard(source,h1):
    def check(x):
        rows=[]
        for k in range(1,len(x)):
            try:proof=source.qualify_segments(x[k-1],x[k],first_height_m=h1 if k==1 else None)
            except ValueError as error:
                from sdf_cfd_mesher_v4.sdf.regional import UnresolvedRegion
                diagnostic=getattr(error,'_outward_segment_diagnostic',{})
                reason=diagnostic.get('attempt',{}).get('reason')
                known_height=(type(error) is ValueError and error.args in (
                    ('UniqueSourceWallNormalUnresolved',),('TrueSourceFullNormalHeightUnresolved',)))
                known_progress=(type(error) is UnresolvedRegion and error.args==('SourceOutwardSegmentUnresolved',)
                    and reason in ('endpoint_positive_unresolved','depth_exhausted_progress_unresolved'))
                if not (known_height or known_progress):raise
                # Mutation/authentication remains fatal even when a geometric
                # predicate failed. Proof resource exhaustion is not recoverable.
                source.verify()
                raise SourceSegmentRefusal(dict(layer=k,type=type(error).__name__,args=list(error.args),diagnostic=diagnostic)) from error
            if proof.get('passed') is not True:raise SourceSegmentRefusal(dict(layer=k,reason='SourceReturnedFalse'))
            rows.append(dict(layer=k,passed=True,segments=len(x[k])))
        return dict(rows=rows,scope='Existing source progress/height predicates; no pair separation or distance bound')
    return check

def update_plan(planner,state,plan,h,collision):
    result=dict(plan);result.update(planner.audit(state,h))
    if not result['numerical_feasible']:raise ValueError('V35CollisionFinalNumericalRefusal')
    x=state.positions(h);delta=np.subtract(x[1:],x[:-1],dtype='<f8')
    result.update(schedule=h,positions=x,delta=delta,collision_construction={k:v for k,v in collision.items() if k not in ('witnesses','history','baseline','final_exact')})
    result['achieved']=dict(layers=h.shape[1],scalar_total_min_m=float(h.sum(1).min()),scalar_total_max_m=float(h.sum(1).max()),
        compressed_vertices=int(np.any(h<planner.request.nominal,axis=1).sum()),row2_increased_vertices=int((h[:,1]>planner.request.nominal[1]).sum()),
        path_totals_m=np.linalg.norm(delta,axis=2).sum(0),per_layer_min_m=h.min(0),per_layer_max_m=h.max(0),
        changed_from_numerical_plan_vertices=int(np.any(h!=plan['schedule'],axis=1).sum()))
    result['increment_guard']=planner.skew.increment_guard_audit(h,state.wall,state.faces,state.vectors[0],accepted_state=state)
    return result

class WitnessLedger:
    def __init__(self,faces,owners,state):
        self.faces=faces;self.owners=owners;self.state=state;self.rows=[];self.seen=set()
    def add(self,check,points,proposal):
        for bad in check['failures']:
            pair=bad['pair'];operands=[digest(points[list(self.faces[i])]) for i in pair]
            key=(tuple(pair),tuple(operands))
            if key in self.seen:continue
            self.seen.add(key)
            self.rows.append(dict(state_identity=state_identity(self.state),proposal=proposal,pair=list(map(int,pair)),operands=operands,
                faces=[dict(point_ids=list(map(int,self.faces[i])),
                    persistent_vertices=[str(self.state.vertex_order[v%len(self.state.wall)]) for v in self.faces[i]],
                    point_layers=[int(v//len(self.state.wall)) for v in self.faces[i]],
                    source_columns_and_layers=self.owners[i],
                    persistent_columns=[dict(layer=k,source_column=c,
                        key=sorted(str(self.state.vertex_order[v]) for v in self.state.faces[c])) for k,c in self.owners[i]]) for i in pair],
                classification=bad['classification'],method=bad['method'],detail=bad.get('detail')))
    def unchanged_forbidden(self,points):
        return [i for i,r in enumerate(self.rows) if all(digest(points[list(self.faces[f])])==h for f,h in zip(r['pair'],r['operands']))]
    def columns(self):
        return sorted({c for r in self.rows for f in r['faces'] for _,c in f['source_columns_and_layers']})

def finite_family(planner,state,h,seed_groups,forecast):
    """Enumerate existing 39-grid/ring taper targets; no collision bisection."""
    _,neighbors=planner.base.build_incidence(state.faces,len(state.wall))
    start=max(3,planner.request.layers-1)
    original=list(np.linspace(.05,1.,39));suggested=forecast['suggested_core_scale']
    # Fair finite traversal: preview target, then spread existing grid samples.
    # These are sample indices, never bisection of a collision predicate.
    order=[int(np.argmin(abs(np.asarray(original)-suggested))),38,19,0,29,9,34,24,14,4]+list(range(38,-1,-1))
    grid=[original[i] for i in dict.fromkeys(order)]
    radii=list(dict.fromkeys([1,2,4,8,12,16,20]+list(range(1,planner.base.MAX_TAPER_RINGS+1))))
    radii=[r for r in radii if r<=planner.base.MAX_TAPER_RINGS]
    combinations=sorted(((i,j) for i in range(len(grid)) for j in range(len(radii))),key=lambda ij:(sum(ij),ij[1],ij[0]))
    distances={}
    for i,j in combinations:
        scale,radius=grid[i],radii[j]
        for label,columns in seed_groups:
            if not columns:continue
            cache=(label,radius)
            if cache not in distances:
                seeds=set(map(int,state.faces[columns].ravel()))
                distances[cache]=planner.base.graph_distances(seeds,neighbors,radius)
            distance=distances[cache]
            factor=planner.base.scale_field(distance,radius,float(scale))
            q=h.copy();q[:,start-1:]=np.minimum(q[:,start-1:],planner.request.nominal[None,start-1:]*factor[:,None])
            yield q,dict(seed_kind=label,seed_columns=columns,radius=radius,core_scale=float(scale),start_layer=start)

def feedback_family(planner,state,h,ledger,forecast):
    """New witnesses expand support, retaining the same fixed finite family."""
    while True:
        columns=ledger.columns()
        groups=[('certified_both_sides',columns),('anticipatory_preview',forecast['seed_columns'])]
        for proposal,description in finite_family(planner,state,h,groups,forecast):
            yield proposal,description
            if ledger.columns()!=columns:break
        else:return

def construct(planner,state,h,folder,*,policy=Policy(),source_guard=None,source_capabilities=None,expected_baseline=None,candidate_guard=None):
    """Return a proposal only. Final V4 prepare remains independent/mandatory."""
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False);tick=time.perf_counter()
    h=np.asarray(h,dtype='<f8').copy();base_x=state.positions(h).copy();n=len(state.wall)
    semantic=dict(request=planner.request.nominal.tolist(),source=source_capabilities)
    binding=hashlib.sha256((state_identity(state)+json.dumps(semantic,sort_keys=True)).encode()).hexdigest()
    def current_binding():
        return hashlib.sha256((state_identity(state)+json.dumps(dict(request=planner.request.nominal.tolist(),source=source_capabilities),sort_keys=True)).encode()).hexdigest()
    faces,owners=topology(state.faces,planner.request.layers,n)
    numerical=planner.audit(state,h)
    from planner import jsonable
    (folder/'BASELINE_NUMERICAL.json').write_text(json.dumps(numerical,indent=2,default=jsonable))
    if not numerical['numerical_feasible']:raise ValueError('V35CollisionRequiresNumericallyFeasibleBaseline')
    if candidate_guard is not None and not candidate_guard(h):raise ValueError('V35CollisionBaselineContractRefusal')
    source_preview=None
    if source_guard is not None:
        t=time.perf_counter();source_preview=source_guard(base_x)
        source_preview=dict(source_preview,seconds=time.perf_counter()-t)
    forecast=preview(state,h,policy)
    forecast['source_segment_preview']=source_preview
    anticipated=next(finite_family(planner,state,h,[('anticipatory_preview',forecast['seed_columns'])],forecast),None)
    if anticipated is not None:
        cap,description=anticipated
        np.savez_compressed(folder/'PREVIEW_TARGETS.npz',baseline=h,suggested_caps=cap)
        forecast['anticipatory_target']=dict(description,changed_vertices=int(np.any(cap!=h,axis=1).sum()),
            total_min_m=float(cap.sum(1).min()),total_max_m=float(cap.sum(1).max()),
            admission='Suggestion only; exact and numerical gates still required')
    (folder/'PREVIEW.json').write_text(json.dumps(forecast,indent=2))
    if current_binding()!=binding:raise ValueError('CollisionStateChanged')
    points=base_x.reshape(-1,3);baseline_tick=time.perf_counter();baseline=exact_check(points,faces,range(len(faces)))
    baseline_seconds=time.perf_counter()-baseline_tick
    ledger=WitnessLedger(faces,owners,state);ledger.add(baseline,points,digest(h))
    history=[];seen={digest(h)};result_h=h;outcome='BaselineCollisionFree';final=None
    baseline_summary={k:v for k,v in baseline.items() if k!='proofs'}
    (folder/'BASELINE.json').write_text(json.dumps(baseline_summary,indent=2))
    if expected_baseline is not None:
        actual=dict(required_pairs=baseline['required_pairs'],failure_pairs=sorted([list(map(int,r['pair'])) for r in baseline['failures']]))
        if actual!=expected_baseline:raise ValueError('CollisionBaselineSignatureMismatch')
    def checkpoint():
        (folder/'HISTORY.json').write_text(json.dumps(dict(input_identity=binding,history=history,witnesses=ledger.rows),indent=2))
    checkpoint()
    if not baseline['passed']:
        outcome='CollisionFamilyExhausted'
        for q,description in feedback_family(planner,state,h,ledger,forecast):
            if len(history)>=policy.max_candidates or time.perf_counter()-tick>=policy.max_seconds:
                outcome='CollisionBudgetExhausted';break
            key=digest(q)
            if key in seen:continue
            seen.add(key);row=dict(index=len(history),proposal=key,**description);history.append(row)
            checkpoint()
            # Save each attempted operand before any check can refuse/interrupt.
            np.savez_compressed(folder/'LAST_CANDIDATE.npz',schedule=q)
            if current_binding()!=binding:raise ValueError('CollisionStateChanged')
            if not np.array_equal(q[:,:max(3,planner.request.layers-1)-1],h[:,:max(3,planner.request.layers-1)-1]) or np.any(q>h) or np.any(q<=0):raise ValueError('CollisionPolicyBoundsViolation')
            x=state.positions(q);p=x.reshape(-1,3)
            unchanged=ledger.unchanged_forbidden(p)
            if unchanged:row.update(outcome='UnchangedKnownForbidden',witness_ids=unchanged);checkpoint();continue
            changed=np.flatnonzero(np.any(x!=base_x,axis=(0,2)))
            active=[i for i,f in enumerate(faces) if p[list(f)].tobytes()!=points[list(f)].tobytes()]
            # Exact changed-vs-ALL includes shared vertices, neighboring cells,
            # all caps and cross-row interactions. Endpoint admission, no swept
            # motion is inserted; proposal changes are never physical movement.
            row.update(changed_vertices=changed.tolist(),active_faces=len(active))
            t=time.perf_counter();audit=planner.audit(state,q)
            row.update(numerical_seconds=time.perf_counter()-t,numerical_feasible=audit['numerical_feasible'])
            if not audit['numerical_feasible']:row['outcome']='NumericalRefusal';checkpoint();continue
            if candidate_guard is not None and not candidate_guard(q):
                row['outcome']='CandidateContractRefusal';checkpoint();continue
            t=time.perf_counter();check=exact_check(p,faces,active)
            row.update(exact_seconds=time.perf_counter()-t,required_pairs=check['required_pairs'],failures=len(check['failures']))
            ledger.add(check,p,key)
            if not check['passed']:row['outcome']='ChangedGeometryCollisionRefusal';checkpoint();continue
            if source_guard is not None:
                t=time.perf_counter()
                try:source_guard(x)
                except SourceSegmentRefusal as error:
                    row.update(outcome='SourceSegmentRefusal',source_seconds=time.perf_counter()-t,source_refusal=error.detail);checkpoint();continue
                row['source_seconds']=time.perf_counter()-t
            # Recompute ALL relations independently; no proof reuse admits shell.
            t=time.perf_counter();final=exact_check(p,faces,range(len(faces)))
            row.update(final_full_seconds=time.perf_counter()-t,final_full_pairs=final['required_pairs'],final_full_failures=len(final['failures']))
            ledger.add(final,p,key)
            if not final['passed']:row['outcome']='IndependentFullCollisionRefusal';checkpoint();continue
            row['outcome']='CollisionCorrected';result_h=q;outcome='CollisionCorrected';checkpoint();break
    if current_binding()!=binding:raise ValueError('CollisionStateChanged')
    report=dict(schema='V35CollisionConstruction/1',outcome=outcome,baseline_exact_seconds=baseline_seconds,
        proposal_collision_passed=outcome in ('BaselineCollisionFree','CollisionCorrected'),
        mesh_accepted=False,source_global_proof='pending_independent_V4_prepare',
        input_identity=binding,baseline_proposal=digest(h),final_proposal=digest(result_h),
        policy=asdict(policy),compression_start_layer=max(3,planner.request.layers-1),
        source_capabilities=source_capabilities or dict(live_source=False,clearance_bound=None),
        baseline=baseline_summary,history=history,witnesses=ledger.rows,
        final_exact=None if final is None else {k:v for k,v in final.items() if k!='proofs'},
        seconds=time.perf_counter()-tick)
    (folder/'COLLISION.json').write_text(json.dumps(report,indent=2))
    np.savez_compressed(folder/'TARGETS.npz',requested=planner.request.nominal,baseline=h,proposed=result_h,positions=state.positions(result_h))
    return result_h,report
