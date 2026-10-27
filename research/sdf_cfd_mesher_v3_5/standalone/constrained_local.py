"""Transactional bounded local constrained cavities; no live publication."""
from pathlib import Path
from dataclasses import dataclass,asdict
from contextlib import ExitStack
from fractions import Fraction
import copy,json,time
import numpy as np
from background_topology_repair import CoreTopology,key,boundary,orient,cavity_geometry
from background_materialization import build
from core_connectivity import stage_commit,describe
from finite_support.transition_front_targets import tetrahedron_ratio
from finite_research.direct_shared_apex import exact_labels
from finite_research.direct_evidence_modules.collision import retained,verify_retained
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,file_hash
from prism_hotpath_v2 import reverse_cycle

def save(p,r):Path(p).write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')

@dataclass(frozen=True)
class Policy:
    rings:tuple=(1,2,3,4)
    point_rules:tuple=('existing','seed','spread')
    max_cavity_cells:int=256
    max_boundary_faces:int=512
    max_native_calls:int=12
    max_snapshots:int=24
    max_seeds:int=2
    max_commits:int=2
    max_new_points:int=128
    max_new_tets:int=4096
    native_seconds:int=360
    seconds:float=7200.
    def __post_init__(self):
        ints=(self.max_cavity_cells,self.max_boundary_faces,self.max_native_calls,self.max_snapshots,self.max_seeds,self.max_commits,self.max_new_points,self.max_new_tets,self.native_seconds)
        if any(type(v) is not int or v<=0 for v in ints) or not np.isfinite(self.seconds) or self.seconds<=0:raise ValueError('InvalidConstrainedBudget')
        if not self.rings or any(type(i) is not int or not 1<=i<=4 for i in self.rings) or tuple(sorted(set(self.rings)))!=self.rings:raise ValueError('InvalidCavityRings')
        if not self.point_rules or len(set(self.point_rules))!=len(self.point_rules) or not set(self.point_rules)<={'existing','seed','spread'}:raise ValueError('InvalidCavityPointRules')
        if self.max_cavity_cells>256 or self.max_boundary_faces>512 or self.max_native_calls>12 or self.max_snapshots>24 or self.max_seeds>2 or self.max_commits>2 or self.max_new_points>128 or self.max_new_tets>4096:raise ValueError('ConstrainedBudgetExpansion')
        if self.native_seconds>360 or self.seconds>7200:raise ValueError('ConstrainedTimeBudgetExpansion')

def population(s,minimum):
    ids=sorted(s.core_ids);rows=np.asarray([s.byid[i]['vertices'] for i in ids],dtype='<i8');q=tetrahedron_ratio(s.points,rows)
    if not len(q) or not np.isfinite(q).all() or np.any(q<=0):raise ValueError('InvalidRetainedBulkPopulation')
    labels=exact_labels(s.points,rows,[minimum])[:,0]
    return ids,rows,q,[dict(id=i,shape=float(v)) for i,v,ok in zip(ids,q,labels) if v<minimum or not ok]

def cavity_ball(s,seed,depth,policy):
    ids={seed}
    for _ in range(depth):
        ids|={i for j in tuple(ids) for f in s.byid[j]['faces'] for i,_ in s.owners[key(f)] if i in s.core_ids}
        if len(ids)>policy.max_cavity_cells:return None
    old=[s.byid[i] for i in sorted(ids)];_,faces,_=boundary(old)
    if len(faces)>policy.max_boundary_faces:return None
    return old,faces

def map_candidate(s,old,faces,points,tets,policy):
    """Map byte-exact fixed sites, then reuse exclusively internal point slots.

    Complete outside incidence guards every moved slot. No prefix/boundary site
    may move. Unrepresentable unused original slots cause a candidate refusal.
    """
    points=np.asarray(points,dtype='<f8');tets=np.asarray(tets,dtype='<i8')
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all() or tets.ndim!=2 or tets.shape[1]!=4 or not len(tets) or tets.min()<0 or tets.max()>=len(points):raise ValueError('InvalidConstrainedArrays')
    if len(tets)>policy.max_new_tets or len(points)>len({v for c in old for v in c['vertices']})+policy.max_new_points:raise ValueError('ConstrainedPopulationBudget')
    if len({p.tobytes() for p in points})!=len(points) or len(np.unique(tets))!=len(points):raise ValueError('DuplicateOrUnusedConstrainedPoint')
    remove={c['id'] for c in old};oldvs={v for c in old for v in c['vertices']};bvs={v for f in faces for v in f}
    interior=oldvs-bvs
    # Derive the complete vertex star from all current cells, including prisms.
    outside={v for i,c in s.byid.items() if i not in remove for v in c['vertices'] if v in interior}
    if outside:raise ValueError('IncompleteMovableVertexStar')
    fixed={v for i,c in s.byid.items() if i not in s.core_ids for v in c['vertices']}|{v for f in s.far for v in f}
    movable=interior-fixed
    lookup={p.tobytes():i for i,p in enumerate(s.points)}
    if len(lookup)!=len(s.points):raise ValueError('NonuniqueRetainedPoints')
    native={p.tobytes():i for i,p in enumerate(points)}
    if any(s.points[v].tobytes() not in native for v in oldvs-movable):raise ValueError('ChangedFixedCavityCoordinate')
    mapping=np.full(len(points),-1,dtype='<i8');new=[]
    for j,p in enumerate(points):
        v=lookup.get(p.tobytes())
        if v is None:new.append(j)
        elif v not in oldvs:raise ValueError('CandidateUsedOutsideVertex')
        else:mapping[j]=v
    used=set(map(int,mapping[mapping>=0]));slots=sorted(movable-used)
    if len(new)<len(slots):raise ValueError('StrandedOriginalInteriorPoint')
    if len(new)-len(slots)>policy.max_new_points:raise ValueError('ConstrainedPointBudget')
    xx=s.points.copy();moved=[];inserted=[]
    for j,v in zip(new,slots):mapping[j]=v;xx[v]=points[j];moved.append(v)
    for j in new[len(slots):]:mapping[j]=len(xx);inserted.append(len(xx));xx=np.vstack((xx,points[j]))
    changed=sorted(moved+inserted)
    if changed:
        # Exact degree one in the ORIGINAL cavity also refuses boundary points.
        from cavity_volume_audit import degree
        samples=[tuple(Fraction(float(v)) for v in xx[i]) for i in changed]
        membership=degree(s.points,faces,samples)
        if not membership['passed']:raise ValueError('NewPointNotStrictlyInOldCavity')
    else:membership=dict(passed=True,values=[],unresolved=[])
    return xx,mapping[tets],dict(moved_vertices=moved,inserted_vertices=inserted,strict_inclusion=membership)

class CavityTopology(CoreTopology):
    def view(self,proposal):
        """V4 actual-owner closure with explicitly authorized interior sites."""
        remove=set(proposal['remove'])
        if not remove or not remove<=self.core_ids:raise ValueError('InvalidCoreRemovalScope')
        old=[self.byid[i] for i in sorted(remove)];rows=orient(self.points,proposal['insert'])
        if len(set(map(key,rows)))!=len(rows):raise ValueError('DuplicateReplacement')
        new=build(np.asarray(rows,dtype='<i8'),self.next_id)
        if boundary(old)[0]!=boundary(new)[0]:raise ValueError('ChangedOrientedCavityBoundary')
        keys={key(f) for c in old+new for f in c['faces']};owners={k:[(i,f) for i,f in self.owners.get(k,[]) if i not in remove] for k in keys}
        for c in new:
            for f in c['faces']:owners[key(f)].append((c['id'],f))
        for k,uses in owners.items():
            if not uses:continue
            if len(uses)==1:
                if k not in self.far or len(self.owners.get(k,[]))!=1:raise ValueError('UnexpectedNewBoundaryFace')
            elif len(uses)!=2 or not reverse_cycle(uses[0][1],uses[1][1]):raise ValueError('ReplacementMustRetainActualOwners')
        lookup={c['id']:c for c in new};needed={i for uses in owners.values() for i,_ in uses}
        return dict(removed=old,added=new,owners=owners,cells=[lookup[i] if i in lookup else self.byid[i] for i in sorted(needed)],touched=keys)

def admit(s,old,faces,points,tets,folder,minimum,policy,check):
    """Private full-cavity transaction; scientific failures never mutate s."""
    xx,rows,mapping=map_candidate(s,old,faces,points,tets,policy)
    candidate=copy.copy(s);candidate.points=xx
    proposal=dict(operation='constrained_local_cavity',remove=[c['id'] for c in old],insert=rows.tolist())
    view=candidate.view(proposal);rows=np.asarray([c['vertices'] for c in view['added']],dtype='<i8');scores=tetrahedron_ratio(xx,rows)
    report=dict(point_mapping=mapping,new_minimum=float(scores.min()) if np.isfinite(scores).all() else None,new_cells=len(rows))
    if not np.isfinite(scores).all() or np.any(scores<minimum) or not exact_labels(xx,rows,[minimum])[:,0].all():return None,dict(report,outcome='BulkShapeRefusal')
    qa=candidate.quality(view);report['quality']=qa
    if not qa['passed']:return None,dict(report,outcome='ActualNeighbourQualityRefusal')
    check();before=cavity_geometry(s.points,old);after=cavity_geometry(xx,view['added'])
    if before['boundary_chain']!=after['boundary_chain'] or before['filling']['cell_volume']!=after['filling']['cell_volume']:raise ValueError('ChangedExactCavityFilling')
    staged=stage_commit(candidate,view)
    allfaces=[uses[0][1] for _,uses in sorted(staged.owners.items())];newkeys={key(f) for c in view['added'] for f in c['faces']}
    changed=set(mapping['moved_vertices'])|set(mapping['inserted_vertices'])
    active=[i for i,f in enumerate(allfaces) if key(f) in newkeys or changed.intersection(f)]
    folder=Path(folder);folder.mkdir()
    np.savez_compressed(folder/'CANDIDATE.npz',points=xx,inserted=rows,removed=np.asarray(proposal['remove'],dtype='<i8'))
    proof=retained(xx,allfaces,active,check,folder/'contacts');report.update(before_geometry=before,after_geometry=after,changed_vs_all=proof)
    if not proof['passed'] or not proof['independent_coverage_passed']:return None,dict(report,outcome='ChangedVsAllCollisionRefusal')
    verify_retained(proof,folder);check()
    oldboundary={str(k):uses for k,uses in sorted(s.owners.items()) if len(uses)==1}
    newboundary={str(k):uses for k,uses in sorted(staged.owners.items()) if len(uses)==1}
    if oldboundary!=newboundary:raise ValueError('ChangedGlobalPhysicalBoundary')
    prefix=[c for i,c in s.byid.items() if i not in s.core_ids]
    if identity(describe(prefix))!=identity(describe([staged.byid[c['id']] for c in prefix])):raise ValueError('ChangedPrismMetadata')
    fixed={v for c in prefix for v in c['vertices']}|{v for f in s.far for v in f}
    if xx[sorted(fixed)].tobytes()!=s.points[sorted(fixed)].tobytes():raise ValueError('ChangedFixedInterfaceCoordinates')
    return staged,dict(report,outcome='Committed',proposal=proposal,added_ids=[c['id'] for c in view['added']])

def native_candidate(s,old,faces,seed,rule,folder,policy,check):
    from native import operation
    from build_native import linux
    from immutable_state_access import guard
    from source_custody import selected
    from standalone_background import probe_backend
    folder.mkdir();vs=np.asarray(sorted({v for c in old for v in c['vertices']}),dtype='<i8')
    np.savez_compressed(folder/'INPUT.npz',points=s.points[vs],tetrahedra=np.searchsorted(vs,np.asarray([c['vertices'] for c in old])),
        boundary=np.searchsorted(vs,np.asarray(faces)),seed_tet=np.searchsorted(vs,np.asarray(s.byid[seed]['vertices'])))
    req=dict(input_sha256=file_hash(folder/'INPUT.npz'),point_rule=rule,max_tets=policy.max_new_tets,max_new_points=policy.max_new_points,
        source_files={p.relative_to(Path(__file__).parent).as_posix():file_hash(p) for p in selected(folder)},locks=probe_backend())
    save(folder/'INPUT.json',req);pin=file_hash(folder/'INPUT.json');root=Path(__file__).parent
    paths=':'.join(linux(root/p) for p in ('safety','v4/runtime/lib/finite_native_packages','v4/tools','v4/src'))
    with ExitStack() as held:
        for name in ('INPUT.npz','INPUT.json'):held.enter_context(guard(folder/name))
        check();operation('/usr/bin/env',['PYTHONPATH='+paths,'PYTHONDONTWRITEBYTECODE=1','python3','-B',linux(root/'constrained_local_worker.py'),linux(folder),pin],folder,seconds=min(policy.native_seconds,max(1,int(check.remaining_seconds()))),name='constrained_cavity')
        check()
        held.enter_context(guard(folder/'RESULT.json'));r=json.loads((folder/'RESULT.json').read_text())
        if not r['transport_passed'] or r['request_sha256']!=pin:raise ValueError('ConstrainedNativeTransportRefusal')
        for name,digest in r['files'].items():
            if Path(name).name!=name or file_hash(folder/name)!=digest:raise ValueError('ChangedConstrainedNativeArtifact')
            held.enter_context(guard(folder/name))
        for stage in r['stages']:
            if stage['stage'] not in ('generated','optimized') or r['files'].get(stage['stage']+'.npz')!=stage['sha256']:raise ValueError('UnboundConstrainedStage')
    return r

def repair(s,folder,*,minimum,policy=Policy(),check=lambda:None,proposer=native_candidate):
    if not np.isfinite(minimum) or not 0<minimum<=1:raise ValueError('InvalidExistingShapeFloor')
    folder=Path(folder);folder.mkdir();start=time.monotonic();deadline=start+policy.seconds
    def budget():
        check()
        if time.monotonic()>=deadline:raise TimeoutError('ConstrainedRepairTimeBudget')
    budget.remaining_seconds=lambda:max(0,deadline-time.monotonic())
    ids,rows,q,bad=population(s,minimum)
    report=dict(schema='V35ConstrainedLocalCavity/1',policy=asdict(policy),minimum=minimum,history=[],cavities=[],commits=[],
        initial_bad_cells=bad,initial_core_cells=len(ids),initial_minimum=float(q.min()),scientific_acceptance=False,live_parent=False)
    original_points=s.points.copy();prefix={i for i in s.byid if i not in s.core_ids};prefix_stamp=identity(describe([s.byid[i] for i in prefix]))
    native_calls=0;snapshots=0;exhausted=True;seen=set();attempted_seeds=set();outcome='ConstrainedFamilyRefusal'
    def persist():report['seconds']=time.monotonic()-start;save(folder/'REPORT.json',report)
    persist()
    try:
      if len(bad)>policy.max_seeds:outcome='ConstrainedSeedBudget';exhausted=False
      else:
       for defect in sorted(bad,key=lambda z:(z['shape'],z['id'])):
        seed=defect['id']
        if seed not in s.core_ids:continue
        attempted_seeds.add(seed)
        committed=False;seed_calls=0;seed_snapshots=0
        for depth in policy.rings:
            budget();cavity=cavity_ball(s,seed,depth,policy)
            if cavity is None:report['cavities'].append(dict(seed=seed,depth=depth,outcome='CompleteCavitySizeBudget'));exhausted=False;persist();continue
            old,faces=cavity;stamp=tuple(c['id'] for c in old)
            if stamp in seen:continue
            seen.add(stamp);report['cavities'].append(dict(seed=seed,depth=depth,cells=len(old),boundary_faces=len(faces),outcome='Constructed'))
            for rule in policy.point_rules:
                budget()
                if seed_calls>=policy.max_native_calls:raise TimeoutError('ConstrainedNativeCallBudget')
                trial=folder/('native_%02d'%native_calls);native_calls+=1;seed_calls+=1
                record=dict(seed=seed,depth=depth,point_rule=rule,native_folder=trial.name,snapshots=[]);report['history'].append(record);persist()
                native=proposer(s,old,faces,seed,rule,trial,policy,budget);record['native_result']=native
                for row in reversed(native['stages']):
                    budget()
                    if seed_snapshots>=policy.max_snapshots:raise TimeoutError('ConstrainedSnapshotBudget')
                    snapshots+=1;seed_snapshots+=1
                    if not row['intermediate_passed']:record['snapshots'].append(dict(stage=row['stage'],outcome='IntermediateInvariantRefusal',refusal=row.get('refusal')));persist();continue
                    from immutable_state_access import guard
                    saved=trial/(row['stage']+'.npz')
                    with guard(saved):
                        if file_hash(saved)!=row['sha256']:raise ValueError('ChangedConstrainedStageSnapshot')
                        with np.load(saved,allow_pickle=False) as z:xx=z['points'].copy();tt=z['tetrahedra'].copy()
                    try:candidate,receipt=admit(s,old,faces,xx,tt,trial/(row['stage']+'_transaction'),minimum,policy,budget)
                    except ValueError as error:
                        if str(error) not in {'ChangedOrientedCavityBoundary','InvalidOrientedCavityIncidence','NonmanifoldCavityBoundary','DegenerateReplacement','InvalidReplacementOrientation','DuplicateReplacement','UnexpectedNewBoundaryFace','ReplacementMustRetainActualOwners','StrandedOriginalInteriorPoint','NewPointNotStrictlyInOldCavity','DuplicateOrUnusedConstrainedPoint','ChangedFixedCavityCoordinate','CandidateUsedOutsideVertex','ConstrainedPopulationBudget','ConstrainedPointBudget'}:raise
                        candidate=None;receipt=dict(outcome='CavityCandidateRefusal',reason=str(error))
                    record['snapshots'].append(dict(stage=row['stage'],**receipt));persist()
                    if candidate is not None:
                        _,_,_,remaining=population(candidate,minimum)
                        if len(remaining)>=len(population(s,minimum)[3]):raise ValueError('NonreducingConstrainedEdit')
                        s=candidate;report['commits'].append(dict(seed=seed,depth=depth,point_rule=rule,stage=row['stage'],native_folder=trial.name,**receipt));committed=True;persist();break
                if committed:break
            if committed:break
        if not committed:break
        if len(report['commits'])>=policy.max_commits:
            if population(s,minimum)[3]:outcome='ConstrainedCommitBudget';exhausted=False
            break
    except TimeoutError as error:outcome=str(error);exhausted=False
    ids,rows,q,bad_final=population(s,minimum);budget_ok=time.monotonic()<deadline;check()
    if not budget_ok:outcome='ConstrainedRepairTimeBudget';exhausted=False
    if not bad_final and budget_ok:outcome='AllBulkCellsAboveFloor'
    report.update(outcome=outcome,construction_passed=not bad_final and budget_ok,final_bad_cells=bad_final,final_minimum=float(q.min()),final_core_cells=len(ids),
        native_calls=native_calls,evaluated_snapshots=snapshots,attempted_seeds=sorted(attempted_seeds),
        family_exhausted=bool(bad_final and exhausted and all(z['id'] in attempted_seeds for z in bad_final)),
        prefix_preserved=identity(describe([s.byid[i] for i in prefix]))==prefix_stamp,
        moved_original_vertices=np.flatnonzero(np.any(s.points[:len(original_points)].view('u8')!=original_points.view('u8'),axis=1)).tolist(),inserted_points=len(s.points)-len(original_points))
    np.savez_compressed(folder/'RESULT.npz',points=s.points,tetrahedra=rows,cell_ids=np.asarray(ids,dtype='<i8'));persist()
    return s,report
