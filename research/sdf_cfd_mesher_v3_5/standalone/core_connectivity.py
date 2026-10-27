"""Bounded generic topology conditioning; never a live mesh publisher.

Reuse V4's existing cavity proposals and scientific predicates. This module
adds finite work limits and the existing direct bulk shape floor as a repair
objective. Every coordinate remains fixed. Retained results are diagnostic.
"""
from dataclasses import dataclass,asdict
from pathlib import Path
import copy,json,time
import numpy as np
from background_topology_repair import CoreTopology,key,cavity_geometry
from background_core_repair import PROPOSAL_REJECTIONS
from finite_support.transition_front_targets import tetrahedron_ratio
from finite_research.direct_shared_apex import exact_labels
from finite_research.direct_evidence_modules.collision import retained,verify_retained
from sdf_cfd_mesher_v4.common.authority import identity,array_hash

class RepairDeadline(TimeoutError):pass


@dataclass(frozen=True)
class Policy:
    max_candidates:int=512
    max_commits:int=8
    max_edge_cells:int=8
    max_cavity_cells:int=40
    seconds:float=1800.
    def __post_init__(self):
        if any(type(v) is not int or v<=0 for v in (self.max_candidates,self.max_commits,self.max_edge_cells,self.max_cavity_cells)):
            raise ValueError('InvalidConnectivityBudget')
        if not 3<=self.max_edge_cells<=8 or self.max_cavity_cells>40 or not np.isfinite(self.seconds) or self.seconds<=0:
            raise ValueError('InvalidConnectivityBudget')


def describe(cells):
    return [dict(c) for c in sorted(cells,key=lambda c:c['id'])]


class BoundedTopology(CoreTopology):
    def __init__(self,*args,policy,**kwargs):
        super().__init__(*args,**kwargs);self.policy=policy
    def edge_star(self,seed,edge,*,_expanded=False):
        if not _expanded:
            ids=super().edge_star(seed,edge,_expanded=False)
            return None if ids and len(ids)>self.policy.max_edge_cells else ids
        # Same closed-core walk as the reused primitive, with an early work
        # bound. An incomplete/oversized star never becomes a proposal.
        pending=[seed];seen=set();edge=set(edge)
        while pending:
            i=pending.pop()
            if i in seen:continue
            seen.add(i)
            if len(seen)>self.policy.max_edge_cells:return None
            for f in self.byid[i]['faces']:
                if edge<=set(f):
                    rows=self.owners[key(f)]
                    if len(rows)!=2 or any(j not in self.core_ids for j,_ in rows):return None
                    pending.extend(j for j,_ in rows if j not in seen)
        return sorted(seen)


def stage_commit(state,view):
    """Prepare a private successor; original state survives any exception."""
    staged=copy.copy(state);staged.byid=state.byid.copy();staged.owners=state.owners.copy();staged.core_ids=state.core_ids.copy()
    staged.commit(view)
    return staged


def repair(points,cells,owners,core_ids,far_faces,folder,*,minimum,policy=Policy(),check=lambda:None):
    """Return (private topology, report). No caller data are mutated."""
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    if type(minimum) not in (int,float) or not np.isfinite(minimum) or not 0<minimum<=1:raise ValueError('InvalidExistingShapeFloor')
    started=time.monotonic();deadline=started+policy.seconds
    original=np.asarray(points,dtype='<f8').copy();original.flags.writeable=False
    core_ids=tuple(core_ids)
    raw=array_hash(original);initial_cells=copy.deepcopy(cells);initial_owners=copy.deepcopy(owners)
    state=BoundedTopology(original,initial_cells,initial_owners,core_ids,far_faces,policy=policy)
    prefix=set(state.byid)-set(core_ids);prefix_stamp=identity(describe([state.byid[i] for i in prefix]))
    boundary_stamp=identity({str(k):rows for k,rows in sorted(state.owners.items()) if len(rows)==1})
    report=dict(schema='V35RetainedCoreConnectivity/1',policy=asdict(policy),minimum=minimum,
        coordinates_sha256=raw,original_cells_identity=identity(describe(cells)),history=[],commits=[],
        scientific_acceptance=False,live_parent=False,mesh_accepted=False,coordinate_changes=0)
    def save():
        report['seconds']=time.monotonic()-started
        (folder/'REPORT.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    def budget():
        check()
        if time.monotonic()>=deadline:raise RepairDeadline('ConnectivityRepairDeadline')
        if array_hash(original)!=raw:raise ValueError('ConnectivityCoordinatesChanged')
    def population(s):
        ids=sorted(s.core_ids);rows=np.asarray([s.byid[i]['vertices'] for i in ids],dtype='<i8')
        q=tetrahedron_ratio(original,rows)
        if not len(q) or not np.isfinite(q).all() or np.any(q<=0):raise ValueError('InvalidCoreShapePopulation')
        return ids,rows,q
    ids,rows,q=population(state)
    exact=exact_labels(original,rows,[minimum])[:,0]
    bad={i:float(v) for i,v,ok in zip(ids,q,exact) if v<minimum or not ok}
    report.update(initial_core_cells=len(ids),initial_bad_cells=[dict(id=i,shape=v) for i,v in sorted(bad.items())],initial_minimum=float(q.min()))
    np.savez_compressed(folder/'INPUT.npz',points=original,tetrahedra=rows,cell_ids=np.asarray(ids,dtype='<i8'))
    seen=set();outcome='AlreadyAboveFloor' if not bad else 'FiniteConnectivityFamilyRefusal'
    save()
    try:
      while bad:
        budget()
        if len(report['commits'])>=policy.max_commits:outcome='ConnectivityCommitBudget';break
        adopted=False
        for seed in sorted(bad,key=lambda i:(bad[i],i)):
            def proposals():
                yield from state.inventory(seed)
                yield from state.expanded_inventory(seed,budget)
            for proposal in proposals():
                budget()
                if len(report['history'])>=policy.max_candidates:outcome='ConnectivityCandidateBudget';break
                if 'generation_rejection' in proposal:
                    report['history'].append(dict(outcome=proposal['generation_rejection'],proposal=proposal));save();continue
                if len(proposal['remove'])>policy.max_cavity_cells:
                    report['history'].append(dict(outcome='CavityCellBudget',proposal=proposal));save();continue
                stamp=identity((len(report['commits']),state.stamp(proposal)))
                if stamp in seen:continue
                seen.add(stamp)
                row=dict(index=len(report['history']),state_version=len(report['commits']),owner_context=stamp,proposal=proposal)
                report['history'].append(row);save()
                if not set(proposal['remove'])&set(bad):raise ValueError('RepairProposalMissedCurrentDefect')
                try:view=state.view(proposal)
                except ValueError as error:
                    if not error.args or error.args[0] not in PROPOSAL_REJECTIONS:raise
                    row.update(outcome='CavityInvariantRefusal',reason=error.args[0]);save();continue
                added=np.asarray([c['vertices'] for c in view['added']],dtype='<i8')
                scores=tetrahedron_ratio(original,added);ok=exact_labels(original,added,[minimum])[:,0]
                row.update(new_minimum=float(scores.min()) if np.isfinite(scores).all() else None,new_cells=len(added))
                if not np.isfinite(scores).all() or np.any(scores<minimum) or not ok.all():row['outcome']='BulkShapeRefusal';save();continue
                qa=state.quality(view);row['quality']=qa
                if not qa['passed']:row['outcome']='ActualNeighbourQualityRefusal';save();continue
                budget();before=cavity_geometry(original,view['removed']);after=cavity_geometry(original,view['added'])
                if before['boundary_chain']!=after['boundary_chain']:raise ValueError('ChangedCertifiedCavityBoundary')
                staged=stage_commit(state,view)
                faces=[uses[0][1] for _,uses in sorted(staged.owners.items())]
                newkeys={key(f) for c in view['added'] for f in c['faces']}
                active=[j for j,f in enumerate(faces) if key(f) in newkeys]
                transaction=folder/('trial_%04d'%row['index']);transaction.mkdir()
                np.savez_compressed(transaction/'CANDIDATE.npz',removed=np.asarray(proposal['remove'],dtype='<i8'),inserted=added,
                    added_ids=np.asarray([c['id'] for c in view['added']],dtype='<i8'))
                certificate=retained(original,faces,active,budget,transaction/'contacts')
                row.update(before_geometry=before,after_geometry=after,changed_vs_all=certificate)
                if not certificate['passed'] or not certificate['independent_coverage_passed']:
                    row['outcome']='ChangedVsAllCollisionRefusal';save();continue
                verify_retained(certificate,transaction)
                budget()
                if identity(describe([staged.byid[i] for i in prefix]))!=prefix_stamp:raise ValueError('ChangedPrismPrefix')
                if identity({str(k):uses for k,uses in sorted(staged.owners.items()) if len(uses)==1})!=boundary_stamp:
                    raise ValueError('ChangedPhysicalBoundary')
                remaining={i:v for i,v in bad.items() if i not in set(proposal['remove'])}
                if len(remaining)>=len(bad):raise ValueError('NonreducingConnectivityEdit')
                row.update(outcome='Committed',bad_before=len(bad),bad_after=len(remaining))
                report['commits'].append(dict(index=row['index'],operation=proposal['operation'],removed=proposal['remove'],
                    added=[dict(id=c['id'],vertices=c['vertices']) for c in view['added']],before_geometry=before,after_geometry=after,
                    changed_vs_all=certificate))
                state=staged;bad=remaining;adopted=True;save();break
            if adopted or outcome=='ConnectivityCandidateBudget':break
        if not adopted:break
      if not bad:outcome='AllBulkCellsAboveFloor' if report['commits'] else 'AlreadyAboveFloor'
    except RepairDeadline as error:
        outcome='ConnectivityTimeBudget';report['budget_exception']=str(error)
    ids,rows,q=population(state)
    final_exact=exact_labels(original,rows,[minimum])[:,0]
    check();within=time.monotonic()<deadline
    if not within:outcome='ConnectivityTimeBudget'
    bad_final=[dict(id=i,shape=float(v)) for i,v,ok in zip(ids,q,final_exact) if v<minimum or not ok]
    report.update(outcome=outcome,final_core_cells=len(ids),final_minimum=float(q.min()),final_bad_cells=bad_final,
        all_bulk_above_floor=not bad_final,coordinates_byte_identical=array_hash(state.points)==raw,
        construction_passed=not bad_final and within,
        prefix_preserved=identity(describe([state.byid[i] for i in prefix]))==prefix_stamp)
    np.savez_compressed(folder/'RESULT.npz',points=state.points,tetrahedra=rows,cell_ids=np.asarray(ids,dtype='<i8'))
    save();return state,report
