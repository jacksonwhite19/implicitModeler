"""Normal-CLI bulk conditioning and deterministic edit custody.

The live caller owns its genuine terminal lease. These shared component
constructors only propose/validate private successors; they never issue leases.
No retained mesh, fixture identity or historical authorization is read here.
"""
from pathlib import Path
from dataclasses import asdict
from contextlib import contextmanager
from contextvars import ContextVar
import json,time
import numpy as np
from core_connectivity import repair as vertex_repair,Policy as VertexPolicy
from constrained_local import repair as cavity_repair,Policy as CavityPolicy,CavityTopology,population,admit
from background_materialization import build
from background_topology_repair import boundary
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,file_hash
from path_preserving_deduplication import durable_json as save

_replay_output=ContextVar('v35_conditioning_replay_output',default=None)

@contextmanager
def replay_output_scope(path):
    """Route replay diagnostics only; never changes admission predicates."""
    token=_replay_output.set(Path(path))
    try:yield
    finally:_replay_output.reset(token)

METHOD=dict(schema='V35BackgroundConditioning/1',vertex_policy=asdict(VertexPolicy()),cavity_policy=asdict(CavityPolicy()),
    order='Current deficient cells -> bounded existing-vertex family -> bounded constrained cavities',
    commit='Whole private passing transactions only; exact fixed interfaces and unchanged quality floors')

def core_arrays(s):
    ids=sorted(s.core_ids)
    return dict(points=s.points,tetrahedra=np.asarray([s.byid[i]['vertices'] for i in ids],dtype='<i8'),cell_ids=np.asarray(ids,dtype='<i8'))

def from_joined(joined):
    prefix=set(joined['prefix_ids'])
    return CavityTopology(joined['points'],joined['cells'],joined['owners'],[c['id'] for c in joined['cells'] if c['id'] not in prefix],joined['farfield'])

def conditioned_joined(joined,s):
    """Propagate the exact accepted point/cell/owner state as one successor."""
    prefix=set(joined['prefix_ids']);cells=[c for c in joined['cells'] if c['id'] in prefix]
    core=[s.byid[i] for i in sorted(s.core_ids)]
    return dict(joined,points=s.points,bulk_tets=np.asarray([c['vertices'] for c in core],dtype='<i8'),cells=cells+core,owners=s.owners,
        provenance=dict(joined['provenance']))

def apply(joined,folder,policy,check,*,native_budget=None,vertex_constructor=vertex_repair,cavity_constructor=cavity_repair):
    """Return a successor, or raise with the caller's joined state untouched."""
    folder=Path(folder);folder.mkdir();started=time.monotonic();s=from_joined(joined)
    minimum=policy['quality_targets']['outer_tetrahedra']['minimum'];check()
    ids,rows,q,bad=population(s,minimum)
    report=dict(method=METHOD,passed=False,initial_bad_cells=bad,initial_minimum=float(q.min()),initial_core_cells=len(ids),
        vertex_invoked=False,cavity_invoked=False,steps=[],native_calls=0,parent=joined['provenance'].get('binding'),
        live_parent=joined['provenance'].get('live_parent'),policy_identity=identity(policy),timings={})
    np.savez_compressed(folder/'INPUT.npz',**core_arrays(s))
    report['timings']['initial_population']=time.monotonic()-started
    try:
        if bad:
            report['vertex_invoked']=True;tick=time.monotonic()
            next_state,vr=vertex_constructor(s.points,list(s.byid.values()),s.owners,s.core_ids,joined['farfield'],folder/'existing_vertices',minimum=minimum,policy=VertexPolicy(),check=check)
            report['timings']['existing_vertices']=time.monotonic()-tick
            report['steps'].append(dict(kind='existing_vertices',outcome=vr['outcome'],commits=len(vr['commits']),report_sha256=file_hash(folder/'existing_vertices/REPORT.json')))
            if vr['outcome'] not in {'FiniteConnectivityFamilyRefusal','AllBulkCellsAboveFloor','AlreadyAboveFloor'}:raise ValueError('BackgroundExistingVertexBudgetOrRefusal',vr['outcome'])
            s=CavityTopology(next_state.points,list(next_state.byid.values()),next_state.owners,next_state.core_ids,joined['farfield'])
            _,_,_,remaining=population(s,minimum)
            if remaining:
                report['cavity_invoked']=True;tick=time.monotonic()
                from constrained_local import native_candidate
                def bounded_native(*args):
                    inner=args[-1]
                    def checked():
                        inner()
                        if native_budget is not None:native_budget.remaining()
                    checked.remaining_seconds=lambda:min(inner.remaining_seconds(),getattr(check,'remaining_seconds',lambda:float('inf'))(),
                        native_budget.remaining() if native_budget is not None else float('inf'))
                    call_started=native_budget.clock() if native_budget is not None else None
                    try:return native_candidate(*args[:-1],checked)
                    finally:
                        if native_budget is not None:native_budget.charge(call_started)
                s,cr=cavity_constructor(s,folder/'constrained',minimum=minimum,policy=CavityPolicy(),check=check,proposer=bounded_native)
                report['timings']['constrained']=time.monotonic()-tick;report['native_calls']=cr['native_calls']
                report['steps'].append(dict(kind='constrained',outcome=cr['outcome'],commits=len(cr['commits']),report_sha256=file_hash(folder/'constrained/REPORT.json')))
                if not cr['construction_passed']:raise ValueError('BackgroundConstrainedCavityRefusal',cr['outcome'])
        check();ids,rows,q,remaining=population(s,minimum)
        if remaining:raise ValueError('BackgroundConditioningFinalFloor',remaining)
        successor=conditioned_joined(joined,s)
        np.savez_compressed(folder/'CONDITIONED.npz',**core_arrays(s))
        report.update(passed=True,outcome='AlreadyAboveFloor' if not bad else 'AllBulkCellsAboveFloor',final_bad_cells=remaining,
            final_minimum=float(q.min()),final_core_cells=len(ids),input_sha256=file_hash(folder/'INPUT.npz'),output_sha256=file_hash(folder/'CONDITIONED.npz'),
            original_points=array_hash(joined['points']),conditioned_points=array_hash(s.points),
            files={p.relative_to(folder).as_posix():file_hash(p) for p in sorted(folder.rglob('*')) if p.is_file()},seconds=time.monotonic()-started)
        save(folder/'CONDITIONING.json',report)
        receipt=dict(method_identity=identity(METHOD),receipt_sha256=file_hash(folder/'CONDITIONING.json'),input_sha256=report['input_sha256'],
            output_sha256=report['output_sha256'],outcome=report['outcome'],initial_bad_count=len(bad),final_bad_count=0,
            original_coordinate_identity=report['original_points'],coordinate_identity=report['conditioned_points'],
            existing_vertex_commits=next((z['commits'] for z in report['steps'] if z['kind']=='existing_vertices'),0),
            cavity_commits=next((z['commits'] for z in report['steps'] if z['kind']=='constrained'),0),native_calls=report['native_calls'],timings=report['timings'])
        successor['provenance']['conditioning']=receipt
        successor['provenance']['local_cavity_generation_invocations']=report['native_calls']
        return successor,receipt
    except BaseException as e:
        report.update(passed=False,error=repr(e),seconds=time.monotonic()-started)
        save(folder/'STOP.json',report)
        raise

def native_start(folder,parent,arrays,state,check):
    """Reconstruct the *fresh current* native output against original prefix."""
    from finite_research.direct_evidence_modules import driver
    import local_termination_topology as topology
    prefix_points=arrays['points'][:parent['prefix_points']]
    core=driver.arrays(folder/'native/generated/NETGEN.npz',dict(points='<f8',tets='<i8'),check)
    capids=np.unique(arrays['cap']);lookup={prefix_points[int(v)].tobytes():int(v) for v in capids}
    if len(lookup)!=len(capids):raise ValueError('NonuniqueConditioningCap')
    mapped=np.empty(len(core['points']),dtype='<i8');other=[];seen=set()
    for i,p in enumerate(core['points']):
        if i%4096==0:check()
        v=lookup.get(p.tobytes())
        if v is None:mapped[i]=len(prefix_points)+len(other);other.append(i)
        else:
            if v in seen:raise ValueError('DuplicateConditioningNativeCap')
            seen.add(v);mapped[i]=v
    if seen!=set(map(int,capids)):raise ValueError('IncompleteConditioningNativeCap')
    points=np.concatenate((prefix_points,core['points'][other]));rows=mapped[core['tets']]
    prefix=set(state['prefix_ids']);cells=[c for c in state['cells'] if c['id'] in prefix]
    additions=build(rows,max(prefix)+1);allcells=cells+additions
    owners,incidence=topology.ownership(allcells,list(map(tuple,arrays['farfield'])),list(map(tuple,arrays['wall'])))
    if not incidence['passed']:raise ValueError('ConditioningOriginalNativeIncidence')
    return CavityTopology(points,allcells,owners,[c['id'] for c in additions],arrays['farfield'])

def verify(folder,parent,arrays,state,policy,check):
    """Replay only committed edits; never invoke proposal search or Gmsh."""
    from contextlib import ExitStack
    from immutable_state_access import guard
    from finite_research.direct_evidence_modules import driver
    path=folder/'conditioning';receipt=state['provenance']['conditioning'];report_path=path/'CONDITIONING.json'
    with ExitStack() as held:
        held.enter_context(guard(report_path));report=driver.read(report_path)
        if file_hash(report_path)!=receipt['receipt_sha256'] or identity(METHOD)!=receipt['method_identity'] or report['method']!=json.loads(json.dumps(METHOD)) or not report['passed']:raise ValueError('ChangedConditioningReceipt')
        if report['parent']!=state['provenance']['binding'] or report['policy_identity']!=identity(policy) or report['live_parent']!=state['provenance']['live_parent']:raise ValueError('ChangedConditioningParent')
        mirrors=dict(input_sha256=report['input_sha256'],output_sha256=report['output_sha256'],outcome=report['outcome'],
            initial_bad_count=len(report['initial_bad_cells']),final_bad_count=len(report['final_bad_cells']),
            original_coordinate_identity=report['original_points'],coordinate_identity=report['conditioned_points'],
            existing_vertex_commits=next((z['commits'] for z in report['steps'] if z['kind']=='existing_vertices'),0),
            cavity_commits=next((z['commits'] for z in report['steps'] if z['kind']=='constrained'),0),native_calls=report['native_calls'],timings=report['timings'])
        if any(receipt.get(k)!=v for k,v in mirrors.items()) or state['provenance']['local_cavity_generation_invocations']!=report['native_calls']:raise ValueError('ChangedConditioningProvenanceMirrors')
        for name,pin in report['files'].items():
            target=(path/name).resolve()
            if not target.is_relative_to(path.resolve()) or file_hash(target)!=pin:raise ValueError('ChangedConditioningArtifact',name)
            held.enter_context(guard(target))
        s=native_start(folder,parent,arrays,state,check)
        before=core_arrays(s)
        with np.load(path/'INPUT.npz',allow_pickle=False) as z:
            for k,a in before.items():driver.same(z[k],a,'fresh native conditioning input '+k)
        minimum=policy['quality_targets']['outer_tetrahedra']['minimum'];_,_,q,initial_bad=population(s,minimum)
        if initial_bad!=report['initial_bad_cells'] or float(q.min())!=report['initial_minimum']:raise ValueError('ChangedConditioningInitialPopulation')
        # First verification is within the live lease; the cold reader writes
        # a distinct audit directory. These files never alter original receipts.
        output=_replay_output.get()
        if output is None:output=folder/('conditioning_replay' if not (folder/'conditioning_replay').exists() else 'conditioning_readback')
        output.mkdir()
        edits=0
        kinds=[step['kind'] for step in report['steps']]
        expected_kinds=(['existing_vertices'] if report['vertex_invoked'] else [])+(['constrained'] if report['cavity_invoked'] else [])
        if kinds!=expected_kinds or (bool(initial_bad)!=report['vertex_invoked']) or (report['cavity_invoked'] and not report['vertex_invoked']):raise ValueError('ChangedConditioningInvocationOrder')
        for step in report['steps']:
            sub=path/step['kind'] if step['kind']=='existing_vertices' else path/'constrained'
            subreport=driver.read(sub/'REPORT.json')
            if file_hash(sub/'REPORT.json')!=step['report_sha256'] or len(subreport['commits'])!=step['commits']:raise ValueError('ChangedConditioningStep')
            declared=METHOD['vertex_policy'] if step['kind']=='existing_vertices' else METHOD['cavity_policy']
            if subreport['policy']!=json.loads(json.dumps(declared)) or len(subreport['commits'])>declared['max_commits']:raise ValueError('ChangedConditioningFrozenBudget')
            _,_,step_q,step_bad=population(s,minimum)
            if step_bad!=subreport['initial_bad_cells'] or float(step_q.min())!=subreport['initial_minimum'] or len(s.core_ids)!=subreport['initial_core_cells']:raise ValueError('ChangedConditioningStepBaseline')
            # Reconcile each component's pre-state and its complete final state.
            input_file=sub/('INPUT.npz' if step['kind']=='existing_vertices' else 'RESULT.npz')
            if step['kind']=='existing_vertices':
                with np.load(input_file,allow_pickle=False) as z:
                    for k,a in core_arrays(s).items():driver.same(z[k],a,'existing vertex component input '+k)
            for commit in subreport['commits']:
                check()
                if step['kind']=='existing_vertices':candidate=sub/('trial_%04d'%commit['index'])/'CANDIDATE.npz'
                else:candidate=sub/commit['native_folder']/(commit['stage']+'_transaction')/'CANDIDATE.npz'
                with np.load(candidate,allow_pickle=False) as z:
                    rows=z['inserted'].copy();remove=list(map(int,z['removed']));points=z['points'].copy() if 'points' in z else s.points
                expected_remove=commit['removed'] if step['kind']=='existing_vertices' else commit['proposal']['remove']
                if remove!=expected_remove or not set(remove)<=s.core_ids:raise ValueError('ChangedConditioningRemovedIDs')
                old=[s.byid[i] for i in remove];faces=boundary(old)[1];vs=np.unique(rows)
                if len(old)>declared['max_cavity_cells'] or step['kind']=='constrained' and len(faces)>declared['max_boundary_faces']:raise ValueError('ConditioningReplayCavityBudget')
                successor,validated=admit(s,old,faces,points[vs],np.searchsorted(vs,rows),output/('edit_%02d'%edits),minimum,CavityPolicy(),check)
                if successor is None:raise ValueError('ConditioningCommittedEditReplayRefusal',validated['outcome'])
                expected_ids=[c['id'] for c in commit['added']] if step['kind']=='existing_vertices' else commit['added_ids']
                if validated['added_ids']!=expected_ids:raise ValueError('ChangedConditioningAddedIDs')
                s=successor;edits+=1
            with np.load(sub/'RESULT.npz',allow_pickle=False) as z:
                for k,a in core_arrays(s).items():driver.same(z[k],a,'component final replay '+k)
        after=core_arrays(s)
        with np.load(path/'CONDITIONED.npz',allow_pickle=False) as z:
            for k,a in after.items():driver.same(z[k],a,'conditioned output replay '+k)
        _,_,q,bad=population(s,minimum)
        if bad or float(q.min())!=report['final_minimum']:raise ValueError('ConditioningReplayedBulkFloor')
        driver.same(after['tetrahedra'],arrays['background_tets'],'conditioned final core rows')
        driver.same(after['cell_ids'],arrays['tetrahedron_ids'],'conditioned final core IDs')
        if array_hash(s.points)!=report['conditioned_points'] or array_hash(before['points'])!=report['original_points']:raise ValueError('ConditioningReplayedCoordinateIdentity')
        if file_hash(path/'INPUT.npz')!=receipt['input_sha256'] or file_hash(path/'CONDITIONED.npz')!=receipt['output_sha256']:raise ValueError('ConditioningArchiveBinding')
        check();return s.points
