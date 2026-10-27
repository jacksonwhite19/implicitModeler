"""Declared AcceptedFrontState targets at the shared V4 consumer boundary.

This is a diagnostic contract, not a production transition authorization.
Source queries produce directions; AcceptedFrontState alone updates positions.
Neither cumulative path length nor a direction is represented as an SDF query.
Pinned authenticated state readers and the scalar capsule route are unchanged.
"""
import ast
import copy
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from contextlib import contextmanager,ExitStack
import weakref
import hashlib

import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash, identity, file_hash, frozen

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
PRESERVED = V4/'src/sdf_cfd_mesher_v4/legacy_full_stack_planner/_support/sdf_guided_advancing_front_qualification/accepted_front_state.py'
_verified_inputs=weakref.WeakKeyDictionary()

def _same_integrity_input(left,right):
    # Preserve type and signed-zero changes that plain numeric equality misses.
    if left is right:return True  # Only immutable leaves survive deepcopy by identity.
    if type(left) is not type(right):return False
    if type(left) is float:return left.hex()==right.hex()
    if type(left) is dict:
        return left.keys()==right.keys() and all(_same_integrity_input(v,right[k]) for k,v in left.items())
    if type(left) in (list,tuple):
        return len(left)==len(right) and all(_same_integrity_input(a,b) for a,b in zip(left,right))
    return left==right


def state_class():
    import importlib.util
    spec=importlib.util.spec_from_file_location('preserved_accepted_state',PRESERVED)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.AcceptedFrontState


def verify_record(record, expected_identity, *, geometry, parent, layer, units):
    if identity(record)!=expected_identity: raise ValueError('AlteredContinuationDiagnostic')
    if record.get('schema') not in ('V4RetainedSF2ContinuationDiagnostic/1','V4RunOwnedTrajectory/1'):
        raise ValueError('UnsupportedContinuationSchema')
    if units!='m' or record['units']!='m': raise ValueError('ContinuationUnitsMismatch')
    if geometry!=record['geometry']: raise ValueError('ContinuationGeometryMismatch')
    if parent!=record['committed_parent']: raise ValueError('StaleContinuationParent')
    if layer>=record['historical_layer_count']: raise ValueError('NoPendingIncrement')
    if layer!=record['committed_layer'] or layer!=1: raise ValueError('ContinuationSchedulePositionMismatch')
    if record['first_scalar_quantity']!='full_normal_height_m' or record['later_scalar_quantity']!='path_length_m':
        raise ValueError('UnsupportedContinuationScalarMeaning')
    if record.get('generation_authorized') is not False: raise ValueError('DiagnosticAuthorizationRequired')
    if record['schema']=='V4RunOwnedTrajectory/1':
        from owned_runtime import SOURCE,verify
        verify()
        paths={p.relative_to(ROOT).as_posix() for p in (PRESERVED,SOURCE/'bind_planner.py',SOURCE/'check_complete_geometry.py')}
        supplied=[row['path'] for row in record['sources']]
        if len(supplied)!=len(paths) or set(supplied)!=paths:
            raise ValueError('RunOwnedTrajectoryProducerMismatch')
        if record['ownership']!='C_prospective_final_fluid':raise ValueError('RunOwnedTrajectoryOwnershipMismatch')
    for row in record['sources']:
        if file_hash(ROOT/row['path'])!=row['sha256']: raise ValueError('ContinuationSourceMismatch:'+row['path'])
    source=next(r for r in record['sources'] if r['path'].endswith('/accepted_front_state.py'))
    if file_hash(PRESERVED)!=source['sha256']: raise ValueError('AcceptedStatePreservedSourceMismatch')


def source_directions(authority, wall, faces, refs, keys, source_path):
    """Reuse ONLY the two pure source-incidence/normal loops in bind_planner.

    This is the same bounded AST extraction pattern as live_spline_first_layer.
    No historical I/O, planner selection, state write, or audit is executed.
    The caller verifies this exact source file against the recovered contract.
    """
    from canonical_spline_authority import LiveSplineAuthority
    from general_sdf_authority import GeneralSdfAuthority
    if type(authority) not in (LiveSplineAuthority,GeneralSdfAuthority): raise ValueError('UnsupportedContinuationTrajectoryOwner')
    authority.verify()
    function=next(n for n in ast.parse(source_path.read_text(encoding='utf-8')).body
                  if isinstance(n,ast.FunctionDef) and n.name=='bind')
    loops=[n for n in function.body if isinstance(n,ast.For)]
    if len(loops)!=2: raise ValueError('ExistingTrajectoryProducerStructureChanged')
    ns=dict(np=np,a=authority,p=wall,f=faces,refs=refs,keys=keys,
            inc=[set() for _ in wall],normals=[],incidents=[],later=[],normal_keys=[])
    exec(compile(ast.Module(body=loops,type_ignores=[]),str(source_path),'exec'),ns)
    return np.asarray(ns['normals']),ns['incidents'],np.asarray(ns['later']),ns['normal_keys']


@dataclass(frozen=True)
class DeclaredTarget:
    owner: str
    parent: str
    layer: int
    quantity: str = 'path_length_m'


class AcceptedSchedule:
    """Layer-major source-vertex increments, never scalar offset targets."""
    def __init__(self, record, selected, mapping):
        if array_hash(selected)!=record['selected_schedule_array_hash']:
            raise ValueError('SelectedScheduleIdentityMismatch')
        if array_hash(mapping)!=record['current_to_historical_vertex_map']:
            raise ValueError('ContinuationMappingMismatch')
        if mapping.shape!=(len(selected),) or sorted(mapping.tolist())!=list(range(len(selected))):
            raise ValueError('NonbijectiveContinuationMapping')
        if selected.shape!=(len(mapping),record['historical_layer_count']) or not np.isfinite(selected).all() or np.any(selected<=0):
            raise ValueError('InvalidScalarSchedule')
        first=float.fromhex(record['nominal_increment_hex'][0])
        if not np.all(selected[:,0]==first): raise ValueError('ImmutableFirstIncrementChanged')
        self.increments=frozen(selected[mapping].T,'<f8')
        self.sha256=identity(dict(record=identity(record),mapping=array_hash(mapping),increments=array_hash(self.increments),
                                 quantity='path_length_m',scope='source_vertex',first='full_normal_height_m'))
        self.first_height=first
        self.source_sha256=identity(record)
        self.targets=()  # Filled once by the identity-bound owner, not numeric SDF targets.

    def remaining(self,layer):
        if not 0<=layer<=len(self.increments): raise ValueError('InvalidCommittedLayer')
        return self.increments[layer:]

    def row_record(self,layer):
        row=self.increments[layer-1]
        return dict(layer=layer,quantity='full_normal_height_m' if layer==1 else 'path_length_m',
                    scope='global' if np.all(row==row[0]) else 'source_vertex',
                    array_hash=array_hash(row),uniform_hex=float(row[0]).hex() if np.all(row==row[0]) else None,
                    minimum_hex=float(row.min()).hex(),maximum_hex=float(row.max()).hex())


_source_plans = weakref.WeakKeyDictionary()


def _array_binding(value):
    a=np.asarray(value)
    return dict(sha256=array_hash(a),shape=list(a.shape),dtype=a.dtype.str)


def _plan_inputs(authority,wall,first,faces,refs,increments,*,_transition=None):
    arrays=dict(wall=wall,first=first,faces=faces,refs=refs,increments=increments)
    result=dict(geometry=authority.geometry_identity,implementation=file_hash(__file__),
        **{name:array_hash(value) for name,value in arrays.items()},
        representations={name:dict(shape=list(np.asarray(value).shape),dtype=np.asarray(value).dtype.str)
                         for name,value in arrays.items()})
    if _transition is not None:result['transition_policy_identity']=identity(_transition)
    return result


def _freeze_transition_proposal(hint,policy):
    """Detached numeric proposal only; no quality or admission is imported."""
    import json
    if type(hint) is not dict or set(hint)!={'cap','faces','fronts','direction','controls_identity','construction'}:
        raise ValueError('ExplicitTransitionRowProposalRequired')
    cap,faces,fronts,direction=(hint[k] for k in ('cap','faces','fronts','direction'))
    rows=policy['tet_transition']['rows']
    if (any(type(v) is not np.ndarray for v in (cap,faces,fronts,direction)) or
            cap.dtype!=np.dtype('<f8') or fronts.dtype!=np.dtype('<f8') or direction.dtype!=np.dtype('<f8') or faces.dtype!=np.dtype('<i8') or
            cap.ndim!=2 or cap.shape[1:]!=(3,) or len(cap)==0 or
            fronts.shape!=(rows+1,len(cap),3) or direction.shape!=cap.shape or faces.ndim!=2 or faces.shape[1:]!=(3,) or not len(faces) or
            faces.min()<0 or faces.max()>=len(cap) or
            not np.isfinite(cap).all() or not np.isfinite(fronts).all() or not np.isfinite(direction).all() or
            not np.allclose(np.linalg.norm(direction,axis=1),1,atol=1e-13,rtol=0) or
            fronts[0].tobytes()!=cap.tobytes() or
            hint['controls_identity']!=identity(policy['tet_transition']) or type(hint['construction']) is not dict):
        raise ValueError('InvalidTransitionRowProposal')
    frozen_hint={k:frozen(hint[k],hint[k].dtype.str) for k in ('cap','faces','fronts','direction')}
    frozen_hint['controls_identity']=hint['controls_identity']
    frozen_hint['construction_json']=json.dumps(hint['construction'],sort_keys=True,allow_nan=False)
    return frozen_hint


def _transition_proposal_binding(hint):
    import json
    return dict(**{k:_array_binding(hint[k]) for k in ('cap','faces','fronts','direction')},
        controls_identity=hint['controls_identity'],construction_identity=identity(json.loads(hint['construction_json'])))


class _SourcePrismPlan:
    __slots__=('vectors','positions','plan_identity','_summary','__weakref__')
    """Call-owned proposal handoff, never an acceptance or restart certificate."""
    def check(self, authority, wall, first, faces, refs, increments):
        held = _source_plans.get(self)
        if held is None or held['authority'] is not authority:
            raise ValueError('LiveSourcePrismPlanRequired')
        authority.verify()
        import json
        transition=None if held['transition_policy_json'] is None else json.loads(held['transition_policy_json'])
        inputs = _plan_inputs(authority,wall,first,faces,refs,increments,_transition=transition)
        if inputs != held['inputs'] or self.plan_identity != held['identity']:
            raise ValueError('ChangedSourcePrismPlanInputs')
        if (_array_binding(self.vectors) != held['vectors'] or
                _array_binding(self.positions) != held['positions'] or
                identity(self._summary) != held['summary']):
            raise ValueError('ChangedSourcePrismPlanProposal')
        hint=held.get('transition_hint')
        if hint is not None and _transition_proposal_binding(hint)!=held['hint_binding']:
            raise ValueError('ChangedTransitionRowProposal')

    def summary(self):
        return copy.deepcopy(self._summary)

    def transition_proposal(self,actual_cap,triangles,controls):
        """Retrieve live checked coordinates; consumers still run all gates."""
        import json
        held=_source_plans.get(self)
        if held is None:raise ValueError('LiveSourcePrismPlanRequired')
        hint=held.get('transition_hint')
        if hint is None:return None
        self.check(held['authority'],**held['hint_source_inputs'])
        from tet_layer_controls import validate
        validate(controls)
        if (type(actual_cap) is not np.ndarray or type(triangles) is not np.ndarray or
                _array_binding(actual_cap)!=held['hint_binding']['cap'] or
                _array_binding(triangles)!=held['hint_binding']['faces'] or
                identity(controls)!=hint['controls_identity']):
            raise ValueError('TransitionRowProposalBindingMismatch')
        return hint['fronts'],json.loads(hint['construction_json']),hint['direction']


def source_front_plan(authority, wall, first, faces, refs, increments, *, _completion=None, _completed_front=False, _transition=None, _observer=None):
    """Source-gradient midpoint lookahead at each front; unchanged scalar lengths.

    The existing dyadic wedge predicate screens the complete proposed schedule
    before expensive first-layer certification. This is only local cell
    feasibility. Optional bounded unit-direction correction uses the existing
    local quality closure. Final source segments, quality, global contacts,
    coverage and filling still belong to the existing acceptance owner.
    """
    import json, math, time
    from general_sdf_authority import GeneralSdfAuthority
    from sdf_cfd_mesher_v4.sdf.regional import numerical_points
    from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_signs, exact_wedge
    if type(authority) is not GeneralSdfAuthority or not authority.stock_prism_queries():
        raise ValueError('StockGeneralSourcePrismPlannerRequired')
    authority.verify()
    if type(_completed_front) is not bool:raise ValueError('InvalidCompletedFrontDeclaration')
    if _completion is not None:
        validate_direction_controls(_completion)
        if not _completed_front:raise ValueError('PrismDirectionCompletionRequiresBackground')
    transition_json=None
    if _transition is not None:
        from standalone_background import validate_policy
        if type(_transition) is not dict or _transition.get('schema')!='V4GradedBackground/6' or not _completed_front:
            raise ValueError('ProspectiveTetTransitionProfileRequired')
        transition_json=json.dumps(_transition,sort_keys=True,allow_nan=False)
        validate_policy(json.loads(transition_json))
    # Identity binds the admitted representation, including integer width.
    # Coordinates must already be the binary64 representation used by the
    # accepted-state owner; silently casting them would bind different inputs.
    wall_array=np.asarray(wall);first_array=np.asarray(first)
    face_array=np.asarray(faces);ref_array=np.asarray(refs)
    if (wall_array.dtype!=np.dtype('<f8') or first_array.dtype!=np.dtype('<f8') or
            face_array.dtype.kind not in 'iu' or ref_array.dtype.kind not in 'iu'):
        raise ValueError('InvalidSourcePrismPlanRepresentation')
    x=frozen(wall_array,wall_array.dtype.str);first=frozen(first_array,first_array.dtype.str)
    f=frozen(face_array,face_array.dtype.str);r=frozen(ref_array,ref_array.dtype.str)
    h=np.asarray(increments,dtype='<f8')
    if h.ndim==1:h=np.broadcast_to(h[:,None],(len(h),len(x)))
    h=frozen(h,'<f8')
    if (x.ndim!=2 or x.shape[1]!=3 or first.shape!=x.shape or
            f.ndim!=2 or f.shape[1]!=3 or not len(f) or f.min()<0 or f.max()>=len(x) or
            r.shape!=(len(f),) or np.any(r!=authority._root_ref) or
            h.ndim!=2 or h.shape[1]!=len(x) or not len(h) or not np.all(h[0]==h[0,0]) or
            not np.isfinite([x,first]).all() or not np.isfinite(h).all() or np.any(h<=0)):
        raise ValueError('InvalidSourcePrismPlanInputs')
    start=time.perf_counter();positions=[x,first];vectors=[];layers=[];completion=None;contacts=None;assembled=None
    inputs=_plan_inputs(authority,x,first,f,r,h,_transition=None if transition_json is None else json.loads(transition_json))
    transition_summary=None;transition_hint=None
    with authority.prism_query_scope(), authority.verified_operation('trajectory'):
        expression,scale,_,_=authority.queries._scope._batch_inputs()
        for layer in range(1,len(h)+1):
            if layer>1:
                low=positions[-1];direction=np.empty_like(low)
                # Midpoint lookahead resolves source-normal rotation across the requested step.
                # Only the final unit vector is used; the declared chord length stays h.
                for phase in (0,1):
                    sites=low if phase==0 else low+.5*h[layer-1,:,None]*direction
                    next_direction=np.empty_like(low)
                    for begin in range(0,len(low),4096):
                        authority.verify()
                        jet=numerical_points(expression,sites[begin:begin+4096],scale)
                        gradients=np.column_stack(jet.gradient)
                        for i,g in enumerate(gradients,begin):
                            length=math.sqrt(sum(float(v)*float(v) for v in g))
                            if not math.isfinite(length) or length<=0:raise ValueError('ConstructionNormalSingular')
                            next_direction[i]=[float(v)/length for v in g]
                        authority.verify()
                    direction=next_direction
                direction[direction==0]=0.
                direction=frozen(direction,'<f8');vectors.append(direction)
                high=low+h[layer-1,:,None]*direction
                if not np.isfinite(high).all():raise ValueError('NonfiniteSourcePrismPlan')
                positions.append(frozen(high,'<f8'))
            low,high=positions[-2:];failures=[]
            for column, ids in enumerate(f):
                prism=np.concatenate((low[ids],high[ids]))
                signs=exact_signs(prism)
                if signs['jacobian_sign']<=0 or signs['volume_sign']<=0:
                    failures.append(dict(column=column,layer=layer,**exact_wedge(prism)))
            layers.append(dict(layer=layer,prisms=len(f),nonpositive_cells=len(failures)))
            if failures and (layer==1 or (_completion is None and transition_json is None)):
                raise ValueError('SourcePrismScheduleInfeasible',dict(layer=layer,
                    failed_cells=len(failures),first_failures=failures[:3],layers=layers,
                    scope='Exact discrete proposed-cell failure. No schedule was shortened; no acceptance or global reach claim.'))
        if _completion is not None:
            positions=np.stack(positions);vectors=np.asarray(vectors).reshape(len(h)-1,len(x),3)
            completion,assembled=_complete_source_directions(authority,x,f,h,positions,vectors,_completion)
        if transition_json is not None:
            from tet_layer_planning import co_design
            policy=json.loads(transition_json)
            hint={}
            positions,vectors,transition_summary=co_design(authority,x,first,f,r,h,
                np.array(positions,dtype='<f8',copy=True),
                np.asarray(vectors,dtype='<f8').reshape(len(h)-1,len(x),3).copy(),policy,_hint_out=hint,_observer=_observer)
            if identity(policy)!=inputs['transition_policy_identity']:
                raise ValueError('ChangedProspectiveTetTransitionPolicy')
            positions=np.asarray(positions);vectors=np.asarray(vectors)
            if (positions.dtype!=np.dtype('<f8') or vectors.dtype!=np.dtype('<f8') or
                    positions.shape!=(len(h)+1,len(x),3) or vectors.shape!=(len(h)-1,len(x),3) or
                    not np.isfinite(positions).all() or not np.isfinite(vectors).all() or
                    positions[0].tobytes()!=x.tobytes() or positions[1].tobytes()!=first.tobytes() or
                    not np.allclose(np.linalg.norm(vectors,axis=2),1,atol=1e-13,rtol=0)):
                raise ValueError('InvalidProspectivePrismTrajectory')
            reconstructed=np.empty_like(positions);reconstructed[0]=x;reconstructed[1]=first
            for k in range(1,len(h)):reconstructed[k+1]=reconstructed[k]+h[k,:,None]*vectors[k-1]
            if reconstructed.tobytes()!=positions.tobytes():raise ValueError('ProspectivePrismTrajectoryArithmeticMismatch')
            if hint:transition_hint=_freeze_transition_proposal(hint,policy)
            # The co-design result is a proposal. Rebuild all ordinary prism
            # quality and contacts on its actual coordinates before issuance.
            assembled=None
        if len(h)>1:
            if assembled is None:assembled=_planned_quality(x,f,np.stack(positions),completed_front=_completed_front)
            if not assembled[4]['quality']['passed']:
                raise ValueError('SourcePrismPlanQualityFailure',dict(failed_records=len(assembled[4]['quality']['failures']),first_failures=assembled[4]['quality']['failures'][:3]))
            contacts=_planned_contacts(assembled)
        authority.verify()
    result=object.__new__(_SourcePrismPlan)
    result.vectors=frozen(np.asarray(vectors).reshape(len(h)-1,len(x),3),'<f8')
    result.positions=frozen(np.stack(positions),'<f8')
    body=dict(inputs=inputs,vectors=_array_binding(result.vectors),positions=_array_binding(result.positions),
        method='SourceGradientMidpointLookahead/1',completion=None if completion is None else {k:v for k,v in completion.items() if k!='seconds'},contacts=None if contacts is None else {k:v for k,v in contacts.items() if k!='seconds'},completed_front=_completed_front,first='unchanged full_normal_height_m',later='path_length_m',
        local_positive_cells=True,layers=layers,
        scope='Proposed discrete-cell feasibility only; no source-segment, quality, collision, coverage, degree or filling acceptance.')
    if transition_json is not None:
        body['initial_proposal_layers']=body.pop('layers')
        body['transition_co_design']=copy.deepcopy(transition_summary)
    if transition_hint is not None:body['transition_row_proposal']=_transition_proposal_binding(transition_hint)
    result.plan_identity=identity(body)
    result._summary=dict(body,identity=result.plan_identity,seconds=time.perf_counter()-start,completion_seconds=None if completion is None else completion['seconds'],contact_seconds=None if contacts is None else contacts['seconds'])
    _source_plans[result]=dict(authority=authority,inputs=inputs,identity=result.plan_identity,
        vectors=body['vectors'],positions=body['positions'],summary=identity(result._summary),transition_policy_json=transition_json)
    if transition_hint is not None:
        _source_plans[result].update(transition_hint=transition_hint,hint_binding=body['transition_row_proposal'],
            hint_source_inputs=dict(wall=x,first=first,faces=f,refs=r,increments=h))
    result.check(authority,wall,first,faces,refs,h)
    return result


def _planned_quality(wall, faces, positions, *, completed_front=True):
    """Completed-role quality/topology verdict only, no acceptance receipt.

    Source, contacts, independent coverage and volume/degree remain separate.
    """
    from unittest.mock import patch
    from contextlib import nullcontext
    import generalize_local_remesh as g
    from prism_hotpath_v2 import completed_incidence, batch_geometry, batch_quality
    # The common CLI already owns the production handoffs. Borrow its selected
    # quality callable; do not reinstall writer/storage/publication scopes.
    # A cold standalone probe selects only the same existing batch mathematics.
    # Custom quality or geometry bindings must retain their original dispatch.
    standard=(g.t.quality is g.t._STANDARD_QUALITY and g.t.geometry is g.t._STANDARD_GEOMETRY)
    def cold_quality(points,cells,owners,base_layer,*,active_cells=None):
        return batch_quality(points,cells,owners,base_layer,active_cells=active_cells,_geometry_out={})
    with patch.object(g.t,'quality',cold_quality) if standard else nullcontext():
        points,cells,outer,checked,_=g.validation(wall,faces,list(positions[1:]),
            np.full(len(wall),len(positions)-1),False)
    owners,_=g.t.ownership(cells,outer,[tuple(reversed(f)) for f in faces])
    if g.t.geometry is g.t._STANDARD_GEOMETRY:
        geometry=batch_geometry(points,cells)
        with patch.object(g.t,'geometry',side_effect=lambda points,cell:geometry[cell['id']]):
            final=completed_incidence(points,cells,outer,owners)
    else:
        final=completed_incidence(points,cells,outer,owners)
    determinants={r['id']:r['determinant'] for r in final['cells']}
    quality=checked['quality']
    quality['cells']=[dict(r,determinant=determinants[r['id']]) if completed_front or r['id']<2000000 else r for r in quality['cells']]
    quality['failures']=[r for r in quality['failures'] if r['kind']!='cell_quality']
    quality['failures'] += [dict(kind='cell_quality',**r) for r in quality['cells']
        if not (r['volume']>0 and r['determinant']>=.0011 and r['aspect']<=1000)]
    quality['passed']=not quality['failures']
    checked.update(complete_incidence=final,first_prism_incidence=dict(
        scope='Proposed completed-role numerical evaluator only; not an acceptance receipt.'))
    checked['passed']=all(checked[k]['passed'] for k in
        ('quality','ownership','vertex_links','complete_incidence'))
    return points,cells,outer,owners,checked


def _complete_source_directions(authority,wall,faces,h,positions,vectors,controls):
    """Bounded unit-direction proposals using the existing local QC closure.

    Solver convergence grants no acceptance. Every proposed change is evaluated
    by LocalCorrectionValidation, including exact wedges and adjacent face
    gates. Historical fronts are never moved; future source trajectories are
    regenerated after a direction change. Final acceptance still runs normally.
    """
    import math,time
    from scipy.optimize import minimize
    from local_correction_validation import LocalCorrectionValidation
    from coordinated_local_repair import LastConstraintValue
    from refined_local_repair import raw_failures
    from sdf_cfd_mesher_v4.advancing_front.local_relaxation import tangent_basis
    from sdf_cfd_mesher_v4.sdf.regional import numerical_points
    # Explicit caller-owned operational budgets; no hidden numerical defaults.
    operational=copy.deepcopy(validate_direction_controls(controls))
    start=time.perf_counter();n=len(wall);last=len(h);history=[];modified=set()
    points,cells,outer,owners,checked=_planned_quality(wall,faces,positions)
    if not checked['ownership']['passed'] or not checked['vertex_links']['passed']:
        raise ValueError('SourcePrismPlanTopologyFailure')
    expression,scale,_,_=authority.queries._scope._batch_inputs()
    def normal(points):
        authority.verify();jet=numerical_points(expression,points,scale);out=[]
        for g in np.column_stack(jet.gradient):
            length=math.sqrt(sum(float(v)*float(v) for v in g))
            if not math.isfinite(length) or length<=0:raise ValueError('ConstructionNormalSingular')
            out.append([float(v)/length for v in g])
        authority.verify();return np.asarray(out)
    while not checked['quality']['passed']:
        if len(history)>=operational['maximum_groups'] or time.perf_counter()-start>=operational['seconds_per_sweep']:
            raise ValueError('PrismDirectionCompletionWorkLimit',dict(groups=len(history),modified_vertices=len(modified),
                remaining_failures=len(checked['quality']['failures']),controls=controls))
        # One real failing-cell support; actual incident closure includes all
        # its neighbours. No fixture coordinates or connected-global solve.
        bad=checked['quality']['failures'][0]
        population={c['id']:c for c in cells}
        ids=[bad['id']] if 'id' in bad else bad['owners']
        layer=min(population[i]['layer'] for i in ids)
        if layer<2:raise ValueError('ImmutableFirstFrontQualityFailure',bad)
        vertices=np.unique([v%n for i in ids for v in population[i]['vertices']])
        if len(vertices)>operational['maximum_group_vertices']:
            raise ValueError('PrismDirectionCompletionGroupLimit',len(vertices))
        future_indices=np.concatenate([vertices+k*n for k in range(layer,last+1)])
        if len(modified|set(map(int,future_indices)))>operational['maximum_modified_vertices']:
            raise ValueError('PrismDirectionCompletionVertexLimit',len(modified|set(map(int,future_indices))))
        affected={c['id'] for c in cells if set(c['vertices'])&set(map(int,future_indices))}
        if len(affected)>operational['maximum_union_cells']:
            raise ValueError('PrismDirectionCompletionCellLimit',len(affected))
        evaluator=LocalCorrectionValidation(wall,faces,list(positions[1:]),np.full(n,last),checked)
        original_positions=positions.copy();original_vectors=vectors.copy()
        seed=vectors[layer-2,vertices].copy();basis=tangent_basis(seed)
        exact_scale={r['id']:abs(float(__import__('fractions').Fraction(r['jacobian'])))
                     for r in checked['quality']['exact'] if r['id'] in affected}
        volume_scale={r['id']:abs(r['volume']) for r in checked['quality']['cells'] if r['id'] in affected}
        if any(v<=0 for v in exact_scale.values()) or any(v<=0 for v in volume_scale.values()):
            raise ValueError('NonpositivePrismDirectionScale')
        scoped_owners={tuple(sorted(face)):owners[tuple(sorted(face))]
            for cell in cells if cell['id'] in affected for face in cell['faces']}
        needed={i for rows in scoped_owners.values() for i,_ in rows}
        scoped_cells=[population[i] for i in sorted(needed)]
        scoped_outer=[face for face in outer if tuple(sorted(face)) in scoped_owners]
        cached={}
        def materialize(z):
            if time.perf_counter()-start>=operational['seconds_per_sweep']:
                raise ValueError('PrismDirectionCompletionDeadline')
            proposed=seed+np.einsum('nij,nj->ni',basis,np.asarray(z).reshape(-1,2))
            proposed/=np.linalg.norm(proposed,axis=1)[:,None]
            positions[:]=original_positions;vectors[:]=original_vectors
            vectors[layer-2,vertices]=proposed
            positions[layer,vertices]=positions[layer-1,vertices]+h[layer-1,vertices,None]*proposed
            for k in range(layer+1,last+1):
                low=positions[k-1,vertices];first=normal(low)
                proposed=normal(low+.5*h[k-1,vertices,None]*first)
                vectors[k-2,vertices]=proposed
                positions[k,vertices]=low+h[k-1,vertices,None]*proposed
            return positions.reshape(-1,3)
        def compute(z):
            coordinates=materialize(z)
            from unittest.mock import patch
            with patch.object(evaluator,'cells',scoped_cells),patch.object(evaluator,'owners',scoped_owners),patch.object(evaluator,'outer',scoped_outer):
                result=evaluator.evaluate(coordinates,local_only=True,_active_cells=affected)
            q=result['quality'];values=[]
            for r in q['cells']:
                if r['id'] in affected:
                    values.extend((r['volume']/volume_scale[r['id']],r['determinant']/.0011-1,1000/r['aspect']-1))
            for r in q['exact']:
                if r['id'] in affected:
                    values.append(float(__import__('fractions').Fraction(r['jacobian']))/exact_scale[r['id']])
            for r in q['faces']:
                if set(r['owners'])&affected:
                    values.extend((r['weight']/.055-1,3.63636-r['skewness'],r['symmetric_volume_ratio']/.01-1))
            value=np.asarray(values)
            if not np.isfinite(value).all():raise ValueError('NonfinitePrismDirectionConstraints')
            cached['checked']=result
            return value
        values=LastConstraintValue(compute);before=values(np.zeros(2*len(vertices)));passing=before>=0
        def constraints(y):
            current=values(y[:-1]);return np.r_[current-y[-1],current[passing]]
        initial=np.r_[np.zeros(2*len(vertices)),before.min()]
        fit=minimize(lambda y:-float(y[-1]),initial,method='SLSQP',
            bounds=[(-1,1)]*(2*len(vertices))+[(None,None)],constraints=[dict(type='ineq',fun=constraints)],
            options=dict(maxiter=operational['maxiter'],ftol=operational['ftol']))
        values(fit.x[:-1]);materialize(fit.x[:-1])
        candidate=evaluator.evaluate(positions.reshape(-1,3));old_bad=raw_failures(checked['quality']);new_bad=raw_failures(candidate['quality'])
        if not new_bad<old_bad:
            raise ValueError('PrismDirectionCompletionNoStrictProgress',dict(layer=layer,vertices=vertices.tolist(),
                old_failures=len(old_bad),new_failures=len(new_bad),solver_success=bool(fit.success)))
        checked=candidate;modified.update(map(int,future_indices))
        history.append(dict(layer=layer,vertices=vertices.tolist(),affected_cells=len(affected),
            solver_success=bool(fit.success),evaluations=values.evaluations,old_failures=len(old_bad),new_failures=len(new_bad)))
    # Verify full, freshly materialized geometry after the provisional merges.
    full_assembled=_planned_quality(wall,faces,positions)
    full=full_assembled[4]
    if not full['quality']['passed']:raise ValueError('PrismDirectionCompletionFullQualityFailure',full['quality']['failures'][:3])
    return dict(passed=True,controls=controls,history=history,modified_vertices=len(modified),
        seconds=time.perf_counter()-start,scope='Construction only; exact source, continuous contacts, independent coverage and filling still required.'),full_assembled


def validate_direction_controls(controls):
    """Explicit construction budgets, never scientific acceptance thresholds."""
    import math
    integers=('maxiter','maximum_groups','maximum_group_vertices',
        'maximum_modified_vertices','maximum_union_cells')
    fields={'schema','ftol','seconds_per_sweep',*integers}
    if type(controls) is not dict or set(controls)!=fields or controls['schema']!='V4PrismDirectionCompletion/1':
        raise ValueError('ExplicitPrismDirectionCompletionControlsRequired')
    for name in integers:
        if type(controls[name]) is not int or controls[name]<1:
            raise ValueError('InvalidPrismDirectionControl',name)
    for name in ('ftol','seconds_per_sweep'):
        value=controls[name]
        if type(value) not in (float,int) or not math.isfinite(value) or value<=0:
            raise ValueError('InvalidPrismDirectionControl',name)
    return controls


def _planned_contacts(assembled):
    """Existing whole-stack broad phase and continuous contact predicates.

    Every physical wall/cap/side face participates, including nonadjacent layers.
    No shared-vertex pairs are dropped. This early screen is not the independent
    final coverage enumeration or an accepted filling/collision certificate.
    """
    import time
    from collections import Counter
    from optimized_mixed_certificate import Native
    from optimized_admission import classify
    from production_collision_localization import topology_relations
    points,cells,outer,owners,checked=assembled
    start=time.perf_counter()
    faces=[owners[key][0][1] for key in sorted(owners)]
    labels=np.asarray([np.asarray(face)[[0,1,3,2]] if len(face)==4 else tuple(face)+(face[-1],) for face in faces],dtype='<i8')
    if any(len(face) not in (3,4) for face in faces):raise ValueError('OrdinaryPrismPlanFacesRequired')
    quads=np.ascontiguousarray(points[labels])
    pairs=Native().pairs(quads,np.arange(len(labels),dtype='<i8'),8).astype(np.int32)
    native=Native();screen=native.screen(quads,labels,pairs,8)
    codes=screen[:,0].copy();unresolved=np.flatnonzero(codes==0)
    relations=topology_relations(labels,pairs[unresolved])
    exact_indices=unresolved[relations==0]
    if len(exact_indices):
        exact=classify(points,labels,pairs[exact_indices]);codes[exact_indices]=exact['codes']
    deferred=unresolved[relations!=0]
    counts=Counter(map(int,codes))
    bad=np.r_[np.flatnonzero(np.isin(codes,[10,11])),exact_indices[codes[exact_indices]==0]]
    summary=dict(faces=len(faces),candidate_pairs=len(pairs),classifications={str(k):v for k,v in counts.items()},
        rejected_pairs=len(bad),deferred_incident_pairs=len(deferred),deferred_unusual_topology=int(np.sum(relations==3)),
        pairs_sha256=array_hash(pairs),deferred_pairs_sha256=array_hash(pairs[deferred]),
        points_sha256=array_hash(points),labels_sha256=array_hash(labels),exact_nonincident_pairs=len(exact_indices),seconds=time.perf_counter()-start,
        scope='All proposed-stack faces screened by existing conservative broad phase and native predicates. Unresolved nonincident pairs use existing exact fallback; incident unresolved pairs are explicitly deferred to unchanged final acceptance. No collision-free, independent coverage, source accuracy or filling claim.')
    if len(bad):raise ValueError('SourcePrismPlanContactFailure',dict(summary,
        first_failures=[dict(pair=pairs[i].tolist(),code=int(codes[i])) for i in bad[:3]]))
    return summary


class AcceptedTrajectoryOwner:
    semantics='AcceptedFrontState/path_length_m/1'
    generation_authorized=False

    @classmethod
    def from_inputs(cls, authority, wall, first, faces, refs, increments, *, parent,
                    surface, request, case_identity, _plan=None):
        """Bind caller requirements to the live producer; no saved planner state.

        All arrays are supplied by this run's qualified surface/first front.
        The unchanged AcceptedFrontState validates normal and path semantics.
        This constructs an owner, not a production grant or a qualified layer.
        """
        from owned_runtime import SOURCE, verify
        verify()
        values=np.asarray(increments,dtype='<f8')
        if values.ndim==1:
            selected=np.broadcast_to(values,(len(wall),len(values)))
        elif values.ndim==2 and values.shape[0]==len(wall):
            selected=values
        else:raise ValueError('SourceVertexScheduleShapeMismatch')
        mapping=np.arange(len(wall),dtype='<i8')
        if selected.shape[1]<1 or not np.isfinite(selected).all() or np.any(selected<=0):
            raise ValueError('InvalidScalarSchedule')
        if not np.all(selected[:,0]==request['increments'][0]):
            raise ValueError('ImmutableFirstIncrementChanged')
        sources=(PRESERVED,SOURCE/'bind_planner.py',SOURCE/'check_complete_geometry.py')
        record=dict(schema='V4RunOwnedTrajectory/1',units='m',geometry=authority.geometry_identity,
            committed_parent=parent,committed_layer=1,historical_layer_count=selected.shape[1],
            first_scalar_quantity='full_normal_height_m',later_scalar_quantity='path_length_m',
            generation_authorized=False,surface=surface,ownership='C_prospective_final_fluid',
            selected_schedule_array_hash=array_hash(selected),current_to_historical_vertex_map=array_hash(mapping),
            nominal_increment_hex=[float(v).hex() for v in selected[0]],
            original_request_file_sha256=request['request_file_sha256'],
            original_request_identity=identity(request['declaration']),original_schedule_identity=request['schedule_identity'],
            input_case=case_identity,wall=array_hash(wall),first=array_hash(first),faces=array_hash(faces),
            refs=array_hash(refs),sources=[dict(path=p.relative_to(ROOT).as_posix(),sha256=file_hash(p)) for p in sources])
        if _plan is not None:
            if type(_plan) is not _SourcePrismPlan:raise ValueError('LiveSourcePrismPlanRequired')
            _SourcePrismPlan.check(_plan,authority,wall,first,faces,refs,selected.T)
            record['source_front_plan'] = _plan.plan_identity
        return cls(authority,record,identity(record),wall,first,faces,refs,mapping,selected,parent=parent,_plan=_plan)

    def __init__(self, authority, record, record_identity, wall, first, faces, refs, mapping, selected, *, parent, layer=1, units='m', _plan=None):
        verify_record(record,record_identity,geometry=authority.geometry_identity,parent=parent,layer=layer,units=units)
        self.authority=authority;self.geometry_identity=authority.geometry_identity;self.units=units
        self.record=copy.deepcopy(record);self.parent=parent;self.layer=layer
        self.schedule=AcceptedSchedule(record,selected,mapping)
        if record['schema']=='V4RunOwnedTrajectory/1':
            for name,value in (('wall',wall),('first',first),('faces',faces),('refs',refs)):
                if array_hash(value)!=record[name]:raise ValueError('RunOwnedTrajectoryInputMismatch:'+name)
            if not np.array_equal(mapping,np.arange(len(wall))):raise ValueError('RunOwnedSourceMappingMismatch')
        else:
            historical_wall=ROOT/next(r['path'] for r in record['sources'] if r['path'].endswith('/selected_state/wall.npy'))
            historical_first=ROOT/next(r['path'] for r in record['sources'] if r['path'].endswith('/selected_state/first_front.npy'))
            for path,actual in ((historical_wall,wall),(historical_first,first)):
                stored=np.load(path,allow_pickle=False,mmap_mode='r')[mapping]
                if stored.shape!=actual.shape or stored.tobytes()!=actual.tobytes():
                    raise ValueError('AcceptedCommittedMappingMismatch')
        path=ROOT/next(r['path'] for r in record['sources'] if r['path'].endswith('/bind_planner.py'))
        keys=tuple(f'source:{i}' for i in range(len(wall)))
        normals,incidents,later,normal_keys=source_directions(authority,wall,faces,refs,keys,path)
        vectors=np.broadcast_to(later,(len(self.schedule.increments)-1,*later.shape))
        if record.get('source_front_plan') is not None:
            if type(_plan) is not _SourcePrismPlan:raise ValueError('LiveSourcePrismPlanRequired')
            _SourcePrismPlan.check(_plan,authority,wall,first,faces,refs,self.schedule.increments)
            if record['source_front_plan']!=_plan.plan_identity:raise ValueError('SourcePrismPlanIdentityMismatch')
            vectors=_plan.vectors
        elif _plan is not None:raise ValueError('UnboundSourcePrismPlan')
        self.state=state_class()(wall,first,faces,keys,normals,incidents,
            vectors,self.schedule.first_height,
            record['later_scalar_quantity'],ownership=record['ownership'])
        # Authenticate all mutable Python objects at consumer entry; bytes-backed
        # snapshots below do not permit callers to turn array writeability on.
        for name in ('wall','first','faces','normals','vectors','pairs'):
            value=getattr(self.state,name);setattr(self.state,name,frozen(value,value.dtype.str))
        self.refs=frozen(refs,'<i8')
        self.source_identity=identity(dict(semantics=self.semantics,record=record_identity,
            parent=parent,geometry=self.geometry_identity,units=units,schedule=self.schedule.sha256,
            source_producer=file_hash(path),accepted_state=file_hash(PRESERVED),implementation=file_hash(__file__),
            wall=array_hash(self.state.wall),first=array_hash(self.state.first),faces=array_hash(self.state.faces),
            refs=array_hash(self.refs),normals=array_hash(self.state.normals),vectors=array_hash(self.state.vectors),
            incidents=identity(self.state.incidents),normal_keys=identity(normal_keys),mapping=array_hash(mapping)))
        self.schedule.targets=tuple(DeclaredTarget(self.source_identity,parent,k,
            'full_normal_height_m' if k==1 else 'path_length_m') for k in range(1,len(self.schedule.increments)+1))
        self._plan_position_binding=None if _plan is None else _array_binding(_plan.positions)
        self._integrity=self.integrity()
        self._positions=None
        if _plan is not None:
            reconstructed=self.state.positions(self.schedule.increments.T)
            if reconstructed.tobytes()!=_plan.positions.tobytes():raise ValueError('SourcePrismPlanArithmeticMismatch')
            self._positions=frozen(reconstructed,'<f8')
        self.seconds=0.

    def _integrity_inputs(self):
        # Original payload before serialization/hashing. The scoped comparison
        # keeps these mutable fields and exact array bytes bound to that payload.
        return dict(record=self.record,source_identity=self.source_identity,parent=self.parent,layer=self.layer,
            geometry=self.geometry_identity,units=self.units,plan_positions=self._plan_position_binding,schedule_identity=self.schedule.sha256,
            first_height=self.schedule.first_height,incidents=self.state.incidents,keys=self.state.vertex_order,
            first=self.state.first_height,quantity=self.state.scalar_quantity,ownership=self.state.ownership,
            height_tolerance=self.state.height_tolerance,targets=[vars(t) for t in self.schedule.targets])

    def _integrity_arrays(self):
        return dict(refs=self.refs,schedule=self.schedule.increments,
            **{name:getattr(self.state,name) for name in ('wall','first','faces','normals','vectors','pairs')})

    @contextmanager
    def verified_input_scope(self):
        """Reuse this owner's verified binding within guarded, unchanged custody."""
        if type(self) is not AcceptedTrajectoryOwner or self in _verified_inputs:
            raise ValueError('VerifiedTrajectoryScopeRequired')
        from immutable_state_access import guard
        with ExitStack() as held:
            state=self.state;schedule=self.schedule;authority=self.authority;integrity=self._integrity
            payload=copy.deepcopy(self._integrity_inputs())
            arrays={k:(a.shape,a.dtype.str,a.tobytes()) for k,a in self._integrity_arrays().items()}
            hashes={k:hashlib.sha256(v[2]).hexdigest() for k,v in arrays.items()}
            captured=identity(dict(payload,refs=hashes['refs'],schedule=hashes['schedule'],
                state={k:hashes[k] for k in ('wall','first','faces','normals','vectors','pairs')}))
            if captured!=integrity:raise ValueError('AlteredAcceptedTrajectoryBinding')
            for row in payload['record']['sources']:
                path=ROOT/row['path'];held.enter_context(guard(path))
                if file_hash(path)!=row['sha256']:raise ValueError('ContinuationSourceMismatch:'+row['path'])
            _verified_inputs[self]=dict(payload=payload,arrays=arrays,
                integrity=integrity,authority=authority,state=state,schedule=schedule)
            try:
                self.check()
                yield self
            finally:_verified_inputs.pop(self,None)

    def integrity(self):
        return identity(dict(record=self.record,source_identity=self.source_identity,parent=self.parent,layer=self.layer,
            geometry=self.geometry_identity,units=self.units,plan_positions=self._plan_position_binding,schedule_identity=self.schedule.sha256,
            first_height=self.schedule.first_height,refs=array_hash(self.refs),schedule=array_hash(self.schedule.increments),
            state={name:array_hash(getattr(self.state,name)) for name in ('wall','first','faces','normals','vectors','pairs')},
            incidents=self.state.incidents,keys=self.state.vertex_order,first=self.state.first_height,
            quantity=self.state.scalar_quantity,ownership=self.state.ownership,height_tolerance=self.state.height_tolerance,
            targets=[vars(t) for t in self.schedule.targets]))

    def check(self):
        scoped=_verified_inputs.get(self)
        if scoped is None:
            if self.integrity()!=self._integrity:raise ValueError('AlteredAcceptedTrajectoryBinding')
        else:
            if (self._integrity!=scoped['integrity'] or self.authority is not scoped['authority'] or
                self.state is not scoped['state'] or self.schedule is not scoped['schedule'] or
                not _same_integrity_input(self._integrity_inputs(),scoped['payload'])):raise ValueError('AlteredAcceptedTrajectoryBinding')
            for k,a in self._integrity_arrays().items():
                shape,dtype,raw=scoped['arrays'][k]
                if a.shape!=shape or a.dtype.str!=dtype or a.tobytes()!=raw:
                    raise ValueError('AlteredAcceptedTrajectoryBinding')
        self.authority.verify()

    def positions(self):
        self.check()
        # Existing operation, including immutable X1 and binary64 multiply/add order.
        if self._positions is None:
            self._positions=frozen(self.state.positions(self.schedule.increments.T),'<f8')
        if self._plan_position_binding is not None and _array_binding(self._positions)!=self._plan_position_binding:
            raise ValueError('ChangedSourcePrismPlanPositions')
        return self._positions

    def check_target(self,target):
        if type(target) is not DeclaredTarget or target.owner!=self.source_identity or target.parent!=self.parent:
            raise ValueError('StaleDeclaredTrajectoryTarget')
        if target.quantity!='path_length_m': raise ValueError('IncompatibleTargetQuantity')
        if not self.layer<target.layer<=len(self.schedule.increments): raise ValueError('ContinuationSchedulePositionMismatch')

    def predict(self,points,target,iteration_cap=64):
        self.check_target(target);positions=self.positions()
        if np.asarray(points).tobytes()!=positions[target.layer-1].tobytes() or np.shape(points)!=positions[target.layer-1].shape:
            raise ValueError('DeclaredTargetParentCoordinatesMismatch')
        return SimpleNamespace(coordinates=positions[target.layer],target=target,
            displacement=positions[target.layer]-positions[target.layer-1],semantics=self.semantics)

    def validate_target(self,points,target):
        self.check_target(target)
        if np.shape(points)!=self.positions()[target.layer].shape or np.asarray(points).tobytes()!=self.positions()[target.layer].tobytes():
            raise ValueError('DeclaredTargetCoordinatesMismatch')
        return dict(admitted=True,scope='Exact declared target consistency only; not trajectory/cell/collision qualification',
                    owner=self.source_identity,target=vars(target),coordinates=array_hash(points),
                    ownership=self.state.ownership,generation_authorized=False)

    def check_adapter(self,adapter,spec):
        self.check()
        if adapter.geometry_binding is not self or adapter.sdf is not self or adapter.schedule is not self.schedule:
            raise ValueError('DifferentForecastGenerationTrajectoryOwner')
        if spec!=adapter.trajectory_spec or spec.get('owner')!=self.source_identity:
            raise ValueError('DeclaredTrajectorySpecMismatch')
        if adapter.binding['committed']!=self.parent or adapter.binding['geometry']!=self.geometry_identity:
            raise ValueError('StaleContinuationParent')
        if adapter.binding['layer']!=self.layer or adapter.binding['next_layer']!=self.layer+1:
            raise ValueError('ContinuationSchedulePositionMismatch')
        if identity(adapter.binding)!=adapter.identity or adapter.binding['trajectory_owner']!=self.source_identity:
            raise ValueError('AlteredConsumerBinding')
        if (adapter.topology.triangles.tobytes()!=self.state.faces.tobytes() or
            adapter.front.coordinates.tobytes()!=self.state.first.tobytes() or
            adapter.front.source_ancestry!=self._ancestry()):
            raise ValueError('TrajectorySourceOwnershipMismatch')

    def _ancestry(self):
        if not hasattr(self,'_source_ancestry'):
            rows=[[] for _ in self.state.wall]
            for i,face in enumerate(self.state.faces):
                for v in face:rows[int(v)].append(i)
            self._source_ancestry=tuple(tuple(row) for row in rows)
        return self._source_ancestry

    def forecast_vertices(self,front,schedule,indices):
        if schedule is not self.schedule or front.committed_layer!=self.layer or front.geometry_authority!=self.geometry_identity:
            raise ValueError('DeclaredForecastBindingMismatch')
        positions=self.positions()
        if front.coordinates.tobytes()!=positions[self.layer].tobytes(): raise ValueError('StaleDeclaredForecastFront')
        from sdf_cfd_mesher_v4.forecast.descriptors import angle,unit
        rows={};previous=None;cumulative=np.zeros(len(indices));base=positions[self.layer,indices]
        for k in range(self.layer+1,len(positions)):
            x=positions[k,indices];delta=x-positions[k-1,indices];direction,length=unit(delta)
            declared=self.state.vectors[k-2,indices]
            if previous is None: previous=declared
            values=dict(coordinates=x,trajectory_direction=declared,displacement=delta,step_length=length,
                nominal_step=schedule.increments[k-1,indices],direction_change=angle(direction,previous),
                cumulative_displacement=np.linalg.norm(x-base,axis=1),path_length=cumulative+length,
                declared_target_valid=np.ones(len(indices),dtype=bool))
            for name,value in values.items(): rows.setdefault(name,[]).append(value)
            cumulative+=length;previous=direction
        return {k:np.stack(v) for k,v in rows.items()},0.


def bind_forecast(base,case,owner):
    """Bind an additional diagnostic trajectory, preserving the sealed request.

    Authenticated ordinary columns are inspected by the existing enrollment
    contract. No deep restart, changed parent authority, or production grant.
    """
    from production_forecast_admission import ProductionForecastAdapter
    from sdf_cfd_mesher_v4.geometry.front import CommittedFrontState,build_topology
    cls=ProductionForecastAdapter
    from composable_state import CommittedState
    from sealed_front_connection import load_case,inspect_front
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    if type(base) is not CommittedState:raise ValueError('QualifiedCommittedStateRequired')
    if type(owner) is not AcceptedTrajectoryOwner:raise ValueError('DeclaredTrajectoryOwnerRequired')
    base.check();owner.check()
    wall,tri,surface,request=load_case(case)
    inspected=inspect_front(base,wall,tri,surface,request,base.expected)
    if (base.expected['committed']!=owner.parent or base.layer!=owner.layer or
        surface['geometry_identity']!=owner.geometry_identity or
        surface['surface_identity']!=owner.record['surface']):raise ValueError('StaleContinuationParent')
    if (request['request_file_sha256']!=owner.record['original_request_file_sha256'] or
        identity(request['declaration'])!=owner.record['original_request_identity'] or
        request['schedule_identity']!=owner.record['original_schedule_identity']):raise ValueError('OriginalRequestMismatch')
    for left,right in ((wall,owner.state.wall),(tri,owner.state.faces),(inspected.front.coordinates,owner.state.first)):
        if left.shape!=right.shape or left.tobytes()!=right.tobytes():raise ValueError('AcceptedCommittedMappingMismatch')
    from pathlib import Path
    import json
    verify_source_binding(case,owner,wall,tri,surface)
    result=cls.__new__(cls);n=len(wall);layer=base.layer
    ids=tuple(f'layer:{layer}:source:{i}' for i in range(n));ancestry=[[] for _ in wall]
    for i,face in enumerate(tri):
        for v in face:ancestry[int(v)].append(i)
    # At this sealed L1 front the stored quantity is full normal height.
    # It is NOT an SDF distance; declared continuation never reads this field.
    result.front=CommittedFrontState.create(layer,inspected.front.coordinates,ids,tri,ancestry,
        np.full(n,owner.state.first_height),owner.geometry_identity,surface['surface_identity'])
    result.topology=build_topology(tri,n);result.schedule=owner.schedule
    result.committed_prefix=frozen(inspected.committed_prefix,'<f8')
    result.remaining_increments=owner.schedule.remaining(layer)
    result.sdf=owner;result.geometry_binding=owner
    result.trajectory_spec=dict(owner=owner.source_identity,semantics=owner.semantics,units='m')
    result.binding=dict(version='V4AcceptedFrontConsumerBinding/1',committed=owner.parent,
        request=identity(request['declaration']),request_file=request['request_file_sha256'],
        original_schedule=request['schedule_identity'],schedule=owner.schedule.sha256,
        continuation=identity(owner.record),trajectory_owner=owner.source_identity,
        quantity='path_length_m',height_scope='source_vertex',generation_authorized=False,
        geometry=owner.geometry_identity,surface=surface['surface_identity'],
        coordinates=result.front.coordinate_hash,topology=result.topology.sha256,
        ownership=identity(dict(vertex_ids=ids,source_ancestry=result.front.source_ancestry)),
        layer=layer,next_layer=layer+1)
    result.identity=identity(result.binding);owner.check_adapter(result,result.trajectory_spec)
    base.check();return result


def verify_source_binding(case,owner,wall,tri,surface):
    """Shared diagnostic/production check against the sealed source authority."""
    if (surface['geometry_identity']!=owner.geometry_identity or
            surface['surface_identity']!=owner.record['surface']):
        raise ValueError('TrajectoryGeometryAuthorityMismatch')
    for left,right in ((wall,owner.state.wall),(tri,owner.state.faces)):
        if left.shape!=right.shape or left.tobytes()!=right.tobytes():
            raise ValueError('AcceptedCommittedMappingMismatch')
    import json
    path=Path(case['surface'])/'REFERENCES.npz'
    replay=json.loads((Path(case['surface'])/'REPLAY.json').read_text(encoding='utf-8'))
    if file_hash(path)!=replay['replay_payloads']['REFERENCES.npz']:
        raise ValueError('ReferenceIdentityMismatch')
    with np.load(path,allow_pickle=False) as refs:
        if not np.array_equal(refs['face_refs'],owner.refs):
            raise ValueError('TrajectorySourceOwnershipMismatch')


def plan_transition(base,adapter,next_layer=None):
    """Typed schedule boundary. The authenticated reader stays byte-identical."""
    from composable_state import CommittedState,plan_transition as scalar_plan
    owner=getattr(adapter,'geometry_binding',None)
    if not isinstance(owner,AcceptedTrajectoryOwner): return scalar_plan(base,adapter,next_layer)
    if type(base) is not CommittedState: raise ValueError('QualifiedCommittedStateRequired')
    base.check();owner.check_adapter(adapter,adapter.trajectory_spec)
    if base.expected['committed']!=owner.parent or base.layer!=owner.layer: raise ValueError('StaleContinuationParent')
    target=base.layer+1
    if next_layer is not None and next_layer!=target: raise ValueError('NonSequentialTransition')
    if target>len(adapter.schedule.increments): raise ValueError('ScheduleComplete')
    if adapter.committed_prefix.shape!=(target,len(adapter.front.coordinates),3): raise ValueError('PrefixShapeMismatch')
    return dict(base_layer=base.layer,target_layer=target,base_root=base.root,base_content=base.content_root,
        base_scientific=base.scientific,schedule_index=target-1,
        increment_record=adapter.schedule.row_record(target),target=vars(adapter.schedule.targets[target-1]),
        binding=copy.deepcopy(adapter.binding),generation_authorized=False,
        forecast_horizon=list(range(target,len(adapter.schedule.increments)+1)),
        admission_run=False,forecast_run=False,geometry_generated=False)


def require_transition_authorization(adapter):
    if isinstance(getattr(adapter,'geometry_binding',None),AcceptedTrajectoryOwner):
        adapter.geometry_binding.check_adapter(adapter,adapter.trajectory_spec)
        raise ValueError('DiagnosticContinuationGenerationUnauthorized')
