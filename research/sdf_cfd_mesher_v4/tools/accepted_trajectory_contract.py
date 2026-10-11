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


class AcceptedTrajectoryOwner:
    semantics='AcceptedFrontState/path_length_m/1'
    generation_authorized=False

    @classmethod
    def from_inputs(cls, authority, wall, first, faces, refs, increments, *, parent,
                    surface, request, case_identity):
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
        return cls(authority,record,identity(record),wall,first,faces,refs,mapping,selected,parent=parent)

    def __init__(self, authority, record, record_identity, wall, first, faces, refs, mapping, selected, *, parent, layer=1, units='m'):
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
        self.state=state_class()(wall,first,faces,keys,normals,incidents,
            np.broadcast_to(later,(len(self.schedule.increments)-1,*later.shape)),self.schedule.first_height,
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
        self._integrity=self.integrity()
        self._positions=None
        self.seconds=0.

    def _integrity_inputs(self):
        # Original payload before serialization/hashing. The scoped comparison
        # keeps these mutable fields and exact array bytes bound to that payload.
        return dict(record=self.record,source_identity=self.source_identity,parent=self.parent,layer=self.layer,
            geometry=self.geometry_identity,units=self.units,schedule_identity=self.schedule.sha256,
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
            geometry=self.geometry_identity,units=self.units,schedule_identity=self.schedule.sha256,
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
