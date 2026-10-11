"""Production enrollment of the preserved AcceptedFrontState operations.

Authorization is separate from trajectory premises and transition admission.
The original diagnostic owner and authenticated state readers stay unchanged.
"""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import accepted_trajectory_contract as diagnostic
from sdf_cfd_mesher_v4.common.authority import identity, array_hash, file_hash, frozen


def validate_authorization(record, expected, owner):
    owner.check()
    if identity(record)!=expected: raise ValueError('ContinuationAuthorizationIdentityMismatch')
    schema=record.get('schema')
    if schema not in ('V4WingContinuationUserAuthorization/1','V4ContinuationAuthorizationSuccessor/1',
                      'V4CurrentFrontContinuationAuthorization/1','V4MeshRangeAuthorization/1') or record.get('user_authorized') is not True:
        raise ValueError('ExplicitContinuationAuthorizationRequired')
    bindings=dict(diagnostic_contract=identity(owner.record), initial_committed_parent=owner.parent,
        original_request_sha256=owner.record['original_request_file_sha256'],
        original_request_identity=owner.record['original_request_identity'],
        trajectory_owner=owner.source_identity,recovered_schedule=owner.schedule.sha256,
        geometry=owner.geometry_identity,surface=owner.record['surface'],units=owner.units,
        first_full_normal_height_m=owner.state.first_height)
    if any(record.get(k)!=v for k,v in bindings.items()): raise ValueError('ContinuationAuthorizationBindingMismatch')
    layers=record['authorized_layers']
    starting_layer=owner.layer
    if schema=='V4CurrentFrontContinuationAuthorization/1':
        previous=record['history']
        validate_authorization(previous,record['history_identity'],owner)
        starting_layer=previous['authorized_layers'][-1]
        anchor=record['approved_parent']
        if (anchor.get('passed') is not True or type(anchor.get('layer')) is not int
            or anchor['layer']!=starting_layer or anchor['reference'].get('kind')!='recursive'
            or not isinstance(anchor.get('scientific'),str) or len(anchor['scientific'])!=64):
            raise ValueError('InvalidCurrentFrontAuthorizationAnchor')
    if schema=='V4ContinuationAuthorizationSuccessor/1':
        previous=record['predecessor']
        validate_authorization(previous,record['predecessor_identity'],owner)
        starting_layer=previous['authorized_layers'][-1]
        anchor=record['approved_parent']
        if (anchor.get('passed') is not True or anchor['layer']!=starting_layer
            or anchor['reference']['kind']!='recursive'
            or not isinstance(anchor['scientific'],str) or len(anchor['scientific'])!=64):
            raise ValueError('InvalidSuccessorAuthorizationAnchor')
    if (not isinstance(layers,list) or not layers or any(type(k) is not int for k in layers)
        or layers!=list(range(starting_layer+1,layers[-1]+1)) or layers[-1]>len(owner.schedule.increments)):
        raise ValueError('NonSequentialAuthorizationScope')
    if len(record['increments'])!=len(layers): raise ValueError('IncompleteAuthorizedSchedule')
    for layer,row in zip(layers,record['increments']):
        expected_row=owner.schedule.row_record(layer)
        if row['layer']!=layer or row['quantity']!='path_length_m': raise ValueError('AuthorizedQuantityMismatch')
        if expected_row['scope']=='source_vertex':
            # Authorize the checked, source-ordered row, not a scalar substitute.
            # The full schedule/mapping is authenticated by owner.check above.
            if row.get('scope')!='source_vertex':raise ValueError('SourceVertexAuthorizationRequired')
            if row!=expected_row:raise ValueError('AuthorizedVertexIncrementMismatch')
        elif expected_row['scope']=='global':
            if row.get('scope','global')!='global':raise ValueError('AuthorizedIncrementScopeMismatch')
            if row['binary64_hex']!=expected_row['uniform_hex'] or float(row['value_m']).hex()!=row['binary64_hex']:
                raise ValueError('AuthorizedIncrementMismatch')
        else:raise ValueError('UnsupportedAuthorizedIncrementScope')
    return identity(record)


def range_authorization(history, parent, through_layer, schedule, *, authorization):
    """Bind one explicit user range to existing exact schedule rows.

    Configuration only: this does not authenticate a front, admit a transition,
    or execute a layer. validate_authorization and CommittedState still do so.
    History and sealed receipts keep their original identities.
    """
    import copy
    start=parent.get('layer')
    if (type(start) is not int or type(through_layer) is not int
        or start<1 or not start<through_layer<=len(schedule.increments)
        or history.get('authorized_layers',[])[-1:]!=[start]):
        raise ValueError('InvalidRequestedLayerRange')
    if (parent.get('passed') is not True or parent.get('reference',{}).get('kind')!='recursive'
        or not isinstance(parent.get('scientific'),str) or len(parent['scientific'])!=64):
        raise ValueError('InvalidCurrentFrontAuthorizationAnchor')
    if not isinstance(authorization,str) or not authorization.strip():
        raise ValueError('ExplicitRangeRequestProvenanceRequired')
    layers=list(range(start+1,through_layer+1))
    rows=[]
    for layer in layers:
        row=schedule.row_record(layer)
        if row['layer']!=layer or row['quantity']!='path_length_m':raise ValueError('AuthorizedQuantityMismatch')
        if row['scope']=='global':
            rows.append(dict(layer=layer,quantity='path_length_m',binary64_hex=row['uniform_hex'],
                             value_m=float.fromhex(row['uniform_hex'])))
        elif row['scope']=='source_vertex':rows.append(copy.deepcopy(row))
        else:raise ValueError('UnsupportedAuthorizedIncrementScope')
    bindings=('diagnostic_contract','initial_committed_parent','original_request_sha256',
        'original_request_identity','trajectory_owner','recovered_schedule','geometry',
        'surface','units','first_full_normal_height_m')
    return dict({k:copy.deepcopy(history[k]) for k in bindings},
        schema='V4CurrentFrontContinuationAuthorization/1',user_authorized=True,
        approved_parent=copy.deepcopy(parent),history=copy.deepcopy(history),history_identity=identity(history),
        authorized_layers=layers,increments=rows,authorization=authorization,
        production_admission_passed=False,
        restrictions=['Stop at first required failure','No tuning, scientific retry or gate relaxation',
                      'This record is authorization, not admission or qualification evidence'])


def authorization_ancestry(record):
    """Layer-to-authorization ownership and exact successor parent anchors.

    Called only after validate_authorization. No old admission is relabelled.
    """
    layers={};anchors={};node=record
    while True:
        digest=identity(node)
        for layer in node['authorized_layers']:
            if layer in layers:raise ValueError('OverlappingAuthorizationScopes')
            layers[layer]=digest
        schema=node['schema']
        if schema=='V4ContinuationAuthorizationSuccessor/1':key,pin='predecessor','predecessor_identity'
        elif schema=='V4CurrentFrontContinuationAuthorization/1':key,pin='history','history_identity'
        elif schema in ('V4WingContinuationUserAuthorization/1','V4MeshRangeAuthorization/1'):break
        else:raise ValueError('UnsupportedAuthorizationAncestry')
        anchor=node['approved_parent'];anchors[anchor['layer']]=anchor
        if identity(node[key])!=node[pin]:
            raise ValueError('StalePredecessorAuthorization')
        node=node[key]
    return layers,anchors


class ProductionSchedule:
    def __init__(self, original, owner, parent):
        self.increments=original.increments; self.sha256=original.sha256
        self.source_sha256=original.source_sha256; self.first_height=original.first_height
        self.original=original
        self.targets=tuple(diagnostic.DeclaredTarget(owner,parent,k,'full_normal_height_m' if k==1 else 'path_length_m')
                           for k in range(1,len(self.increments)+1))
    def remaining(self,layer): return self.original.remaining(layer)
    def row_record(self,layer): return self.original.row_record(layer)


class ProductionTrajectory:
    semantics=diagnostic.AcceptedTrajectoryOwner.semantics
    # Reuse exact arithmetic and diagnostic geometry operations; no predictor copy.
    forecast_vertices=diagnostic.AcceptedTrajectoryOwner.forecast_vertices
    predict=diagnostic.AcceptedTrajectoryOwner.predict
    check_target=diagnostic.AcceptedTrajectoryOwner.check_target

    def __init__(self, original, authorization, authorization_identity, base):
        self.authorization_identity=validate_authorization(authorization,authorization_identity,original)
        self.original=original;self.authorization=copy.deepcopy(authorization)
        self.parent=base.scientific;self.layer=base.layer;self.state=original.state
        self.geometry_identity=original.geometry_identity;self.units=original.units
        self.record=original.record;self.refs=original.refs;self.seconds=0.
        self.base=base
        self._check_parent()
        self.source_identity=identity(dict(root_owner=original.source_identity,authorization=authorization_identity,
            parent=self.parent,layer=self.layer,implementation=file_hash(__file__)))
        self.schedule=ProductionSchedule(original.schedule,self.source_identity,self.parent)
        self._schedule_targets=identity([vars(t) for t in self.schedule.targets])
        self.premises=None

    def _check_parent(self, *, require_pending=True):
        self.base.check()
        if self.authorization.get('schema')=='V4CurrentFrontContinuationAuthorization/1':
            return self._check_current_front()
        successor=self.authorization.get('schema')=='V4ContinuationAuthorizationSuccessor/1'
        authorized,anchors=authorization_ancestry(self.authorization) if successor else ({}, {})
        seen=set()
        node=self.base
        while node.kind=='recursive':
            if node.layer in anchors:
                anchor=anchors[node.layer]
                if (node.scientific!=anchor['scientific'] or node.reference!=anchor['reference']):
                    raise ValueError('WrongSuccessorCommittedParent')
                seen.add(node.layer)
            if successor and node.parent.layer!=node.layer-1:raise ValueError('NonSequentialAuthorizationAncestry')
            path=Path(node.plan['output'])/'ADMISSION.json'
            admission=__import__('json').loads(node.verify(path).read_text(encoding='utf-8'))
            binding=admission['evidence']['binding']
            required=authorized.get(node.layer) if successor else self.authorization_identity
            if (required is None or binding.get('authorization')!=required or
                binding.get('trajectory_root')!=self.original.source_identity or
                binding['next_layer']!=node.layer or not admission['decision']['generation_attempt_authorized']):
                raise ValueError('UnboundContinuationAncestry')
            node=node.parent
        if successor and set(anchors)-seen:raise ValueError('MissingSuccessorAuthorizationAnchor')
        if node.scientific!=self.original.parent or node.layer!=1: raise ValueError('StaleContinuationParent')
        if self.base.scientific!=self.parent or self.base.layer!=self.layer: raise ValueError('StaleContinuationParent')
        if require_pending and self.layer+1 not in self.authorization['authorized_layers']: raise ValueError('ContinuationOutsideAuthorizedLayers')

    def _check_current_front(self, *, require_pending=True):
        """Use a newly authorized authenticated front; never rewrite old receipts.

        Historical authorization identities still own every past admission. Old
        access anchors are history, not the access reference for this new grant.
        Recovery is separately proved under guard by the existing recovery scope.
        """
        from recursive_epoch_recovery import component_source
        anchor=self.authorization['approved_parent']
        authorized,_=authorization_ancestry(self.authorization['history'])
        authorized.update({k:self.authorization_identity for k in self.authorization['authorized_layers']})
        node=self.base;seen=False
        while node.kind=='recursive':
            if node.parent.layer!=node.layer-1:raise ValueError('NonSequentialAuthorizationAncestry')
            if node.layer==anchor['layer']:
                if node.reference!=anchor['reference'] or node.scientific!=anchor['scientific']:
                    raise ValueError('WrongCurrentFrontAuthorizationParent')
                seen=True
            component_source(node.body,node.kind)
            out=Path(node.plan['output'])
            admission=__import__('json').loads(node.verify(out/'ADMISSION.json').read_text(encoding='utf-8'))
            receipt=__import__('json').loads(node.verify(Path(node.plan['continuation_contract'])).read_text(encoding='utf-8'))
            required=authorized.get(node.layer)
            validate_authorization(receipt['authorization'],receipt['identity'],self.original)
            binding=admission['evidence']['binding']
            if (required is None or receipt['identity']!=required or binding.get('authorization')!=required
                or receipt['root_owner']!=self.original.source_identity
                or receipt['diagnostic']!=identity(self.original.record)
                or receipt['parent']!=node.parent.scientific
                or receipt['plan']['target_layer']!=node.layer
                or binding.get('trajectory_root')!=self.original.source_identity
                or binding.get('committed')!=node.parent.scientific
                or binding.get('next_layer')!=node.layer
                or admission['decision'].get('generation_attempt_authorized') is not True):
                raise ValueError('UnboundCurrentFrontAuthorizationAncestry')
            node=node.parent
        if not seen:raise ValueError('MissingCurrentFrontAuthorizationParent')
        if node.scientific!=self.original.parent or node.layer!=1:raise ValueError('StaleContinuationParent')
        if self.base.scientific!=self.parent or self.base.layer!=self.layer:raise ValueError('StaleContinuationParent')
        if require_pending and self.layer+1 not in self.authorization['authorized_layers']:raise ValueError('ContinuationOutsideAuthorizedLayers')

    def check(self):
        self.original.check()
        if identity(self.authorization)!=self.authorization_identity: raise ValueError('AlteredProductionAuthorization')
        if self.schedule.sha256!=self.original.schedule.sha256 or self.schedule.increments is not self.original.schedule.increments:
            raise ValueError('AlteredProductionSchedule')
        if identity([vars(t) for t in self.schedule.targets])!=self._schedule_targets: raise ValueError('AlteredProductionTargets')
        self._check_parent()

    def positions(self): self.check();return self.original.positions()

    def check_adapter(self,adapter,spec):
        self.check()
        if adapter.geometry_binding is not self or adapter.sdf is not self or adapter.schedule is not self.schedule:
            raise ValueError('DifferentForecastGenerationTrajectoryOwner')
        if spec!=dict(owner=self.source_identity,semantics=self.semantics,units=self.units): raise ValueError('DeclaredTrajectorySpecMismatch')
        if identity(adapter.binding)!=adapter.identity or adapter.identity!=self.binding_identity or adapter.binding['committed']!=self.parent:
            raise ValueError('AlteredProductionBinding')
        if adapter.binding['layer']!=self.layer or adapter.binding['next_layer']!=self.layer+1:
            raise ValueError('ContinuationSchedulePositionMismatch')
        # One original, checked snapshot supplies both comparisons. Do not
        # repeat production ancestry walks for each read of this same snapshot.
        positions=self.original.positions()
        if (adapter.topology.triangles.tobytes()!=self.state.faces.tobytes() or
            adapter.committed_prefix.tobytes()!=positions[:self.layer+1].tobytes() or
            adapter.front.coordinates.tobytes()!=positions[self.layer].tobytes() or
            adapter.front.source_ancestry!=self.original._ancestry()):raise ValueError('TrajectorySourceOwnershipMismatch')

    def trajectory_premises(self):
        self.check()
        if self.premises is None:
            from live_spline_first_layer import branch_evidence
            from general_sdf_authority import GeneralSdfAuthority
            if type(self.original.authority) is GeneralSdfAuthority:
                positions=self.positions()
                proof=self.original.authority.qualify_segments(positions[self.layer],positions[self.layer+1])
                self.premises=dict(passed=proof['passed'],branch=proof,owner=self.source_identity,parent=self.parent,
                    layer=self.layer+1,target=vars(self.schedule.targets[self.layer]),
                    scope='Exact real-source strict field progress and positive endpoints for actual vertex segments; existing cell/contact/volume checks remain required.')
                self._premises_identity=identity(self.premises)
                return copy.deepcopy(self.premises)
            branch=branch_evidence(self.original.authority)
            path=diagnostic.ROOT/next(r['path'] for r in self.record['sources'] if r['path'].endswith('/check_complete_geometry.py'))
            expected=next(r['sha256'] for r in self.record['sources'] if r['path'].endswith('/check_complete_geometry.py'))
            if file_hash(path)!=expected: raise ValueError('ContinuationPremiseSourceMismatch')
            # Reuse the accepted continuation's normal_delta expression exactly.
            nodes=[n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.Assign)
                   and any(isinstance(t,ast.Name) and t.id in ('pairs','normal_delta') for t in n.targets)]
            if len(nodes)!=2:raise ValueError('ExistingTrajectoryPremiseStructureChanged')
            ns=dict(np=np,state=self.state,x=self.positions()[self.layer:self.layer+2])
            exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),ns)
            values=ns['normal_delta'][0]
            bad=np.flatnonzero(~np.isfinite(values)|(values<=0))
            self.premises=dict(passed=bool(branch['passed'] and not len(bad)),branch=branch,
                source=file_hash(path),owner=self.source_identity,parent=self.parent,layer=self.layer+1,
                target=vars(self.schedule.targets[self.layer]),equations=len(values),
                minimum_source_normal_increment_m=float(values.min()),
                failed_incident_pairs=self.state.pairs[bad].tolist(),
                scope='Existing convex-source supporting-normal outward vertex-segment premise. Not a cell, swept-side/contact, degree or volume certificate.',
                required_remaining='Existing exact wedge, direct affected incidence, continuous face collisions, full realized degree/volume and transaction qualification')
            self._premises_identity=identity(self.premises)
        if identity(self.premises)!=self._premises_identity:raise ValueError('AlteredTrajectoryPremises')
        return copy.deepcopy(self.premises)

    def validate_target(self,points,target):
        self.check_target(target)
        if target.layer!=self.layer+1:raise ValueError('NonSequentialTransition')
        positions=self.positions()
        if np.shape(points)!=positions[target.layer].shape or np.asarray(points).tobytes()!=positions[target.layer].tobytes():
            raise ValueError('DeclaredTargetCoordinatesMismatch')
        premise=self.trajectory_premises()
        return dict(admitted=premise['passed'],scope=premise['scope'],premises=premise,
            owner=self.source_identity,target=vars(target),coordinates=array_hash(points),
            authorization=self.authorization_identity,not_transition_qualification=True)


def bind_production(base,case,original,authorization,authorization_identity):
    from composable_state import CommittedState
    from production_forecast_admission import ProductionForecastAdapter
    from sealed_front_connection import load_case, inspect_front
    from sdf_cfd_mesher_v4.geometry.front import CommittedFrontState,build_topology
    if type(base) is not CommittedState:raise ValueError('QualifiedCommittedStateRequired')
    wall,tri,surface,request=load_case(case)
    diagnostic.verify_source_binding(case,original,wall,tri,surface)
    owner=ProductionTrajectory(original,authorization,authorization_identity,base)
    # Existing inventory/ownership inspector needs only the authorized horizon
    # here. Original request/schedule authority fields remain the sealed L1 ones.
    inspection_scope=dict(request,increments=original.schedule.increments[:max(authorization['authorized_layers'])])
    inspected=inspect_front(base,wall,tri,surface,inspection_scope,base.expected)
    if inspected.committed_prefix.tobytes()!=owner.positions()[:base.layer+1].tobytes():raise ValueError('CommittedDeclaredPrefixMismatch')
    if request['request_file_sha256']!=authorization['original_request_sha256']:raise ValueError('OriginalRequestMismatch')
    n=len(wall);ids=tuple(f'layer:{base.layer}:source:{i}' for i in range(n))
    adapter=ProductionForecastAdapter.__new__(ProductionForecastAdapter)
    adapter.front=CommittedFrontState.create(base.layer,inspected.front.coordinates,ids,tri,original._ancestry(),
        np.full(n,original.state.first_height),owner.geometry_identity,surface['surface_identity'])
    adapter.topology=build_topology(tri,n);adapter.schedule=owner.schedule
    adapter.committed_prefix=frozen(inspected.committed_prefix,'<f8');adapter.remaining_increments=owner.schedule.remaining(base.layer)
    adapter.sdf=owner;adapter.geometry_binding=owner
    adapter.trajectory_spec=dict(owner=owner.source_identity,semantics=owner.semantics,units='m')
    adapter.binding=dict(version='V4AcceptedFrontProductionBinding/1',committed=base.scientific,
        request=identity(request['declaration']),request_file=request['request_file_sha256'],
        original_schedule=request['schedule_identity'],schedule=owner.schedule.sha256,
        continuation=identity(original.record),trajectory_root=original.source_identity,trajectory_owner=owner.source_identity,
        authorization=authorization_identity,quantity='path_length_m',height_scope='source_vertex',
        generation_authorized=True,geometry=owner.geometry_identity,surface=surface['surface_identity'],
        coordinates=adapter.front.coordinate_hash,topology=adapter.topology.sha256,
        ownership=identity(dict(vertex_ids=ids,source_ancestry=adapter.front.source_ancestry)),layer=base.layer,next_layer=base.layer+1)
    adapter.identity=identity(adapter.binding);owner.binding_identity=adapter.identity
    owner.check_adapter(adapter,adapter.trajectory_spec)
    return adapter,inspected


def production_owner(adapter):
    owner=getattr(adapter,'geometry_binding',None)
    return owner if type(owner) is ProductionTrajectory else None


def require_transition_authorization(adapter):
    owner=production_owner(adapter)
    if owner is None:return diagnostic.require_transition_authorization(adapter)
    if owner.layer+1 not in owner.authorization['authorized_layers']:
        raise ValueError('ContinuationOutsideAuthorizedLayers')
    owner.check_adapter(adapter,adapter.trajectory_spec)


def verify_committed_ancestry(base,original,authorization):
    """Read-only use of the same ancestry verifier, not a production owner."""
    original.check()
    pin=validate_authorization(authorization,identity(authorization),original)
    view=SimpleNamespace(base=base,original=original,authorization=authorization,
        authorization_identity=pin,parent=base.scientific,layer=base.layer)
    view._check_current_front=lambda:ProductionTrajectory._check_current_front(view,require_pending=False)
    ProductionTrajectory._check_parent(view,require_pending=False)


def plan_transition(base,adapter,next_layer=None):
    owner=production_owner(adapter)
    if owner is None:return diagnostic.plan_transition(base,adapter,next_layer)
    require_transition_authorization(adapter)
    if base is not owner.base:raise ValueError('DifferentAuthenticatedParent')
    target=base.layer+1
    if next_layer is not None and next_layer!=target:raise ValueError('NonSequentialTransition')
    return dict(base_layer=base.layer,target_layer=target,base_root=base.root,base_content=base.content_root,
        base_scientific=base.scientific,schedule_index=target-1,increment_record=owner.schedule.row_record(target),
        target=vars(owner.schedule.targets[target-1]),binding=copy.deepcopy(adapter.binding),
        generation_authorized=True,authorization=owner.authorization_identity,
        forecast_horizon=list(range(target,len(owner.schedule.increments)+1)),
        admission_run=False,forecast_run=False,geometry_generated=False)


def dependency_schedule(adapter):
    if production_owner(adapter) is None:return adapter.schedule.targets.tolist()
    return [dict(adapter.schedule.row_record(k),target=vars(t)) for k,t in enumerate(adapter.schedule.targets,1)]


def seal_publication(base,adapter,out,plan):
    """Add contract evidence to the existing authenticated publication references."""
    owner=production_owner(adapter)
    if owner is None:return plan
    from path_preserving_deduplication import durable_json
    import immutable_state_access as access
    require_transition_authorization(adapter)
    path=out/'CONTINUATION_CONTRACT.json'
    durable_json(path,dict(authorization=owner.authorization,identity=owner.authorization_identity,
        diagnostic=identity(owner.record),root_owner=owner.original.source_identity,
        parent=base.scientific,plan=plan_transition(base,adapter),implementation=file_hash(__file__)))
    plan['references'][str(path.resolve())]=access.attest(path,file_hash(path))
    plan['continuation_contract']=str(path.resolve())
    return plan
