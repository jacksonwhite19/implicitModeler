"""Authenticated AcceptedFrontState connection to the existing FastLayer.

Authorized ordinary transitions anchored to a sealed parent. Results are private,
unsealed memory candidates. This binding supplies no new prediction or proof.
"""
from pathlib import Path
import numpy as np
import copy
from accepted_production_contract import production_owner,require_transition_authorization,dependency_schedule
from accepted_production_contract import ProductionTrajectory,ProductionSchedule
from sealed_front_connection import predict_bound
from native_proof_segments import LoadedSegment,segment_objects
from sdf_cfd_mesher_v4.common.authority import file_hash,identity,frozen
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest


class QualityItems:
    """Explicit existing deep reader for one-time numeric bootstrap only."""
    def __init__(self,base):self.base=base
    def items(self):
        self.base.check()
        yield from self.base['quality'].items_deep()
        self.base.check()


class PrivateProductionTrajectory(ProductionTrajectory):
    """Existing geometry operations, owned by a successful MemoryTransaction.

    The parent is an explicit private content/generation receipt, NOT a sealed
    scientific state. No recursive publication, restart, or admission is implied.
    """
    def __init__(self,provider,receipt):
        anchor=provider.anchor_owner
        for name in ('original','authorization','authorization_identity','state',
                     'geometry_identity','units','record','refs','base'):
            setattr(self,name,getattr(anchor,name))
        self.seconds=0.
        self.provider=provider;self.private_receipt=receipt
        self.parent=identity(receipt);self.layer=provider.layer
        self.source_identity=identity(dict(root_owner=self.original.source_identity,
            authorization=self.authorization_identity,private_parent=self.parent,
            layer=self.layer,implementation=file_hash(__file__)))
        self.schedule=ProductionSchedule(self.original.schedule,self.source_identity,self.parent)
        self._schedule_targets=identity([vars(t) for t in self.schedule.targets])
        self.premises=None

    def _check_parent(self):
        self.provider.anchor_owner.check()
        self.provider.check_memory_parent()
        if (self.private_receipt is not self.provider.private_receipt or
            identity(self.private_receipt)!=self.parent or self.layer!=self.provider.layer):
            raise ValueError('StalePrivateTrajectoryParent')
        if self.layer not in self.authorization['authorized_layers']:
            raise ValueError('ContinuationOutsideAuthorizedLayers')

    def check_target(self,target):
        super().check_target(target)
        if target.layer!=self.layer+1 or target.layer not in self.authorization['authorized_layers']:
            raise ValueError('ContinuationOutsideAuthorizedLayers')


class AcceptedFastProvider:
    def __init__(self,base,adapter,folder,*,warm=False):
        if production_owner(adapter) is None:raise ValueError('ProductionTrajectoryRequired')
        require_transition_authorization(adapter)
        self.base=base;self.adapter=adapter;self.owner=adapter.geometry_binding
        if self.owner.base is not base:raise ValueError('DifferentAuthenticatedFastParent')
        self.x=adapter.committed_prefix[0];self.tri=adapter.topology.triangles
        self.schedule=adapter.schedule;self.spec=adapter.trajectory_spec
        self.folder=Path(folder);self.authority=dict(base['authority'])
        self.layer=base.layer
        self.warm=warm;self.session=None;self.anchor_owner=self.owner
        self.private_receipt=None;self.exhausted=False;self._prepare_before=None
        self.last_transition_owner=None
        self.snapshot=None
        self.binding=dict(schema='V4AcceptedFrontProductionFastBinding/1',
            parent=base.scientific,reference=base.reference,layer=self.layer,
            adapter=adapter.identity,authorization=self.owner.authorization_identity,
            geometry=self.owner.geometry_identity,quantity='path_length_m',units='m',
            owner=self.owner.source_identity,schedule=self.schedule.sha256,
            implementation=file_hash(__file__),warm_private_parent=warm,
            scope=('Authorized ordinary private MemoryTransaction generations' if warm else 'One authorized ordinary complete prism transition')+'; independent seal remains required')
        self.binding_identity=identity(self.binding)

    def check(self):
        self.base.check()
        if self.private_receipt is None:require_transition_authorization(self.adapter)
        else:
            self.anchor_owner.check();self.check_memory_parent()
            self.owner.check_adapter(self.adapter,self.spec)
        if identity(self.binding)!=self.binding_identity:raise ValueError('AlteredFastBinding')
        if self.anchor_owner.base is not self.base or self.anchor_owner.parent!=self.base.scientific:
            raise ValueError('ChangedFastParent')

    def attach_session(self,session):
        if not self.warm:return
        from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction
        if type(session) is not MemoryTransaction or session.current is not self.snapshot or session.generation!=0:
            raise ValueError('DifferentFastMemorySession')
        if self.session is not None:raise ValueError('FastSessionAlreadyAttached')
        self.session=session

    def check_memory_parent(self):
        if identity(self.private_receipt)!=self._private_receipt_identity:
            raise ValueError('AlteredPrivateParentReceipt')
        expected=self.snapshot if self._prepare_before is None else self._prepare_before
        generation=self.private_receipt['generation']-(self._prepare_before is not None)
        if (self.session is None or self.session.current is not expected or
            self.session.generation!=generation):
            raise ValueError('StaleFastMemoryGeneration')
        if self.private_receipt['layer']!=self.layer:
            raise ValueError('PrivateParentScheduleMismatch')

    def prepare_commit(self,before,candidate,generation,layer):
        """Validate the next binding BEFORE the existing private transaction.

        Keep a full component digest guard on the next submitted state. This is
        not a new trust/restart mechanism and does not authenticate a disk state.
        """
        if not self.warm:return None
        self.check()
        if (self.session is None or before is not self.snapshot or
            self.session.current is not before or self.session.generation!=generation or
            layer!=self.layer+1 or layer not in self.anchor_owner.authorization['authorized_layers']):
            raise ValueError('UncommittedFastPrivateParent')
        if candidate.get('_production_unsealed') is not True:
            raise ValueError('PrivateFastStateRequired')
        if candidate['authority']!=self.authority:raise ValueError('ChangedScientificImplementation')
        if any(candidate[name] is not self._receipt_handles[name] for name in self._receipt_handles):
            raise ValueError('ChangedFastReceiptHandle')
        digests={name:scientific_digest(value) for name,value in candidate.items() if name not in self._receipt_handles}
        receipt=dict(schema='V4PrivateFastParent/1',sealed_anchor=self.base.scientific,
            sealed_reference=self.base.reference,authorization=self.anchor_owner.authorization_identity,
            previous=(self.base.scientific if self.private_receipt is None else identity(self.private_receipt)),
            generation=generation+1,layer=layer,components=digests,
            durability='Unsealed MemoryTransaction generation; not a published scientific identity')
        prepared=copy.copy(self)
        prepared.snapshot=candidate;prepared._snapshot_digests=digests;prepared.layer=layer;prepared.private_receipt=receipt
        prepared._private_receipt_identity=identity(receipt);prepared._prepare_before=before
        prepared.exhausted=layer+1 not in self.anchor_owner.authorization['authorized_layers']
        prepared.last_transition_owner=self.owner
        owner=PrivateProductionTrajectory(prepared,receipt)
        adapter=copy.copy(self.adapter)
        from sdf_cfd_mesher_v4.geometry.front import CommittedFrontState
        n=len(self.x);ids=tuple(f'layer:{layer}:source:{i}' for i in range(n))
        adapter.committed_prefix=frozen(owner.positions()[:layer+1],'<f8')
        adapter.front=CommittedFrontState.create(layer,adapter.committed_prefix[-1],ids,self.tri,
            owner.original._ancestry(),np.full(n,owner.state.first_height),owner.geometry_identity,
            self.adapter.front.surface_authority)
        adapter.schedule=owner.schedule;adapter.remaining_increments=owner.schedule.remaining(layer)
        adapter.sdf=owner;adapter.geometry_binding=owner
        adapter.trajectory_spec=dict(owner=owner.source_identity,semantics=owner.semantics,units=owner.units)
        adapter.binding=dict(self.adapter.binding,version='V4AcceptedPrivateFastBinding/1',
            committed=owner.parent,parent_kind='private_memory_generation',private_parent=receipt,
            trajectory_owner=owner.source_identity,coordinates=adapter.front.coordinate_hash,
            ownership=identity(dict(vertex_ids=ids,source_ancestry=adapter.front.source_ancestry)),
            layer=layer,next_layer=layer+1)
        adapter.identity=identity(adapter.binding);owner.binding_identity=adapter.identity
        prepared.adapter=adapter;prepared.owner=owner;prepared.schedule=adapter.schedule;prepared.spec=adapter.trajectory_spec
        owner.check_adapter(adapter,prepared.spec)
        prepared.verify_snapshot(candidate,prepared.spec,self.folder,integrity=False)
        n=len(self.x)
        if len(candidate['vertices'])!=(layer+1)*n:raise ValueError('IncompleteFastVertexInventory')
        for key,p in candidate['vertices'].items():
            _,k,_,source=key.split(':');k=int(k);source=int(source)
            if (not 0<=k<=layer or not 0<=source<n or key!=f'layer:{k}:source:{source}' or
                np.asarray(p,dtype='<f8').tobytes()!=adapter.committed_prefix[k,source].tobytes()):
                raise ValueError('FastCommittedPrefixMismatch')
        return prepared

    def verify_prepared(self,prepared,candidate,generation):
        if not self.warm:return prepared is None
        if (type(prepared) is not AcceptedFastProvider or prepared._prepare_before is not self.snapshot or
            prepared.snapshot is not candidate or prepared.session is not self.session or
            self.session.current is not self.snapshot or self.session.generation!=generation):
            raise ValueError('UncommittedFastPrivateParent')
        prepared.check()
        return True

    def finish_commit(self,prepared):
        """Only publish already validated binding data; no rejecting checks here."""
        if not self.warm:return
        prepared._prepare_before=None
        self.__dict__.update(prepared.__dict__)
        self.owner.provider=self

    def materialize(self):
        """Bootstrap numeric/native caches once, not per production layer.

        Immutable proofs remain compact and quality rows remain streamed.
        Deep geometry copying is reported separately from hot-path timing.
        """
        self.check()
        if self.snapshot is not None:raise ValueError('FastBootstrapAlreadyMaterialized')
        state={}
        for name in self.base:
            if name=='certificate':state[name]=self.base[name]
            elif name=='quality':state[name]=QualityItems(self.base)
            else:
                value=self.base[name]
                state[name]=dict(value) if hasattr(value,'items') else value
        self.snapshot=state
        self._receipt_handles={name:state[name] for name in ('quality','certificate')}
        self._snapshot_digests={name:scientific_digest(value) for name,value in state.items()
                                if name not in self._receipt_handles}
        self.check()
        return state

    def verify_snapshot(self,state,spec,folder,*,integrity=True):
        self.check()
        if state is not self.snapshot or spec!=self.spec or Path(folder)!=self.folder:
            raise ValueError('DifferentFastSnapshot')
        if state['authority']!=self.authority:raise ValueError('ChangedScientificImplementation')
        if set(state)!=set(self._snapshot_digests)|set(self._receipt_handles):
            raise ValueError('ChangedFastSnapshotComponents')
        if any(state[name] is not handle for name,handle in self._receipt_handles.items()):
            raise ValueError('ChangedFastReceiptHandle')
        if state['quality'].base is not self.base:raise ValueError('ChangedFastReceiptOwner')
        if getattr(state['certificate'],'owner',self.base) is not self.base:
            raise ValueError('ChangedFastCertificateOwner')
        if integrity and any(scientific_digest(state[name])!=digest for name,digest in self._snapshot_digests.items()):
            raise ValueError('ChangedFastSnapshotContent')
        # No caller-edited capability declarations select this route.
        if state['events'] or any(c['kind']!='prism' for c in state['cells'].values()):
            raise ValueError('RequiresQualifiedMixedContinuationProvider')
        if (set(state['prism_front'])!=set(state['columns']) or
            any(c['last_prism_layer']!=self.layer for c in state['columns'].values())):
            raise ValueError('RequiresQualifiedLocalCohortProvider')

    def geometry(self,state,extra=0):
        # Full content check before advancement; constructor already checked
        # the bootstrap. Target validation uses the same private snapshot.
        if self._prepare_before is not None:raise ValueError('UncommittedFastPrivateParent')
        if extra and self.exhausted:raise ValueError('ContinuationOutsideAuthorizedLayers')
        self.verify_snapshot(state,self.spec,self.folder,integrity=bool(extra))
        if extra not in (0,1):raise ValueError('UnsupportedFastContinuationHorizon')
        n=len(self.x)
        if len(state['vertices'])!=(self.layer+1)*n:raise ValueError('IncompleteFastVertexInventory')
        levels=np.zeros(n,dtype=int)
        fronts=[np.empty_like(self.x) for _ in range(self.layer)]
        for key,p in state['vertices'].items():
            _,layer,_,source=key.split(':');layer=int(layer);source=int(source)
            if (not 0<=source<n or not 0<=layer<=self.layer
                or key!=f'layer:{layer}:source:{source}'):raise ValueError('FastVertexSourceRange')
            if layer==0:
                if np.asarray(p,dtype='<f8').tobytes()!=self.x[source].tobytes():raise ValueError('FastWallMismatch')
            else:fronts[layer-1][source]=p;levels[source]=max(levels[source],layer)
        if np.any(levels!=self.layer):raise ValueError('IncompleteFastCommittedPrefix')
        if np.asarray(fronts).tobytes()!=self.adapter.committed_prefix[1:].tobytes():
            raise ValueError('FastCommittedPrefixMismatch')
        if extra:
            target=self.schedule.targets[self.layer]
            predicted,validated=predict_bound(self.adapter,self.spec,fronts[-1],target,iteration_cap=256)
            if not validated['admitted']:raise ValueError('FastTrajectoryPremiseRejected')
            fronts.append(predicted.coordinates)
        return fronts,levels

    def validate_fast_target(self,state,points,layer):
        self.verify_snapshot(state,self.spec,self.folder,integrity=False)
        if layer!=self.layer+1:raise ValueError('FastParentRebindingRequired')
        return self.owner.validate_target(points,self.schedule.targets[layer-1])

    def fast_dependency_schedule(self):
        self.check()
        if self.private_receipt is not None:
            return [dict(self.schedule.row_record(k),target=vars(t)) for k,t in enumerate(self.schedule.targets,1)]
        return dependency_schedule(self.adapter)

    def load_fast_segments(self,indices):
        self.check()
        segments=[]
        for descriptor,directory,verify in self.base.proof_segments():
            for row in segment_objects(descriptor):verify(Path(directory)/row['file'])
            segments.append(LoadedSegment(descriptor,directory,indices,True))
        if sum(s.descriptor['count'] for s in segments)!=len(self.base['certificate']):
            raise ValueError('IncompleteAuthenticatedFastProofs')
        self.check()
        return segments
