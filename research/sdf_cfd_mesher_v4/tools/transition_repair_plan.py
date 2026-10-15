"""Private same-call scheduling and diagnostics; no acceptance predicate.

The synchronous placement owner exclusively owns mutable state. References and
generations register controlled edits, not arbitrary mutation detection. Local
custody/exact owners stay mandatory; final independent whole QA is authoritative.
No file, saved report, or externally supplied quality can issue this scope.
"""
from contextvars import ContextVar
from contextlib import contextmanager
from copy import deepcopy
from collections import Counter
import numpy as np
from background_core_repair import atoms,affected_rows,replace_rows
from refined_local_repair import record_key
from background_topology_repair import key
from sdf_cfd_mesher_v4.common.authority import identity,array_hash

_active=ContextVar('private_transition_quality_scope',default=None)
_FIELDS=('points','cells','additions','owners','outer','controls','original','cap_ids','cap_faces',
         'compact_ids','vertex_envelope','old_face_keys','bounds')

def diagnostic_plain(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,dict):return {diagnostic_plain(k):diagnostic_plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [diagnostic_plain(v) for v in value]
    return value

class _QualitySession:
    def __init__(self,state,initial):
        self.generation=0;self.live=True;self.edits=0
        self.controls=identity(state['controls']);self.outer=array_hash(np.asarray(state['outer']))
        self.prefix=state['old_point_count'];self._bind(state,initial['failures'])

    def _bind(self,state,failures):
        self.refs=tuple(state.get(k) for k in _FIELDS)
        self.current=dict(passed=not failures,failures=deepcopy(failures),
            diagnostic_failure_inventory=True,scientific_mesh_acceptance=False,
            _repair_generation=self.generation)
        if len({record_key(r) for r in failures})!=len(failures):raise ValueError('AliasedRepairFailureRows')
        self.stamp=identity(self.current)

    def require(self,state,q=None):
        if not self.live or _active.get() is not self:raise ValueError('ExpiredOrForeignRepairQualityScope')
        if any(state.get(k) is not v for k,v in zip(_FIELDS,self.refs)):raise ValueError('ChangedRepairStateGeneration')
        if state['old_point_count']!=self.prefix or identity(state['controls'])!=self.controls:
            raise ValueError('ChangedRepairControlsOrPrefix')
        if array_hash(np.asarray(state['outer']))!=self.outer:raise ValueError('ChangedRepairProspectiveOuter')
        if identity(self.current)!=self.stamp or q is not None and q is not self.current:
            raise ValueError('ChangedRepairQualityGeneration')
        return self.current

    def advance(self,state,failures,*,changed=True):
        if not self.live or _active.get() is not self:raise ValueError('ExpiredOrForeignRepairQualityScope')
        self.generation+=1;self.edits+=bool(changed);self._bind(state,failures)
        # Compaction can change outer indices, never its physical geometry.
        self.outer=array_hash(np.asarray(state['outer']))
        return self.current

    def before(self,state,q,ids,faces,local):
        self.require(state,q)
        selected=affected_rows({record_key(r):r for r in q['failures']},set(ids),set(faces))
        if atoms(dict(failures=selected))!=atoms(local):raise ValueError('RepairAffectedInventoryMismatch')

    def coordinate_commit(self,state,q,ids,faces,local):
        self.require(state,q)
        reports={record_key(r):r for r in q['failures']}
        replace_rows(reports,set(ids),set(faces),local['failures'])
        return self.advance(state,list(reports.values()))

@contextmanager
def quality_session(state,initial):
    session=_QualitySession(state,initial);token=_active.set(session)
    try:yield session
    finally:
        session.live=False;session.current=None;session.refs=();_active.reset(token)

def failure_closure(state,q):
    """Small diagnostic geometry only; not a restart or proof credential."""
    byid={c['id']:c for c in state['cells']}
    seeds={i for r in q['failures'] for i in (r['owners'] if r['kind']=='face_quality' else [r['id']])}
    free={v for i in seeds for v in byid[i]['vertices'] if v>=state['old_point_count']}
    affected=seeds|{c['id'] for c in state['additions'] if free.intersection(c['vertices'])}
    faces={key(f) for i in affected for f in byid[i]['faces']}
    needed=affected|{i for f in faces for i,_ in state['owners'][f]}
    vertices=sorted({v for i in needed for v in byid[i]['vertices']})
    return dict(failure_records=len(q['failures']),failure_atoms=len(atoms(q)),
        metric_counts=dict(Counter(a[-1] for a in atoms(q))),failures=deepcopy(q['failures']),
        affected_cells=sorted(affected),total_closure_cells=len(needed),
        cells=[dict(id=i,kind=byid[i]['kind'],vertices=list(map(int,byid[i]['vertices'])),
            faces=[list(map(int,f)) for f in byid[i]['faces']]) for i in sorted(needed)],
        vertices=[dict(id=int(v),point=list(map(float,state['points'][v]))) for v in vertices],
        owners=[dict(face=list(f),uses=[dict(id=int(i),face=list(map(int,face))) for i,face in state['owners'][f]]) for f in sorted(faces)],
        prospective_outer=[list(map(int,f)) for f in state['outer'] if key(f) in faces],
        old_point_count=state['old_point_count'],controls=deepcopy(state['controls']),
        scientific_mesh_acceptance=False,restart_supported=False)

def stage_diagnostic(check,stage,state,q,**extra):
    emit=getattr(check,'repair_diagnostics',None)
    if emit is not None:
        from repair_timing import RepairProgress
        progress=RepairProgress.from_check(check)
        with progress.span('diagnostic.prepare'):payload=dict(failure_closure(state,q),**extra)
        emit(stage,payload)

def constraint_labels(cells,owners):
    labels=[dict(kind='cell_'+m,cells=[c['id']]) for c in cells for m in ('volume','determinant','aspect')]
    paired=[(f,u) for f,u in sorted(owners.items()) if len(u)==2]
    for m in ('weight','volume_ratio','skewness'):
        labels.extend(dict(kind='face_'+m,cells=[i for i,_ in u],face=list(f)) for f,u in paired)
    return labels

def group_diagnostic(state,row,cells,owners,selected,outer,labels,initial):
    """Exact current local coordinates/roles before a guided solver call."""
    vertices=sorted({v for c in cells for v in c['vertices']})
    envelope=[]
    for v in row:
        j=v-state['old_point_count'];ti=int(state['vertex_envelope']['triangle_ids'][j])
        cap_ids=state['cap_ids'][state['cap_faces'][ti]]
        envelope.append(dict(vertex=int(v),cap_vertex_ids=cap_ids.tolist(),
            cap=state['original'][cap_ids].tolist(),weights=state['vertex_envelope']['weights'][j].tolist()))
    return diagnostic_plain(dict(moving_vertices=list(row),selected_cells=sorted(selected),
        cells=[dict(id=c['id'],kind=c['kind'],vertices=list(map(int,c['vertices'])),
            faces=[list(map(int,f)) for f in c['faces']]) for c in cells],
        vertices=[dict(id=int(v),point=state['points'][v].tolist()) for v in vertices],
        owners=[dict(face=list(f),uses=[dict(id=int(i),face=list(face)) for i,face in uses]) for f,uses in sorted(owners.items())],
        prospective_outer=[list(f) for f in outer],envelope=envelope,
        bounds=None if state['bounds'] is None else state['bounds'].tolist(),
        controls=deepcopy(state['controls']),old_point_count=state['old_point_count'],
        ordinary_labels=labels,initial_constraints=initial.tolist(),
        scientific_mesh_acceptance=False,restart_supported=False))

def guided_vertex(row,labels,values,byid,prefix):
    """Rank proposal-bound slack, add ONE eligible site; no new tolerance.

    This is a proposal heuristic, not a proof that the selected row is active
    or that the enlarged support is feasible. Prefix restrictions stay exact.
    """
    if len(values)<len(labels):raise ValueError('IncompleteGuidedConstraintVector')
    values=np.asarray(values[:len(labels)])
    if not np.isfinite(values).all():return None,dict(reason='NonfiniteGuidedProposal')
    for index in np.argsort(values,kind='stable'):
        label=labels[int(index)]
        eligible=sorted({v for i in label['cells'] for v in byid[i]['vertices'] if v>=prefix}-set(row))
        if eligible:
            return eligible[0],dict(label=label,slack=float(values[index]),
                reason='LeastSlackOwnerWithAdditionalFreeSite',proposal_only=True)
    return None,dict(reason='NoAdditionalFreeConstraintOwner',proposal_only=True)
