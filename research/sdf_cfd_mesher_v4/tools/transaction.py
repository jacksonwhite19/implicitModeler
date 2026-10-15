"""Prepare one coherent private changed-point/connectivity state; no publisher."""
import numpy as np
from background_materialization import build
from background_topology_repair import key
from prism_hotpath_v2 import reverse_cycle
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,frozen
from shape_transition_domain import verify


def prepare(state,patch,candidate,quality_result,*,membership,check,_return_mapping=False):
    """No mutation of state. Validate local edit, then map all fresh geometry.

    Source containers are exclusively owned and never edited during this call.
    This does not grant a LiveAcceptance or scientific replay credential.
    The caller must run fresh whole-stage quality and ordinary admit.
    """
    if type(_return_mapping) is not bool:raise ValueError('ExplicitPreparedMappingRequestRequired')
    check()
    # This is one owned in-memory call, not a deserialized edit credential.
    # An alternate state cannot consume a quality result from another source.
    for name in ('points','cells','additions','owners','outer','original','cap_ids',
                 'cap_faces','compact_ids','vertex_envelope','old_face_keys','controls'):
        if state[name] is not patch.source[name]:raise ValueError('ChangedPreparedSourceState',name)
    if state.get('bounds') is not patch.source.get('bounds'):raise ValueError('ChangedPreparedSourceState','bounds')
    if state['old_point_count']!=patch.source['old_point_count']:raise ValueError('ChangedPreparedSourcePrefix')
    admission=patch.validate(candidate,quality_result,membership=membership)
    original_n=state['old_point_count'];old_active={c['id'] for c in state['additions']}
    if not patch.ids<=old_active:raise ValueError('CoupledEditOutsideFreshBand')
    old=[c for c in state['cells'] if c['id'] not in old_active]
    if any(any(v>=original_n for v in c['vertices']) for c in old):raise ValueError('InheritedCellUsesFreshSite')
    retained=[c for c in state['additions'] if c['id'] not in patch.ids]
    additions=retained+patch.cells
    ids=[c['id'] for c in additions]
    if len(ids)!=len(set(ids)) or set(ids)&{c['id'] for c in old}:raise ValueError('AliasedPreparedCellIds')
    rows=np.asarray([c['vertices'] for c in additions],dtype='<i8')
    used=np.zeros(len(candidate),dtype=bool);used[rows.ravel()]=True;used[np.asarray(state['outer']).ravel()]=True
    fresh=np.flatnonzero(used & (np.arange(len(candidate))>=original_n))
    keep=np.r_[np.arange(original_n),fresh].astype('<i8');mapping=np.full(len(candidate),-1,dtype='<i8');mapping[keep]=np.arange(len(keep))
    if np.any(mapping[rows]<0):raise ValueError('RetiredUsedFreshVertex')
    newrows=mapping[rows];rebuilt=build(newrows,0)
    for previous,current in zip(additions,rebuilt):
        current.update({k:v for k,v in previous.items() if k not in ('vertices','faces')})
    points=np.asarray(candidate[keep],dtype='<f8');outer=np.asarray(mapping[state['outer']],dtype='<i8')
    if points[:original_n].tobytes()!=state['original'].tobytes():raise ValueError('ChangedPreparedPrismPrefix')
    cap_ids=np.asarray(state['cap_ids'],dtype='<i8');env=patch.envelope_state['vertex_envelope']
    offset=fresh-original_n
    if len(env['triangle_ids'])!=len(candidate)-original_n:raise ValueError('IncompletePreparedEnvelope')
    envelope=dict(vertex_ids=frozen(np.arange(len(cap_ids),len(cap_ids)+len(fresh),dtype='<i8'),'<i8'),
        triangle_ids=frozen(np.asarray(env['triangle_ids'])[offset],'<i8'),
        weights=frozen(np.asarray(env['weights'])[offset],'<f8'))
    compact=np.r_[cap_ids,np.arange(original_n,len(points))].astype('<i8')
    domain=verify(points[compact],state['original'][cap_ids],state['cap_faces'],state['controls']['maximum_distance_m'],envelope,check)
    owners={}
    for c in old+rebuilt:
        for f in c['faces']:owners.setdefault(key(f),[]).append((c['id'],f))
    physical={key(f) for f in outer}
    for k,uses in owners.items():
        if len(uses)==1:
            # Existing physical wall/prism boundaries are inherited. New
            # singleton faces must be exactly the declared transition outer.
            if k not in state['old_face_keys'] and k not in physical:raise ValueError('UnexpectedPreparedBoundaryFace')
        elif len(uses)!=2 or not reverse_cycle(uses[0][1],uses[1][1]):raise ValueError('InvalidPreparedOwnerIncidence')
    inherited={c['id'] for c in old};fresh_ids=set(ids)
    for f in state['cap_ids'][state['cap_faces']]:
        uses=owners[key(f)]
        if len(uses)!=2 or sum(i in fresh_ids for i,_ in uses)!=1:raise ValueError('ChangedPreparedCapAttachment')
        expected=[u for u in state['owners'][key(f)] if u[0] in inherited]
        if [u for u in uses if u[0] in inherited]!=expected or len(expected)!=1:raise ValueError('ChangedInheritedCapOwner')
    construction=dict(state['construction'],coupled_collar=dict(
        removed_cell_ids=sorted(patch.ids),added_cell_ids=sorted(patch.active),
        moved_old_sites=len([v for v in patch.moving if v<len(state['points'])]),
        inserted_sites=len(patch.new_site_ids),retired_unused_fresh_sites=int(len(candidate)-original_n-len(fresh)),
        local_admission_identity=identity(admission),points_sha256=array_hash(points),
        tets_sha256=array_hash(newrows),proposal_only=True))
    result=dict(state,points=points,cells=old+rebuilt,additions=rebuilt,owners=owners,outer=outer,
        compact_ids=compact,vertex_envelope=envelope,construction=construction)
    # Enriched source states carry indices into their old cell/point namespace.
    # Rebuild those indices after retirement and compaction before another
    # controller/native request can consume the prepared state.
    if any(name in state for name in ('byid','editable_ids','stars','physical_outer_keys','physical_outer_vertices')):
        byid={c['id']:c for c in result['cells']};stars={}
        if len(byid)!=len(result['cells']):raise ValueError('AliasedPreparedSourceCellIds')
        for j,c in enumerate(rebuilt):
            if j%1024==0:check()
            for v in c['vertices']:
                if v>=original_n:stars.setdefault(v,[]).append(c['id'])
        result.update(byid=byid,editable_ids=set(ids),stars=stars,
            physical_outer_keys=physical,physical_outer_vertices=set(map(int,outer.ravel())))
    result.pop('topology_construction',None)
    if result['old_face_keys'] is not state['old_face_keys']:raise ValueError('ChangedInheritedCollisionScope')
    check()
    receipt=dict(passed=True,admission=admission,vertex_envelope=domain,
        original_prism_points_and_cells_unchanged=True,inherited_old_face_keys_unchanged=True,
        old_points=len(state['points']),new_points=len(points),old_tets=len(state['additions']),new_tets=len(rebuilt),
        scientific_mesh_acceptance=False,scope='Prepared private coherent state. Fresh whole QA/admit and publication remain mandatory.')
    if _return_mapping:return result,receipt,frozen(mapping,'<i8')
    return result,receipt
