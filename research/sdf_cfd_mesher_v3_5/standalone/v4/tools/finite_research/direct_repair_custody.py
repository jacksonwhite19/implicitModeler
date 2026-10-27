"""Bind one generic repair to final joined evidence and independently replay moves."""
import numpy as np
from finite_research.direct_evidence_modules import driver
from finite_research.direct_fill_contract import binding
from sdf_cfd_mesher_v4.common.authority import array_hash,identity

def apply(folder,joined,prefix,additions,policy,check):
    from finite_research import direct_shared_apex as repair
    from path_preserving_deduplication import durable_json
    arrays=dict(points=joined['points'],background_tets=joined['bulk_tets'],
        prisms=np.asarray([c['vertices'] for c in prefix['cells']],dtype='<i8'),
        prism_ids=np.asarray(joined['prefix_ids'],dtype='<i8'),
        tetrahedron_ids=np.asarray([c['id'] for c in additions],dtype='<i8'),
        wall=joined['wall'],cap=joined['prefix_front'],farfield=joined['farfield'])
    check();points,result=repair.run(arrays,joined,policy,check)
    result.update(acceptance_contract=binding(policy),repair_invocations=1,
        parent_native_sha256=driver.sha(folder/'native/generated/NETGEN.npz'))
    if 'conditioning' in joined['provenance']:result['conditioning_receipt_sha256']=joined['provenance']['conditioning']['receipt_sha256']
    durable_json(folder/'SHARED_APEX_REPAIR.json',result)
    return points,dict(method_identity=identity(repair.METHOD),receipt_sha256=driver.sha(folder/'SHARED_APEX_REPAIR.json'),
        original_coordinate_identity=result['original_coordinates_sha256'],coordinate_identity=array_hash(points),
        changed_count=result['changed_count'],changed_vertices=result['changed_vertices'],repair_invocations=1,
        acceptance_contract=binding(policy))

def verify(folder,parent,arrays,state,policy,check):
    """Replay committed moves, never rerun candidate selection or optimization."""
    from finite_research import direct_shared_apex as repair
    from finite_research.direct_fill_contract import require_binding
    from immutable_state_access import guard
    with guard(folder/'SHARED_APEX_REPAIR.json'):
        result=driver.read(folder/'SHARED_APEX_REPAIR.json');stamp=driver.sha(folder/'SHARED_APEX_REPAIR.json')
        provenance=state['provenance']['repair'];require_binding(result['acceptance_contract'],policy)
        require_binding(provenance['acceptance_contract'],policy)
        driver.require(provenance['receipt_sha256']==stamp and provenance['method_identity']==identity(repair.METHOD)
            and result['method']==repair.METHOD and result['repair_invocations']==provenance['repair_invocations']==1
            and state['provenance']['construction_invocations']==1,'repair method and receipt identity')
        core=driver.arrays(folder/'native/generated/NETGEN.npz',dict(points='<f8',tets='<i8'),check)
        driver.require(driver.sha(folder/'native/generated/NETGEN.npz')==result['parent_native_sha256'],'pre-repair native hash')
        if 'conditioning' in state['provenance']:
            from background_conditioning import verify as verify_conditioning
            driver.require(result['conditioning_receipt_sha256']==state['provenance']['conditioning']['receipt_sha256'],'apex conditioned input receipt')
            original=verify_conditioning(folder,parent,arrays,state,policy,check)
        else:
            original=_native_original(core,parent,arrays,check)
        driver.require(original.shape==arrays['points'].shape and array_hash(original)==result['original_coordinates_sha256']==provenance['original_coordinate_identity'],'pre-repair coordinate identity')
        before=dict(state,points=original);ctx=repair.Context(dict(arrays,points=original),before,policy,check)
        return _moves(folder,arrays,state,policy,check,repair,result,provenance,original,ctx)


def _native_original(core,parent,arrays,check):
        """Unconditioned historical path; topology must remain native-exact."""
        capids=np.unique(arrays['cap']);prefix=arrays['points'][:parent['prefix_points']]
        lookup={prefix[int(i)].tobytes():int(i) for i in capids}
        driver.require(len(lookup)==len(capids),'unique cap coordinates')
        mapped=np.empty(len(core['points']),dtype='<i8');other=[];seen=set()
        for i,p in enumerate(core['points']):
            if i%4096==0:check()
            selected=lookup.get(p.tobytes())
            if selected is None:mapped[i]=len(prefix)+len(other);other.append(i)
            else:
                driver.require(selected not in seen,'unique native cap mapping');seen.add(selected);mapped[i]=selected
        driver.require(seen==set(map(int,capids)),'complete native cap mapping')
        original=np.concatenate([prefix,core['points'][other]])
        driver.same(mapped[core['tets']],arrays['background_tets'],'native repair connectivity')
        return original


def _moves(folder,arrays,state,policy,check,repair,result,provenance,original,ctx):
        records=result['records'];changed=[]
        driver.require(len(records)<=repair.METHOD['maximum_vertices'] and [r['vertex'] for r in records]==result['selected_vertices']
            and result['selected_vertices']==sorted(set(result['selected_vertices'])),'bounded one-pass repair records')
        for record in records:
            check()
            if record['status']=='UNCHANGED_ALREADY_FEASIBLE':continue
            if record['status']=='REFUSED_ROLLED_BACK':continue
            driver.require(record['status']=='COMMITTED','known repair record status')
            v=record['vertex'];driver.require(type(v) is int and v not in changed,'unique committed apex')
            star=ctx.star(v);driver.require(sorted(star.selected)==record['incident_tet_ids'] and len(star.rows)==record['incident_tets']
                and len(star.owners)==record['checked_faces'] and len(star.cells)==record['halo_cells']
                and sum(star.lv not in uses[0][1] for uses in star.owners.values())==record['fixed_opposite_faces']
                and star.radius==record['radius_m'],'complete committed star and movement domain')
            driver.same(ctx.points[v],np.asarray(record['before_m'],dtype='<f8'),'committed before bits')
            after=np.asarray(record['after_m'],dtype='<f8');delta=after-ctx.points[v]
            driver.require(float(np.linalg.norm(delta))==record['displacement_m'],'committed displacement')
            direct=star.base.copy();direct[star.lv]=after
            from fractions import Fraction
            import local_termination_topology as topology
            from prism_hotpath_v2 import batch_quality
            driver.require(repair.movement_allowed(ctx.points[v],after,star.radius),'stored movement bound')
            batch=batch_quality(direct,star.cells,star.owners,0,active_cells=star.selected,mixed_cells=True,compact=True,face_batch_size=4096)
            scalar=topology.quality(direct,star.cells,star.owners,0,active_cells=star.selected)
            exact=sum((Fraction(topology.exact_cell(direct,c)['signed_volume']) for c in star.selected_cells),Fraction())
            labels=repair.exact_labels(direct,star.tets,ctx.limits)
            scores=repair.tetrahedron_ratio(direct,star.tets)
            driver.require(batch['passed'] and scalar['passed'] and exact==star.old_exact and np.all(labels[:,0]) and np.all(scores>=ctx.limits[0]),'stored exact move hard QA and minimum')
            initial=batch_quality(star.base,star.cells,star.owners,0,active_cells=star.selected,mixed_cells=True,compact=True,face_batch_size=4096)
            driver.require(batch['summary']==record['source_batch_summary'] and record['source_scalar_passed'] is True
                and record['exact_star_volume_conserved'] is True
                and batch['summary']['minimum_weight']>initial['summary']['minimum_weight']
                and type(record['evaluations']) is int and 0<record['evaluations']<=repair.METHOD['maximum_search_evaluations_per_vertex']
                and len(record['trials'])==2 and all(type(r['iterations']) is int and 0<=r['iterations']<=repair.METHOD['maxiter'] for r in record['trials']),
                'stored repair strict QA improvement and work bounds')
            ctx.points[v]=after;ctx.q[star.rows]=scores;ctx.labels[star.rows]=(scores[:,None]>=ctx.limits)&labels
            ctx.affected_union.update(map(int,star.rows));ctx.version+=1;ctx.expected=array_hash(ctx.points);changed.append(v)
        actual=np.flatnonzero(np.any(original.view('<u8')!=arrays['points'].view('<u8'),axis=1)).tolist()
        driver.require(sorted(changed)==actual==result['changed_vertices']==provenance['changed_vertices']
            and len(changed)==result['changed_count']==provenance['changed_count'] and not set(actual)&ctx.protected
            and len(ctx.affected_union)==result['union_affected_tets'],'complete repair changed inventory')
        driver.same(ctx.points,arrays['points'],'exact replayed final points')
        driver.require(array_hash(arrays['points'])==result['coordinates_sha256']==provenance['coordinate_identity'],'final repaired coordinate identity')
        return dict(passed=True,repair_invocations=1,changed_count=len(changed),optimizer_replays=0)
