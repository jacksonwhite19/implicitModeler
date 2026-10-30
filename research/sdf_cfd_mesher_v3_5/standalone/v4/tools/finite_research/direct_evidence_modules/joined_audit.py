from pathlib import Path
from fractions import Fraction
import time
import numpy as np
from finite_research.direct_evidence_modules import driver,background_audit
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from finite_support.stored_wall_contract import SCHEMA

REQUIRED=("preservation","domain","incidence","winding","interface_qa","joined_qa","physical_qa","surfaces","boundary_collision","enclosure","collision","filling")

def geometry_identity(state):
    return identity(dict(arrays={name:dict(hash=array_hash(state[name]),dtype=state[name].dtype.str,shape=state[name].shape)
        for name in ('points','bulk_tets','prefix_front','farfield','wall','bounds')},
        cells=state['cells'],owners=repr(sorted(state['owners'].items())),prefix_ids=state['prefix_ids'],
        prefix_points=state['prefix_points'],prefix_digest=state['prefix_digest'],provenance=state['provenance']))


def custody(state,expected):
    driver.require(geometry_identity(state)==expected,'whole joined geometry/input/ownership custody')


def compact_surface(points,faces):
    vertices=np.unique(faces);indices=np.full(len(points),-1,dtype='<i8');indices[vertices]=np.arange(len(vertices))
    return points[vertices],indices[faces]


def readback(out,receipts,stamp):
    driver.require(set(receipts)==set(REQUIRED),'complete required proof receipt set')
    for name,wanted in receipts.items():
        value=driver.read(out/(name.upper()+'.json'))
        driver.require(identity(value)==wanted['identity'] and driver.sha(out/(name.upper()+'.json'))==wanted['sha256']
            and value['geometry_identity']==stamp and value['wall_contract']==SCHEMA,'proof receipt binding '+name)
    return len(receipts)


def audit(state,preservation,helpers,construction,check,out,*,collision_reader=None,filling_producer=None):
    from finite_research.direct_evidence_modules.collision import retained
    if collision_reader is not None:
        retained=collision_reader
    import local_termination_topology as topology
    from finite_support.stored_wall_contract import verify
    from prism_hotpath_v2 import batch_quality
    from background_mixed_geometry import batch_geometry
    from capsule_core_control_audit import boundary_skewness
    stamp=geometry_identity(state);receipts={};timings={};started=time.perf_counter();current='start'
    def checkpoint(name,call,accept):
        nonlocal current
        current=name;check();custody(state,stamp);tick=time.perf_counter()
        value=call();timings[name]=time.perf_counter()-tick
        custody(state,stamp)
        bound=dict(value,geometry_identity=stamp,wall_contract=SCHEMA)
        construction.d.publish(out/(name.upper()+'.json'),bound)
        receipts[name]=dict(identity=identity(bound),sha256=driver.sha(out/(name.upper()+'.json')))
        driver.require(accept(value),'whole joined proof '+name);check()
    points=state['points'];cells=state['cells'];owners=state['owners']
    def domain():
        vertices,faces=compact_surface(points,state['farfield'])
        helpers['finite_stage_background'].check_box(vertices,faces,state['bounds'])
        driver.require(np.all(points>=state['bounds'][:,0]) and np.all(points<=state['bounds'][:,1]),'all joined points in world box')
        return dict(passed=True,bounds_m=state['bounds'].tolist(),farfield_faces=len(faces),
            retained_front_role='INTERNAL_INTERFACE_NOT_OUTER_DOMAIN',wall_contract=SCHEMA)
    def incidence():
        actual,value=topology.ownership(cells,list(map(tuple,state['farfield'])),list(map(tuple,state['wall'])))
        driver.require(repr(sorted(actual.items()))==repr(sorted(owners.items())),'all joined owner cycles')
        return value
    def physical():
        faces=np.vstack((state['wall'],state['farfield']));byid={cell['id']:cell for cell in cells}
        ids=[owners[tuple(sorted(face))][0][0] for face in faces]
        geometry=batch_geometry(points,[byid[owner] for owner in sorted(set(ids))])
        values=boundary_skewness(points[faces],np.asarray([geometry[owner]['centre'] for owner in ids]))
        driver.require(np.isfinite(values).all(),'finite physical boundary skew')
        failed=np.flatnonzero(values>3.63636)
        return dict(passed=not len(failed),faces=len(faces),maximum_skewness=float(values.max()),
            failures=[dict(vertices=list(map(int,faces[index])),owner=ids[index],skewness=float(values[index])) for index in failed])
    def surfaces():
        return {name:construction.surface(*compact_surface(points,state[name]),check) for name in ('wall','prefix_front','farfield')}
    def enclosure():
        outer,faces=compact_surface(points,state['farfield'])
        ids=np.unique(np.r_[state['wall'].ravel(),state['prefix_front'].ravel()])
        samples=[tuple(Fraction(float(value)) for value in points[index]) for index in ids]
        value=construction.degree(outer,list(map(tuple,faces)),samples)
        value.update(witness_vertices=list(map(int,ids)),scope='All wall and retained-front vertices inside actual farfield')
        return value
    result=dict(schema='Task135WholeRetainedJoinedGeometry/1',geometry_identity=stamp,whole_joined_geometry_passed=False,
        scientific_acceptance=False,extent_reached=False,original_sdf_fluid_proof='NOT_PERFORMED',
        field_attainment='NOT_MEASURED',generation_invocations=state['provenance']['generation_invocations'],construction_invocations=state['provenance'].get('construction_invocations',0),live_parent=state['provenance']['live_parent'],
        lease_issued=False,publication='NOT_RUN',cells=len(cells),points=len(points),bulk_cells=len(state['bulk_tets']))
    try:
        checkpoint('preservation',lambda:preservation,lambda value:value['passed'])
        checkpoint('domain',domain,lambda value:value['passed'])
        checkpoint('incidence',incidence,lambda value:value['passed'])
        checkpoint('winding',lambda:verify(cells,state['farfield'],state['wall'],SCHEMA,check),lambda value:value['passed'])
        checkpoint('interface_qa',lambda:background_audit.interface_metrics(points,cells,owners,state['prefix_front'],set(state['prefix_ids']),check),lambda value:value['passed'])
        checkpoint('joined_qa',lambda:batch_quality(points,cells,owners,0,mixed_cells=True,compact=True,face_batch_size=4096),
            lambda value:value['passed'] and value['summary']['affected_cells']==len(cells) and value['summary']['affected_faces']==sum(len(uses)==2 for uses in owners.values()))
        checkpoint('physical_qa',physical,lambda value:value['passed'])
        checkpoint('surfaces',surfaces,lambda value:set(value)=={'wall','prefix_front','farfield'})
        boundary_faces=list(map(tuple,np.vstack((state['wall'],state['farfield']))))
        checkpoint('boundary_collision',lambda:retained(points,boundary_faces,list(range(len(boundary_faces))),check,out/'boundary_collision_witnesses'),
            lambda value:value['passed'] and value['independent_coverage_passed'])
        checkpoint('enclosure',enclosure,lambda value:value['passed'])
        all_faces=[uses[0][1] for unused,uses in sorted(owners.items())]
        active=list(range(len(all_faces)))
        checkpoint('collision',lambda:retained(points,all_faces,active,check,out/'collision_witnesses'),
            lambda value:value['passed'] and value['independent_coverage_passed'])
        checkpoint('filling',lambda:(helpers['finite_stage_adapter'].filling(dict(points=points,cells=cells),None)
            if filling_producer is None else filling_producer(dict(points=points,cells=cells))),
            lambda value:value['positive_cells'] and value['exact_volume_balance'] and value['interior_degree']['passed'] and len(value['exact_cells'])==len(cells) and len(value['interior_degree']['values'])==len(cells))
        result.update(whole_joined_geometry_passed=True,receipt_readbacks=readback(out,receipts,stamp),
            all_faces_checked=len(all_faces),collision_scope='ALL_FACES_INCLUDING_ALL_OLD_OLD_RELATIONS')
    except (ValueError,TimeoutError) as error:
        result.update(failure=repr(error),failed_stage=current)
    custody(state,stamp)
    result.update(post_audit_geometry_custody=True,proof_receipts=receipts,stage_seconds=timings,
        seconds=time.perf_counter()-started,not_run=[name for name in REQUIRED if name not in receipts])
    construction.d.publish(out/'JOINED_PROOF_RESULT.json',result)
    return result
