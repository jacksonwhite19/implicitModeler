"""Fresh complete joined geometry checks for the explicit graded live route."""
from fractions import Fraction
from pathlib import Path
import json
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,file_hash
from path_preserving_deduplication import durable_json
from finite_support.stored_wall_contract import SCHEMA,verify

REQUIRED={'domain','incidence','winding','surfaces','boundary_collision','enclosure','collision','filling','physical_qa','joined_qa'}

def compact(points,faces):
    vertices=np.unique(faces)
    lookup=np.full(len(points),-1,dtype='<i8');lookup[vertices]=np.arange(len(vertices))
    return points[vertices],lookup[faces]

def audit(points,cells,owners,wall,farfield,front,bounds,folder,check,require_parent):
    from finite_stage_command import reviewed_construction
    from finite_stage_background import check_box
    from finite_stage_adapter import filling
    import local_termination_topology as topology
    from prism_hotpath_v2 import batch_quality
    from background_mixed_geometry import batch_geometry
    from capsule_core_control_audit import boundary_skewness
    folder=Path(folder);folder.mkdir()
    wall=np.asarray(wall,dtype='<i8');farfield=np.asarray(farfield,dtype='<i8');front=np.asarray(front,dtype='<i8')
    def stamp():
        return identity(dict(points=array_hash(points),cells=cells,owners=repr(sorted(owners.items())),
            wall=array_hash(wall),farfield=array_hash(farfield),front=array_hash(front),bounds=array_hash(bounds)))
    expected=stamp();receipts={};values={}
    def custody():
        check();require_parent()
        if stamp()!=expected:raise ValueError('ChangedJoinedGradedGeometry')
    def record(name,call,accept):
        custody();value=call();custody()
        if not accept(value):raise ValueError('JoinedGradedProofFailure',name)
        bound=dict(value,geometry_identity=expected,wall_contract=SCHEMA)
        path=folder/(name.upper()+'.json');durable_json(path,bound)
        receipts[name]=dict(sha256=file_hash(path),identity=identity(bound))
        values[name]=bound
    def domain():
        check_box(*compact(points,farfield),bounds)
        if not np.all((points>=bounds[:,0])&(points<=bounds[:,1])):raise ValueError('JoinedGradedOutsideWorld')
        return dict(passed=True,bounds_m=bounds.tolist(),front_role='INTERNAL_INTERFACE')
    def incidence():
        actual,value=topology.ownership(cells,list(map(tuple,farfield)),list(map(tuple,wall)))
        if repr(sorted(actual.items()))!=repr(sorted(owners.items())):raise ValueError('ChangedJoinedOwnerCycles')
        return value
    def physical():
        faces=np.vstack((wall,farfield));byid={cell['id']:cell for cell in cells}
        ids=[owners[tuple(sorted(face))][0][0] for face in faces]
        geometry=batch_geometry(points,[byid[owner] for owner in sorted(set(ids))])
        skew=boundary_skewness(points[faces],np.asarray([geometry[owner]['centre'] for owner in ids]))
        failed=np.flatnonzero(skew>3.63636)
        return dict(passed=bool(np.isfinite(skew).all() and not len(failed)),faces=len(faces),
            maximum_skewness=float(skew.max()),failure_indices=failed.tolist())
    with reviewed_construction() as construction:
        def enclosure():
            outer,faces=compact(points,farfield);vertices=np.unique(np.r_[wall.ravel(),front.ravel()])
            result=construction.degree(outer,list(map(tuple,faces)),
                [tuple(Fraction(float(value)) for value in points[index]) for index in vertices])
            result['witness_vertices']=vertices.tolist()
            return result
        record('domain',domain,lambda value:value['passed'])
        record('incidence',incidence,lambda value:value['passed'])
        record('winding',lambda:verify(cells,farfield,wall,SCHEMA,check),lambda value:value['passed'])
        record('surfaces',lambda:{name:construction.surface(*compact(points,faces),check)
            for name,faces in (('wall',wall),('front',front),('farfield',farfield))},
            lambda value:set(value)=={'wall','front','farfield'})
        physical_faces=list(map(tuple,np.vstack((wall,farfield))))
        record('boundary_collision',lambda:construction.collision(points,physical_faces,range(len(physical_faces)),check),
            lambda value:value['passed'] and value['independent_coverage_passed'])
        record('enclosure',enclosure,lambda value:value['passed'] and not value['unresolved']
            and len(value['values'])==len(value['witness_vertices']) and all(row['degree']==1 for row in value['values']))
        faces=[uses[0][1] for unused,uses in sorted(owners.items())]
        record('collision',lambda:construction.collision(points,faces,range(len(faces)),check),
            lambda value:value['passed'] and value['independent_coverage_passed'])
        record('filling',lambda:filling(dict(points=points,cells=cells),None),
            lambda value:value['positive_cells'] and value['exact_volume_balance'] and value['interior_degree']['passed']
            and len(value['exact_cells'])==len(cells)==len(value['interior_degree']['values']))
        record('physical_qa',physical,lambda value:value['passed'])
        record('joined_qa',lambda:batch_quality(points,cells,owners,0,mixed_cells=True,compact=True,face_batch_size=4096),
            lambda value:value['passed'] and value['summary']['affected_cells']==len(cells)
            and value['summary']['affected_faces']==sum(len(rows)==2 for rows in owners.values()))
    if set(receipts)!=REQUIRED:raise ValueError('IncompleteJoinedGradedReceiptSet')
    for name,binding in receipts.items():
        path=folder/(name.upper()+'.json');value=json.loads(path.read_text())
        if (file_hash(path)!=binding['sha256'] or identity(value)!=binding['identity']
                or value.get('geometry_identity')!=expected or value.get('wall_contract')!=SCHEMA):
            raise ValueError('ChangedJoinedGradedReadback',name)
    custody()
    filling_result=values['filling']
    summary={name:value for name,value in filling_result.items() if name not in ('exact_cells','interior_degree')}
    summary['interior_degree']=dict(passed=True,witnesses=len(cells),unresolved=[])
    result=dict(schema='V4JoinedGradedGeometry/1',passed=True,geometry_identity=expected,
        wall_contract=SCHEMA,cells=len(cells),points=len(points),unique_faces=len(owners),
        receipts=receipts,receipt_readbacks=len(receipts),post_audit_custody=True,
        collision_scope='ALL_FACES_INCLUDING_OLD_OLD',original_sdf_fluid_proof='NOT_PERFORMED',
        field_attainment='NOT_MEASURED')
    durable_json(folder/'PROOFS.json',result)
    return result,values['collision'],summary,values['joined_qa'],values['physical_qa']

