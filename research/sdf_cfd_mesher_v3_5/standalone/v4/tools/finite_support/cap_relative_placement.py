"""Pure exact cap-relative vertex evidence; no coordinate contraction."""
from fractions import Fraction
import numpy as np
from shape_transition_domain import _inputs,_array,_closest,_tick
from finite_research.flexible_controls import extent,PLACEMENT
from sdf_cfd_mesher_v4.common.authority import array_hash,identity


def exact_distance_within(point,anchor,distance):
    return sum((Fraction(float(value))-Fraction(float(origin)))**2
        for value,origin in zip(point,anchor))<=Fraction(float(distance))**2


def predicate(point,geometry,weights,policy,bounds):
    first,second=(Fraction(float(value)) for value in weights)
    if first<0 or second<0 or first+second>1:return False
    origin,edge_one,edge_two,normal=geometry
    exact=tuple(Fraction(float(value)) for value in point)
    anchor=tuple(origin[index]+first*edge_one[index]+second*edge_two[index] for index in range(3))
    if sum(normal[index]*(exact[index]-origin[index]) for index in range(3))<=0:return False
    if any(not Fraction(float(bounds[index,0]))<exact[index]<Fraction(float(bounds[index,1]))
            for index in range(3)):return False
    if policy['mode']=='strict' and sum((exact[index]-anchor[index])**2
            for index in range(3))>Fraction(float(policy['maximum_distance_m']))**2:return False
    return True


def inputs(points,cap,triangles,policy,bounds,check):
    extent(policy)
    values=_inputs(points,cap,triangles,policy['nominal_distance_m'],check)
    box=_array(bounds,'<f8',(3,2),'InvalidPlacementWorldBounds')
    if np.any(box[:,0]>=box[:,1]):raise ValueError('InvalidPlacementWorldBounds')
    return (*values,box)


def verify(points,cap,triangles,policy,bounds,witnesses,check=None):
    points,cap,triangles,unused,geometry,box=inputs(points,cap,triangles,policy,bounds,check)
    if type(witnesses) is not dict or set(witnesses)!={'vertex_ids','triangle_ids','weights'}:
        raise ValueError('InvalidCapRelativeWitnessFields')
    count=len(points)-len(cap)
    vertices=_array(witnesses['vertex_ids'],'<i8',(count,),'InvalidPlacementVertexIDs')
    faces=_array(witnesses['triangle_ids'],'<i8',(count,),'InvalidPlacementTriangleIDs')
    weights=_array(witnesses['weights'],'<f8',(count,2),'InvalidPlacementWeights')
    if not np.array_equal(vertices,np.arange(len(cap),len(points),dtype='<i8')):
        raise ValueError('IncompletePlacementVertexPopulation')
    if np.any(faces<0) or np.any(faces>=len(triangles)):raise ValueError('InvalidPlacementTriangleIDs')
    distances=[]
    for vertex,face,weight in zip(vertices,faces,weights):
        _tick(check)
        if not predicate(points[vertex],geometry[face],weight,policy,box):
            raise ValueError('InvalidCapRelativeVertexPlacement')
        origin,edge_one,edge_two,normal=geometry[face]
        anchor=np.asarray([float(origin[index]+Fraction(float(weight[0]))*edge_one[index]
            +Fraction(float(weight[1]))*edge_two[index]) for index in range(3)])
        distances.append(float(np.linalg.norm(points[vertex]-anchor)))
    return dict(schema=PLACEMENT,passed=True,exact_dyadic=True,
        points_sha256=array_hash(points),cap_sha256=array_hash(cap),triangles_sha256=array_hash(triangles),
        extent_policy_identity=identity(policy),bounds_sha256=array_hash(box),
        witness_identity=identity({name:array_hash(value) for name,value in witnesses.items()}),
        vertices_checked=count,strict_bound_performed=policy['mode']=='strict',
        supplied_anchor_distance_quantiles_m=np.quantile(distances,[0,.01,.5,.99,1]).tolist() if count else [],
        vertex_only=True,nearest_source_distance=False,every_cell_tube=False,
        original_sdf_continuous_exclusion='NOT_PERFORMED',scientific_mesh_acceptance=False)


def witness(points,cap,triangles,policy,bounds,check=None):
    from scipy.spatial import cKDTree
    points,cap,triangles,unused,geometry,box=inputs(points,cap,triangles,policy,bounds,check)
    vertices=np.arange(len(cap),len(points),dtype='<i8')
    faces=np.empty(len(vertices),dtype='<i8');weights=np.empty((len(vertices),2),dtype='<f8')
    scale=max(float(np.max(np.abs(points))),float(np.max(np.abs(cap))),1.)
    tree=cKDTree((cap[triangles]/scale).mean(axis=1))
    for row,vertex in enumerate(vertices):
        _tick(check)
        unused,near=tree.query(points[vertex]/scale,k=min(32,len(triangles)))
        found=None
        for face in np.atleast_1d(near):
            for weight in _closest(points[vertex],cap[triangles[face]]):
                _tick(check)
                if predicate(points[vertex],geometry[face],weight,policy,box):
                    found=(face,weight);break
            if found is not None:break
        if found is None:raise ValueError('NoNonmutatingCapRelativeWitness')
        faces[row],weights[row]=found
    result=dict(vertex_ids=vertices,triangle_ids=faces,weights=weights)
    return result,verify(points,cap,triangles,policy,bounds,result,check)
