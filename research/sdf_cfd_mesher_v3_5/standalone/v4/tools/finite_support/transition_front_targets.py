"""Explicit front-distribution construction targets; existing hard QC is separate.

Population requirements use counts, not a fitted Gaussian or interpolated
percentile. No production target values or geometry names are supplied here.
"""
from fractions import Fraction
from itertools import combinations
import math
import numpy as np
from background_topology_repair import key
from sdf_cfd_mesher_v4.common.authority import identity,array_hash

POPULATIONS=('outer_tetrahedra','outer_triangles')

def _unit(value,label):
    if type(value) not in (float,int) or not math.isfinite(value) or not 0<value<=1:
        raise ValueError('InvalidFrontTarget',label)

def validate(targets):
    if type(targets) is not dict or set(targets)!=set(POPULATIONS):
        raise ValueError('ExplicitFrontPopulationsRequired')
    for name in POPULATIONS:
        t=targets[name]
        if type(t) is not dict or set(t)!={'minimum','population_targets'}:
            raise ValueError('ExplicitFrontDistributionRequired',name)
        _unit(t['minimum'],name+'.minimum')
        if type(t['population_targets']) is not list or not t['population_targets']:
            raise ValueError('ExplicitFrontPopulationTargetsRequired',name)
        for row in t['population_targets']:
            if type(row) is not dict or set(row)!={'fraction','minimum'}:
                raise ValueError('InvalidFrontPopulationTarget',name)
            _unit(row['fraction'],name+'.fraction');_unit(row['minimum'],name+'.minimum')
            if row['minimum']<t['minimum']:raise ValueError('PopulationTargetBelowMinimum',name)
    return targets

def population_report(values,target):
    a=np.asarray(values,dtype=float)
    if a.ndim!=1 or len(a)==0 or not np.isfinite(a).all() or np.any(a<=0):
        raise ValueError('InvalidFrontShapePopulation')
    n=len(a);rules=[]
    for row in target['population_targets']:
        # Decimal user fraction -> exact count requirement, rounded upward.
        need=math.ceil(Fraction(str(row['fraction']))*n)
        got=int(np.count_nonzero(a>=row['minimum']))
        rules.append(dict(**row,required_count=need,actual_count=got,actual_fraction=got/n,passed=got>=need))
    minimum=float(a.min());minimum_passed=minimum>=target['minimum']
    return dict(passed=minimum_passed and all(r['passed'] for r in rules),count=n,
        minimum=minimum,required_minimum=target['minimum'],minimum_passed=minimum_passed,
        population_targets=rules,quantiles=[0,.01,.1,.5,.9,1],
        diagnostic_quantile_values=np.quantile(a,[0,.01,.1,.5,.9,1]).tolist())

def tetrahedron_ratio(points,tets):
    p=np.asarray(points)[np.asarray(tets,dtype='<i8')]
    six=np.einsum('ni,ni->n',p[:,1]-p[:,0],np.cross(p[:,2]-p[:,0],p[:,3]-p[:,0]))
    edges=sum(np.sum((p[:,i]-p[:,j])**2,axis=1) for i,j in combinations(range(4),2))
    with np.errstate(invalid='ignore',divide='ignore'):
        return 12*np.power(six*.5,2/3)/edges

def triangle_ratio(points,faces):
    p=np.asarray(points)[np.asarray(faces,dtype='<i8')]
    squared=sum(np.sum((p[:,i]-p[:,j])**2,axis=1) for i,j in ((0,1),(1,2),(2,0)))
    with np.errstate(invalid='ignore',divide='ignore'):
        return 2*np.sqrt(3)*np.linalg.norm(np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=1)/squared

def populations(s):
    outer=np.asarray(s['outer'],dtype='<i8');ids=set()
    for face in outer:
        uses=s['owners'][key(face)]
        if len(uses)!=1:raise ValueError('FrontTargetBoundaryOwnerRequired')
        ids.add(uses[0][0])
    cells=[c for c in s['additions'] if c['id'] in ids]
    if len(cells)!=len(ids) or any(c['kind']!='tetrahedron' for c in cells):
        raise ValueError('FrontTargetTetrahedronOwnerRequired')
    tets=np.asarray([c['vertices'] for c in cells],dtype='<i8')
    return tets,outer

def evaluate(s):
    targets=validate(s['controls']['quality_targets']);tets,faces=populations(s)
    report={POPULATIONS[0]:population_report(tetrahedron_ratio(s['points'],tets),targets[POPULATIONS[0]]),
            POPULATIONS[1]:population_report(triangle_ratio(s['points'],faces),targets[POPULATIONS[1]])}
    return dict(schema='V4TransitionFrontTargets/1',passed=all(r['passed'] for r in report.values()),
        targets_identity=identity(targets),points_sha256=array_hash(s['points']),
        outer_tets_sha256=array_hash(tets),outer_faces_sha256=array_hash(faces),populations=report,
        scope='Configured outer-front construction objectives only; all existing scientific admission remains required.')

def objective_scores(points,tets,faces):
    # Common ideal value1. Equal dimensionless shape measures; no quality floor
    # or population requirement is hidden in the optimization objective.
    with np.errstate(invalid='ignore',divide='ignore'):
        return np.r_[1/tetrahedron_ratio(points,tets),1/triangle_ratio(points,faces)]
