"""Analytic geometry inputs, independent of production meshing decisions.

Dimensions are design inputs in metres. No mesh outcome changes them.
Positive distance points into the requested extrusion domain.
"""
from types import SimpleNamespace
import numpy as np
from complete_capacity_fixtures import sphere
from sdf_cfd_mesher_v4.common.authority import identity

VERSION = 'GeometryFamilyAuthority/1'


def unit(x):
    n = np.linalg.norm(x, axis=-1)
    return np.divide(x, n[..., None], out=np.zeros_like(x), where=n[..., None] != 0)


def ellipsoid(x, axes):
    """Euclidean nearest point via the ellipsoid secular equation.

    Includes stationary solutions on a missing shortest-axis component. Fixed
    bisection ordering is deterministic; wall admission uses source roundoff.
    """
    a = np.asarray(axes, float); aa = a*a
    inside = np.sum((x/a)**2, axis=1) < 1
    lo = np.full(len(x), -float(min(aa)) + np.spacing(float(min(aa))))
    hi = np.maximum(np.linalg.norm(x*a, axis=1), max(aa))
    for _ in range(80):
        mid = (lo+hi)/2
        f = np.sum((a*x/(aa+mid[:, None]))**2, axis=1)-1
        lo = np.where(f > 0, mid, lo); hi = np.where(f > 0, hi, mid)
    near = aa*x/(aa+hi[:, None]); best = np.linalg.norm(near-x, axis=1)
    best[np.sum((near/a)**2,axis=1)<1-64*np.finfo(float).eps]=np.inf
    for j in range(3):
        other = [i for i in range(3) if i != j]
        same=[i for i in other if aa[i]==aa[j]]
        different=[i for i in other if aa[i]!=aa[j]]
        p = np.zeros_like(x); p[:, different] = aa[different]*x[:, different]/(aa[different]-aa[j])
        remainder = 1-np.sum((p[:, other]/a[other])**2, axis=1)
        valid = (x[:, j] == 0) & (remainder >= 0) & np.all(x[:,same]==0,axis=1)
        p[:, j] = a[j]*np.sqrt(np.maximum(remainder, 0))
        distance = np.linalg.norm(p-x, axis=1)
        take = valid & (distance < best)
        near[take] = p[take]; best[take] = distance[take]
    return np.where(inside, -best, best), unit(near/aa)


def primitive(x, d):
    k = d['primitive']; rho = np.linalg.norm(x[:, :2], axis=1)
    radial = unit(x[:, :2])
    if k == 'ellipsoid':
        return ellipsoid(x, d['axes'])
    if k == 'torus':
        q = np.c_[rho-d['major'], x[:, 2]]; n = unit(q)
        return np.linalg.norm(q, axis=1)-d['radius'], np.c_[radial*n[:, :1], n[:, 1]]
    if k == 'capsule':
        v = x.copy(); v[:, 2] -= np.clip(x[:, 2], -d['half_length'], d['half_length'])
        return np.linalg.norm(v, axis=1)-d['radius'], unit(v)
    if k == 'bent_capsule':
        angle = np.clip(np.arctan2(x[:, 1], x[:, 0]), -d['angle'], d['angle'])
        nearest = np.c_[d['major']*np.cos(angle), d['major']*np.sin(angle), np.zeros(len(x))]
        v = x-nearest
        return np.linalg.norm(v, axis=1)-d['radius'], unit(v)
    if k == 'rounded_box':
        q = abs(x)-d['half']; out = np.maximum(q, 0); n = unit(out)
        zero = np.linalg.norm(out, axis=1) == 0
        n[zero] = np.eye(3)[q[zero].argmax(1)]
        return np.linalg.norm(out, axis=1)+np.minimum(q.max(1), 0)-d['radius'], n*np.sign(x)
    if k == 'rounded_cylinder':
        q = np.c_[rho-d['radial'], abs(x[:, 2])-d['half_length']]
        out = np.maximum(q, 0); n = unit(out); zero = np.linalg.norm(out, axis=1) == 0
        n[zero] = np.eye(2)[q[zero].argmax(1)]
        return np.linalg.norm(out, axis=1)+np.minimum(q.max(1), 0)-d['radius'], np.c_[radial*n[:, :1], n[:, 1]*np.sign(x[:, 2])]
    if k == 'rounded_cone':
        h=d['height']; r1,r2=d['radii']; b=(r1-r2)/h; a=np.sqrt(1-b*b)
        q=np.c_[rho,x[:, 2]+h/2]; s=q@np.array([-b,a]); n=np.tile([a,b],(len(x),1))
        phi=q@n[0]-r1
        for mask, centre, radius in [(s<0,np.array([0,0]),r1),(s>a*h,np.array([0,h]),r2)]:
            v=q[mask]-centre; phi[mask]=np.linalg.norm(v,axis=1)-radius; n[mask]=unit(v)
        return phi,np.c_[radial*n[:, :1],n[:,1]]
    raise ValueError('UnknownAnalyticPrimitive:'+k)


class Source:
    def __init__(self, spec):
        self.spec=spec; self.source_identity=identity(dict(version=VERSION,definition=spec))
    def evaluate(self, points):
        x=np.asarray(points,float); values=[]; gradients=[]
        for d in self.spec['components']:
            p,g=primitive(x-np.asarray(d.get('centre',[0,0,0])),d)
            values.append(p); gradients.append(g)
        values=np.asarray(values).T; owner=values.argmin(1); phi=values[np.arange(len(x)),owner]
        gradient=np.asarray(gradients)[owner,np.arange(len(x))]
        if self.spec.get('cavity'): phi=-phi; gradient=-gradient
        return SimpleNamespace(phi=phi,gradient=gradient,normal=gradient,gradient_norm=np.linalg.norm(gradient,axis=1),
            source_error_bound=16*np.finfo(float).eps*(1+np.linalg.norm(x,axis=1)),
            section_curvature=np.zeros(len(x)),boundary_normal_selection=owner)


def radial_wall(d, level):
    directions,faces=sphere(1.,level,inward=False)
    centre=np.asarray(d.get('centre',[0,0,0]),float)
    anchor=np.asarray(d.get('ray_anchor',[0,0,0]),float)
    # The declared bounding radius is part of wall extraction authority.
    lo=np.zeros(len(directions)); hi=np.full(len(directions),d['ray_bound'])
    if np.any(primitive(anchor[None],d)[0]>=0) or np.any(primitive(anchor+hi[:,None]*directions,d)[0]<=0):
        raise ValueError('InvalidRadialWallAuthority')
    for _ in range(64):
        mid=(lo+hi)/2; outside=primitive(anchor+mid[:,None]*directions,d)[0]>0
        hi=np.where(outside,mid,hi);lo=np.where(outside,lo,mid)
    return centre+anchor+((lo+hi)/2)[:,None]*directions,faces


def build(spec):
    points=[]; faces=[]; base=0
    for d in spec['components']:
        if d['primitive']=='torus':
            nu,nv=spec.get('resolution',[16,8]); u=np.arange(nu)*2*np.pi/nu; v=np.arange(nv)*2*np.pi/nv
            x=np.array([[(d['major']+d['radius']*np.cos(b))*np.cos(a),
                         (d['major']+d['radius']*np.cos(b))*np.sin(a),d['radius']*np.sin(b)] for a in u for b in v])
            x+=d.get('centre',[0,0,0]); f=[]
            for i in range(nu):
                for j in range(nv):
                    a=i*nv+j;b=((i+1)%nu)*nv+j;c=((i+1)%nu)*nv+(j+1)%nv;e=i*nv+(j+1)%nv
                    f.extend([(a,b,c),(a,c,e)])
            f=np.asarray(f)
        else: x,f=radial_wall(d,spec.get('level',2))
        if spec.get('cavity'): f=f[:,[0,2,1]]
        points.append(x); faces.append(f+base);base+=len(x)
    x=np.concatenate(points);f=np.concatenate(faces); sample=Source(spec).evaluate(x)
    if np.any(abs(sample.phi)>sample.source_error_bound):raise ValueError('WallSourceAdmissionFailure')
    return x,f


def front(spec,offset):
    # Normal source-branch proposal. Admission and all mesh gates remain in the
    # production provider. Crossing a medial axis never earns acceptance here.
    x,_=build(spec); return x+float(offset)*Source(spec).evaluate(x).normal


def sdf_record(spec,x,target):
    sample=Source(spec).evaluate(x)
    return dict(target=np.asarray(target).tolist(),residual_max=float(abs(sample.phi-target).max()),
        admitted=bool(np.all(abs(sample.phi-target)<=sample.source_error_bound)),sdf_identity=Source(spec).source_identity)


def definitions():
    """Design ratios fixed before execution, not chosen from mesh outcomes.

    T=0.02 m is the design length (not a reconstructed binary64 schedule).
    Radial bounds enclose the declared primitive; no bound is a mesh gate.
    """
    T=.02
    def case(name,classes,components,**kw):
        return dict(name=name,kind='analytic_family',classes=classes,components=components,level=2,
                    design_length=T,capacity=None,**kw)
    return [
      case('ellipsoid_concavity',['strong_concavity','variable_scale'],[dict(primitive='ellipsoid',axes=[3*T,2*T,1.5*T],ray_bound=4*T)],cavity=True),
      case('toroidal_saddle',['saddle','mixed_convex_concave'],[dict(primitive='torus',major=4*T,radius=T)]),
      case('capsule_control',['wide_clearance','curvature_transition'],[dict(primitive='capsule',half_length=2*T,radius=2*T,ray_bound=5*T)]),
      case('rounded_box',['multiple_separated_regions','curvature_transition'],[dict(primitive='rounded_box',half=[2*T,1.5*T,T],radius=T,ray_bound=5*T)]),
      case('rounded_cylinder',['rapid_curvature'],[dict(primitive='rounded_cylinder',radial=2*T,half_length=2*T,radius=T/2,ray_bound=5*T)]),
      case('rounded_cone',['acute_smooth','variable_scale'],[dict(primitive='rounded_cone',height=4*T,radii=[2*T,T/2],ray_bound=5*T)]),
      case('bent_capsule',['thin_curved_passage'],[dict(primitive='bent_capsule',major=3*T,angle=np.pi/3,radius=T,ray_anchor=[3*T,0,0],ray_bound=6*T)],cavity=True),
      case('competing_ellipsoids',['opposing_surfaces','nearby_features','medial_axis'],[
          dict(primitive='ellipsoid',axes=[T,1.5*T,T],centre=[-1.5*T,0,0],ray_bound=2*T),
          dict(primitive='ellipsoid',axes=[T,T,1.5*T],centre=[1.5*T,0,0],ray_bound=2*T)]),
    ]


def scale_variant(spec, factor):
    """Uniform dimensionless scale sweep; topology and h1 are unchanged."""
    from copy import deepcopy
    out=deepcopy(spec);out['base_case']=spec['name'];out['scale_factor']=factor
    out['name']=spec['name']+'__scale_'+str(factor).replace('.','p')
    for component in out['components']:
        for key in ('axes','major','radius','half_length','half','radial','height','radii','centre','ray_anchor','ray_bound'):
            if key in component:
                v=np.asarray(component[key])*factor
                component[key]=v.tolist()
    return out


def cohort():
    return definitions()+[scale_variant(s,factor) for s in definitions() for factor in (.25,.5)]


def multi_region_definitions():
    from copy import deepcopy
    pair=scale_variant(definitions()[-1],.25)
    separated=deepcopy(pair);separated['name']='four_separated_competing_regions';separated['components']=[]
    for shift in ([0,0,0],[.1,0,0],[0,.1,0],[.1,.1,0]):
        for part in pair['components']:
            d=deepcopy(part);d['centre']=(np.asarray(d['centre'])+shift).tolist();separated['components'].append(d)
    separated['classes']=['multiple_separated_regions','nearby_features','medial_axis']
    separated['design_reason']='Four identical local competition regions on a square; 0.1 m separation exceeds twice the requested 0.02 m extrusion plus obstacle extent'
    nearby=deepcopy(pair);nearby['name']='three_intersecting_source_surfaces'
    third=deepcopy(pair['components'][0]);third['centre']=[0,.005,0];nearby['components'].append(third)
    nearby['classes']=['nearby_features','interacting_regions','medial_axis']
    nearby['design_reason']='Third ellipsoid is offset from the pair axis to create competing supports; no event outcome is prescribed'
    disjoint=deepcopy(nearby);disjoint['name']='three_interacting_features'
    distance=np.linalg.norm(np.asarray(pair['components'][0]['centre'])-pair['components'][1]['centre'])
    disjoint['components'][-1]['centre']=[0,float(np.sqrt(3)*distance/2),0]
    disjoint['design_reason']='Equilateral arrangement with the original pair centre spacing; disjoint analytic walls and interacting future offsets. Original overlapping-source extraction failure remains a separate case.'
    return [separated,nearby,disjoint]
