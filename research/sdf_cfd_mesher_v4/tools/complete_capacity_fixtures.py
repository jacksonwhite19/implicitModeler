"""New complete fixture authorities. Original capacity probes are unchanged."""
from types import SimpleNamespace
from itertools import combinations
from math import sqrt
import numpy as np
from sdf_cfd_mesher_v4.forecast.capacity import identity

VERSION='CompleteCapacityFixtures/1'

def source_midpoint_norm(v):
    """Binary64 reduction order of the sealed source geometry authority."""
    x,y,z=map(float,v)
    return sqrt((x*x+z*z)+y*y)

def orient(x,f,inward=False):
    f=np.asarray(f,dtype=np.int64);p=x[f];d=np.einsum('ij,ij->i',np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),p.mean(1))
    flip=(d>0) if inward else (d<0);f[flip]=f[flip][:,[0,2,1]];return f

def sphere(radius,level,inward=True,centre=(0,0,0),normalizer=source_midpoint_norm):
    x=np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],float)
    f=np.array([[a,b,c] for a in (0,1) for b in (2,3) for c in (4,5)])
    for _ in range(level):
        vertices=x.tolist();edges={};new=[]
        def mid(a,b):
            key=tuple(sorted((int(a),int(b))))
            if key not in edges:
                q=x[a]+x[b];q/=(normalizer(q) if normalizer else np.linalg.norm(q));edges[key]=len(vertices);vertices.append(q.tolist())
            return edges[key]
        for a,b,c in f:
            ab,bc,ca=mid(a,b),mid(b,c),mid(c,a)
            new.extend([(a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)])
        x=np.array(vertices);f=np.array(new)
    x*=radius;f=orient(x,f,inward);return x+np.asarray(centre),f

def box(half):
    fine=np.array([-.05,-.005,-.002,-.0012,-.0004,.0004,.0012,.002,.005,.05])
    axes=[np.array([-half,0,half]),fine,fine];vertices=[];lookup={};faces=[];core=[]
    def vertex(q):
        key=tuple(q)
        if key not in lookup:lookup[key]=len(vertices);vertices.append(q)
        return lookup[key]
    for dim in range(3):
        a,b=[j for j in range(3) if j!=dim]
        for sign in (0,-1):
            for i in range(len(axes[a])-1):
                for j in range(len(axes[b])-1):
                    ids=[]
                    for da,db in ((0,0),(1,0),(1,1),(0,1)):
                        q=[0.,0.,0.];q[dim]=axes[dim][sign];q[a]=axes[a][i+da];q[b]=axes[b][j+db];ids.append(vertex(q))
                    for tri in ((ids[0],ids[1],ids[2]),(ids[0],ids[2],ids[3])):
                        k=len(faces);faces.append(tri)
                        if dim==0 and max(abs(vertices[v][1]) for v in tri)<=.0004 and max(abs(vertices[v][2]) for v in tri)<=.0004:core.append(k)
    x=np.array(vertices);return x,orient(x,faces,True),np.array(core,dtype=np.int64)

def cylinder(radius,n=128):
    z=np.array([-.05,-.005,-.002,-.0012,-.0004,.0004,.0012,.002,.005,.05]);theta=np.arange(n)*2*np.pi/n
    x=np.array([(radius*np.cos(t),radius*np.sin(t),v) for v in z for t in theta]);faces=[];core=[]
    for j in range(len(z)-1):
        for i in range(n):
            a=j*n+i;b=j*n+(i+1)%n;c=b+n;d=a+n
            for tri in ((a,b,c),(a,c,d)):
                if z[j]==-.0004 and z[j+1]==.0004:core.append(len(faces))
                faces.append(tri)
    vertices=x.tolist()
    for j in (0,len(z)-1):
        centre=len(vertices);vertices.append([0,0,float(z[j])])
        for i in range(n):faces.append((centre,j*n+i,j*n+(i+1)%n))
    x=np.array(vertices);return x,orient(x,faces,True),np.array(core,dtype=np.int64)


def wedge(spec):
    # Reuse the complete box connectivity, then map every x face to its plane.
    x,f,core=box(spec['half_gap']);h=spec['half_gap']
    y=x[:,1].copy();y[y==-.05]=-.007;y[y==.05]=.1;x[:,1]=y
    lo=-h-spec['left_slope']*y;hi=h+spec['right_slope']*y
    u=(x[:,0]+h)/(2*h);x[:,0]=lo+(hi-lo)*u
    return x,orient(x,f,True),core

def polyplanes(spec):
    # a.x <= b; exact fixture geometry is the intersection of these six planes.
    a=np.array([[1,-spec['right_slope'],0],[-1,-spec['left_slope'],0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],float)
    b=np.array([spec['half_gap'],spec['half_gap'],.1,.007,.05,.05]);length=np.linalg.norm(a,axis=1)
    return a/length[:,None],b/length

def poly_distance(x,spec):
    a,b=polyplanes(spec);slack=b-x@a.T;inside=np.all(slack>=0,axis=1);which=np.argmin(slack,axis=1)
    phi=slack[np.arange(len(x)),which];g=-a[which].copy()
    # Outside distance is the true closest feasible face/edge/vertex, not max
    # plane distance. Numerical feasibility uses rounding error, not clearance.
    for i in np.flatnonzero(~inside):
        choices=[]
        for size in (1,2,3):
            for ids in combinations(range(len(a)),size):
                aa=a[list(ids)];bb=b[list(ids)];gram=aa@aa.T
                if np.linalg.matrix_rank(gram)!=size:continue
                q=x[i]-aa.T@np.linalg.solve(gram,aa@x[i]-bb)
                if np.all(a@q-b <= 32*np.finfo(float).eps*(1+np.linalg.norm(q))):choices.append(q)
        if not choices:raise ValueError('NoPolyhedronClosestPoint')
        dist=np.linalg.norm(np.array(choices)-x[i],axis=1);q=choices[int(np.argmin(dist))];length=float(dist.min())
        phi[i]=-length;g[i]=(q-x[i])/length if length else -a[which[i]]
    return phi,g

def frustum(spec):
    x,f,core=cylinder(spec['radius'],128)
    z=x[:,2].copy();z[z==-.05]=-.007;z[z==.05]=.1;x[:,2]=z
    rho=np.linalg.norm(x[:,:2],axis=1);x[:,:2]*=np.divide(spec['radius']+spec['slope']*z,rho,out=np.ones_like(rho),where=rho!=0)[:,None]
    return x,orient(x,f,True),core

def frustum_distance(x,spec):
    rho=np.linalg.norm(x[:,:2],axis=1);p=np.c_[rho,x[:,2]];r=spec['radius'];s=spec['slope'];low=-.007;high=.1
    segments=[(np.array([r+s*low,low]),np.array([r+s*high,high])),(np.array([0,low]),np.array([r+s*low,low])),(np.array([0,high]),np.array([r+s*high,high]))]
    candidates=[]
    for index,(a,b) in enumerate(segments):
        t=np.clip((p-a)@(b-a)/np.dot(b-a,b-a),0,1);q=a+t[:,None]*(b-a)
        if index>0:q[:,0]=np.clip(rho,0,b[0]);q[:,1]=a[1]
        candidates.append(q)
    q=np.stack(candidates,axis=1);dist=np.linalg.norm(q-p[:,None],axis=2);which=np.argmin(dist,axis=1);near=q[np.arange(len(p)),which];length=dist[np.arange(len(p)),which]
    inside=(rho<=r+s*x[:,2])&(x[:,2]>=low)&(x[:,2]<=high);sign=np.where(inside,1.,-1.);phi=sign*length
    d=np.divide(sign[:,None]*(p-near),length[:,None],out=np.zeros_like(p),where=length[:,None]!=0)
    # Interior distance gradient points away from nearest boundary.
    normals=np.array([[-1,s],[0,1],[0,-1]],float);normals/=np.linalg.norm(normals,axis=1)[:,None]
    d[length==0]=normals[which[length==0]]
    g=np.c_[np.divide(x[:,:2],rho[:,None],out=np.zeros_like(x[:,:2]),where=rho[:,None]!=0)*d[:,0,None],d[:,1]]
    return phi,g

class FixtureSDF:
    def __init__(self,definition):self.d=definition;self.source_identity=identity(dict(version=VERSION,definition=definition))
    def evaluate(self,points):
        x=np.asarray(points,float);d=self.d;k=d['kind'];g=np.zeros_like(x)
        if k=='wedge':phi,g=poly_distance(x,d)
        elif k=='frustum':phi,g=frustum_distance(x,d)
        elif k=='box':
            half=np.array(d['half']);q=np.abs(x)-half;outside=np.maximum(q,0);norm=np.linalg.norm(outside,axis=1)
            phi=-(norm+np.minimum(q.max(1),0));ix=np.argmax(q,axis=1)
            g[np.arange(len(x)),ix]=-np.sign(x[np.arange(len(x)),ix])
            mask=norm>0;g[mask]=-np.sign(x[mask])*outside[mask]/norm[mask,None]
        elif k=='cylinder':
            radial=np.linalg.norm(x[:,:2],axis=1);v=np.c_[radial-d['radius'],np.abs(x[:,2])-d['half_length']]
            outside=np.maximum(v,0);length=np.linalg.norm(outside,axis=1);phi=-(length+np.minimum(v.max(1),0))
            radial_gradient=np.divide(x[:,:2],radial[:,None],out=np.zeros_like(x[:,:2]),where=radial[:,None]!=0)
            side=v[:,0]>=v[:,1];g[side,:2]=-radial_gradient[side];g[~side,2]=-np.sign(x[~side,2])
            m=length>0;g[m,:2]=-radial_gradient[m]*outside[m,0,None]/length[m,None];g[m,2]=-np.sign(x[m,2])*outside[m,1]/length[m]
        else:
            centres=np.asarray(d['centres']);r=np.asarray(d['radii']);v=x[:,None,:]-centres;length=np.linalg.norm(v,axis=2);dist=length-r
            pick=np.argmin(dist,axis=1);phi=dist[np.arange(len(x)),pick];vv=v[np.arange(len(x)),pick];ll=length[np.arange(len(x)),pick]
            g=np.divide(vv,ll[:,None],out=np.zeros_like(vv),where=ll[:,None]!=0)
            if d.get('cavity'):phi=-phi;g=-g
        norm=np.linalg.norm(g,axis=1);normal=np.divide(g,norm[:,None],out=np.zeros_like(g),where=norm[:,None]!=0)
        return SimpleNamespace(phi=phi,gradient=g,normal=normal,gradient_norm=norm,
             source_error_bound=16*np.finfo(float).eps*(1+np.linalg.norm(x,axis=1)),section_curvature=np.zeros(len(x)),boundary_normal_selection=np.zeros(len(x),dtype=np.int64))

def definitions(targets):
    mid=lambda k:float((targets[k-2]+targets[k-1])/2)
    specs=[]
    for name,h in [('parallel_gap',mid(10)),('near_contact',float(targets[0])/2),('symmetric_contact',float(targets[7])),('parallel_wide',2*float(targets[-1]))]:
        specs.append(dict(name=name,kind='box',half=[h,.05,.05],capacity=dict(kind='parallel_gap',spacing=2*h)))
    for name,r in [('spherical_cavity',mid(13)),('curved_wide',2*float(targets[-1]))]:
        specs.append(dict(name=name,kind='spheres',centres=[[0,0,0]],radii=[r],cavity=True,capacity=dict(kind='spherical_cavity',radius=r)))
    specs.append(dict(name='cylindrical_cavity',kind='cylinder',radius=mid(16),half_length=.05,capacity=dict(kind='cylindrical_cavity',radius=mid(16))))
    specs.extend([
        dict(name='narrowing_escape',kind='wedge',half_gap=.003,left_slope=.3,right_slope=.3,capacity=None),
        dict(name='asymmetric_narrowing',kind='wedge',half_gap=.003,left_slope=.2,right_slope=.4,capacity=None),
        dict(name='curved_passage',kind='frustum',radius=.003,slope=.3,capacity=None),
        dict(name='general_asymmetric_passage',kind='spheres',centres=[[-.003,0,0],[.004,0,0],[.0005,.0045,.0003]],radii=[.002,.003,.0015],cavity=False,capacity=None)])
    return specs

def build(spec,level=3,normalizer=source_midpoint_norm):
    if spec['kind']=='wedge':return wedge(spec)
    if spec['kind']=='frustum':return frustum(spec)
    if spec['kind']=='box':return box(spec['half'][0])
    if spec['kind']=='cylinder':return cylinder(spec['radius'])
    xs=[];fs=[];offset=0
    for r,c in zip(spec['radii'],spec['centres']):
        x,f=sphere(r,level,spec['cavity'],c,normalizer);xs.append(x);fs.append(f+offset);offset+=len(x)
    x=np.concatenate(xs);f=np.concatenate(fs);return x,f,np.arange(len(f))
