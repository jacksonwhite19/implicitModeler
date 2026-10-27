"""Separate closure authorities; scientific core coordinates stay frozen."""
import numpy as np
from itertools import combinations
from types import SimpleNamespace
from complete_capacity_fixtures import FixtureSDF,build,identity
VERSION='ExtendedCapacityChannelClosure/1'

def definition(old):
    return dict(old,closure_low=-.0065,closure_high=.2,closure_z=.1,closure_version=VERSION)

def extended_front(old):
    x,f,core=build(old);new=definition(old)
    if old['kind']=='wedge':
        y=x[:,1].copy();lo=-old['half_gap']-old['left_slope']*y;hi=old['half_gap']+old['right_slope']*y
        u=(x[:,0]-lo)/(hi-lo);mask=(y==-.007)|(y==.1);y[y==-.007]=new['closure_low'];y[y==.1]=new['closure_high']
        x[mask,0]=(-old['half_gap']-old['left_slope']*y[mask])+(2*old['half_gap']+(old['left_slope']+old['right_slope'])*y[mask])*u[mask]
        x[:,1]=y;x[x[:,2]==-.05,2]=-new['closure_z'];x[x[:,2]==.05,2]=new['closure_z']
    else:
        z=x[:,2].copy();mask=(z==-.007)|(z==.1);oldrad=old['radius']+old['slope']*z
        z[z==-.007]=new['closure_low'];z[z==.1]=new['closure_high']
        x[mask,:2]*=((old['radius']+old['slope']*z[mask])/oldrad[mask])[:,None];x[:,2]=z
    return x,f,core,new

class ExtendedSDF(FixtureSDF):
    def __init__(self,spec):super().__init__(spec);self.source_identity=identity(dict(version=VERSION,definition=spec))
    def evaluate(self,points):
        x=np.asarray(points,float);s=self.d
        if s['kind']=='wedge':
            a=np.array([[1,-s['right_slope'],0],[-1,-s['left_slope'],0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],float)
            b=np.array([s['half_gap'],s['half_gap'],s['closure_high'],-s['closure_low'],s['closure_z'],s['closure_z']]);length=np.linalg.norm(a,axis=1);a/=length[:,None];b/=length
            slack=b-x@a.T;inside=np.all(slack>=0,axis=1);which=np.argmin(slack,axis=1);phi=slack[np.arange(len(x)),which];g=-a[which].copy()
            for i in np.flatnonzero(~inside):
                candidates=[]
                for size in (1,2,3):
                    for ids in combinations(range(6),size):
                        aa=a[list(ids)];bb=b[list(ids)];gram=aa@aa.T
                        if np.linalg.matrix_rank(gram)!=size:continue
                        q=x[i]-aa.T@np.linalg.solve(gram,aa@x[i]-bb)
                        if np.all(a@q-b<=32*np.finfo(float).eps*(1+np.linalg.norm(q))):candidates.append(q)
                if not candidates:raise ValueError('NoPolyhedronClosestPoint')
                distances=np.linalg.norm(np.array(candidates)-x[i],axis=1);j=int(np.argmin(distances));length=distances[j];phi[i]=-length;g[i]=(candidates[j]-x[i])/length if length else -a[which[i]]
        else:
            rho=np.linalg.norm(x[:,:2],axis=1);p=np.c_[rho,x[:,2]];r=s['radius'];slope=s['slope'];low=s['closure_low'];high=s['closure_high']
            segments=[(np.array([r+slope*low,low]),np.array([r+slope*high,high])),(np.array([0,low]),np.array([r+slope*low,low])),(np.array([0,high]),np.array([r+slope*high,high]))];candidates=[]
            for index,(a,b) in enumerate(segments):
                t=np.clip((p-a)@(b-a)/np.dot(b-a,b-a),0,1);q=a+t[:,None]*(b-a)
                if index>0:q[:,0]=np.clip(rho,0,b[0]);q[:,1]=a[1]
                candidates.append(q)
            q=np.stack(candidates,axis=1);dist=np.linalg.norm(q-p[:,None],axis=2);which=np.argmin(dist,axis=1);near=q[np.arange(len(p)),which];length=dist[np.arange(len(p)),which]
            inside=(rho<=r+slope*x[:,2])&(x[:,2]>=low)&(x[:,2]<=high);sign=np.where(inside,1.,-1.);phi=sign*length
            d=np.divide(sign[:,None]*(p-near),length[:,None],out=np.zeros_like(p),where=length[:,None]!=0)
            normals=np.array([[-1,slope],[0,1],[0,-1]],float);normals/=np.linalg.norm(normals,axis=1)[:,None];d[length==0]=normals[which[length==0]]
            g=np.c_[np.divide(x[:,:2],rho[:,None],out=np.zeros_like(x[:,:2]),where=rho[:,None]!=0)*d[:,0,None],d[:,1]]
        norm=np.linalg.norm(g,axis=1);normal=np.divide(g,norm[:,None],out=np.zeros_like(g),where=norm[:,None]!=0)
        return SimpleNamespace(phi=phi,gradient=g,normal=normal,gradient_norm=norm,source_error_bound=16*np.finfo(float).eps*(1+np.linalg.norm(x,axis=1)),section_curvature=np.zeros(len(x)),boundary_normal_selection=np.zeros(len(x),dtype=np.int64))

# Revision 2: close the solid walls behind the channel, not the fluid aperture.
# This retains the exact scientific inner faces and removes inward end caps.
def exterior_front(old):
    from complete_capacity_fixtures import box,cylinder,orient
    original,of,oc=build(old)
    if old['kind']=='wedge':
        xs=[];fs=[];cores=[];offset=0
        for side in (-1,1):
            x,f,c=box(old['half_gap']);u=(x[:,0]+old['half_gap'])/(2*old['half_gap']);y=x[:,1].copy();y[y==-.05]=-.007;y[y==.05]=.1;x[:,1]=y
            boundary=(-old['half_gap']-old['left_slope']*y) if side<0 else (old['half_gap']+old['right_slope']*y)
            outer=side*.1
            x[:,0]=outer+(boundary-outer)*u if side<0 else boundary+(outer-boundary)*u
            inner=(u==1) if side<0 else (u==0)
            # Copy the old complete inner wall values, including rounding.
            for i in np.flatnonzero(inner):
                ids=np.flatnonzero((original[:,1]==x[i,1])&(original[:,2]==x[i,2])&(np.sign(original[:,0])==side))
                if len(ids):x[i]=original[ids[0]]
            centre=np.array([side*.07,.04,0]);p=x[f];cross=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]);flip=np.einsum('ij,ij->i',cross,p.mean(1)-centre)<0;f[flip]=f[flip][:,[0,2,1]]
            core=np.flatnonzero(np.all(inner[f],axis=1)&np.all(np.abs(x[f,1])<=.0004,axis=1)&np.all(np.abs(x[f,2])<=.0004,axis=1))
            xs.append(x);fs.append(f+offset);cores.extend((core+sum(len(v) for v in fs[:-1])).tolist());offset+=len(x)
        x=np.concatenate(xs);f=np.concatenate(fs);core=np.array(cores)
    else:
        # Preserve all inner rings; replace axial cap fans by annular end faces
        # and a distant closed outer cylinder.
        n=128;rings=10;inner=original[:n*rings].copy();theta=np.arange(n)*2*np.pi/n
        outer=inner.copy();outer[:,:2]=np.tile(np.c_[.1*np.cos(theta),.1*np.sin(theta)],(rings,1))
        x=np.r_[inner,outer];faces=[]
        for j in range(rings-1):
            for i in range(n):
                a=j*n+i;b=j*n+(i+1)%n;c=b+n;d=a+n
                faces.extend([(a,b,c),(a,c,d)])
        f=np.array(faces);p=x[f];cross=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]);desired=np.c_[-p.mean(1)[:,:2],np.linalg.norm(p.mean(1)[:,:2],axis=1)*old['slope']];flip=np.einsum('ij,ij->i',cross,desired)<0;f[flip]=f[flip][:,[0,2,1]]
        faces=f.tolist();faces.extend((f[:,[0,2,1]]+n*rings).tolist())
        for j,sign in ((0,-1),(rings-1,1)):
            for i in range(n):
                a=j*n+i;b=j*n+(i+1)%n;c=b+n*rings;d=a+n*rings
                for tri in ((a,b,c),(a,c,d)):
                    p=x[list(tri)];normal=np.cross(p[1]-p[0],p[2]-p[0]);faces.append(tri if normal[2]*sign>0 else (tri[0],tri[2],tri[1]))
        f=np.array(faces);core=oc.copy()
    spec=dict(old,closure_version='ExteriorSolidClosure/2',outer_extent=.1)
    return x,f,core,spec

class ConvexDistance:
    def __init__(self,a,b):
        a=np.asarray(a,float);b=np.asarray(b,float);length=np.linalg.norm(a,axis=1);self.a=a/length[:,None];self.b=b/length;self.maps=[]
        for size in (1,2,3):
            for ids in combinations(range(len(a)),size):
                aa=self.a[list(ids)];gram=aa@aa.T
                if np.linalg.matrix_rank(gram)==size:self.maps.append((aa,self.b[list(ids)],np.linalg.solve(gram,aa)))
    def evaluate(self,x):
        violation=x@self.a.T-self.b;inside=np.all(violation<=0,axis=1);which=np.argmax(violation,axis=1);phi=violation[np.arange(len(x)),which];g=self.a[which].copy()
        ids=np.flatnonzero(~inside);p=x[ids];best=np.full(len(p),np.inf);nearest=np.zeros_like(p)
        for aa,bb,m in self.maps:
            q=p-(p@aa.T-bb)@m;feasible=np.all(q@self.a.T-self.b<=32*np.finfo(float).eps*(1+np.linalg.norm(q,axis=1))[:,None],axis=1);d=np.sum((p-q)**2,axis=1);pick=feasible&(d<best);best[pick]=d[pick];nearest[pick]=q[pick]
        if len(ids):
            dist=np.sqrt(best);phi[ids]=dist;g[ids]=np.divide(p-nearest,dist[:,None],out=g[ids].copy(),where=dist[:,None]!=0)
        return phi,g

class ExteriorSDF(FixtureSDF):
    def __init__(self,spec):
        super().__init__(spec);self.source_identity=identity(dict(version='ExteriorSolidClosure/2',definition=spec));self.solids=[]
        if spec['kind']=='wedge':
            for side in (-1,1):
                # Solid lies outside the original channel plane.
                slope=spec['left_slope'] if side<0 else spec['right_slope']
                a=[[side*-1,slope,0],[side,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]
                b=[-spec['half_gap'],.1,.1,.007,.05,.05];self.solids.append(ConvexDistance(a,b))
    def evaluate(self,points):
        x=np.asarray(points,float);s=self.d
        if s['kind']=='wedge':
            allv=[solid.evaluate(x) for solid in self.solids];v=np.stack([z[0] for z in allv],1);grad=np.stack([z[1] for z in allv],1);which=np.argmin(v,axis=1);phi=v[np.arange(len(x)),which];g=grad[np.arange(len(x)),which]
        else:
            rho=np.linalg.norm(x[:,:2],axis=1);p=np.c_[rho,x[:,2]];r=s['radius'];a=s['slope'];low=-.007;high=.1;outer=.1
            vertices=[np.array([r+a*low,low]),np.array([r+a*high,high]),np.array([outer,high]),np.array([outer,low])];candidates=[]
            for i in range(4):
                u=vertices[i];v=vertices[(i+1)%4];t=np.clip((p-u)@(v-u)/np.dot(v-u,v-u),0,1);z=u+t[:,None]*(v-u)
                if i in (1,3):z[:,0]=np.clip(rho,min(u[0],v[0]),max(u[0],v[0]));z[:,1]=u[1]
                if i==2:z[:,0]=outer;z[:,1]=np.clip(p[:,1],low,high)
                candidates.append(z)
            q=np.stack(candidates,1);dist=np.linalg.norm(q-p[:,None],axis=2);which=np.argmin(dist,axis=1);nearest=q[np.arange(len(x)),which];length=dist[np.arange(len(x)),which]
            inside=(rho>=r+a*x[:,2])&(rho<=outer)&(x[:,2]>=low)&(x[:,2]<=high);sign=np.where(inside,-1.,1.);phi=length*sign
            d=np.divide(sign[:,None]*(p-nearest),length[:,None],out=np.zeros_like(p),where=length[:,None]!=0)
            normals=np.array([[-1,a],[0,1],[1,0],[0,-1]],float);normals/=np.linalg.norm(normals,axis=1)[:,None];d[length==0]=normals[which[length==0]]
            g=np.c_[np.divide(x[:,:2],rho[:,None],out=np.zeros_like(x[:,:2]),where=rho[:,None]!=0)*d[:,0,None],d[:,1]]
        norm=np.linalg.norm(g,axis=1);return SimpleNamespace(phi=phi,gradient=g,normal=np.divide(g,norm[:,None],out=np.zeros_like(g),where=norm[:,None]!=0),gradient_norm=norm,source_error_bound=16*np.finfo(float).eps*(1+np.linalg.norm(x,axis=1)),section_curvature=np.zeros(len(x)),boundary_normal_selection=np.zeros(len(x),dtype=np.int64))
