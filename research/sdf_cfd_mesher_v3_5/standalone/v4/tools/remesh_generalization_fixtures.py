"""Clean closed study geometries. No changes to historical authorities."""
import itertools
from complete_capacity_fixtures import source_midpoint_norm
import local_cavity_remesh as s
np=s.np;q=s.q

def channel(offset=0.):
    # Exact planar inward offset: intersection of translated support planes.
    # Miter vertices preserve distance to every incident wall face; unlike
    # single-nearest-face Newton, corners do not stick to another wall face.
    slope=.2;h=.0012;L=np.sqrt(1+slope*slope)
    ys=np.linspace(-.003+offset,.005-offset,9);zs=np.linspace(-.003+offset,.003-offset,7);us=np.linspace(-1,1,3)
    axes=[us,ys,zs];verts=[];lookup={};faces=[]
    def vertex(a):
        u,y,z=a;v=(u*(h+slope*y-offset*L),y,z)
        if v not in lookup:lookup[v]=len(verts);verts.append(v)
        return lookup[v]
    for dim in range(3):
        a,b=[j for j in range(3) if j!=dim]
        for side in (0,-1):
            for i in range(len(axes[a])-1):
                for j in range(len(axes[b])-1):
                    ids=[]
                    for da,db in ((0,0),(1,0),(1,1),(0,1)):
                        x=[0.]*3;x[dim]=axes[dim][side];x[a]=axes[a][i+da];x[b]=axes[b][j+db];ids.append(vertex(x))
                    faces.extend([(ids[0],ids[1],ids[2]),(ids[0],ids[2],ids[3])])
    x=np.array(verts);f=np.array(faces)
    # Orient by fixed parametric incidence, using inward toward an interior point.
    if offset==0:
        centre=np.array([0,.001,0]);normal=np.cross(x[f[:,1]]-x[f[:,0]],x[f[:,2]]-x[f[:,0]])
        flip=np.einsum('ij,ij->i',normal,centre-x[f].mean(1))<0;f[flip]=f[flip][:,[0,2,1]]
    else:f=channel(0.)[1]
    return x,f

def definitions():
    return [dict(name='clean_narrowing_channel',kind='planar_miter_channel',slope=.2,half_gap=.0012,y_bounds=[-.003,.005],z_bounds=[-.003,.003],
                 authority='Intersection of six normalized support planes; exact inward parallel offsets'),
            dict(name='clean_curved_passage',kind='spheres',centres=[[-.003,0,0],[.003,0,0]],radii=[.002,.0025],cavity=False,capacity=None),
            dict(name='analytic_cavity_with_control',kind='spheres',centres=[[0,0,0],[.15,0,0]],
                 radii=[float(sum(q.load_schedule().targets[11:13])/2),.008],cavity=True,level=1,
                 capacity=dict(first_geometric_limit=13,mechanism='Small spherical offset collapse; separate wide sphere remains layered'))]

def build(spec):
    if spec['kind']=='planar_miter_channel':return channel()
    x,f,core=q.build(spec,level=spec.get('level',2),normalizer=source_midpoint_norm);return x,f


def front(spec,offset):
    if spec['kind']=='planar_miter_channel':return channel(float(offset))[0]
    x,f=build(spec);centres=np.array(spec['centres']);radii=np.array(spec['radii']);n=len(x)//len(radii);parts=[]
    for i,(c,r) in enumerate(zip(centres,radii)):
        # Analytic source-branch offset, checked against authoritative union SDF.
        parts.append(c+(x[i*n:(i+1)*n]-c)*(1+(-1 if spec.get('cavity') else 1)*float(offset)/r))
    return np.concatenate(parts)

def sdf_record(spec,x,target):
    if spec['kind']=='planar_miter_channel':
        a=np.array([[1,-.2,0],[-1,-.2,0],[0,-1,0],[0,1,0],[0,0,-1],[0,0,1]],float)
        b=np.array([.0012,.0012,.003,.005,.003,.003]);norm=np.linalg.norm(a,axis=1)
        slack=(b-x@a.T)/norm;phi=slack.min(1);owner=slack.argmin(1);normal=-a[owner]/norm[owner,None]
        # All admitted points are inside the convex polyhedron, where minimum
        # normalized support-plane slack IS exact signed distance to its boundary.
        return dict(target=float(target),inside=bool(np.all(slack>=0)),residual_max=float(abs(phi-target).max()),
            admitted=bool(np.all(slack>=0) and np.all(abs(phi-target)<=16*np.finfo(float).eps*(1+np.linalg.norm(x,axis=1)))),
            normalized_gradient=normal.tolist(),distance_scope='Interior convex polyhedron only; outside not admitted')
    sample=q.FixtureSDF(spec).evaluate(x)
    return dict(target=float(target),residual_max=float(abs(sample.phi-target).max()),
        admitted=bool(np.all(abs(sample.phi-target)<=sample.source_error_bound)),sdf_identity=q.FixtureSDF(spec).source_identity)
