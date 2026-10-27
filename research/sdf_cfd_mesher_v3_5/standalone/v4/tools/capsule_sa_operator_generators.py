"""Independent finite-angle generator checks on deterministic retained-cell samples."""
import numpy as np
from scipy.spatial import cKDTree
import capsule_sa_operator_directions as d
from capsule_sa_rotation_checks import linear_stencil
o=d.o;c=d.c;h=d.h;a=d.a


def rotate(x,theta):
    out=x.copy();co=np.cos(theta);si=np.sin(theta)
    out[...,0]=co*x[...,0]-si*x[...,1]
    out[...,1]=si*x[...,0]+co*x[...,1]
    return out


def active_scalar_generator(points,gradient):
    return points[:,1]*gradient[:,0]-points[:,0]*gradient[:,1]


def quadratic_stencil(tree,centers,pts,k=48):
    dist,ids=tree.query(pts,k=k,workers=1)
    xyz=(centers[ids]-pts[:,None,:])/dist[:,-1,None,None]
    x,y,z=xyz.transpose(2,0,1)
    basis=np.stack((np.ones_like(x),x,y,z,x*x,y*y,z*z,x*y,x*z,y*z),axis=-1)
    weight=1/np.maximum(dist,1e-30)**2;weight/=weight.sum(axis=1)[:,None]
    gram=np.einsum('nki,nk,nkj->nij',basis,weight,basis)
    rhs=np.zeros((len(dist),10));rhs[:,0]=1
    coeff=np.linalg.solve(gram,rhs[...,None])[...,0]
    weights=np.einsum('nki,ni,nk->nk',basis,coeff,weight)
    return (dist,ids,weights),dict(max_condition=float(np.linalg.cond(gram).max()),max_weight_L1=float(abs(weights).sum(axis=1).max()))


def run(order=1):
    result={}
    for name in c.NAMES:
        fields=np.load(o.ROOT/(name+'_1000_operators.npz'));C=fields['C'];V=fields['V'];tree=cKDTree(C)
        regions=a.regions(name,C);selected=[];labels=[]
        for label,mask in regions.items():
            # A rotated query must stay inside the original square box.
            ids=np.flatnonzero(mask&(np.linalg.norm(C[:,:2],axis=1)<.70)&(abs(C[:,2])<1.20))
            ids=ids[np.linspace(0,len(ids)-1,min(len(ids),2000),dtype=int)] if len(ids) else ids
            selected.extend(ids);labels.extend([label]*len(ids))
        ids=np.array(selected);labels=np.array(labels);pts=C[ids];v=V[ids]
        arrays={f:fields[f] for f in ('U','p','nuTilda','nut')}
        arrays['synthetic_axis_scalar']=np.sum(C[:,:2]**2,axis=1)[:,None]
        arrays['synthetic_m1']=C[:,0,None]
        arrays['synthetic_axis_vector']=np.column_stack((-C[:,1],C[:,0],C[:,2]))
        records={};previous={}
        for epsilon in (.01,.003,.001):
            sides=[];quality=[]
            for angle in (epsilon,-epsilon):
                stencil=linear_stencil if order==1 else quadratic_stencil
                (dist,indices,w),q=stencil(tree,C,rotate(pts,-angle),24 if order==1 else 48)
                q['nearest_distance_median']=float(np.median(dist[:,0]));q['nearest_distance_max']=float(dist[:,0].max())
                q['negative_weights_fraction']=float((w<0).mean());quality.append(q)
                sampled={f:np.einsum('nk,nkd->nd',w,values[indices]) for f,values in arrays.items()}
                for f in ('U','synthetic_axis_vector'):sampled[f]=rotate(sampled[f],angle)
                sides.append(sampled)
            record=dict(interpolation=quality,fields={})
            for f,g in [('U','gU'),('p','gP'),('nuTilda','gN'),('nut','gNT')]:
                gen=(sides[0][f]-sides[1][f])/(2*epsilon);grad=fields[g][ids]
                rr=dict(finite_angle_RMS=d.rms(gen,v),gradient_RMS=d.rms(grad,v),
                        difference_RMS=d.rms(gen-grad,v),cosine=d.inner(gen,grad,v)/(d.rms(gen,v)*d.rms(grad,v)),
                        regions={r:dict(finite_angle_RMS=d.rms(gen[labels==r],v[labels==r]),difference_RMS=d.rms((gen-grad)[labels==r],v[labels==r])) for r in np.unique(labels)})
                if f in previous:rr['epsilon_relative_change']=d.rms(gen-previous[f],v)/d.rms(gen,v)
                previous[f]=gen;record['fields'][f]=rr
            synth={}
            for f,target in [('synthetic_axis_scalar',np.zeros((len(ids),1))),('synthetic_m1',pts[:,1,None]),('synthetic_axis_vector',np.zeros((len(ids),3)))]:
                gen=(sides[0][f]-sides[1][f])/(2*epsilon)
                synth[f]=dict(error_RMS=d.rms(gen-target,v),error_Linf=float(np.linalg.norm(gen-target,axis=1).max()))
            record['synthetic']=synth;records[str(epsilon)]=record
        result[name]=dict(sample_count=len(ids),sample_ids_hash=h.array_hash(ids),epsilon=records,
                         exclusion='Query radii below 0.70 m and |z| below 1.20 m. No rotated-box extrapolation.')
        np.savez_compressed(o.ROOT/(name+'_generator_samples_order'+str(order)+'.npz'),ids=ids,labels=labels,points=pts,weights=v,**previous)
        print(name,'finite-angle construction checked',flush=True)
    theta=np.linspace(0,2*np.pi,4096,endpoint=False)
    distance=.75/np.maximum(abs(np.cos(theta)),abs(np.sin(theta)))
    ft=np.fft.rfft(distance)/len(distance)
    result['box_geometry']=dict(radial_distance_min=float(distance.min()),max=float(distance.max()),mean=float(distance.mean()),
                               m2_amplitude=float(2*abs(ft[2])),m4_amplitude=float(2*abs(ft[4])),
                               interpretation='Geometric fourfold symmetry breaking, not an operator pinning measurement.')
    h.durable_json(o.ROOT/('GENERATORS.json' if order==1 else 'GENERATORS_QUADRATIC.json'),h.seal(result))


if __name__=='__main__':
    import sys
    run(int(sys.argv[1]) if len(sys.argv)>1 else 1)
