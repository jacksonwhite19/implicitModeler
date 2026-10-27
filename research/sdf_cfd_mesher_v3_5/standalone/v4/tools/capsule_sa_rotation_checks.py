"""Interpolation and angular-resolution sensitivity, not acceptance gates."""
import numpy as np
from scipy.spatial import cKDTree
import capsule_sa_rotation as r
c=r.c;h=r.h

def linear_stencil(tree,centers,pts,k=24):
    d,ids=tree.query(pts.reshape(-1,3),k=k,workers=1)
    offsets=(centers[ids]-pts.reshape(-1,1,3))/d[:,-1,None,None]
    a=np.concatenate((np.ones(d.shape+(1,)),offsets),axis=-1)
    w=1/np.maximum(d,1e-30)**2;w/=w.sum(axis=1)[:,None]
    gram=np.einsum('nki,nk,nkj->nij',a,w,a)
    rhs=np.zeros((len(d),4));rhs[:,0]=1
    coeff=np.linalg.solve(gram,rhs[...,None])[...,0]
    weights=np.einsum('nki,ni,nk->nk',a,coeff,w)
    assert np.allclose(weights.sum(axis=1),1,atol=1e-10)
    return (d,ids,weights),dict(max_condition=float(np.linalg.cond(gram).max()),
        max_weight_L1=float(abs(weights).sum(axis=1).max()),negative_weight_fraction=float((weights<0).mean()))

def lowpass(a,limit):
    f=np.fft.fft(a,axis=1);m=np.fft.fftfreq(a.shape[1])*a.shape[1]
    f[:,abs(m)>limit,:]=0
    return np.fft.ifft(f,axis=1).real

def run():
    grid=np.load(r.ROOT/'SAMPLING.npz');pts=grid['points'];groups=grid['groups'];w=grid['weights'];theta=np.arange(r.N)*2*np.pi/r.N
    results=dict(reconstruction={},comparisons={},mode_projection={});cache={}
    for name in c.NAMES:
        source=c.sa.mesh_source(name[3:]);x=np.load(source/'assembly/coordinates.npy')
        centers=np.concatenate([x[np.load(source/'assembly'/(f+'.npy'))].mean(axis=1) for f in ('prisms','tets')])
        tree=cKDTree(centers);st,quality=linear_stencil(tree,centers,pts)
        rows=h.read(c.ROOT/name/'HISTORY.json')['rows'];cache[name]={};results['reconstruction'][name]=quality
        for step in (800,900,1000):
            arrays=c.global_arrays(c.ROOT/name/'pilot',step);data={}
            angle=-np.arctan2(rows[step-1]['force'][1],rows[step-1]['force'][0])
            aligned,_=linear_stencil(tree,centers,r.rotate(pts,-angle))
            for f,a in arrays.items():
                v=r.sample(a,st,pts.shape[:2]);can=r.sample(a,aligned,pts.shape[:2])
                if f=='U':v=r.cylindrical(v,theta[None,:]);can=r.cylindrical(r.rotate(can,angle),theta[None,:])
                data[f]=v;data[f+'_aligned']=can
            data['force_angle']=np.array(-angle);cache[name][step]=data
            np.savez_compressed(r.ROOT/(name+'_linear_'+str(step)+'.npz'),**data)
        results['comparisons'][name]={};results['mode_projection'][name]={}
        for i,j in [(800,900),(900,1000),(800,1000)]:
            key=f'{i}-{j}';a=cache[name][i];b=cache[name][j];angle=float(b['force_angle']-a['force_angle'])
            results['comparisons'][name][key]={};results['mode_projection'][name][key]={}
            for group in ('body','wake'):
                mask=groups=='body' if group=='body' else np.char.startswith(groups,'wake');wg=w[mask]
                results['comparisons'][name][key][group]={};results['mode_projection'][name][key][group]={}
                for f in r.FIELDS:
                    av=a[f][mask];bv=b[f][mask];value=r.pair(av,bv,wg,angle)
                    scale=33.528 if f=='U' else (.5*33.528**2 if f=='p' else r.norm(bv,wg))
                    value['normalization_scale']=scale
                    value['normalized_raw_L2']=value['raw']['L2']/scale
                    value['normalized_aligned_L2']=value['force_aligned']['L2']/scale
                    value['finite_rotation_removed_squared_fraction']=1-value['ratio']**2
                    value['half_angular_grid_optimal']=r.optimal(av[:,::2],bv[:,::2],wg)
                    results['comparisons'][name][key][group][f]=value
                    results['mode_projection'][name][key][group][f]={str(m):r.projection(lowpass(av,m),lowpass(bv,m),wg) for m in (1,2,4,8,16)}
        print(name,'linear reconstruction and modal sensitivity complete',flush=True)
    results['cross_seed']={}
    a=cache[c.NAMES[0]][1000];b=cache[c.NAMES[1]][1000]
    for group in ('body','wake'):
        mask=groups=='body' if group=='body' else np.char.startswith(groups,'wake');wg=w[mask]
        results['cross_seed'][group]={f:dict(raw=r.difference(a[f][mask],b[f][mask],wg),aligned=r.difference(a[f+'_aligned'][mask],b[f+'_aligned'][mask],wg)) for f in r.FIELDS}
    h.durable_json(r.ROOT/'SENSITIVITY.json',h.seal(results))

if __name__=='__main__':run()
