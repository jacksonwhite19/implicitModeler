"""Low-mode seed on physical rings; no arbitrary extension onto the whole box."""
import numpy as np
from scipy.spatial import cKDTree
import capsule_sa_tangent as t
import capsule_sa_rotation as r
from capsule_sa_operator_generators import quadratic_stencil


def mode_tangent(values,mode=1):
    n=values.shape[1];m=np.fft.fftfreq(n)*n
    coeff=np.fft.fft(values,axis=1)
    coeff[:,abs(m)!=mode,:]=0
    return np.fft.ifft(coeff*(-1j*m[None,:,None]),axis=1).real


def run():
    points,w,groups,rz=r.sampling();theta=np.arange(r.N)*2*np.pi/r.N
    result={}
    for name in t.c.NAMES:
        old=np.load(t.old.ROOT/(name+'_1000_operators.npz'));C=old['C'];tree=cKDTree(C)
        flat=points.reshape(-1,3);samples={key:[] for key in ('U','p','nuTilda')};quality=[]
        for start in range(0,len(flat),4096):
            (dist,ids,weight),q=quadratic_stencil(tree,C,flat[start:start+4096])
            quality.append((float(dist[:,0].max()),q['max_condition'],q['max_weight_L1']))
            for key in samples:samples[key].append(np.einsum('nk,nkd->nd',weight,old[key][ids]))
        arrays={};record={}
        for key,chunks in samples.items():
            values=np.concatenate(chunks).reshape(points.shape[:2]+(-1,))
            if key=='U':values=r.cylindrical(values,theta[None,:])
            g=mode_tangent(values)
            arrays[key]=values;arrays[key+'_m1_tangent']=g
            # Body rings use volume weights; wake disks use area weights.
            # Never combine these different measures into one norm.
            record[key]={str(group):dict(modes=r.modes(values[groups==group],w[groups==group]),
                m1_tangent_norm=r.norm(g[groups==group],w[groups==group])) for group in np.unique(groups)}
        np.savez_compressed(t.ROOT/(name+'_LOW_MODES.npz'),points=points,weights=w,groups=groups,**arrays)
        record['sampling']=dict(rings=len(points),angles=r.N,points=len(flat),
            nearest_distance_max=max(x[0] for x in quality),condition_max=max(x[1] for x in quality),
            weight_L1_max=max(x[2] for x in quality),method='48-cell weighted quadratic, native finite-volume centers')
        record['status']='LowModeRingSeedConstructed; WholeMeshBoundaryConservativeLiftNotQualified'
        result[name]=record
        print(name,record['sampling'],flush=True)
    t.h.durable_json(t.ROOT/'LOW_MODES.json',t.h.seal(result))


if __name__=='__main__':run()
