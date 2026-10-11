"""Canonical arrays and volume-weighted equation-wise residual evidence."""
import numpy as np
import capsule_sa_operator as o
h=o.h;c=o.c


def canonical(parts,ids):
    n=sum(map(len,ids));assert np.array_equal(np.sort(np.concatenate(ids)),np.arange(n))
    result=np.empty((n,parts[0].shape[1]))
    for values,index in zip(parts,ids):
        assert len(values)==len(index)
        result[index]=values
    return result


def evidence_status(generator_qualified,conservative_flux_tangent,wall_tangent):
    if not all((generator_qualified,conservative_flux_tangent,wall_tangent)):
        return 'V4CapsuleSteadySAExactDiscreteOperatorEvidenceInsufficient'
    return 'QualifiedDirectionalEvidenceRequiresScientificInterpretation'

def load(name,step,key,dim=1):
    case=o.ROOT/'scratch'/name
    ids=[c.sa.binary_labels(case/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
    parts=[]
    for rank,index in enumerate(ids):
        v=np.fromfile(o.ROOT/'outputs'/name/str(step)/str(rank)/(key+'.bin'),dtype='<f8').reshape(-1,dim)
        assert len(v)==len(index);parts.append(v)
    return canonical(parts,ids)

def stats(a,vol,mask=None,centers=None):
    mask=np.ones(len(vol),bool) if mask is None else mask
    mag=np.linalg.norm(a,axis=1);ids=np.flatnonzero(mask);j=ids[np.argmax(mag[mask])]
    result=dict(cells=int(mask.sum()),volume=float(vol[mask].sum()),
        L1=float(np.average(mag[mask],weights=vol[mask])),L2=float(np.sqrt(np.average(mag[mask]**2,weights=vol[mask]))),
        Linf=float(mag[j]),Linf_cell=int(j),integrated_squared=float(np.sum(vol[mask]*mag[mask]**2)),
        component_RMS=np.sqrt(np.average(a[mask]**2,axis=0,weights=vol[mask])).tolist(),signed_integral=np.sum(a[mask]*vol[mask,None],axis=0).tolist())
    if centers is not None:result['Linf_coordinate']=centers[j].tolist()
    return result

def regions(name,centers):
    n=len(centers);prism=np.arange(n)<371646;r=np.linalg.norm(centers[:,:2],axis=1);z=centers[:,2]
    near=(r<.16)&(abs(z)<.16);wake=(r<.08)&(z>=.08)&(z<=.72)
    far=(np.max(abs(centers[:,:2]),axis=1)>.71)|(abs(z)>1.21)
    # Disjoint volume partition, prism first. Width 0.04 m is the sealed G1 cap.
    masks=dict(prism=prism,near_body_core=(~prism)&near,
        protected_wake=(~prism)&(~near)&wake,near_farfield=(~prism)&(~near)&(~wake)&far)
    masks['outer_core']=~np.logical_or.reduce(list(masks.values()))
    return masks

def baseline():
    result={}
    for name in c.NAMES:
        result[name]={}
        for step in (900,1000):
            C=load(name,step,'C',3);V=load(name,step,'V')[:,0];masks=regions(name,C)
            fields={k:load(name,step,k,d) for k,d in [('RU0',3),('RU1',3),('RN0',1),('RN1',1),('RC0',1),('RC1',1),('constitutive',1),('U',3),('p',1),('nuTilda',1),('nut',1),('gU',3),('gP',1),('gN',1),('gNT',1)]}
            for eq in ('RU','RN','RC'):assert np.array_equal(fields[eq+'0'],fields[eq+'1']),eq
            original=c.global_arrays(c.ROOT/name/'pilot',step)
            for f in ('U','p','nuTilda','nut'):assert np.array_equal(original[f],fields[f]),f
            record=dict(replay_exact=True,internal_inputs_unchanged=True,operators={},hashes={k:h.array_hash(v) for k,v in fields.items()})
            for eq in ('RU0','RN0','RC0','constitutive'):
                record['operators'][eq]=dict(all=stats(fields[eq],V,centers=C),regions={region:stats(fields[eq],V,mask,C) for region,mask in masks.items()})
            record['nut_RMS']=stats(fields['nut'],V)['L2']
            record['constitutive_relative_RMS']=record['operators']['constitutive']['all']['L2']/record['nut_RMS']
            result[name][str(step)]=record
            np.savez_compressed(o.ROOT/(name+'_'+str(step)+'_operators.npz'),C=C,V=V,**fields)
            print(name,step,{k:v['all']['L2'] for k,v in record['operators'].items()},'nut relative',record['constitutive_relative_RMS'],flush=True)
    h.durable_json(o.ROOT/'BASELINE.json',h.seal(result))

if __name__=='__main__':baseline()
