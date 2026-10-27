"""Analyze the projection separately from qualification of a physical tangent."""
import numpy as np
import capsule_sa_tangent as t
import capsule_sa_operator_analysis as a
import capsule_sa_operator_directions as d


def load(name,key,dim=1,canonical=True):
    folder=t.ROOT/'outputs'/name
    arrays=[np.fromfile(folder/str(i)/(key+'.bin'),dtype='<f8').reshape(-1,dim) for i in range(6)]
    if not canonical:return np.concatenate(arrays)
    case=t.old.ROOT/'scratch'/name
    ids=[t.c.sa.binary_labels(case/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
    return a.canonical(arrays,ids)


def faces(name,key):
    parts=[];indices=[]
    for rank in range(6):
        addressing=t.c.sa.binary_labels(t.old.ROOT/'scratch'/name/f'processor{rank}/constant/polyMesh/faceProcAddressing')
        x=np.fromfile(t.ROOT/'outputs'/name/str(rank)/(key+'.bin'),dtype='<f8')
        assert len(x)==len(addressing)
        parts.append(x*np.sign(addressing));indices.append(abs(addressing)-1)
    n=max(x.max() for x in indices)+1;out=np.full(n,np.nan);difference=0.
    for x,ids in zip(parts,indices):
        old=out[ids];valid=np.isfinite(old)
        if valid.any():difference=max(difference,float(abs(old[valid]-x[valid]).max()))
        out[ids]=x
    assert np.isfinite(out).all()
    return out,difference


def analyze():
    result={}
    for name in t.c.NAMES:
        V=load(name,'V')[:,0];C=load(name,'C',3)
        native=load(name,'native',3);unlimited=load(name,'unlimited',3)
        old=np.load(t.old.ROOT/(name+'_1000_operators.npz'))
        previous=np.load(t.old.ROOT/(name+'_generator_samples_order2.npz'))
        ids=previous['ids'];weights=previous['weights'];rec=previous['U']
        record=dict(generator=dict(
            native_norm=d.rms(native,V),unlimited_norm=d.rms(unlimited,V),
            limiter_change_norm=d.rms(unlimited-native,V),
            reconstruction_native_cosine=d.inner(rec,native[ids],weights)/(d.rms(rec,weights)*d.rms(native[ids],weights)),
            reconstruction_unlimited_cosine=d.inner(rec,unlimited[ids],weights)/(d.rms(rec,weights)*d.rms(unlimited[ids],weights)),
            reconstruction_unlimited_difference=d.rms(rec-unlimited[ids],weights),
            reconstruction_norm=d.rms(rec,weights)))
        for seed in ('native','unlimited'):
            before=load(name,seed+'0_before');after=load(name,seed+'0_after')
            du=load(name,seed+'0_du',3);corr=load(name,seed+'0_correction',3)
            initial=native if seed=='native' else unlimited
            cell=load(name,seed+'0_cellFlux')
            boundary={}
            for patch in ('wall','farfield'):
                x=load(name,seed+'0_after_'+patch,canonical=False)[:,0]
                mismatch=load(name,seed+'0_mismatch_'+patch,canonical=False)[:,0]
                boundary[patch]=dict(net=float(x.sum()),Linf=float(abs(x).max()),
                    cell_flux_mismatch_Linf=float(abs(mismatch).max()),cell_flux_mismatch_L2=float(np.sqrt(np.mean(mismatch*mismatch))))
            defect=load(name,seed+'0_pressureBCdefect',canonical=False)
            replay={key:bool(np.array_equal(load(name,seed+'0_'+key,dim),load(name,seed+'1_'+key,dim)))
                    for key,dim in [('before',1),('after',1),('du',3),('psi',1)]}
            record[seed]=dict(before=a.stats(before,V,centers=C),after=a.stats(after,V,centers=C),
                divergence_reduction=d.rms(after,V)/d.rms(before,V),
                seed_norm=d.rms(initial,V),correction_norm=d.rms(corr,V),corrected_norm=d.rms(du,V),
                correction_over_seed=d.rms(corr,V)/d.rms(initial,V),
                corrected_seed_cosine=d.inner(du,initial,V)/(d.rms(du,V)*d.rms(initial,V)),
                cell_flux_divergence=d.rms(cell,V),boundary=boundary,
                pressure_BC_defect_Linf=float(abs(defect).max()),pressure_BC_defect_RMS=float(np.sqrt(np.mean(defect*defect))),
                replay=replay,regions={r:a.stats(after,V,m,C) for r,m in a.regions(name,C).items()},
                hashes={key:t.h.array_hash(load(name,seed+'0_'+key,dim)) for key,dim in [('du',3),('after',1),('psi',1)]})
        N=old['nuTilda'];gN=old['gN'];der=t.sa_derivative(N)*gN
        record['SA_interior']={}
        for e in (1e-3,1e-4,1e-5,1e-6):
            fd=(t.sa_nut(N+e*gN)-t.sa_nut(N-e*gN))/(2*e)
            record['SA_interior'][str(e)]=dict(relative_error=d.rms(fd-der,V)/d.rms(der,V),
                min_perturbed_N=float(np.minimum(N+e*gN,N-e*gN).min()))
        record['wall_nut']={};previous=None
        for ei,e in enumerate((1e-3,1e-4,1e-5,1e-6)):
            x=load(name,'wallNutDerivative'+str(ei)+'_wall',canonical=False)
            q=dict(RMS=float(np.sqrt(np.mean(x*x))),Linf=float(abs(x).max()))
            if previous is not None:q['relative_array_change']=float(np.linalg.norm(x-previous)/np.linalg.norm(x))
            previous=x;record['wall_nut'][str(e)]=q
        record['validation']={}
        for label in ('conservativeValidation','nonconservativeValidation'):
            b=load(name,label+'0_before');p=load(name,label+'0_after')
            f,dup=faces(name,label+'0_allFaces');f0,_=faces(name,label+'0_seedFaces')
            fr,_=faces(name,label+'1_allFaces')
            record['validation'][label]=dict(before_RMS=d.rms(b,V),after_RMS=d.rms(p,V),
                before_Linf=float(abs(b).max()),after_Linf=float(abs(p).max()),
                flux_correction_relative=float(np.linalg.norm(f-f0)/np.linalg.norm(f0)),
                processor_face_duplicate_Linf=dup,replay_exact=bool(np.array_equal(f,fr)))
        for seed in ('native','unlimited'):
            f,dup=faces(name,seed+'0_allFaces');fr,_=faces(name,seed+'1_allFaces')
            record[seed]['flux_canonical']=dict(sha256=t.h.array_hash(f),faces=len(f),
                processor_duplicate_Linf=dup,replay_exact=bool(np.array_equal(f,fr)))
            assert all(record[seed]['replay'].values()) and np.array_equal(f,fr)
        result[name]=record
        print(name,record['generator'],flush=True)
        for seed in ('native','unlimited'):
            q=record[seed];print(seed,{k:q[k] for k in ('divergence_reduction','correction_over_seed','corrected_seed_cosine','cell_flux_divergence','pressure_BC_defect_Linf')},q['boundary'],flush=True)
    t.h.durable_json(t.ROOT/'PROJECTION.json',t.h.seal(result))


if __name__=='__main__':analyze()
