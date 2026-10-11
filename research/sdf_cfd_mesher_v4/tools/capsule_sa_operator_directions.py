"""Explicit augmented-state tangent paths; none is a SIMPLE iteration Jacobian."""
import numpy as np
from pathlib import Path
import capsule_sa_operator_analysis as a
o=a.o;c=o.c;h=o.h


def inner(x,y,v):
    return float(np.sum(v[:,None]*x*y)/v.sum())


def rms(x,v):return np.sqrt(inner(x,x,v))


def normalize(x,v,scale=1.):
    n=rms(x,v)
    if not np.isfinite(n) or n<=0:raise ValueError('Zero/nonfinite direction')
    return x*(scale/n)


def projection(x,g,v):
    gg=inner(g,g,v)
    if gg==0:return dict(status='ZeroDirectionProjectionUndefined')
    beta=inner(x,g,v)/gg
    residual=x-beta*g
    return dict(beta=beta,total=rms(x,v),projected=rms(beta*g,v),remainder=rms(residual,v),
                squared_fraction=inner(beta*g,beta*g,v)/inner(x,x,v) if inner(x,x,v)>0 else None)


def prepare():
    metadata={};lines=['source /opt/openfoam13/etc/bashrc','set -e']
    for name in c.NAMES:
        d=np.load(o.ROOT/(name+'_1000_operators.npz'));old=np.load(o.ROOT/(name+'_900_operators.npz'))
        V=d['V'];C=d['C'];U=d['U'];N=d['nuTilda'];sc=1.1302005732789914e-5
        zU=np.zeros_like(U);zN=np.zeros_like(N)
        stream=zU.copy();stream[:,2]=U[:,2]
        strain=zU.copy();strain[:,0]=C[:,0];strain[:,1]=-C[:,1]
        directions={
            'zero_control':(zU,zN,zN,zN),
            'U_rotation':(normalize(d['gU'],V),zN,zN,zN),
            'U_streamwise':(normalize(stream,V),zN,zN,zN),
            'U_lateral_strain':(normalize(strain,V),zN,zN,zN),
            'U_late_change':(normalize(U-old['U'],V),zN,zN,zN),
            'SA_rotation':(zU,zN,normalize(d['gN'],V,sc),zN),
            'SA_amplitude':(zU,zN,normalize(N,V,sc),zN),
            'SA_late_change':(zU,zN,normalize(N-old['nuTilda'],V,sc),zN),
            'rotation_augmented':(d['gU'],d['gP'],d['gN'],d['gNT']),
        }
        meta={};case=o.ROOT/'scratch'/name
        ids=[c.sa.binary_labels(case/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
        for label,arrays in directions.items():
            folder=o.ROOT/'directions'/name/label
            for rank,index in enumerate(ids):
                (folder/str(rank)).mkdir(parents=True,exist_ok=True)
                for key,values in zip(('dU','dp','dN','dNT'),arrays):
                    np.asarray(values[index],dtype='<f8').tofile(folder/str(rank)/(key+'.bin'))
            meta[label]=dict(norms={key:rms(values,V) for key,values in zip(('U','p','nuTilda','nut'),arrays)},
                             hashes={key:h.array_hash(values) for key,values in zip(('U','p','nuTilda','nut'),arrays)})
            # Field-block comparisons use fixed retained phi/nut. Full augmented
            # rotation also gets an explicitly conditional velocity-flux path.
            modes=('fixed','velocity') if label=='rotation_augmented' else ('fixed',)
            for mode in modes:
                out=o.ROOT/'outputs'/name/(label+'_'+mode);log=o.ROOT/(name+'_'+label+'_'+mode+'.log')
                lines.append(f"/usr/bin/time -v mpirun -np 6 {o.linux(o.ROOT/'build/bin/saReadOnlyOperator')} -parallel -case {o.linux(case)} -snapshot 1000 -direction {o.linux(folder)} -fluxMode {mode} -output {o.linux(out)} > {o.linux(log)} 2>&1")
        meta['field_change_projection']={f:projection(d[f]-old[f],d[g],V) for f,g in [('U','gU'),('p','gP'),('nuTilda','gN'),('nut','gNT')]}
        metadata[name]=meta
    h.durable_json(o.ROOT/'DIRECTIONS.json',h.seal(dict(cases=metadata,epsilon=[.01,.001,.0001,.00001,.000001,.0000001],
        limitation='Augmented-state block actions: retained physical nut boundary; fixed phi or anchored velocity-induced flux. Not the reduced nonlinear/SIMPLE Jacobian.')))
    (o.ROOT/'evaluate_directions.sh').write_text('\n'.join(lines)+'\n',newline='\n')


def analyze():
    metadata=h.read(o.ROOT/'DIRECTIONS.json');result={}
    for name in c.NAMES:
        d=np.load(o.ROOT/(name+'_1000_operators.npz'));V=d['V'];C=d['C'];regions=a.regions(name,C)
        result[name]={}
        for label in metadata['cases'][name]:
            if label=='field_change_projection':continue
            for mode in (('fixed','velocity') if label=='rotation_augmented' else ('fixed',)):
                tag=label+'_'+mode;record={};previous={}
                for ei,eps in enumerate(metadata['epsilon']):
                    item={}
                    for eq,dim,base in [('JU',3,'RU0'),('JN',1,'RN0'),('JC',1,'RC0')]:
                        x=a.load(name,tag,eq+str(ei),dim)
                        st=a.stats(x,V,centers=C)
                        st['residual_projection']=projection(d[base],x,V)
                        if eq in previous:st['relative_epsilon_change']=rms(x-previous[eq],V)/max(rms(x,V),np.finfo(float).tiny)
                        if ei==len(metadata['epsilon'])-1:
                            st['regions']={r:a.stats(x,V,m,C) for r,m in regions.items()}
                            bits=a.load(name,tag,'patchBits')[:,0].astype(int)
                            st['box_patches']={p:a.stats(x,V,(bits&(1<<j))!=0,C) for j,p in enumerate(('xmin','xmax','ymin','ymax','zmin','zmax'))}
                            np.savez_compressed(o.ROOT/(name+'_'+tag+'_'+eq+'.npz'),values=x)
                        previous[eq]=x;item[eq]=st
                    record[str(eps)]=item
                result[name][tag]=record
        # Same field-space normalization; no cross-equation norm is constructed.
        result[name]['ratios']={}
        for eq in ('JU','JN'):
            for group,rot,controls in [('U','U_rotation_fixed',['U_streamwise_fixed','U_lateral_strain_fixed','U_late_change_fixed']),('SA','SA_rotation_fixed',['SA_amplitude_fixed','SA_late_change_fixed'])]:
                if eq=='JU' and group=='SA':continue  # Frozen nut decouples this block; only roundoff remains.
                result[name]['ratios'][eq+'_'+group]={control:{e:result[name][rot][e][eq]['L2']/max(result[name][control][e][eq]['L2'],1e-300) for e in map(str,metadata['epsilon'])} for control in controls}
        print(name,'directional actions analyzed',flush=True)
    h.durable_json(o.ROOT/'JACOBIAN.json',h.seal(result))


if __name__=='__main__':
    import sys
    {'prepare':prepare,'analyze':analyze}[sys.argv[1]]()
