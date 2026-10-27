"""Native wall post-processing on ordinary scratch copies, never retained cases."""
import shutil
import subprocess
import numpy as np
from scipy.spatial import cKDTree
import capsule_sa_rotation as r
import capsule_steady_models_wall_stress as stress
w=r.wall;h=r.h;c=r.c

def extract():
    source='/opt/openfoam13/src/functionObjects/field/wallShearStress/wallShearStress.C'
    snapshot=subprocess.check_output(['wsl','cat',source])
    assert snapshot==(c.s.ROOT/'model_sources/wallShearStress.C').read_bytes()
    (r.ROOT/'wallShearStress.C').write_bytes(snapshot)
    evidence={}
    theta=np.arange(r.N)*2*np.pi/r.N
    z=-.08+(np.arange(64)+.5)*.16/64
    radius=np.sqrt(.04**2-np.maximum(abs(z)-.04,0)**2)
    pts=np.stack((radius[:,None]*np.cos(theta),radius[:,None]*np.sin(theta),np.broadcast_to(z[:,None],(64,r.N))),axis=-1)
    np.savez_compressed(r.ROOT/'WALL_SAMPLING.npz',points=pts,z=z,weights=np.ones(64)*2*np.pi*.04*.16/64)
    for name in c.NAMES:
        old=c.ROOT/name/'pilot';target=r.ROOT/'scratch'/name/'pilot'
        target.mkdir(parents=True,exist_ok=False)
        for folder in ('constant','system'):
            shutil.copytree(old/folder,target/folder)
        for rank in range(6):
            shutil.copytree(old/f'processor{rank}/constant',target/f'processor{rank}/constant')
            for step in r.STEPS:
                shutil.copytree(old/f'processor{rank}/{step}',target/f'processor{rank}/{step}')
        before={str(p.relative_to(target)):h.file_hash(p) for p in target.rglob('*') if p.is_file()}
        folder=target.parent;linux='/mnt/c/'+str(folder.resolve())[3:].replace('\\','/')
        script=r.ROOT/(name+'_wall.sh')
        script.write_text('source /opt/openfoam13/etc/bashrc\nset -e\ncd '+linux+'\nmpirun -np 6 foamPostProcess -solver incompressibleFluid -parallel -case pilot -time 500,600,700,800,900,1000 -func wallShearStress\n',newline='\n')
        with (r.ROOT/(name+'_wall.log')).open('w') as log:
            subprocess.run(['wsl','bash','/mnt/c/'+str(script.resolve())[3:].replace('\\','/')],stdout=log,stderr=subprocess.STDOUT,check=True)
        for rel,d in before.items():assert h.file_hash(target/rel)==d,rel
        geometry=np.load(c.s.ROOT/'wall_reference'/name[3:]/'WALL_YPLUS.npz')
        tree=cKDTree(geometry['centers']);base=r.stencil(tree,pts)
        rows=h.read(c.ROOT/name/'HISTORY.json')['rows'];start,count=w.patch_layout(old/'constant/polyMesh/boundary')
        evidence[name]=dict(scratch_inputs=before,preexisting_scratch_files_unchanged=True,checkpoints={})
        for step in r.STEPS:
            shear=np.full((count,3),np.nan);pressure=np.full((count,1),np.nan);seen=np.zeros(count,int)
            for rank in range(6):
                proc=target/f'processor{rank}';first,n=w.patch_layout(proc/'constant/polyMesh/boundary')
                ids=abs(w.labels(proc/'constant/polyMesh/faceProcAddressing')[first:first+n])-1-start
                seen[ids]+=1;shear[ids]=stress.vector_patch(proc/str(step)/'wallShearStress',n)
                pressure[ids,0]=w.pressure_patch(proc,str(step))
            assert np.all(seen==1) and np.isfinite(shear).all() and np.isfinite(pressure).all()
            angle=-np.arctan2(rows[step-1]['force'][1],rows[step-1]['force'][0]);aligned=r.stencil(tree,r.rotate(pts,-angle))
            lab=r.cylindrical(r.sample(shear,base,(64,r.N)),theta[None,:])
            can=r.cylindrical(r.rotate(r.sample(shear,aligned,(64,r.N)),angle),theta[None,:])
            p=r.sample(pressure,base,(64,r.N));pcan=r.sample(pressure,aligned,(64,r.N))
            np.savez_compressed(r.ROOT/(name+'_wall_'+str(step)+'.npz'),shear=lab,shear_aligned=can,p=p,p_aligned=pcan)
            evidence[name]['checkpoints'][str(step)]=dict(pressure_hash=h.array_hash(pressure),shear_hash=h.array_hash(shear),
                nearest_distance=c.statistics(base[0][:,0]),eighth_distance=c.statistics(base[0][:,-1]),
                direct_vs_spectral_shear=r.difference(can,r.shift(lab,angle),np.ones(64)),
                direct_vs_spectral_pressure=r.difference(pcan,r.shift(p,angle),np.ones(64)))
        print(name,'wall postprocessing complete; scratch input bytes unchanged',flush=True)
    h.durable_json(r.ROOT/'WALL_EXTRACTION.json',h.seal(evidence))

def analyze():
    grid=np.load(r.ROOT/'WALL_SAMPLING.npz');z=grid['z'];weights=grid['weights'];result={}
    for name in c.NAMES:
        rows=h.read(c.ROOT/name/'HISTORY.json')['rows'];run=dict(states={},pairs={});result[name]=run
        for step in r.STEPS:
            d=np.load(r.ROOT/(name+'_wall_'+str(step)+'.npz'))
            # +Z projected tangent in cylindrical coordinates.
            nr=np.sqrt(.04**2-np.maximum(abs(z)-.04,0)**2)/.04
            nz=np.sign(z)*np.maximum(abs(z)-.04,0)/.04
            axial=-nz[:,None]*d['shear'][:,:,0]+nr[:,None]*d['shear'][:,:,2]
            sign=np.sign(np.median(axial[abs(z)<.04]))
            state={}
            for region,mask in [('all',np.ones(len(z),bool)),('rear',z>.04),('body',abs(z)<=.04)]:
                p=d['p'][mask];shear=d['shear'][mask];wg=weights[mask]
                state[region]=dict(pressure_modes=r.modes(p,wg),Cp=c.statistics((2*p/33.528**2).ravel()),
                    shear_magnitude_Pa=c.statistics((1.225*np.linalg.norm(shear,axis=-1)).ravel()),
                    reverse_shear_area_fraction=float(np.mean(axial[mask]*sign<0)))
            run['states'][str(step)]=state
        for i,j in [(600,700),(700,800),(800,900),(900,1000),(800,1000)]:
            a=np.load(r.ROOT/(name+'_wall_'+str(i)+'.npz'));b=np.load(r.ROOT/(name+'_wall_'+str(j)+'.npz'))
            angle=np.arctan2(rows[j-1]['force'][1],rows[j-1]['force'][0])-np.arctan2(rows[i-1]['force'][1],rows[i-1]['force'][0])
            run['pairs'][f'{i}-{j}']={f:r.pair(a[f],b[f],weights,angle) for f in ('p','shear')}
    a=np.load(r.ROOT/(c.NAMES[0]+'_wall_1000.npz'));b=np.load(r.ROOT/(c.NAMES[1]+'_wall_1000.npz'))
    result['cross_seed']={f:dict(raw=r.difference(a[f],b[f],weights),aligned=r.difference(a[f+'_aligned'],b[f+'_aligned'],weights)) for f in ('p','shear')}
    h.durable_json(r.ROOT/'WALL_ANALYSIS.json',h.seal(result))

if __name__=='__main__':
    import sys
    {'extract':extract,'analyze':analyze}[sys.argv[1]]()
