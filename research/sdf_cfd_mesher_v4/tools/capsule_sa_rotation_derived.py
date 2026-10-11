"""Supplement sealed vector results with explicit speed/deficit/shear-magnitude norms."""
import numpy as np
import capsule_sa_rotation as r
h=r.h

def fields(d,suffix=''):
    u=d['U'+suffix]
    return dict(speed=np.linalg.norm(u,axis=-1,keepdims=True),
                positive_axial_deficit=np.maximum(33.528-u[...,2:3],0),
                lateral_velocity=u[...,:2])

def run():
    parent=h.read(r.ROOT/'RESULT.json');h.verify_record(parent)
    for rel,digest in parent['evidence'].items():assert h.file_hash(r.ROOT/rel)==digest
    grid=np.load(r.ROOT/'SAMPLING.npz');groups=grid['groups'];w=grid['weights']
    result=dict(parent=parent['identity'],pairs={},cross_seed={},source_sha256=h.file_hash(__file__))
    for name in r.c.NAMES:
        result['pairs'][name]={}
        for i,j in [(800,900),(900,1000),(800,1000)]:
            a=np.load(r.ROOT/(name+'_linear_'+str(i)+'.npz'));b=np.load(r.ROOT/(name+'_linear_'+str(j)+'.npz'))
            angle=float(b['force_angle']-a['force_angle']);fa=fields(a);fb=fields(b)
            result['pairs'][name][f'{i}-{j}']={}
            for group in dict.fromkeys(groups):
                mask=groups==group;wg=w[mask]
                result['pairs'][name][f'{i}-{j}'][group]={f:r.pair(fa[f][mask],fb[f][mask],wg,angle) for f in fa}
            aw=np.load(r.ROOT/(name+'_wall_'+str(i)+'.npz'));bw=np.load(r.ROOT/(name+'_wall_'+str(j)+'.npz'))
            result['pairs'][name][f'{i}-{j}']['wall_shear_magnitude']=r.pair(np.linalg.norm(aw['shear'],axis=-1,keepdims=True),np.linalg.norm(bw['shear'],axis=-1,keepdims=True),np.ones(64),angle)
    a=np.load(r.ROOT/(r.c.NAMES[0]+'_linear_1000.npz'));b=np.load(r.ROOT/(r.c.NAMES[1]+'_linear_1000.npz'))
    aa=fields(a,'_aligned');bb=fields(b,'_aligned');fa=fields(a);fb=fields(b)
    for group in dict.fromkeys(groups):
        mask=groups==group;wg=w[mask]
        result['cross_seed'][group]={f:dict(raw=r.difference(fa[f][mask],fb[f][mask],wg),aligned=r.difference(aa[f][mask],bb[f][mask],wg)) for f in fa}
    # Additive record: no previous seal or report is overwritten.
    h.durable_json(r.ROOT/'DERIVED.json',h.seal(result))
    print(h.read(r.ROOT/'DERIVED.json')['identity'])
    for name in r.c.NAMES:
        print(name,'900-1000', {g:{f:(v['raw']['L2'],v['force_aligned']['L2']) for f,v in fs.items()} for g,fs in result['pairs'][name]['900-1000'].items() if g in ('body','wake_0.5','wake_2','wake_8')})
        print('wall shear magnitude',result['pairs'][name]['900-1000']['wall_shear_magnitude'])

if __name__=='__main__':run()
