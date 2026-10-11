"""Read-only rotational analysis of retained SA states. Never launches a solver."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import brentq
import capsule_sa_stabilization as c
import capsule_steady_models_wall as wall

h=c.h
ROOT=c.ROOT.parent/'capsule_sa_rotation'
STEPS=(500,600,700,800,900,1000)
N=256
FIELDS=('U','p','nuTilda','nut')
HEAD='c90f336b2744d9fbe69a1a3a4de373ea121d41de'

def rotate(x,angle):
    out=np.array(x,copy=True); co=np.cos(angle);si=np.sin(angle)
    out[...,0]=co*x[...,0]-si*x[...,1]
    out[...,1]=si*x[...,0]+co*x[...,1]
    return out

def cylindrical(v,theta,inverse=False):
    return rotate(v,theta if inverse else -theta)

def shift(a,angle):
    """Active rotation in cylindrical components: q_new(theta)=q_old(theta-angle)."""
    modes=np.fft.fftfreq(a.shape[1])*a.shape[1]
    phase=np.exp(-1j*modes*angle)[None,:,None]
    spectrum=np.fft.fft(a,axis=1)
    if a.shape[1]%2==0:spectrum[:,a.shape[1]//2,:]=0
    return np.fft.ifft(spectrum*phase,axis=1).real

def inner(a,b,w):
    return float(np.sum(w[:,None,None]*a*b)/a.shape[1]/w.sum())

def norm(a,w):
    return float(np.sqrt(max(0,inner(a,a,w))))

def difference(a,b,w):
    d=a-b
    return dict(L2=norm(d,w),Linf=float(np.linalg.norm(d,axis=-1).max()))

def optimal(a,b,w):
    # Global angular grid followed by refinement of every local minimum.
    fa=np.fft.fft(a,axis=1);fb=np.fft.fft(b,axis=1)
    cross=np.sum(w[:,None,None]*fa*np.conj(fb),axis=(0,2))
    cross[0]=0  # Angle-independent mean must not consume minimizer precision.
    if a.shape[1]%2==0:cross[a.shape[1]//2]=0
    modes=np.fft.fftfreq(a.shape[1])*a.shape[1]
    def objective(t):return -float(np.real(np.sum(cross*np.exp(-1j*modes*t))))
    def derivative(t):return float(np.real(np.sum(1j*modes*cross*np.exp(-1j*modes*t))))
    grid=np.arange(a.shape[1])*2*np.pi/a.shape[1]-np.pi
    values=np.array([derivative(t) for t in grid]);step=2*np.pi/a.shape[1]
    candidates=[]
    for i in range(len(grid)):
        if values[i]<=0 and values[(i+1)%len(grid)]>0:
            t=brentq(derivative,grid[i],grid[i]+step,xtol=1e-13)
            candidates.append((objective(t),t))
    if not candidates:
        assert np.all(cross==0),'UnresolvedAngularMinimum'
        candidates=[(0.,0.)]
    angle=min(candidates)[1];angle=(angle+np.pi)%(2*np.pi)-np.pi
    return dict(angle_deg=float(np.degrees(angle)),**difference(shift(a,angle),b,w))

def projection(a,b,w):
    # One angular sample and half-sample provide a finite-angle consistency check.
    eps=2*np.pi/a.shape[1]
    g=(shift(a,eps)-shift(a,-eps))/(2*eps)
    g2=(shift(a,eps/2)-shift(a,-eps/2))/eps
    delta=b-a;den=inner(g,g,w);alpha=inner(delta,g,w)/den if den else 0.
    total=norm(delta,w);projected=norm(alpha*g,w);residual=norm(delta-alpha*g,w)
    return dict(alpha_rad=alpha,total=total,projected=projected,nonrotational=residual,
                explained_squared_fraction=projected**2/total**2 if total else None,
                generator_half_step_relative_difference=norm(g-g2,w)/norm(g2,w) if norm(g2,w) else 0.)

def modes(a,w):
    f=np.fft.fft(a,axis=1)/a.shape[1]
    energy=np.sum(w[:,None,None]*abs(f)**2,axis=(0,2))/w.sum()
    # Coherent aggregate complex coefficient; cancellation and local phase are retained separately.
    result={str(m):dict(rms_coefficient=float(np.sqrt(energy[m])),
          coherent_phase_deg=np.degrees(np.angle(np.sum(w[:,None]*f[:,m,:],axis=0))).tolist()) for m in (0,1,2)}
    result['dominant_nonaxisymmetric_mode']=int(np.argmax(energy[1:a.shape[1]//2])+1)
    result['nonaxisymmetric_rms']=float(np.sqrt(energy[1:].sum()))
    return result

def sampling():
    theta=np.arange(N)*2*np.pi/N
    # Ring midpoint area quadrature. Radial spacing matches the retained D/32 grid.
    rings=[];groups=[];weights=[]
    for station in (.5,1,2,4,6,8):
        for j in range(32):
            r=(j+.5)*.0025;rings.append((r,.08+.08*station));groups.append('wake_'+str(station))
            weights.append(np.pi*((j+1)**2-j*j)*.0025**2)
    # Body-surrounding fluid volume, bounded inside the square farfield.
    for z in np.linspace(-.12,.12,25):
        for r in np.arange(.045,.165,.01):
            rings.append((r,z));groups.append('body');weights.append(2*np.pi*r*.01*.01)
    rz=np.asarray(rings);pts=np.stack((rz[:,0,None]*np.cos(theta),rz[:,0,None]*np.sin(theta),np.broadcast_to(rz[:,1,None],(len(rz),N))),axis=-1)
    return pts,np.array(weights),np.array(groups),rz

def stencil(tree,pts,k=8):
    d,ids=tree.query(pts.reshape(-1,3),k=k,workers=1)
    if k==1:d=d[:,None];ids=ids[:,None]
    weights=1/np.maximum(d,1e-30)**2;weights/=weights.sum(axis=1)[:,None]
    return d,ids,weights

def sample(a,st,shape):
    _,ids,w=st
    return np.sum(a[ids]*w[:,:,None],axis=1).reshape(shape+(a.shape[-1],))

def verify():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    prior=h.read(c.ROOT/'RESULT.json');h.verify_record(prior)
    assert prior['identity']=='1ba9474d5c29f6f044938c599cc4f374431573cb81f82f2006aac7d3fa9541e7'
    for rel,digest in prior['evidence'].items():assert h.file_hash(c.ROOT/rel)==digest,rel
    for rel,digest in prior['implementation'].items():assert h.file_hash(c.s.f.prior.control.V4/rel)==digest,rel
    final=h.read(c.ROOT/'FINAL_VERIFICATION.json');h.verify_record(final)
    assert final['identity']=='d1f685ec5409a0c443ab0b6ee84edcefd46be783e9180a44dc1873e4859fd53f'
    retention=h.read(c.ROOT/'RETENTION.json');h.verify_record(retention)
    for rel,digest in retention['keep'].items():assert h.file_hash(c.ROOT/rel)==digest,rel
    out=dict(prior=prior['identity'],decision=prior['decision'],final_verification=final['identity'],cases={},retained_files=len(retention['keep']))
    for name in c.NAMES:
        con=c.audit(name);source=c.sa.mesh_source(name[3:]);candidate=h.read(source/'CANDIDATE.json');h.verify_record(candidate)
        assert candidate['identity']==con['candidate']
        for rel,digest in candidate['evidence'].items():assert h.file_hash(source/rel)==digest,rel
        family=h.read(source.parent/'FAMILY_CONTRACT.json');h.verify_record(family)
        assert family['identity']==candidate['family_contract']=='dd239298d7e671b70e1248120e6ebd51c25a52dd6c7013c10fbf3059d1eb0c90'
        for f,d in candidate['assembly']['hashes'].items():assert h.array_hash(np.load(source/'assembly'/(f+'.npy')))==d
        measured=h.read(c.ROOT/name/'MEASUREMENTS.json');h.verify_record(measured)
        out['cases'][name]=dict(candidate=candidate['identity'],frozen=con['frozen'],measurement=measured['identity'],
            prism_scientific=candidate['prism_scientific_identity'],prism_qualification=candidate['prism_qualification'],L11_authority=candidate['L11_authority'],sizing=family['identity'])
    return out

def extract():
    ROOT.mkdir(exist_ok=True)
    audit=verify();h.durable_json(ROOT/'ADMISSION.json',h.seal(audit))
    pts,w,groups,rz=sampling();np.savez_compressed(ROOT/'SAMPLING.npz',points=pts,weights=w,groups=groups,rz=rz)
    theta=np.arange(N)*2*np.pi/N
    diagnostics={}
    for name in c.NAMES:
        source=c.sa.mesh_source(name[3:]);case=c.ROOT/name/'pilot'
        x=np.load(source/'assembly/coordinates.npy');pr=np.load(source/'assembly/prisms.npy');tet=np.load(source/'assembly/tets.npy')
        centers=np.concatenate((x[pr].mean(axis=1),x[tet].mean(axis=1)));tree=cKDTree(centers)
        base=stencil(tree,pts);nearest=stencil(tree,pts,k=1)
        rows=h.read(c.ROOT/name/'HISTORY.json')['rows'];assert [r['iteration'] for r in rows]==list(range(1,1001))
        measured=h.read(c.ROOT/name/'MEASUREMENTS.json');diagnostics[name]={}
        for step in STEPS:
            arrays=c.global_arrays(case,step)
            hashes={f:hashlib.sha256(a.astype('<f8').tobytes()).hexdigest() for f,a in arrays.items()}
            for f,d in hashes.items():assert d==measured['checkpoints'][str(step)][f]['sha256']
            angle=-np.arctan2(rows[step-1]['force'][1],rows[step-1]['force'][0])
            query=rotate(pts,-angle);aligned=stencil(tree,query)
            save={};quality={}
            for f,a in arrays.items():
                lab=sample(a,base,pts.shape[:2]);near=sample(a,nearest,pts.shape[:2]);can=sample(a,aligned,pts.shape[:2])
                if f=='U':
                    lab=cylindrical(lab,theta[None,:]);near=cylindrical(near,theta[None,:])
                    can=cylindrical(rotate(can,angle),theta[None,:])
                save[f]=lab;save[f+'_aligned']=can
                quality[f]=dict(nearest_vs_IDW=difference(lab,near,w),direct_vs_spectral_alignment=difference(can,shift(lab,angle),w))
            save['force_angle']=np.array(-angle)
            np.savez_compressed(ROOT/(name+'_'+str(step)+'.npz'),**save)
            diagnostics[name][str(step)]=dict(hashes=hashes,force_alignment_deg=float(np.degrees(angle)),interpolation=quality,
                nearest_distance=c.statistics(base[0][:,0]),eighth_distance=c.statistics(base[0][:,-1]),
                aligned_nearest_distance=c.statistics(aligned[0][:,0]))
            print(name,step,'verified and sampled',flush=True)
    h.durable_json(ROOT/'EXTRACTION.json',h.seal(diagnostics))

def pair(a,b,weights,angle):
    raw=difference(a,b,weights);aligned=difference(shift(a,angle),b,weights)
    return dict(raw=raw,force_aligned=aligned,ratio=aligned['L2']/raw['L2'] if raw['L2'] else None,
                optimal=optimal(a,b,weights),projection=projection(a,b,weights))

def analyze():
    grid=np.load(ROOT/'SAMPLING.npz');w=grid['weights'];groups=grid['groups'];rz=grid['rz'];theta=np.arange(N)*2*np.pi/N
    result=dict(runs={},cross_seed={},discrete_residual='DiscreteResidualProjectionNotEvaluated',combined_state='Not evaluated: no arbitrary relative field weights')
    states={}
    for name in c.NAMES:
        rows=h.read(c.ROOT/name/'HISTORY.json')['rows'];angles=np.unwrap(np.arctan2(np.array([r['force'][1] for r in rows]),np.array([r['force'][0] for r in rows])))
        moments=np.unwrap(np.arctan2(np.array([r['moment'][1] for r in rows]),np.array([r['moment'][0] for r in rows])))
        run=dict(rates={},states={},pairs={});result['runs'][name]=run;states[name]={}
        for label,series in [('force',angles),('moment',moments)]:
            run['rates'][label]={}
            for lo,hi in [(801,900),(901,1000),(951,1000)]:
                v=np.degrees(series[lo-1:hi]);st=c.statistics(v)
                st['mean_first_difference']=float(np.diff(v).mean());st['rate_std']=float(np.diff(v).std())
                st['quadratic_acceleration']=float(2*np.polyfit(np.arange(len(v)),v,2)[0])
                run['rates'][label][f'{lo}-{hi}']=st
        for step in STEPS:
            data=np.load(ROOT/(name+'_'+str(step)+'.npz'));states[name][step]=data
            state=dict(force_angle_deg=float(np.degrees(angles[step-1])),moment_angle_deg=float(np.degrees(moments[step-1])),
                force_moment_offset_deg=float(np.degrees(np.angle(np.exp(1j*(moments[step-1]-angles[step-1]))))),regions={})
            run['states'][str(step)]=state
            for group in dict.fromkeys(groups):
                mask=groups==group;wg=w[mask];region=dict(modes={f:modes(data[f][mask],wg) for f in FIELDS})
                if group.startswith('wake'):
                    deficit=np.maximum(33.528-data['U'][mask,:,2],0);mass=np.sum(wg[:,None]*deficit)/N
                    xy=rz[mask,0,None]*np.exp(1j*theta[None,:]);centroid=np.sum(wg[:,None]*deficit*xy)/N/mass
                    width=np.sqrt(np.sum(wg[:,None]*deficit*abs(xy-centroid)**2)/N/mass)
                    region['wake']=dict(deficit_integral=float(mass),centroid=[float(centroid.real),float(centroid.imag)],centroid_magnitude=float(abs(centroid)),centroid_angle_deg=float(np.degrees(np.angle(centroid))),rms_width=float(width),force_offset_deg=float(np.degrees(np.angle(centroid*np.exp(-1j*angles[step-1])))))
                state['regions'][group]=region
        for i,j in [(500,600),(600,700),(700,800),(800,900),(900,1000),(600,1000),(800,1000)]:
            key=f'{i}-{j}';a=states[name][i];b=states[name][j];angle=angles[j-1]-angles[i-1]
            run['pairs'][key]={}
            for group in ('body','wake'):
                mask=groups=='body' if group=='body' else np.char.startswith(groups,'wake');wg=w[mask]
                run['pairs'][key][group]={f:pair(a[f][mask],b[f][mask],wg,angle) for f in FIELDS}
    a=states[c.NAMES[0]][1000];b=states[c.NAMES[1]][1000]
    for group in ('body','wake'):
        mask=groups=='body' if group=='body' else np.char.startswith(groups,'wake');wg=w[mask]
        result['cross_seed'][group]={}
        for f in FIELDS:
            raw=difference(a[f][mask],b[f][mask],wg);aligned=difference(a[f+'_aligned'][mask],b[f+'_aligned'][mask],wg)
            result['cross_seed'][group][f]=dict(raw=raw,aligned=aligned,ratio=aligned['L2']/raw['L2'],optimal=optimal(a[f][mask],b[f][mask],wg))
    h.durable_json(ROOT/'ANALYSIS.json',h.seal(result))
    print('analysis sealed',flush=True)

if __name__=='__main__':
    import sys
    {'extract':extract,'analyze':analyze,'verify':lambda:print(verify())}[sys.argv[1]]()
