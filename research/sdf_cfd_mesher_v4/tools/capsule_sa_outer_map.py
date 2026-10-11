"""Complete temporary steady-iteration map: blockwise analysis, never continuation."""
import hashlib
import re
import subprocess
from functools import lru_cache
import numpy as np
import capsule_sa_pressure_map as prior

h=prior.h;c=prior.c
ROOT=prior.ROOT.parent/'capsule_sa_outer_map'
HEAD='1a181102a88c04bcea150861488579aba3d88ac4'
FIELDS={'U':3,'p':1,'N':1,'NT':1,'phi':1}
EPS=(1e-3,1e-4,1e-5,1e-6)


def inner(x,y,w):
    return float(np.sum(w*np.sum(x*y,axis=1))/np.sum(w))


def norm(x,w):return float(np.sqrt(inner(x,x,w)))


def response(g,y,w):
    ng=norm(g,w);ny=norm(y,w)
    if ng==0:return dict(input_RMS=ng,output_RMS=ny,zero_input=True)
    mu=inner(g,y,w)/(ng*ng)
    return dict(input_RMS=ng,output_RMS=ny,gain=ny/ng,
        cosine=inner(g,y,w)/(ng*ny) if ny else None,retention=mu,
        orthogonal_fraction=norm(y-mu*g,w)/ny if ny else None,
        identity_error=norm(y-g,w)/ng)


@lru_cache(None)
def addressing(name,kind):
    return [c.sa.binary_labels(prior.previous.old.ROOT/'scratch'/name/f'processor{i}/constant/polyMesh/{kind}ProcAddressing') for i in range(6)]


def load(name,direction,key,dim=1,face=False,unsigned=False):
    parts=[np.fromfile(ROOT/'outputs'/name/direction/str(i)/(key+'.bin'),dtype='<f8').reshape(-1,dim) for i in range(6)]
    if not face:return prior.metrics.canonical(parts,addressing(name,'cell'))
    ids=addressing(name,'face');out=np.full((max(abs(i).max() for i in ids),dim),np.nan)
    for x,index in zip(parts,ids):out[abs(index)-1]=x if unsigned else x*np.sign(index)[:,None]
    assert np.isfinite(out).all();return out


def field(name,direction,prefix,k):return load(name,direction,prefix+k,FIELDS[k],k=='phi')


@lru_cache(4)
def weights(name,direction,k):
    return load(name,direction,'geometry_area',face=True,unsigned=True)[:,0] if k=='phi' else load(name,direction,'V')[:,0]


def summary(x,w):return dict(RMS=norm(x,w),Linf=float(abs(x).max()),sha256=h.array_hash(x))


def base():
    result={}
    for name in c.NAMES:
        V=load(name,'base','V')[:,0];C=load(name,'base','C',3)
        late=np.load(prior.previous.old.ROOT/(name+'_900_operators.npz'))
        original=np.load(prior.previous.old.ROOT/(name+'_1000_operators.npz'))
        regions=prior.metrics.regions(name,C);record={}
        for k,dim in FIELDS.items():
            q=field(name,'base','original_',k);s=field(name,'base','base0_',k)
            replay=field(name,'base','base1_',k)
            rk={'N':'nuTilda','NT':'nut'}.get(k,k)
            ref=load(name,'reference','reference_'+rk,dim,k=='phi')
            w=weights(name,'base',k);delta=s-q
            entry=dict(output=summary(s,w),defect=summary(delta,w),relative_defect=norm(delta,w)/norm(q,w),
                replay_exact=bool(np.array_equal(s,replay)),native_reference_exact=bool(np.array_equal(s,ref)))
            assert entry['replay_exact'] and entry['native_reference_exact'],(name,k)
            if k!='phi':
                observed=original[rk].reshape(-1,dim)-late[rk].reshape(-1,dim)
                entry['late_change_relation']=response(observed,delta,w)
                entry['regions']={r:summary(delta[mask],w[mask]) for r,mask in regions.items()}
            record[k]=entry
        result[name]=record
    h.durable_json(ROOT/'BASE.json',h.seal(result));return result


def analyze_direction(direction):
    result={}
    for name in c.NAMES:
        folder=ROOT/'outputs'/name/direction
        eps=(1e-6,1e-7,1e-8) if direction.endswith('_fine') else EPS
        count=len(eps)
        if not (folder/f'5/d{count}_U.bin').exists():continue
        V=load(name,direction,'V')[:,0];C=load(name,direction,'C',3);regions=prior.metrics.regions(name,C)
        record=[];previous={}
        for i,e in enumerate(eps):
            row=dict(epsilon=e,fields={})
            for k in FIELDS:
                g=field(name,direction,f'g{i}_',k);y=field(name,direction,f'd{i}_',k);w=weights(name,direction,k)
                assert np.isfinite(y).all() and np.isfinite(g).all()
                item=response(g,y,w);item['sha256']=h.array_hash(y)
                if k in previous:item['adjacent_array_difference']=norm(y-previous[k],w)/max(norm(y,w),np.finfo(float).tiny)
                previous[k]=y
                even=field(name,direction,f'even{i}_',k)
                item['centered_defect_RMS']=norm(even,w)
                item['centered_defect_over_perturbation']=norm(even,w)/(e*norm(g,w)) if norm(g,w) else None
                q=field(name,direction,'original_',k);s=field(name,direction,'base0_',k)
                item['fixed_point_defect_projection']=response(g,s-q,w)
                if k!='phi':
                    diff=y-g;total=float(np.sum(w*np.sum(diff*diff,axis=1)))
                    item['regions']={r:dict(identity_error_RMS=norm(diff[mask],w[mask]),
                        squared_identity_error_share=float(np.sum(w[mask]*np.sum(diff[mask]**2,axis=1)))/total if total else 0.) for r,mask in regions.items()}
                row['fields'][k]=item
            gu=field(name,direction,f'g{i}_','U')
            row['stages']={stage:response(gu,load(name,direction,f'd{i}_'+stage,3),V) for stage in ('momentumU','HbyA','preBoundaryU','U')}
            row['continuity']=summary(load(name,direction,f'd{i}_div'),V)
            record.append(row)
        replay={}
        for k in FIELDS:
            replay[k]=bool(np.array_equal(field(name,direction,f'd{count-1}_',k),field(name,direction,f'd{count}_',k)))
        assert all(replay.values()),(name,direction,replay)
        result[name]=dict(epsilon_study=record,replay=replay)
        print(name,direction,[(r['epsilon'],r['fields']['U'].get('gain'),r['fields']['U'].get('retention'),r['fields']['U'].get('adjacent_array_difference')) for r in record],flush=True)
    h.durable_json(ROOT/(direction.upper()+'.json'),h.seal(result));return result


def verify():
    result=h.read(prior.ROOT/'RESULT.json');h.verify_record(result)
    assert result['identity']=='d82f89aff454aa73404706e8407ea8854d2ad7c55e8f3755a115410f2eb5459c'
    v4=c.s.f.prior.control.V4
    for rel,sha in result['evidence'].items():assert h.file_hash(prior.ROOT/rel)==sha,rel
    for rel,sha in result['implementation'].items():assert h.file_hash(v4/rel)==sha,rel
    retained=h.read(c.ROOT/'RETENTION.json')
    for rel,sha in retained['keep'].items():assert h.file_hash(c.ROOT/rel)==sha,rel
    scratch=h.read(prior.previous.old.ROOT/'SCRATCH_INPUTS.json')
    for rel,sha in scratch['files'].items():assert h.file_hash(prior.previous.old.ROOT/'scratch'/rel)==sha,rel
    assert {str(x.relative_to(prior.previous.old.ROOT/'scratch')) for x in (prior.previous.old.ROOT/'scratch').rglob('*') if x.is_file()}==set(scratch['files'])
    audit=h.read(c.ROOT/'UNRELATED_AUDIT.json')
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=[s for s in status if 'capsule_sa_outer_map' not in s and '/sa_outer_map/' not in s]
    assert unrelated==audit['status']
    exclusions=[':(exclude)research/sdf_cfd_mesher_v4/'+p for p in ('tools/capsule_sa_outer_map*.py','tools/sa_outer_map/**','results/capsule_sa_outer_map/**','tests/test_capsule_sa_outer_map.py')]
    assert hashlib.sha256(subprocess.check_output(['git','diff','--binary','--','.',*exclusions],stderr=subprocess.DEVNULL)).hexdigest()==audit['tracked_diff_sha256']
    return dict(retained_files=len(retained['keep']),scratch_files=len(scratch['files']),unrelated_count=len(unrelated),scientific_continuation_iterations=0,new_retained_times=0,mesh_changes=0,prism_changes=0,retained_field_mutations=0)


if __name__=='__main__':
    import sys
    command=sys.argv[1]
    if command=='base':base()
    elif command=='verify':print(verify())
    else:analyze_direction(command)
