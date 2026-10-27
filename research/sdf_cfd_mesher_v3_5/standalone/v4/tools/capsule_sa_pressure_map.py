"""Read-only Foundation pressure-map audit and deterministic array analysis."""
import hashlib
import re
import subprocess
from functools import lru_cache
from pathlib import Path

import numpy as np
import capsule_sa_tangent as previous
import capsule_sa_operator_analysis as metrics
import capsule_sa_operator_directions as directions

h=previous.h
c=previous.c
ROOT=previous.ROOT.parent/'capsule_sa_pressure_map'
HEAD='bd30a890b7bee5b15790044a69a81f2d60c15f1e'
EPS=(1e-3,1e-4,1e-5,1e-6)


@lru_cache(None)
def addressing(name):
    return [c.sa.binary_labels(previous.old.ROOT/'scratch'/name/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]


def load(name,key,dim=1,canonical=True):
    arrays=[np.fromfile(ROOT/'outputs'/name/str(i)/(key+'.bin'),dtype='<f8').reshape(-1,dim) for i in range(6)]
    if not canonical:return np.concatenate(arrays)
    ids=addressing(name)
    return metrics.canonical(arrays,ids)


def stats(x,V):
    return dict(RMS=directions.rms(x,V),Linf=float(np.abs(x).max()),min=float(x.min()),max=float(x.max()))


def fraction(u,n,sign):
    return .5+sign*.5*np.sum(u*n,axis=-1,keepdims=True)/np.linalg.norm(u,axis=-1,keepdims=True)


def fraction_derivative(u,du,n,sign):
    mag=np.linalg.norm(u,axis=-1,keepdims=True)
    return sign*.5*(np.sum(du*n,axis=-1,keepdims=True)/mag-np.sum(u*n,axis=-1,keepdims=True)*np.sum(u*du,axis=-1,keepdims=True)/mag**3)


def boundary_audit(name,prefix):
    """Differentiate the installed formulas; do not silently impose fixed-point BCs."""
    b=lambda key,dim=1:load(name,'base0_'+key,dim,False)
    d=lambda key,dim=1:load(name,prefix+key,dim,False)
    n=b('normal_farfield',3); u=b('Ub_farfield',3);ui=b('Ui_farfield',3); pi=b('pi_farfield')
    du=d('Ub_farfield',3);di=d('Ui_farfield',3);dpi=d('pi_farfield');dp=d('pb_farfield')
    pv=b('pressureVelocity_farfield',3);dpv=d('pressureVelocity_farfield',3)
    fp=fraction(pv,n,1);dfp=fraction_derivative(pv,dpv,n,1)
    stage=dp-((1-fp)*dpi-dfp*pi)
    finalfp=fraction(u,n,1);finaldf=fraction_derivative(u,du,n,1)
    coupled=dp-((1-finalfp)*dpi-finaldf*pi)
    w=.5*(ui+u);dw=.5*(di+du);fu=fraction(w,n,-1);dfu=fraction_derivative(w,dw,n,-1)
    ugap=du-((1-fu)*di+dfu*(np.array([0.,0.,33.528])-ui))
    simple=lambda x:dict(RMS=float(np.sqrt(np.mean(x*x))),Linf=float(abs(x).max()))
    return dict(pressure_stage_formula_defect=simple(stage),pressure_simultaneous_formula_defect=simple(coupled),velocity_simultaneous_formula_defect=simple(ugap),
        pressure_fraction_derivative_defect=simple(d('pFraction')-dfp),
        pressure_stage_velocity_vs_final_derivative=simple(dpv-du))


def faces(name,key):
    parts=[];indices=[]
    for rank in range(6):
        ids=c.sa.binary_labels(previous.old.ROOT/'scratch'/name/f'processor{rank}/constant/polyMesh/faceProcAddressing')
        values=np.fromfile(ROOT/'outputs'/name/str(rank)/(key+'.bin'),dtype='<f8')
        assert len(ids)==len(values)
        parts.append(values*np.sign(ids));indices.append(abs(ids)-1)
    out=np.full(max(x.max() for x in indices)+1,np.nan);duplicate=0.
    for values,ids in zip(parts,indices):
        mask=np.isfinite(out[ids])
        if mask.any():duplicate=max(duplicate,float(abs(out[ids][mask]-values[mask]).max()))
        out[ids]=values
    assert np.isfinite(out).all()
    return out,duplicate


def face_audit():
    result={}
    for name in c.NAMES:
        V=load(name,'V')[:,0]
        mesh=previous.old.ROOT/'scratch'/name/'constant/polyMesh'
        def labels(path):
            text=path.read_text();match=re.search(r'\n(\d+)\s*\n\(',text)
            values=np.fromstring(text[match.end():].split(')')[0],dtype=np.int64,sep=' ')
            assert len(values)==int(match[1]);return values
        owner=labels(mesh/'owner');nei=labels(mesh/'neighbour')
        record={}
        for key in ['base0_phi','base0_phiChange']+[f'{s}_d{i}_phi' for s in ('native','unlimited') for i in range(4)]:
            x,dup=faces(name,key)
            div=(np.bincount(owner,weights=x,minlength=len(V))-np.bincount(nei,weights=x[:len(nei)],minlength=len(V)))/V
            record[key]=dict(sha256=h.array_hash(x),faces=len(x),processor_duplicate_Linf=dup,
                unweighted_face_RMS=float(np.sqrt(np.mean(x*x))),Linf=float(abs(x).max()),boundary_net=float(x[len(nei):].sum()),divergence=stats(div[:,None],V))
        for key in ('pDiag','pSource'):
            x=load(name,'base0_'+key);record[key]={**stats(x,V),'sha256':h.array_hash(x)}
        # Off-diagonal and patch matrices retain rank/face ordering; no false claim of repartition invariance.
        matrix_files={str(x.relative_to(ROOT)):h.file_hash(x) for x in (ROOT/'outputs'/name).glob('*/base0_p*.bin') if any(k in x.name for k in ('Diag','Upper','Source','Coeff'))}
        record['partition_ordered_matrix_identity']=h.seal(matrix_files)['identity']
        result[name]=record
    h.durable_json(ROOT/'FACE_MATRIX_ANALYSIS.json',h.seal(result))
    return result


def analyze():
    result={}
    for name in c.NAMES:
        V=load(name,'V')[:,0]; C=load(name,'C',3)
        base={}
        for key,dim in [('A',1),('H',3),('H1',1),('rAU',1),('rAtU',1),('HbyA',3),('p',1),('U',3),('pChange',1),('UChange',3),('div',1)]:
            x=load(name,'base0_'+key,dim)
            base[key]={**stats(x,V), 'sha256':h.array_hash(x)}
        base['regions']={r:{k:stats(load(name,'base0_'+k,dim)[mask],V[mask]) for k,dim in [('UChange',3),('pChange',1),('div',1)]} for r,mask in metrics.regions(name,C).items()}
        replay={}
        for f in (ROOT/'outputs'/name).glob('*/base0_*.bin'):
            other=f.with_name(f.name.replace('base0_','base1_'))
            replay[str(f.relative_to(ROOT))]=h.file_hash(f)==h.file_hash(other)
        assert all(replay.values())
        item=dict(base=base,base_replay_files=len(replay),base_replay_exact=True,seeds={})
        for seed in ('native','unlimited'):
            if not (ROOT/'outputs'/name/'0'/(seed+'_d3_U.bin')).exists():continue
            initial=load(name,seed+'_seedU',3)
            sequence=[]; old={}
            for ei,e in enumerate(EPS):
                prefix=seed+'_d'+str(ei)+'_'; rec=dict(epsilon=e,fields={})
                for key,dim in [('A',1),('H',3),('H1',1),('rAU',1),('rAtU',1),('HbyA',3),('p',1),('U',3),('pChange',1),('UChange',3),('div',1)]:
                    x=load(name,prefix+key,dim)
                    q={**stats(x,V),'sha256':h.array_hash(x)}
                    if key in old:q['adjacent_relative_array_difference']=directions.rms(x-old[key],V)/max(directions.rms(x,V),np.finfo(float).tiny)
                    old[key]=x;rec['fields'][key]=q
                final=old['U']
                rec['velocity_seed_RMS']=directions.rms(initial,V)
                rec['velocity_correction_ratio']=directions.rms(final-initial,V)/directions.rms(initial,V)
                rec['velocity_seed_cosine']=directions.inner(final,initial,V)/(directions.rms(final,V)*directions.rms(initial,V))
                rec['boundary']={}
                for patch in ('wall','farfield'):
                    b={}
                    for key,dim in [('Ub',3),('pb',1),('Nb',1),('nutb',1),('phib',1)]:
                        x=load(name,prefix+key+'_'+patch,dim,False)
                        b[key]=dict(RMS=float(np.sqrt(np.mean(x*x))),Linf=float(abs(x).max()),sum=float(x.sum()))
                    rec['boundary'][patch]=b
                if (ROOT/'outputs'/name/'0'/(prefix+'Ui_farfield.bin')).exists():rec['mixed_boundary_audit']=boundary_audit(name,prefix)
                sequence.append(rec)
            replay=[]
            for f in (ROOT/'outputs'/name).glob('*/'+seed+'_d3_*.bin'):
                replay.append(h.file_hash(f)==h.file_hash(f.with_name(f.name.replace('_d3_','_d4_'))))
            item['seeds'][seed]=dict(epsilon_study=sequence,replay_exact=all(replay),replay_files=len(replay))
        result[name]=item
    h.durable_json(ROOT/'MAP_ANALYSIS.json',h.seal(result))
    for name,item in result.items():
        print(name,'base correction',item['base']['UChange'],'continuity',item['base']['div'])
        for seed,q in item['seeds'].items():
            print(seed,'replay',q['replay_exact'])
            for r in q['epsilon_study']:
                print(r['epsilon'],r['velocity_correction_ratio'],r['velocity_seed_cosine'],r['fields']['div'],r['fields']['U'].get('adjacent_relative_array_difference'))
    return result


def verify():
    prior=h.read(previous.ROOT/'RESULT.json');h.verify_record(prior)
    assert prior['identity']=='54a64b588292884770a3f680ad53bc7fbfb6b34b94c118cf56a64e52e12f62bf'
    for rel,digest in prior['evidence'].items():assert h.file_hash(previous.ROOT/rel)==digest,rel
    v4=c.s.f.prior.control.V4
    for rel,digest in prior['implementation'].items():assert h.file_hash(v4/rel)==digest,rel
    audit=h.read(c.ROOT/'UNRELATED_AUDIT.json')
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=[s for s in status if 'capsule_sa_pressure_map' not in s and '/sa_pressure_map/' not in s]
    assert unrelated==audit['status']
    exclusions=[':(exclude)research/sdf_cfd_mesher_v4/'+p for p in (
        'tools/capsule_sa_pressure_map*.py','tools/sa_pressure_map/**',
        'results/capsule_sa_pressure_map/**','tests/test_capsule_sa_pressure_map.py')]
    assert hashlib.sha256(subprocess.check_output(['git','diff','--binary','--','.',*exclusions],stderr=subprocess.DEVNULL)).hexdigest()==audit['tracked_diff_sha256']
    retained=h.read(c.ROOT/'RETENTION.json');h.verify_record(retained)
    for rel,digest in retained['keep'].items():assert h.file_hash(c.ROOT/rel)==digest,rel
    scratch=h.read(previous.old.ROOT/'SCRATCH_INPUTS.json');h.verify_record(scratch)
    for rel,digest in scratch['files'].items():assert h.file_hash(previous.old.ROOT/'scratch'/rel)==digest,rel
    assert {str(x.relative_to(previous.old.ROOT/'scratch')) for x in (previous.old.ROOT/'scratch').rglob('*') if x.is_file()}==set(scratch['files'])
    return dict(retained_files=len(retained['keep']),scratch_files=len(scratch['files']),unrelated_count=len(unrelated),new_cfd_iterations=0,mesh_changes=0,prism_changes=0,retained_state_mutations=0)


if __name__=='__main__':
    import sys
    {'analyze':analyze,'faces':face_audit,'verify':lambda:print(verify())}[sys.argv[1]]()
