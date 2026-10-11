"""Seal complete-map health and per-block evidence without scientific continuation."""
import json,re,subprocess,shutil
from pathlib import Path
import numpy as np
import capsule_sa_outer_map as m
h=m.h;R=m.ROOT


def patch(name,direction,key,dim=1):
    return np.concatenate([np.fromfile(R/'outputs'/name/direction/str(i)/(key+'.bin')).reshape(-1,dim) for i in range(6)])


def health():
    result={}
    for p in sorted(R.glob('SA-GW1-*.log')):
        if any(t in p.stem for t in ('rejected','initial')):continue
        s=p.read_text();solves=re.findall(r'Solving for (\w+), Initial residual = ([\deE.+-]+), Final residual = ([\deE.+-]+), No Iterations (\d+)',s)
        if 'Exit status: 0' not in s:continue
        begin=s.count('OUTER_MAP_BEGIN');end=s.count('OUTER_MAP_END')
        assert begin==end,(p,begin,end)
        assert not re.search(r'FOAM FATAL|bounding nuTilda|\bnan\b|Floating point exception(?! trapping)',s,re.I),p
        assert all(int(row[3])<5000 for row in solves)
        if begin:assert len(solves)==5*begin,(p,len(solves),begin)
        result[p.name]=dict(map_calls=begin or 1,linear_solves=len(solves),
            iterations={k:sum(int(x[3]) for x in solves if x[0]==k) for k in ('Ux','Uy','Uz','p','nuTilda')},
            branches=[int(x) for x in re.findall(r'FARFIELD_BRANCH_CROSSINGS (\d+)',s)],
            boundedness_events=0,maxIter_events=0,exit_status=0,
            time_statistics=[line.strip() for line in s.splitlines() if any(t in line for t in ('User time','System time','Elapsed (wall clock)','Maximum resident set size'))])
    h.durable_json(R/'HEALTH.json',h.seal(result));return result


def zero():
    result={}
    for name in m.c.NAMES:
        result[name]={}
        for k in m.FIELDS:
            d=m.field(name,'zero','d0_',k);e=m.field(name,'zero','even0_',k);w=m.weights(name,'zero',k)
            assert np.count_nonzero(d)==0
            result[name][k]=dict(derivative=m.summary(d,w),even=m.summary(e,w))
    h.durable_json(R/'ZERO.json',h.seal(result));return result


def boundary(direction):
    result={}
    count=3 if direction.endswith('_fine') else 4
    for name in m.c.NAMES:
        if not (R/'outputs'/name/direction/'5'/f'd{count}_U.bin').exists():continue
        V=m.load(name,direction,'V')[:,0]
        rows={}
        for tag in ['base0']+[f'd{i}' for i in range(count)]:
            div=m.load(name,direction,tag+'_div');far=patch(name,direction,tag+'_phib_farfield')[:,0];wall=patch(name,direction,tag+'_phib_wall')[:,0]
            S=float(far.sum());M=float(abs(far).sum());inflow=(M-S)/2;outflow=(M+S)/2
            rows[tag]=dict(divergence=m.summary(div,V),signed_volume_integrated_divergence=float(np.sum(V*div[:,0])),
                farfield_net=S,farfield_abs_sum=M,imbalance=abs(S)/max(inflow,outflow),wall_flux_Linf=float(abs(wall).max()),
                wall_U_Linf=float(abs(patch(name,direction,tag+'_Ub_wall',3)).max()),
                wall_N_Linf=float(abs(patch(name,direction,tag+'_Nb_wall')).max()),
                flux_formula_error_Linf=float(abs(m.load(name,direction,tag+'_fluxFormulaError',face=True)).max()))
        # Patch-adjacent region contributions overlap at corners; explicitly not a partition.
        bits=m.prior.metrics.load(name,1000,'patchBits')[:,0].astype(int)
        w=V;g=m.field(name,direction,f'g{count-1}_','U');y=m.field(name,direction,f'd{count-1}_','U');e=y-g
        total=np.sum(w*np.sum(e*e,axis=1))
        rows['patch_adjacent_identity_error']={p:dict(RMS=m.norm(e[(bits&(1<<j))!=0],w[(bits&(1<<j))!=0]),
            squared_share=float(np.sum(w[(bits&(1<<j))!=0]*np.sum(e[(bits&(1<<j))!=0]**2,axis=1))/total))
            for j,p in enumerate(('xmin','xmax','ymin','ymax','zmin','zmax'))}
        result[name]=rows
    h.durable_json(R/(direction.upper()+'_BOUNDARY.json'),h.seal(result));return result

if __name__=='__main__':
    import sys
    if sys.argv[1]=='health':health()
    elif sys.argv[1]=='zero':zero()
    else:boundary(sys.argv[1])
