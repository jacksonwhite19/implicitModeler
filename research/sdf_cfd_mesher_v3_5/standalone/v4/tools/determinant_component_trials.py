"""Supervise one local proposal at a time, with compact progress and evidence."""
import sys,subprocess,time,json,shutil,os
from pathlib import Path
import numpy as np
from constrained_cavity import *
from determinant_component_audit import OUT,V4
from run_constrained_cavity import run as mesh

def incenter(q):
    areas=np.array([np.linalg.norm(np.cross(q[j]-q[i],q[k]-q[i])) for i,j,k in ((1,2,3),(0,2,3),(0,1,3),(0,1,2))])
    return np.einsum('i,ij->j',areas,q)/areas.sum()

def point_set(d,strategy):
    q,m=quality(d,d['x'],d['t']);bad=np.flatnonzero(m['determinant']<.0011)
    bad=sorted(bad,key=lambda i:(m['determinant'][i],scientific_key(d['x'][d['t'][i]])))
    if strategy=='worst_incenter':points=[incenter(d['x'][d['t'][bad[0]]])]
    elif strategy=='bad_incenters':points=[incenter(d['x'][d['t'][i]]) for i in bad]
    elif strategy=='mixed_support':
        interior=np.setdiff1d(np.unique(d['t']),np.unique(d['f']));points=list(d['x'][interior])
        points += [incenter(d['x'][d['t'][i]]) for i in bad]
        points += [d['x'][d['t'][i]].mean(0) for i in np.argsort(-m['volume'],kind='stable')[:3]]
    else:raise ValueError('UnknownSteinerStrategy')
    unique={p.tobytes():p for p in points};points=np.array([unique[k] for k in sorted(unique)])
    if not all(interior_point_valid(p,d['x'],d['t']) for p in points):raise ValueError('InvalidDerivedSteinerPoint')
    return points

def topology_trial(label,depth,strategy):
    original=OUT/'forensics'/f'{label}_r{depth}.npz';d=dict(np.load(original));d['steiner_points']=point_set(d,strategy)
    name=f'{label}_r{depth}_{strategy}';inp=OUT/(name+'.npz');np.savez_compressed(inp,**d);dest=OUT/name
    mesh(inp,dest,'netgen_safe','provided');r=json.loads((dest/'RESULT.json').read_text());last=r['stages'][-1]
    print(json.dumps(dict(trial=name,points=len(d['steiner_points']),result=last)),flush=True)
    if last['invariants']['passed']:
        ca=dest/(last['stage']+'_CANDIDATE.npz')
        if not last['quality']['passed']:solve(inp,ca,OUT/(name+'_solve'))
    return r

def solve(inp,ca,out,mode='margin'):
    free=shutil.disk_usage(out.parent).free;need=inp.stat().st_size+ca.stat().st_size
    if free<need:raise ValueError('InsufficientMeasuredLocalSpace')
    cmd=[sys.executable,'-B',str(V4/'tools/determinant_local_solve.py'),str(inp),str(ca),str(out),mode];tick=time.perf_counter()
    with out.with_suffix('.log').open('w') as f:
        p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
        while p.poll() is None:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print(json.dumps(dict(stage=out.name,elapsed=time.perf_counter()-tick,free=shutil.disk_usage(out.parent).free)),flush=True)
                if time.perf_counter()-tick>=120:p.kill();p.wait();break
    r=dict(seconds=time.perf_counter()-tick,exit_code=p.returncode,limit=120,free_before=free,free_after=shutil.disk_usage(out.parent).free)
    save(out.with_suffix('.process.json'),r)
    if (out/'RESULT.json').exists():
        a=json.loads((out/'RESULT.json').read_text());print(json.dumps(dict(trial=out.name,process=r,accepted=a['accepted_proposal'],failures=a['after']['failures'],minimum=a['after']['minimum'],moving=len(a['moving_vertices']))),flush=True)
    else:print(json.dumps(dict(trial=out.name,process=r)),flush=True)
    return r

def existing(label,depth):
    inp=OUT/'forensics'/f'{label}_r{depth}.npz';d=np.load(inp);ca=OUT/f'{label}_r{depth}_existing.npz';np.savez_compressed(ca,x=d['x'],t=d['t']);return solve(inp,ca,OUT/f'{label}_r{depth}_existing_solve')

if __name__=='__main__':
    if sys.argv[1]=='existing':existing(sys.argv[2],int(sys.argv[3]))
    elif sys.argv[1]=='topology':topology_trial(sys.argv[2],int(sys.argv[3]),sys.argv[4])
    else:solve(*map(Path,sys.argv[1:4]))
