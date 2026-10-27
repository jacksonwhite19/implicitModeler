"""Bounded, compact deterministic one-point and multi-point cavity trials."""
import sys,time,json
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial import Delaunay,QhullError
from constrained_cavity import *

def run(inputs,out):
    out.mkdir(exist_ok=False);results=[];start=time.perf_counter()
    for file in sorted(inputs.glob('star_*.npz')):
        d=dict(np.load(file));x,t,f=d['x'],d['t'],d['f'];p,kr=kernel_point(x,f)
        row=dict(input=file.name,method='one_point_kernel_cone',kernel=kr)
        tick=time.perf_counter()
        if kr.get('strict_kernel'):
            xx=np.vstack((x,p));tt=np.c_[np.full(len(f),len(x)),f];iv=invariants(xx,t,tt);row['invariants']=iv;row['point']=p;row['inside_old_cell_union']=strict_inside(p,x,t)
            if iv['passed']:
                q,m=quality(d,xx,tt);row['quality']=q
                np.savez_compressed(out/(file.stem+'_cone.npz'),x=xx,t=tt,point=p)
        row['seconds']=time.perf_counter()-tick;results.append(row)
        print(json.dumps(dict(input=file.name,kernel=kr,failures=row.get('quality',{}).get('failures')),default=lambda v:v.tolist()),flush=True)
    save(out/'TRIALS.json',results);save(out/'PERFORMANCE.json',dict(seconds=time.perf_counter()-start))

if __name__=='__main__':run(*map(Path,sys.argv[1:3]))
