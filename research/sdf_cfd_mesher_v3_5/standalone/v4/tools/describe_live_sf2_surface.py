"""Read-only description of retained fresh/SF2 arrays; no qualification claims."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
import psutil
V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from audit_adaptive_surface import dist
from core_handoff_audit import surface
from historical_capsule_surface import canonical,array_hash,sha
from sf2_surface_policy import ROOT as SF2

def describe(x,f,refs,names,m=None,curves=None):
    s=surface(x,f)
    def face_stats(data):
        return {k:dist(data[k]) for k in ('minimum_angle','maximum_angle','edge_ratio','radius_ratio','area')}
    out=dict(vertices=len(x),triangles=len(f),edges=len(s['edges']),edge_m=dist(s['edge_length']),triangle=face_stats(s),regions={})
    for ref in np.unique(refs):
        ids=np.flatnonzero(refs==ref);sub=surface(x,f[ids])
        out['regions'][str(ref)]=dict(name=names.get(int(ref),str(ref)),faces=len(ids),edge_m=dist(sub['edge_length']),triangle=face_stats(sub))
    extrema={}
    for key,choose in [('minimum_angle',np.argmin),('maximum_angle',np.argmax),('edge_ratio',np.argmax)]:
        i=int(choose(s[key]));verts=set(map(int,f[i]))
        incident=[] if curves is None else [list(map(int,row)) for row in curves if int(row[0]) in verts or int(row[1]) in verts]
        extrema[key]=dict(face=i,value=float(s[key][i]),sheet=int(refs[i]),name=names.get(int(refs[i])),
            vertices=f[i].tolist(),coordinates=x[f[i]].tolist(),incident_curve_edges=incident)
    out['extrema']=extrema
    if m is not None:
        e=s['edges'];d=x[e[:,1]]-x[e[:,0]];avg=(m[e[:,0]]+m[e[:,1]])/2
        lengths=np.sqrt(np.einsum('ni,nij,nj->n',d,avg,d))
        out['metric_edge_length']=dist(lengths)
        if curves is not None:
            ce=curves[:,:2];d=x[ce[:,1]]-x[ce[:,0]];avg=(m[ce[:,0]]+m[ce[:,1]])/2
            out['curve_edge_m']=dist(np.linalg.norm(d,axis=1))
            out['curve_metric_length']=dist(np.sqrt(np.einsum('ni,nij,nj->n',d,avg,d)))
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();run=args.run
    start=time.perf_counter();pre=json.loads((run/'PREFLIGHT.json').read_text(encoding='utf-8'))
    names={int(k):v for k,v in pre['admission']['references']['sheet_names'].items()}
    with np.load(run/'fresh/SURFACE.npz') as z,np.load(run/'fresh/REFERENCES.npz') as r:
        x,f,m=z['coordinates'],z['triangles'],z['metric'];refs=r['face_refs'];curves=r['curve_edges']
        current=describe(x,f,refs,names,m,curves)
        current['stored_owning_residual_m']=dist(abs(z['residual']))
        current['array_hashes']={key:array_hash(z[key]) for key in z.files}
        with np.load(SF2/'SF2/A/wing/surface/A/QUALIFICATION_ARRAYS.npz') as old:
            past=describe(old['wall'],old['faces'],old['refs'],names,curves=old['feature_edges'])
            old_x,old_f,order,_,face_order=canonical(old['wall'],old['faces'],return_permutations=True)
            same_shape=x.shape==old_x.shape and f.shape==old_f.shape
            comparison=dict(shape_matches=same_shape,coordinates_exact=np.array_equal(x,old_x),
                triangles_exact=np.array_equal(f,old_f),sheet_refs_exact=np.array_equal(refs,old['refs'][face_order]),
                maximum_corresponding_coordinate_difference=float(abs(x-old_x).max()) if x.shape==old_x.shape else None)
    result=dict(scope='Read-only descriptive array comparison. Qualification outcomes remain in original run records.',
        current=current,historical_SF2=past,canonical_comparison=comparison,
        polygon_diagnostic=dict(minimum_angle_deg=.08176285584319411,maximum_angle_deg=179.76671024787927,maximum_edge_ratio=141.73271354257162,
            scope='Different stored geometry representation; not an isolated metric-effect experiment.'),
        source_files={str(p):sha(p) for p in (run/'fresh/SURFACE.npz',run/'fresh/REFERENCES.npz',SF2/'SF2/A/wing/surface/A/QUALIFICATION_ARRAYS.npz')},
        seconds=time.perf_counter()-start,peak_working_set_bytes=psutil.Process().memory_info().peak_wset)
    print(json.dumps(result,indent=2,allow_nan=False))

if __name__=='__main__':main()
