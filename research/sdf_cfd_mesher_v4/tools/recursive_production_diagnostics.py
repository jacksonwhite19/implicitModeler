"""Read-only diagnostics of the realized L6 delta; no forecast or meshing."""
from qualify_recursive_layers7to8 import *
from sdf_cfd_mesher_v4.common.authority import array_hash
from geometry_family_authority import Source
from fractions import Fraction
import numpy as np
from production_first_layer_diagnostics import stats,angles,triangle
from sdf_cfd_mesher_v4.common.authority import array_hash

def run(out, layer):
    global OUT
    OUT = out
    tick=measured()
    with CommittedState(parent(layer)[0]) as base:
        a,spec=inputs(base)
        with np.load(OUT/'GENERATED_NEXT.npz') as z: high=z['coordinates'];tri=z['topology']
        low=a.front.coordinates; d=high-low; normal=Source(spec).evaluate(low).normal
        h=np.einsum('ij,ij->i',d,normal); inc=plan_transition(base,a,layer)['increment']
        height={k:stats(v) for k,v in dict(euclidean=np.linalg.norm(d,axis=1),normal=h,sdf_normal=h,
            tangential=np.linalg.norm(d-h[:,None]*normal,axis=1),requested_minus_achieved=inc-h,ratio=h/inc,
            angle_sdf=angles(d,normal),angle_prior=angles(d,low-a.committed_prefix[-2])).items()}
        edges=a.topology.edges; fronts={}; lengths=[];areas=[]
        for i,x in enumerate([*a.committed_prefix,high]):
            t=triangle(x,tri);e=np.linalg.norm(x[edges[:,0]]-x[edges[:,1]],axis=1)
            lengths.append(e);areas.append(t['area']);norm=Source(spec).evaluate(x).normal
            cross=np.cross(x[tri[:,1]]-x[tri[:,0]],x[tri[:,2]]-x[tri[:,0]])
            fronts[str(i)]=dict(edges=stats(e),triangles={k:stats(v) for k,v in t.items()},
                normal_variation=stats(angles(norm[edges[:,0]],norm[edges[:,1]])),zero_area=int((t['area']<=0).sum()),
                reversal=int((np.einsum('ij,ij->i',cross,norm[tri].mean(1))<=0).sum()),nonfinite=int((~np.isfinite(x)).sum()))
        matched={str(i):dict(edge_ratio=stats(lengths[-1]/lengths[i]),area_ratio=stats(areas[-1]/areas[i])) for i in (0,layer-1)}
    q=read(OUT/'NEW_QUALITY.json'); quality={}
    for key,rows,gate,upper in [('volume',q['cells'],0,False),('determinant',q['cells'],.0011,False),
        ('aspect',q['cells'],1000,True),('weight',q['faces'],.055,False),('skewness',q['faces'],3.63636,True),
        ('symmetric_volume_ratio',q['faces'],.01,False)]:
        v=np.array([r[key] for r in rows]);i=int(v.argmax() if upper else v.argmin())
        bad=v>gate if upper else (v<=gate if key=='volume' else v<gate)
        quality[key]=dict(distribution=stats(v),failures=int((bad|~np.isfinite(v)).sum()),margin=float(gate-v[i] if upper else v[i]-gate),worst=rows[i])
    v=np.array([float(Fraction(r['jacobian'])) for r in q['exact']]);i=int(v.argmin())
    quality['exact_jacobian']=dict(distribution=stats(v),failures=sum(r['jacobian_sign']<=0 for r in q['exact']),margin=float(v[i]),worst=q['exact'][i])
    durable_json(OUT/'DIAGNOSTICS.json',dict(height=height,nonpositive_height=int((h<=0).sum()),fronts=fronts,matched=matched,
        quality=quality,coordinates=array_hash(high),metrics=metrics(*tick),scope='Realized layer and read-only front diagnostics; no combined historical quality traversal'))
