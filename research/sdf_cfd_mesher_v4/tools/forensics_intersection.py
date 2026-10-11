"""Smallest known exact pair, complete entity tracing, independent membership."""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'));sys.dont_write_bytecode=True
import numpy as np
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.forecast.forensics import interpret_certificate,GeometryUse
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import CertificateSession
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
OUT=ROOT/'results/nominal_forecast_forensics'
def load_coordinates(front):
    saved=[]
    for first in range(1,21,5):
        with np.load(ROOT/f'results/task1/forecast_layers_{first:02}_{first+4:02}.npz') as z:saved.append(z['coordinates'])
    return np.concatenate((front.coordinates[None],*saved))
def make_side(x,top,edge,layer):
    a,b=map(int,top.edges[edge]);points=x[layer-1:layer+1,[a,b]].reshape(4,3)
    ids=((layer-1,a),(layer-1,b),(layer,a),(layer,b))
    return kernel.Patch.binary(points,ids),points,ids
def reproduce(front,top,x,sdf,schedule):
    original=json.loads((ROOT/'results/task2/CONTINUOUS.json').read_text())[-1]['failure'];trace=[];patches=[]
    for layer,edge in (original['side_a'],original['side_b']):
        patch,points,ids=make_side(x,top,edge,layer);patches.append(patch);query=sdf.evaluate(points);v=top.edges[edge]
        previous=x[layer-1,v];displacement=x[layer,v]-previous;length=np.linalg.norm(displacement,axis=1)
        trace.append(dict(source_edge_id=edge,layer=layer,vertex_indices=v.tolist(),stable_vertex_ids=[front.vertex_ids[i] for i in v],source_ancestry=[front.source_ancestry[i] for i in v],
                          source_wall_coordinates=front.coordinates[v],columns=top.edge_triangles.row(edge),column_triangles=top.triangles[top.edge_triangles.row(edge)],
                          side_point_order='lower_a, lower_b, upper_a, upper_b',topology_labels=ids,points=points,
                          targets=np.repeat([0. if layer==1 else schedule.targets[layer-2],schedule.targets[layer-1]],2),
                          phi=query.phi,gradients=query.gradient,normals=query.normal,gradient_norm=query.gradient_norm,boundary_normal_selection=query.boundary_normal_selection,branch_tie=query.branch_tie,
                          displacement=displacement,step_length=length,direction=displacement/length[:,None],wall_normals=sdf.evaluate(front.coordinates[v]).normal))
    result=CertificateSession().pair(*patches)
    assert result=={k:v for k,v in original.items() if k not in ('side_a','side_b')},'IntersectionReproducerMismatch'
    from fractions import Fraction
    witness=tuple(Fraction(v) for v in result['point']);rg=kernel.module('rank_geometry')
    member=[rg.membership(witness,p) for p in patches];owned=rg.owned(witness,rg.owned_set(*patches))
    assert all(member) and not owned
    checks=[]
    for p in patches:
        if p.rank==2:
            g=p.planar
            checks.append(dict(rank=2,method='Exact plane equality and convex image halfspaces',plane_residual=str(rg.dot(g['normal'],witness)-g['offset']),halfspace_signs=[rg.sign(rg.dot(n,rg.sub(witness,a))) for n,a in g['halfspaces']]))
        else:
            C,*_=p.terms;u,v,w=[rg.dot(r,rg.sub(witness,C)) for r in p.inverse]
            checks.append(dict(rank=p.rank,method='Exact inverse bilinear coordinates',u=str(u),v=str(v),uv_residual=str(w-u*v)))
    return plain(dict(patch_id='patch_002',classification='GenuineNominalForecastHazard',surfaces=trace,certificate=result,
                      independent_exact_membership=member,allowed_owned_contact=owned,membership_details=checks,forecast_meaning=interpret_certificate(result,GeometryUse.FORECAST)))
if __name__=='__main__':
    started=time.perf_counter();front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));x=load_coordinates(front);sdf=NativeSdf();schedule=load_schedule()
    r=reproduce(front,top,x,sdf,schedule);(OUT/'INTERSECTION_REPRODUCER.json').write_text(json.dumps(r,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(classification=r['classification'],membership=r['independent_exact_membership'],seconds=time.perf_counter()-started,scientific_hash=scientific_hash(r))))
