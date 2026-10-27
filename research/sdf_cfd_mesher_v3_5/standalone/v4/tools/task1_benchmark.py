"""Measured SF2 direct forecast. No prism, optimizer, certificate or solver calls."""
from pathlib import Path
import sys,os,time,json,csv,dataclasses,gc
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,file_hash
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.forecast.engine import ForecastCache,cache_key
from sdf_cfd_mesher_v4.runtime.regions import extract_regions
from sdf_cfd_mesher_v4.runtime.scaling import v3_scaling_case
OUT=ROOT/'results/task1';OUT.mkdir(parents=True,exist_ok=True)
def write(name,value):(OUT/name).write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')
def stats(x):
    x=np.asarray(x);finite=x[np.isfinite(x)]
    return dict(count=x.size,finite_count=finite.size,**(dict(zip(['min','p01','p10','median','p90','p99','max'],map(float,np.quantile(finite,[0,.01,.1,.5,.9,.99,1])))) if finite.size else {}))

def main():
    started=time.perf_counter();front=load_sf2_wall();t=time.perf_counter();top=build_topology(front.triangles,len(front.coordinates));topology_seconds=time.perf_counter()-t
    sdf=NativeSdf();sdf.clear_memo();schedule=load_schedule();wall=sdf.evaluate(front.coordinates)
    sign_points=np.array([[.5,0,0],[1.01,0,0],[.5,.26,0]])
    sign=sdf.evaluate(sign_points)
    assert sign.phi[0]<0 and sign.phi[1]>0 and sign.phi[2]>0
    assert np.array_equal(sign.gradient[1:3],[[1.,0,0],[0,1.,0]])
    assert np.all(abs(wall.phi)<=wall.source_error_bound),'WallExceedsNativeDistanceErrorBound'
    authority=dict(vertices=len(front.coordinates),edges=len(top.edges),triangles=len(top.triangles),coordinate_hash=front.coordinate_hash,connectivity_hash=front.topology_hash,edge_hash=array_hash(top.edges),vertex_id_hash=identity(front.vertex_ids),ancestry_hash=identity(front.source_ancestry),sdf_source_identity=sdf.source_identity,target_distance=0.,geometry_authority=front.geometry_authority,surface_authority=front.surface_authority)
    write('INITIAL_FRONT_AUTHORITY.json',authority)
    write('SCHEDULE.json',dict(source_sha256=schedule.source_sha256,schedule_sha256=schedule.sha256,h1=schedule.first_height,growth=schedule.growth,arithmetic='Preserved V3 binary64 increments; numpy binary64 cumsum in layer order',layers=[dict(layer=i+1,increment_m=float(h),target_m=float(d)) for i,(h,d) in enumerate(zip(schedule.increments,schedule.targets))],total_thickness_m=float(schedule.targets[-1])))
    print('START cold full forecast',flush=True);sdf.clear_memo();cache=ForecastCache();a=cache.forecast(front,top,schedule,sdf);cold=cache.last_stats.copy();hash1=a.scientific_hash();print('COLD',cold,'hash',hash1,flush=True)
    warm=cache.forecast(front,top,schedule,sdf);warm_stats=cache.last_stats.copy();assert warm.scientific_hash()==hash1
    changed=np.array([0,len(front.coordinates)//2],dtype=np.int64);invalid=cache.invalidate(top,changed);cache.forecast(front,top,schedule,sdf);local_stats=cache.last_stats.copy();assert a.scientific_hash()==hash1
    mask=np.zeros(len(front.coordinates),bool);mask[changed]=True;regions=extract_regions(top,mask,rings=1,max_vertices=1)
    assert any(r.oversized for r in regions)
    print('START independent clean replay',flush=True);sdf.clear_memo();second=ForecastCache();b=second.forecast(front,top,schedule,sdf);replay=second.last_stats.copy();hash2=b.scientific_hash();assert hash1==hash2
    for group in ('vertex','edge','triangle','fine_ids'):
        for name,value in getattr(a,group).items():assert np.array_equal(value,getattr(b,group)[name],equal_nan=True) if value.dtype.kind not in 'US' else np.array_equal(value,getattr(b,group)[name])
    del b,second;gc.collect()
    summary=dict(target_residual=stats(a.vertex['residual']),absolute_target_residual=stats(abs(a.vertex['residual'])),gradient_norm=stats(a.vertex['gradient_norm']),predicted_step_length=stats(a.vertex['step_length']),direction_change_degrees=stats(np.degrees(a.vertex['direction_change'])),maximum_direction_change_per_vertex_degrees=stats(np.degrees(a.vertex['direction_change'].max(0))),neighbor_normal_angle_degrees=stats(np.degrees(a.edge['normal_angle'])),edge_length_ratio=stats(a.edge['ratio']),triangle_area_ratio=stats(a.triangle['ratio']),projection_iterations=stats(a.vertex['iterations']),normal_convergence=stats(a.edge['normal_convergence']),section_curvature=stats(a.vertex['section_curvature']))
    # Compact scientific arrays: three groups avoid a large monolithic archive.
    for first in range(0,20,5):
        last=first+5
        np.savez_compressed(OUT/f'forecast_layers_{first+1:02}_{last:02}.npz',coordinates=a.vertex['coordinates'][first:last],normals=a.vertex['normal'][first:last],phi=a.vertex['phi'][first:last],residual=a.vertex['residual'][first:last],iterations=a.vertex['iterations'][first:last],gradient_norm=a.vertex['gradient_norm'][first:last],direction_change=a.vertex['direction_change'][first:last],edge_ratio=a.edge['ratio'][first:last],neighbor_angle=a.edge['normal_angle'][first:last],triangle_ratio=a.triangle['ratio'][first:last])
    selectors=np.unique(np.linspace(0,len(front.coordinates)-1,17,dtype=np.int64));edgeids=np.unique(np.linspace(0,len(top.edges)-1,11,dtype=np.int64));triids=np.unique(np.linspace(0,len(top.triangles)-1,11,dtype=np.int64))
    np.savez_compressed(OUT/'selected_verifier_samples.npz',vertices=selectors,wall=front.coordinates[selectors],wall_phi=wall.phi[selectors],wall_gradient=wall.gradient[selectors],coordinates=a.vertex['coordinates'][:,selectors],normal=a.vertex['normal'][:,selectors],phi=a.vertex['phi'][:,selectors],residual=a.vertex['residual'][:,selectors],gradient_norm=a.vertex['gradient_norm'][:,selectors],iterations=a.vertex['iterations'][:,selectors],edge_ids=edgeids,edges=top.edges[edgeids],edge_ratio=a.edge['ratio'][:,edgeids],triangle_ids=triids,triangles=top.triangles[triids],triangle_ratio=a.triangle['ratio'][:,triids])
    extrema=[]
    for entity,name,values in [('vertex','absolute_target_residual',abs(a.vertex['residual'])),('vertex','direction_change_degrees',np.degrees(a.vertex['direction_change'])),('edge','normal_angle_degrees',np.degrees(a.edge['normal_angle'])),('edge','length_ratio',a.edge['ratio']),('triangle','area_ratio',a.triangle['ratio'])]:
        for sense,fn in [('min',np.nanargmin),('max',np.nanargmax)]:
            k,i=np.unravel_index(fn(values),values.shape);ids=np.array([i]) if entity=='vertex' else (top.edges if entity=='edge' else top.triangles)[i];point=a.vertex['coordinates'][k,ids].mean(0)
            extrema.append(dict(entity=entity,descriptor=name,extremum=sense,layer=int(k+1),source_index=int(i),source_vertex_ids=';'.join(front.vertex_ids[j] for j in ids),value=float(values[k,i]),x=float(point[0]),y=float(point[1]),z=float(point[2])))
    with (OUT/'DESCRIPTOR_EXTREMA.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(extrema[0]));w.writeheader();w.writerows(extrema)
    result=dict(initial_front=authority,wall_residual=stats(wall.phi),wall_gradient_norm=stats(wall.gradient_norm),wall_normal_selection_count=int(wall.boundary_normal_selection.sum()),wall_branch_tie_count=int(wall.branch_tie.sum()),sign_samples=dict(points=sign_points.tolist(),phi=sign.phi.tolist(),gradient=sign.gradient.tolist(),sign='positive outside; negative inside'),schedule_total=float(schedule.targets[-1]),forecast_layers=20,provisional_vertex_states=20*len(front.coordinates),provisional_edge_states=20*len(top.edges),provisional_triangle_states=20*len(top.triangles),topology_seconds=topology_seconds,initial_sdf_seconds=sdf.seconds-cold['sdf_seconds']-local_stats['sdf_seconds']-replay['sdf_seconds'],cold=cold,warm=warm_stats,local=local_stats,replay=replay,summary=summary,hard_events=a.hard_events(),cache_key=dataclasses.asdict(a.key),cache_key_sha256=a.key.sha256,invalidation=dict(requested_vertices=changed.tolist(),vertices=invalid['vertices'].tolist(),edges=invalid['edges'].tolist(),triangles=invalid['triangles'].tolist(),recomputed_equal=True),regions=[dataclasses.asdict(r) for r in regions],scaling=dataclasses.asdict(v3_scaling_case()),scientific_hash_run1=hash1,scientific_hash_run2=hash2,deterministic_exact=True,seconds=time.perf_counter()-started)
    write('BENCHMARK.json',result);print('BENCHMARK COMPLETE',result['seconds'],result['hard_events'],flush=True)

if __name__=='__main__':main()
