"""Bounded diagnosis from unchanged archived forecast coordinates."""
from pathlib import Path
import sys,os,time,json,threading,collections
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'));sys.dont_write_bytecode=True
import numpy as np,psutil
from types import SimpleNamespace
from task2_describe import read as task2read
from forensics_intersection import load_coordinates,make_side,reproduce
from sdf_cfd_mesher_v4.common.authority import frozen,file_hash
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.forecast.forensics import cluster_failures,first_invalid,dependency_window,exact_wedge,cell_events,face_events
from sdf_cfd_mesher_v4.forecast.detailed.patch import Patch,require_closure,column_patch,LocalityFailure
from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache
from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure
from sdf_cfd_mesher_v4.forecast.detailed.continuous import SideIndex
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import CertificateSession
OUT=ROOT/'results/nominal_forecast_forensics'
def write(name,value):(OUT/name).write_text(json.dumps(plain(value),indent=2,sort_keys=True,allow_nan=False)+'\n')
def patch_for(top,columns,max_columns=128):
    core=np.array(sorted(columns),dtype=np.int64);edges=np.unique(top.triangle_edges[core]);halo=set(map(int,core))
    for e in edges:halo.update(map(int,top.edge_triangles.row(e)))
    if len(halo)>max_columns:raise LocalityFailure('ForensicClosureBoundExceeded')
    cols=np.array(sorted(halo),dtype=np.int64)
    p=Patch(*(frozen(v,'<i8') for v in (core,cols,np.unique(top.triangles[cols]),np.unique(top.triangles[core]),edges)),top.sha256);require_closure(p,top);return p
def same(a,b):return np.array_equal(a,b,equal_nan=True)

def investigate(front,top,x,sdf,schedule,cells,faces,groups,representative=True):
    future=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':x[1:]});cache=DetailedCache();histories=[];components=[];exact=[];closure=[];traces=[]
    original={r['cell_id']:r for r in cells};face_original={(r['kind'],r['layer'],r['entity']):r for r in faces}
    for g in groups:
        patch=patch_for(top,g['columns']);r=cache.get(front,future,top,patch);assert cache.get(front,future,top,patch) is r
        # Enlarge only the halo, leaving measured core and core sides unchanged.
        extra=set(map(int,patch.columns))
        for c in patch.columns:
            for e in top.triangle_edges[c]:extra.update(map(int,top.edge_triangles.row(e)))
        assert len(extra)<=128,'ForensicHaloExpansionNotLocal'
        cols=np.array(sorted(extra));larger=Patch(patch.core_columns,frozen(cols,'<i8'),frozen(np.unique(top.triangles[cols]),'<i8'),patch.core_vertices,patch.side_edges,top.sha256)
        rr=measure(front,future,top,larger)
        assert scientific_hash(r['cells'])==scientific_hash(rr['cells']) and scientific_hash(r['faces'])==scientific_hash(rr['faces']),'HaloDependentMetric'
        closure.append(dict(component=g['component'],normal_columns=len(patch.columns),expanded_columns=len(extra),core_columns=len(patch.core_columns),core_vertices=len(patch.core_vertices),halo_vertices=len(patch.vertices)-len(patch.core_vertices),metrics_equal=True,prisms_with_halo=20*len(patch.columns)))
        vids=patch.vertices;query=sdf.evaluate(x[:,vids].reshape(-1,3));shape=(21,len(vids));qphi=query.phi.reshape(shape);qnormal=query.normal.reshape(shape+(3,));qgrad=query.gradient.reshape(shape+(3,));qerr=query.source_error_bound.reshape(shape)
        # Compare the native source against sealed archive diagnostics locally.
        for first in range(1,21,5):
            with np.load(ROOT/f'results/task1/forecast_layers_{first:02}_{first+4:02}.npz') as z:
                assert same(qphi[first:first+5],z['phi'][:,vids]);assert same(qnormal[first:first+5],z['normals'][:,vids])
        targets=np.r_[0.,schedule.targets];assert np.all(abs(qphi-targets[:,None])<=qerr)
        traces.append(dict(component=g['component'],vertices=vids,stable_vertex_ids=[front.vertex_ids[v] for v in vids],coordinates=x[:,vids],target_distances=targets,
                           phi=qphi,gradient=qgrad,normal=qnormal,gradient_norm=query.gradient_norm.reshape(shape),
                           boundary_normal_selection=query.boundary_normal_selection.reshape(shape),branch_tie=query.branch_tie.reshape(shape)))
        ci=r['cells'];matches=0;fmatches=0
        for j,column in enumerate(patch.core_columns):
            v=top.triangles[column];local=np.searchsorted(vids,v);wall=front.coordinates[v];walln=np.cross(wall[1]-wall[0],wall[2]-wall[0]);walln/=np.linalg.norm(walln)
            for k in range(20):
                row={key:value[k,j].tolist() for key,value in ci.items()};id=row['cell_id']
                if id in original:
                    for key in ci:assert same(row[key],original[id][key]),(id,key,row[key],original[id][key])
                    matches+=1
                delta=x[k+1,v]-x[k,v];normal_step=delta@walln;tangent=np.linalg.norm(delta-normal_step[:,None]*walln,axis=1)
                lower=x[k,v];upper=x[k+1,v];edge_lengths=np.linalg.norm(upper[[1,2,0]]-upper,axis=1)
                area=.5*np.linalg.norm(np.cross(upper[1]-upper[0],upper[2]-upper[0]));p=np.concatenate((lower,upper))
                if id in original and original[id]['jacobian']<=0:
                    ex=exact_wedge(p);assert ex['jacobian_sign']<=0,('Binary64JacobianFalseFailure',id,ex)
                    assert (ex['volume_sign']<=0)==(original[id]['signed_volume']<=0),('VolumeSignDisagrees',id)
                    exact.append(dict(cell_id=id,column=int(column),layer=k+1,**ex))
                history=dict(component=g['component'],**row,vertices=v,normal_step_from_wall=normal_step,tangential_step_from_wall=tangent,
                             source_wall_normal=walln,edge_lengths=edge_lengths,front_triangle_area=area,gradient_normals=qnormal[k+1,local],
                             trajectory_directions=delta/np.linalg.norm(delta,axis=1)[:,None],predicted_displacement=delta,
                             sdf_target_residual=qphi[k+1,local]-schedule.targets[k])
                histories.append(history)
        face_history=[]
        for i,fid in enumerate(r['face_ids']):
            kind,k,e,a,b=fid;row=dict(kind=kind,layer=k,entity=e,owner_column=a,neighbor_column=b,**{key:float(value[i]) for key,value in r['faces'].items()})
            if (kind,k,e) in face_original:
                for key in r['faces']:assert same(row[key],face_original[(kind,k,e)][key]),('FaceReplayMismatch',fid,key)
                fmatches+=1
            face_history.append(row)
        points=front.coordinates[top.triangles[g['columns']]]
        counts=collections.Counter(ev['kind'] for ev in g['events']);first=first_invalid(g['events']);coplanar=[]
        for c in g['columns']:
            q=x[:,top.triangles[c]]
            axes=[a for a in range(3) if np.all(q[...,a]==q[0,0,a])]
            if axes:coplanar.append(dict(column=c,constant_axes=axes,values=[float(q[0,0,a]) for a in axes]))
        comps=dict(component=g['component'],columns=g['columns'],observed_events=g['events'],event_counts=dict(counts),original_core_cell_matches=matches,original_face_matches=fmatches,
                   source_coordinate_min=points.min((0,1)),source_coordinate_max=points.max((0,1)),layer_range=[min(k for e in g['events'] for k in e['layers']),max(k for e in g['events'] for k in e['layers'])],
                   first_invalid_by_kind=first,dependency_by_kind={kind:dependency_window(k) for kind,k in first.items()},
                   all_forecast_vertices_coplanar=coplanar,face_histories=face_history,classification='GenuineNominalForecastHazard')
        components.append(comps)
    inter=reproduce(front,top,x,sdf,schedule)
    session=CertificateSession();pair_history=[]
    # Complete 20x20 history only for the two known intersecting source edges.
    # Not a search over other surface pairs or over the entire shell.
    for a in range(1,21):
        pa,_,_=make_side(x,top,1,a)
        for b in range(1,21):
            pb,_,_=make_side(x,top,729,b);result=session.pair(pa,pb)
            pair_history.append(dict(layer_a=a,layer_b=b,available_after_layer=max(a,b),classification=result['classification'],proof_hash=scientific_hash(result)))
    forbidden=[r for r in pair_history if r['classification']=='CertifiedForbiddenIntersection']
    assert forbidden;first_pair=min(forbidden,key=lambda r:(r['available_after_layer'],r['layer_a'],r['layer_b']))
    cert=[]
    if representative:
        index=SideIndex(front,future,top)
        # One new full local column scope, supplemented by the two sealed
        # successful Task-2 scopes. No requirement that nominal forecasts pass.
        p=column_patch(top,58145)
        cr=index.certify(p,seconds_budget=120)
        cert=[{key:value for key,value in cr.items() if key!='seconds'}|dict(core_column=58145)]
    return plain(dict(intersection=inter,components=components,cell_histories=histories,vertex_traces=traces,closure=closure,exact_jacobian_sign_checks=exact,
                      pair_history=pair_history,earliest_known_edge_pair_intersection=first_pair,intersection_dependency=dependency_window(first_pair['available_after_layer']),
                      representative_certification=cert)),dict(detailed_hits=cache.hits,detailed_misses=cache.misses,invalidated_entries=0,sdf_calls=sdf.calls,sdf_vertex_queries=sdf.vertex_queries)

def main():
    start=time.perf_counter();memory={'peak_rss':0};done=threading.Event()
    def monitor():
        proc=psutil.Process()
        while not done.wait(.05):
            memory['peak_rss']=max(memory['peak_rss'],proc.memory_info().rss)
            if memory['peak_rss']>12884901888:os._exit(91)
    thread=threading.Thread(target=monitor,daemon=True);thread.start()
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));x=load_coordinates(front);sdf=NativeSdf();schedule=load_schedule()
    cells=task2read('DIRECT_PRISM_ROWS.json');faces=task2read('DIRECT_FACE_ROWS.json');inter=task2read('CONTINUOUS.json')[-1]['failure'];groups=cluster_failures(top,cells,faces,inter)
    write('CLUSTER_DEFINITION.json',groups);print('FAILURE COMPONENTS',len(groups),flush=True)
    t=time.perf_counter();a,counters=investigate(front,top,x,sdf,schedule,cells,faces,groups);run1=time.perf_counter()-t;h1=scientific_hash(a);write('SCIENTIFIC_OUTPUTS.json',a)
    print('PASS1',run1,h1,flush=True)
    t=time.perf_counter();b,counters2=investigate(front,top,x,sdf,schedule,cells,faces,groups);run2=time.perf_counter()-t;h2=scientific_hash(b)
    assert h1==h2,'ForensicScientificNondeterminism';write('DETERMINISM.json',dict(run1=h1,run2=h2,passed=True,scope='All final forensic scientific outputs, including qualified proof hashes and representative certification'))
    done.set();thread.join();write('PERFORMANCE.json',dict(total_seconds=time.perf_counter()-start,run1_seconds=run1,run2_seconds=run2,**memory,run1_cache=counters,run2_cache_cumulative_sdf=counters2,archive_loads=4,full_forecast_runs=0,forecast_projections=0,coordinate_changes=0,max_workers=1))
    print('FORENSICS REPLAY PASS',h1,flush=True)
if __name__=='__main__':
    try:main()
    except Exception as e:
        import traceback
        write('STOP.json',dict(type=type(e).__name__,message=str(e),traceback=traceback.format_exc()));raise
