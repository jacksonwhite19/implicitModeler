"""Fresh frozen-wall forecast and complete direct diagnostic coverage."""
from pathlib import Path
import os,sys,json,time
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
import numpy as np,psutil
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.forecast.engine import vertex_trajectories
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer,discover_regions
from sdf_cfd_mesher_v4.common.authority import array_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
from task3c_io import write
OUT=Path(os.environ.get('TASK4_OUTPUT_DIRECTORY',str(ROOT/'results/task4'))).resolve()
if not any(OUT.is_relative_to((ROOT/'results'/name).resolve()) for name in ('task4','task4e','task4i','task4l')):raise ValueError('Task4OutputScope')
OUT.mkdir(parents=True,exist_ok=True)

def retain_projection_buffers(data):
    """Keep only the two trajectories consumed by the production scan.

    Projection diagnostics remain part of the measured construction result;
    they are not inputs to any subsequent layer metric or scientific record.
    """
    return {key:data[key] for key in ('coordinates','normal')}


def initial_scan(context=None):
    OUT=context.output if context is not None else globals()['OUT']
    start=time.perf_counter()
    if context is None:front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));sdf=NativeSdf();schedule=load_schedule()
    else:front,top,sdf,schedule=context.front,context.top,context.sdf,context.schedule
    if (OUT/'INITIAL_FORECAST.npz').exists():
        with np.load(OUT/'INITIAL_FORECAST.npz') as z:vdata={k:z[k] for k in ('coordinates','normals')}
        vdata['normal']=vdata.pop('normals');projection=None
    else:
        vdata,projection=vertex_trajectories(front,schedule,sdf,np.arange(len(front.coordinates)))
        np.savez_compressed(OUT/'INITIAL_FORECAST.npz',coordinates=vdata['coordinates'],normals=vdata['normal'])
    vdata=retain_projection_buffers(vdata)
    forecast_seconds=time.perf_counter()-start;previous=None;low=front.coordinates;records=[];all_events=[]
    for k,high in enumerate(vdata['coordinates'],1):
        result=evaluate_layer(top,low,high,k,previous);records.append(dict(layer=k,gate=result['gate'],events=result['events']))
        all_events+=result['events'];previous=result['cells'];low=high
        if k==1:np.savez_compressed(OUT/'INITIAL_LAYER1_METRICS.npz',**{f'cell_{n}':x for n,x in result['cells'].items()},**{f'side_{n}':x for n,x in result['sides'].items()})
        print('SCAN',k,result['seconds'],result['gate']['violations'],flush=True)
    regions={name:[dict(vertices=r.vertex_ids,columns=r.triangle_ids,oversized=r.oversized) for r in discover_regions(top,events)] for name,events in [('immediate',records[0]['events']),('all_future',all_events)]}
    scientific=dict(wall=front.coordinate_hash,topology=top.sha256,forecast_coordinates=array_hash(vdata['coordinates']),schedule=schedule.targets,records=records,regions=regions)
    write(OUT/'INITIAL_SCAN.json',scientific)
    timing=dict(total_seconds=time.perf_counter()-start,forecast_seconds=forecast_seconds,projection_seconds=projection,peak_rss=psutil.Process().memory_info().peak_wset)
    (OUT/'INITIAL_TIMINGS.json').write_text(json.dumps(timing,indent=2)+'\n')
    print('REGIONS',[(k,[(len(r['vertices']),len(r['columns'])) for r in x]) for k,x in regions.items()],flush=True)
    print('DONE',scientific_hash(scientific),timing,flush=True)

if __name__=='__main__':initial_scan()
