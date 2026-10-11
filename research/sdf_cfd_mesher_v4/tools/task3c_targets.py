"""Exact selected-cell histories of the proposed cap corrections."""
from task3_repair_study import *
from task3c_io import read as read_evidence
OUT=ROOT/'results/task3c'

def run(replay=False):
    started=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,*_=ctx
    cert=json.loads((OUT/'CERTIFICATION.json').read_text());rows=[]
    for r in cert['proposals']:
        ids=np.array(r['changed_vertices']);trial=np.array(r['layer1_coordinates']);fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=fc.forecast(ids,trial)
        for spec in components():
            if spec['name'] not in r['members']:continue
            records=[]
            for col in spec['columns']:
                vertices=top.triangles[col]
                for layer in range(1,21):
                    low0=front.coordinates[vertices] if layer==1 else coords[layer-2,vertices]
                    low1=front.coordinates[vertices] if layer==1 else future.vertex['coordinates'][layer-2,vertices]
                    upper=future.vertex['coordinates'][layer-1,vertices]
                    records.append(dict(column=col,layer=layer,vertices=vertices,
                        baseline=exact_wedge(np.concatenate((low0,coords[layer-1,vertices]))),
                        candidate=exact_wedge(np.concatenate((low1,upper))),
                        candidate_displacement=np.linalg.norm(upper-coords[layer-1,vertices],axis=1),
                        actual_step=np.linalg.norm(upper-low1,axis=1)))
            rows.append(dict(component=spec['name'],records=records,
                first_exact_nonpositive_before=min((r['layer'] for r in records if r['baseline']['jacobian_sign']<=0),default=None),
                first_exact_nonpositive_after=min((r['layer'] for r in records if r['candidate']['jacobian_sign']<=0),default=None)))
    h=scientific_hash(rows)
    if replay:assert scientific_hash(json.loads((OUT/'TARGET_TRAJECTORIES.json').read_text()))==h
    else:(OUT/'TARGET_TRAJECTORIES.json').write_text(json.dumps(plain(rows),indent=2,sort_keys=True)+'\n')
    (OUT/('TARGET_TRAJECTORIES_REPLAY.json' if replay else 'TARGET_TRAJECTORIES_HASH.json')).write_text(json.dumps(dict(scientific_hash=h),indent=2)+'\n')
    (OUT/('TARGET_TRAJECTORIES_REPLAY_TIMINGS.json' if replay else 'TARGET_TRAJECTORIES_TIMINGS.json')).write_text(json.dumps(dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)
