from task4h_common import *
import cProfile,pstats,psutil
from sdf_cfd_mesher_v4.forecast.forensics import exact_wedge
import task4_prepare_next as prod
def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();tri=front.triangles
    hi=np.load(BASE/'build_A/FRONT_01.npy');p=np.concatenate((front.coordinates[tri[:4096]],hi[tri[:4096]]),axis=1)
    lat=[];results=[]
    for cell in p:
        t=time.perf_counter();results.append(exact_wedge(cell));lat.append(time.perf_counter()-t)
    pr=cProfile.Profile();pr.enable()
    for cell in p:exact_wedge(cell)
    pr.disable();rows=[]
    for (file,line,name),(cc,nc,tt,ct,callers) in pstats.Stats(pr).stats.items():
        rows.append(dict(file=Path(file).name,line=line,name=name,calls=nc,self_seconds=tt,total_seconds=ct))
    rows.sort(key=lambda r:-r['self_seconds'])
    save('JACOBIAN_PROFILE.json',dict(cells=len(p),unprofiled_seconds=sum(lat),latency_quantiles=np.quantile(lat,[0,.5,.9,.99,1]).tolist(),call_tree=rows,scientific_hash=scientific_hash(results),setup_and_profile_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset))
    print('PROFILE_DONE',sum(lat),flush=True)
if __name__=='__main__':run()
