from task4h_spots import *
import cProfile,pstats
def run():
    front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));planner=SpotPlanner();pr=cProfile.Profile();records=[]
    for layer,trial,before,after,patch,known,expected in cohort(front,top,(2,)):
        require_time();t=time.perf_counter()
        pr.enable();planner.select(front,top,before,patch,layer,32);planner.select(front,top,after,patch,layer,32);pr.disable()
        records.append(dict(layer=layer,trial=trial,seconds=time.perf_counter()-t))
        if trial==10:break
    rows=[]
    for (f,line,name),(cc,nc,tt,ct,callers) in pstats.Stats(pr).stats.items():rows.append(dict(file=Path(f).name,line=line,name=name,calls=nc,self_seconds=tt,total_seconds=ct))
    save('SPOT_SELECTION_PROFILE.json',dict(rows=records,call_tree=sorted(rows,key=lambda r:-r['self_seconds']),spatial=dict(planner.spatial.stats),selection=dict(planner.stats)))
if __name__=='__main__':run()
