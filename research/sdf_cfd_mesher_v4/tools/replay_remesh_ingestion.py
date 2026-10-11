"""Replay existing Gmsh smoke tools; keep timing outside scientific identities."""
import subprocess,time
import generalize_local_remesh as g
s=g.s;q=g.q
wsl=lambda p:'/mnt/'+str(p.resolve()).replace('\\','/')[0].lower()+str(p.resolve()).replace('\\','/')[2:]
rows={}
for name in ['general_sdf_baseline']+[v['name'] for v in g.fixtures.definitions()]:
    folder=g.OUT/name;before=s.load(folder/'CORE_INGESTION.json');st=time.perf_counter()
    subprocess.run(['wsl','--','python3',wsl(s.ROOT/'tools/cavity_core_ingestion_smoke.py'),wsl(folder/'CORE_INGEST_INPUT.json')],check=True,capture_output=True)
    after=s.load(folder/'CORE_INGESTION.json');assert before==after
    rows[name]=dict(equal=True,scientific_hash=q.identity(after))
    g.save(folder/'ingestion_replay_runtime.json',dict(seconds=time.perf_counter()-st))
g.save(g.OUT/'INGESTION_REPLAY.json',dict(equal=True,cases=rows))
folder=g.OUT/'clean_curved_passage';before=s.load(folder/'TET_SMOKE_RAW.json');st=time.perf_counter()
subprocess.run(['wsl','--','python3',wsl(s.ROOT/'tools/remesh_generalization_tet_smoke.py'),wsl(folder/'CORE_INGEST_INPUT.json')],check=True,capture_output=True)
after=s.load(folder/'TET_SMOKE_RAW.json');assert before==after
g.save(g.OUT/'TET_REPLAY.json',dict(equal=True,scientific_hash=q.identity(after)))
g.save(g.OUT/'TET_REPLAY_RUNTIME.json',dict(seconds=time.perf_counter()-st))
print('All ingestion and tet replays exact')
