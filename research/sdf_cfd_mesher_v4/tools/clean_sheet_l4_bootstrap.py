"""L4-only enrollment and strict oracle-isolation bootstrap. No oracle reader."""
from pathlib import Path
from contextlib import ExitStack
import json,os,sys,time,subprocess,shutil
import numpy as np
import psutil
import immutable_state_access as access
from path_preserving_deduplication import durable_json,file_hash,metadata,physical_key
from rolling_layer_retention import checkpoint_closure
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from production_surface_authority import load_production_wall
from production_prism_sizing import read_request
from production_forecast_admission import ProductionForecastAdapter,BoundProductionForecastCache
from geometry_family_authority import Source
from clean_sheet_isolation import ReadBoundary,deny_native_reads

V4=Path(__file__).resolve().parents[1];REPO=V4.parents[1]
OUT=V4/'results/clean_sheet_incremental_l4_to_l5'
BASE=V4/'results/production_adaptive_capsule_layers4_to5/L4/A/checkpoints'
SURFACE=V4/'results/production_adaptive_capsule_surface_foundation/fresh'
REQUEST=V4/'results/qualification_prism_progression_convention/REQUEST.json'
L4='8673f3f242b41e9733aa94bae7e0ca409ccee24b0e944afe6ca87f563e2a04f4'

def read(p):return json.loads(Path(p).read_text())
def save(n,v):OUT.mkdir(parents=True,exist_ok=True);durable_json(OUT/n,v)
def metrics(start,cpu,io):
 p=psutil.Process();m=p.memory_info();now=p.io_counters()
 return dict(seconds=time.perf_counter()-start,cpu=time.process_time()-cpu,peak_rss=m.peak_wset,retained_rss=m.rss,
             io={k:getattr(now,k)-getattr(io,k) for k in io._fields})

def preflight():
 head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
 assert head=='6bb1c1d30b1e555e5cadc3170bd97af1a13bc40b'
 assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
 status=subprocess.check_output(['git','status','--porcelain=v1','-z']);rows=[r for r in status.split(b'\0') if r and b'clean_sheet' not in r];assert len(rows)==155
 refs,env=checkpoint_closure(BASE);assert env['scientific']==L4
 surface=read(SURFACE/'SURFACE_AUTHORITY.json')
 inputs=[REQUEST,SURFACE/'SURFACE_AUTHORITY.json',SURFACE.parent/'GEOMETRY.json']+[SURFACE/n for n in surface['files']]
 for p in inputs:refs[str(p.resolve())]=file_hash(p)
 request_refs={}
 for row in read(REQUEST)['source_documents']:
  p=(REQUEST.parent/row['path']).resolve();assert file_hash(p)==row['sha256'];request_refs[str(p)]=row['sha256']
 save('REQUEST_INPUT_CLOSURE.json',dict(allowed_inputs=request_refs,source=str(REQUEST),
      reason='Hash-bound pre-L5 physical request source documents; required by unchanged read_request'))
 records=[metadata(p) for p in refs];unique={physical_key(r):r for r in records}
 free=shutil.disk_usage(REPO.anchor).free
 save('BASELINE.json',dict(head=head,status_hex=b'\0'.join(rows).hex(),allowed_inputs=refs,L4=L4,free=free,
      referenced_physical=sum(r['cluster_bytes'] for r in unique.values()),oracle_payload_reads=0,
      disk_policy='Enrollment writes only packed L4 coordinates and compact front metadata; no geometry/proof work authorized by preflight'))
 print('PREFLIGHT',free,flush=True)

def enroll():
 start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
 b=read(OUT/'BASELINE.json');refs=dict(b['allowed_inputs'])
 refs.update(read(OUT/'REQUEST_INPUT_CLOSURE.json')['allowed_inputs'])
 env=read(BASE/read(BASE/'CURRENT.json')['checkpoint'])
 allowed_ids={(Path(p).stat().st_dev,Path(p).stat().st_ino) for p in refs}
 # Metadata-only inventory. No file contents or oracle hashes are read.
 forbidden=[]
 oracle_trees=['production_adaptive_capsule_layers4_to5','incremental_immutable_state_access',
               'incremental_transaction_publication','dependency_graph_scaling_audit']
 for tree in oracle_trees:
  for directory,_,files in os.walk(V4/'results'/tree):
   for name in files:
    p=Path(directory)/name
    if str(p.resolve()) in refs:continue
    forbidden.append(p)
 runtime=[sys.prefix,sys.base_prefix]+[p for p in sys.path if p and not Path(p).resolve().is_relative_to(REPO)]
 boundary=ReadBoundary(REPO,refs,OUT,runtime)
 # Native metadata/guard opens use one audited API object. DLLs process arrays;
 # arbitrary child processes are prohibited by the Python audit hook.
 import path_preserving_deduplication as fs
 original_api=fs.api
 class Api:
  def __init__(self):self.k=original_api()
  def __getattr__(self,key):return getattr(self.k,key)
  def CreateFileW(self,path,desired,*args):
   boundary.check(path,bool(desired & 0x40000000),'Win32.CreateFileW')
   return self.k.CreateFileW(path,desired,*args)
 try:
  with deny_native_reads(forbidden,allowed_ids) as locks:
   save('ORACLE_LOCKS.json',dict(entries=locks,oracle_bytes_read=0,scope='Metadata-only inventory and no-data exclusive handles; legitimate L4 physical aliases exempt'))
   boundary.install();fs.api=Api;access.api=Api
   with ExitStack() as held:
    for p in refs:held.enter_context(access.guard(p))
    state=NativeCheckpointStore(BASE).restart();assert scientific_hash(state)==L4
    wall,tri,surface=load_production_wall(SURFACE);bound=read_request(REQUEST,surface)
    definition=read(SURFACE.parent/'GEOMETRY.json')['definition'];source=Source(dict(components=[definition]))
    coordinates=np.array([state['vertices'][f'layer:4:source:{i}'] for i in range(len(wall))])
    expected=dict(committed=L4,geometry_identity=surface['geometry_identity'],surface_identity=surface['surface_identity'],
       request=identity(bound['declaration']),request_file=bound['request_file_sha256'],schedule=bound['schedule_identity'],layer=4,front_coordinates=array_hash(coordinates))
    assert expected['geometry_identity']=='a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65'
    assert expected['surface_identity']=='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'
    assert expected['request']=='ec28d584b04afb8bd4679d6719d0f600140b63dcff26a0cc50c2607537d33b91'
    assert expected['schedule']=='ae2cfd505b0cb52218c4f000688df150e8f2c65f2d2d28de78785f1de39f0597'
    adapter=ProductionForecastAdapter(state,wall,tri,surface,REQUEST,source,expected)
    assert float(adapter.remaining_increments[0]).hex()=='0x1.37351a26971d3p-12'
    target=OUT/'L4_access';objects=target/'objects';objects.mkdir(parents=True,exist_ok=True)
    def object_file(name,value,array=False):
     p=objects/name
     if array:
      with p.open('wb') as f:np.save(f,value,allow_pickle=False);f.flush();os.fsync(f.fileno())
     else:durable_json(p,value)
     return dict(path=str(p.resolve()),sha256=file_hash(p))
    keys=sorted(state['vertices']);values=np.asarray([state['vertices'][k] for k in keys],dtype='<f8')
    hot=dict(vertex_keys=object_file('keys.json',keys),vertex_values=object_file('coordinates.npy',values,True),
       front_faces=object_file('front_faces.json',{k:state['faces'][k] for k in state['prism_front'].values()}))
    for row in hot.values():refs[row['path']]=row['sha256']
    import production_forecast_admission as pf
    content=dict(schema='V4LegacyComponentMapping/1',legacy_scientific=L4,legacy_checkpoint_sha256=read(BASE/'CURRENT.json')['sha256'],
       legacy_components=env['components'],hot={k:v['sha256'] for k,v in hot.items()},
       implementation={str(Path(m.__file__).resolve()):file_hash(m.__file__) for m in (access,pf)},
       receipt_roles='Unchanged L4 authenticated components; no target inputs',quality='Exact immutable L4 quality')
    body=dict(schema=access.SCHEMA,complete=True,epoch=access.epoch(),store=str(BASE),legacy_pointer=read(BASE/'CURRENT.json'),
       components=env['components'],hot=hot,files={p:access.attest(p,h) for p,h in refs.items()},content=content,content_root=identity(content),
       counts={k:len(state[k]) for k in ('vertices','cells','faces','quality','certificate')},expected=expected,binding=adapter.binding)
    root=access.publish_manifest(target,body)
    with access.ImmutableState(target,root) as fresh:
     adapted=ProductionForecastAdapter.from_immutable_access(fresh,wall,tri,surface,REQUEST,source,expected)
     assert adapted.binding==adapter.binding
     cache=BoundProductionForecastCache(adapted);assert cache.cache.result is None
    save('L4_ENROLLMENT.json',dict(root=root,content_root=body['content_root'],scientific=L4,binding=adapter.binding,
       increment=float(adapter.remaining_increments[0]),counts=body['counts'],no_forecast=True,no_candidate_L5=True,
       enrollment_scope='One-time full L4 authentication to create missing L4 access enrollment; not incremental transition work',**metrics(start,cpu,io)))
 except BaseException as exc:
  save('ENROLLMENT_FAILURE.json',dict(error=type(exc).__name__+':'+str(exc),oracle_unlocked=False))
  raise
 finally:
  fs.api=original_api;access.api=original_api;save('L4_READ_LOG.json',boundary.record())
 print('L4 ENROLLED WITHOUT ORACLE',root,flush=True)

def reopen():
 start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
 enrolled=read(OUT/'L4_ENROLLMENT.json');refs=read(OUT/'BASELINE.json')['allowed_inputs']
 ids={(Path(p).stat().st_dev,Path(p).stat().st_ino) for p in refs}
 paths=[r['path'] for r in read(OUT/'ORACLE_LOCKS.json')['entries']]
 from unittest.mock import patch
 import sqlite3
 def denied(*args,**kwargs):raise AssertionError('NoHistoryTraversalOrForecastInL4Reopen')
 with deny_native_reads(paths,ids),access.ImmutableState(OUT/'L4_access',enrolled['root']) as state:
  wall,tri,surface=load_production_wall(SURFACE);source=Source(dict(components=[read(SURFACE.parent/'GEOMETRY.json')['definition']]))
  with patch.object(sqlite3,'connect',denied),patch.object(NativeCheckpointStore,'restart',denied),patch.object(BoundProductionForecastCache,'forecast',denied):
   adapter=ProductionForecastAdapter.from_immutable_access(state,wall,tri,surface,REQUEST,source,state.body['expected'])
   cache=BoundProductionForecastCache(adapter);assert cache.cache.result is None
  assert adapter.binding==enrolled['binding']
  save('L4_REOPEN.json',dict(root=enrolled['root'],scientific=L4,increment=float(adapter.remaining_increments[0]),
       stats=dict(state.stats),historical_proof_rows=0,historical_quality_rows=0,forecast_invoked=False,
       no_candidate_L5=True,**metrics(start,cpu,io)))
 print('TRUSTED L4 REOPEN PASSED',flush=True)

if __name__=='__main__':{'preflight':preflight,'enroll':enroll,'reopen':reopen}[sys.argv[1]]()
