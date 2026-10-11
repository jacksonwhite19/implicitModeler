"""Exactly-one-layer production adapter; no fixture wall or future proposals."""
import json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import psutil
from run_geometry_family import AnalyticProvider
import generalize_local_remesh as g
from geometry_family_authority import Source
from production_adaptive_surface import CapsuleGeometry
from production_surface_authority import load_production_wall,verify_surface_sidecars
from production_prism_sizing import inspect_first_layer_inputs,read_request
from qualification_prism_progression import verify_bound_request
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front import rebuild_dependencies,scientific_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
V4=Path(__file__).resolve().parents[1];OUT=V4/'results/production_adaptive_capsule_first_layer'
SURFACE=V4/'results/production_adaptive_capsule_surface_foundation/fresh'
REQUEST=V4/'results/qualification_prism_progression_convention/REQUEST.json'

def save(p,x):g.save(Path(p),x)

class ProductionFirstLayerProvider(AnalyticProvider):
 def __init__(self,folder,*,surface=None,request=None,binding=None):
  if binding is not None:
   self._bound_inputs(folder,surface,request,binding)
   return
  self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True);self.times={}
  t=time.perf_counter();self.x,self.tri,self.wall_authority=load_production_wall(SURFACE);self.times['wall_load']=time.perf_counter()-t
  t=time.perf_counter();self.admission=inspect_first_layer_inputs(SURFACE,REQUEST)
  if file_hash(REQUEST)!='37375760bf77a7df2ff6e34ca0a87b6d437dcee604a5d8bf8ddb3c3483c40d80':raise ValueError('RequestFileMismatch')
  req=verify_bound_request(REQUEST,self.wall_authority,'ec28d584b04afb8bd4679d6719d0f600140b63dcff26a0cc50c2607537d33b91')
  if self.admission['schedule_identity']!='ae2cfd505b0cb52218c4f000688df150e8f2c65f2d2d28de78785f1de39f0597':raise ValueError('ScheduleIdentityMismatch')
  if self.wall_authority['surface_identity']!='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd':raise ValueError('WallIdentityMismatch')
  self.h1=float(req['increments'][0]);self.schedule=SimpleNamespace(targets=np.array([self.h1]))
  self.times['request_decode']=time.perf_counter()-t
  geo=json.loads((SURFACE.parent/'GEOMETRY.json').read_text());authority=CapsuleGeometry(geo['definition'])
  if authority.geometry_identity!=self.wall_authority['geometry_identity']:raise ValueError('GeometryIdentityMismatch')
  self.spec=dict(components=[geo['definition']]);self.source=Source(self.spec)
  import first_layer_metric_state
  from native_mixed_face_certificate import SOURCE
  self.authority=dict(schedule=req['schedule_identity'],quality=file_hash(Path(g.t.__file__)),certificate=file_hash(Path(g.s.cert.__file__)),
   certificate_filter=file_hash(SOURCE),first_prism_metric=file_hash(Path(first_layer_metric_state.__file__)),sdf=self.wall_authority['geometry_identity'],
   production_surface=self.wall_authority['surface_identity'],request=self.admission['request_identity'])
  self.records=[];self.checked={};self.meshes={};self.numerical={};self.reuse_certificates=True;self.previous_certificate_state=None
  self.x.setflags(write=False);self.tri.setflags(write=False)
 def _bound_inputs(self,folder,surface,request,binding):
  """Explicit inputs to the unchanged analytic L1 generator and predicates."""
  from standalone_geometry import is_scalar,branch_evidence
  from production_prism_sizing import require_generation_authorization
  if not is_scalar(binding) or surface is None or request is None:raise ValueError('UnsupportedAnalyticFirstLayerBinding')
  self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=False);self.times={}
  self.surface=Path(surface);self.request=Path(request);self.binding=binding
  binding.qualification.verify(binding.authority)
  self.x,self.tri,self.wall_authority=load_production_wall(surface)
  if binding.geometry_identity!=self.wall_authority['geometry_identity']:raise ValueError('GeometryIdentityMismatch')
  self.admission=inspect_first_layer_inputs(surface,request);require_generation_authorization(self.admission)
  req=read_request(request,self.wall_authority)
  if req['declaration']['fluid_side']!='positive_sdf':raise ValueError('UnsupportedAnalyticFluidSide')
  self.h1=float(req['increments'][0]);self.schedule=SimpleNamespace(targets=req['targets'])
  self.spec=dict(components=[dict(binding.authority.definition)]);self.source=Source(self.spec)
  self.source.sheet_names=dict(binding.authority.sheet_names)
  _,reference_hash=verify_surface_sidecars(self.surface,self.wall_authority,verify_implementation=False)
  if file_hash(self.surface/'REFERENCES.npz')!=reference_hash:raise ValueError('ReferenceIdentityMismatch')
  with np.load(self.surface/'REFERENCES.npz',allow_pickle=False) as z:self.refs={k:z[k] for k in z.files}
  if set(self.refs['face_refs'])!={1} or len(self.refs['curve_edges']) or np.any(self.refs['corner_refs']):
   raise ValueError('AnalyticSourceReferenceMismatch')
  self.branch=branch_evidence(binding)
  import first_layer_metric_state
  from native_mixed_face_certificate import SOURCE
  self.authority=dict(schedule=req['schedule_identity'],quality=file_hash(Path(g.t.__file__)),certificate=file_hash(Path(g.s.cert.__file__)),
   certificate_filter=file_hash(SOURCE),first_prism_metric=file_hash(Path(first_layer_metric_state.__file__)),sdf=self.wall_authority['geometry_identity'],
   production_surface=self.wall_authority['surface_identity'],request=self.admission['request_identity'],
   source_references=file_hash(self.surface/'REFERENCES.npz'))
  self.records=[];self.checked={};self.meshes={};self.numerical={};self.reuse_certificates=True;self.previous_certificate_state=None
  self.x.setflags(write=False);self.tri.setflags(write=False)
  save(self.folder/'ADMISSION.json',dict(admission=self.admission,branch=self.branch,authority=self.authority,
   top_role='ProspectiveFirstPrismDeterminant; no future centroid; not a permanent terminal cap'))
 def verify_wall(self):
  x,f,r=load_production_wall(self.surface)
  if x.tobytes()!=self.x.tobytes() or f.tobytes()!=self.tri.tobytes() or r!=self.wall_authority:raise ValueError('WallMutation')
  self.binding.qualification.verify(self.binding.authority)
  return True
 def propose(self,*args,**kw):raise ValueError('ExactlyOneLayerScope')
 def geometry(self,state,extra=0):
  if extra:raise ValueError('NoLaterFrontConstruction')
  n=len(self.x);high=np.empty_like(self.x)
  for i in range(n):high[i]=state['vertices'][f'layer:1:source:{i}']
  if any(int(k.split(':')[1])>1 for k in state['vertices']):raise ValueError('LaterLayerState')
  return [high],np.ones(n,int)
 def phase(self,name,fn):
  print(name,'start',flush=True);t=time.perf_counter();r=fn();self.times[name]=time.perf_counter()-t
  print(name,'done',self.times[name],flush=True);return r
 def generate_candidate(self):
  sample=self.phase('normal_construction',lambda:self.source.evaluate(self.x));norm=np.linalg.norm(sample.normal,axis=1)
  if not np.isfinite(sample.normal).all() or np.any(norm<=0):raise ValueError('InvalidSourceNormal')
  # Same generic normal-offset proposal as the production analytic provider,
  # applied to the immutable admitted wall instead of regenerating a fixture.
  direction=sample.normal
  high=self.phase('first_front_construction',lambda:self.x+self.h1*direction)
  target=self.phase('target_admission',lambda:self.source.evaluate(high))
  residual=target.phi-self.h1
  self.high=high;self.direction=direction;self.source_sample=sample
  np.savez(self.folder/'CANDIDATE.npz',wall=self.x,triangles=self.tri,front=high,direction=direction,gradient=sample.gradient,residual=residual)
  self.assigned=dict(admitted=bool(np.all(np.abs(residual)<=target.source_error_bound)),residual_max=float(np.abs(residual).max()),projection_operations=0,projection_failures=0,normal_fallbacks=0,zero_gradients=0)
  if hasattr(self,'binding'):
   save(self.folder/'TRAJECTORY.json',dict(self.assigned,branch=self.branch,full_height_m=self.h1,
    full_height_hex=self.h1.hex(),semantics='Existing analytic normal offset and scalar target audit; no feature normalization'))
   if not self.assigned['admitted']:raise ValueError('FirstAnalyticTargetRejected')
  p,c,o,check,times=self.phase('raw_quality',lambda:g.validation(self.x,self.tri,[high],np.ones(len(self.x),int),False))
  self.times.update(times)
  check=self.phase('correct_first_layer_metric',lambda:self.initial_quality(check,high))
  save(self.folder/'RAW_QUALITY.json',check)
  # Complete certificate even for a failed candidate, to distinguish blockers.
  check=self.phase('continuous_and_volume',lambda:self.certify_candidate([high],np.ones(len(self.x),int),check))
  self.points,self.cells,self.outer,self.check=p,c,o,check
  save(self.folder/'QUALITY.json',check['quality']);save(self.folder/'TOPOLOGY.json',dict(ownership=check['ownership'],vertex_links=check['vertex_links']))
  save(self.folder/'VOLUME.json',check['volume'])
  cert=check['certificate'];save(self.folder/'CERTIFICATE_SUMMARY.json',{k:v for k,v in cert.items() if k!='proofs'})
  # Preserve every certificate row compactly in a stream, not a second full list.
  with (self.folder/'CERTIFICATE.jsonl').open('w') as f:
   for row in cert['proofs']:f.write(json.dumps(row,sort_keys=True,separators=(',',':'))+'\n')
  passed=check['passed'] and self.assigned['admitted']
  save(self.folder/'RESULT.json',dict(passed=passed,sdf=self.assigned,quality_passed=check['quality']['passed'],times=self.times,correction_used=False))
  return passed
 def commit_candidate(self):
  if not self.check['passed'] or not self.assigned['admitted']:raise ValueError('FailedFirstLayerCommit')
  state=self.pack([self.high],np.ones(len(self.x),int));self.bind(state,self.check)
  rebuild_dependencies(None,state,[self.h1])
  from first_layer_metric_state import receipt
  state['quality']['first_prism_incidence']=receipt(self.points,self.cells,self.outer,state['versions'])
  return MixedFrontRunner(self.folder/'checkpoints',self).initialize(state)

def run(label):
 t=time.perf_counter();provider=ProductionFirstLayerProvider(OUT/label);passed=provider.generate_candidate()
 committed=None
 if passed:
  state=provider.phase('commit_checkpoint',provider.commit_candidate);committed=scientific_hash(state)
 x,f,wall=load_production_wall(SURFACE)
 assert np.array_equal(x,provider.x) and np.array_equal(f,provider.tri) and wall==provider.wall_authority
 result=dict(passed=passed,committed=committed,front_coordinate_hash=array_hash(provider.high),front_topology_hash=array_hash(provider.tri),
  prism_connectivity_hash=array_hash(np.array([c['vertices'] for c in provider.cells],dtype=np.int64)),ownership_hash=identity([(c['id'],c['column']) for c in provider.cells]),
  quality_hash=file_hash(provider.folder/'QUALITY.json'),certificate_hash=provider.check['certificate']['proof_hash'],wall_preserved=True,
  times=provider.times,total_seconds=time.perf_counter()-t,peak_rss_bytes=psutil.Process().memory_info().peak_wset,retained_rss_bytes=psutil.Process().memory_info().rss)
 save(provider.folder/'FINAL.json',result);print(json.dumps(result),flush=True)
if __name__=='__main__':
 import sys
 run(sys.argv[1])
