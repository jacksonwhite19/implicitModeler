"""Publication-only control. Scientific inputs are reused; final outputs stay sealed."""
from pathlib import Path
from collections import Counter
import sys, os, json, time, shutil, subprocess, hashlib, sqlite3
from datetime import datetime, timezone
import psutil
from urllib.parse import urlsplit, unquote
from urllib.request import url2pathname
import numpy as np
import immutable_state_access as access
from path_preserving_deduplication import durable_json, file_hash, metadata
from clean_sheet_l4_bootstrap import metrics
from composable_state import CommittedState, plan_transition
from qualify_recursive_layer6 import inputs
from incremental_pair_storage import PairIncidence
from sdf_cfd_mesher_v4.common.authority import identity

V4=Path(__file__).resolve().parents[1]
SOURCE=V4/'results/recursive_layer6'
_name=os.environ.get('V4_CONTROL_DIRECTORY','optimized_l6_publication_control')
if _name not in ('optimized_l6_publication_control','optimized_l6_publication_control_copy'):
 raise ValueError('UnqualifiedControlOutputDirectory')
OUT=V4/'results'/ _name
PARTIAL=V4/'results/l6_write_pair_scaling_resume'
ALLOWED=['GENERATION.json','GEOMETRY_DELTA.json','CHANGED_INPUTS.json','NEW_QUALITY.json',
 'AFFECTED_INCIDENCE.json','FRESH_CERTIFICATE.json','VOLUME_DELTA.json','ADMISSION.json','GENERATED_NEXT.npz']
def tick():return time.perf_counter(),time.process_time(),psutil.Process().io_counters()
def now():return datetime.now(timezone.utc).isoformat()
def read(p):return access.read(p)

def copy_scientific_input(source,target,expected):
 before=access.token(source)
 shutil.copyfile(source,target)
 if access.token(source)!=before:raise ValueError('ScientificInputTokenChangedDuringStaging')
 if file_hash(target)!=expected:raise ValueError('StagedScientificInputCorrupt')
 return before

class ReadGuard:
 def __init__(self,allowed):
  self.allowed={os.path.normcase(str(Path(p).resolve())) for p in allowed};self.reads=Counter();self.forbidden=[]
  self.prefix=os.path.normcase(str(SOURCE.resolve()))+os.sep
 def __call__(self,event,args):
  if event not in ('open','sqlite3.connect'):return
  p=args[0]
  if not isinstance(p,(str,bytes,os.PathLike)):return
  p=os.fsdecode(p)
  if p.startswith('file:'):p=url2pathname(unquote(urlsplit(p).path))
  p=os.path.normcase(os.path.abspath(p));self.reads[p]+=1
  if p.startswith(self.prefix) and p not in self.allowed:
   self.forbidden.append(p);raise RuntimeError('ForbiddenFinalL6OracleRead:'+p)

def preflight():
 OUT.mkdir(exist_ok=False)
 assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()=='39a63034f6dd7c8da3e1d1b49b5718a73d2f1d9d'
 assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
 pre=read(PARTIAL/'PREFLIGHT.json');auth=read(PARTIAL/'AUTHORITY.json')
 assert auth['report_identity']=='656e88281d2b24541c1ca5684dbaf81727b376f75b3dc82dc40f036790fe0c81'
 assert file_hash(PARTIAL/'REPORT.json')==auth['files']['REPORT.json']
 import recursive_epoch_recovery as recovery
 ref=recovery.discover(V4/'results/recursive_epoch_recovery/access')['reference']
 assert ref['root']=='d247c8e6bf3803e8ad74db6ba2b6e7f8468e73903fcb2c1141d481b777b4c793'
 with CommittedState(ref) as current:
  assert current.layer==6 and current.content_root=='15259f4e506a7ae574666283b40d33428f788de8f9d5f3e29314d15ef27178f0'
 # Root access above verifies the supplied authority only, before construction.
 # No finalized descriptor is exported into the candidate input closure.
 cert=read(SOURCE/'FRESH_CERTIFICATE.json')
 paths=[SOURCE/n for n in ALLOWED]+[SOURCE/'objects'/cert['segment'][k]['file'] for k in ('faces','pairs','codes','payloads')]
 closure={str(p.resolve()):file_hash(p) for p in paths}
 for k in ('faces','pairs','codes','payloads'):
  assert closure[str((SOURCE/'objects'/cert['segment'][k]['file']).resolve())]==cert['segment'][k]['sha256']
 durable_json(OUT/'PREFLIGHT.json',dict(parent=pre['parent'],allowed=closure,free=shutil.disk_usage(V4).free,
  status_hex=subprocess.check_output(['git','status','--porcelain=v1','-z']).hex(),epoch=access.epoch(),authority_access_verified=True))

def construct(mode):
 pre=read(OUT/'PREFLIGHT.json');guard=ReadGuard(pre['allowed']);sys.addaudithook(guard)
 t=tick()
 try:
  if mode=='stage':
   (OUT/'objects').mkdir()
   for name,h in pre['allowed'].items():
    p=Path(name);assert file_hash(p)==h
    target=OUT/'objects'/p.name if p.parent.name=='objects' else OUT/p.name
    # Even adding a hardlink changes NTFS ChangeTime on the source object.
    # A bounded scientific-input copy preserves existing trusted tokens.
    copy_scientific_input(p,target,h)
   with CommittedState(pre['parent']) as base:
    adapter,_=inputs(base);new=plan_transition(base,adapter)
    g=read(OUT/'GENERATION.json');old=g['transition_plan']
    differences={k:[old.get(k),new.get(k)] for k in set(old)|set(new) if old.get(k)!=new.get(k)}
    assert set(differences)=={'base_root'},differences
    g['transition_plan']=new;durable_json(OUT/'GENERATION.json',g)
    durable_json(OUT/'ACCESS_REBIND.json',dict(differences=differences,original_sha256=pre['allowed'][str((SOURCE/'GENERATION.json').resolve())],scientific_inputs_unchanged=True))
  elif mode=='assembly':
   import composable_assembly
   with CommittedState(pre['parent']) as base:
    adapter,_=inputs(base)
    with PairIncidence(base,PARTIAL/'L5_incidence/MANIFEST.json','704ac2088ddf7dd669fe1fee0d783091e724807477f4d13c734b4270e912ff6b') as index:
     composable_assembly.assemble(base,adapter,OUT,index)
  elif mode=='publish':
   import composable_publication
   import incremental_pair_storage
   real_index=incremental_pair_storage.write_proof_index
   def measured_index(*args):
    start=tick();result=real_index(*args)
    durable_json(OUT/'INDEX_MEASURED.json',dict(metrics=metrics(*start),writer=result,file=metadata(args[0]),sha256=file_hash(args[0])))
    return result
   incremental_pair_storage.write_proof_index=measured_index
   real_prepare=composable_publication.prepare
   def prepare_with_incidence(base,adapter,out):
    plan=real_prepare(base,adapter,out);start=tick()
    cert=read(out/'FRESH_CERTIFICATE.json');descriptor=cert['segment']
    def verify(p):
     expected=next(v['sha256'] for v in descriptor.values() if isinstance(v,dict) and v.get('file')==p.name)
     if file_hash(p)!=expected:raise ValueError('ChangedNewProofSegment')
    class NewSegmentInput:
     content_root=identity(dict(new_segments=[descriptor]))
     def proof_segments(self):yield descriptor,out/'objects',verify
     def __getitem__(self,key):
      if key!='certificate':raise KeyError(key)
      return range(descriptor['count'])
    root=incremental_pair_storage.build_incidence(NewSegmentInput(),out/'new_incidence')
    added=read(out/'new_incidence/MANIFEST.json')
    with PairIncidence(base,PARTIAL/'L5_incidence/MANIFEST.json','704ac2088ddf7dd669fe1fee0d783091e724807477f4d13c734b4270e912ff6b') as parent:
     assert parent.body['parent_count']+descriptor['count']==cert['combined_pairs']
     merged=dict(added,parent_content=plan['scientific'],parent_count=cert['combined_pairs'],segments=parent.body['segments']+added['segments'],files={**parent.body['files'],**added['files']},inherited_index_root=parent.root)
    path=out/'INCIDENCE_MANIFEST.json';durable_json(path,merged)
    plan['pair_incidence_root']=identity(merged);plan['pair_incidence_manifest']=str(path.resolve())
    plan['references'].update(merged['files']);plan['references'][str(path.resolve())]=access.attest(path,file_hash(path))
    durable_json(out/'CANDIDATE_PLAN.json',plan)
    durable_json(out/'INCIDENCE_MEASURED.json',dict(root=identity(merged),new_segment_root=root,inherited_segments=3,new_pairs=descriptor['count'],prior_pair_rows_scanned=0,metrics=metrics(*start)))
    return plan
   composable_publication.prepare=prepare_with_incidence
   original=sqlite3.connect;events=[]
   class MeasuredConnection(sqlite3.Connection):
    def __init__(self,*args,**kw):
     self.filename=str(args[0]);super().__init__(*args,**kw)
    def commit(self):
     before=tick();result=super().commit();events.append(dict(file=self.filename,operation='commit',metrics=metrics(*before)));return result
    def executemany(self,*args,**kw):
     before=tick();result=super().executemany(*args,**kw);events.append(dict(file=self.filename,operation='executemany',metrics=metrics(*before)));return result
   def connect(*args,**kw):kw['factory']=MeasuredConnection;return original(*args,**kw)
   sqlite3.connect=connect
   with CommittedState(pre['parent']) as base:
    adapter,_=inputs(base)
    root=composable_publication.publish(base,adapter,OUT)
   durable_json(OUT/'SQL_COMMITS.json',events)
   durable_json(OUT/'REFERENCE.json',dict(kind='recursive',directory=str((OUT/'candidate').resolve()),root=root))
  else:raise ValueError(mode)
  durable_json(OUT/(mode.upper()+'_MEASURED.json'),dict(metrics=metrics(*t),forbidden=guard.forbidden,reads=dict(guard.reads)))
 except BaseException as e:
  durable_json(OUT/(mode.upper()+'_FAILURE.json'),dict(error=repr(e),metrics=metrics(*t),forbidden=guard.forbidden,reads=dict(guard.reads)))
  raise

if __name__=='__main__':
 if sys.argv[1]=='preflight':preflight()
 else:construct(sys.argv[1])
