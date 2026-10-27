"""Real full-delta publication safety controls in isolated metadata stores."""
from optimized_l6_publication_control import *
from incremental_transaction import Store,writer
from unittest.mock import patch
import incremental_transaction as tx

def main():
 start=tick();pre=read(OUT/'PREFLIGHT.json');guard=ReadGuard(pre['allowed']);sys.addaudithook(guard)
 plan=read(OUT/'CANDIDATE_PLAN.json');quality=read(OUT/'QUALITY_DELTA.json');bindings=read(OUT/'BINDING_PATCH.json')
 parent=read(OUT/'PUBLICATION.json')['parent'];results={};directory=OUT/'safety';directory.mkdir(exist_ok=False)
 class Interrupted(Exception):pass
 def setup(name):
  store=Store(directory/name,object_directory=directory/'objects')
  initial=store.initialize(dict(epoch=access.epoch(),scientific=plan['content']['parent_scientific'],plan=dict(references={})))
  acceptance=dict(read(OUT/'CANDIDATE_ACCEPTANCE.json'),base=initial)
  assert initial==parent
  return store,acceptance
 def run(store,a,callback=None):return store.publish(parent,plan,quality,bindings,a,identity(a),callback)
 for stage in ('prepared','component_partial','before_pointer','after_pointer'):
  store,a=setup(stage)
  def interrupt(s):
   if s==stage:raise Interrupted(stage)
  try:run(store,a,interrupt);raise AssertionError('MissingInterruption')
  except Interrupted:pass
  before=store.current()['root']
  assert (before!=parent)==(stage=='after_pointer')
  root=run(store,a);assert root==store.current()['root']
  assert run(store,a)==root
  with store.open(root):pass
  results[stage]=dict(passed=True,recovery_root=root,exact_base_before_recovery=before==parent,idempotent_retry=True)
 store,a=setup('pointer_failure');real=tx.durable_json
 def fail_pointer(path,value):
  if Path(path)==store.path/'CURRENT.json':raise OSError('InjectedAtomicPointerFailure')
  return real(path,value)
 try:
  with patch.object(tx,'durable_json',fail_pointer):run(store,a)
  raise AssertionError('MissingPointerFailure')
 except OSError:pass
 assert store.current()['root']==parent
 root=run(store,a);results['pointer_failure']=dict(passed=True,recovery_root=root)
 for name in ('stale_base','concurrent_writer','corrupt_changed_reference'):
  store,a=setup(name)
  try:
   if name=='stale_base':
    stale=dict(a,base='0'*64);store.publish('0'*64,plan,quality,bindings,stale,identity(stale))
   elif name=='concurrent_writer':
    with writer(store.path):run(store,a)
   else:
    # Corrupt an isolated reference token, never a production/candidate file.
    altered=dict(plan,references=dict(plan['references']));p=next(iter(altered['references']))
    altered['references'][p]=dict(altered['references'][p],token={'invalid':True})
    aa=dict(a,plan=identity(altered));store.publish(parent,altered,quality,bindings,aa,identity(aa))
   raise AssertionError('ExpectedRejection')
  except ValueError as e:results[name]=dict(passed=True,error=str(e))
  assert store.current()['root']==parent
 durable_json(OUT/'SAFETY_CONTROL.json',dict(passed=True,results=results,metrics=metrics(*start),forbidden=guard.forbidden,reads=dict(guard.reads),full_quality_rows_per_attempt=len(quality),production_modified=False))
 print(json.dumps(results,indent=2),flush=True)

if __name__=='__main__':main()
