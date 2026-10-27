"""Fresh reopen, explicit deep verification, immutable seal, then oracle comparison."""
from optimized_l6_publication_control import *
from contextlib import ExitStack
from unittest.mock import patch
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
from probe_l6_index_writes import semantic

def incidence_deep(state):
 m=read(OUT/'INCIDENCE_MANIFEST.json');total=0
 assert identity(m)==state.body['plan']['pair_incidence_root']
 for s,(d,objects,verify) in zip(m['segments'],state.proof_segments()):
  assert identity(d)==s['identity'];verify(objects/d['pairs']['file'])
  p=np.load(objects/d['pairs']['file'],mmap_mode='r');o=np.load(s['offsets'],mmap_mode='r');v=np.load(s['ordinals'],mmap_mode='r')
  assert o[0]==0 and o[-1]==len(v)==2*len(p)
  counts=np.diff(o).astype(np.int64);assert np.all(counts>=0)
  ids=np.repeat(np.arange(s['face_count']),counts)
  assert np.all((p[v,0]==ids)|(p[v,1]==ids)) and np.all(np.bincount(v,minlength=len(p))==2)
  dif=np.diff(v.astype(np.int64));edges=o[1:-1].astype(np.int64)-1;edges=edges[(edges>=0)&(edges<len(dif))];dif[edges]=1
  assert np.all(dif>0)
  keys=np.load(s['keys'],mmap_mode='r');mapping=np.load(s['mapping'],mmap_mode='r')
  assert np.array_equal(keys,np.asarray(read(s['faces']),dtype='S64')[mapping]) and np.all(keys[1:]>keys[:-1])
  total+=len(p)
 assert total==m['parent_count']==len(state['certificate'])
 return dict(pairs=total,postings=2*total)

def run(mode):
 t=tick();pre=read(OUT/'PREFLIGHT.json');guard=ReadGuard(pre['allowed'])
 if mode!='oracle':sys.addaudithook(guard)
 ref=read(OUT/'REFERENCE.json')
 if mode=='reopen':
  def deny(*args,**kw):raise AssertionError('HistoricalSQLForbiddenOnReopen')
  with patch.object(sqlite3,'connect',deny),CommittedState(ref) as state:
   adapter,_=inputs(state);plan=plan_transition(state,adapter)
   assert state.layer==6 and plan['increment_hex']=='0x1.bd6749eb1b62ap-12'
   result=dict(passed=True,plan=plan,stats=dict(state.stats),proof_rows=0,quality_rows=0,sql_rows=0,full_prefix_hashes=0,full_prefix_copies=0)
 elif mode=='deep':
  with CommittedState(ref) as state:
   refs={};s=state;depth=0
   while s is not None:
    depth+=1
    if s.kind=='native':refs.update(s.wrapped.body['files'])
    elif s.kind=='clean_sheet':refs.update(s.wrapped.plan['references']);refs.update(s.wrapped.body['produced'])
    else:refs.update(s.body['plan']['references']);refs.update(s.body['produced'])
    s=getattr(s,'parent',None)
   total_bytes=0
   for p,r in refs.items():
    assert file_hash(p)==r['sha256'],p;total_bytes+=Path(p).stat().st_size
   incidence=incidence_deep(state)
   # Authenticate the entire closure once, then stream the complete candidate
   # semantics once. No repeated scientific digest of each ancestor is needed.
   for row in state.rows.values():row.deep=True
   try:science=scientific_digest(state)
   finally:
    for row in state.rows.values():row.deep=False
   dbpath=OUT/'objects/new-proof-key-index.sqlite'
   with sqlite3.connect(dbpath.as_uri()+'?mode=ro&immutable=1',uri=True) as db:index=semantic(db)
   result=dict(passed=True,scientific=science,content=state.content_root,incidence=incidence,index=index,
    files_authenticated=len(refs),bytes_authenticated=total_bytes,depth=depth,stats=dict(state.stats),normal_path=False)
 elif mode=='seal':
  deep=read(OUT/'DEEP_CONTROL.json');assert deep['passed'] and read(OUT/'REOPEN_CONTROL.json')['passed']
  for n in ('STAGE','ASSEMBLY','PUBLISH'):assert not read(OUT/(n+'_MEASURED.json'))['forbidden']
  with CommittedState(ref) as state:
   refs={**state.body['plan']['references'],**state.body['produced']}
   files={p:file_hash(p) for p in refs if Path(p).is_relative_to(OUT)}
   for p in (OUT/'candidate').glob('*.json'):files[str(p.resolve())]=file_hash(p)
   for n in ('CANDIDATE_PLAN.json','CANDIDATE_ACCEPTANCE.json','REFERENCE.json','INCIDENCE_MANIFEST.json'):
    files[str((OUT/n).resolve())]=file_hash(OUT/n)
   result=dict(sealed_at=now(),scientific=deep['scientific'],content=state.content_root,access=ref['root'],files=files,incidence_root=state.body['plan']['pair_incidence_root'],candidate_immutable=True)
 elif mode=='oracle':
  seal=read(OUT/'SEAL_CONTROL.json')
  with ExitStack() as locked:
   for p,h in seal['files'].items():
    locked.enter_context(access.guard(p));assert file_hash(p)==h
   unlock=now();durable_json(OUT/'ORACLE_UNLOCK.json',dict(timestamp=unlock,seal_sha256=file_hash(OUT/'SEAL_CONTROL.json')))
   old=read(SOURCE/'CANDIDATE_PLAN.json');new=read(OUT/'CANDIDATE_PLAN.json')
   components={k:v==old['components'][k] for k,v in new['components'].items()}
   assert all(components.values()),components
   oldscience=read(SOURCE/'DEEP.json')['scientific']
   assert seal['scientific']==oldscience=='4d49baf5d372827ea853edbfdedf80ca0909827be865dbdde58faa3da7630170'
   assert seal['content']==old['scientific']
   oldaccount=read(SOURCE/'ASSEMBLY.json');newaccount=read(OUT/'ASSEMBLY.json')
   fields=['prior_proofs_reused','prior_bindings_changed','prior_bindings_unchanged','new_proofs','quality_new','quality_input_only_updates','quality_numerical_updates']
   accounting={k:oldaccount[k]==newaccount[k] for k in fields};assert all(accounting.values())
   oldindex=SOURCE/'objects/new-proof-key-index.sqlite'
   assert file_hash(oldindex)==file_hash(OUT/'objects/new-proof-key-index.sqlite')
   with sqlite3.connect(oldindex.as_uri()+'?mode=ro&immutable=1',uri=True) as db:oldsemantic=semantic(db)
   assert oldsemantic==read(OUT/'DEEP_CONTROL.json')['index']
   for p,h in seal['files'].items():assert file_hash(p)==h
   result=dict(passed=True,components=components,accounting=accounting,scientific=oldscience,content_equal=True,index_bytes_equal=True,index_semantic=oldsemantic,seal_timestamp=seal['sealed_at'],oracle_unlock=unlock,no_post_oracle_mutation=True)
 else:raise ValueError(mode)
 result['metrics']=metrics(*t);result['forbidden']=guard.forbidden;result['reads']=dict(guard.reads)
 durable_json(OUT/(mode.upper()+'_CONTROL.json'),result)
 print(json.dumps({k:v for k,v in result.items() if k not in ('reads','files','plan')},indent=2),flush=True)

if __name__=='__main__':run(sys.argv[1])
