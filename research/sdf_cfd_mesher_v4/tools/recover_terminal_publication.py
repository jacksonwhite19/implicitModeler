"""Recover a fully staged terminal publication and independently seal it."""
import sys,json,time,traceback,os,gc
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import terminal_shell_store as store
from terminal_capacity import remesh_admitted,prevent_terminal_advance
from sdf_cfd_mesher_v4.common.authority import file_hash,identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
from validation_row_stream import clean_rows_digest
from audit_capsule_scaling import final_coverage,AABBIndex
from bounded_fallback_control import Events
from investigate_large_fallback import save
import generalize_local_remesh as g
r=ROOT/'results/terminal_capacity_aware_remesh_acceptance';cp=r/'checkpoint';e=Events(r/'SEAL_EVENTS.jsonl',1800);out={};start=time.perf_counter()
try:
 e('stage_start',stage='verify_staged_publication')
 pending=cp/'CURRENT.pending';record=json.loads(pending.read_text())
 assert record['schema']=='V4TerminalMixedShellCheckpoint/1'
 assert Path(record['payload']).name==record['payload'] and file_hash(cp/record['payload'])==record['sha256']
 import gzip,pickle
 from compact_proof_rows import CompactProofRows
 from native_proof_segments import SegmentProofRows
 class PendingCompact(CompactProofRows):
  def extend(self,rows):pass
 class PendingSegments(SegmentProofRows):
  def extend(self,rows):pass
 class PendingReader(pickle.Unpickler):
  def find_class(self,module,name):
   if (module,name)==('compact_proof_rows','CompactProofRows'):return PendingCompact
   if (module,name)==('native_proof_segments','SegmentProofRows'):return PendingSegments
   return super().find_class(module,name)
 with gzip.open(cp/record['payload'],'rb') as f:d=PendingReader(f).load()
 d['proofs'].__class__=SegmentProofRows;d['proofs'].fresh.__class__=CompactProofRows
 assert remesh_admitted(d['receipt']) and d['scientific']==record['scientific']
 assert d['state']['local_schedule']['terminal_capacity']==record['capacity']
 geom=scientific_digest({k:d['state'][k] for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','local_schedule')})
 assert geom==d['geometry_digest']
 assert identity(dict(schema='V4TerminalMixedShellScientific/1',geometry=geom,quality=d['quality_digest'],proofs=d['proof_digest'],capacity=record['capacity']['witness'],closure=json.loads((r/'PUBLICATION_CLOSURE.json').read_text())['closure_identity']))==d['scientific']
 assert not (cp/'CURRENT.json').exists()
 save(r/'UNPUBLISHED_V1_DESCRIPTOR.json',record)
 record=store.publish(cp,d,lambda candidate:remesh_admitted(candidate['receipt']));out['atomic_publication']=True
 e('stage_end',stage='atomic_publication',scientific=d['scientific'],bytes=record['bytes'])
 del d;gc.collect();e('stage_start',stage='restart');d=store.restart(cp);assert remesh_admitted(d['receipt']);e('stage_end',stage='restart',exact=True)
 try:prevent_terminal_advance(d['state']);raise AssertionError('TerminalReactivated')
 except ValueError as exc:assert str(exc)=='GlobalTerminalCapacityAlreadyResolved'
 p,c,o=d['points'],d['cells'],d['outer'];owners,own=g.t.ownership(c,o,[tuple(reversed(f)) for f in d['wall']]);assert own['passed'] and g.t.vertex_links(o)['passed']
 ingest=json.loads((r/'CORE_INGEST_INPUT.json').read_text());ids=sorted({v for f in o for v in f});lookup={v:i+1 for i,v in enumerate(ids)}
 assert ingest['coordinates']==p[ids].tolist() and ingest['triangles']==[[lookup[v] for v in f] for f in o] and ingest['outer_hash']==identity(o)
 e('stage_start',stage='independent_quality');q=g.t.quality(p,c,owners,1);assert q['passed'] and scientific_digest(q)==d['quality_digest'];e('stage_end',stage='independent_quality')
 from final_shell_incidence import check
 e('stage_start',stage='independent_incidence');incidence=check(p,c,o);assert incidence['passed'];e('stage_end',stage='independent_incidence')
 e('stage_start',stage='independent_global_degree');v=g.volume.audit(p,c,o,d['wall']);assert v['positive_cells'] and v['exact_volume_balance'] and v['interior_degree']['passed'];e('stage_end',stage='independent_global_degree')
 original=AABBIndex.pairs
 def pairs(self,*a,**kw):
  for i,pair in enumerate(original(self,*a,**kw),1):
   if i%500000==0:e('coverage_progress',pairs=i)
   yield pair
 AABBIndex.pairs=pairs;e('stage_start',stage='independent_coverage');coverage=final_coverage(p,d['faces'],d['proofs']);assert coverage['missing']==0 and coverage['duplicates']==0;e('stage_end',stage='independent_coverage',**coverage)
 def rows():
  for i,row in enumerate(d['proofs'],1):
   if i%500000==0:e('proof_identity_progress',rows=i)
   yield row
 e('stage_start',stage='independent_proof_identity');assert clean_rows_digest(rows())==d['proof_digest'];methods,bad=d['proofs'].summary();assert not bad;e('stage_end',stage='independent_proof_identity')
 from collections import Counter
 result=dict(passed=True,scientific=d['scientific'],geometry_digest=d['geometry_digest'],quality_digest=d['quality_digest'],proof_digest=d['proof_digest'],capacity=d['receipt']['terminal_capacity'],coverage=coverage,forbidden=0,unresolved=0,global_degree=True,exact_volume_balance=True,wall_first_layer_exact=True,restart_exact=True,
 determinant_min=min(z['determinant'] for z in q['cells']),weight_min=min(z['weight'] for z in q['faces']),total_cells=len(c),cell_types=dict(Counter(z['kind'] for z in c)),last_prism_layers=dict(Counter(z['last_prism_layer'] for z in d['state']['columns'].values())),terminal_columns=len(d['state']['columns']),seconds=time.perf_counter()-start)
 save(r/'SEAL.json',result);out['seal']=result
except BaseException as exc:out.update(error=str(exc),traceback=traceback.format_exc());e('failure',error=str(exc))
out['seconds']=time.perf_counter()-start;save(r/'RECOVERY_RESULT.json',out)
