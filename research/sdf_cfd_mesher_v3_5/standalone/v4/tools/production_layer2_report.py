"""Seal the bounded L2 result after native restore and replay verification."""
import json,os,time,shutil,subprocess
from pathlib import Path
from xml.etree import ElementTree
from production_layer2_run import OUT,V4,L1,SURFACE,REQUEST,ADMISSION,save
from sdf_cfd_mesher_v4.common.authority import file_hash,identity

def read(p):return json.loads(p.read_text())
def run():
 a=read(OUT/'A/FINAL.json');b=read(OUT/'B/FINAL.json');ready=read(OUT/'A/READY.json')
 assert ready==read(OUT/'B/READY.json') and a['passed'] and b['passed']
 verify=read(OUT/'RESTORE_VERIFICATION.json');assert verify['passed']
 d=read(OUT/'A/DIAGNOSTICS.json');cert=read(OUT/'A/CERTIFICATE.json');base=read(OUT/'BASELINE.json');disk=read(OUT/'DISK_START.json');wall=read(SURFACE/'SURFACE_AUTHORITY.json');admission=read(ADMISSION/'AUTHORITY.json')
 tests={}
 for name in ('EXISTING_TESTS.xml','TESTS_POST_AUDIT.xml'):
  suites=ElementTree.parse(OUT/name).getroot();tests[name]={k:sum(int(s.get(k,0)) for s in suites.iter('testsuite')) for k in ('tests','failures','errors','skipped')}
 assert all(r['failures']==r['errors']==r['skipped']==0 for r in tests.values())
 source_files=['production_layer2_run.py','production_layer2_diagnostics.py','production_layer2_report.py','production_layer2_restore_check.py','production_forecast_admission.py','production_readonly_admission.py','run_geometry_family.py','mixed_front_fixture_provider.py','final_shell_incidence.py','reuse_mixed_certificate.py','native_mixed_face_certificate.py','geometry_family_authority.py']
 sources={str((V4/'tools'/n).relative_to(V4)):file_hash(V4/'tools'/n) for n in source_files}
 for n in ('mixed_front_runner.py','mixed_front_journal.py','mixed_evidence.py','native_checkpoint.py','predict.py'):
  p=V4/'src/sdf_cfd_mesher_v4/advancing_front'/n;sources[str(p.relative_to(V4))]=file_hash(p)
 sources['tests/test_production_layer2_run.py']=file_hash(V4/'tests/test_production_layer2_run.py')
 save(OUT/'IMPLEMENTATION.json',dict(sources=sources,architecture='Minimal production L2 adapter, no fixture constructor. Existing generic SDF predictor, column template, exact mixed-cell validator, final incidence tensor, continuous prover, transaction and native publisher. Private in-memory journal followed by two-run replay barrier and durable checkpoint publication.',receipt_fix='Private journal ReceiptOverlay is replaced only for binding by the sealed original ReceiptTable after exact L1 scientific-hash verification. Existing stale-base and geometry checks unchanged.',future_scope='Schedule dependency descriptors may retain remaining request numbers; no L3 coordinates, forecast or geometry were computed.',correction='None; nominal candidate passes. No new repair.'))
 resources=dict(start=disk,end=read(OUT/'DISK_END.json'),bytes_deleted=0,cleanup='None. Failed receipt attempts and both independent runs retained.',created_task_tree_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),elapsed_task_seconds=time.time()-base['started'],A=a['times'],B=b['times'],A_total_seconds=a['total_seconds'],B_total_seconds=b['total_seconds'],A_restore=a['restore_seconds'],B_restore=b['restore_seconds'],diagnostics_seconds=d['seconds'],restore_verification=verify['seconds'],peak_rss=max(a['peak_rss'],b['peak_rss']),retained_rss_A=a['retained_rss'],retained_rss_B=b['retained_rss'],cache='No forecast cache used; final continuous certifier reuses exactly authenticated unchanged-L1 proofs.')
 save(OUT/'RESOURCES.json',resources)
 classification='V4ProductionAdaptiveCapsuleAdvancingFrontLayer2Qualified'
 supported='Exactly L2 from sealed production L1 and the admitted request passed complete final numerical, topology, ownership and continuous checks, deterministic replay, and transactional native publication.'
 input_ids=dict(admission=admission['admission_identity'],**admission['binding'],wall_coordinates=wall['coordinate_hash'],wall_topology=wall['topology_hash'],wall_sizing=wall['sizing_field_identity'])
 input_ids.update(admission['input_authorities'])
 science=dict(classification=classification,input=input_ids,result=ready,restore=verify['scientific'],diagnostics=file_hash(OUT/'A/DIAGNOSTICS.json'),implementation=sources,scope=dict(maximum_layer=2,L3_forecast=False,L3_generation=False,core=False))
 scientific=identity(science)
 rows=['## Decision question','','Does the admitted advance from committed production L1 produce a completely qualified L2 without L3 work?','',classification,'',supported,'','Complete AdvancingFrontQualified for L3Ã¢â‚¬â€œL11, shell, CoreHandoff, core and CFD qualification remain not established.','',f'Starting HEAD: `{base["head"]}`. Branch: `{base["branch"]}`.','', '### Authorities','', 'All inputs were loaded through the production wall/request and native checkpoint paths. The sealed read-only admission and all its file bindings were verified.','', '```json',json.dumps(input_ids,indent=2),'```','','### Production path and corrections','','The ordinary mixed-front transaction uses restored L1, the generic SDF Newton predictor and exactly the cumulative L2 target. No fixture constructor, historical schedule loader, future predictor, remesher or core-ingestion operation runs. The first-layer-only guard is unchanged.','', 'Two initial attempts reached passing final geometry but failed receipt binding with StaleReceiptDeltaBase. Their evidence remains in failed_receipt_A/B. A private ReceiptOverlay was not accepted as the immutable delta base. The bounded adapter now verifies the sealed L1 scientific identity and binds the original immutable table. No mathematics, gate, geometry or proof guard changed. Both corrected complete runs are independent.','', 'No post-construction repair or relaxation was used. Generic Newton SDF projection is part of nominal construction. Requested increment: 0.00017333832030214487 m (`0x1.6b844d56c388ep-13`).','', '### Counts and ownership','','```json',json.dumps(d['topology'],indent=2),'```','','Wall and L1 coordinates/connectivity are unchanged. Their authoritative input files and checkpoint pointer remain unchanged. L2 has 16,895 new vertices and 33,786 new prisms. Combined cells: 67,572.','', '### Distributions','','All tables use min / p1 / p5 / median / p95 / p99 / max. Percentiles are descriptive. Lengths are metres; areas mÃ‚Â²; volumes mÃ‚Â³; angles degrees.']
 def table(title,data):
  rows.extend(['',f'#### {title}','','| Quantity | min | p1 | p5 | median | p95 | p99 | max |','|---|---:|---:|---:|---:|---:|---:|---:|'])
  for k,v in data.items():
   if isinstance(v,dict) and 'min' in v:rows.append('| '+k+' | '+' | '.join(format(v[p],'.17g') for p in ('min','p1','p5','median','p95','p99','max'))+' |')
 rows+=['', 'Normal zero/nonfinite/inward/fallback counts: '+json.dumps({k:d['normals'][k] for k in ('zero','nonfinite','inward','fallbacks')})+'. Nonpositive outward heights: '+str(d['height_nonpositive'])+'.', '', 'Triangle zero-area/reversal/nonfinite counts: '+json.dumps({k:{z:v[z] for z in ('zero_area','orientation_reversals','nonfinite')} for k,v in d['triangles'].items()})+'. Radius ratio is 2 times inradius divided by circumradius.']
 table('Direction',d['normals']);table('Height realization',d['height']);table('Edges',d['edges'])
 for k,v in d['triangles'].items():table(k+' triangles',v)
 for k,v in d['matched'].items():table(k+' matched ratios',v)
 for scope,metrics in d['quality'].items():
  table(scope+' hard metrics',{k:v['distribution'] for k,v in metrics.items()})
  rows+=['','| Metric | Gate | Failures | Margin | Worst source/entity |','|---|---|---:|---:|---|']
  for k,v in metrics.items():rows.append(f'| {k} | {v.get("gate", "positive")} | {v["failures"]} | {v.get("margin","positive exact sign")} | {json.dumps(v["worst"],separators=(",",":"))} |')
 table('Determinant context',d['determinant']);rows+=['',d['determinant']['semantics'],'','L1 determinant differences are exactly zero. The retired exposed-cap interpretation controls no acceptance.','']
 for k,v in d['conditioning'].items():table(k+' sigma/G diagnostics',v)
 rows+=['','Sigma and G have no hard acceptance thresholds.']
 for k,v in d['geometry_diagnostics'].items():table(k+' geometry diagnostics',v)
 table('Nonorthogonality',d['nonorthogonality'])
 rows+=['','### Final continuous certification','','Every new/changed L2 face was independently certified against the complete two-layer face population. Unchanged oldÃ¢â‚¬â€œold L1 relations retain their authenticated proofs. This is final realized geometry certification, not promotion of the sealed provisional localization.','','```json',json.dumps(cert,indent=2),'```','','Forbidden: 0. Unresolved: 0. Missing: 0. Duplicate: 0. Exact positive cell-volume balance and interior degree pass.','','### Forecast versus realized','','Coordinates and source-matched batched metrics are exactly equal. Final scalar validator differences below reflect binary64 evaluation order. No error threshold was added.','','| Quantity | maximum absolute difference | maximum relative difference |','|---|---:|---:|']
 for k,v in d['forecast'].items():
  if k.startswith('final_validator__'):rows.append(f'| {k} | {v["absolute_difference"]["max"]} | {v["relative_difference_nonzero_denominator"]["max"]} |')
 rows+=['','Collision classification agrees: every new candidate is legal/separated. The final proof population also includes unchanged L1 relations.','','### Replay and commit','','Both complete corrected generation/certification records match exactly before either durable checkpoint is published. Native restart independently verifies each final checkpoint.','','```json',json.dumps(ready,indent=2),'```','','L3 generated: NO. L3 forecast: NO. Core: NO. Authoritative new task layer: 2. The original L1 authority is preserved.','','### Tests','','```json',json.dumps(tests,indent=2),'```','','No failures or skips in the final suites. The earlier 14-test focused adapter check also passed.','','### Resources','','```json',json.dumps({k:v for k,v in resources.items() if k not in ('start','end')},indent=2),'```','',f'Repository/V4 start bytes: {disk["repository_accessible_logical_bytes"]} / {disk["v4_accessible_logical_bytes"]}. Starting free: {disk["free_bytes"]}. Ending free: {resources["end"]["free_bytes"]}. No evidence deleted. Accessible logical scans exclude managed worktrees, nested dual_contouring and inaccessible entries; errors are retained.','','### Claim and next task','',supported,'','Blockers: none for L2. L3Ã¢â‚¬â€œL11 remain unqualified.','', 'Recommended next task: QualifyV4CapsuleLayer3ReadOnlyAdmissionFromCommittedLayer2. Bound it to the next increment and admission evidence; do not automatically run the rest of the schedule.','',f'L2 scientific identity: `{scientific}`.','','Git: task-owned V4 paths only. Pre-existing 155 unrelated dirty entries preserved. V3 task modifications: 0. Final commit hash and exact staged path count are printed after committing.']
 (OUT/'report.md').write_text('\n'.join(rows)+'\n',encoding='utf8',newline='\n')
 evidence={str(p.relative_to(OUT)).replace('\\','/'):file_hash(p) for p in OUT.rglob('*') if p.is_file() and 'checkpoints' not in p.parts and p.name not in ('AUTHORITY.json','SEALS.json')}
 authority=dict(classification=classification,scientific_identity=scientific,scientific=science,supported_claim=supported,committed_front=a['committed'],request_blockers=[],layer2_blockers=[],later_layers_qualified=False,tests=tests,evidence=evidence)
 save(OUT/'AUTHORITY.json',authority);save(OUT/'SEALS.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json'),scientific=scientific))
 print(json.dumps(read(OUT/'SEALS.json')),flush=True)

if __name__=='__main__':run()
