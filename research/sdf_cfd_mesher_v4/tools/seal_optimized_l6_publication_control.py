"""Seal measured publication-control results; production authority is unchanged."""
from optimized_l6_publication_control import *
from path_preserving_deduplication import physical_key
import gzip

def main():
 oracle=read(OUT/'ORACLE_CONTROL.json');assert oracle['passed']
 deep=read(OUT/'DEEP_CONTROL.json');seal=read(OUT/'SEAL_CONTROL.json')
 assert seal['sealed_at']<oracle['oracle_unlock']
 for p,h in seal['files'].items():assert file_hash(p)==h
 assert read(OUT/'SAFETY_CONTROL.json')['passed'] and read(OUT/'STALE_BASE_CONTROL.json')['passed']
 # Only redundant successful-control payloads are retired. The unique partial
 # component and all failure/recovery records remain. Production is untouched.
 cleanup=[]
 for p in (OUT/'safety/objects').iterdir():
  if p.suffix=='.pending':continue
  assert p.resolve().is_relative_to((OUT/'safety/objects').resolve())
  canonical=OUT/'candidate/objects'/p.name
  assert canonical.is_file() and file_hash(p)==file_hash(canonical)
  row=metadata(p);row.update(sha256=file_hash(p),retained_equal=str(canonical.resolve()))
  cleanup.append(row)
 durable_json(OUT/'CLEANUP.json',dict(paths=cleanup,bytes=sum(r['bytes'] for r in cleanup),physical=sum(r['cluster_bytes'] for r in cleanup),reason='Disposable duplicate successful safety-control payload; regenerate from retained exact bytes and full input plan. Unique partial failure component retained.'))
 for r in cleanup:Path(r['path']).unlink()
 for p,h in seal['files'].items():assert file_hash(p)==h
 # Retire only byte-identical, unshared payloads of the superseded control.
 # Its hardlinks must remain: unlinking them would change production tokens.
 oldout=V4/'results/optimized_l6_publication_control'
 retired=[]
 selected=[oldout/n for n in ('QUALITY_DELTA.json','NEW_ATLAS.json','NEW_RECEIPT_INPUTS.json','NEW_FORECAST_DEPENDENCIES.json','BINDING_PATCH.json','CANDIDATE_HOT.json')]
 selected += [oldout/'objects/new-proof-key-index.sqlite']
 selected += list((oldout/'new_incidence').glob('*.npy'))+list((oldout/'candidate/objects').glob('*'))
 for p in selected:
  if not p.is_file():continue
  target=OUT/p.relative_to(oldout)
  if not target.is_file():continue
  row=metadata(p)
  if row['links']!=1:continue
  h=file_hash(p)
  if h!=file_hash(target):continue
  row.update(sha256=h,retained_equal=str(target.resolve()));retired.append(row)
 durable_json(OUT/'RETIRED_CONTROL_DUPLICATES.json',dict(paths=retired,physical=sum(r['cluster_bytes'] for r in retired),first_control_root_record_retained=True,source_hardlinks_preserved=True,scientific_comparison=file_hash(oldout/'ORACLE_CONTROL.json'),reason='Superseded control duplicate payloads retired after independent successful comparison; all unique token failure and partial-component evidence remains.'))
 for r in retired:
  assert Path(r['path']).resolve().is_relative_to(oldout.resolve())
  Path(r['path']).unlink()
 for p,h in seal['files'].items():assert file_hash(p)==h
 import recursive_epoch_recovery as recovery
 current=recovery.discover(V4/'results/recursive_epoch_recovery/access')
 with CommittedState(current['reference']) as state:
  assert state.layer==6 and state.content_root=='15259f4e506a7ae574666283b40d33428f788de8f9d5f3e29314d15ef27178f0'
  repair=read(V4/'results/optimized_l6_publication_control/PRODUCTION_ACCESS_REPAIR.json')
  assert current['reference']==repair['reference']
  assert current['scientific']==seal['scientific']
 pre=read(PARTIAL/'PREFLIGHT.json')
 partial=read(PARTIAL/'REPORT.json')
 assert partial['architecture_identity']=='168f27425c8027881002e78877e08af4285e8cf40a88a69c5ccce5d0410ed50e'
 for p,h in partial['implementation'].items():assert file_hash(V4/p)==h
 for p,h in pre['protected'].items():assert file_hash(p)==h
 v3pre=read(V4/'results/recursive_epoch_recovery/PREFLIGHT.json')
 with gzip.open(V4/'results/recursive_epoch_recovery'/v3pre['v3_inventory']['file'],'rt',encoding='utf8') as f:old=json.load(f)
 v3={}
 for directory,dirs,files in os.walk('research/sdf_cfd_mesher_v3',followlinks=False):
  for n in files:
   p=Path(directory)/n;s=p.lstat();v3[str(p)]=[s.st_size,s.st_mtime_ns]
 assert v3==old
 own=['tools/optimized_l6_publication_control.py','tools/verify_optimized_l6_control.py','tools/test_optimized_l6_recovery_control.py','tools/seal_optimized_l6_publication_control.py','tools/optimized_l6_publication_control.md','tests/test_optimized_l6_publication_control.py']
 names=[('research/sdf_cfd_mesher_v4/'+n).encode() for n in own]+[b'research/sdf_cfd_mesher_v4/results/optimized_l6_publication_control']
 status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
 untouched=b'\0'.join(x for x in status.split(b'\0') if x and not any(n in x for n in names))+b'\0'
 assert untouched.hex()==pre['status_hex']
 rows=[metadata(p) for p in OUT.rglob('*') if p.is_file()];unique={physical_key(r):r for r in rows}
 durable_json(OUT/'FINAL_STORAGE.json',dict(rows=rows,free=shutil.disk_usage(OUT).free))
 phases={n:read(OUT/(n+'_MEASURED.json')) for n in ('STAGE','ASSEMBLY','PUBLISH','INDEX','INCIDENCE')}
 for v in phases.values():assert not v.get('forbidden',[])
 totals={k:sum(phases[n]['metrics']['io'][k] for n in ('ASSEMBLY','PUBLISH')) for k in ('read_bytes','write_bytes','read_count','write_count')}
 index=phases['INDEX'];assert index['writer']['rows']==3576991
 accounted=index['metrics']['io']['write_bytes']+phases['INCIDENCE']['metrics']['io']['write_bytes']
 sources={n:file_hash(V4/n) for n in own+['tools/incremental_pair_storage.py','tools/composable_publication.py','tools/composable_assembly.py','tools/incremental_transaction.py']}
 report=dict(classification='V4IncrementalWriteAmplificationAndPairIndexScalingQualified',qualified=True,
  starting_head='39a63034f6dd7c8da3e1d1b49b5718a73d2f1d9d',ending_head='Commit containing this report',
  task='CompleteV4OptimizedSealedL5ToL6PublicationControl',production_unchanged=True,scientific=seal['scientific'],content=seal['content'],candidate_access=seal['access'],current_production_access=current['reference']['root'],
  construction_boundary='Allowed pre-publication scientific payloads copied with source-token preservation checks. Only host-access base_root rebound; all other plan fields exact. Final target publication outputs blocked until seal.',
  staging_incident=dict(first_attempt='Hardlinks changed NTFS change tokens, without changing scientific bytes. First sealed candidate not repaired; independent copy-staging control rerun.',failure=file_hash(V4/'results/optimized_l6_publication_control/STAGING_TOKEN_FAILURE.json'),production_reauthentication=repair),
  forbidden_oracle_reads=0,seal_time=seal['sealed_at'],oracle_unlock=oracle['oracle_unlock'],oracle={k:v for k,v in oracle.items() if k!='reads'},
  phases={k:{a:b for a,b in v.items() if a!='reads'} for k,v in phases.items()},normal_assembly_publication_io=totals,
  index_baseline=dict(rows=3576991,wall=69.6198698,read_bytes=48940535872,write_bytes=17934304828,peak_rss=185487360),
  measured_index_write_reduction=17934304828-index['metrics']['io']['write_bytes'],
  historical=dict(assembly_writes=690408544,publication_writes=19459242659,complete_normal_writes=20539570632),
  attribution=dict(publication_writes=phases['PUBLISH']['metrics']['io']['write_bytes'],index_writes=index['metrics']['io']['write_bytes'],incidence_writes=phases['INCIDENCE']['metrics']['io']['write_bytes'],remaining_publication_writes=phases['PUBLISH']['metrics']['io']['write_bytes']-accounted,
   limitation='Remaining traffic includes quality SQLite page churn, input/metadata serialization, binding storage and transaction staging. No per-file kernel trace exists. Index insertion/commit counters include journal traffic; main/journal split is not separately measured. No new complete-L6 total is claimed.'),
  assembly=read(OUT/'ASSEMBLY.json'),publication=read(OUT/'PUBLICATION.json'),reopen={k:v for k,v in read(OUT/'REOPEN_CONTROL.json').items() if k!='reads'},deep={k:v for k,v in deep.items() if k!='reads'},
  safety={k:v for k,v in read(OUT/'SAFETY_CONTROL.json').items() if k!='reads'},stale=read(OUT/'STALE_BASE_CONTROL.json'),
  tests=(OUT/'TESTS.txt').read_text(),whitespace=(OUT/'WHITESPACE.txt').read_text(),
  storage=dict(start_free=read(OUT/'PREFLIGHT.json')['free'],end_free=shutil.disk_usage(OUT).free,logical=sum(r['bytes'] for r in rows),physical=sum(r['cluster_bytes'] for r in unique.values()),new_unshared=sum(r['cluster_bytes'] for r in unique.values() if r['links']==1),shared=sum(r['cluster_bytes'] for r in unique.values() if r['links']>1),sampled_peak=read(OUT/'STORAGE_SNAPSHOT.json')['physical'],continuous_peak_measured=False,cleanup=read(OUT/'CLEANUP.json'),retired_duplicate_control=read(OUT/'RETIRED_CONTROL_DUPLICATES.json'),historical_deletions=0),
  implementation=sources,architecture_identity=identity(sources),unrelated_entries=155,unrelated_status_preserved=True,v3_entries_checked=len(v3),v3_modifications=0,
  L6_admission=False,L6_forecast=False,L6_geometry_generation=False,L7_admission=False,L7_forecast=False,L7_generation=False,core=False,
  concerns=['Quality publication still contributes unlocalized write traffic.','Control reuses already-qualified scientific inputs; it does not qualify another physical advancement.','Future advancement must explicitly supply authenticated incidence metadata; no legacy scan fallback is enabled.'],
  recommended_next_task='QualifyV4CapsuleLayers7To8IncrementalEndToEndBounded, with independent per-layer admission, certification, publication, restore, retention and stop on failure.')
 report['identity']=identity(report);durable_json(OUT/'REPORT.json',report)
 compact=['REPORT.json','PREFLIGHT.json','ACCESS_REBIND.json','STAGE_MEASURED.json','ASSEMBLY_MEASURED.json','PUBLISH_MEASURED.json','INDEX_MEASURED.json','INCIDENCE_MEASURED.json','ASSEMBLY.json','PUBLICATION.json','REFERENCE.json','REOPEN_CONTROL.json','DEEP_CONTROL.json','SEAL_CONTROL.json','ORACLE_UNLOCK.json','ORACLE_CONTROL.json','SAFETY_CONTROL.json','STALE_BASE_CONTROL.json','CLEANUP.json','RETIRED_CONTROL_DUPLICATES.json','FINAL_STORAGE.json','SQL_COMMITS.json','TESTS.txt','WHITESPACE.txt']
 authority=dict(classification=report['classification'],qualification=report['identity'],scientific=seal['scientific'],content=seal['content'],candidate_access=seal['access'],incidence_root=seal['incidence_root'],architecture=report['architecture_identity'],production_unchanged=True,files={n:file_hash(OUT/n) for n in compact})
 authority['identity']=identity(authority);durable_json(OUT/'AUTHORITY.json',authority)
 print(json.dumps(dict(classification=report['classification'],qualification=report['identity'],report_sha256=file_hash(OUT/'REPORT.json'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),architecture=report['architecture_identity'],storage=report['storage']),indent=2))

if __name__=='__main__':main()
