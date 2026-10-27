"""Seal a partial study honestly; this is not a production qualification."""
from pathlib import Path
import json,gzip,os,subprocess,shutil
import immutable_state_access as a
import recursive_epoch_recovery as recovery
from composable_state import CommittedState
from path_preserving_deduplication import file_hash,durable_json,metadata,physical_key
from sdf_cfd_mesher_v4.common.authority import identity
V4=Path(__file__).resolve().parents[1];OUT=V4/'results/l6_write_pair_scaling_resume'
OWN=['tools/composable_assembly.py','tools/composable_publication.py','tools/incremental_pair_storage.py','tools/incremental_pair_storage.md','tools/append_pair_incidence.py','tools/qualify_pair_accounting.py','tools/benchmark_pair_accounting.py','tools/verify_pair_incidence.py','tools/seal_pair_scaling_partial.py','tests/test_incremental_pair_storage.py']
def main():
 pre=a.read(OUT/'PREFLIGHT.json')
 for p,h in pre['protected'].items():assert file_hash(p)==h,p
 assert a.read(OUT.parent/'l6_write_pair_scaling/AUTHORITY.json')['report_identity']=='93c68c7a92ed913dc953d4df89568914cad937aa642972f7f7b4027eae8bbc35'
 record=recovery.discover(OUT.parent/'recursive_epoch_recovery/access')
 with CommittedState(record['reference']) as current:assert current.layer==6 and current.content_root=='15259f4e506a7ae574666283b40d33428f788de8f9d5f3e29314d15ef27178f0'
 v3pre=a.read(OUT.parent/'recursive_epoch_recovery/PREFLIGHT.json')
 with gzip.open(OUT.parent/'recursive_epoch_recovery'/v3pre['v3_inventory']['file'],'rt',encoding='utf8') as f:old=json.load(f)
 now={}
 for directory,dirs,files in os.walk('research/sdf_cfd_mesher_v3',followlinks=False):
  for n in files:
   p=Path(directory)/n;s=p.lstat();now[str(p)]=[s.st_size,s.st_mtime_ns]
 assert now==old
 status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
 names=[('research/sdf_cfd_mesher_v4/'+n).encode() for n in OWN]+[b'research/sdf_cfd_mesher_v4/results/l6_write_pair_scaling_resume']
 untouched=b'\0'.join(x for x in status.split(b'\0') if x and not any(n in x for n in names))+b'\0'
 assert untouched.hex()==pre['status_hex']
 rows=[metadata(p) for p in OUT.rglob('*') if p.is_file()];unique={physical_key(r):r for r in rows}
 historical=a.read(OUT.parent/'l6_write_pair_scaling/PREFLIGHT.json')['historical_phases']
 sources={n:file_hash(V4/n) for n in OWN};architecture=identity(sources)
 result=dict(classification='V4IncrementalPairAccountingVerifiedWritePathControlPendingInputBoundary',qualified=False,
  starting_head=pre['head'],scientific=record['scientific'],content=record['content'],current_reference=record['reference'],
  historical_write_bytes={k:v['io']['write_bytes'] for k,v in historical.items()},historical_total_writes=sum(v['io']['write_bytes'] for v in historical.values()),
  write_fix=dict(integrated=True,production_size_control_run=False,cache_policy='max(previous_capacity, ceil(measured_allocated_pages * total_rows / inserted_rows)) after each existing 10000-row batch',durability='Same unique B-trees/order; synchronous FULL; journaling/spill enabled; integrity/count checks; fsync',small_exact_byte_equality_test=True,improved_total_write_traffic=None,unattributed='Historical per-file attribution unavailable; no subtraction of non-equivalent control traffic from historical totals'),
  baseline=a.read(OUT/'PAIR_BASELINE.json'),optimized=a.read(OUT/'PAIR_OPTIMIZED.json'),enrollment=a.read(OUT/'INCIDENCE_ENROLLMENT.json'),deep=a.read(OUT/'INCIDENCE_DEEP.json'),recursive=a.read(OUT/'RECURSIVE_SCALING.json'),append=a.read(OUT/'APPEND_CONTROL.json'),
  control_scope='L5-only pair accounting, native L4/incremental L5 lookup, and L4-to-L5 derived-index append. No complete L6 candidate was constructed or compared.',
  production_scientific_equivalence_verified=False,oracle_target_data_used_for_construction=False,
  pending='Clarify whether authenticated pre-publication L6 proof/quality deltas are permitted as storage-control inputs. Otherwise an independently recomputed L5-to-L6 control must resolve its required admission lookahead under the no-L7-forecast instruction.',
  safety='117 focused/regression tests; existing transaction/recursive-access mechanics unchanged. Complete optimized L6 transaction and recovery control remains pending.',tests=(OUT/'TESTS_FINAL.txt').read_text().strip(),
  storage=dict(start_free=pre['free'],end_free=shutil.disk_usage(OUT).free,retained_logical=sum(r['bytes'] for r in rows),retained_physical=sum(r['cluster_bytes'] for r in unique.values()),files=len(rows),objects=len(unique),continuous_peak_measured=False,cleanup=a.read(OUT/'CLEANUP_MANIFEST.json'),historical_deletions=0),
  implementation=sources,architecture_identity=architecture,unrelated_entries=155,unrelated_status_exact=True,v3_entries_checked=len(now),v3_modifications=0,
  L7_admission=False,L7_forecast=False,L7_generation=False,core=False,
  next_task='Complete the exact L5-to-L6 optimized publication control after resolving its input boundary')
 result['identity']=identity(result);durable_json(OUT/'REPORT.json',result)
 files={p.name:file_hash(p) for p in OUT.iterdir() if p.suffix in ('.json','.txt') and p.name!='AUTHORITY.json'}
 authority=dict(classification=result['classification'],qualified=False,report_identity=result['identity'],architecture_identity=architecture,files=files,scientific=record['scientific'],content=record['content'])
 authority['identity']=identity(authority);durable_json(OUT/'AUTHORITY.json',authority)
 print(json.dumps(dict(classification=result['classification'],report_identity=result['identity'],report_sha256=file_hash(OUT/'REPORT.json'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),architecture_identity=architecture,storage={k:v for k,v in result['storage'].items() if k!='cleanup'}),indent=2))
if __name__=='__main__':main()
