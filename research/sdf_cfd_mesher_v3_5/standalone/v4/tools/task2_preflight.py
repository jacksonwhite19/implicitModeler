"""Read-only bounded authority audit; writes only the new Task-2 evidence."""
from pathlib import Path
import sys,json,subprocess,datetime,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
from sdf_cfd_mesher_v4.common.authority import REPO,file_hash
OUT=ROOT/'results/task2';OUT.mkdir(exist_ok=True)
EXPECTED={
 'authorities/V3_SOURCE_FORK_MANIFEST.json':'949faff4837568fd67449758d34dd15667ab9c32a92ca80c8439934ad4eac5cb',
 'authorities/DATA_AUTHORITIES.json':'20f1aeba70dc21dd3e4d8fe19bdb8999d75d5a73ce3a0231f255fe5595838241',
 'authorities/V4_SETUP_AUTHORITY.json':'190d04c817c6164f7a004adfa8f257f0d61cf55c19d9b0c970a85493b868d87e',
 'TASK1_REPORT.md':'a710d184a386d609c39b7199e6c5fde229e71eb4431895aa50b870a0057870f9',
 'authorities/V4_TASK1_AUTHORITY.json':'a84200a78b7c5c73d03f6b9a53d3e61dfdfb1d72dc1a8260292fe6c5e11573a7'}
def git(*args):return subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()
def audit():
 t=time.perf_counter(); rows=[]
 def check(p,sha,role):
  actual=file_hash(p);rows.append(dict(path=p.relative_to(REPO).as_posix(),expected=sha,actual=actual,role=role));assert actual==sha,rows[-1]
 for p,h in EXPECTED.items():check(ROOT/p,h,'frozen V4 authority')
 a=json.loads((ROOT/'authorities/V4_TASK1_AUTHORITY.json').read_text())
 for p,h in a['source_hashes'].items():check(ROOT/p,h,'Task-1 implementation')
 for p,h in a['evidence_hashes'].items():check(ROOT/p,h,'Task-1 evidence')
 for p,r in a['forecast_archives'].items():check(ROOT/p,r['sha256'],'Task-1 archive')
 check(ROOT/'authorities/TASK1_SOURCE_BINDING.json',a['source_binding_sha256'],'Task-1 binding')
 for r in json.loads((ROOT/'authorities/V3_SOURCE_FORK_MANIFEST.json').read_text())['files']:
  check(REPO/r['original_path'],r['original_sha256'],'bounded original source');check(ROOT/r['destination'],r['original_sha256'],'frozen copy')
 for r in json.loads((ROOT/'authorities/TASK1_SOURCE_BINDING.json').read_text())['files']:check(REPO/r['path'],r['sha256'],'Task-1 original binding')
 for r in json.loads((ROOT/'authorities/DATA_AUTHORITIES.json').read_text())['references']:
  if r['role'] in ('common geometry','SF2 surface','certificate integration authority'):check(REPO/r['path'],r['sha256'],r['role'])
  if r['role']=='SF2 accepted polyMesh':check(REPO/r['identity_record_path'],r['identity_record_sha256'],'accepted mesh identity record; mesh not read')
 return dict(passed=True,rows=rows,seconds=time.perf_counter()-t)
if __name__=='__main__':
 branch=git('branch','--show-current');head=git('rev-parse','HEAD')
 assert branch=='feature/sdf-cfd-mesher-v4-advancing-front' and head=='50b9959bc37d9e101c6ac632eb6783f20ae2067b'
 assert not git('diff','--cached','--name-only'),'Unrelated staged work'
 r=audit();r.update(branch=branch,starting_head=head,started_utc='2026-09-16T13:27:27Z',audit_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),starting_status=git('status','--porcelain'),maximum_worker_rss=12884901888,maximum_heavy_workers=1)
 (OUT/'PREFLIGHT.json').write_text(json.dumps(r,indent=2,sort_keys=True)+'\n')
 print(json.dumps(dict(passed=r['passed'],checks=len(r['rows']),seconds=r['seconds'],head=head)))
