"""Recover and disclose a WindowsPath case-folding gap in the first inventory.

This cannot create a retroactive before-task content hash. It records existing
timestamps and two current full-content reads, without changing scientific data.
"""
import concurrent.futures
import json
from pathlib import Path
import sys
import time
import os
import gzip
from datetime import datetime
sys.dont_write_bytecode=True
from canonical_bridge_evidence import digest

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'research/sdf_cfd_mesher_v4/results/generic_canonical_bridge'
inventory=json.loads((OUT/'UNRELATED_INVENTORY_AFTER.json').read_text())
baseline_path=OUT/'UNRELATED_BEFORE.json'
baseline=json.loads(baseline_path.read_text() if baseline_path.exists() else gzip.decompress(baseline_path.with_suffix('.json.gz').read_bytes()))
names={r['path'].casefold():r['path'] for r in baseline['files']}
assert inventory['status_equal'] and not inventory['removed']
assert all(p.casefold() in names for p in inventory['added'])
start=time.perf_counter()
first_artifact=datetime.fromisoformat('2026-09-28T03:16:02.592634+00:00').timestamp()
def check(name):
    p=ROOT/name
    if p.is_symlink():
        target=os.readlink(p);s=p.lstat()
        return dict(path=name,baseline_casefold_partner=names[name.casefold()],kind='symlink',
                    bytes=s.st_size,target=target,two_reads_equal=target==os.readlink(p),
                    predates_task_artifact=s.st_mtime<first_artifact)
    s=p.stat();a=digest(p);b=digest(p)
    assert a==b,name
    return dict(path=name,baseline_casefold_partner=names[name.casefold()],bytes=s.st_size,
                sha256=a,two_reads_equal=True,mtime=s.st_mtime,
                predates_task_artifact=s.st_mtime<first_artifact)
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:rows=list(pool.map(check,inventory['added']))
result=dict(classification='InitialCaseSensitiveInventoryCoverageGapDisclosed',
            additional_names=len(rows),all_have_baseline_casefold_partner=True,
            all_predate_task_artifact=all(r['predates_task_artifact'] for r in rows),
            all_two_read_hashes_equal=True,bytes=sum(r['bytes'] for r in rows),
            exact_before_task_hash_claim=False,seconds=time.perf_counter()-start,files=rows,
            note='No added field generation inferred: all names are distinct case variants collapsed by initial WindowsPath set(). Missing initial hashes cannot be recovered retroactively.')
(OUT/'CASE_SENSITIVE_PRESERVATION.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='files'}))
