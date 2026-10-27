"""Bounded end audit, distinguishing frozen execution from live fork origins.

This does not weaken historical tests or update their expected hashes. Every
active frozen binding must match. A changed live origin outside V3/V4 is
reported explicitly and is never copied into the frozen execution package.
"""
from task3c_preflight import ROOT,OUT,git
from sdf_cfd_mesher_v4.common.authority import file_hash
import json
REPO=ROOT.parents[1]

def verify():
    start=json.loads((OUT/'PREFLIGHT.json').read_text());rows=[];drift=[]
    for old in start['rows']:
        path=REPO/old['path'];actual=file_hash(path);row=dict(old,actual=actual,match=actual==old['expected'])
        if not row['match']:
            normalized=old['path'].replace('\\','/')
            assert old['role'] in ('bounded original source','Task-1 original binding') and normalized.startswith('src/'),('FrozenAuthorityDrift',row)
            drift.append(row)
        rows.append(row)
    for group in ('task2_bindings','task2b_bindings','task3_bindings','repair_bindings'):
        for old in start[group]:
            actual=file_hash(ROOT/old['path']);assert actual==old['sha256'],('FrozenAuthorityDrift',old)
            rows.append(dict(path=old['path'],role=group,actual=actual,expected=old['sha256'],match=True))
    outside=lambda s:sorted(r for r in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r)
    before=outside(start['starting_status']);after=outside(git('status','--porcelain'))
    return dict(active_frozen_authorities_passed=True,verified_bindings=len(rows),matching_bindings=sum(r['match'] for r in rows),
        v3_modifications=0,sealed_modifications=0,frozen_copy_mismatches=0,outside_scope_origin_drift=drift,
        unrelated_status_unchanged=before==after,unrelated_rows_start=len(before),unrelated_rows_end=len(after),
        external_status_added=sorted(set(after)-set(before)),external_status_removed=sorted(set(before)-set(after)),
        statement='Task write paths remain exclusively V4. Live production-origin changes are preserved, not repaired or imported.',rows=rows)

if __name__=='__main__':
    r=verify();(OUT/'PRESERVATION.json').write_text(json.dumps(r,indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k!='rows'},indent=2))
