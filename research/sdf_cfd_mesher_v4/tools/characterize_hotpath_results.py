"""Qualification-only comparisons and bounded writer benchmark."""
import json
import numpy as np
from profile_meshing_hotpath import OUT,SEALED
import capsule_user_domain_handoff as h


def quality():
    import capsule_core_control_audit as audit
    audit.OUT=OUT/'raw_release_optimized_core'
    tick=h.measured();audit.prepare_prisms();result=audit.audit('C1','NETGEN.npz');audit.outer_quality('C1')
    expected=h.read(SEALED/'C1/NETGEN_AUDIT.json')
    for r in (result,expected):
        r.pop('seconds');r['exact_jacobians'].pop('seconds')
    checks=dict(quality=result==expected,
        outer_quality=h.read(audit.OUT/'C1/OUTER_QUALITY.json')==h.read(SEALED/'C1/OUTER_QUALITY.json'))
    for filename in ('QUERIES.npy','C1/CORE.npz','C1/OUTER.npz','C1/NETGEN.npz','C1/FINAL_CORE_OWNERS.npy'):
        a=np.load(OUT/'raw_release_baseline_core'/filename);b=np.load(audit.OUT/filename)
        checks[filename]=all(a[k].tobytes()==b[k].tobytes() for k in a.files) if hasattr(a,'files') else a.tobytes()==b.tobytes()
    h.durable_json(OUT/'CORE_REPLAY_EQUIVALENCE.json',dict(passed=all(checks.values()),checks=checks,metrics=h.metrics(*tick)))
    assert all(checks.values())


def writer():
    from path_preserving_deduplication import durable_json as old
    from optimized_durable_json import durable_json as new
    value=h.read(OUT/'baseline_prism/QUALITY_DELTA.json');records=[]
    for name,function in (('old',old),('new',new)):
        path=OUT/(name+'_json_control.json');tick=h.measured();function(path,value)
        records.append(dict(writer=name,metrics=h.metrics(*tick),bytes=path.stat().st_size,sha256=h.file_hash(path)))
    assert records[0]['sha256']==records[1]['sha256']
    h.durable_json(OUT/'JSON_WRITER_CONTROL.json',records)


if __name__=='__main__':
    import sys
    {'quality':quality,'writer':writer}[sys.argv[1]]()
