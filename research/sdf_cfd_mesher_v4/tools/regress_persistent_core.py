"""Bounded exact C1 regression after opt-in prism infrastructure changes."""
from pathlib import Path
import shutil
import numpy as np
import capsule_user_domain_handoff as h
import capsule_core_control_audit as audit

OUT=Path(__file__).resolve().parents[1]/'results/persistent_publication_optimization'
SEALED=OUT.parent/'capsule_c1_core_qualification'

if __name__=='__main__':
    folder=OUT/'raw_persistent_optimized_core';audit.OUT=folder;tick=h.measured()
    source=SEALED/'TERMINAL_PRISMS.npz'
    assert h.file_hash(source)=='d627423b464e4c968fd073c9c0f6829bcd1ae752d64d1ec264c897517820f470'
    shutil.copyfile(source,folder/'TERMINAL_PRISMS.npz')
    audit.audit('C1','NETGEN.npz');audit.outer_quality('C1')
    checks={}
    for name in ('NETGEN_AUDIT.json','OUTER_QUALITY.json'):
        actual=h.read(folder/'C1'/name);expected=h.read(SEALED/'C1'/name)
        for value in (actual,expected):
            value.pop('seconds',None)
            if 'exact_jacobians' in value:value['exact_jacobians'].pop('seconds',None)
        checks[name]=actual==expected
    records=h.read(OUT.parent/'prism_assembly_optimization/CORE_CONTROL_EVIDENCE.json')['files']
    for name in ('CORE.npz','NETGEN.npz','OUTER.npz','FINAL_CORE_OWNERS.npy'):
        reference=next(r for r in records if r['path'].replace('\\','/').endswith('/C1/'+name))
        checks[name]=h.file_hash(folder/'C1'/name)==reference['sha256']
    checks['callback_binary64']=np.array_equal(np.load(folder/'QUERIES.npy'),np.load(OUT.parent/'production_hotpath_optimization/CORE_QUERIES.npy'))
    h.durable_json(OUT/'CORE_REGRESSION.json',dict(passed=all(checks.values()),checks=checks,metrics=h.metrics(*tick)))
    if not all(checks.values()):raise ValueError(checks)
