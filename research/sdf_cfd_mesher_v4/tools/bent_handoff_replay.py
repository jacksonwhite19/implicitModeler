"""Replay diagnostics and the rejected patch; discard only byte-bound scratch."""
import json,time,os,shutil
from pathlib import Path
from bent_handoff_inputs import OUT
from fixed_core_study import save
from sdf_cfd_mesher_v4.common.authority import file_hash
import bent_handoff_study as study
import bent_handoff_patch_trial as patch

def run():
    tick=time.perf_counter();dest=OUT/'replay_scratch';dest.mkdir(exist_ok=False)
    inputs=('SEALED_GEOMETRY.npz','SOURCE.json','CELL_LINEAGE.json','SCHEDULE.json')
    for name in inputs:os.link(OUT/name,dest/name)
    study.OUT=dest;patch.OUT=dest
    study.stack();study.correlations();patch.run()
    scientific=('LAYER_STATISTICS.json','EDGE_LINEAGE.json','MAX_EDGE.json','STACK_SUMMARY.json','PRE_CORE_HANDOFF.json',
                'FAILURE_CONTROL_MAP.json','DIAGNOSTIC_CORE_QUALITY.json','CORRELATION.json','PATCH_QUALITY.json','PATCH_BASELINE_QUALITY.json',
                'terminal_edges.vtk','terminal_faces.vtk','core_failures.vtk','overlap.vtk')
    checks={name:file_hash(OUT/name)==file_hash(dest/name) for name in scientific}
    a=json.loads((OUT/'PATCH_RESULT.json').read_text());b=json.loads((dest/'PATCH_RESULT.json').read_text())
    a.pop('seconds');b.pop('seconds');checks['PATCH_RESULT_without_clock']=a==b
    assert all(checks.values()),checks
    files=[dict(path=str(p),bytes=p.stat().st_size,sha256=file_hash(p),replacement=str(OUT/p.name) if (OUT/p.name).exists() else 'REPLAY.json timing',
                reason='Verified deterministic replay duplicate or replay-only timing; not referenced by any accepted authority') for p in dest.iterdir() if p.is_file()]
    record=dict(passed=True,checks=checks,seconds=time.perf_counter()-tick,logical_scratch_bytes=sum(r['bytes'] for r in files),
        hardlinked_input_bytes=sum((dest/n).stat().st_size for n in inputs),files=files)
    save(OUT/'REPLAY.json',record)
    # Resolve containment before removal; no wildcard, no cross-shell deletion.
    target=dest.resolve();assert target.parent==OUT.resolve() and target.name=='replay_scratch'
    for r in files:
        p=Path(r['path']);assert p.resolve().parent==target and file_hash(p)==r['sha256']
        p.unlink()
    target.rmdir()
    save(OUT/'REPLAY_CLEANUP.json',dict(deleted=files,bytes_deleted=record['logical_scratch_bytes'],
         uniquely_allocated_payload_bytes=record['logical_scratch_bytes']-record['hardlinked_input_bytes']))
    print(json.dumps({k:v for k,v in record.items() if k!='files'}),flush=True)

if __name__=='__main__':run()
