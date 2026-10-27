"""One-time read-only import of qualified legacy receipts into compact memory.

Uses the same dependency and complete-coverage checks as the preserved native
bootstrap. No legacy checkpoint or raw receipt table is rewritten.
"""
import time,json
from compact_proof_rows import CompactProofRows
from native_proof_segments import SegmentProofRows
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key

def prepare(run):
    table=run.tx.current['certificate']
    if getattr(table,'proof_segments',None):return
    started=time.perf_counter();n=len(run.provider.x)
    def source(v):return f'layer:{v//n}:source:{v%n}'
    faces=[run.topology.owners[k][0][1] for k in sorted(run.topology.owners)]
    keys=[face_key(tuple(source(v) for v in f)) for f in faces];indices={k:i for i,k in enumerate(keys)}
    rows=CompactProofRows()
    for k,r in table.items():
        if identity(r['faces'])!=k or any(run.inputs.get(dep)!=h for dep,h in r['inputs'].items()):raise ValueError('BootstrapStaleReceipt')
        rows.append(dict(pair=sorted(indices[f] for f in r['faces']),**r['value']))
    rows.ordered_unique()
    if len(rows)!=len(table) or rows.summary()[1]:raise ValueError('BootstrapRejectedProofOutcome')
    converted=time.perf_counter()-started
    from audit_capsule_scaling import final_coverage
    coverage=final_coverage(run.points,faces,rows)
    run.face_keys=keys;run.proofs=SegmentProofRows([],rows)
    record=dict(passed=True,relations=len(rows),receipt_import_seconds=converted,total_seconds=time.perf_counter()-started,coverage=coverage,source_modified=False)
    (run.folder/'LEGACY_PROOF_IMPORT.json').write_text(json.dumps(record,indent=2))
    return record
