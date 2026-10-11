"""One-time explicit migration of a copied, verified native checkpoint."""
import json,time
from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs,face_key
from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import bootstrap
from sdf_cfd_mesher_v4.common.authority import identity
from compact_proof_rows import CompactProofRows
from native_proof_segments import persist_segment,add_dependency_index


def migrate(checkpoints,output):
    store=NativeCheckpointStore(checkpoints);start=time.perf_counter();state=store.restart()
    old=scientific_hash(state);table=state['certificate'];legacy=scientific_digest(table)
    inputs=geometry_inputs(state);phases=dict(verified_load=time.perf_counter()-start)
    print('BOOTSTRAP_STAGE '+json.dumps(phases),flush=True)
    tick=time.perf_counter()
    if not getattr(table,'proof_segments',None):
        # Match the production face ordering using source-local vertex IDs;
        # reject unsupported source encoding instead of inventing an ordering.
        n=len(state['wall']['vertices'])
        def index(v):
            parts=v.split(':')
            if len(parts)!=4 or parts[0]!='layer' or parts[2]!='source':raise ValueError('BootstrapRequiresProductionFaceOrder')
            return int(parts[1])*n+int(parts[3])
        keys=sorted(state['faces'],key=lambda k:tuple(sorted(index(v) for v in state['faces'][k]['vertices'])))
        indices={k:i for i,k in enumerate(keys)};rows=CompactProofRows()
        for key,receipt in table.items():
            if identity(receipt['faces'])!=key or any(inputs.get(k)!=v for k,v in receipt['inputs'].items()):
                raise ValueError('BootstrapStaleReceipt')
            rows.append(dict(pair=sorted(indices[f] for f in receipt['faces']),**receipt['value']))
        rows.ordered_unique()
        # A valid content hash alone does not prove complete coverage. An
        # unindexed input must establish the induction base before we grant
        # qualified_context. This is a one-time migration audit.
        from audit_capsule_scaling import final_coverage
        vertex_keys=sorted(state['vertices'],key=index)
        vertex_indices={key:i for i,key in enumerate(vertex_keys)}
        points=np.asarray([state['vertices'][key] for key in vertex_keys])
        faces=[tuple(vertex_indices[v] for v in state['faces'][key]['vertices']) for key in keys]
        audit_start=time.perf_counter()
        final_coverage(points,faces,rows)
        if rows.summary()[1]:raise ValueError('BootstrapRejectedProofOutcome')
        phases['unindexed_input_coverage']=time.perf_counter()-audit_start
        table.proof_segments=[persist_segment(rows,keys,store.objects)]
        del rows
    else:
        if table.qualified_context!=identity(inputs):raise ValueError('BootstrapStaleCoverage')
    table.proof_segments=[add_dependency_index(s,store.objects) for s in table.proof_segments]
    table.proof_objects=store.objects;table.qualified_context=identity(inputs)
    phases['proof_index_bootstrap']=time.perf_counter()-tick;tick=time.perf_counter()
    print('BOOTSTRAP_STAGE '+json.dumps(phases),flush=True)
    descriptor=bootstrap(table,store.objects)
    phases['root_bootstrap']=time.perf_counter()-tick;tick=time.perf_counter()
    print('BOOTSTRAP_STAGE '+json.dumps(phases),flush=True)
    new=store.publish(state)
    phases['publication']=time.perf_counter()-tick
    result=dict(old_scientific=old,new_scientific=new,legacy_receipt_digest=legacy,
                authority=descriptor,phases=phases,total_seconds=time.perf_counter()-start,
                count=len(table),full_rows_bootstrapped=len(table))
    Path(output).write_text(json.dumps(result,indent=2)+'\n')
    print('BOOTSTRAP '+json.dumps(result),flush=True)
    return result
