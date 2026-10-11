"""Read-only receipt-boundary diagnosis after the stopped assembly attempt."""
import json,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import psutil
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs
from composable_state import CommittedState
from sealed_front_connection import load_case
from first_layer_receipt_connection import validate_boundary,validate_cell_dependencies,validate_absent_final_receipts
from contextlib import ExitStack
import immutable_state_access as access


def run():
    start=time.perf_counter();read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
    original=V4/'results/authorized_continuation/encoding_resume/L2'
    evidence=read(V4/'results/authorized_continuation/EVIDENCE.json')
    records={str((ROOT/r['path']).resolve()):r for r in evidence['files']}
    with ExitStack() as held:
        for name in ('AFFECTED_INCIDENCE.json','GEOMETRY_DELTA.json','CHANGED_INPUTS.json'):
            p=original/name;held.enter_context(access.guard(p))
            if file_hash(p)!=records[str(p)]['sha256']:raise ValueError('ChangedRetainedCandidate')
        inc=read(original/'AFFECTED_INCIDENCE.json');delta=read(original/'GEOMETRY_DELTA.json')
        assert inc['passed']
        prior=[r for r in inc['cells'] if str(r['id']) not in delta['cells']]
        reference=read(V4/'results/sealed_front_connection/WING_REFERENCE.json')
        # Wall-authority cycles are oriented identity keys, not the original
        # numeric cell-template ordering. Use the sealed surface's exact order.
        case=read(V4/'results/sealed_front_connection/CASES.json')['wing']
        wall_coordinates,triangles,_,_=load_case(case)
        with CommittedState(reference) as base:
            wall=base['wall'];n=len(wall_coordinates)
            vertices=base['vertices'];cells=base['cells'];faces=base['faces']
            adapter=SimpleNamespace(committed_prefix=np.array([[vertices[f'layer:{k}:source:{i}'] for i in range(n)] for k in range(2)]),
                topology=SimpleNamespace(triangles=triangles))
            print('authenticated L1 geometry view',time.perf_counter()-start,flush=True)
            boundary=validate_boundary(base,adapter,prior)
            print('aggregate verified',time.perf_counter()-start,flush=True)
            old_inputs=geometry_inputs(dict(vertices=vertices,cells=cells,faces=faces,wall=wall,authority=base['authority']))
            print('dependency identities',time.perf_counter()-start,flush=True)
            validate_absent_final_receipts(base)
            checked=0;absent=len(prior);actual_neighbors=0;deps=0
            for r in prior:
                key=str(r['id']);receipt=base['quality']['cell:'+key]
                validate_cell_dependencies(key,receipt,cells[key],old_inputs);checked+=1;deps+=len(receipt['inputs'])
                top=base['prism_front'][cells[key]['source']]
                updated=delta['faces'][top]
                if key not in updated['owners'] or len(updated['owners'])!=2:raise ValueError('MissingActualTopNeighbor')
                neighbor=next(k for k in updated['owners'] if k!=key)
                if neighbor not in delta['cells']:raise ValueError('WrongActualNeighbor')
                actual_neighbors+=1
            base.check()
            result=dict(boundary=boundary,passed=True,prior_cells=checked,dependency_entries=deps,
                prospective_receipts_preserved=True,new_receipts_written=0,missing_old_final_receipts=absent,
                actual_L1_L2_neighbors=actual_neighbors,minimum_actual_determinant=min(r['determinant'] for r in prior),
                determinant_violations=sum(r['determinant']<.0011 for r in prior),
                scope='Read-only diagnosis of sealed L1 and retained L2 incidence; not an assembly retry or transition pass',
                seconds=time.perf_counter()-start,peak_working_set=psutil.Process().memory_info().peak_wset)
            durable_json(V4/'results/first_layer_receipt_connection/READ_ONLY_RECEIPT_CHECK.json',result)
            print(json.dumps(result,indent=2))
if __name__=='__main__':run()
