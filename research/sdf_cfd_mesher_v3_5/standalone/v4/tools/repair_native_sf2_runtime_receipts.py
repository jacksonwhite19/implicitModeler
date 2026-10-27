"""Revalidate fresh geometry after a numerical-runtime binding defect.

No reference coordinates or conditioning values are construction inputs.
Previously certified coordinates and exact proofs remain immutable.
"""
import json,time,sys
import numpy as np
from qualify_native_sf2_manifest import OUT
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeForecast
from sdf_cfd_mesher_v4.advancing_front.native_prism_entities import write_array
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer
from sdf_cfd_mesher_v4.advancing_front.validation_evidence import make_receipt
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json
from sdf_cfd_mesher_v4.common.authority import array_hash

def run():
    assert np.__version__=='1.26.4'
    start=time.perf_counter();path=OUT/'fresh';store=NativeCheckpointStore(path/'checkpoints')
    state=store.restart();forecast=NativeForecast(state,store.objects);b=forecast.backing
    old=dict(state['quality']);quality={};previous=None;records=[]
    for layer in range(1,b.layer+1):
        result=evaluate_layer(forecast.top,b.coordinates(layer-1),b.coordinates(layer),layer,previous)
        previous=result['cells'];row=old[f'layer:{layer}'];receipt=row['receipt']
        conditioning={k:result['cells'][k] for k in ('sigma','G_bound','G_bound_available')}
        hashes={f:{k:array_hash(v) for k,v in result[f].items() if isinstance(v,np.ndarray)} for f in ('cells','sides','caps')}
        metric_path=path/f'LAYER_{layer:02}_METRIC_ARRAY_HASHES.json'
        before=json.loads(metric_path.read_text())
        rebuilt=make_receipt(forecast.top,receipt['inputs'],result['gate'],receipt['sdf'],receipt['exact'],state['certificate'][f'layer:{layer}']['ledger'],conditioning)
        quality[f'layer:{layer}']=dict(receipt=rebuilt,conditioning={k:write_array(store.objects,v) for k,v in conditioning.items()})
        records.append(dict(layer=layer,before=before,after=hashes,old_receipt=receipt['receipt_hash'],new_receipt=rebuilt['receipt_hash']))
        atomic_json(metric_path,hashes)
    state['quality']=quality;digest=store.publish(state)
    seconds=time.perf_counter()-start
    atomic_json(path/'RUNTIME_RECEIPT_REPAIR.json',dict(cause='NumPy 2.4.0 versus qualified NumPy 1.26.4 LAPACK rounding',
        executable=sys.executable,numpy=np.__version__,seconds=seconds,records=records,scientific=digest,
        geometry_changed=False,proofs_recomputed=0,reference_values_used=False))
    execution=json.loads((path/'EXECUTION.json').read_text());execution['scientific_seconds']+=seconds
    atomic_json(path/'EXECUTION.json',execution)
    print('REVALIDATED',seconds,flush=True)

if __name__=='__main__':run()
