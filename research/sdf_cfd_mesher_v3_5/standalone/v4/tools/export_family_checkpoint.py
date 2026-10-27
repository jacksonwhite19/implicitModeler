"""Export a stopped committed checkpoint without declaring its schedule complete."""
import sys,json
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'src')]
from collections import Counter
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs
from qualify_broader_geometry import export
from run_geometry_family import OUT,remaining_columns,g


def run(name,label='Q'):
    folder=OUT/name/label;store=NativeCheckpointStore(folder/'checkpoints');state=store.restart()
    inputs=geometry_inputs(state);proofs=0
    for _,receipt in state['certificate'].items():
        assert all(inputs[k]==v for k,v in receipt['inputs'].items())
        assert receipt['value']['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
        proofs+=1
    record=dict(scope='Exact committed prefix only; requested schedule not qualified',case=name,
        scientific=scientific_hash(state),cells=dict(Counter(c['kind'] for c in state['cells'].values())),
        columns=dict(Counter(c['status'] for c in state['columns'].values())),remaining_active_columns=remaining_columns(state),
        local_layers={k:c['last_prism_layer'] for k,c in state['columns'].items()},
        remesh_events=len(state['events']),proofs=proofs,interface=export(state,folder/'interface'),
        restart_exact=scientific_hash(store.restart())==scientific_hash(state))
    g.save(folder/'PARTIAL_SHELL_RESULT.json',record)

if __name__=='__main__':run(*sys.argv[1:])
