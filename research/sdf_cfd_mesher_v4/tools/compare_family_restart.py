"""Compare native committed states after independent transaction continuation."""
import sys,json
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'src')]
from run_geometry_family import OUT,remaining_columns,g
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from summarize_geometry_family import committed_history


def event_sequence(folder):
    return [row['event']['event'] for row in committed_history(folder,'OPERATIONS.json')
            if row['operation']=='RollbackRemesh']


def run(name,primary='T',replay='X'):
    root=OUT/name
    a=NativeCheckpointStore(root/primary/'checkpoints').restart()
    b=NativeCheckpointStore(root/replay/'checkpoints').restart()
    first_events=event_sequence(root/primary);second_events=event_sequence(root/replay)
    record=dict(primary=scientific_hash(a),replay=scientific_hash(b),primary_label=primary,replay_label=replay,
        exact=scientific_hash(a)==scientific_hash(b),wall_exact=a['wall']==b['wall'],
        first_layer_exact=a['first_layer']==b['first_layer'],event_order_exact=first_events==second_events,
        primary_event_sequence=first_events,replay_event_sequence=second_events,
        pending_active_columns=remaining_columns(a),scope='Independent continuation from a committed checkpoint between transactions')
    g.save(root/'DETERMINISM.json',record)
    assert all(record[k] for k in ('exact','wall_exact','first_layer_exact','event_order_exact'))


if __name__=='__main__':run(*sys.argv[1:])
