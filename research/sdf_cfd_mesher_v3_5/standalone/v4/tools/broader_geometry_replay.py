"""Bounded reconstruction and native checkpoint replay of a cohort shell."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import json,time
import numpy as np
import generalize_local_remesh as g
from broader_geometry_cohort import OUT,CASES
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,cycle
from sdf_cfd_mesher_v4.core.interface import extract


def run(folder):
    tick=time.perf_counter();store=NativeCheckpointStore(folder/'checkpoints');state=store.restart()
    # Reconstruct the qualified aliased prism/transition topology from its
    # source-keyed committed coordinates and variable local layer depths.
    ids=sorted(state['wall']['vertices'],key=int)
    assert ids==list(map(str,range(len(ids))))
    wall=np.array([state['wall']['vertices'][k] for k in ids])
    tri=np.array([state['wall']['triangles'][k] for k in sorted(state['wall']['triangles'],key=int)],dtype=np.int64)
    depth=np.zeros(len(wall),dtype=int)
    for key in state['vertices']:
        _,layer,_,source=key.split(':'); depth[int(source)]=max(depth[int(source)],int(layer))
    fronts=[wall.copy() for _ in range(max(depth))]
    for key,point in state['vertices'].items():
        _,layer,_,source=key.split(':')
        if int(layer):fronts[int(layer)-1][int(source)]=point
    points,cells,outer=g.s.construct(wall,tri,fronts,depth)
    expected={int(k):v for k,v in state['cells'].items()}
    for c in cells:
        reference=expected[c['id']]
        keys=[f'layer:{v//len(wall)}:source:{v%len(wall)}' for v in c['vertices']]
        faces=[[f'layer:{v//len(wall)}:source:{v%len(wall)}' for v in f] for f in c['faces']]
        # Wall authority stores cyclic triangles. Retain multiplicity and exact
        # oriented face perimeters; a cyclic starting vertex is not new topology.
        assert c['kind']==reference['kind'] and sorted(keys)==sorted(reference['vertices'])
        assert sorted(list(cycle(f)) for f in faces)==reference['faces']
        assert all(np.array_equal(points[v].view('u8'),np.asarray(state['vertices'][k]).view('u8')) for v,k in zip(c['vertices'],keys))
    assert len(cells)==len(expected)
    _,_,_,interface=extract(state)
    preserved=json.loads((folder/'interface/INTERFACE.json').read_text())
    assert interface==preserved
    replay=scientific_hash(store.restart())==scientific_hash(state)
    assert replay
    local=state.get('local_schedule')
    if local is None:
        # Early nominal mixed checkpoints bind the immutable schedule by hash.
        # Resolve that authority; never synthesize rounded schedule values.
        schedule=g.q.load_schedule()
        assert g.q.array_hash(schedule.targets)==state['authority']['schedule']
        local=dict(nominal=schedule.targets.tolist(),actual={},default='nominal')
    targets=local['nominal']
    g.save(folder/'GEOMETRY_AUTHORITY.json',dict(wall_coordinates=g.q.array_hash(wall),
        wall_topology=g.q.array_hash(tri),schedule=targets,requested_layers=len(targets),h1=targets[0],
        adjusted_schedule=local,layer_distribution=dict(g.t.Counter(str(c['last_prism_layer']) for c in state['columns'].values())),
        source_context=state['authority'],wall_identity=interface['wall_identity'],first_layer_identity=interface['first_layer_identity']))
    g.save(folder/'BOUNDED_GEOMETRY_REPLAY.json',dict(passed=True,scientific=scientific_hash(state),
        reconstructed_cells=len(cells),connectivity_exact=True,coordinates_exact=True,
        interface_exact=True,restart_exact=replay,seconds=time.perf_counter()-tick,
        scope='Source-keyed committed geometry reconstruction and complete native restart; not a fresh optimization search'))


if __name__=='__main__':run(Path(sys.argv[1]))
