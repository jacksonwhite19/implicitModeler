"""Export the committed native interface without reconstructing prism tops."""
from pathlib import Path
import argparse
import gc
import json
import time
import numpy as np
from native_memory_artifacts import materialize
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore, scientific_digest
from sdf_cfd_mesher_v4.common.authority import array_hash


from sdf_cfd_mesher_v4.core.interface import extract


def owner_vertices(cell):
    """Export real cell topology, not the six-slot historical alias stencil."""
    vertices = list(dict.fromkeys(cell['vertices']))
    expected = {'prism': 6, 'pyramid': 5, 'tetrahedron': 4}[cell['kind']]
    if len(vertices) != expected:
        raise ValueError('OwnerCellTypeConnectivityMismatch')
    if cell['kind'] == 'pyramid':
        bases = [face for face in cell['faces'] if len(face) == 4]
        if len(bases) != 1:
            raise ValueError('PyramidBaseMissing')
        base = list(bases[0])
        vertices = base + [v for v in vertices if v not in base]
    return vertices


def main():
    p = argparse.ArgumentParser()
    p.add_argument('case', choices=['fresh', 'general_regression'])
    p.add_argument('output', type=Path)
    args = p.parse_args()
    start = time.perf_counter()
    with materialize(args.case) as source:
        checkpoint = source / 'checkpoints' if args.case == 'fresh' else source
        if not (checkpoint / 'CURRENT.json').exists():
            candidates = list(source.rglob('CURRENT.json'))
            if len(candidates) != 1:
                raise ValueError('AmbiguousNativeCheckpoint:' + str(candidates))
            checkpoint = candidates[0].parent
        store = NativeCheckpointStore(checkpoint)
        state = store.restart()
        points, offsets, connectivity, metadata = extract(state)
        args.output.mkdir(parents=True, exist_ok=True)
        for name, value in [('points', points), ('offsets', offsets), ('connectivity', connectivity)]:
            np.save(args.output / (name + '.npy'), value, allow_pickle=False)
        # Bounded interface owner geometry, not an expanded full shell copy.
        owner_cells = [state['cells'][key] for key in metadata['shell_owners']]
        owner_nodes = [owner_vertices(row) for row in owner_cells]
        owner_arities = sorted({len(row) for row in owner_nodes})
        for arity in owner_arities:
            selected = [i for i, row in enumerate(owner_nodes) if len(row) == arity]
            values = np.asarray([[state['vertices'][v] for v in owner_nodes[i]] for i in selected])
            np.save(args.output / f'owner_geometry_{arity}.npy', values)
            np.save(args.output / f'owner_faces_{arity}.npy', np.asarray(selected, dtype='<i8'))
        (args.output / 'OWNER_TYPES.json').write_text(json.dumps([row['kind'] for row in owner_cells])+'\n', encoding='utf8')
        (args.output / 'INTERFACE.json').write_text(json.dumps(metadata, sort_keys=True) + '\n', encoding='utf8')
        print(json.dumps(dict(faces=metadata['faces'], vertices=metadata['vertices'], arities=metadata['face_arities'], seconds=time.perf_counter()-start)), flush=True)
        del state, store
        gc.collect()


if __name__ == '__main__':
    main()
