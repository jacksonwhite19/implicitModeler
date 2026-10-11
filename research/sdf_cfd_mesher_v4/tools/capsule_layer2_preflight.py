"""Read-only layer-2 restoration and admission audit. Never constructs geometry.

This is task evidence tooling, not a replacement production forecaster.
"""
import collections
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np
import psutil
from production_surface_authority import load_production_wall
from production_prism_sizing import read_request
from sdf_cfd_mesher_v4.common.authority import array_hash, file_hash, identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
OUT = V4 / 'results/capsule_advancing_front_layer2'
L1 = V4 / 'results/production_adaptive_capsule_first_layer'
SURFACE = V4 / 'results/production_adaptive_capsule_surface_foundation/fresh'
REQUEST = V4 / 'results/qualification_prism_progression_convention/REQUEST.json'
COMMITTED = '5b036bc4cd5180b9e4a7e3e6aafe57fe21c7b2ae1cae82cf3c42e446b1805fd0'
SURFACE_ID = '14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'
REQUEST_ID = 'ec28d584b04afb8bd4679d6719d0f600140b63dcff26a0cc50c2607537d33b91'


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, indent=2, allow_nan=False)+'\n')


def disk_audit():
    start = time.perf_counter()
    totals = collections.Counter(); largest = []; errors = []
    def error(e): errors.append(str(e))
    for base, dirs, files in os.walk(ROOT, onerror=error, followlinks=False):
        # Managed worktrees and nested checkouts are outside the task authority.
        dirs[:] = [d for d in dirs if not Path(base, d).is_symlink()
                   and Path(base, d) not in (ROOT/'.claude/worktrees', ROOT/'dual_contouring')]
        for name in files:
            p = Path(base, name)
            try: size = p.stat().st_size
            except OSError as e: error(e); continue
            totals['repository_accessible_logical_bytes'] += size
            if p.is_relative_to(V4):
                totals['v4_accessible_logical_bytes'] += size
                largest.append((size, p.relative_to(ROOT).as_posix()))
            if p.is_relative_to(L1): totals['retained_l1_bytes'] += size
    return dict(totals, free_bytes=shutil.disk_usage(ROOT).free,
                largest_v4_files=sorted(largest, reverse=True)[:10],
                errors=errors, seconds=time.perf_counter()-start,
                exclusions=['symlink directories', '.claude/worktrees', 'dual_contouring'])


def select_layer2(bound):
    if identity(bound['declaration']) != REQUEST_ID: raise ValueError('RequestIdentityMismatch')
    if bound['declaration']['height_scope'] != 'global': raise ValueError('GlobalHeightRequired')
    h = float(bound['increments'][1])
    if h.hex() != '0x1.6b844d56c388ep-13': raise ValueError('Layer2IncrementMismatch')
    return dict(layer=2, increment=h, increment_hex=h.hex(),
                cumulative_target=float(bound['targets'][1]),
                schedule_identity=bound['schedule_identity'])


def metadata_audit(state):
    rows = list(state['forecast'].values())
    return dict(columns=len(state['columns']), forecast_rows=len(rows),
                column_layers=dict(collections.Counter(str(r['last_prism_layer']) for r in state['columns'].values())),
                forecast_layer_counts=dict(collections.Counter(str(r['current_layer']) for r in rows)),
                remaining_schedule_length_counts=dict(collections.Counter(str(len(r['remaining_schedule'])) for r in rows)),
                forecast_field_sets=sorted({tuple(sorted(r)) for r in rows}),
                atlas_rows=len(state['atlas']),
                meaning='Dependency descriptors only; not geometric predictions or admission receipts')


def restore(label):
    t = time.perf_counter()
    wall, tri, surface = load_production_wall(SURFACE)
    if surface['surface_identity'] != SURFACE_ID: raise ValueError('WallIdentityMismatch')
    wall_seconds = time.perf_counter()-t
    t = time.perf_counter(); bound = read_request(REQUEST, surface); request = select_layer2(bound)
    request_seconds = time.perf_counter()-t
    checkpoint = L1/'A/checkpoints'
    if (checkpoint/'BLOCK_TRANSACTION.json').exists(): raise ValueError('PendingTransactionReadOnlyAuditRequired')
    pointer_before = file_hash(checkpoint/'CURRENT.json')
    print('native restore start', label, flush=True); t = time.perf_counter()
    state = NativeCheckpointStore(checkpoint).restart()
    restore_seconds = time.perf_counter()-t
    print('native restore complete', label, restore_seconds, flush=True)
    restored_id = scientific_hash(state)
    if restored_id != COMMITTED: raise ValueError('RestoredStateIdentityMismatch')
    front = np.array([state['vertices'][f'layer:1:source:{i}'] for i in range(len(wall))])
    restored_wall = np.array([state['vertices'][f'layer:0:source:{i}'] for i in range(len(wall))])
    if not np.array_equal(restored_wall, wall): raise ValueError('RestoredWallMismatch')
    layers = collections.Counter(str(c['layer']) for c in state['cells'].values())
    if set(layers) != {'1'}: raise ValueError('NotExactlyCommittedLayer1')
    from production_first_layer_run import ProductionFirstLayerProvider
    # Exercise the existing non-generative scope guard; do not call a fixture provider.
    try: ProductionFirstLayerProvider.propose(None, state)
    except ValueError as e: rejection = str(e)
    else: raise ValueError('ProductionScopeChangedRequiresReaudit')
    if rejection != 'ExactlyOneLayerScope': raise ValueError('UnexpectedProductionScopeRejection')
    if file_hash(checkpoint/'CURRENT.json') != pointer_before: raise ValueError('CheckpointMutation')
    x2, f2, a2 = load_production_wall(SURFACE)
    if not np.array_equal(wall, x2) or not np.array_equal(tri, f2) or surface != a2:
        raise ValueError('WallMutation')
    edges = np.unique(np.sort(np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]), axis=1), axis=0)
    scientific = dict(restored_state=restored_id, pointer_sha256=pointer_before,
        wall_identities={k:surface[k] for k in ('surface_identity','geometry_identity','coordinate_hash','topology_hash','sizing_field_identity')},
        request_identity=identity(bound['declaration']), request_file_sha256=file_hash(REQUEST), request=request,
        l1_front_coordinate_hash=array_hash(front), l1_front_topology_hash=array_hash(tri),
        restored_wall_coordinate_hash=array_hash(restored_wall),
        counts=dict(wall_vertices=len(wall), wall_edges=len(edges), wall_triangles=len(tri),
                    l1_front_vertices=len(front), l1_front_edges=len(edges), l1_front_triangles=len(tri),
                    stored_vertices=len(state['vertices']), stored_cells=len(state['cells']),
                    stored_faces=len(state['faces']), cell_layers=dict(layers)),
        metadata=metadata_audit(state), provider_rejection=rejection,
        decision='BlockedProductionForecastIntegration', layer2_authorized=False,
        wall_mutations=0, committed_layer_after=1,
        generated=dict(layer2_vertices=0, layer2_prisms=0, layer3_vertices=0, layer3_prisms=0, provisional_future_fronts=0, core_cells=0))
    result=dict(scientific=scientific, scientific_identity=identity(scientific),
        times=dict(wall_load=wall_seconds,request_decode=request_seconds,native_restore=restore_seconds),
        peak_rss_bytes=psutil.Process().memory_info().peak_wset,
        retained_rss_bytes=psutil.Process().memory_info().rss)
    save(OUT/(label+'.json'),result)
    print(json.dumps(result),flush=True)


def baseline():
    status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=ROOT,text=True).splitlines()
    save(OUT/'GIT_START.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
         branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),entries=status,
         unrelated_entries=[s for s in status if 'capsule_layer2_preflight.py' not in s and 'capsule_advancing_front_layer2' not in s]))
    disk=disk_audit();save(OUT/'DISK_START.json',disk)
    print(json.dumps({k:v for k,v in disk.items() if k!='errors'}), 'inaccessible_entries', len(disk['errors']), flush=True)


if __name__ == '__main__':
    import sys
    if sys.argv[1]=='baseline': baseline()
    else: restore(sys.argv[1])
