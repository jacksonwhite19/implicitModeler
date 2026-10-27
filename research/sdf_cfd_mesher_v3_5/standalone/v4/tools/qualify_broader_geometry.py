"""Cohort execution through the existing native mixed runner.

Case selection is evidence configuration. It does not change provider science,
trigger decisions, hard gates, or certificate mathematics.
"""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
import argparse, json, time, traceback
from collections import Counter
import numpy as np
import psutil
from mixed_front_fixture_provider import Provider
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.core.interface import extract
from export_native_core_interface import owner_vertices

OUT = Path(__file__).resolve().parents[1] / 'results/broader_mixed_geometry_end_to_end'


def summarize_checkpoint(folder):
    """Recover reporting from committed state without repeating a transaction."""
    from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
    store = MixedFrontJournal(folder/'checkpoints')
    state = store.restart()
    operations = json.loads((folder/'OPERATIONS.json').read_text())
    result = dict(final=scientific_hash(state),
        cells=dict(Counter(c['kind'] for c in state['cells'].values())),
        column_states=dict(Counter(c['status'] for c in state['columns'].values())),
        remesh_count=sum(o['operation']=='RollbackRemesh' for o in operations),
        operations=operations, interface=export(state,folder/'interface'),
        restart_exact=scientific_hash(state)==scientific_hash(store.restart()),
        reporting_recovered_from_committed_checkpoint=True)
    g.save(folder/'SHELL_RESULT.json',result)
    return result


def export(state, folder):
    folder.mkdir(parents=True, exist_ok=True)
    points, offsets, connectivity, metadata = extract(state)
    for name, value in [('points', points), ('offsets', offsets), ('connectivity', connectivity)]:
        np.save(folder / (name + '.npy'), value, allow_pickle=False)
    cells = [state['cells'][key] for key in metadata['shell_owners']]
    nodes = [owner_vertices(c) for c in cells]
    for arity in sorted({len(n) for n in nodes}):
        ids = [i for i, n in enumerate(nodes) if len(n) == arity]
        np.save(folder / f'owner_faces_{arity}.npy', np.asarray(ids, dtype='<i8'))
        np.save(folder / f'owner_geometry_{arity}.npy', np.asarray([
            [state['vertices'][v] for v in nodes[i]] for i in ids]))
    g.save(folder / 'INTERFACE.json', metadata)
    g.save(folder / 'OWNER_TYPES.json', [c['kind'] for c in cells])
    return metadata


def run(name, replay):
    folder = OUT / name / replay
    folder.mkdir(parents=True, exist_ok=True)
    if (folder / 'checkpoints/CURRENT.json').exists():
        raise ValueError('FreshCohortRunRequiresUnusedOutput')
    tick = time.perf_counter()
    if name == 'general_asymmetric_passage':
        from migrate_general_native import HistoricalProvider
        provider = HistoricalProvider(folder)
    else:
        specs = {s['name']: s for s in g.fixtures.definitions()}
        # The complete-capacity authority uses subdivision level 3. Carry that
        # source resolution explicitly; do not inherit the remesh-study default.
        from complete_capacity_fixtures import definitions
        specs.update({s['name']: dict(s, level=3) for s in definitions(g.q.load_schedule().targets)
                      if s['name'] == 'curved_wide'})
        provider = Provider(specs[name], folder)
    operations = []
    try:
        initial = provider.initialize()
        initial_hash = scientific_hash(initial)
        print('INITIAL', name, initial_hash, flush=True)
        runner = MixedFrontRunner(folder / 'checkpoints', provider)
        runner.initialize(initial)
        current_layer = max(c['layer'] for c in initial['cells'].values())
        for _ in range(min(2, len(provider.schedule.targets)-current_layer)):
            operations.append(runner.step())
            g.save(folder / 'OPERATIONS.json', operations)
            print('COMMITTED', name, operations[-1]['operation'], flush=True)
        final = runner.store.restart()
        interface = export(final, folder / 'interface')
        final_hash = scientific_hash(final)
        assert final_hash == scientific_hash(runner.store.restart())
        assert initial['wall'] == final['wall']
        assert initial['first_layer'] == final['first_layer']
        result = dict(case=name, scope='bounded native shell', initial=initial_hash,
            final=final_hash, wall_exact=True, first_layer_exact=True, restart_exact=True,
            cells=dict(Counter(c['kind'] for c in final['cells'].values())),
            column_states=dict(Counter(c['status'] for c in final['columns'].values())),
            remesh_count=sum(o['operation'] == 'RollbackRemesh' for o in operations),
            operations=operations, interface=interface, records=provider.records,
            numerical='NotEvaluatedDueToMissingAuthoritativeCaseControls')
        g.save(folder / 'SHELL_RESULT.json', result)
    except BaseException:
        g.save(folder / 'FAILURE.json', dict(traceback=traceback.format_exc(), operations=operations))
        raise
    finally:
        rss = psutil.Process().memory_info()
        g.save(folder / 'PERFORMANCE.json', dict(seconds=time.perf_counter()-tick,
            peak_rss_bytes=getattr(rss, 'peak_wset', rss.rss), stages=provider.times))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('case')
    parser.add_argument('replay', choices=['A', 'B'])
    parser.add_argument('--summarize-committed', action='store_true')
    args = parser.parse_args()
    if args.summarize_committed: summarize_checkpoint(OUT/args.case/args.replay)
    else: run(args.case, args.replay)
