"""Fresh source-reviewed repeat of the existing retained-L6 storage control.

This configures the existing benchmark; it does not execute a mesh transition.
Old baseline, source pins, private controls and scientific states stay intact.
"""
import argparse
import subprocess
import time
from pathlib import Path

import benchmark_retained_l6 as control

OUT = control.V4/'results/compact_reuse_l6'
OLD = control.OUT
control.OUT = OUT
control.EXTRA_RUNTIME += ('benchmark_compact_reuse.py',)


def setup():
    if OUT.exists():raise ValueError('PreserveRepeatEvidence')
    before = control.read(OLD/'BASELINE.json')
    before.update(start=time.time(), head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        unrelated=control.status(), disk=control.disk(),
        repeat_of=str(OLD), old_baseline_sha256=control.file_hash(OLD/'BASELINE.json'),
        original_control_results={str(OLD/n/'RESULT.json'):control.file_hash(OLD/n/'RESULT.json') for n in ('baseline','optimized')},
        preservation={str(p):control.file_hash(p) for p in (
            control.ROOT/'src/sdf/aerospace/wing.rs',control.ROOT/'src/sdf/aerospace/airfoil.rs',
            control.ROOT/'src/sdf/aerospace/naca_spline.rs',
            control.V4/'tools/critical_transaction.py',control.V4/'tools/composable_state.py',
            control.V4/'results/remaining_prism_critical_path/AUTHORITY.json',
            control.V4/'results/wing_continuation_layers6to8/SUMMARY.json')},
        implementation_before={str(control.V4/'tools'/n):control.file_hash(control.V4/'tools'/n) for n in (
            'single_quality_component.py','benchmark_retained_l6.py')},
        authorization='Remove traced redundant work; preserve accuracy; roughly linear scaling is a target, not a gate.',
        scope='Existing full retained-L6 storage control only. No new cells, proof recomputation, front publication or advancement.')
    OUT.mkdir()
    control.durable_json(OUT/'BASELINE.json',before)
    print(dict(status_entries=len(before['unrelated']),head=before['head'],disk=before['disk']))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['setup','worker','monitor'])
    args=parser.parse_args()
    if args.command=='setup':setup()
    elif args.command=='worker':control.worker('optimized',ordered_atlas=True)
    else:
        # Keep the existing resource monitor and minimum-free policy. Route its
        # fresh worker through this wrapper so output/source pins stay selected.
        import finish_retained_l5 as monitor
        import sys
        monitor.OUT=OUT/'monitor';monitor.OUT.mkdir(exist_ok=False)
        sys.exit(monitor.monitor(worker_command=[sys.executable,'-B',__file__,'worker'],
            extra_storage_roots=[control.V4/'results/current_front_transition_round_trip',
                control.V4/'results/wing_continuation_layers6to8',OUT/'optimized',OLD/'baseline',OLD/'optimized'],
            enforce_output_cap=False))
